library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.dual_port_ram_pkg.all;
    use work.fixed_dsp_pkg.all;
    use work.output_bridge_pkg.all;

-- Sigma-delta modulators with a common clock, as the power stage's isolated current
-- modulators: the FPGA drives the modulator clock and reads each one's bit stream, which is
-- filtered at the full bit rate (20 MHz) and averaged over exactly one switching period.
--
-- The modulator clock and the bit streams run from adc_clock (200 MHz: 20 MHz modulator clock
-- by default, as the power supply design); the registers and the results are in the main
-- clock domain. Settings cross over when a register is written, as in board_adc.
--
-- The clock is high for the first half of each period of clocks adc_clock clocks. A modulator
-- changes its data after the clock's rising edge, which comes back through the pins, the
-- isolator and two synchronizer flops, so each bit is read at clock phase of the period:
-- register 2 sets it for the delays of the board.
--
-- Each bit, +1 or -1 (full scale), goes through three first order low pass sections at the
-- bit rate, y = y + (x - y) / 2^k: three equal poles, no overshoot and a nearly linear phase
-- (as a Bessel filter), enough of them for a second order modulator's shaped noise. k = 4 at
-- 20 MHz is -3 dB at ~100 kHz, a 2.4 us delay: the switching ripple shows, the modulator's
-- noise does not (~0.05 A rms of 128 A in a model of the modulator).
--
-- The filtered stream's sum over the last window bits, the window a switching period exactly
-- (output_bridge_pkg: 500 bits = 25 us, the output bridge's 40 kHz), divided by the
-- window: the mean over one period, the ripple and all its harmonics gone, updated every bit.
-- The sum is the difference of a running sum now and a window ago, kept in a block RAM
-- (the memory library's dual_port_ram), so it is exact and a new window takes effect after a
-- window of bits without upsetting it.
--
-- results (the filtered values) and means (the period means) are sampled every R = 2^log2 bits
-- (64: 312.5 kS/s at 20 MHz), the main clock domain's is_new; both scaled to 16 bits: full
-- scale +-32768, saturated to 32767.
--
-- sample (a clock's pulse, e.g. the modulator's at each period start and middle) takes the
-- period means at that bit as well: sampled, with sampled_ready a pulse ~0.1 us later. The
-- window's sum is taken as it is a few adc_clocks after the pulse (its synchronizer), the last
-- bit's: within a bit (50 ns) of the pulse. A mean over the last period is the current half a
-- period before, for a current changing steadily.
--
-- register map, offsets from g_base_address
--   0  RW control: bit0 run (the modulator clock and the filters), bits 11:8 log2 of the
--      output decimation R, 5..8 (6), bits 15:12 the filter's k, 2..6 (4); values out of
--      range keep the previous
--   1  RW adc_clock clocks per modulator clock period, 4..255 (10: 20 MHz)
--   2  RW the phase of the period at which each bit is read, 0..period-1. A bit that changes
--      d ns after the rising edge at the pin is read at its change at phase d/5 + 2 (the
--      clock's output flop and two synchronizer flops); the default 1 reads mid bit for d
--      from 0 to about 40 ns
--   3  R  results since power up
--   4..6  R  latest filtered result of channel 0..2, 16 bit full scale
--   7  RW the mean's window in bits, 16..1023 (output_period_bits: 500)
--   8..10 R  latest period mean of channel 0..2, 16 bit full scale
--   11..13 R the period mean of channel 0..2 at the latest sample pulse
--   14 R  sample pulses since power up
--   16 RW the midpoint sampler's ki: bits per count of the difference, 18 bit signed with 16
--         fractional bits (ki + kp under 2; 0.05)
--   17 RW its kp, the same (0.1)
--   18 R  its sample point tau, bits from the period's start (16 fractional bits)
--   19 R  the latest midpoint sample, 16 bit full scale
--
-- The midpoint sampler (channel g_raw_channel, the output bridge's current): with complementary
-- modulation the current rises while leg 1 is high (duty d of the period) and falls the rest;
-- the middles of the rise and of the fall are at d T / 2 and T / 2 + d T / 2, half a period
-- apart at any duty, and there the current is the period's average. The sampler takes the
-- filtered stream at bit tau and at tau + 250 (half of the period's 500 bits, modulo 500) from
-- the period's start (period_start, the modulator's): their difference, the one on the rise less
-- the one on the fall, is positive with tau late, negative early; a PI moves tau (once a period)
-- to make it none. Each sample is the average current, given as midpoint with midpoint_ready.
entity sdm_adc is
    generic(
        g_base_address : natural := 740;
        g_channels     : positive := 3;
        g_raw_channel  : natural := 1     -- the output bridge's current
    );
    port(
        clock     : in std_logic;
        bus_in    : in fpga_interconnect_record;
        bus_out   : out fpga_interconnect_record;
        -- the results to the scope, synchronous to clock
        results   : out scope_sample_array(0 to g_channels-1);
        means     : out scope_sample_array(0 to g_channels-1);
        is_new    : out boolean;
        -- the period means at a pulse of sample
        sample        : in boolean := false;
        sampled       : out scope_sample_array(0 to g_channels-1);
        sampled_ready : out boolean;
        -- a diagnostic probe of channel g_raw_channel's bits as read, each with marker's state
        -- at its read (e.g. a gate of the bridge whose current it measures): 7 bits a word,
        -- bits 6..0 the bits (0 the latest), 13..7 the marker's, 15..14 the words' count; a new
        -- word every 7 bits (350 ns at 20 MHz), stable until the next
        marker        : in std_logic := '0';
        raw_bits      : out scope_sample;
        -- the midpoint sampler: the modulator's period starts in, the samples out
        period_start   : in boolean := false;
        midpoint       : out scope_sample;
        midpoint_ready : out boolean;
        -- the modulators
        adc_clock : in std_logic;
        sdm_clock : out std_logic;
        sdm_data  : in std_logic_vector(0 to g_channels-1)
    );
end entity sdm_adc;

architecture rtl of sdm_adc is

    -- the filter: a bit is +-2^31 (16 bit full scale with 16 fractional bits)
    constant filter_width : natural := 34;
    subtype filter_value is signed(filter_width-1 downto 0);
    type filter_array is array (natural range <>) of filter_value;
    constant fraction : natural := 16;

    -- the running sum: the filtered values with 8 fractional bits, wrapping; a window's sum of
    -- them fits: 1023 * 2^23 < 2^35
    constant sum_width : natural := 36;
    subtype sum_value is signed(sum_width-1 downto 0);
    type sum_array is array (natural range <>) of sum_value;
    constant sum_shift : natural := fraction - 8;

    -- the reciprocal of the window for the mean: 2^24 / window
    constant reciprocal_bits : natural := 24;

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    -- registers, clock domain
    signal control_r : std_logic_vector(31 downto 0) := x"00004601";   -- run, R = 64, k = 4
    signal period_r  : std_logic_vector(31 downto 0) := to_slv(10);
    signal phase_r   : std_logic_vector(31 downto 0) := to_slv(1);
    signal window_r  : std_logic_vector(31 downto 0) := to_slv(output_period_bits);
    signal settings_toggle : std_logic := '0';
    signal conversions     : unsigned(31 downto 0) := (others => '0');
    signal results_r, means_r : scope_sample_array(0 to g_channels-1) := (others => (others => '0'));
    signal result_sync     : std_logic_vector(2 downto 0) := "000";

    -- the mean: the window's reciprocal (a division when the window is written), the products
    signal reciprocal    : unsigned(reciprocal_bits downto 0) := to_unsigned(2**reciprocal_bits / output_period_bits, reciprocal_bits + 1);
    signal dividing      : natural range 0 to reciprocal_bits + 1 := 0;
    signal remainder     : unsigned(11 downto 0) := (others => '0');
    signal quotient      : unsigned(reciprocal_bits downto 0) := (others => '0');
    signal divisor       : unsigned(11 downto 0) := to_unsigned(output_period_bits, 12);
    signal captured_sums : sum_array(0 to g_channels-1) := (others => (others => '0'));
    signal captured_filtered : filter_array(0 to g_channels-1) := (others => (others => '0'));
    type product_array is array (natural range <>) of signed(sum_width + reciprocal_bits + 1 downto 0);
    signal products      : product_array(0 to g_channels-1) := (others => (others => '0'));
    signal mean_stage    : natural range 0 to 2 := 0;

    -- the sample pulses: to the adc_clock domain and their sums back
    signal sample_toggle   : std_logic := '0';
    signal sampled_sync    : std_logic_vector(2 downto 0) := "000";
    signal sample_stage    : natural range 0 to 2 := 0;
    signal sample_sums     : sum_array(0 to g_channels-1) := (others => (others => '0'));
    signal sample_products : product_array(0 to g_channels-1) := (others => (others => '0'));
    signal sampled_r       : scope_sample_array(0 to g_channels-1) := (others => (others => '0'));
    signal samples         : unsigned(31 downto 0) := (others => '0');
    signal raw_sync        : std_logic_vector(2 downto 0) := "000";
    -- the midpoint sampler, main clock domain: its gains, the period starts to the adc_clock
    -- domain, the samples back
    signal ki_r            : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(3277, 32));
    signal kp_r            : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(6554, 32));
    signal period_toggle   : std_logic := '0';
    signal mid_sync        : std_logic_vector(2 downto 0) := "000";
    signal midpoint_r      : scope_sample := (others => '0');
    signal tau_main        : signed(31 downto 0) := (others => '0');
    signal raw_bits_r      : scope_sample := (others => '0');

    -- adc_clock domain
    signal toggle_sync : std_logic_vector(2 downto 0) := "000";
    signal run         : boolean := true;
    signal log2_r      : natural range 5 to 8 := 6;
    signal shift_k     : natural range 2 to 6 := 4;
    signal window      : natural range 16 to 1023 := output_period_bits;
    signal period      : natural range 4 to 255 := 10;
    signal phase       : natural range 0 to 255 := 1;
    signal count       : natural range 0 to 255 := 0;
    signal clock_r     : std_logic := '0';
    signal data_sync1, data_sync2 : std_logic_vector(0 to g_channels-1) := (others => '0');
    signal stage0, stage1, stage2 : filter_array(0 to g_channels-1) := (others => (others => '0'));
    signal running_sum : sum_array(0 to g_channels-1) := (others => (others => '0'));
    signal window_sum  : sum_array(0 to g_channels-1) := (others => (others => '0'));
    signal decimation  : natural range 0 to 255 := 0;
    -- a bit's steps after it is read: 1 the running sum into the RAM and the read of a window
    -- ago, 2 the RAM's data, 3 the window's sum, 4 the output (when it is a result's bit)
    signal step        : natural range 0 to 4 := 0;
    signal output_due  : boolean := false;
    signal output_filtered : filter_array(0 to g_channels-1) := (others => (others => '0'));
    signal output_sums     : sum_array(0 to g_channels-1) := (others => (others => '0'));
    signal result_toggle : std_logic := '0';
    signal write_address : unsigned(9 downto 0) := (others => '0');
    signal sample_sync     : std_logic_vector(2 downto 0) := "000";
    -- the raw bits' probe: the bits and the marker's states collected, a word every 7 bits
    signal marker_sync1, marker_sync2 : std_logic := '0';
    signal raw_data, raw_marker : std_logic_vector(6 downto 0) := (others => '0');
    signal raw_count   : natural range 0 to 6 := 0;
    signal raw_words   : unsigned(1 downto 0) := (others => '0');
    signal raw_word    : std_logic_vector(15 downto 0) := (others => '0');
    signal raw_toggle  : std_logic := '0';
    -- the midpoint sampler, adc_clock domain
    signal start_sync    : std_logic_vector(2 downto 0) := "000";
    signal restart       : boolean := false;
    signal period_bit    : natural range 0 to output_period_bits - 1 := 0;
    -- 18 bits each, a product one 18 x 18 DSP: the gains +-2 bits a count (16 fractional bits),
    -- the difference saturated to +-1024 counts (4 A): a correction at most ~150 bits, under half
    -- a period, and a switching spike's kick limited
    signal ki            : signed(17 downto 0) := to_signed(3277, 18);   -- 0.05
    signal kp            : signed(17 downto 0) := to_signed(6554, 18);   -- 0.1
    signal tau_integral  : signed(31 downto 0) := to_signed(172 * 65536, 32);   -- bits, 16 fractional
    signal tau           : natural range 0 to output_period_bits - 1 := 172;
    signal tau_fall      : natural range 0 to output_period_bits - 1 := 172 + output_period_bits / 2;
    signal pi_step       : natural range 0 to 4 := 0;
    signal difference_r  : signed(17 downto 0) := (others => '0');
    signal integral_r, point_r : signed(31 downto 0) := (others => '0');
    subtype pi_dsp_in_record  is fixed_dsp_in_record(a(17 downto 0), d(17 downto 0), b(17 downto 0), c(35 downto 0));
    subtype pi_dsp_out_record is fixed_dsp_out_record(result(35 downto 0));
    signal integral_dsp_in, point_dsp_in   : pi_dsp_in_record := init_fixed_dsp_in(18);
    signal integral_dsp_out, point_dsp_out : pi_dsp_out_record;
    signal rise_sample   : filter_value := (others => '0');
    signal mid_value     : filter_value := (others => '0');
    signal mid_toggle    : std_logic := '0';
    signal tau_value     : signed(31 downto 0) := (others => '0');
    signal sampled_toggle  : std_logic := '0';
    signal sampled_sums    : sum_array(0 to g_channels-1) := (others => (others => '0'));

    -- the running sums of a window ago: one RAM word holds every channel's
    constant ram_subtype : dpram_ref_record := create_ref_subtypes(datawidth => sum_width * g_channels, addresswidth => 10);
    constant ram_zeros   : ram_array(0 to 1023)(sum_width * g_channels - 1 downto 0) := (others => (others => '0'));
    signal ram_write_in, ram_read_in   : ram_subtype.ram_in'subtype;
    signal ram_write_out, ram_read_out : ram_subtype.ram_out'subtype;

    attribute async_reg : boolean;
    attribute async_reg of toggle_sync, result_sync, data_sync1, data_sync2, sample_sync, sampled_sync,
        raw_sync, marker_sync1, marker_sync2, start_sync, mid_sync : signal is true;

    -- 16 bit full scale, saturated
    function to_probe_saturated(v : signed) return scope_sample is
    begin
        if v > 32767 then
            return to_probe(32767);
        elsif v < -32768 then
            return to_probe(-32768);
        end if;
        return to_probe(resize(v, 16));
    end function;

begin

    results   <= results_r;
    means     <= means_r;
    sampled   <= sampled_r;
    raw_bits  <= raw_bits_r;
    midpoint  <= midpoint_r;
    sdm_clock <= clock_r;

    u_integral_dsp : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => true)
    port map(clock => adc_clock, fixed_dsp_in => integral_dsp_in, fixed_dsp_out => integral_dsp_out);

    u_point_dsp : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => true)
    port map(clock => adc_clock, fixed_dsp_in => point_dsp_in, fixed_dsp_out => point_dsp_out);

    u_window_ram : entity work.dual_port_ram
    generic map(g_dpram_subtype => ram_subtype, g_ram_init_values => ram_zeros, g_output_register => false)
    port map(clock => adc_clock, ram_a_in => ram_write_in, ram_a_out => ram_write_out,
             ram_b_in => ram_read_in, ram_b_out => ram_read_out);

------------------------------------------------------------------------
    registers : process(clock)
        variable trial : unsigned(12 downto 0);
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 0, control_r);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 1, period_r);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 2, phase_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(conversions));
            connect_data_to_address          (bus_in, bus_out, g_base_address + 7, window_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 14, std_logic_vector(samples));
            connect_data_to_address          (bus_in, bus_out, g_base_address + 16, ki_r);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 17, kp_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 18, std_logic_vector(tau_main));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 19,
                std_logic_vector(resize(signed(midpoint_r), 32)));
            for c in 0 to g_channels-1 loop
                connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4 + c,
                    std_logic_vector(resize(signed(results_r(c)), 32)));
                connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8 + c,
                    std_logic_vector(resize(signed(means_r(c)), 32)));
                connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11 + c,
                    std_logic_vector(resize(signed(sampled_r(c)), 32)));
            end loop;

            for offset in 0 to 17 loop
                if write_is_requested_to_address(bus_in, g_base_address + offset) then
                    settings_toggle <= not settings_toggle;
                end if;
            end loop;

            -- a new window: its reciprocal 2^24 / window, a bit a clock
            if write_is_requested_to_address(bus_in, g_base_address + 7) then
                if unsigned(get_slv_data(bus_in)(11 downto 0)) >= 16 and unsigned(get_slv_data(bus_in)(11 downto 0)) <= 1023 then
                    divisor   <= unsigned(get_slv_data(bus_in)(11 downto 0));
                    remainder <= (others => '0');
                    quotient  <= (others => '0');
                    dividing  <= reciprocal_bits + 1;
                end if;
            elsif dividing > 0 then
                -- the dividend 2^24: its one bit comes in at the first step
                if dividing = reciprocal_bits + 1 then
                    trial := remainder & '1';
                else
                    trial := remainder & '0';
                end if;
                if trial >= divisor then
                    remainder <= resize(trial - divisor, 12);
                    quotient  <= quotient(reciprocal_bits - 1 downto 0) & '1';
                else
                    remainder <= resize(trial, 12);
                    quotient  <= quotient(reciprocal_bits - 1 downto 0) & '0';
                end if;
                dividing <= dividing - 1;
                if dividing = 1 then
                    mean_stage <= 0;
                end if;
            elsif dividing = 0 and quotient /= 0 then
                reciprocal <= quotient;
                quotient   <= (others => '0');
            end if;

            -- the midpoint sampler: the period's start over, its samples back (stable until the next,
            -- 250 bits later)
            if period_start then
                period_toggle <= not period_toggle;
            end if;
            mid_sync <= mid_sync(1 downto 0) & mid_toggle;
            midpoint_ready <= false;
            if mid_sync(2) /= mid_sync(1) then
                midpoint_r     <= to_probe_saturated(shift_right(mid_value, fraction));
                tau_main       <= tau_value;
                midpoint_ready <= true;
            end if;

            -- the raw bits' word, stable for 7 bits when its toggle arrives
            raw_sync <= raw_sync(1 downto 0) & raw_toggle;
            if raw_sync(2) /= raw_sync(1) then
                raw_bits_r <= raw_word;
            end if;

            -- a sample pulse to the adc clock domain; its sums back, stable until the next pulse
            if sample then
                sample_toggle <= not sample_toggle;
            end if;
            sampled_sync  <= sampled_sync(1 downto 0) & sampled_toggle;
            sampled_ready <= false;
            case sample_stage is
                when 0 =>
                    if sampled_sync(2) /= sampled_sync(1) then
                        sample_sums  <= sampled_sums;
                        sample_stage <= 1;
                    end if;
                when 1 =>
                    for c in 0 to g_channels-1 loop
                        sample_products(c) <= sample_sums(c) * signed('0' & reciprocal);
                    end loop;
                    sample_stage <= 2;
                when 2 =>
                    for c in 0 to g_channels-1 loop
                        sampled_r(c) <= to_probe_saturated(shift_right(sample_products(c), reciprocal_bits + 8));
                    end loop;
                    samples       <= samples + 1;
                    sampled_ready <= true;
                    sample_stage  <= 0;
            end case;

            -- a result from the adc clock domain: its values are stable until the next one, at
            -- least 32 modulator periods later
            result_sync <= result_sync(1 downto 0) & result_toggle;
            is_new <= false;
            case mean_stage is
                when 0 =>
                    if result_sync(2) /= result_sync(1) then
                        captured_filtered <= output_filtered;
                        captured_sums     <= output_sums;
                        mean_stage        <= 1;
                    end if;
                when 1 =>
                    for c in 0 to g_channels-1 loop
                        products(c) <= captured_sums(c) * signed('0' & reciprocal);
                    end loop;
                    mean_stage <= 2;
                when 2 =>
                    for c in 0 to g_channels-1 loop
                        results_r(c) <= to_probe_saturated(shift_right(captured_filtered(c), fraction));
                        means_r(c)   <= to_probe_saturated(shift_right(products(c), reciprocal_bits + 8));
                    end loop;
                    conversions <= conversions + 1;
                    is_new      <= true;
                    mean_stage  <= 0;
            end case;
        end if;
    end process;

------------------------------------------------------------------------
    modulators : process(adc_clock)
        variable bit_value : filter_value;
        variable sum       : sum_value;
        variable packed    : std_logic_vector(sum_width * g_channels - 1 downto 0);
        variable stored    : std_logic_vector(sum_width * g_channels - 1 downto 0);
        variable position  : natural range 0 to output_period_bits - 1;
        variable tau_bits  : natural range 0 to 2 * output_period_bits;
        variable difference : filter_value;
    begin
        if rising_edge(adc_clock) then
            init_ram(ram_write_in);
            init_ram(ram_read_in);

            -- settings: the registers have been stable for several clocks when the toggle arrives
            toggle_sync <= toggle_sync(1 downto 0) & settings_toggle;
            if toggle_sync(2) /= toggle_sync(1) then
                run <= control_r(0) = '1';
                if unsigned(control_r(11 downto 8)) >= 5 and unsigned(control_r(11 downto 8)) <= 8 then
                    log2_r <= to_integer(unsigned(control_r(11 downto 8)));
                end if;
                if unsigned(control_r(15 downto 12)) >= 2 and unsigned(control_r(15 downto 12)) <= 6 then
                    shift_k <= to_integer(unsigned(control_r(15 downto 12)));
                end if;
                if unsigned(period_r(7 downto 0)) >= 4 then
                    period <= to_integer(unsigned(period_r(7 downto 0)));
                end if;
                phase <= to_integer(unsigned(phase_r(7 downto 0)));
                ki <= signed(ki_r(17 downto 0));
                kp <= signed(kp_r(17 downto 0));
                if unsigned(window_r(11 downto 0)) >= 16 and unsigned(window_r(11 downto 0)) <= 1023 then
                    window <= to_integer(unsigned(window_r(11 downto 0)));
                end if;
            end if;

            -- a sample pulse: the window's sums as they are, the last bit's
            sample_sync <= sample_sync(1 downto 0) & sample_toggle;
            if sample_sync(2) /= sample_sync(1) then
                sampled_sums   <= window_sum;
                sampled_toggle <= not sampled_toggle;
            end if;

            -- the midpoint sampler: a period's start restarts the bits' count at the next bit
            start_sync <= start_sync(1 downto 0) & period_toggle;
            if start_sync(2) /= start_sync(1) then
                restart <= true;
            end if;

            data_sync1 <= sdm_data;
            marker_sync1 <= marker;
            marker_sync2 <= marker_sync1;
            data_sync2 <= data_sync1;

            -- the modulator clock: high for the first half of the period
            if count < period - 1 then
                count <= count + 1;
            else
                count <= 0;
            end if;
            if run and count < period / 2 then
                clock_r <= '1';
            else
                clock_r <= '0';
            end if;

            -- a bit of each modulator through the three sections
            step <= 0;
            -- the midpoint sampler at each bit: the filtered value at tau (on the rise) and a half
            -- period on (on the fall); after the second the PI moves tau by their difference
            if run and count = phase then
                if restart then
                    position := 0;
                    restart  <= false;
                elsif period_bit < output_period_bits - 1 then
                    position := period_bit + 1;
                else
                    position := 0;
                end if;
                period_bit <= position;
                if position = tau then
                    rise_sample <= stage2(g_raw_channel);
                    mid_value   <= stage2(g_raw_channel);
                    mid_toggle  <= not mid_toggle;
                elsif position = tau_fall then
                    mid_value  <= stage2(g_raw_channel);
                    mid_toggle <= not mid_toggle;
                    -- the difference in counts (16 bit full scale): positive, tau late
                    difference := shift_right(rise_sample - stage2(g_raw_channel), fraction);
                    if difference > 1024 then
                        difference_r <= to_signed(1024, 18);
                    elsif difference < -1024 then
                        difference_r <= to_signed(-1024, 18);
                    else
                        difference_r <= resize(difference, 18);
                    end if;
                    pi_step <= 1;
                end if;
            end if;

            -- the PI, once a period (a bit is 10 clocks), on two fixed_dsps: the integral,
            -- integral - ki e, and the sample point, integral - (ki + kp) e + 1/2 (the pre-adder
            -- summing the gains, so the old integral does: the same modulo the period); ki and kp
            -- in bits a count with 16 fractional bits, so the products are tau's units (ki + kp
            -- under 2, the pre-adder's 18 bits). The fixed_dsp's invert negates its whole result:
            -- -(product - c) = c - product
            init_fixed_dsp(integral_dsp_in);
            init_fixed_dsp(point_dsp_in);
            case pi_step is
                when 1 =>
                    fmac(integral_dsp_in, a => ki, d => to_signed(0, 18), b => difference_r,
                         c => resize(tau_integral, 36), post_subtract_with_1 => '1', invert_result_with_1 => '1');
                    fmac(point_dsp_in, a => ki, d => kp, b => difference_r,
                         c => resize(tau_integral, 36) + 32768, post_subtract_with_1 => '1', invert_result_with_1 => '1');
                    pi_step <= 2;
                when 2 =>
                    if integral_dsp_out.ready_with_1 = '1' then
                        integral_r <= resize(integral_dsp_out.result, 32);
                        point_r    <= resize(point_dsp_out.result, 32);
                        pi_step <= 3;
                    end if;
                when 3 =>
                    -- within the period: the corrections are under half a period (the difference
                    -- saturated), so one wrap does
                    if integral_r < 0 then
                        integral_r <= integral_r + to_signed(output_period_bits * 65536, 32);
                    elsif integral_r >= to_signed(output_period_bits * 65536, 32) then
                        integral_r <= integral_r - to_signed(output_period_bits * 65536, 32);
                    end if;
                    if point_r < 0 then
                        point_r <= point_r + to_signed(output_period_bits * 65536, 32);
                    elsif point_r >= to_signed(output_period_bits * 65536, 32) then
                        point_r <= point_r - to_signed(output_period_bits * 65536, 32);
                    end if;
                    pi_step <= 4;
                when 4 =>
                    tau_integral <= integral_r;
                    tau_value    <= integral_r;
                    -- the new points: the rise's, a half period on the fall's
                    tau_bits := to_integer(shift_right(point_r, 16));
                    if tau_bits >= output_period_bits then
                        tau_bits := tau_bits - output_period_bits;
                    end if;
                    tau <= tau_bits;
                    if tau_bits < output_period_bits / 2 then
                        tau_fall <= tau_bits + output_period_bits / 2;
                    else
                        tau_fall <= tau_bits - output_period_bits / 2;
                    end if;
                    pi_step <= 0;
                when others =>
            end case;

            -- the raw bits' probe: this bit and the marker now, a word every 7
            if run and count = phase then
                raw_data   <= raw_data(5 downto 0) & data_sync2(g_raw_channel);
                raw_marker <= raw_marker(5 downto 0) & marker_sync2;
                if raw_count < 6 then
                    raw_count <= raw_count + 1;
                else
                    raw_count  <= 0;
                    raw_word   <= std_logic_vector(raw_words) & raw_marker(5 downto 0) & marker_sync2 &
                                  raw_data(5 downto 0) & data_sync2(g_raw_channel);
                    raw_words  <= raw_words + 1;
                    raw_toggle <= not raw_toggle;
                end if;
            end if;

            if run and count = phase then
                for c in 0 to g_channels-1 loop
                    if data_sync2(c) = '1' then
                        bit_value := shift_left(to_signed(1, filter_width), 31);
                    else
                        bit_value := -shift_left(to_signed(1, filter_width), 31);
                    end if;
                    stage0(c) <= stage0(c) + shift_right(bit_value - stage0(c), shift_k);
                    stage1(c) <= stage1(c) + shift_right(stage0(c) - stage1(c), shift_k);
                    stage2(c) <= stage2(c) + shift_right(stage1(c) - stage2(c), shift_k);
                end loop;
                if decimation < 2**log2_r - 1 then
                    decimation <= decimation + 1;
                else
                    decimation <= 0;
                    output_due <= true;
                end if;
                step <= 1;
            end if;

            -- the running sum with the new filtered value into the RAM, the one a window ago out
            if step = 1 then
                for c in 0 to g_channels-1 loop
                    sum := running_sum(c) + resize(shift_right(stage2(c), sum_shift), sum_width);
                    running_sum(c) <= sum;
                    packed((c + 1) * sum_width - 1 downto c * sum_width) := std_logic_vector(sum);
                end loop;
                write_data_to_ram(ram_write_in, to_integer(write_address), packed);
                request_data_from_ram(ram_read_in, to_integer(write_address - to_unsigned(window, 10)));
                write_address <= write_address + 1;
                step <= 2;
            end if;
            if step = 2 then
                step <= 3;
            end if;

            -- the window's sum: the running sum now less the one a window ago
            if ram_read_is_ready(ram_read_out) then
                stored := get_ram_data(ram_read_out);
                for c in 0 to g_channels-1 loop
                    window_sum(c) <= running_sum(c) - signed(stored((c + 1) * sum_width - 1 downto c * sum_width));
                end loop;
                step <= 4;
            end if;

            -- a result: the filtered values and the window's sums, held until the next one
            if step = 4 and output_due then
                output_filtered <= stage2;
                output_sums     <= window_sum;
                result_toggle   <= not result_toggle;
                output_due      <= false;
            end if;
        end if;
    end process;

end architecture rtl;
