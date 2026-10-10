library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;

-- The ADS7056 of the control board behind an 8 input analog mux.
--
-- The ADC and the mux run from adc_clock (200 MHz: 50 MHz SCLK, 2.5 MS/s); the registers
-- and the results are in the main clock domain. Settings cross over when a register is
-- written, the results come back through a dual clock FIFO.
--
-- The enabled mux positions are sampled in turn. The ADC delivers a sample in the frame
-- after the one that took it. Its sample and hold takes the sample at the CS falling edge and
-- holds it 100 ns later, so the mux is switched to the next position hold clocks after CS
-- falls and settles while the frame runs; the next frame starts when settle clocks have
-- passed since the switch. The result read in a frame belongs to the position sampled by
-- the previous frame, the first result after a calibration or a stop is dropped.
--
-- register map, offsets from g_base_address, times in adc_clock clocks
--   0  RW control: bit0 run, bits 15:8 enabled mux positions
--      W: bit1 recalibrate the ADC offset
--   1  RW SCLK: bits 3:0 high clocks, bits 7:4 low clocks
--   2  RW settle clocks after switching the mux. On the control board a floating input
--      leaves charge that takes about 2 us to settle (the default, about 470 kS/s over
--      eight positions); inputs driven at low impedance may use much less, a single
--      enabled position converts at the full rate regardless.
--   3  R  conversions
--   4  R  status: bit0 calibrated, bits 10:8 mux position of the last conversion
--   5  RW clocks between frames (CS high), a frame is 18 SCLKs plus these
--   6  RW sample delay: a bit is read delay+1 clocks after the SCLK falling edge that ends
--      its period. On the board SDO changes 5..10 ns after the SCLK rising edges as seen
--      from the core, at 50 MHz SCLK delays 1 and 2 read the right bits, 3 reads every
--      bit one SCLK late (twice the value)
--   7  R  frames with a bad leading (bits 15:0) / trailing (bits 31:16) zero bit pair
--   8..15  R  latest result of mux position 0..7
--   16 RW hold clocks: the mux switches this long after CS falls, 20 (100 ns) by default
entity board_adc is
    generic(
        g_base_address : natural := 700;
        -- power-up sample delay (register 6): 1 for board_test's ADC, measured on the board
        g_sample_delay : natural range 0 to 31 := 1
    );
    port(
        clock        : in std_logic;
        bus_in       : in fpga_interconnect_record;
        bus_out      : out fpga_interconnect_record;

        -- latest results for the scope, 0..16383
        latest       : out unsigned(13 downto 0);
        positions    : out scope_sample_array(0 to 7);
        is_new       : out boolean;

        adc_clock    : in std_logic;
        mux          : out std_logic_vector(2 downto 0);
        sclk         : out std_logic;
        cs_n         : out std_logic;
        sdo          : in std_logic;

        -- the mux position of the result is_new announces (positions(new_position) is it)
        new_position : out natural range 0 to 7
    );
end entity board_adc;

architecture rtl of board_adc is

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    -- main clock domain
    signal control      : std_logic_vector(31 downto 0) := x"0000FF01";
    signal sclk_setting : std_logic_vector(31 downto 0) := x"00000022";   -- 50 MHz
    signal settle_r     : std_logic_vector(31 downto 0) := to_slv(400);   -- 2 us, see below
    signal cs_high_r    : std_logic_vector(31 downto 0) := to_slv(8);     -- 400 ns frames
    signal delay_r      : std_logic_vector(31 downto 0) := to_slv(g_sample_delay);
    signal hold_r       : std_logic_vector(31 downto 0) := to_slv(20);    -- 100 ns
    signal conversions  : unsigned(31 downto 0) := (others => '0');
    signal leading_errors, trailing_errors : unsigned(15 downto 0) := (others => '0');
    signal status       : std_logic_vector(31 downto 0);
    signal settings_toggle, calibrate_toggle : std_logic := '0';
    signal calibrated_sync : std_logic_vector(1 downto 0) := "00";

    type result_array is array (0 to 7) of std_logic_vector(31 downto 0);
    signal results       : result_array := (others => (others => '0'));
    signal positions_r   : scope_sample_array(0 to 7) := (others => (others => '0'));
    signal latest_r      : unsigned(13 downto 0) := (others => '0');
    signal last_position : natural range 0 to 7 := 0;

    -- result FIFO: trailing error, leading error, position, value
    constant fifo_width : natural := 19;
    signal fifo_write, fifo_read : boolean := false;
    signal fifo_full, fifo_empty : boolean;
    signal fifo_in, fifo_out : std_logic_vector(fifo_width-1 downto 0) := (others => '0');
    signal fifo_valid : boolean_vector(0 to 0) := (others => false);

    -- adc clock domain
    signal toggle_sync, calibrate_sync : std_logic_vector(2 downto 0) := "000";
    signal run          : boolean := true;
    signal mask         : std_logic_vector(7 downto 0) := x"FF";
    signal sclk_high, sclk_low : natural range 1 to 15 := 2;
    signal settle       : natural range 0 to 2**16-1 := 400;
    signal hold         : natural range 0 to 255 := 20;
    signal cs_high      : natural range 1 to 255 := 8;
    signal sample_delay : natural range 0 to 31 := g_sample_delay;
    signal calibrated_bit : std_logic := '0';

    signal start, calibrate : boolean := false;
    signal adc_is_ready, is_calibrated, result_is_new, sample_taken : boolean;
    signal leading_error, trailing_error : boolean;
    signal result : unsigned(13 downto 0);

    signal position          : natural range 0 to 7 := 0;   -- mux selection
    signal frame_position    : natural range 0 to 7 := 0;   -- sampled by the running frame
    signal sampled_position  : natural range 0 to 7 := 0;   -- sampled by the previous frame
    signal sample_is_pending : boolean := false;
    signal frame_has_sample  : boolean := false;
    signal settle_count      : natural range 0 to 2**16-1 := 0;
    signal hold_count        : natural range 0 to 255 := 0;
    signal switch_pending    : boolean := false;            -- the mux moves on after hold

    attribute async_reg : boolean;
    attribute async_reg of toggle_sync, calibrate_sync, calibrated_sync : signal is true;

    function next_enabled(mask : std_logic_vector(7 downto 0); current : natural) return natural is
        variable p : natural range 0 to 7;
    begin
        for i in 1 to 8 loop
            p := (current + i) mod 8;
            if mask(p) = '1' then
                return p;
            end if;
        end loop;
        return current;
    end function;

    function at_least_1(v : std_logic_vector(3 downto 0)) return natural is
    begin
        if v = "0000" then
            return 1;
        end if;
        return to_integer(unsigned(v));
    end function;

    function to_std(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

begin

    status    <= x"00000" & '0' & std_logic_vector(to_unsigned(last_position, 3))
                 & "0000000" & calibrated_sync(1);
    positions <= positions_r;
    latest    <= latest_r;
    mux       <= std_logic_vector(to_unsigned(position, 3));

------------------------------------------------------------------------
    registers : process(clock)
        variable p : natural range 0 to 7;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 0, control);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 1, sclk_setting);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 2, settle_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(conversions));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, status);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 5, cs_high_r);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 6, delay_r);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 16, hold_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7,
                std_logic_vector(trailing_errors) & std_logic_vector(leading_errors));
            for i in 0 to 7 loop
                connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8 + i, results(i));
            end loop;

            -- a written setting is taken over by the adc clock domain
            for offset in 0 to 6 loop
                if write_is_requested_to_address(bus_in, g_base_address + offset) then
                    settings_toggle <= not settings_toggle;
                end if;
            end loop;
            if write_is_requested_to_address(bus_in, g_base_address + 16) then
                settings_toggle <= not settings_toggle;
            end if;
            if write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(1) = '1' then
                calibrate_toggle <= not calibrate_toggle;
            end if;
            calibrated_sync <= calibrated_sync(0) & calibrated_bit;

            -- results from the adc clock domain: fifo_out is valid the clock after a read
            fifo_read  <= not fifo_empty and not fifo_read;
            fifo_valid(0) <= fifo_read and not fifo_empty;
            is_new <= false;
            if fifo_valid(0) then
                p := to_integer(unsigned(fifo_out(16 downto 14)));
                results(p)     <= x"0000" & "00" & fifo_out(13 downto 0);
                positions_r(p) <= "00" & fifo_out(13 downto 0);
                latest_r       <= unsigned(fifo_out(13 downto 0));
                last_position  <= p;
                new_position   <= p;
                is_new         <= true;
                conversions    <= conversions + 1;
                if fifo_out(17) = '1' then
                    leading_errors <= leading_errors + 1;
                end if;
                if fifo_out(18) = '1' then
                    trailing_errors <= trailing_errors + 1;
                end if;
            end if;
        end if;
    end process;

    u_result_fifo : entity work.async_fifo
    generic map(g_width => fifo_width, g_address_bits => 4)
    port map(adc_clock, fifo_write, fifo_in, fifo_full, clock, fifo_read, fifo_out, fifo_empty, open);

------------------------------------------------------------------------
    u_ads7056 : entity work.ads7056
    port map(
        clock           => adc_clock
        ,sclk_high      => sclk_high
        ,sclk_low       => sclk_low
        ,cs_high        => cs_high
        ,sample_delay   => sample_delay
        ,start          => start
        ,calibrate      => calibrate
        ,is_ready       => adc_is_ready
        ,is_calibrated  => is_calibrated
        ,result         => result
        ,result_is_new  => result_is_new
        ,leading_error  => leading_error
        ,trailing_error => trailing_error
        ,sample_taken   => sample_taken
        ,sclk           => sclk
        ,cs_n           => cs_n
        ,sdo            => sdo);

    sequencer : process(adc_clock)
    begin
        if rising_edge(adc_clock) then
            calibrated_bit <= to_std(is_calibrated);

            -- settings: the registers have been stable for several clocks when the toggle arrives
            toggle_sync    <= toggle_sync(1 downto 0) & settings_toggle;
            calibrate_sync <= calibrate_sync(1 downto 0) & calibrate_toggle;
            if toggle_sync(2) /= toggle_sync(1) then
                run          <= control(0) = '1';
                mask         <= control(15 downto 8);
                sclk_high    <= at_least_1(sclk_setting(3 downto 0));
                sclk_low     <= at_least_1(sclk_setting(7 downto 4));
                settle       <= to_integer(unsigned(settle_r(15 downto 0)));
                if cs_high_r(7 downto 0) = x"00" then
                    cs_high <= 1;
                else
                    cs_high <= to_integer(unsigned(cs_high_r(7 downto 0)));
                end if;
                sample_delay <= to_integer(unsigned(delay_r(4 downto 0)));
                hold         <= to_integer(unsigned(hold_r(7 downto 0)));
            end if;
            calibrate <= calibrate_sync(2) /= calibrate_sync(1);

            start      <= false;
            fifo_write <= false;

            -- the mux settles from its last switch
            if settle_count < 2**16-1 then
                settle_count <= settle_count + 1;
            end if;

            if not run or mask = x"00" or calibrate then
                sample_is_pending <= false;
                frame_has_sample  <= false;
            elsif adc_is_ready and not start and not switch_pending and settle_count >= settle then
                if mask(position) = '1' then
                    start <= true;
                else
                    -- the selected position was disabled, move on without a sample
                    position     <= next_enabled(mask, position);
                    settle_count <= 0;
                end if;
            end if;

            if sample_taken then
                -- the sample is taken, the mux moves on once it is held
                frame_position   <= position;
                frame_has_sample <= true;
                hold_count       <= 0;
                switch_pending   <= next_enabled(mask, position) /= position;
            elsif switch_pending then
                if hold_count + 1 >= hold then
                    position       <= next_enabled(mask, position);
                    settle_count   <= 0;
                    switch_pending <= false;
                else
                    hold_count <= hold_count + 1;
                end if;
            end if;

            if result_is_new then
                -- this frame read out the sample of the previous frame
                if sample_is_pending then
                    fifo_in    <= to_std(trailing_error) & to_std(leading_error)
                                  & std_logic_vector(to_unsigned(sampled_position, 3)) & std_logic_vector(result);
                    fifo_write <= true;
                end if;
                sampled_position  <= frame_position;
                sample_is_pending <= frame_has_sample and run;
                frame_has_sample  <= false;
            end if;
        end if;
    end process;

end architecture rtl;
