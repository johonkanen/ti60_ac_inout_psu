library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_dsp_pkg.all;
    use work.fixed_format_pkg.all;
    use work.pi_controller_pkg.all;
    use work.dhb_modulator_pkg.all;

-- The DHB's voltage control: the DHB is a DC link isolator, so the secondary DC link follows
-- the primary's. A PI controller (pi_controller) takes the scaled primary DC link as its
-- reference and the scaled secondary as its measurement, runs each time the measurement
-- scaler has a new secondary value, and gives a normalized command, nominally -1.0 .. 1.0,
-- kept within the lower and upper limits. A second fixed_dsp scales that to the phase shift in
-- carrier clock counts for dhb_bridge, rounded: phase = command * scale, the scale a quarter
-- of the carrier period by default (333.25 of 1333 counts), the most power a phase shift moves.
-- > 0 the primary leads: power to the secondary.
--
-- The PI takes the DC links low pass filtered, a first order filter on each new reading,
--   filtered = filtered + (reading - filtered) / 2^n
-- (n 6: a time constant of 64 readings, ~0.6 ms of the secondary's): the readings carry the
-- switching's noise, +-30 V around a ~95 V link. The filters run all the time.
--
-- Disabled, or while the bridge is not running (run false: off or precharging), the integrator
-- is cleared and the command and phase held at 0, so a run starts from phase 0 rather than from
-- what the controller wound up to while the bridge was not following it.
--
-- register map, offsets from g_base_address; fixed point with 20 fractional bits
--   0  RW bit0 enable
--   1  RW kp, command per volt
--   2  RW ki, command per volt and update
--   3  RW upper limit of the command (1.0)
--   4  RW lower limit of the command (-1.0)
--   5  RW scale, counts of phase per 1.0 of command (333.25)
--   6  R  the phase now, counts (signed integer)
--   7  R  updates since power up
--   8  R  the command now
--   9  RW the filters' n, 0..12 (6): 2^n readings of time constant
--   10 R  the filtered primary DC link
--   11 R  the filtered secondary DC link
entity dhb_voltage_controller is
    generic(
        g_base_address      : natural := 1100;
        g_primary_channel   : natural := 9;    -- the measurement scaler's pri 1 vdc_in
        g_secondary_channel : natural := 0;    -- sec 0 vdc_out
        g_product_register  : boolean := true
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        -- from measurement_scaler
        scaled         : in fixed_array;
        scaled_ready   : in boolean;
        scaled_channel : in natural;
        -- the bridge running: the controller acts only then
        run            : in boolean := true;
        -- to dhb_bridge
        phase          : out dhb_phase;
        -- the filtered secondary DC link (register 11), e.g. for the output bridge's modulator
        secondary_voltage : out fixed_number
    );
end entity dhb_voltage_controller;

architecture rtl of dhb_voltage_controller is

    function to_slv(v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_signed(v, 32));
    end function;

    function to_slv(v : real) return std_logic_vector is
    begin
        return std_logic_vector(to_fixed(v));
    end function;

    signal enable_r : std_logic_vector(31 downto 0) := (others => '0');
    signal kp_r     : std_logic_vector(31 downto 0) := (others => '0');
    signal ki_r     : std_logic_vector(31 downto 0) := (others => '0');
    signal high_r   : std_logic_vector(31 downto 0) := to_slv(1.0);
    signal low_r    : std_logic_vector(31 downto 0) := to_slv(-1.0);
    signal scale_r  : std_logic_vector(31 downto 0) := to_slv(1333.0 / 4.0);
    signal updates  : unsigned(31 downto 0) := (others => '0');
    signal phase_r  : dhb_phase := 0;
    signal command  : fixed_number := fixed_zero;
    signal filter_r : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(6, 32));
    signal primary_filtered, secondary_filtered : fixed_number := fixed_zero;

    -- y + (x - y) / 2^n, the difference a bit wider so it cannot wrap
    function low_pass(y, x : fixed_number; n : natural) return fixed_number is
        constant difference : signed(fixed_wordlength downto 0) :=
            resize(x, fixed_wordlength + 1) - resize(y, fixed_wordlength + 1);
    begin
        return y + resize(shift_right(difference, n), fixed_wordlength);
    end function;

    signal pi_in  : pi_controller_in_record := pi_controller_in_init;
    signal pi_out : pi_controller_out_record;

    subtype dsp_in_record  is fixed_dsp_in_record(a(31 downto 0), d(31 downto 0), b(31 downto 0), c(63 downto 0));
    subtype dsp_out_record is fixed_dsp_out_record(result(63 downto 0));
    signal dsp_in  : dsp_in_record := init_fixed_dsp_in(32);
    signal dsp_out : dsp_out_record;

    -- half a count at the product's 40 fractional bits: the rounding
    constant half_count : signed(63 downto 0) := shift_left(to_signed(1, 64), 2 * fixed_radix - 1);

begin

    secondary_voltage <= secondary_filtered;

    phase <= phase_r;

    u_pi : entity work.pi_controller
    generic map(g_product_register => g_product_register)
    port map(clock, pi_in, pi_out);

    u_scaling : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => g_product_register)
    port map(clock => clock, fixed_dsp_in => dsp_in, fixed_dsp_out => dsp_out);

    process(clock)
        variable counts : signed(63 downto 0);
        variable n      : natural range 0 to 12;
        variable secondary_new : fixed_number;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address(bus_in, bus_out, g_base_address + 0, enable_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, kp_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, ki_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, high_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, low_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, scale_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, to_slv(phase_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7, std_logic_vector(updates));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, std_logic_vector(command));
            connect_data_to_address(bus_in, bus_out, g_base_address + 9, filter_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, std_logic_vector(primary_filtered));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, std_logic_vector(secondary_filtered));

            -- the DC links filtered on each new reading
            if unsigned(filter_r) > 12 then
                n := 12;
            else
                n := to_integer(unsigned(filter_r));
            end if;
            secondary_new := secondary_filtered;
            if scaled_ready and scaled_channel = g_primary_channel then
                primary_filtered <= low_pass(primary_filtered, scaled(g_primary_channel), n);
            end if;
            if scaled_ready and scaled_channel = g_secondary_channel then
                secondary_new      := low_pass(secondary_filtered, scaled(g_secondary_channel), n);
                secondary_filtered <= secondary_new;
            end if;

            -- a new secondary voltage: an update
            pi_in.request     <= enable_r(0) = '1' and run and scaled_ready and scaled_channel = g_secondary_channel;
            pi_in.reset       <= enable_r(0) = '0' or not run;
            pi_in.reference   <= primary_filtered;
            pi_in.measurement <= secondary_new;
            pi_in.kp          <= signed(kp_r);
            pi_in.ki          <= signed(ki_r);
            pi_in.limit_high  <= signed(high_r);
            pi_in.limit_low   <= signed(low_r);

            -- the command to counts of phase: command * scale + half a count, rounded down
            init_fixed_dsp(dsp_in);
            if pi_out.ready then
                command <= pi_out.output;
                add(dsp_in, a => pi_out.output, b => signed(scale_r), c => half_count);
            end if;

            if enable_r(0) = '0' or not run then
                command <= fixed_zero;
                phase_r <= 0;
            elsif dsp_out.ready_with_1 = '1' then
                counts := shift_right(dsp_out.result, 2 * fixed_radix);
                if counts > 2**15 - 1 then
                    phase_r <= 2**15 - 1;
                elsif counts < -2**15 then
                    phase_r <= -2**15;
                else
                    phase_r <= to_integer(counts);
                end if;
                updates <= updates + 1;
            end if;
        end if;
    end process;

end architecture rtl;
