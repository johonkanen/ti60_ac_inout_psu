library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.output_bridge_pkg.all;
    use work.scope_pkg.all;

-- A test of the input bridge (grid_inu, a full bridge on the AC input): one switch at a time
-- pulsing at a duty for a burst, the other three off, to find the input's polarity. With a DC
-- source on the input, the low side switch of the leg on its positive terminal boosts: the input
-- inductor's current, built while it is on, flows on through that leg's high side body diode into
-- the primary DC link when it turns off, and the link's voltage rises; the other low side only
-- parallels a body diode already carrying the return current. The input current's reading
-- (iac_in) during the burst gives its sensor's sign.
--
-- Only the chosen switch is ever on, so there is no leg to shoot through and no dead time. A burst
-- runs for its periods at 40 kHz (output_bridge_pkg's period) from the period's start, and stops
-- at once on the protection's trip, on stop, or when the primary DC link's voltage passes the
-- overvoltage stop (the link has no load with the DHB off: a boost keeps raising it).
--
-- register map, offsets from g_base_address
--   0  W  bit0 start a burst (when not running), bit1 stop
--      R  bit0 running, bit1 the last burst stopped at the overvoltage, bit2 by a trip
--   1  RW the switch: 0 leg 1's low side, 1 leg 2's low side, 2 leg 1's high side, 3 leg 2's
--         high side
--   2  RW the duty, 20 fractional bits (0.1), at most 0.2
--   3  RW the burst's periods (400: 10 ms), 1 .. 4000 (100 ms)
--   4  RW the overvoltage stop, V, 20 fractional bits (120.0)
--   5  R  periods run in the last burst
--   6  R  the primary DC link at the burst's start, V (20 fractional bits)
--   7  R  its highest during the burst, V
entity input_bridge_test is
    generic(
        g_base_address : natural := 1240
    );
    port(
        clock      : in std_logic;
        bus_in     : in fpga_interconnect_record;
        bus_out    : out fpga_interconnect_record;
        trip       : in boolean;           -- the protection's: everything off
        dc_voltage : in fixed_number;      -- the primary DC link, V
        leg1       : out dhb_leg_gates;
        leg2       : out dhb_leg_gates;
        -- a scope probe: lane 0 running, 1 the switch on, 2 the last burst stopped at the
        -- overvoltage
        probe      : out scope_sample
    );
end entity input_bridge_test;

architecture rtl of input_bridge_test is

    constant period     : natural := output_period_clocks;   -- 3125: 40 kHz
    constant max_bursts : natural := 4000;                   -- 100 ms
    constant max_duty   : fixed_number := to_fixed(0.2);

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    signal switch_r   : std_logic_vector(31 downto 0) := (others => '0');
    signal duty_r     : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(0.1));
    signal periods_r  : std_logic_vector(31 downto 0) := to_slv(400);
    signal limit_r    : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(120.0));

    signal running    : boolean := false;
    signal switch     : natural range 0 to 3 := 0;
    signal on_clocks  : natural range 0 to period := 0;
    signal burst      : natural range 1 to max_bursts := 400;
    signal count      : natural range 0 to period - 1 := 0;
    signal periods    : natural range 0 to max_bursts := 0;
    signal gate_on    : boolean := false;
    signal overvoltage, tripped : boolean := false;
    signal at_start, highest : fixed_number := fixed_zero;

begin

    process(clock)
        variable duty : fixed_number;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                (31 downto 3 => '0') & bit_of(tripped) & bit_of(overvoltage) & bit_of(running));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, switch_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, duty_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, periods_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, limit_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5, to_slv(periods));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, std_logic_vector(at_start));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7, std_logic_vector(highest));

            if running then
                if dc_voltage > highest then
                    highest <= dc_voltage;
                end if;
                -- the periods: the switch on for on_clocks from each one's start
                if count < period - 1 then
                    count <= count + 1;
                else
                    count <= 0;
                    periods <= periods + 1;
                end if;
                gate_on <= count < on_clocks and periods < burst;
            else
                gate_on <= false;
            end if;

            if trip or (write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(1) = '1') then
                running <= false;
                gate_on <= false;
                if trip and running then
                    tripped <= true;    -- a burst stopped by it (not a trip pending while idle)
                end if;
            elsif running and dc_voltage > signed(limit_r) then
                running     <= false;
                gate_on     <= false;
                overvoltage <= true;
            elsif running and periods >= burst then
                running <= false;
                gate_on <= false;
            elsif write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(0) = '1'
                  and not running then
                -- the settings as they are now, limited
                duty := signed(duty_r);
                if duty > max_duty then
                    duty := max_duty;
                elsif duty < 0 then
                    duty := fixed_zero;
                end if;
                on_clocks <= to_integer(shift_right(duty * to_signed(period, 32), fixed_radix)(31 downto 0));
                switch    <= to_integer(unsigned(switch_r(1 downto 0)));
                if unsigned(periods_r) < 1 then
                    burst <= 1;
                elsif unsigned(periods_r) > max_bursts then
                    burst <= max_bursts;
                else
                    burst <= to_integer(unsigned(periods_r));
                end if;
                count       <= 0;
                periods     <= 0;
                overvoltage <= false;
                tripped     <= false;
                at_start    <= dc_voltage;
                highest     <= dc_voltage;
                running     <= true;
            end if;

            -- the one switch, the others off
            leg1 <= dhb_leg_gates_off;
            leg2 <= dhb_leg_gates_off;
            if gate_on and running and not trip then
                case switch is
                    when 0 => leg1.low  <= '1';
                    when 1 => leg2.low  <= '1';
                    when 2 => leg1.high <= '1';
                    when 3 => leg2.high <= '1';
                end case;
            end if;

            probe <= x"000" & '0' & bit_of(overvoltage) & bit_of(gate_on and running) & bit_of(running);
        end if;
    end process;

end architecture rtl;
