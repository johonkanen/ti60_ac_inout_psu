library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.dhb_bridge_pkg.all;

-- A hardware test of the DHB: its precharge, a burst of the primary leg alone that stops itself
-- after a set number of carrier periods, its pulses at a fixed width or widening on a ramp, and
-- with a run time set, the bridge running after it for that long: both legs, the phase shift
-- from the voltage controller (dhb_voltage_controller), then everything off. The transformer's
-- winding direction is not known, so the secondary leg's gates can be swapped (inverted), the
-- same as half a period of phase.
--
-- trip (dhb_protection: overcurrent, overvoltage) turns all four gates off at once, ends the
-- burst and keeps it from starting again until the trip is cleared.
--
-- dhb_bridge runs on dhb_clock (200 MHz: 150 kHz is 1333 counts, 100 ns of dead time 20) in its
-- precharge: its start dead time is half a period less the pulse width, and with a ramp the
-- dead time comes down a clock every ramp periods towards the nominal 20 (pulses of half a
-- period less 100 ns): 124 periods a clock takes the 40 clock (200 ns) start pulse to nominal
-- in 500 ms. Its voltages are the measured DC links, primary (pri mux 1, vdc_in) and
-- secondary (sec mux 0, vdc_out), which have the same scaling, so their ADC counts compare
-- directly: the precharge ends when the secondary is within margin counts of the primary
-- (charged); without a run time the burst ends there, with one the bridge runs for it. A
-- precharge also ends after its set number of periods, and everything on stop.
-- The start pulse, the burst length, the ramp's speed and the margin are limited here.
--
-- The precharge compares averages: the readings carry the switching's noise (the secondary's
-- +-15 V at ~95 V during a burst), so a single one can look charged long before it is. Each
-- voltage is read every 64 clocks and 4096 readings averaged (2.1 ms, several of the noise's
-- beats); the averages cross to dhb_clock together, a toggle telling dhb_clock, which takes
-- them when it sees it, the latch then long stable. The phase crosses the same way, latched
-- every 16 clocks, and the bridge takes it at each carrier period start, within +-333 counts.
--
-- register map, offsets from g_base_address
--   0  W  bit0 start a burst (from off), bit1 stop
--      R  bits 1:0 the bridge's mode (0 off, 1 precharging, 2 running, 3 failed),
--         bit 8 a burst running, bit 9 the last burst ended charged
--   1  RW burst length in carrier periods, 1..75000 (3000: 20 ms; at most 500 ms)
--   2  RW pulse width in dhb_clock counts, 1..100 (40: 200 ns, twice the dead time)
--   3  R  carrier periods of the last burst
--   4  RW periods a ramp step of one clock, 0 no ramp (the default), else 20..65535
--   5  RW charged within this many ADC counts of the primary, 0..1000 (84: 5 V)
--   6  RW run time after the precharge in carrier periods, 0..75000 (0: stop when charged)
--   7  RW bit0 the secondary leg inverted (its high and low side gates swapped)
--   8  R  carrier periods run in the last burst
entity dhb_test is
    generic(
        g_base_address : natural := 780
    );
    port(
        clock         : in std_logic;
        bus_in        : in fpga_interconnect_record;
        bus_out       : out fpga_interconnect_record;
        -- a scope probe, synchronous to clock: lane 0 a burst running, 1 and 2 the primary's
        -- high and low side gates, 3 the bridge running, 4 and 5 the secondary's gates
        probe         : out scope_sample;
        -- the phase shift when running, carrier counts, synchronous to clock
        phase         : in dhb_phase;
        -- the protection's trip, synchronous to clock: everything off
        trip          : in boolean;
        -- the bridge running, synchronous to clock: the voltage controller acts only then
        bridge_running : out boolean;
        -- the DC links, ADC counts of the same scaling, synchronous to clock
        primary_voltage   : in scope_sample;
        secondary_voltage : in scope_sample;
        dhb_clock     : in std_logic;
        primary_gates   : out dhb_leg_gates;
        secondary_gates : out dhb_leg_gates
    );
end entity dhb_test;

architecture rtl of dhb_test is

    constant max_pulse     : natural := 100;
    constant max_periods   : natural := 75000;   -- 500 ms: the whole 500 ms ramp
    constant period_counts : natural := 1333;

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    -- registers, clock domain
    signal periods_r      : std_logic_vector(31 downto 0) := to_slv(3000);
    signal pulse_r        : std_logic_vector(31 downto 0) := to_slv(40);
    signal ramp_r         : std_logic_vector(31 downto 0) := to_slv(0);
    signal margin_r       : std_logic_vector(31 downto 0) := to_slv(84);
    signal run_r          : std_logic_vector(31 downto 0) := to_slv(0);
    signal invert_r       : std_logic_vector(31 downto 0) := to_slv(0);
    signal run_count_sync : std_logic_vector(31 downto 0) := (others => '0');
    signal phase_count    : natural range 0 to 15 := 0;
    signal phase_toggle   : std_logic := '0';
    signal phase_hold     : dhb_phase := 0;
    signal volts_count    : natural range 0 to 63 := 0;
    signal volts_toggle   : std_logic := '0';
    signal primary_hold, secondary_hold : natural range 0 to 2**16-1 := 0;
    constant average_bits : natural := 12;          -- 4096 readings
    signal readings       : natural range 0 to 2**average_bits-1 := 0;
    signal primary_sum, secondary_sum : unsigned(16 + average_bits - 1 downto 0) := (others => '0');
    signal start_toggle   : std_logic := '0';
    signal stop_toggle    : std_logic := '0';
    signal status_sync    : std_logic_vector(7 downto 0) := (others => '0');   -- 2 x (mode, burst, charged)
    signal count_sync     : std_logic_vector(31 downto 0) := (others => '0');
    signal gates_sync     : std_logic_vector(9 downto 0) := (others => '0');   -- 2 x 5 lanes

    -- dhb_clock domain
    signal start_sync, stop_sync : std_logic_vector(2 downto 0) := "000";
    signal dhb_in     : dhb_input_record;
    signal start_dhb, stop_dhb : boolean := false;
    signal deadtime_start      : dhb_count := period_counts / 2 - 40;
    signal ramp_step           : dhb_count := 0;
    signal ramp_periods        : dhb_count := 1;
    constant min_ramp_periods  : natural := 20;   -- the fastest ramp: start to nominal in ~80 ms
    signal dhb_out    : dhb_output_record;
    signal bursting   : boolean := false;
    signal burst_periods, burst_count : natural range 0 to max_periods := 0;
    signal mode_bits  : std_logic_vector(1 downto 0) := "00";
    signal burst_bit  : std_logic := '0';
    signal charged_bit : std_logic := '0';   -- the last burst ended charged
    signal running_bit : std_logic := '0';
    signal run_periods, run_count : natural range 0 to max_periods := 0;
    signal inverted    : boolean := false;
    signal phase_sync  : std_logic_vector(2 downto 0) := "000";
    signal phase_now   : dhb_phase := 0;
    signal secondary_pins : dhb_leg_gates := dhb_leg_gates_off;
    signal trip_sync   : std_logic_vector(1 downto 0) := "00";
    signal tripped     : boolean := false;
    signal margin      : natural range 0 to 1000 := 84;
    signal volts_sync  : std_logic_vector(2 downto 0) := "000";
    -- the measured DC links; "not charged" until the first ones arrive
    signal primary_volts   : natural range 0 to 2**20 := 2**20;
    signal secondary_volts : natural range 0 to 2**16-1 := 0;

    attribute async_reg : boolean;
    attribute async_reg of start_sync, stop_sync, status_sync, gates_sync, volts_sync, phase_sync, trip_sync : signal is true;

    function limit(v : std_logic_vector; low, high : natural) return natural is
    begin
        if unsigned(v) > high then
            return high;
        elsif unsigned(v) < low then
            return low;
        end if;
        return to_integer(unsigned(v));
    end function;

begin

    -- tripped, the gates off at once, whatever the bridge does
    primary_gates   <= dhb_leg_gates_off when tripped else dhb_out.primary_gates;
    secondary_gates <= dhb_leg_gates_off when tripped else secondary_pins;
    -- the secondary leg's gates, swapped when inverted: half a period of phase
    secondary_pins  <= (high => dhb_out.secondary_gates.low, low => dhb_out.secondary_gates.high) when inverted
                       else dhb_out.secondary_gates;

------------------------------------------------------------------------
    registers : process(clock)
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                x"00000" & "00" & status_sync(7) & status_sync(6) & "000000" & status_sync(5 downto 4));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, periods_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, pulse_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, ramp_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, margin_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 6, run_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, invert_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8, run_count_sync);

            -- the phase to dhb_clock every 16 clocks
            if phase_count < 15 then
                phase_count <= phase_count + 1;
            else
                phase_count  <= 0;
                phase_hold   <= phase;
                phase_toggle <= not phase_toggle;
            end if;

            -- the voltages read every 64 clocks, 4096 readings averaged, to dhb_clock together
            if volts_count < 63 then
                volts_count <= volts_count + 1;
            else
                volts_count <= 0;
                if readings < 2**average_bits - 1 then
                    readings      <= readings + 1;
                    primary_sum   <= primary_sum + unsigned(primary_voltage);
                    secondary_sum <= secondary_sum + unsigned(secondary_voltage);
                else
                    readings       <= 0;
                    primary_hold   <= to_integer(shift_right(primary_sum + unsigned(primary_voltage), average_bits));
                    secondary_hold <= to_integer(shift_right(secondary_sum + unsigned(secondary_voltage), average_bits));
                    primary_sum    <= (others => '0');
                    secondary_sum  <= (others => '0');
                    volts_toggle   <= not volts_toggle;
                end if;
            end if;
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, count_sync);

            if write_is_requested_to_address(bus_in, g_base_address) then
                if get_slv_data(bus_in)(0) = '1' then
                    start_toggle <= not start_toggle;
                end if;
                if get_slv_data(bus_in)(1) = '1' then
                    stop_toggle <= not stop_toggle;
                end if;
            end if;

            -- back from dhb_clock: the mode and the burst flag, and the count, which changes
            -- once a period (thousands of clocks) and is only read
            status_sync <= status_sync(3 downto 0) & charged_bit & burst_bit & mode_bits;
            bridge_running <= gates_sync(7) = '1';   -- running_bit, through two flip-flops
            count_sync     <= std_logic_vector(to_unsigned(burst_count, 32));
            run_count_sync <= std_logic_vector(to_unsigned(run_count, 32));
            gates_sync  <= gates_sync(4 downto 0) & secondary_pins.low & secondary_pins.high & running_bit
                           & dhb_out.primary_gates.low & dhb_out.primary_gates.high;
            -- lane 0 the burst, 1 and 2 the primary's gates, 3 running, 4 and 5 the secondary's
            probe       <= x"00" & "00" & gates_sync(9 downto 5) & status_sync(6);
        end if;
    end process;

------------------------------------------------------------------------
    test : process(dhb_clock)
        variable pulse : natural range 1 to max_pulse;
    begin
        if rising_edge(dhb_clock) then
            start_sync <= start_sync(1 downto 0) & start_toggle;
            stop_sync  <= stop_sync(1 downto 0) & stop_toggle;
            volts_sync <= volts_sync(1 downto 0) & volts_toggle;
            phase_sync <= phase_sync(1 downto 0) & phase_toggle;
            if phase_sync(2) /= phase_sync(1) then
                phase_now <= phase_hold;
            end if;
            if volts_sync(2) /= volts_sync(1) then
                primary_volts   <= primary_hold;
                secondary_volts <= secondary_hold;
            end if;

            start_dhb <= false;
            stop_dhb  <= false;
            if trip then
                trip_sync <= trip_sync(0) & '1';
            else
                trip_sync <= trip_sync(0) & '0';
            end if;
            tripped   <= trip_sync(1) = '1';

            if stop_sync(2) /= stop_sync(1) or tripped then
                stop_dhb <= true;
                bursting    <= false;
            elsif start_sync(2) /= start_sync(1) and not bursting and dhb_out.state = off and not tripped then
                -- the settings as they are now, limited
                pulse := limit(pulse_r, 1, max_pulse);
                deadtime_start <= period_counts / 2 - pulse;
                burst_periods <= limit(periods_r, 1, max_periods);
                if unsigned(ramp_r) = 0 then
                    ramp_step    <= 0;
                    ramp_periods <= 1;
                else
                    ramp_step    <= 1;
                    ramp_periods <= limit(ramp_r, min_ramp_periods, 2**16-1);
                end if;
                margin        <= limit(margin_r, 0, 1000);
                run_periods   <= limit(run_r, 0, max_periods);
                inverted      <= invert_r(0) = '1';
                run_count     <= 0;
                charged_bit   <= '0';
                burst_count   <= 0;
                bursting      <= true;
                start_dhb <= true;
            elsif bursting then
                if dhb_out.state = failed then
                    stop_dhb <= true;
                    bursting <= false;
                elsif dhb_out.charged and run_periods = 0 then
                    -- the secondary at the primary's voltage and no run time: done, before the
                    -- bridge would run
                    stop_dhb    <= true;
                    bursting    <= false;
                    charged_bit <= '1';
                elsif dhb_out.state = running then
                    -- running: both legs, run_periods of them
                    charged_bit <= '1';
                    if dhb_out.period_start then
                        if run_count >= run_periods then
                            stop_dhb <= true;
                            bursting <= false;
                        else
                            run_count <= run_count + 1;
                        end if;
                    end if;
                elsif dhb_out.period_start and not dhb_out.charged then
                    -- precharging: the start of period n+1 after n whole periods: done
                    if burst_count >= burst_periods then
                        stop_dhb <= true;
                        bursting <= false;
                    else
                        burst_count <= burst_count + 1;
                    end if;
                end if;
            end if;

            mode_bits <= std_logic_vector(to_unsigned(dhb_mode'pos(dhb_out.state), 2));
            if bursting then burst_bit <= '1'; else burst_bit <= '0'; end if;
            if dhb_out.state = running then running_bit <= '1'; else running_bit <= '0'; end if;
        end if;
    end process;

    -- the precharge from the start dead time on the ramp if any, on the measured voltages (at the
    -- nominal dead time it holds there, the burst ends long before the timeout); running, the
    -- controller's phase within a quarter period
    dhb_in <= (start => start_dhb, stop => stop_dhb, phase => phase_now,
               primary_voltage => primary_volts, secondary_voltage => secondary_volts,
               settings => (period => period_counts, max_phase => period_counts / 4, run_deadtime => 20,
                            precharge => (deadtime_start => deadtime_start, deadtime_final => 20,
                                          ramp_step => ramp_step, ramp_periods => ramp_periods,
                                          margin => margin, timeout_periods => 1_000_000)));

    u_bridge : entity work.dhb_bridge
    port map(dhb_clock, dhb_in, dhb_out);

end architecture rtl;
