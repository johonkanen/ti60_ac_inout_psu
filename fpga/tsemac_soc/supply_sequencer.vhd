library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

-- The supply's start and stop, in order, whatever the software does in between:
--
--   start: the gate driver supplies held on (gate_power brings them up one at a time), and once
--          all are on, settle_ms for the current modulators they power; then the trip latched
--          while they were off (the modulators' nonsense) cleared, and 10 ms later, still no
--          trip, the DHB started (its precharge, as dhb_test's registers have it). Once the
--          DHB runs, the output bridge armed (as output_bridge_test's registers have it), and
--          once it runs: running.
--   stop:  the bus's stop, or stop_in (the face panel's STOP): stop_out pulsed (the output
--          brought down, then the DHB stopped, as the panel's stop does); the supplies stay on.
--   power off: the supplies let go (with a stop first).
--
-- A trip in a start or while running ends it (the trip itself stops everything); so does a
-- step not done in its time (the supplies 2 s, the DHB dhb_timeout_ms, the output 100 ms),
-- stopping what had started. Why the last start ended is kept. A run whose output finishes by
-- itself (its waveform's end) ends when the DHB has stopped after it.
--
-- register map, offsets from g_base_address
--   0  W  bit0 start (when idle), bit1 stop, bit2 power off
--      R  bits 3:0 the state (0 idle, 1 the supplies coming on, 2 settling, 3 the trip
--         checked, 4 the DHB starting, 5 the output starting, 6 running, 7 stopping),
--         bit 8 the supplies held on, bits 15:12 why the last start ended (0 none or a stop,
--         1 a trip, 2 the supplies not on in time, 3 the DHB not running in time, 4 the
--         output not running in time, 5 the output and the DHB stopped by themselves)
--   1  RW settle time after the supplies are on, ms (200)
--   2  RW the DHB's time to run, ms (2000)
entity supply_sequencer is
    generic(
        g_base_address : natural := 1340;
        g_clocks_a_ms  : natural := 125000
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        trip           : in boolean;
        gates_all_on   : in boolean;
        dhb_running    : in boolean;    -- the DHB's bridge running (after its precharge)
        output_running : in boolean;
        output_active  : in boolean;    -- armed or running, its ramp down too
        stop_in        : in boolean;    -- a clock's pulse: stop
        gates_on       : out boolean;   -- the supplies held on
        clear_trip     : out boolean;   -- pulses
        dhb_start      : out boolean;
        output_arm     : out boolean;
        stop_out       : out boolean
    );
end entity supply_sequencer;

architecture rtl of supply_sequencer is

    type state_type is (s_idle, s_gates, s_settle, s_check, s_dhb, s_output, s_running, s_stopping);
    signal state : state_type := s_idle;

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    signal settle_r  : std_logic_vector(31 downto 0) := to_slv(200);
    signal dhb_ms_r  : std_logic_vector(31 downto 0) := to_slv(2000);
    signal hold      : boolean := false;
    signal why       : natural range 0 to 15 := 0;
    signal ms_count  : natural range 0 to g_clocks_a_ms - 1 := 0;
    signal elapsed   : natural range 0 to 65535 := 0;     -- ms in this state
    signal pulse_clear, pulse_dhb, pulse_arm, pulse_stop : boolean := false;

begin

    gates_on   <= hold;
    clear_trip <= pulse_clear;
    dhb_start  <= pulse_dhb;
    output_arm <= pulse_arm;
    stop_out   <= pulse_stop;

    process(clock)
        procedure go(next_state : state_type) is
        begin
            state    <= next_state;
            elapsed  <= 0;
            ms_count <= 0;
        end procedure;

        -- a start ended: what had started stopped
        procedure fail(reason : natural) is
        begin
            why        <= reason;
            pulse_stop <= true;
            go(s_stopping);
        end procedure;

        variable command : std_logic_vector(2 downto 0);
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                x"0000" & std_logic_vector(to_unsigned(why, 4)) & "000" & bit_of(hold)
                & x"0" & std_logic_vector(to_unsigned(state_type'pos(state), 4)));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, settle_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, dhb_ms_r);

            pulse_clear <= false;
            pulse_dhb   <= false;
            pulse_arm   <= false;
            pulse_stop  <= false;

            if ms_count < g_clocks_a_ms - 1 then
                ms_count <= ms_count + 1;
            else
                ms_count <= 0;
                if elapsed < 65535 then
                    elapsed <= elapsed + 1;
                end if;
            end if;

            command := "000";
            if write_is_requested_to_address(bus_in, g_base_address) then
                command := get_slv_data(bus_in)(2 downto 0);
            end if;

            case state is
                when s_idle =>
                    if command(0) = '1' and command(1) = '0' and command(2) = '0' then
                        why  <= 0;
                        hold <= true;
                        go(s_gates);
                    end if;

                when s_gates =>
                    if gates_all_on then
                        go(s_settle);
                    elsif elapsed >= 2000 then
                        fail(2);
                    end if;

                when s_settle =>
                    if elapsed >= to_integer(unsigned(settle_r(15 downto 0))) then
                        pulse_clear <= true;
                        go(s_check);
                    end if;

                when s_check =>
                    if elapsed >= 10 then
                        if trip then
                            fail(1);
                        else
                            pulse_dhb <= true;
                            go(s_dhb);
                        end if;
                    end if;

                when s_dhb =>
                    if trip then
                        fail(1);
                    elsif dhb_running then
                        pulse_arm <= true;
                        go(s_output);
                    elsif elapsed >= to_integer(unsigned(dhb_ms_r(15 downto 0))) then
                        fail(3);
                    end if;

                when s_output =>
                    if trip then
                        fail(1);
                    elsif output_running then
                        go(s_running);
                    elsif elapsed >= 100 then
                        fail(4);
                    end if;

                when s_running =>
                    if trip then
                        fail(1);
                    elsif not output_active and not dhb_running then
                        why <= 5;       -- finished by itself (the waveform's end)
                        go(s_idle);
                    end if;

                when s_stopping =>
                    -- the output brought down and the DHB stopped (the panel's stop path)
                    if not output_active and not dhb_running and elapsed >= 1 then
                        go(s_idle);
                    end if;
            end case;

            -- a stop, whatever the state: everything stopped, the supplies kept; power off lets
            -- them go too
            if command(1) = '1' or command(2) = '1' or stop_in then
                if state /= s_idle and state /= s_stopping then
                    pulse_stop <= true;
                    go(s_stopping);
                end if;
                if command(2) = '1' then
                    hold <= false;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
