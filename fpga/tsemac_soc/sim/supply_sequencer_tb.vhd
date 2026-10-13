LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- supply_sequencer against a model of what it starts (a millisecond here 100 clocks): the gate
-- supplies all on 5 * sim_ms after held on; a trip latched while they were off, cleared by its pulse
-- (or not, a real one); the boost running 1 * sim_ms after its start, at its reference 20 * sim_ms
-- after (or never), stopping on a stop or a trip (or by itself); the DHB running 5 * sim_ms after its
-- start, held running while dhb_hold or the output is active, stopping on a stop or a trip; the
-- output running 1 * sim_ms after its arm, brought down in 2 * sim_ms on its stop or the stop.
-- Checked: a start in order to idle (the supplies, the trip cleared, the boost, the DHB once the
-- boost is at its reference; the output off), the output on and off from idle, stop, power off,
-- the panel's stop, a trip, the boost not at its reference in time or stopping while up, a DHB
-- that does not run, an output that ends by itself (back to idle), the output on only from idle.
entity supply_sequencer_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of supply_sequencer_tb is

    constant base : natural := 1340;
    constant sim_ms : time := 100 * 8 ns;     -- the DUT's millisecond
    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal trip, gates_all_on, boost_running, boost_at_reference, dhb_running, output_running,
           output_active, stop_in : boolean := false;
    signal gates_on, clear_trip, boost_start, dhb_start, dhb_hold, output_arm, output_stop, stop_out : boolean;

    -- the model
    signal latched_trip  : boolean := true;    -- latched while the supplies were off
    signal real_trip     : boolean := false;   -- a trip a clear does not clear
    signal boost_never   : boolean := false;   -- the boost never at its reference
    signal boost_fails   : boolean := false;   -- the boost stops by itself
    signal dhb_never     : boolean := false;   -- the DHB does not come to run
    signal finish_output : boolean := false;   -- the output's waveform ends
    signal t_gates, t_clear, t_boost, t_reference, t_dhb, t_arm : time := 0 ns;

begin

    clock <= not clock after 4 ns;

    dut : entity work.supply_sequencer
    generic map(g_base_address => base, g_clocks_a_ms => 100)
    port map(clock, bus_in, bus_out, trip, gates_all_on, boost_running, boost_at_reference, dhb_running,
             output_running, output_active, stop_in, gates_on, clear_trip, boost_start, dhb_start, dhb_hold,
             output_arm, output_stop, stop_out);

    capture_bus_reads : process(clock)
    begin
        if rising_edge(clock) then
            bus_read_done <= false;
            if write_to_address_is_requested(bus_out, 0) then
                bus_read_data <= get_slv_data(bus_out);
                bus_read_done <= true;
            end if;
        end if;
    end process;

    trip <= latched_trip or real_trip;

    model : process
    begin
        wait until rising_edge(clock);
        -- the supplies
        if gates_on and not gates_all_on then
            wait for 5 * sim_ms;
            if gates_on then gates_all_on <= true; t_gates <= now; end if;
        elsif not gates_on then
            gates_all_on <= false;
        end if;
    end process;

    trips : process(clock)
    begin
        if rising_edge(clock) then
            if clear_trip then
                latched_trip <= false;
                t_clear <= now;
            end if;
            if not gates_on then
                latched_trip <= true;
            end if;
        end if;
    end process;

    boost : process
    begin
        wait until rising_edge(clock) and boost_start;
        t_boost <= now;
        wait for 1 * sim_ms;
        boost_running <= true;
        if not boost_never then
            wait for 20 * sim_ms;
            boost_at_reference <= true;
            t_reference <= now;
        end if;
        wait until rising_edge(clock) and (stop_out or trip or boost_fails);
        boost_running <= false;
        boost_at_reference <= false;
    end process;

    dhb : process
    begin
        wait until rising_edge(clock) and dhb_start;
        t_dhb <= now;
        if not dhb_never then
            wait for 5 * sim_ms;
            dhb_running <= true;
            -- running while held (the supply up) or the output active; a stop or a trip stops it
            wait until rising_edge(clock) and (stop_out or trip or not (dhb_hold or output_active));
            if stop_out and output_active then
                wait until not output_active;
            end if;
            dhb_running <= false;
        end if;
    end process;

    output : process
    begin
        wait until rising_edge(clock) and output_arm;
        t_arm <= now;
        output_active <= true;
        wait for 1 * sim_ms;
        output_running <= true;
        wait until rising_edge(clock) and (stop_out or output_stop or finish_output or trip);
        if not trip then
            wait for 2 * sim_ms;      -- brought down
        end if;
        output_running <= false;
        output_active  <= false;
    end process;

    main : process
        procedure bus_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(to_unsigned(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        procedure bus_read(address : natural; data : out integer) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := to_integer(unsigned(bus_read_data));
        end procedure;

        procedure read_state(s, w : out natural) is
            variable v : integer;
        begin
            bus_read(base, v);
            s := v mod 16;
            w := (v / 4096) mod 16;
        end procedure;

        procedure check_why(expected : natural; what : string) is
            variable s, w : natural;
        begin
            read_state(s, w);
            check_equal(w, expected, what);
        end procedure;

        procedure wait_state(expected : natural; limit : time) is
            constant until_time : time := now + limit;
            variable s, w : natural;
        begin
            loop
                read_state(s, w);
                exit when s = expected or now >= until_time;
                wait for 20 us;
            end loop;
            check_equal(s, expected, "the state");
        end procedure;

        -- a start to idle: the boost and the DHB up, the output off
        procedure start_to_idle is
        begin
            bus_write(base, 1);
            wait_state(6, 300 * sim_ms);
            check(gates_on, "the supplies held on");
            check(t_gates < t_clear and t_clear < t_boost and t_reference < t_dhb,
                  "in order: the supplies " & time'image(t_gates) & ", the trip cleared " & time'image(t_clear) &
                  ", the boost " & time'image(t_boost) & ", at its reference " & time'image(t_reference) &
                  ", the DHB " & time'image(t_dhb));
            check(t_clear - t_gates >= 200 * sim_ms, "the supplies settled 200 * sim_ms first");
            check(t_dhb - t_reference >= 50 * sim_ms, "the boost at its reference 50 * sim_ms first");
            check(not trip and boost_running and dhb_running, "idle: the boost and the DHB running, no trip");
            check(not output_active, "the output off");
        end procedure;

        procedure output_on is
        begin
            bus_write(base, 8);
            wait_state(8, 10 * sim_ms);
            check(output_running and dhb_running and boost_running, "running: the output on the DHB, the boost");
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("starts_to_idle_in_order_the_output_on_and_off_and_stops") then
                start_to_idle;
                wait for 10 * sim_ms;
                check(dhb_running and not output_active, "idle holds: the DHB on, the output off");
                output_on;
                bus_write(base, 16);
                wait_state(6, 10 * sim_ms);
                check(not output_active and dhb_running and boost_running, "the output off: idle again");
                bus_write(base, 2);
                wait_state(0, 20 * sim_ms);
                check(not output_active and not dhb_running and not boost_running, "stopped: all of it");
                check(gates_on, "the supplies kept on");
                check_why(0, "a stop");
                -- and again, the supplies already on, then power off from running
                start_to_idle;
                output_on;
                bus_write(base, 4);
                wait_state(0, 20 * sim_ms);
                check(not gates_on and not dhb_running and not output_active and not boost_running,
                      "power off: stopped, the supplies let go");

            elsif run("the_output_on_only_from_idle") then
                bus_write(base, 8);
                wait for 20 * sim_ms;
                check(t_arm = 0 ns and not gates_on, "off: the output on does nothing");

            elsif run("the_panel_stop") then
                start_to_idle;
                output_on;
                wait until rising_edge(clock);
                stop_in <= true;
                wait until rising_edge(clock);
                stop_in <= false;
                wait_state(0, 20 * sim_ms);
                check(not output_active and not dhb_running and not boost_running and gates_on,
                      "stopped by the panel, the supplies on");

            elsif run("a_trip_while_running") then
                start_to_idle;
                output_on;
                real_trip <= true;
                wait_state(0, 20 * sim_ms);
                check_why(1, "ended by a trip");
                -- a start with the trip still there: through the supplies and their settling,
                -- then ended by it again
                bus_write(base, 1);
                wait for 2 * sim_ms;
                check_why(0, "a new start");
                wait_state(0, 300 * sim_ms);
                check_why(1, "ended by the trip again");
                check(not dhb_running and not output_active and not boost_running, "nothing started");
                real_trip <= false;

            elsif run("a_real_trip_stops_the_start") then
                real_trip <= true;
                bus_write(base, 1);
                wait_state(0, 300 * sim_ms);
                check_why(1, "a trip that the clear did not clear");
                check(t_boost = 0 ns and t_dhb = 0 ns, "neither the boost nor the DHB started");

            elsif run("the_boost_not_at_its_reference") then
                boost_never <= true;
                bus_write(base + 3, 100);
                bus_write(base, 1);
                wait_state(0, 400 * sim_ms);
                check_why(6, "the boost not at its reference in time");
                check(t_dhb = 0 ns and not boost_running, "the DHB never started, the boost stopped");

            elsif run("the_boost_stopping_while_up") then
                start_to_idle;
                boost_fails <= true;
                wait_state(0, 20 * sim_ms);
                check_why(7, "the boost stopped by itself");
                check(not dhb_running and not output_active, "all of it stopped");
                boost_fails <= false;

            elsif run("the_dhb_not_running") then
                dhb_never <= true;
                bus_write(base + 2, 50);
                bus_write(base, 1);
                wait_state(0, 400 * sim_ms);
                check_why(3, "the DHB not running in time");
                check(t_arm = 0 ns and not boost_running, "the output never armed, the boost stopped");

            elsif run("an_output_that_ends_by_itself") then
                start_to_idle;
                output_on;
                finish_output <= true;
                wait_state(6, 20 * sim_ms);
                check(dhb_running and boost_running and not output_active, "back at idle, the DHB and the boost on");
                check_why(0, "nothing ended");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 sec);

end vunit_simulation;
