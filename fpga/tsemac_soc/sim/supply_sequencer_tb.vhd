LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- supply_sequencer against a model of what it starts (a millisecond here 100 clocks): the gate
-- supplies all on 5 * sim_ms after held on; a trip latched while they were off, cleared by its pulse
-- (or not, a real one); the DHB running 5 * sim_ms after its start, stopping when the output is no
-- longer active after a stop; the output running 1 * sim_ms after its arm, brought down in 2 * sim_ms on a
-- stop. Checked: a start in order (the supplies, the trip cleared, the DHB, then the output),
-- stop, power off, the panel's stop, a trip, a DHB that does not run, a run that ends by itself.
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

    signal trip, gates_all_on, dhb_running, output_running, output_active, stop_in : boolean := false;
    signal gates_on, clear_trip, dhb_start, output_arm, stop_out : boolean;

    -- the model
    signal latched_trip  : boolean := true;    -- latched while the supplies were off
    signal real_trip     : boolean := false;   -- a trip a clear does not clear
    signal dhb_never     : boolean := false;   -- the DHB does not come to run
    signal finish_output : boolean := false;   -- the output's waveform ends
    signal t_gates, t_clear, t_dhb, t_arm : time := 0 ns;

begin

    clock <= not clock after 4 ns;

    dut : entity work.supply_sequencer
    generic map(g_base_address => base, g_clocks_a_ms => 100)
    port map(clock, bus_in, bus_out, trip, gates_all_on, dhb_running, output_running, output_active,
             stop_in, gates_on, clear_trip, dhb_start, output_arm, stop_out);

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

    dhb : process
    begin
        wait until rising_edge(clock) and dhb_start;
        t_dhb <= now;
        if not dhb_never then
            wait for 5 * sim_ms;
            dhb_running <= true;
            -- running until the output, once active, is no longer (a stop, or its end); a
            -- stop or a trip before the output came stops it too
            wait until rising_edge(clock) and (output_active or stop_out or trip);
            if output_active then
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
        wait until rising_edge(clock) and (stop_out or finish_output or trip);
        if stop_out or finish_output then
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

        procedure start_and_run is
        begin
            bus_write(base, 1);
            wait_state(6, 300 * sim_ms);
            check(gates_on, "the supplies held on");
            check(t_gates < t_clear and t_clear < t_dhb and t_dhb < t_arm,
                  "in order: the supplies " & time'image(t_gates) & ", the trip cleared " & time'image(t_clear) &
                  ", the DHB " & time'image(t_dhb) & ", the output " & time'image(t_arm));
            check(t_clear - t_gates >= 200 * sim_ms, "the supplies settled 200 * sim_ms first");
            check(not trip and dhb_running and output_running, "running, no trip: trip " & boolean'image(trip) &
                  ", the DHB " & boolean'image(dhb_running) & ", the output " & boolean'image(output_running));
        end procedure;

        variable v : integer;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("starts_in_order_and_stops") then
                start_and_run;
                bus_write(base, 2);
                wait_state(0, 20 * sim_ms);
                check(not output_active and not dhb_running, "stopped: the output, then the DHB");
                check(gates_on, "the supplies kept on");
                check_why(0, "a stop");
                -- and again, the supplies already on
                start_and_run;
                bus_write(base, 4);
                wait_state(0, 20 * sim_ms);
                check(not gates_on and not dhb_running and not output_active, "power off: stopped, the supplies let go");

            elsif run("the_panel_stop") then
                start_and_run;
                wait until rising_edge(clock);
                stop_in <= true;
                wait until rising_edge(clock);
                stop_in <= false;
                wait_state(0, 20 * sim_ms);
                check(not output_active and not dhb_running and gates_on, "stopped by the panel, the supplies on");

            elsif run("a_trip_while_running") then
                start_and_run;
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
                check(not dhb_running and not output_active, "nothing started");
                real_trip <= false;

            elsif run("a_real_trip_stops_the_start") then
                real_trip <= true;
                bus_write(base, 1);
                wait_state(0, 300 * sim_ms);
                check_why(1, "a trip that the clear did not clear");
                check(t_dhb = 0 ns, "the DHB never started");

            elsif run("the_dhb_not_running") then
                dhb_never <= true;
                bus_write(base + 2, 50);
                bus_write(base, 1);
                wait_state(0, 400 * sim_ms);
                check_why(3, "the DHB not running in time");
                check(t_arm = 0 ns, "the output never armed");

            elsif run("a_run_that_ends_by_itself") then
                start_and_run;
                finish_output <= true;
                wait_state(0, 20 * sim_ms);
                check_why(5, "finished by itself");
                check(gates_on, "the supplies kept on");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 sec);

end vunit_simulation;
