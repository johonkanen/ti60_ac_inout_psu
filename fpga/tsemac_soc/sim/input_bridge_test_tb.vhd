LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.scope_pkg.all;

-- input_bridge_test through its registers: a burst of leg 1's low side at 10 % (312 clocks of
-- each 3125, the others never on) for its periods; the duty's limit; stopped at once by the
-- overvoltage, a trip or stop; the link's voltage at the start and its highest read back.
entity input_bridge_test_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of input_bridge_test_tb is

    constant base : natural := 1240;
    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;
    signal trip : boolean := false;
    signal dc_voltage : fixed_number := to_fixed(94.6);
    signal leg1, leg2 : dhb_leg_gates;
    signal probe : scope_sample;

    -- each switch's on clocks, and its longest pulse
    type count_array is array (0 to 3) of natural;
    signal on_count, longest : count_array := (others => 0);

begin

    clock <= not clock after 4 ns;

    dut : entity work.input_bridge_test
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, trip, dc_voltage, leg1, leg2, probe);

    watch : process(clock)
        variable run_length : count_array := (others => 0);
        variable gates : std_logic_vector(0 to 3);
    begin
        if rising_edge(clock) then
            gates := (leg1.low, leg2.low, leg1.high, leg2.high);
            for n in 0 to 3 loop
                if gates(n) = '1' then
                    on_count(n) <= on_count(n) + 1;
                    run_length(n) := run_length(n) + 1;
                    if run_length(n) > longest(n) then
                        longest(n) <= run_length(n);
                    end if;
                else
                    run_length(n) := 0;
                end if;
            end loop;
        end if;
    end process;

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

    main : process
        procedure bus_write(address : natural; data : signed) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(resize(data, 32)));
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
            data := to_integer(signed(bus_read_data));
        end procedure;

        variable v : integer;
        variable started : time;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("a_burst_of_one_switch") then
                -- leg 1's low side, 10 %, 20 periods
                bus_write(base + 1, to_signed(0, 32));
                bus_write(base + 3, to_signed(20, 32));
                bus_write(base, to_signed(1, 32));
                started := now;
                wait until probe(0) = '1' for 1 us;
                wait until probe(0) = '0' for 1 ms;
                check(probe(0) = '0', "the burst over");
                check(now - started > 19.9 * 25 us and now - started < 20.2 * 25 us,
                      "20 periods: " & time'image(now - started));
                check_equal(on_count(0), 20 * 312, "20 pulses of 312 clocks (2.5 us)");
                check_equal(longest(0), 312, "each 312 clocks");
                check_equal(on_count(1) + on_count(2) + on_count(3), 0, "the other switches never on");
                bus_read(base + 5, v);
                check_equal(v, 20, "20 periods run");
                bus_read(base, v);
                check_equal(v, 0, "stopped, not by the overvoltage or a trip");
                bus_read(base + 6, v);
                check_equal(v, to_integer(to_fixed(94.6)), "the link at the start");

            elsif run("each_switch_alone") then
                for s in 1 to 3 loop
                    bus_write(base + 1, to_signed(s, 32));
                    bus_write(base + 3, to_signed(4, 32));
                    bus_write(base, to_signed(1, 32));
                    wait until probe(0) = '1' for 1 us;
                    wait until probe(0) = '0' for 1 ms;
                end loop;
                check_equal(on_count(0), 0, "leg 1's low side not used");
                for s in 1 to 3 loop
                    check_equal(on_count(s), 4 * 312, "switch " & integer'image(s) & ": 4 pulses");
                end loop;

            elsif run("the_duty_is_limited") then
                bus_write(base + 2, to_fixed(0.5));
                bus_write(base + 3, to_signed(3, 32));
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 1 us;
                wait until probe(0) = '0' for 1 ms;
                check_equal(longest(0), 624, "at most 20 %: 624 clocks (0.2 at 20 fractional bits, truncated)");

            elsif run("stopped_at_the_overvoltage") then
                bus_write(base + 3, to_signed(400, 32));
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 1 us;
                wait for 100 us;
                dc_voltage <= to_fixed(110.0);
                wait for 100 us;
                check(probe(0) = '1', "running below the 120 V stop");
                dc_voltage <= to_fixed(120.5);
                wait for 40 ns;
                check(probe(0) = '0' and leg1 = dhb_leg_gates_off, "stopped at once over 120 V");
                bus_read(base, v);
                check_equal(v, 2, "stopped by the overvoltage");
                bus_read(base + 7, v);
                check_equal(v, to_integer(to_fixed(120.5)), "its highest");
                dc_voltage <= to_fixed(94.6);

            elsif run("a_trip_and_stop") then
                bus_write(base + 3, to_signed(400, 32));
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 1 us;
                wait for 100 us;
                trip <= true;
                wait for 30 ns;
                check(leg1 = dhb_leg_gates_off and probe(0) = '0', "a trip: off at once");
                trip <= false;
                bus_read(base, v);
                check_equal(v, 4, "stopped by a trip");
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 1 us;
                wait for 100 us;
                bus_write(base, to_signed(2, 32));
                wait for 30 ns;
                check(probe(0) = '0', "stop: off");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 50 ms);

end vunit_simulation;
