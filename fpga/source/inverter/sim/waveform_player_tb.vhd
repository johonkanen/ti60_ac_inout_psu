LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;

-- waveform_player as output_bridge_test uses it: the table written over the bus, a start,
-- then at each period start the reference taken and a step; the references period by period
-- against the points, the passes and the finish, "until stopped", a full table.
entity waveform_player_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of waveform_player_tb is

    constant base : natural := 1200;
    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal start, step : boolean := false;
    signal running : boolean := true;
    signal v_ref : fixed_number;
    signal finished : boolean;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

begin

    clock <= not clock after 4 ns;

    dut : entity work.waveform_player
    generic map(g_base_address => base, g_points => 512)
    port map(clock, bus_in, bus_out, start, step, running, v_ref, finished);

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

        type real_array is array (natural range <>) of real;
        type natural_array is array (natural range <>) of natural;

        procedure load(voltages : real_array; periods : natural_array; passes : natural) is
        begin
            bus_write(base + 0, to_signed(voltages'length, 32));
            bus_write(base + 1, to_signed(passes, 32));
            bus_write(base + 2, to_signed(0, 32));
            for n in voltages'range loop
                bus_write(base + 3, to_fixed(voltages(n)));
                bus_write(base + 4, to_signed(periods(n), 32));
            end loop;
            wait for 100 ns;
        end procedure;

        procedure pulse(signal s : out boolean) is
        begin
            wait until rising_edge(clock);
            s <= true;
            wait until rising_edge(clock);
            s <= false;
        end procedure;

        -- a period of the bridge: the reference taken at its start, then the step; a few
        -- hundred clocks apart (a period is 3125)
        procedure period(value : out real) is
        begin
            wait until rising_edge(clock);
            value := to_real(v_ref);
            pulse(step);
            for n in 1 to 20 loop
                wait until rising_edge(clock);
            end loop;
        end procedure;

        procedure expect(expected : real_array; what : string) is
            variable value : real;
        begin
            for n in expected'range loop
                period(value);
                check(abs(value - expected(n)) < 1.0e-5, what & ", period " & integer'image(n) & ": " &
                      real'image(value) & " V, expected " & real'image(expected(n)) & " V");
            end loop;
        end procedure;

        variable value : real;
        variable status, passes : integer;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("the_points_in_order_and_the_passes") then
                load((1.0, -2.0, 5.0), (3, 1, 2), 2);
                pulse(start);
                expect((1.0, 1.0, 1.0, -2.0, 5.0, 5.0, 1.0, 1.0, 1.0, -2.0, 5.0, 5.0), "two passes");
                check(finished, "finished after the second pass");
                bus_read(base + 5, status);
                check_equal(status mod 4, 2, "finished, not playing");
                bus_read(base + 6, passes);
                check_equal(passes, 2, "two passes counted");
                for n in 1 to 3 loop
                    period(value);
                    check(abs(value - 5.0) < 1.0e-5, "the last point held after the end");
                end loop;
                -- a start again: the first point at once
                running <= false;
                pulse(step);
                running <= true;
                wait for 100 ns;
                pulse(start);
                wait for 1 ns;   -- taken at that edge
                check(not finished, "a start clears finished");
                expect((1.0, 1.0, 1.0, -2.0), "started again");

            elsif run("until_stopped") then
                load((3.0, 4.0), (1, 2), 0);
                pulse(start);
                for pass in 1 to 20 loop
                    expect((3.0, 4.0, 4.0), "pass " & integer'image(pass));
                end loop;
                check(not finished, "never finished");
                bus_read(base + 6, passes);
                check_equal(passes, 20, "the passes counted");
                running <= false;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                bus_read(base + 5, status);
                check_equal(status mod 2, 0, "stopped");

            elsif run("one_point") then
                load((0 => 7.5), (0 => 4), 3);
                pulse(start);
                expect((7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5, 7.5), "one point, three passes");
                check(finished, "finished");

            elsif run("a_full_table") then
                -- 512 points of a period each: -255.5 .. 255.5 V; the index past the last wraps to 0
                bus_write(base + 0, to_signed(512, 32));
                bus_write(base + 1, to_signed(1, 32));
                bus_write(base + 2, to_signed(0, 32));
                for n in 0 to 511 loop
                    bus_write(base + 3, to_fixed(real(n) - 255.5));
                    bus_write(base + 4, to_signed(1, 32));
                end loop;
                bus_read(base + 2, status);
                check_equal(status, 0, "the index wrapped after the last entry");
                wait for 100 ns;
                pulse(start);
                for n in 0 to 511 loop
                    period(value);
                    check(abs(value - (real(n) - 255.5)) < 1.0e-5, "point " & integer'image(n) & ": " & real'image(value));
                end loop;
                check(finished, "finished after the 512th");
                bus_read(base + 5, status);
                check_equal(status / 65536, 511, "the last point held");

            elsif run("periods_of_zero_count_as_one") then
                load((1.0, 2.0), (0, 2), 1);
                pulse(start);
                expect((1.0, 2.0, 2.0), "a hold of 0 is one period");
                check(finished, "finished");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
