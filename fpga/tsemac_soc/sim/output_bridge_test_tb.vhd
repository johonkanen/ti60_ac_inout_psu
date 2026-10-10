LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.fixed_format_pkg.all;
    use work.output_bridge_pkg.all;
    use work.dhb_modulator_pkg.all;

-- output_bridge_test through its registers: armed, it waits for the DHB and starts with it, a
-- duty step after the set periods, the end after all of them, and off at once when the DHB stops
-- or on a trip; in voltage mode the duty from the voltage reference and the DC link, stepping
-- the voltage.
entity output_bridge_test_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of output_bridge_test_tb is

    constant base   : natural := 1140;
    constant period : natural := output_period_clocks;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal allowed, trip : boolean := false;
    signal leg1, leg2    : dhb_leg_gates;
    signal probe         : scope_sample;
    signal sample        : boolean;
    signal running_out   : boolean;
    signal applied_reference : fixed_number;
    signal closed_loop_out, period_start_out : boolean;
    signal voltage_reference : fixed_number;
    signal control_u     : fixed_number := fixed_zero;
    signal control_ready : boolean := false;
    signal both_on       : natural := 0;
    signal dc_voltage    : fixed_number := to_fixed(94.3);

begin

    clock <= not clock after 4 ns;

    dut : entity work.output_bridge_test
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, allowed, trip, dc_voltage, leg1, leg2, sample, running_out, applied_reference,
             closed_loop_out, voltage_reference, period_start_out, control_u, control_ready, probe);

    watch : process(clock)
    begin
        if rising_edge(clock) then
            if (leg1.high and leg1.low) = '1' or (leg2.high and leg2.low) = '1' then
                both_on <= both_on + 1;
            end if;
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

        -- leg 1's next high side pulse, in clocks
        procedure width(clks : out integer) is
            variable rise : time;
        begin
            wait until rising_edge(leg1.high) for 1 ms;
            rise := now;
            wait until falling_edge(leg1.high);
            clks := (now - rise) / 8 ns;
        end procedure;

        variable v, w : integer;
        variable started : time;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            bus_write(base + 1, to_fixed(0.5));
            bus_write(base + 2, to_fixed(0.6));
            bus_write(base + 3, to_signed(20, 32));
            bus_write(base + 4, to_signed(40, 32));

            if run("armed_it_waits_for_the_dhb") then
                bus_write(base, to_signed(1, 32));
                wait for 200 us;
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "nothing without the DHB running");
                bus_read(base, v);
                check_equal(v, 4, "armed, not running");
                allowed <= true;
                wait until probe(0) = '1' for 30 us;   -- the start: the sample timer's next period
                check(probe(0) = '1', "running once the DHB runs");

            elsif run("a_step_after_the_set_periods") then
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 30 us;   -- the start: the sample timer's next period
                started := now;
                width(w);   -- the first, its dead time counted from the enable
                width(w);
                check_equal(w, period / 2 - 13, "0.5 before the step");
                wait until probe(1) = '1' for 2 ms;
                info("stepped " & time'image(now - started) & " after the start");
                check(now - started > 19 * period * 8 ns and now - started < 21 * period * 8 ns, "after 20 periods");
                width(w);
                width(w);
                check_equal(w, integer(real(period) * 0.6 - 0.5) - 13, "0.6 after it");
                wait until probe(0) = '0' for 2 ms;
                check(now - started > 39 * period * 8 ns and now - started < 41 * period * 8 ns, "40 periods in all");
                bus_read(base + 6, v);
                check_equal(v, 40, "40 periods run");
                wait for 1 us;
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "off after them");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("voltage_mode_steps_the_voltage") then
                bus_write(base + 7, to_signed(1, 32));
                bus_write(base + 8, to_fixed(-18.86));   -- 0.4 of 94.3 V
                bus_write(base + 9, to_fixed(37.72));    -- 0.7
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait until probe(0) = '1' for 30 us;   -- the start: the sample timer's next period
                width(w);
                width(w);
                width(w);
                check(abs(w - (integer(real(period) * 0.4 - 0.5) - 13)) <= 1, "-18.86 V of 94.3 V: duty 0.4, " & integer'image(w));
                wait until probe(1) = '1' for 2 ms;
                width(w);
                width(w);
                check(abs(w - (integer(real(period) * 0.7 - 0.5) - 13)) <= 1, "37.72 V: duty 0.7, " & integer'image(w));
                check(applied_reference = to_fixed(37.72), "the applied reference the one after the step");
                -- the duty follows the DC link: the same 37.72 V of 75.44 V is all of it
                dc_voltage <= to_fixed(75.44);
                width(w);
                width(w);
                check(abs(w - (integer(real(period) * 0.75 - 0.5) - 13)) <= 1, "37.72 V of 75.44 V: duty 0.75, " & integer'image(w));
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("closed_loop_square_wave_and_references") then
                -- a square of +-15 V, 3 periods a half; a stand in control answering each period
                -- start with 18.86 V (duty 0.6 of 94.3 V) a microsecond later
                bus_write(base + 7, to_signed(2, 32));
                bus_write(base + 10, to_fixed(15.0));
                bus_write(base + 11, to_signed(3, 32));
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait until closed_loop_out for 30 us;
                check(closed_loop_out, "closed loop running");
                check(voltage_reference = to_fixed(-15.0), "the square wave starts low");
                for k in 1 to 8 loop
                    wait until period_start_out;
                    wait for 20 ns;   -- its registers' updates
                    if k = 3 then
                        check(voltage_reference = to_fixed(15.0), "high after 3 periods");
                        check(probe(1) = '1', "lane 1 the square wave high");
                    elsif k = 6 then
                        check(voltage_reference = to_fixed(-15.0), "low again after 3 more");
                    end if;
                    if k >= 3 then
                        check(applied_reference = to_fixed(18.86), "the control's reference applied the period after");
                    end if;
                    wait for 1 us;
                    wait until rising_edge(clock);
                    control_u     <= to_fixed(18.86);
                    control_ready <= true;
                    wait until rising_edge(clock);
                    control_ready <= false;
                end loop;
                width(w);
                width(w);
                check(abs(w - (integer(real(period) * 0.6 - 0.5) - 13)) <= 1, "the control's reference as the duty, " & integer'image(w));
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("closed_loop_ends_with_the_ramp_down") then
                -- 10 periods of the square wave, then the reference at 0 V for 200 more (5 ms),
                -- the bridge running, then off
                bus_write(base + 4, to_signed(10, 32));
                bus_write(base + 7, to_signed(2, 32));
                bus_write(base + 10, to_fixed(15.0));
                bus_write(base + 11, to_signed(3, 32));
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait until closed_loop_out for 30 us;
                started := now;
                for k in 1 to 10 loop
                    wait until period_start_out;
                end loop;
                wait for 20 ns;
                check(voltage_reference = fixed_zero, "the reference 0 V after the run's periods");
                check(closed_loop_out, "still running: the output brought down");
                bus_read(base + 6, v);
                check_equal(v, 10, "the run's periods");
                wait until not closed_loop_out for 6 ms;
                info("off " & time'image(now - started) & " after the start");
                check(not closed_loop_out, "off after the ramp down");
                check(now - started > 209 * 25 us and now - started < 211 * 25 us, "off 200 periods after the run's 10");
                bus_read(base + 6, v);
                check_equal(v, 10, "the periods counter not counting the ramp down");

            elsif run("closed_loop_stop_ramps_down") then
                -- a stop in closed loop: the reference at 0 V for 5 ms first; a second stop or a
                -- trip at once
                bus_write(base + 4, to_signed(40000, 32));
                bus_write(base + 7, to_signed(2, 32));
                bus_write(base + 10, to_fixed(15.0));
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait until closed_loop_out for 30 us;
                wait for 100 us;
                started := now;
                bus_write(base, to_signed(2, 32));
                wait until period_start_out;
                wait for 20 ns;
                check(voltage_reference = fixed_zero, "the reference 0 V on stop");
                check(closed_loop_out, "still running: the output brought down");
                wait until not closed_loop_out for 6 ms;
                check(now - started > 199 * 25 us and now - started < 202 * 25 us, "off 5 ms after the stop");
                -- a trip during the ramp down: off at once
                bus_write(base, to_signed(1, 32));
                wait until closed_loop_out for 30 us;
                bus_write(base, to_signed(2, 32));
                wait for 100 us;
                check(closed_loop_out, "ramping down");
                trip <= true;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                check(not closed_loop_out, "a trip stops it at once");
                trip <= false;

            elsif run("samples_while_stopped") then
                -- not armed, no DHB: the timer's samples, half a period apart
                wait until sample for 100 us;
                check(sample, "a sample while stopped");
                started := now;
                wait until rising_edge(clock);
                wait until sample for 100 us;
                check(abs(integer((now - started) / 8 ns) - integer(period / 2)) <= 1, "half a period on: " & integer'image((now - started) / 8 ns));
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "the gates off");

            elsif run("off_when_the_dhb_stops_or_trips") then
                allowed <= true;
                bus_write(base, to_signed(1, 32));
                wait for 300 us;
                allowed <= false;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                wait for 1 ns;
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "off a clock after the DHB stops");
                allowed <= true;
                wait for 100 us;
                check(leg1 = dhb_leg_gates_off, "and not again without a new start: disarmed");
                bus_write(base, to_signed(1, 32));
                wait for 300 us;
                trip <= true;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                wait for 1 ns;
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "off a clock after a trip");
                check_equal(both_on, 0, "clocks with both switches of a leg on");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
