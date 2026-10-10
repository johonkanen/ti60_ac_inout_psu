LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.full_bridge_modulator_pkg.all;

-- full_bridge (its modulator, carrier and dead time joined) at 125 MHz, 22 kHz (5682 counts),
-- 13 counts (104 ns) of dead time; and the modulator alone against get_switch_times:
-- leg 1's high side duty * period less the dead time, leg 2 its complement, the dead times, a
-- duty change only at a period start, the duty's limits, disable, never both switches of a leg.
entity full_bridge_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of full_bridge_tb is

    constant clock_period : time := 8 ns;
    constant period       : natural := 5682;
    constant deadtime     : natural := 13;

    signal clock        : std_logic := '0';
    signal enable       : boolean := false;
    signal duty         : fixed_number := to_fixed(0.5);
    signal leg1, leg2   : dhb_leg_gates;
    signal period_start : boolean;
    signal sample       : boolean;
    signal modulation_requested, modulation_ready : boolean := false;
    signal modulator_duty : fixed_number := fixed_zero;
    signal switch_times : fb_switch_times;
    signal both_on      : natural := 0;

    function clocks(t : time) return integer is
    begin
        return t / clock_period;
    end function;

begin

    clock <= not clock after clock_period / 2;

    dut : entity work.full_bridge
    port map(clock, enable, period, duty, deadtime, leg1, leg2, period_start, sample);

    -- the modulator alone, for its function
    u_modulator : entity work.full_bridge_modulator
    port map(clock, modulation_requested, period, modulator_duty, modulation_ready, switch_times);

    shoot_through : process(clock)
    begin
        if rising_edge(clock) then
            if (leg1.high and leg1.low) = '1' or (leg2.high and leg2.low) = '1' then
                both_on <= both_on + 1;
            end if;
        end if;
    end process;

    main : process
        procedure measure(signal gate : std_logic; width, to_next : out integer) is
            variable rise, fall : time;
        begin
            wait until rising_edge(gate) for 1 ms;
            check(gate = '1', "a gate pulse");
            rise := now;
            wait until falling_edge(gate);
            fall := now;
            wait until rising_edge(gate);
            width   := clocks(fall - rise);
            to_next := clocks(now - rise);
        end procedure;

        procedure gap(signal falling, rising : std_logic; clks : out integer) is
            variable fall : time;
        begin
            wait until falling_edge(falling);
            fall := now;
            wait until rising_edge(rising);
            clks := clocks(now - fall);
        end procedure;

        variable width, to_next, clks : integer;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("half_duty") then
                enable <= true;
                wait for 3 * period * clock_period;
                measure(leg1.high, width, to_next);
                check_equal(to_next, period, "22 kHz");
                check_equal(width, period / 2 - deadtime, "leg 1 high: half a period less the dead time");
                measure(leg2.high, width, to_next);
                check_equal(width, period - period / 2 - deadtime, "leg 2 high: the other half");
                measure(leg1.low, width, to_next);
                check_equal(width, period - period / 2 - deadtime, "leg 1 low with leg 2 high");
                gap(leg1.high, leg1.low, clks);
                check_equal(clks, deadtime, "leg 1 dead time");
                gap(leg2.low, leg2.high, clks);
                check_equal(clks, deadtime, "leg 2 dead time");
                -- complementary: leg 2's high side follows leg 1's low side
                wait until rising_edge(leg1.low);
                check(leg2.high = '1', "leg 2 high with leg 1 low");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("a_quarter_and_a_step") then
                duty   <= to_fixed(0.25);
                enable <= true;
                wait for 3 * period * clock_period;
                measure(leg1.high, width, to_next);
                check_equal(width, integer(real(period) * 0.25 - 0.5) - deadtime, "a quarter");
                -- a new duty in the middle of a period: that period's pulse unchanged
                wait until rising_edge(leg1.high);
                wait for 100 * clock_period;
                duty <= to_fixed(0.75);
                wait until falling_edge(leg1.high);
                measure(leg1.high, width, to_next);
                check_equal(width, integer(real(period) * 0.75 - 0.5) - deadtime, "the next period at three quarters");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("duty_limits") then
                duty   <= to_fixed(1.5);
                enable <= true;
                wait for 3 * period * clock_period;
                for k in 1 to period loop
                    wait until rising_edge(clock);
                    check(leg1.high = '1' and leg2.low = '1', "duty 1.5 held at 1: leg 1 high, leg 2 low all the time");
                end loop;
                duty <= to_fixed(-0.5);
                wait for 3 * period * clock_period;
                for k in 1 to period loop
                    wait until rising_edge(clock);
                    check(leg1.low = '1' and leg2.high = '1', "duty -0.5 held at 0");
                end loop;

            elsif run("samples_twice_a_period") then
                enable <= true;
                wait until period_start for 1 ms;
                wait until rising_edge(clock);
                for k in 1 to 6 loop
                    clks := 0;
                    loop
                        wait until rising_edge(clock);
                        clks := clks + 1;
                        exit when sample;
                    end loop;
                    if k mod 2 = 1 then
                        check(not period_start, "the middle of the period");
                        check_equal(clks, period / 2, "the middle: half a period from the start");
                    else
                        check(period_start, "a sample at the period start");
                        check_equal(clks, period - period / 2, "the start: the rest of the period");
                    end if;
                end loop;
                enable <= false;
                for k in 1 to 2 * period loop
                    wait until rising_edge(clock);
                    check(not sample, "no samples while disabled");
                end loop;

            elsif run("the_modulator_is_its_function") then
                -- random duties (some outside 0 .. 1): the modulator's times are get_switch_times'
                for k in 1 to 200 loop
                    wait until rising_edge(clock);
                    modulator_duty <= to_fixed(real((k * 7919) mod 1300) / 1000.0 - 0.15);
                    modulation_requested <= true;
                    wait until rising_edge(clock);
                    modulation_requested <= false;
                    wait until modulation_ready for 100 ns;
                    check(modulation_ready, "the times within 100 ns");
                    wait for 1 ns;
                    check(switch_times = get_switch_times(period, modulator_duty),
                          "duty " & real'image(to_real(modulator_duty)) & ": leg1_off " & integer'image(switch_times.leg1_off) &
                          ", the function's " & integer'image(get_switch_times(period, modulator_duty).leg1_off));
                end loop;

            elsif run("disable_turns_everything_off") then
                enable <= true;
                wait for 3 * period * clock_period + 1234 * clock_period;
                enable <= false;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                wait for 1 ns;
                check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "all off a clock after disable");
                for k in 1 to 2 * period loop
                    wait until rising_edge(clock);
                    check(leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "and staying off");
                end loop;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
