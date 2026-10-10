LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.dhb_modulator_pkg.all;

-- The DHB modulator: the package's segments against the carrier's states, and the modulator,
-- carrier and dead time chained at 200 MHz with a 150 kHz period (1333 counts): periods, pulse
-- widths, phase shift, dead times, no leg ever with both switches on, a phase change only at a
-- period start, the phase limit, and disable; a long dead time's short pulses, a leg left off,
-- and the precharge against a simple model of the secondary DC link.
entity dhb_modulator_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of dhb_modulator_tb is

    constant clock_period : time := 5 ns;
    constant period       : natural := 1333;
    constant half         : natural := period / 2;

    signal clock : std_logic := '0';

    signal enable               : boolean := false;
    signal phase                : dhb_phase := 200;
    signal max_phase            : dhb_count := 600;
    signal deadtime             : dhb_count_array := (12, 20);
    signal modulation_requested : boolean;
    signal modulation_ready     : boolean;
    signal switch_times         : dhb_switch_times;
    signal period_start         : boolean;
    signal bridge_state         : dhb_bridge_state;
    signal counter              : dhb_count;
    signal gates                : dhb_gates;

    -- shoot through: clocks with both switches of a leg on
    signal both_on : natural := 0;

    -- the dead time module's inputs: the test's, or the precharge's
    signal leg_enable     : std_logic_vector(0 to 1) := "11";
    signal deadtimes      : dhb_count_array;
    signal leg_enables    : std_logic_vector(0 to 1);
    signal use_precharge  : boolean := false;

    -- the precharge, and a model of the secondary DC link: a primary switch on charges it
    -- towards the primary's 400 V by coupling of the difference a clock (0 for none)
    signal precharge_start, precharge_stop : boolean := false;
    signal precharge_settings : dhb_precharge_settings := (
        deadtime_start => half - 10, deadtime_final => 12, ramp_step => 5, ramp_periods => 1,
        margin => 50, timeout_periods => 20);
    signal precharge_deadtime   : dhb_count;
    signal precharge_leg_enable : std_logic_vector(0 to 1);
    signal precharge_state      : dhb_precharge_state;
    constant primary_volts      : real := 400.0;
    signal secondary_volts      : real := 0.0;
    signal coupling             : real := 2.0e-4;
    signal secondary_gate_clocks : natural := 0;   -- clocks with a secondary gate on

    function clocks(t : time) return integer is
    begin
        return t / clock_period;
    end function;

begin

    clock <= not clock after clock_period / 2;

    u_modulator : entity work.dhb_modulator
    port map(clock, modulation_requested, period, phase, max_phase, modulation_ready, switch_times);

    u_carrier : entity work.dhb_carrier
    port map(clock, enable, switch_times, modulation_requested, period_start, bridge_state, counter);

    u_deadtime : entity work.dhb_deadtime
    port map(clock, enable, leg_enables, bridge_state, deadtimes, gates);

    deadtimes   <= (precharge_deadtime, deadtime(secondary)) when use_precharge else deadtime;
    leg_enables <= precharge_leg_enable when use_precharge else leg_enable;

    -- voltages in 0.1 V
    u_precharge : entity work.dhb_precharge
    port map(clock, precharge_start, precharge_stop, period_start,
             integer(primary_volts * 10.0), integer(secondary_volts * 10.0), precharge_settings,
             precharge_deadtime, precharge_leg_enable, precharge_state);

    secondary_dc_link : process(clock)
    begin
        if rising_edge(clock) then
            if (gates(primary).high or gates(primary).low) = '1' then
                secondary_volts <= secondary_volts + coupling * (primary_volts - secondary_volts);
            end if;
            if (gates(secondary).high or gates(secondary).low) = '1' then
                secondary_gate_clocks <= secondary_gate_clocks + 1;
            end if;
        end if;
    end process;

    shoot_through : process(clock)
    begin
        if rising_edge(clock) then
            if (gates(primary).high and gates(primary).low) = '1' or
               (gates(secondary).high and gates(secondary).low) = '1' then
                both_on <= both_on + 1;
            end if;
        end if;
    end process;

    main : process

        -- the next rising edge of a gate, and its time
        procedure next_rise(signal gate : std_logic; at : out time) is
        begin
            wait until rising_edge(gate) for 50 us;
            check(gate = '1', "a gate pulse");
            at := now;
        end procedure;

        -- a gate's next whole pulse: its width and the time to the next rise, in clocks
        procedure measure(signal gate : std_logic; width, to_next : out integer) is
            variable rise, fall, rise2 : time;
        begin
            next_rise(gate, rise);
            wait until falling_edge(gate);
            fall := now;
            next_rise(gate, rise2);
            width   := clocks(fall - rise);
            to_next := clocks(rise2 - rise);
        end procedure;

        -- clocks from a falling edge of one gate to the next rising edge of another
        procedure gap(signal falling, rising : std_logic; clks : out integer) is
            variable fall : time;
        begin
            wait until falling_edge(falling);
            fall := now;
            wait until rising_edge(rising);
            clks := clocks(now - fall);
        end procedure;

        -- clocks from a rise of the primary high side to the next rise of the secondary's
        procedure shift(clks : out integer) is
            variable p, s : time;
        begin
            next_rise(gates(primary).high, p);
            next_rise(gates(secondary).high, s);
            clks := clocks(s - p);
        end procedure;

        variable width, to_next, clks : integer;
        variable segments : dhb_segment_array;
        variable times    : dhb_switch_times;
        variable c, total : natural;
        variable shortest : integer;
        variable per      : natural;
        variable ph       : integer;
        constant periods  : integer_vector := (1333, 1000, 101, 8, 5);

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("segments_match_the_carrier") then
                -- for an ODE simulation stepping by sub-interval: the same states as the
                -- carrier gives count by count, and the lengths summing to the period
                for p in periods'range loop
                    for k in -12 to 12 loop
                        per := periods(p);
                        ph  := k * per / 20;   -- past +-period/2 too
                            segments := get_segments(per, ph);
                            times    := get_switch_times(per, ph);
                            total    := 0;
                            c        := 0;
                            for s in segments'range loop
                                total := total + segments(s).length;
                                for n in 1 to segments(s).length loop
                                    check(get_bridge_state(c, times) = segments(s).state,
                                          "period " & integer'image(per) & ", phase " & integer'image(ph)
                                          & ", count " & integer'image(c));
                                    c := c + 1;
                                end loop;
                            end loop;
                            check_equal(total, per, "segments sum to the period");
                    end loop;
                end loop;
                -- the direction of the power: the primary leads for a positive phase
                check(get_segments(1333, 100)(0).state = "10", "positive phase: primary first");
                check(get_segments(1333, -100)(1).state = "10" and get_segments(1333, -100)(3).state = "01",
                      "negative phase: secondary first");

            elsif run("gates_of_a_running_bridge") then
                enable <= true;
                wait for 5 * period * clock_period;
                measure(gates(primary).high, width, to_next);
                check_equal(to_next, period, "primary period");
                check_equal(width, half - 12, "primary high side: half a period less the dead time");
                measure(gates(primary).low, width, to_next);
                check_equal(width, period - half - 12, "primary low side");
                measure(gates(secondary).high, width, to_next);
                check_equal(to_next, period, "secondary period");
                check_equal(width, half - 20, "secondary high side");
                shift(clks);
                check_equal(clks, 200 + 20 - 12, "phase shift, each rise late by its leg's dead time");
                gap(gates(primary).low, gates(primary).high, clks);
                check_equal(clks, 12, "primary dead time");
                gap(gates(primary).high, gates(primary).low, clks);
                check_equal(clks, 12, "primary dead time");
                gap(gates(secondary).low, gates(secondary).high, clks);
                check_equal(clks, 20, "secondary dead time");
                gap(gates(secondary).high, gates(secondary).low, clks);
                check_equal(clks, 20, "secondary dead time");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("phase_change_at_a_period_start") then
                enable <= true;
                wait for 3 * period * clock_period;
                -- reversed in the middle of a period, then back
                wait until counter = 500;
                phase <= -200;
                shortest := integer'high;
                for n in 1 to 4 loop
                    measure(gates(secondary).high, width, to_next);
                    shortest := minimum(shortest, width);
                end loop;
                shift(clks);
                check_equal(clks, period - 200 + 20 - 12, "reversed: the secondary leads by 200");
                wait until counter = 900;
                phase <= 300;
                for n in 1 to 4 loop
                    measure(gates(secondary).high, width, to_next);
                    shortest := minimum(shortest, width);
                    measure(gates(secondary).low, width, to_next);
                    shortest := minimum(shortest, width);
                end loop;
                shift(clks);
                check_equal(clks, 300 + 20 - 12, "forward again");
                info("shortest secondary pulse across the changes: " & integer'image(shortest) & " clocks");
                -- a phase step of d counts moves one secondary edge by d: the pulse that spans
                -- the period start (a negative phase) loses at most the step, 500 here
                check(shortest >= half - 500 - 20, "no pulse shorter than the phase step leaves");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("phase_limited") then
                phase <= 2000;
                enable <= true;
                wait for 3 * period * clock_period;
                shift(clks);
                check_equal(clks, 600 + 20 - 12, "phase held at max_phase");
                phase <= -2000;
                wait for 3 * period * clock_period;
                shift(clks);
                check_equal(clks, period - 600 + 20 - 12, "and at -max_phase");

            elsif run("long_deadtime_gives_short_pulses") then
                -- the precharge's start: a dead time of all but 10 clocks of a half period,
                -- the secondary leg off
                deadtime   <= (half - 10, 20);
                leg_enable <= "10";
                enable     <= true;
                wait for 3 * period * clock_period;
                measure(gates(primary).high, width, to_next);
                check_equal(width, 10, "a 10 clock high side pulse");
                check_equal(to_next, period, "a period apart");
                measure(gates(primary).low, width, to_next);
                check_equal(width, period - half - (half - 10), "and a low side pulse");
                check_equal(secondary_gate_clocks, 0, "the secondary's gates stay off");
                -- half a period of dead time: no pulses at all
                deadtime <= (half + 1, 20);
                wait for 2 * period * clock_period;
                wait until gates(primary).high = '1' or gates(primary).low = '1' for 3 * period * clock_period;
                check(gates(primary).high = '0' and gates(primary).low = '0', "no pulses");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("precharge_ramps_to_the_primary_voltage") then
                use_precharge <= true;
                enable        <= true;
                wait for 2 * period * clock_period;
                check(gates = dhb_gates_off, "idle: the bridge off");
                precharge_start <= true;
                wait until rising_edge(clock);
                precharge_start <= false;
                wait until rising_edge(clock);
                check(precharge_state = charging, "charging");
                -- the first pulses: 10 clocks
                measure(gates(primary).high, width, to_next);
                check(width <= 10 + 5 * 3, "short first pulses");
                wait until precharge_state /= charging for 20 ms;
                check(precharge_state = charged, "charged");
                info("charged to " & real'image(secondary_volts) & " V, the dead time down to " &
                     integer'image(precharge_deadtime) & " clocks");
                check(secondary_volts >= primary_volts - 5.0, "within the 5 V margin");
                check(precharge_deadtime > 12 and precharge_deadtime < half - 10, "stopped on the ramp");
                check_equal(secondary_gate_clocks, 0, "the secondary's gates stayed off");
                -- charged: the primary goes on at the dead time it got to
                measure(gates(primary).high, width, to_next);
                check_equal(width, half - precharge_deadtime, "pulses held");
                precharge_stop <= true;
                wait until rising_edge(clock);
                precharge_stop <= false;
                wait for 2 * period * clock_period;
                check(precharge_state = idle and gates = dhb_gates_off, "stopped: idle, the bridge off");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("precharge_fails_without_the_voltage") then
                coupling      <= 0.0;    -- nothing charges
                use_precharge <= true;
                enable        <= true;
                wait for 2 * period * clock_period;
                precharge_start <= true;
                wait until rising_edge(clock);
                precharge_start <= false;
                wait until precharge_state /= charging for 20 ms;
                check(precharge_state = failed, "failed");
                info("failed at a dead time of " & integer'image(precharge_deadtime) & " clocks");
                check_equal(precharge_deadtime, 12, "after ramping to the final dead time");
                wait for 2 * period * clock_period;
                check(gates = dhb_gates_off, "failed: the bridge off");

            elsif run("disable_turns_everything_off") then
                enable <= true;
                wait for 3 * period * clock_period + 123 * clock_period;
                enable <= false;
                wait until rising_edge(clock);
                wait until rising_edge(clock);
                wait for 1 ns;
                check(gates = dhb_gates_off, "all off the clock after disable");
                for n in 1 to 3 * period loop
                    wait until rising_edge(clock);
                    check(gates = dhb_gates_off, "and staying off");
                end loop;
                -- enabled again: a period from the start, the first switches after their dead time
                enable <= true;
                wait until period_start;
                wait until gates(primary).high = '1' and gates(secondary).low = '1' for 1 us;
                check(gates(primary).high = '1', "primary high side on after the restart");
                measure(gates(primary).high, width, to_next);
                check_equal(to_next, period, "running again");
                check_equal(both_on, 0, "clocks with both switches of a leg on");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 ms);

end vunit_simulation;
