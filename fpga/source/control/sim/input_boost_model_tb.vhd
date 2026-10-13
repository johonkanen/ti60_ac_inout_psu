LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.input_boost_model_pkg.all;
    use work.input_boost_plant_pkg.all;

-- The input boost's voltage control in real numbers (input_boost_model_pkg, the boost programs'
-- steps) on its power stage (input_boost_plant_pkg, hVHDL_ode's rk4), a switching period at a
-- time with no clock: seconds of the loop in a second. input_boost_control_tb checks the RTL
-- against the same steps.
--
-- The periods as the RTL has them: the link read three times a period (the value at the
-- period's start, the noise on each reading), the readings' mean handed to the control at the
-- next period's start (a run's first: the reset), its duty switching from the period after.
--
--   holds_the_link     94.6 V to 105 V on the converter, the duty within its 0.05; a 0.5 A load
--   a_load_pulsing_at_100_hz   an AC output's power, 0 .. 0.6 A at 100 Hz
--   noisy_readings     +-1 V on each reading (the board's 0.1 V rms)
--   a_reference_step_without_the_prefilter_overshoots / ..._with_the_prefilter
--                      105 V to 112 V at a third of the default gains (the board's loop gain a
--                      third of the converter model's)
--   the_board_at_5_hz  the defaults on the plant fitted to the board, 64 V to 67 V: the step as
--                      on the board (0.3 s, under 0.25 V over), the link's capacitance doubled too
entity input_boost_model_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of input_boost_model_tb is
begin

    main : process
        variable settings : boost_law_settings;
        variable law      : boost_law_state;
        variable plant    : boost_plant;
        variable fitted   : fitted_plant;
        variable use_fitted : boolean;
        variable started  : boolean;        -- the run's first step is its reset
        variable reading_mean, pending_mean : real;
        variable next_duty, duty : real;    -- the step's duty, then the period's
        variable i_load   : real;
        variable pulsing  : boolean;
        variable noise    : real;           -- up to +- this on each reading
        variable seed1, seed2 : positive := 7;
        variable periods_run : natural;
        variable v_low, v_high, i_high, duty_low, duty_high : real;
        variable widest   : natural;
        variable settled_at : real;         -- s, the last time the link was 0.2 V off the reference

        impure function link return real is
        begin
            if use_fitted then
                return fitted.v;
            end if;
            return plant.v;
        end function;

        procedure reset_extremes is
        begin
            v_low := 1.0e9; v_high := -1.0e9; i_high := 0.0;
            duty_low := 1.0e9; duty_high := -1.0e9; widest := 0;
        end procedure;

        procedure start(s : boost_law_settings) is
        begin
            settings := quantized(s);
            law := boost_law_zero;
            started := false;
            next_duty := 0.0; duty := 0.0;
            pending_mean := link;
            periods_run := 0;
        end procedure;

        -- n switching periods: the readings of each period's start, the control's step at the
        -- next one, its duty from the period after
        procedure periods(n : natural; settle_ref : real := 0.0) is
            variable x, sum : real;
            variable load : real;
            variable t : real;
        begin
            for p in 1 to n loop
                t := real(periods_run) * boost_t_period;
                -- the step with the period before's readings
                if not started then
                    boost_law_reset(law, true, pending_mean);
                    started := true;
                else
                    boost_law_step(law, true, pending_mean, settings);
                end if;
                -- this period switches with the step before's duty
                duty := next_duty;
                next_duty := law.duty;
                -- this period's readings: the link at its start
                sum := 0.0;
                for r in 1 to 3 loop
                    uniform(seed1, seed2, x);
                    sum := sum + link + 2.0 * noise * (x - 0.5);
                end loop;
                pending_mean := sum / 3.0;
                -- the period
                if pulsing then
                    load := 0.3 * (1.0 - cos(math_2_pi * 100.0 * t));
                else
                    load := i_load;
                end if;
                if use_fitted then
                    fitted_plant_period(fitted, duty);
                else
                    boost_plant_period(plant, real(boost_on_clocks(duty)) * 8.0e-9, load);
                    i_high := realmax(i_high, plant.i_mean);
                end if;
                widest := maximum(widest, boost_on_clocks(duty));
                v_low  := realmin(v_low, link);
                v_high := realmax(v_high, link);
                duty_low  := realmin(duty_low, law.duty);
                duty_high := realmax(duty_high, law.duty);
                if abs(link - settle_ref) > 0.2 then
                    settled_at := t;
                end if;
                periods_run := periods_run + 1;
            end loop;
        end procedure;

        -- a time's periods
        procedure run_for(seconds : real; settle_ref : real := 0.0) is
        begin
            periods(natural(seconds / boost_t_period), settle_ref);
        end procedure;

        variable s : boost_law_settings;
        variable step_at : real;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            plant := boost_plant_init;
            fitted := fitted_plant_init;
            use_fitted := false;
            i_load := 0.05;         -- a bleed
            pulsing := false;
            noise := 0.0;

            if run("holds_the_link") then
                start(boost_law_defaults);
                reset_extremes;
                run_for(0.8);
                info("after 800 ms: the link " & real'image(link) & " V, its highest " & real'image(v_high) &
                     " V; the input current's highest " & real'image(i_high) & " A; the widest pulse " &
                     integer'image(widest) & " clocks");
                check(abs(link - 105.0) < 0.3, "the link at 105 V");
                check(v_high < 106.0, "overshoot under 1 V");
                check(widest <= 157, "the duty within 0.05 (156 clocks)");
                check(abs(law.target - 105.0) < 0.01, "the reference ramped to 105 V");
                -- a load step: 0.5 A
                i_load := 0.5;
                reset_extremes;
                run_for(0.3);
                info("the 0.5 A load: the link down to " & real'image(v_low) & " V, after 300 ms " &
                     real'image(link) & " V");
                check(v_low > 100.0, "the dip within 5 V");
                reset_extremes;
                run_for(0.3);
                check(abs(link - 105.0) < 0.3, "back at 105 V with the load");
                check(v_low > 104.5, "and steady");
                check(plant.i_mean > 0.5, "the input current carrying the load");
                info("the duty with the 0.5 A load " & real'image(law.duty));
                check(law.duty > 0.01 and law.duty < 0.05, "the duty in its range");

            elsif run("a_load_pulsing_at_100_hz") then
                start(boost_law_defaults);
                run_for(0.3);
                pulsing := true;
                run_for(0.5);
                reset_extremes;
                run_for(0.1);
                info("the 100 Hz load: the link " & real'image(v_low) & " .. " & real'image(v_high) &
                     " V, the duty " & real'image(duty_low) & " .. " & real'image(duty_high));
                check(abs((v_low + v_high) / 2.0 - 105.0) < 0.5, "the link about 105 V");
                check(v_high - v_low < 2.0, "its ripple under 2 V");
                check(duty_high < 0.05, "the duty under its limit");

            elsif run("noisy_readings") then
                -- +-1 V on each reading (0.58 V rms; the board's 0.1 V rms, 2026-10-13): the period's
                -- mean and the loop average it out. (+-3 V swings the duty by kp's 0.022 a volt of
                -- the period's mean about the model's 0.03 here: clipped at 0 a period in ten, the
                -- integral held there, the link 0.4 V high.)
                noise := 1.0;
                i_load := 0.3;
                start(boost_law_defaults);
                run_for(0.8);
                reset_extremes;
                run_for(0.2);
                info("noisy readings: the link " & real'image(v_low) & " .. " & real'image(v_high) & " V");
                check(v_low > 104.5 and v_high < 105.5, "the link within 0.5 V of 105 V");

            elsif run("a_reference_step_without_the_prefilter_overshoots")
                  or run("a_reference_step_with_the_prefilter") then
                -- the board's loop: its gain a third of the model's, so kp and ki a third of the
                -- defaults here (the prefilter's alpha, ki / kp, the default's); 105 V, then 112 V
                s := boost_law_defaults;
                s.kp := s.kp / 3.0;
                s.ki := s.ki / 3.0;
                if running_test_case = "a_reference_step_without_the_prefilter_overshoots" then
                    s.alpha := 1.0;
                end if;
                start(s);
                reset_extremes;
                run_for(0.8);
                info("the start: the link's highest " & real'image(v_high) & " V, now " & real'image(link) & " V");
                settings.v_ref := 112.0;
                reset_extremes;
                run_for(1.5);
                info("105 V -> 112 V: the link's highest " & real'image(v_high) & " V, after 1.5 s " &
                     real'image(link) & " V");
                if running_test_case = "a_reference_step_without_the_prefilter_overshoots" then
                    -- at a third of the defaults 0.47 V over (the prefilter's 0.02 V)
                    check(v_high > 112.3, "without the prefilter the step overshoots (the test sees it)");
                else
                    check(v_high < 112.2, "with the prefilter no overshoot (within 0.2 V)");
                    check(abs(link - 112.0) < 0.2, "at 112 V");
                end if;

            elsif run("the_board_at_5_hz") then
                -- the defaults on the plant fitted to the board, 64 V to 67 V as there; on the
                -- board: settled to 0.2 V in 0.31 s, 0.20 V over
                for c_scale in 1 to 2 loop
                    use_fitted := true;
                    fitted := fitted_plant_init;
                    fitted.c_scale := real(c_scale);
                    s := boost_law_defaults;
                    s.v_ref := 64.0;
                    s.d_max := 0.2;
                    start(s);
                    run_for(1.0);
                    check(abs(link - 64.0) < 0.1, "at 64 V");
                    settings.v_ref := 67.0;
                    reset_extremes;
                    step_at := real(periods_run) * boost_t_period;
                    settled_at := step_at;
                    run_for(1.0, 67.0);
                    info("the link's capacitance x" & integer'image(c_scale) & ": 64 V -> 67 V settled to 0.2 V in " &
                         real'image(settled_at - step_at) & " s, the highest " & real'image(v_high) & " V");
                    check(settled_at - step_at < 0.45, "settled in under 0.45 s");
                    check(v_high < 67.25, "under 0.25 V over");
                end loop;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
