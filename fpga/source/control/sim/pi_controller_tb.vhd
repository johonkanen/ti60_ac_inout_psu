LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fixed_format_pkg.all;
    use work.pi_controller_pkg.all;

-- pi_controller against a PI computed in reals step by step, saturation and the anti-windup,
-- reset, a request while busy, and a closed loop: the DHB's secondary DC link as a simple
-- model, the controller's output the phase shift, the primary's voltage the reference.
entity pi_controller_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of pi_controller_tb is

    signal clock  : std_logic := '0';
    signal pi_in  : pi_controller_in_record := pi_controller_in_init;
    signal pi_out : pi_controller_out_record;
    signal readies : natural := 0;

begin

    clock <= not clock after 4 ns;

    dut : entity work.pi_controller
    port map(clock, pi_in, pi_out);

    count : process(clock)
    begin
        if rising_edge(clock) then
            if pi_out.ready then
                readies <= readies + 1;
            end if;
        end if;
    end process;

    main : process

        -- one request, its output
        procedure step(reference, measurement : real; output : out real; clocks : out natural) is
            variable start : time;
        begin
            wait until rising_edge(clock);
            pi_in.reference   <= to_fixed(reference);
            pi_in.measurement <= to_fixed(measurement);
            pi_in.request     <= true;
            start := now;
            wait until rising_edge(clock);
            pi_in.request <= false;
            wait until pi_out.ready for 1 us;
            check(pi_out.ready, "an output");
            output := to_real(pi_out.output);
            clocks := (now - start) / 8 ns;
        end procedure;

        procedure reset is
        begin
            wait until rising_edge(clock);
            pi_in.reset <= true;
            wait until rising_edge(clock);
            pi_in.reset <= false;
        end procedure;

        -- the same PI in reals
        variable integ, model : real := 0.0;
        procedure model_step(e, kp, ki, high, low : real) is
        begin
            integ := realmax(low, realmin(high, integ + ki * e));
            model := realmax(low, realmin(high, kp * e + integ));
        end procedure;

        variable output, worst, v, phase : real;
        variable clocks : natural;
        variable before : natural;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            pi_in.kp    <= to_fixed(2.0);
            pi_in.ki    <= to_fixed(0.05);
            pi_in.limit_high <= to_fixed(300.0);
            pi_in.limit_low  <= to_fixed(-300.0);
            integ := 0.0;
            worst := 0.0;

            if run("matches_a_pi_in_reals") then
                for k in 0 to 199 loop
                    -- errors of both signs, small and large
                    v := 95.0 - 30.0 * sin(real(k) * 0.07) - 0.123 * real(k mod 7);
                    step(95.0, v, output, clocks);
                    model_step(95.0 - v, 2.0, 0.05, 300.0, -300.0);
                    worst := realmax(worst, abs(output - model));
                end loop;
                info("200 steps: worst difference " & real'image(worst) & ", " & integer'image(clocks) & " clocks a step");
                check(worst < 1.0e-3, "within 1e-3 of the PI in reals");

            elsif run("saturates_without_windup") then
                -- a large error holds the output at the limit, the integrator too and no further
                for k in 0 to 99 loop
                    step(95.0, 0.0, output, clocks);
                    model_step(95.0, 2.0, 0.05, 300.0, -300.0);
                end loop;
                check_equal(output, 300.0, "at the limit");
                check(pi_out.saturated, "saturated");
                -- the error reversed: the output leaves the limit at once
                step(95.0, 100.0, output, clocks);
                model_step(-5.0, 2.0, 0.05, 300.0, -300.0);
                info("after 100 steps at the limit, a -5 error: " & real'image(output));
                check(output < 300.0 - 9.9, "off the limit at the first step, no wound up integrator");
                check(not pi_out.saturated, "not saturated");
                check(abs(output - model) < 1.0e-3, "as the PI in reals");
                -- and the negative limit
                for k in 0 to 99 loop
                    step(0.0, 95.0, output, clocks);
                end loop;
                check_equal(output, -300.0, "at the negative limit");

            elsif run("asymmetric_limits") then
                -- 0 .. 50: no negative output, the integrator held at 0 meanwhile too
                pi_in.limit_high <= to_fixed(50.0);
                pi_in.limit_low  <= to_fixed(0.0);
                for k in 0 to 49 loop
                    step(95.0, 100.0, output, clocks);
                    model_step(-5.0, 2.0, 0.05, 50.0, 0.0);
                end loop;
                check_equal(output, 0.0, "held at the lower limit 0");
                check(pi_out.saturated, "saturated");
                for k in 0 to 49 loop
                    step(95.0, 0.0, output, clocks);
                    model_step(95.0, 2.0, 0.05, 50.0, 0.0);
                    worst := realmax(worst, abs(output - model));
                end loop;
                check_equal(output, 50.0, "at the upper limit 50");
                check(worst < 1.0e-3, "as the PI in reals");

            elsif run("reset_clears") then
                for k in 0 to 9 loop
                    step(95.0, 50.0, output, clocks);
                end loop;
                check(output > 100.0, "an output");
                reset;
                wait for 1 ns;
                check_equal(to_real(pi_out.output), 0.0, "output cleared");
                step(95.0, 95.0, output, clocks);
                check_equal(output, 0.0, "and the integrator: no error, no output");

            elsif run("a_request_while_busy") then
                before := readies;
                wait until rising_edge(clock);
                pi_in.reference   <= to_fixed(95.0);
                pi_in.measurement <= to_fixed(90.0);
                pi_in.request     <= true;
                wait until rising_edge(clock);
                wait until rising_edge(clock);   -- a second one while the first is computed
                pi_in.request <= false;
                wait for 1 us;
                check_equal(readies - before, 2, "both answered, one after the other");

            elsif run("closes_a_dc_link_loop") then
                -- the secondary DC link: 400 uF fed by the DHB, ~ 0.02 A of output per count of
                -- phase at these voltages, a 1 kOhm load; the controller run every 10 us. The
                -- plant integrates 5e-4 V a count a step: kp 40 gives a loop time constant of
                -- 50 steps, ki 0.4 with it a damping of ~0.7
                pi_in.kp <= to_fixed(40.0);
                pi_in.ki <= to_fixed(0.4);
                v := 60.0;
                for k in 0 to 1999 loop
                    step(95.0, v, phase, clocks);
                    v := v + 10.0e-6 / 400.0e-6 * (0.02 * phase - v / 1000.0);
                end loop;
                info("20 ms: the secondary at " & real'image(v) & " V, the phase " & real'image(phase));
                check(abs(v - 95.0) < 0.05, "at the reference");
                check(phase > 0.0 and phase < 300.0, "a positive phase feeding the load, not at the limit");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 100 ms);

end vunit_simulation;
