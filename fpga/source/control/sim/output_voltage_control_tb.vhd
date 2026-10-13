LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;
    use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fixed_format_pkg.all;
    use work.output_bridge_pkg.all;
    use work.output_observer_pkg.all;
    use work.output_voltage_control_pkg.all;
    use work.ode_pkg.all;

-- The output voltage's cascaded control (output_voltage_control_pkg) on a model of the board,
-- to tune it: a +-20 V square wave reference of 10 ms.
--
-- The plant, hVHDL_ode's rk4: the bridge's average voltage over each switching period (1.03 of
-- its reference less 4.2 V, as measured, within the 94 V DC link) into the two stage filter,
-- L1 0.83 mH (0.51 ohm), C1 11 uF, the common mode filter's L2 5 uH (0.1 ohm), C2 7 uF.
-- The measurements as the FPGA has them: the inductor current's mean over the last period
-- twice a period, C1's voltage read two or three times a half period with 15 V rms of noise.
-- The output observer as the FPGA computes it (its fixed point step: the fitted lumped filter,
-- g_observer_current_hz for the current: 1 kHz, robust to the inductor 30 % off the model's;
-- g_observer_voltage_hz for the voltage: 100 Hz (30 Hz drifted on the board and misses load steps;
-- slower averages the noisy readings longer), the voltage
-- between them the model's from the measured current). board_like (the default): the bridge
-- switched (+-vdc by the duty on the sawtooth carrier) and the measured current through the
-- sigma-delta's filter, as the board; else its average voltage and the plant's current. The control at each period start from the observer's estimates, its
-- bridge reference applied from the next period.
--
--   tune_the_loops_at_1500_hz, .._2000_hz, .._3000_hz   the step's figures for a range of the
--                    voltage loop's bandwidths and setpoint weights, a test a current loop's
--   the_observers_voltage_bandwidth   the chosen ones with the observer's voltage loop at
--                    100, 30 and 10 Hz
--   the_observers_current_bandwidth   its current loop at 3, 1.5 and 1 kHz
--   the_plant_against_the_model       the plant's L1 and C1 off the model's: 1 kHz robust
--   the_voltage_pis_zero  the setpoint weight and the voltage PI's zero
--   the_prefilter    the reference's prefilter, its slope fed forward or not
--   square_wave      the chosen ones (3 kHz, 1000 Hz with its zero at 0.1, b 1, the prefilter
--                    1 kHz fed forward; the observer's 100 Hz) over four
--                    cycles, written to g_out_file; checked
entity output_voltage_control_tb is
  generic (
      runner_cfg  : string;
      g_out_file  : string := "output_voltage_control.csv";
      g_current_hz : real := 3000.0;
      g_voltage_hz : real := 1000.0;
      g_setpoint_weight : real := 1.0;
      g_voltage_zero : real := 0.1;
      -- the reference's prefilter (its slope fed forward), 0: none
      g_prefilter_hz : real := 1000.0;
      g_observer_voltage_hz : real := 100.0;
      g_observer_current_hz : real := 1000.0
  );
end;

architecture vunit_simulation of output_voltage_control_tb is

    constant period : real := output_period;
    constant dt     : real := period / 2.0;      -- the observer's step

    -- the plant
    constant r1 : real := 0.51;
    constant l2 : real := 5.0e-6;
    constant r2 : real := 0.1;
    constant c2 : real := 7.0e-6;
    constant vdc : real := 94.0;

    -- the limits
    constant i_max  : real := 8.0;
    constant vl_max : real := 60.0;
    constant u_max  : real := 0.95 * vdc;

begin

    main : process
        variable u_bridge : real := 0.0;
        -- the plant's L1 and C1 (the model's: 0.83 mH, C1 + C2 18 uF)
        variable l1 : real := 0.83e-3;
        variable c1 : real := 11.0e-6;

        impure function deriv(t : real; s : real_vector) return real_vector is
            variable retval : real_vector(0 to 3);
        begin
            retval(0) := (u_bridge - r1 * s(0) - s(1)) / l1;
            retval(1) := (s(0) - s(2)) / c1;
            retval(2) := (s(1) - r2 * s(2) - s(3)) / l2;
            retval(3) := s(2) / c2;
            return retval;
        end function;

        procedure rk4 is new generic_rk4 generic map(deriv);


        variable seed1, seed2 : positive := 11;
        impure function gaussian return real is
            variable u1, u2 : real;
        begin
            uniform(seed1, seed2, u1);
            uniform(seed1, seed2, u2);
            return sqrt(-2.0 * log(realmax(u1, 1.0e-12))) * cos(MATH_2_PI * u2);
        end function;

        -- one run: the square wave for cycles of 10 ms; its figures, and the trace when written
        type figures_record is record
            overshoot, rise_time, settling_time, steady_error, steady_noise, i_peak : real;
            limited_steps : natural;
            -- the bridge reference's change from a period to the next, rms (settled): an
            -- oscillation at half the control's rate shows here
            u_alternation : real;
        end record;

        constant substeps : natural := 200;   -- ~0.11 us steps: the 35 kHz second stage, the PWM's edges
        -- the sigma-delta's filter (sdm_adc): three sections of ~0.8 us (16 bits at 20 MHz)
        constant sdm_alpha : real := 1.0 - exp(-(dt / real(substeps)) / 0.8e-6);

        impure function run_square(current_hz, voltage_hz, b : real; cycles : natural; write_trace : boolean;
            observer_voltage_hz : real := 100.0; current_limit : real := i_max;
            observer_current_hz : real := g_observer_current_hz;
            board_like : boolean := true; voltage_zero : real := g_voltage_zero;
            prefilter_hz : real := g_prefilter_hz; feedforward : real := 1.0) return figures_record is
            constant observer_k : observer_coefficients_record :=
                observer_coefficients(observer_design(0.83e-3, 18.0e-6, 0.51, dt, observer_current_hz, observer_voltage_hz));
            constant observer_kf : observer_fixed_coefficients_record := to_fixed(observer_k);
            constant d : voltage_control_design_record :=
                voltage_control_design(period, 0.83e-3, 18.0e-6, 0.51, current_hz, voltage_hz, current_limit, vl_max, u_max, b,
                                       voltage_zero => voltage_zero, prefilter_hz => prefilter_hz,
                                       feedforward => feedforward);
            variable plant : real_vector(0 to 3) := (others => 0.0);
            variable t : real := 0.0;
            variable obs : observer_fixed_record := init_observer_fixed;
            variable control : voltage_control_record := init_voltage_control;
            variable u_applied, u_next, v_ref, i_sum_this, i_sum_last : real := 0.0;
            variable reading_sum : fixed_number := fixed_zero;
            variable reading_count : natural := 0;
            variable readings_this_step : natural;
            variable duty, position, f1, f2, f3, i_measured : real := 0.0;
            variable obs_real : observer_real_record;
            variable figures : figures_record := (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0);
            variable u_previous, alternation_sum : real := 0.0;
            variable alternation_count : natural := 0;
            constant half_steps : natural := natural(real(cycles) * 10.0e-3 / dt);
            -- the edges' figures: 5 % band of the 40 V step (2 V) for the settling, 10 .. 90 % rise
            variable edge_time, peak, last_level, error_sum, error_square_sum : real := 0.0;
            variable rising : boolean := true;
            variable rise_start, rise_end, settled_at : real;
            variable counted, edges : natural := 0;
            variable settling_sum, rise_sum, overshoot_max : real := 0.0;
            file trace : text;
            variable ln : line;
        begin
            if write_trace then
                file_open(trace, g_out_file, write_mode);
                write(ln, string'("t,v_ref,v_c1,v_c2,i_l1,v_estimate,i_estimate,i_ref,u_ref,du,iload"));
                writeline(trace, ln);
            end if;
            for h in 0 to half_steps - 1 loop
                -- the reference: -20 V the first half of a cycle, +20 V the second
                if (natural(t / 5.0e-3) mod 2) = 0 then v_ref := -20.0; else v_ref := 20.0; end if;

                -- the plant over the half period: the bridge's voltage, its average (1.03 of the
                -- reference less 4.2 V, as measured) or (board_like) switched: +-vdc by the duty on
                -- the modulator's sawtooth carrier, from the period's start
                u_bridge   := realmax(-vdc, realmin(vdc, 1.03 * u_applied - 4.2));
                duty       := 0.5 + 0.5 * u_bridge / vdc;
                i_sum_last := i_sum_this;
                i_sum_this := 0.0;
                if h mod 3 = 0 then readings_this_step := 3; else readings_this_step := 2; end if;
                for n in 1 to substeps loop
                    if board_like then
                        position := (real((h mod 2) * substeps + n) - 0.5) / real(2 * substeps);
                        if position < duty then u_bridge := vdc; else u_bridge := -vdc; end if;
                    end if;
                    rk4(t, plant, dt / real(substeps));
                    t := t + dt / real(substeps);
                    -- the measured current: (board_like) through the sigma-delta's filter
                    if board_like then
                        f1 := f1 + sdm_alpha * (plant(0) - f1);
                        f2 := f2 + sdm_alpha * (f1 - f2);
                        f3 := f3 + sdm_alpha * (f2 - f3);
                        i_measured := f3;
                    else
                        i_measured := plant(0);
                    end if;
                    i_sum_this := i_sum_this + i_measured;
                    if n mod (substeps / readings_this_step) = 0 and reading_count < max_readings then
                        reading_sum   := reading_sum + to_fixed(plant(1) + 15.0 * gaussian);
                        reading_count := reading_count + 1;
                    end if;
                end loop;

                -- the observer's step, with the reference the bridge applies over the next half
                -- period (at a period start the one computed at the last start)
                if (h + 1) mod 2 = 0 then
                    u_applied := u_next;
                end if;
                obs := observer_step(obs, observer_kf, to_fixed(u_applied),
                                     to_fixed((i_sum_this + i_sum_last) / real(2 * substeps)),
                                     readings_mean(reading_sum, reading_count, obs));
                reading_sum   := fixed_zero;
                reading_count := 0;
                obs_real := to_real(obs);

                -- a period start: the control from the estimates, its reference for the next period
                if (h + 1) mod 2 = 0 then
                    control := voltage_control_step(control, d, observer_k, v_ref, obs_real.i, obs_real.v,
                                                    obs_real.integral_i, -obs_real.integral_v, u_applied);
                    -- the reference's change a period on, where it should be steady
                    if t > 10.0e-3 and (natural(t / 1.0e-3) mod 5) >= 3 then
                        alternation_sum := alternation_sum + (control.u_ref - u_previous)**2;
                        alternation_count := alternation_count + 1;
                    end if;
                    u_previous := control.u_ref;
                    u_next := control.u_ref;
                    if control.current_limited or control.voltage_limited or control.bridge_limited then
                        figures.limited_steps := figures.limited_steps + 1;
                    end if;
                end if;

                -- the figures, after the first cycle: each edge's rise, overshoot, settling
                if t > 10.0e-3 then
                    if (v_ref > 0.0) /= rising or edges = 0 then
                        if edges > 0 then
                            settling_sum := settling_sum + (settled_at - edge_time);
                            rise_sum := rise_sum + (rise_end - rise_start);
                        end if;
                        rising := v_ref > 0.0;
                        edge_time := t;
                        peak := 0.0;
                        rise_start := -1.0; rise_end := -1.0; settled_at := t;
                        edges := edges + 1;
                    end if;
                    -- C1's voltage relative to the step: 0 at the old level, 1 at the new
                    last_level := (plant(1) - (-v_ref)) / (2.0 * v_ref);
                    if rise_start < 0.0 and last_level > 0.1 then rise_start := t; end if;
                    if rise_end < 0.0 and last_level > 0.9 then rise_end := t; end if;
                    peak := realmax(peak, last_level - 1.0);
                    overshoot_max := realmax(overshoot_max, peak);
                    if abs(last_level - 1.0) > 0.05 then settled_at := t; end if;
                    -- the last 2 ms of each half cycle: the steady error and noise
                    if t - edge_time > 3.0e-3 then
                        counted := counted + 1;
                        error_sum := error_sum + (plant(1) - v_ref);
                        error_square_sum := error_square_sum + (plant(1) - v_ref)**2;
                    end if;
                    -- the inductor current's mean over the last period: the switching ripple off
                    figures.i_peak := realmax(figures.i_peak, abs((i_sum_this + i_sum_last) / real(2 * substeps)));
                end if;

                if write_trace then
                    write(ln, t, right, 0, 8); write(ln, string'(","));
                    write(ln, v_ref, right, 0, 2); write(ln, string'(","));
                    write(ln, plant(1), right, 0, 4); write(ln, string'(","));
                    write(ln, plant(3), right, 0, 4); write(ln, string'(","));
                    write(ln, plant(0), right, 0, 4); write(ln, string'(","));
                    write(ln, obs_real.v, right, 0, 4); write(ln, string'(","));
                    write(ln, obs_real.i, right, 0, 4); write(ln, string'(","));
                    write(ln, control.i_ref, right, 0, 4); write(ln, string'(","));
                    write(ln, control.u_ref, right, 0, 3); write(ln, string'(","));
                    write(ln, obs_real.integral_i, right, 0, 3); write(ln, string'(","));
                    write(ln, -obs_real.integral_v, right, 0, 4);
                    writeline(trace, ln);
                end if;
            end loop;
            if write_trace then
                file_close(trace);
            end if;
            figures.overshoot     := overshoot_max * 100.0;
            figures.rise_time     := rise_sum / real(edges - 1);
            figures.settling_time := settling_sum / real(edges - 1);
            figures.steady_error  := error_sum / real(counted);
            figures.steady_noise  := sqrt(error_square_sum / real(counted) - figures.steady_error**2);
            figures.u_alternation := sqrt(alternation_sum / real(realmax(1.0, real(alternation_count))));
            return figures;
        end function;

        procedure show(current_hz, voltage_hz, b : real; f : figures_record) is
        begin
            info("current " & integer'image(integer(current_hz)) & " Hz, voltage " & integer'image(integer(voltage_hz)) &
                 " Hz, b " & real'image(b) & ": overshoot " & integer'image(integer(f.overshoot)) & " %, 10..90 % in " &
                 integer'image(integer(f.rise_time * 1.0e6)) & " us, settled (5 %) in " &
                 integer'image(integer(f.settling_time * 1.0e6)) & " us, steady " &
                 real'image(f.steady_error) & " V +- " & real'image(f.steady_noise) & " V rms, peak current " &
                 real'image(f.i_peak) & " A, limited steps " & integer'image(f.limited_steps) &
                 ", the reference's change a period " & real'image(f.u_alternation) & " V rms");
        end procedure;

        type real_list is array (natural range <>) of real;
        constant currents : real_list := (1500.0, 2000.0, 3000.0);
        constant voltages : real_list := (300.0, 500.0, 800.0);
        constant weights  : real_list := (1.0, 0.6, 0.3);
        constant observer_hz : real_list := (100.0, 30.0, 10.0);
        constant observer_current : real_list := (3000.0, 1500.0, 1000.0);
        constant plant_l1 : real_list := (0.6e-3, 0.83e-3, 1.1e-3);
        constant plant_c1 : real_list := (11.0e-6, 6.0e-6);
        constant prefilter_scan : real_list := (0.0, 500.0, 1000.0, 2000.0);
        type zero_scan_record is record
            b, zero, voltage_hz : real;
        end record;
        type zero_scan_array is array (0 to 4) of zero_scan_record;
        constant zero_scan : zero_scan_array := ((0.6, 0.2, 800.0), (1.0, 0.2, 800.0), (1.0, 0.05, 1200.0),
                                                 (1.0, 0.02, 1200.0), (1.0, 0.0, 1200.0));
        constant scan_current : real_list := (3000.0, 1000.0);
        variable f : figures_record;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            -- the sweep a test a current loop's bandwidth (run side by side)
            if run("tune_the_loops_at_1500_hz") or run("tune_the_loops_at_2000_hz")
               or run("tune_the_loops_at_3000_hz") then
                for ci in currents'range loop
                    if running_test_case = "tune_the_loops_at_" & integer'image(integer(currents(ci))) & "_hz" then
                        for vi in voltages'range loop
                            if voltages(vi) < currents(ci) / 2.5 then
                                for bi in weights'range loop
                                    f := run_square(currents(ci), voltages(vi), weights(bi), 3, false);
                                    show(currents(ci), voltages(vi), weights(bi), f);
                                end loop;
                            end if;
                        end loop;
                    end if;
                end loop;

            elsif run("the_voltage_pis_zero") then
                -- the setpoint weight and the voltage PI's zero (a share of its bandwidth): with
                -- the observer's load current fed forward, little integral is needed, and a low
                -- zero leaves no slow tail after the step
                for n in 0 to 4 loop
                    f := run_square(g_current_hz, zero_scan(n).voltage_hz, zero_scan(n).b, 4, false, g_observer_voltage_hz,
                                    i_max, g_observer_current_hz, true, zero_scan(n).zero, 0.0);
                    info("b " & real'image(zero_scan(n).b) & ", the zero " & real'image(zero_scan(n).zero) & ":");
                    show(g_current_hz, zero_scan(n).voltage_hz, zero_scan(n).b, f);
                end loop;

            elsif run("the_prefilter") then
                -- the reference through the prefilter before the voltage PI, its slope fed forward
                -- (0 Hz: none), and the prefilter without the feedforward
                for n in 0 to 3 loop
                    f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 4, false, g_observer_voltage_hz,
                                    i_max, g_observer_current_hz, true, g_voltage_zero, prefilter_scan(n));
                    info("the prefilter " & real'image(prefilter_scan(n)) & " Hz, fed forward:");
                    show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);
                end loop;
                f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 4, false, g_observer_voltage_hz,
                                i_max, g_observer_current_hz, true, g_voltage_zero, g_prefilter_hz, 0.0);
                info("the prefilter " & real'image(g_prefilter_hz) & " Hz, not fed forward:");
                show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);

            elsif run("the_observers_voltage_bandwidth") then
                for n in 0 to 2 loop
                    f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 4, false, observer_hz(n));
                    info("the observer's voltage loop " & real'image(observer_hz(n)) & " Hz:");
                    show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);
                end loop;

            elsif run("the_current_limit_holds") then
                -- 1 A: the 40 V steps slew at 1 A / 18 uF ~ 55 V/ms, no windup after
                f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 4, false, g_observer_voltage_hz, 1.0);
                show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);
                check(f.i_peak < 1.3, "the inductor current's mean within the 1 A limit and the current loop's overshoot as it takes hold");
                check(f.overshoot < 10.0, "no windup: overshoot under 10 %");
                check(f.settling_time < 3.0e-3, "settled within 3 ms");

            elsif run("the_observers_current_bandwidth") then
                -- the board's 3 kHz oscillation (2026-10-10): the switched bridge and the
                -- sigma-delta's filter show it; the averaged model did not
                for n in 0 to 2 loop
                    f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 3, false, g_observer_voltage_hz,
                                    i_max, observer_current(n), true);
                    info("the observer's current loop " & real'image(observer_current(n)) & " Hz (switched, filtered):");
                    show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);
                end loop;
                f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 3, false, g_observer_voltage_hz,
                                i_max, 3000.0, false);
                info("the observer's current loop 3000 Hz, the averaged model:");
                show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);

            elsif run("the_plant_against_the_model") then
                -- the plant's L1 and C1 off the observer's and control's model. The board's
                -- oscillation at half the control's rate (the observer's current loop at 3 kHz,
                -- 2026-10-10) comes with the inductor below the model's: 0.6 mH oscillates at 3 kHz
                -- (54 V rms the reference's change a period, 64 % overshoot), not at 1 kHz
                for li in 0 to 2 loop
                    for ci in 0 to 1 loop
                        l1 := plant_l1(li);
                        c1 := plant_c1(ci);
                        for oi in 0 to 1 loop
                            f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 3, false, g_observer_voltage_hz,
                                            i_max, scan_current(oi), true);
                            info("plant L1 " & real'image(l1 * 1.0e3) & " mH, C1 " & real'image(c1 * 1.0e6) &
                                 " uF, the observer's current loop " & real'image(scan_current(oi)) & " Hz: overshoot " &
                                 integer'image(integer(f.overshoot)) & " %, steady +- " & real'image(f.steady_noise) &
                                 " V, the reference's change a period " & real'image(f.u_alternation) & " V rms");
                            -- the default 1 kHz holds with the inductor 30 % low or high
                            if scan_current(oi) = g_observer_current_hz then
                                check(f.u_alternation < 15.0, "no oscillation at half the control's rate");
                                -- with C1 at 6 uF the second stage resonates near the 40 kHz
                                -- switching: +-3 V of ripple on the voltage, ~7 % of the overshoot
                                check(f.overshoot < 25.0, "overshoot under 25 %");
                            end if;
                        end loop;
                    end loop;
                end loop;
                l1 := 0.83e-3;
                c1 := 11.0e-6;

            elsif run("square_wave") then
                f := run_square(g_current_hz, g_voltage_hz, g_setpoint_weight, 4, true, g_observer_voltage_hz);
                show(g_current_hz, g_voltage_hz, g_setpoint_weight, f);
                check(f.overshoot < 10.0, "overshoot under 10 %");
                check(f.settling_time < 2.0e-3, "settled within 5 % in 2 ms");
                check(f.steady_noise < 1.0, "steady within 1 V rms");
                check(abs(f.steady_error) < 0.5, "the steady error within 0.5 V");
                check(f.i_peak <= i_max + 1.0, "the current within its limit (and the ripple of a period)");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
