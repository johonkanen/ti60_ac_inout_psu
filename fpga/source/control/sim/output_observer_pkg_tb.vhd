LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fixed_format_pkg.all;
    use work.output_bridge_pkg.all;
    use work.output_observer_pkg.all;
    use work.ode_pkg.all;

-- output_observer_pkg against hVHDL_ode's rk4 on the output filter, with the fit's values
-- (output_filter_fit_tb: L 0.83 mH, C1 + C2 18 uF, R 0.51 ohm) and a step of half the
-- switching period (output_bridge_pkg):
--   the zero order hold step is the filter's exact solution over a step;
--   the fixed point step follows the real one;
--   the observer on a plant with the board's bridge error (3 % gain, -4.2 V), a load current
--   step and the board's voltage reading noise finds the disturbances: du the bridge's error,
--   iload the load, and a voltage estimate far less noisy than the readings.
entity output_observer_pkg_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of output_observer_pkg_tb is

    constant l  : real := 0.83e-3;
    constant c  : real := 18.0e-6;
    constant r  : real := 0.51;
    constant dt : real := output_step;

begin

    main : process
        -- the plant: the filter with the bridge's real voltage and the load current
        variable u_plant, i_load_plant : real := 0.0;

        impure function deriv(t : real; states : real_vector) return real_vector is
            variable retval : real_vector(0 to 1);
        begin
            retval(0) := (u_plant - r * states(0) - states(1)) / l;
            retval(1) := (states(0) - i_load_plant) / c;
            return retval;
        end function;

        procedure rk4 is new generic_rk4 generic map(deriv);

        constant design : observer_design_record := observer_design(l, c, r, dt, 3000.0, 100.0);
        constant k      : observer_coefficients_record := observer_coefficients(design);
        constant kf     : observer_fixed_coefficients_record := to_fixed(k);

        constant substeps : natural := 100;
        variable plant : real_vector(0 to 1);
        variable t     : real := 0.0;
        variable seed1, seed2 : positive := 3;

        -- a gaussian, Box-Muller
        impure function gaussian return real is
            variable u1, u2 : real;
        begin
            uniform(seed1, seed2, u1);
            uniform(seed1, seed2, u2);
            return sqrt(-2.0 * log(realmax(u1, 1.0e-12))) * cos(MATH_2_PI * u2);
        end function;

        variable obs_real  : observer_real_record := init_observer_real;
        variable obs_fixed : observer_fixed_record := init_observer_fixed;
        variable fixed_as_real : observer_real_record;
        variable i_sum_this, i_sum_last, v_sum, v_meas, i_meas, u_ref : real;
        variable expected_i, expected_v, worst_i, worst_v, worst_du, worst_iload : real;
        variable v_error_sum, v_reading_error_sum : real;
        variable counted : natural;
        variable du_sum, iload_sum : real;

        -- the plant over a step, its current's and voltage's sums for the measurements
        procedure plant_step is
        begin
            i_sum_last := i_sum_this;
            i_sum_this := 0.0;
            v_sum      := 0.0;
            for n in 1 to substeps loop
                rk4(t, plant, dt / real(substeps));
                t := t + dt / real(substeps);
                i_sum_this := i_sum_this + plant(0);
                v_sum      := v_sum + plant(1);
            end loop;
        end procedure;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("the_zoh_step_is_the_filters_solution") then
                info("phi " & real'image(k.phi11) & " " & real'image(k.phi12) & " " &
                     real'image(k.phi21) & " " & real'image(k.phi22));
                info("gamma " & real'image(k.gamma11) & " " & real'image(k.gamma12) & " " &
                     real'image(k.gamma21) & " " & real'image(k.gamma22));
                info("kp_i " & real'image(k.kp_i) & ", ki_dt_i " & real'image(k.ki_dt_i) &
                     ", kp_v " & real'image(k.kp_v) & ", ki_dt_v " & real'image(k.ki_dt_v));
                plant := (2.0, 10.0);
                u_plant := 30.0;
                i_load_plant := 1.5;
                expected_i := k.phi11 * 2.0 + k.phi12 * 10.0 + k.gamma11 * 30.0 + k.gamma12 * 1.5;
                expected_v := k.phi21 * 2.0 + k.phi22 * 10.0 + k.gamma21 * 30.0 + k.gamma22 * 1.5;
                for n in 1 to 1000 loop
                    rk4(t, plant, dt / 1000.0);
                end loop;
                info("rk4 " & real'image(plant(0)) & " A, " & real'image(plant(1)) & " V; zoh " &
                     real'image(expected_i) & " A, " & real'image(expected_v) & " V");
                check(abs(plant(0) - expected_i) < 1.0e-9, "the current after a step");
                check(abs(plant(1) - expected_v) < 1.0e-9, "the voltage after a step");

            elsif run("the_fixed_point_step_follows_the_real_one") then
                -- the observer on clean measurements of a plant ringing after reference steps
                plant := (0.0, 0.0);
                i_sum_this := 0.0;
                worst_i := 0.0; worst_v := 0.0; worst_du := 0.0; worst_iload := 0.0;
                for step in 0 to 8000 loop
                    if step < 2000 then u_ref := -25.0; else u_ref := 25.0; end if;
                    if step = 5000 then i_load_plant := 2.0; end if;
                    u_plant := 1.03 * u_ref - 4.2;
                    plant_step;
                    i_meas := (i_sum_this + i_sum_last) / real(2 * substeps);
                    v_meas := v_sum / real(substeps);
                    obs_real  := observer_step(obs_real, k, u_ref, i_meas, v_meas);
                    obs_fixed := observer_step(obs_fixed, kf, to_fixed(u_ref), to_fixed(i_meas), to_fixed(v_meas));
                    fixed_as_real := to_real(obs_fixed);
                    worst_i     := realmax(worst_i, abs(fixed_as_real.i - obs_real.i));
                    worst_v     := realmax(worst_v, abs(fixed_as_real.v - obs_real.v));
                    worst_du    := realmax(worst_du, abs(fixed_as_real.du - obs_real.du));
                    worst_iload := realmax(worst_iload, abs(fixed_as_real.iload - obs_real.iload));
                end loop;
                info("at the end: v " & real'image(fixed_as_real.v - obs_real.v) & " V, du " &
                     real'image(fixed_as_real.du - obs_real.du) & " V");
                -- the coefficients' rounding (24 fractional bits) shows in the steps' transients
                -- only, ~0.01 % of the 50 V step; settled the two agree to microvolts
                check(abs(fixed_as_real.v - obs_real.v) < 5.0e-4, "settled: the voltage within 0.5 mV");
                check(abs(fixed_as_real.du - obs_real.du) < 1.0e-3, "settled: du within 1 mV");
                info("fixed vs real, worst: i " & real'image(worst_i) & " A, v " & real'image(worst_v) &
                     " V, du " & real'image(worst_du) & " V, iload " & real'image(worst_iload) & " A");
                check(worst_i < 1.0e-3, "the current within 1 mA");
                check(worst_v < 2.0e-2, "the voltage within 20 mV through the steps (0.04 % of the 50 V step)");
                check(worst_du < 2.0e-2, "du within 20 mV through the steps");
                check(worst_iload < 1.0e-3, "iload within 1 mA");

            elsif run("finds_the_disturbances_through_the_noise") then
                -- the bridge 3 % over its reference and 4.2 V low, a 2 A load from 80 ms, two
                -- voltage readings a step with 15 V rms of noise each (the board's vfilt_out)
                plant := (0.0, 0.0);
                i_sum_this := 0.0;
                v_error_sum := 0.0; v_reading_error_sum := 0.0; counted := 0;
                du_sum := 0.0; iload_sum := 0.0;
                for step in 0 to 6000 loop   -- 136 ms
                    if step < 1500 then u_ref := -25.0; else u_ref := 25.0; end if;
                    if step = 3500 then i_load_plant := 2.0; end if;
                    u_plant := 1.03 * u_ref - 4.2;
                    plant_step;
                    i_meas := (i_sum_this + i_sum_last) / real(2 * substeps);
                    v_meas := v_sum / real(substeps) + 15.0 * (gaussian + gaussian) / 2.0;
                    obs_fixed := observer_step(obs_fixed, kf, to_fixed(u_ref), to_fixed(i_meas), to_fixed(v_meas));
                    fixed_as_real := to_real(obs_fixed);
                    -- settled with the load: its last 1500 steps
                    if step > 4500 then
                        counted := counted + 1;
                        du_sum := du_sum + fixed_as_real.du;
                        iload_sum := iload_sum + fixed_as_real.iload;
                        v_error_sum := v_error_sum + (fixed_as_real.v - plant(1))**2;
                        v_reading_error_sum := v_reading_error_sum + (v_meas - v_sum / real(substeps))**2;
                    end if;
                end loop;
                info("du " & real'image(du_sum / real(counted)) & " V (the bridge's error " &
                     real'image(0.03 * 25.0 - 4.2) & " V), iload " & real'image(iload_sum / real(counted)) & " A");
                info("voltage: the estimate " & real'image(sqrt(v_error_sum / real(counted))) &
                     " V rms off, the readings " & real'image(sqrt(v_reading_error_sum / real(counted))) & " V rms");
                check(abs(du_sum / real(counted) - (0.03 * 25.0 - 4.2)) < 0.3, "du: the bridge's error within 0.3 V");
                check(abs(iload_sum / real(counted) - 2.0) < 0.1, "iload: the 2 A load within 0.1 A");
                check(sqrt(v_error_sum / real(counted)) < 2.0, "the voltage estimate within 2 V rms");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
