LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;
    use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.ode_pkg.all;

-- Fits an averaged model of the output bridge's filter to a measured voltage step
-- (data/output_step_m25_p25.csv, data/export_step_csv.py: the voltage mode's reference from
-- -25 V to +25 V on the board, 2026-10-09), with hVHDL_ode's rk4. The filter is two stages:
-- L1 (1 mH) and C1 (10 uF), then the common mode filter's stray inductance L2 (a few uH) and
-- C2 (7 uF):
--
--   L1 di1/dt = u - R1 i1 - v1      i1 the bridge's inductor current (iac_out)
--   C1 dv1/dt = i1 - i2             v1 the first capacitor's voltage (vfilt_out)
--   L2 di2/dt = v1 - R2 i2 - v2
--   C2 dv2/dt = i2
--   u         = k * vref(t - td) + u0
--
-- u is the bridge's average voltage: k the gain from the reference (1 with an exact DC link
-- reading), u0 an offset (dead time, delays), td the delay from the "stepped" lane to the
-- step (a switching period and the measurements' filters). The second stage resonates at
-- ~35 kHz with L2 = 5 uH, above what one-period means at 500 kS/s show: below it C1 and C2 act
-- as one, so L2, R2 and C2 are held at their nominal values and the fit finds C1. Nelder-Mead
-- searches L1, C1, R1 (as logarithms), k, u0 and td for the least squares of the measured
-- one-period means, the voltage and the current each scaled by their step's size. It starts
-- from the state measured at the window's first sample, C2 at C1's voltage.
--
-- The fit's parameters go to the log and with the model next to the measurement into
-- <g_out_file> (t, the measured and the modelled v and i) for plotting.
entity output_filter_fit_tb is
  generic (
      runner_cfg  : string;
      g_data_file : string := "output_step_m25_p25.csv";
      g_out_file  : string := "output_filter_fit.csv"
  );
end;

architecture vunit_simulation of output_filter_fit_tb is
begin

    main : process
        constant max_rows : natural := 20000;
        type real_array is array (natural range <>) of real;
        variable t_data, vref, v_meas, i_meas : real_array(0 to max_rows-1);
        variable rows : natural := 0;
        variable dt   : real;

        -- the second stage, nominal: the common mode filter's stray inductance and a little
        -- damping, the output capacitor
        constant l2 : real := 5.0e-6;
        constant r2 : real := 0.1;
        constant c2 : real := 7.0e-6;

        -- the model's parameters for deriv
        variable l, c, r, k, u0, td : real;
        variable u_before, u_after   : real;

        -- states: i1, v1, i2, v2
        impure function deriv(t : real; states : real_vector) return real_vector is
            variable retval : real_vector(0 to 3);
            variable u      : real;
        begin
            if t < td then u := u_before; else u := u_after; end if;
            retval(0) := (u - r * states(0) - states(1)) / l;
            retval(1) := (states(0) - states(2)) / c;
            retval(2) := (states(1) - r2 * states(2) - states(3)) / l2;
            retval(3) := states(2) / c2;
            return retval;
        end function;

        procedure rk4 is new generic_rk4 generic map(deriv);

        constant n_par : natural := 6;
        subtype par_vector is real_vector(0 to n_par-1);
        type simplex_array is array (0 to n_par) of par_vector;

        variable v_scale, i_scale : real;
        variable evaluations : natural := 0;

        procedure set_parameters(p : par_vector) is
        begin
            l  := exp(p(0));
            c  := exp(p(1));
            r  := exp(p(2));
            k  := p(3);
            u0 := p(4);
            td := p(5);
            u_before := k * vref(0) + u0;
            u_after  := k * vref(rows-1) + u0;
        end procedure;

        -- the model over the window: the scaled squared error summed, the model into
        -- v_model/i_model when keep
        variable v_model, i_model : real_array(0 to max_rows-1);
        impure function cost(p : par_vector; keep : boolean := false) return real is
            variable state : real_vector(0 to 3);
            variable t, e  : real := 0.0;
        begin
            set_parameters(p);
            evaluations := evaluations + 1;
            if l <= 0.0 or c <= 0.0 then return 1.0e30; end if;
            -- the current shared by the capacitors as at a slow change, C2 at C1's voltage
            state := (i_meas(0), v_meas(0), i_meas(0) * c2 / (c + c2), v_meas(0));
            t := t_data(0);
            for n in 0 to rows-1 loop
                if keep then
                    v_model(n) := state(1);
                    i_model(n) := state(0);
                end if;
                e := e + ((state(1) - v_meas(n)) / v_scale)**2 + ((state(0) - i_meas(n)) / i_scale)**2;
                rk4(t, state, dt);
                t := t + dt;
            end loop;
            return e / real(rows);
        end function;

        procedure read_data is
            file f : text open read_mode is g_data_file;
            variable ln : line;
            variable value : real;
            variable comma : character;
            variable fields : real_vector(0 to 7);
        begin
            readline(f, ln);   -- the header
            while not endfile(f) and rows < max_rows loop
                readline(f, ln);
                for n in fields'range loop
                    read(ln, fields(n));
                    read(ln, comma);
                end loop;
                -- t, vref, vdc_out, vfilt_out, iac_out, vdc_mean, vfilt_mean, iac_mean, ...
                t_data(rows) := fields(0);
                vref(rows)   := fields(1);
                v_meas(rows) := fields(6);
                i_meas(rows) := fields(7);
                rows := rows + 1;
            end loop;
            dt := (t_data(rows-1) - t_data(0)) / real(rows-1);
        end procedure;

        -- Nelder-Mead: reflection 1, expansion 2, contraction 0.5, shrink 0.5
        procedure nelder_mead(p : inout par_vector; steps : par_vector; max_evaluations : natural;
                              best_cost : out real) is
            variable s : simplex_array;
            variable f : real_vector(0 to n_par);
            variable centroid, xr, xe, xc : par_vector;
            variable fr, fe, fc : real;
            variable lo, hi, second : natural;
            variable start : natural := evaluations;
        begin
            for n in 0 to n_par loop
                s(n) := p;
                if n > 0 then s(n)(n-1) := s(n)(n-1) + steps(n-1); end if;
                f(n) := cost(s(n));
            end loop;
            while evaluations - start < max_evaluations loop
                lo := 0; hi := 0;
                for n in 1 to n_par loop
                    if f(n) < f(lo) then lo := n; end if;
                    if f(n) > f(hi) then hi := n; end if;
                end loop;
                second := lo;
                for n in 0 to n_par loop
                    if n /= hi and f(n) > f(second) then second := n; end if;
                end loop;
                exit when abs(f(hi) - f(lo)) <= 1.0e-9 * abs(f(lo)) + 1.0e-15;
                centroid := (others => 0.0);
                for n in 0 to n_par loop
                    if n /= hi then
                        for m in par_vector'range loop
                            centroid(m) := centroid(m) + s(n)(m) / real(n_par);
                        end loop;
                    end if;
                end loop;
                for m in par_vector'range loop
                    xr(m) := centroid(m) + (centroid(m) - s(hi)(m));
                end loop;
                fr := cost(xr);
                if fr < f(lo) then
                    for m in par_vector'range loop
                        xe(m) := centroid(m) + 2.0 * (centroid(m) - s(hi)(m));
                    end loop;
                    fe := cost(xe);
                    if fe < fr then s(hi) := xe; f(hi) := fe; else s(hi) := xr; f(hi) := fr; end if;
                elsif fr < f(second) then
                    s(hi) := xr; f(hi) := fr;
                else
                    for m in par_vector'range loop
                        xc(m) := centroid(m) + 0.5 * (s(hi)(m) - centroid(m));
                    end loop;
                    fc := cost(xc);
                    if fc < f(hi) then
                        s(hi) := xc; f(hi) := fc;
                    else
                        for n in 0 to n_par loop
                            if n /= lo then
                                for m in par_vector'range loop
                                    s(n)(m) := s(lo)(m) + 0.5 * (s(n)(m) - s(lo)(m));
                                end loop;
                                f(n) := cost(s(n));
                            end if;
                        end loop;
                    end if;
                end if;
            end loop;
            lo := 0;
            for n in 1 to n_par loop
                if f(n) < f(lo) then lo := n; end if;
            end loop;
            p := s(lo);
            best_cost := f(lo);
        end procedure;

        procedure write_fit is
            file f : text open write_mode is g_out_file;
            variable ln : line;
        begin
            write(ln, string'("t,vfilt_mean,iac_mean,v_model,i_model"));
            writeline(f, ln);
            for n in 0 to rows-1 loop
                write(ln, t_data(n), right, 0, 9);
                write(ln, string'(","));
                write(ln, v_meas(n), right, 0, 4);
                write(ln, string'(","));
                write(ln, i_meas(n), right, 0, 5);
                write(ln, string'(","));
                write(ln, v_model(n), right, 0, 4);
                write(ln, string'(","));
                write(ln, i_model(n), right, 0, 5);
                writeline(f, ln);
            end loop;
        end procedure;

        function rms_of(a, b : real_array; count : natural) return real is
            variable s : real := 0.0;
        begin
            for n in 0 to count-1 loop
                s := s + (a(n) - b(n))**2;
            end loop;
            return sqrt(s / real(count));
        end function;

        -- the share of the measurement's variance around its mean that the model explains
        function r_squared(model, measured : real_array; count : natural) return real is
            variable mean, total, residual : real := 0.0;
        begin
            for n in 0 to count-1 loop
                mean := mean + measured(n) / real(count);
            end loop;
            for n in 0 to count-1 loop
                total    := total + (measured(n) - mean)**2;
                residual := residual + (measured(n) - model(n))**2;
            end loop;
            return 1.0 - residual / total;
        end function;

        variable p     : par_vector;
        variable steps : par_vector;
        variable best, previous : real;
        variable v_rms, i_rms, v_r2, i_r2, f0, z0, tau : real;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("fit_the_lc_filter_to_the_50_v_step") then
                read_data;
                info(integer'image(rows) & " samples, " & real'image(dt * 1.0e6) & " us apart");
                v_scale := abs(vref(rows-1) - vref(0));      -- 50 V
                i_scale := v_scale / sqrt(1.0e-3 / 17.0e-6);  -- its current peak, ~6.5 A

                -- the start: the design's 1 mH and 10 uF, R from the ringing's ~3.7 ms decay,
                -- the bridge exact, the offset seen at 0 V, a switching period
                p     := (log(1.0e-3), log(10.0e-6), log(0.5), 1.0, -4.3, 45.0e-6);
                steps := (0.2, 0.2, 0.5, 0.05, 1.0, 20.0e-6);
                previous := cost(p);
                info("start: cost " & real'image(previous));
                -- restarts from the best until it stops improving
                for restart in 1 to 6 loop
                    nelder_mead(p, steps, 1500, best);
                    info("restart " & integer'image(restart) & ": cost " & real'image(best) &
                         " after " & integer'image(evaluations) & " evaluations");
                    exit when best > previous * 0.9999;
                    previous := best;
                    steps := (0.05, 0.05, 0.2, 0.01, 0.3, 5.0e-6);
                end loop;

                best  := cost(p, keep => true);
                set_parameters(p);
                v_rms := rms_of(v_model, v_meas, rows);
                i_rms := rms_of(i_model, i_meas, rows);
                v_r2  := r_squared(v_model, v_meas, rows);
                i_r2  := r_squared(i_model, i_meas, rows);
                -- the ringing's: L1 with C1 and C2 together
                f0    := 1.0 / (MATH_2_PI * sqrt(l * (c + c2)));
                z0    := sqrt(l / (c + c2));
                tau   := 2.0 * l / r;
                info("L1 " & real'image(l * 1.0e3) & " mH");
                info("C1 " & real'image(c * 1.0e6) & " uF (with C2 " & real'image(c2 * 1.0e6) & " uF held)");
                info("R1 " & real'image(r) & " ohm");
                info("k  " & real'image(k));
                info("u0 " & real'image(u0) & " V");
                info("td " & real'image(td * 1.0e6) & " us");
                info("resonance " & real'image(f0) & " Hz, impedance " & real'image(z0) &
                     " ohm, decay time constant " & real'image(tau * 1.0e3) & " ms, Q " &
                     real'image(z0 / r));
                info("rms error: v " & real'image(v_rms) & " V, i " & real'image(i_rms) & " A (with " &
                     "the measurements' noise: vfilt_out's +-30 V spikes)");
                info("explained (R^2): v " & real'image(v_r2) & ", i " & real'image(i_r2));
                write_fit;

                -- the fit holds the measurement: most of the step's response explained, what is
                -- left mostly the measurements' noise (vfilt_out's spikes alone are ~9 V rms of
                -- a 50 V step: R^2 ~0.8 at best)
                check(v_r2 > 0.75, "the model explains the voltage's step response (R^2 > 0.75)");
                check(i_r2 > 0.8, "the model explains the current's step response (R^2 > 0.8)");
                check(l > 0.5e-3 and l < 2.0e-3, "L1 near the design's 1 mH");
                check(c > 5.0e-6 and c < 20.0e-6, "C1 near the design's 10 uF");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
