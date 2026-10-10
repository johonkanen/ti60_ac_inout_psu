LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;
    use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.output_bridge_pkg.all;
    use work.output_observer_pkg.all;
    use work.output_voltage_control_pkg.all;
    use work.scope_pkg.all;
    use work.ode_pkg.all;

-- output_control_processor fed as measurement_scaler and the output bridge feed it, its results
-- against output_observer_pkg's and output_voltage_control_pkg's steps in real numbers on the
-- same inputs (the processor's 24 fractional bits, truncated, follow them to well below the
-- noise; no longer bit for bit):
--   the observer on hVHDL_ode's model of the filter with the board's bridge error, a load step
--   and noisy readings, its coefficients written on the bus in a burst while a step runs;
--   the observer on the measured -25 V -> +25 V step (inverter/sim/data/output_step_m25_p25.csv),
--   its estimates written to g_out_file next to the measurement;
--   the bridge off between two runs: du held, no current, the voltage following;
--   disabled: no current, the voltage following the readings;
--   the closed loop on the two stage filter: the +-20 V square wave, every period's u_ref
--   against the functions', then with a 1 A current limit (its anti-windup). The functions'
--   integrators see the processor's estimates' few bits of difference but not their own
--   effect on the plant: their u_ref wanders ~20 mV from the processor's, bounded, so 50 mV.
entity output_control_processor_tb is
  generic (
      runner_cfg  : string;
      g_data_file : string := "output_step_m25_p25.csv";
      g_out_file  : string := "output_observer_replay.csv"
  );
end;

architecture vunit_simulation of output_control_processor_tb is

    constant observer_base : natural := 1160;
    constant control_base  : natural := 1180;
    constant current_channel : natural := 5;
    constant voltage_channel : natural := 1;
    constant l    : real := 0.83e-3;
    constant c    : real := 18.0e-6;
    constant r    : real := 0.51;
    constant dt   : real := output_step;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;

    signal scaled         : fixed_array(0 to 5) := (others => fixed_zero);
    signal scaled_ready   : boolean := false;
    signal scaled_channel : natural range 0 to 5 := 0;
    signal bridge_on      : boolean := false;
    signal u              : fixed_number := fixed_zero;
    signal closed_loop    : boolean := false;
    signal v_ref          : fixed_number := fixed_zero;
    signal period_start   : boolean := false;
    signal u_ref          : fixed_number;
    signal ready          : boolean;
    signal observer_probes : scope_sample_array(0 to 3);
    signal control_probes  : scope_sample_array(0 to 1);

begin

    clock <= not clock after 4 ns;


    dut : entity work.output_control_processor
    generic map(g_observer_address => observer_base, g_control_address => control_base,
                g_current_channel => current_channel, g_voltage_channel => voltage_channel)
    port map(clock, bus_in, bus_out, scaled, scaled_ready, scaled_channel, bridge_on, u,
             closed_loop, v_ref, period_start, u_ref, ready, observer_probes, control_probes);


    main : process
        alias i_estimate is << signal .output_control_processor_tb.dut.i_estimate : fixed_number >>;
        alias v_estimate is << signal .output_control_processor_tb.dut.v_estimate : fixed_number >>;
        alias du         is << signal .output_control_processor_tb.dut.du : fixed_number >>;
        alias iload      is << signal .output_control_processor_tb.dut.iload : fixed_number >>;
        alias observer_steps is << signal .output_control_processor_tb.dut.observer_steps : unsigned(31 downto 0) >>;

        variable u_plant, i_load_plant : real := 0.0;

        impure function deriv(t : real; states : real_vector) return real_vector is
            variable retval : real_vector(0 to 1);
        begin
            retval(0) := (u_plant - r * states(0) - states(1)) / l;
            retval(1) := (states(0) - i_load_plant) / c;
            return retval;
        end function;
        procedure rk4 is new generic_rk4 generic map(deriv);

        -- the closed loop's plant: the board's two stage filter
        constant l1 : real := 0.83e-3;  constant r1 : real := 0.51;  constant c1 : real := 11.0e-6;
        constant l2 : real := 5.0e-6;   constant r2 : real := 0.1;   constant c2 : real := 7.0e-6;
        constant vdc : real := 94.0;
        variable u_bridge : real := 0.0;
        impure function deriv2(t : real; s : real_vector) return real_vector is
            variable retval : real_vector(0 to 3);
        begin
            retval(0) := (u_bridge - r1 * s(0) - s(1)) / l1;
            retval(1) := (s(0) - s(2)) / c1;
            retval(2) := (s(1) - r2 * s(2) - s(3)) / l2;
            retval(3) := s(2) / c2;
            return retval;
        end function;
        procedure rk4_2 is new generic_rk4 generic map(deriv2);

        -- the tests' observer, 1.2 kHz / 120 Hz (not the defaults: written on the bus)
        constant k : observer_coefficients_record :=
            observer_coefficients(observer_design(l, c, r, dt, 1200.0, 120.0));
        constant kf : observer_fixed_coefficients_record := to_fixed(k);
        -- the defaults the closed loop runs on
        constant k_default : observer_coefficients_record :=
            observer_coefficients(observer_design(l, c, r, dt, 1000.0, 100.0));

        procedure bus_write(address : natural; data : signed) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(resize(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        -- the observer's coefficients, a write every other clock
        procedure write_coefficients is
            type coefficient_list is array (1 to 12) of coefficient_number;
            constant list : coefficient_list := (kf.phi11, kf.phi12, kf.phi21, kf.phi22, kf.gamma11, kf.gamma12,
                                                 kf.gamma21, kf.gamma22, kf.kp_i, kf.ki_dt_i, kf.kp_v, kf.ki_dt_v);
        begin
            for n in 1 to 12 loop
                bus_write(observer_base + n, list(n));
            end loop;
        end procedure;

        procedure give(channel : natural; value : fixed_number) is
        begin
            wait until rising_edge(clock);
            scaled(channel) <= value;
            scaled_channel  <= channel;
            scaled_ready    <= true;
            wait until rising_edge(clock);
            scaled_ready    <= false;
            wait for 40 ns;
        end procedure;

        variable reference : observer_real_record := init_observer_real;
        variable reading_sum : real := 0.0;
        variable reading_count : natural := 0;
        variable steps_checked : natural := 0;
        variable max_i, max_v, max_du, max_iload : real := 0.0;
        variable coefficients_written : boolean := false;
        variable reference_k : observer_coefficients_record;

        procedure reading(value : real) is
            constant fixed : fixed_number := to_fixed(value);
        begin
            give(voltage_channel, fixed);
            if reading_count < max_readings then
                reading_sum   := reading_sum + to_real(fixed);
                reading_count := reading_count + 1;
            end if;
        end procedure;

        -- an observer step: the sampled current with u, the results against the function's
        procedure step(u_now, i_meas : real; burst : boolean := false) is
            variable v_meas : real;
            variable last   : unsigned(31 downto 0);
        begin
            u <= to_fixed(u_now);
            last := observer_steps;
            give(current_channel, to_fixed(i_meas));
            if reading_count = 0 then
                v_meas := (reference.v + reference.v_prev) / 2.0;
            else
                v_meas := reading_sum / real(reading_count);
            end if;
            reference := observer_step(reference, reference_k, to_real(to_fixed(u_now)), to_real(to_fixed(i_meas)),
                                       v_meas, bridge_on);
            reading_sum   := 0.0;
            reading_count := 0;
            wait until observer_steps /= last for 10 us;
            check(observer_steps /= last, "a step within 10 us");
            wait until rising_edge(clock);
            if burst then
                -- the coefficients while the control's reset runs after the step: from the next
                -- step on
                write_coefficients;
                coefficients_written := true;
            end if;
            if coefficients_written then
                reference_k := k;
            end if;
            steps_checked := steps_checked + 1;
            max_i     := realmax(max_i, abs(to_real(i_estimate) - reference.i));
            max_v     := realmax(max_v, abs(to_real(v_estimate) - reference.v));
            max_du    := realmax(max_du, abs(to_real(du) - reference.du));
            max_iload := realmax(max_iload, abs(to_real(iload) - reference.iload));
        end procedure;

        procedure report_and_check is
        begin
            info(integer'image(steps_checked) & " steps; the largest differences from the function: i " &
                 real'image(max_i) & " A, v " & real'image(max_v) & " V, du " & real'image(max_du) &
                 " V, iload " & real'image(max_iload) & " A");
            check(max_i < 1.0e-3, "the current estimate the function's within 1 mA");
            check(max_v < 1.0e-3, "the voltage estimate within 1 mV");
            check(max_du < 1.0e-3, "du within 1 mV");
            check(max_iload < 1.0e-3, "iload within 1 mA");
        end procedure;

        variable seed1, seed2 : positive := 5;
        impure function gaussian return real is
            variable u1, u2 : real;
        begin
            uniform(seed1, seed2, u1);
            uniform(seed1, seed2, u2);
            return sqrt(-2.0 * log(realmax(u1, 1.0e-12))) * cos(MATH_2_PI * u2);
        end function;

        variable plant : real_vector(0 to 1) := (0.0, 0.0);
        variable t : real := 0.0;
        variable i_sum_this, i_sum_last, u_now : real := 0.0;
        variable readings_this_step : natural;
        constant substeps : natural := 30;
        variable du_at_stop : real;

        -- the replay
        constant max_rows : natural := 20000;
        type real_array is array (natural range <>) of real;
        variable t_data, vref_data, vfilt_data, iac_mean_data, vfilt_mean_data : real_array(0 to max_rows-1);
        variable rows : natural := 0;
        variable row_dt : real;

        procedure read_data is
            file f : text open read_mode is g_data_file;
            variable ln : line;
            variable comma : character;
            variable fields : real_vector(0 to 7);
        begin
            readline(f, ln);
            while not endfile(f) and rows < max_rows loop
                readline(f, ln);
                for n in fields'range loop
                    read(ln, fields(n));
                    read(ln, comma);
                end loop;
                -- t, vref, vdc_out, vfilt_out, iac_out, vdc_mean, vfilt_mean, iac_mean
                t_data(rows)          := fields(0);
                vref_data(rows)       := fields(1);
                vfilt_data(rows)      := fields(3);
                vfilt_mean_data(rows) := fields(6);
                iac_mean_data(rows)   := fields(7);
                rows := rows + 1;
            end loop;
            row_dt := (t_data(rows-1) - t_data(0)) / real(rows-1);
        end procedure;

        -- a column at a time, the nearest row
        impure function row_at(time_s : real) return natural is
        begin
            return natural(realmax(0.0, realmin(real(rows-1), round((time_s - t_data(0)) / row_dt))));
        end function;

        file out_file : text;
        variable ln : line;
        variable step_time, du_sum, iload_sum, v_error_sum, v_noise_sum : real;
        variable counted, first_row, last_row : natural;

        -- the closed loop: the observer and the control as the functions, on the same inputs
        constant design : voltage_control_design_record :=
            voltage_control_design(output_period, 0.83e-3, 18.0e-6, 0.51, 3000.0, 1000.0, 5.0, 60.0, 0.95 * 94.0, 1.0, 20.0,
                                   voltage_zero => 0.1, prefilter_hz => 1000.0);
        variable max_u, i_peak : real := 0.0;
        variable periods_checked : natural := 0;

        procedure run_loop(periods : natural; d : voltage_control_design_record) is
            variable plant2 : real_vector(0 to 3) := (others => 0.0);
            variable t2 : real := 0.0;
            variable obs : observer_real_record := init_observer_real;
            variable ctl : voltage_control_record := voltage_control_start(0.0);
            variable u_applied, u_next : fixed_number := fixed_zero;
            variable i_sum_now, i_sum_before : real := 0.0;
            variable sum : real;
            variable count, readings : natural;
            variable reference_v : fixed_number;
            variable fixed : fixed_number;
            variable last : unsigned(31 downto 0);
        begin
            for h in 0 to 2 * periods - 1 loop
                if (natural(t2 / 5.0e-3) mod 2) = 0 then reference_v := to_fixed(-20.0); else reference_v := to_fixed(20.0); end if;
                u_bridge := realmax(-vdc, realmin(vdc, 1.03 * to_real(u_applied) - 4.2));
                i_sum_before := i_sum_now;
                i_sum_now := 0.0;
                sum := 0.0;
                count := 0;
                if h mod 3 = 0 then readings := 3; else readings := 2; end if;
                for n in 1 to 100 loop
                    rk4_2(t2, plant2, dt / 100.0);
                    t2 := t2 + dt / 100.0;
                    i_sum_now := i_sum_now + plant2(0);
                    if n mod (100 / readings) = 0 then
                        fixed := to_fixed(plant2(1) + 15.0 * gaussian);
                        give(voltage_channel, fixed);
                        sum   := sum + to_real(fixed);
                        count := count + 1;
                    end if;
                end loop;
                if (h + 1) mod 2 = 0 then
                    -- the period's start: the bridge applies the reference computed in the one before
                    u_applied := u_next;
                    wait until rising_edge(clock);
                    v_ref        <= reference_v;
                    period_start <= true;
                    wait until rising_edge(clock);
                    period_start <= false;
                end if;
                u <= u_applied;
                fixed := to_fixed((i_sum_now + i_sum_before) / 200.0);
                last := observer_steps;
                give(current_channel, fixed);
                -- the core's step done (on the board the steps are 12.5 us apart)
                wait until observer_steps /= last for 10 us;
                check(observer_steps /= last, "a step within 10 us");
                obs := observer_step(obs, k_default, to_real(u_applied), to_real(fixed), sum / real(count), true);
                if (h + 1) mod 2 = 0 then
                    if not ready then
                        wait until ready for 10 us;
                    end if;
                    check(ready, "the control's step within 10 us");
                    ctl := voltage_control_step(ctl, d, k_default, to_real(reference_v), obs.i, obs.v,
                                                obs.integral_i, -obs.integral_v, to_real(u_applied));
                    wait for 1 ns;
                    periods_checked := periods_checked + 1;
                    max_u := realmax(max_u, abs(to_real(u_ref) - ctl.u_ref));
                    u_next := u_ref;
                    if t2 > 10.0e-3 then
                        i_peak := realmax(i_peak, abs(plant2(0)));
                    end if;
                end if;
            end loop;
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            reference_k := k_default;
            if run("observer_on_the_plant") then
                bus_write(observer_base, to_signed(1, 32));
                bridge_on <= true;
                wait for 100 ns;
                for n in 0 to 6000 loop   -- 75 ms: 45 ms after the load step
                    if n < 1000 then u_now := -25.0; else u_now := 25.0; end if;
                    if n = 2400 then i_load_plant := 2.0; end if;
                    u_plant := 1.03 * u_now - 4.2;
                    i_sum_last := i_sum_this;
                    i_sum_this := 0.0;
                    if n mod 3 = 0 then readings_this_step := 3; else readings_this_step := 2; end if;
                    for m in 1 to substeps loop
                        rk4(t, plant, dt / real(substeps));
                        t := t + dt / real(substeps);
                        i_sum_this := i_sum_this + plant(0);
                        if m mod (substeps / readings_this_step) = 0 then
                            reading(plant(1) + 15.0 * gaussian);
                        end if;
                    end loop;
                    step(u_now, (i_sum_this + i_sum_last) / real(2 * substeps), burst => n = 500);
                end loop;
                report_and_check;
                info("du " & real'image(to_real(du)) & " V, iload " & real'image(to_real(iload)) & " A");
                check(abs(reference.integral_i - (0.03 * 25.0 - 4.2)) < 1.0, "du near the bridge's error");
                check(abs(-reference.integral_v - 2.0) < 0.3, "iload near the load");

            elsif run("observer_replays_the_measured_step") then
                read_data;
                bus_write(observer_base, to_signed(1, 32));
                bridge_on <= true;
                wait for 100 ns;
                -- the first step half a period from the window's start; the voltage readings as
                -- the raw vfilt_out column changes (a new ADC value every ~10 us); the current
                -- sampled at a step is the mean over the period before it: the centred mean half
                -- a period back; the bridge applies the reference a period after it changes
                file_open(out_file, g_out_file, write_mode);
                write(ln, string'("t,vfilt_mean,iac_mean,v_estimate,i_estimate,du,iload"));
                writeline(out_file, ln);
                step_time := t_data(0) + 2.0 * dt;
                first_row := row_at(step_time - dt);
                du_sum := 0.0; iload_sum := 0.0; v_error_sum := 0.0; v_noise_sum := 0.0; counted := 0;
                while step_time < t_data(rows-1) loop
                    last_row := row_at(step_time);
                    for n in first_row + 1 to last_row loop
                        if vfilt_data(n) /= vfilt_data(n-1) then
                            reading(vfilt_data(n));
                        end if;
                    end loop;
                    first_row := last_row;
                    step(vref_data(row_at(step_time - 2.0 * dt)), iac_mean_data(row_at(step_time - dt)));
                    write(ln, step_time, right, 0, 9); write(ln, string'(","));
                    write(ln, vfilt_mean_data(last_row), right, 0, 4); write(ln, string'(","));
                    write(ln, iac_mean_data(last_row), right, 0, 5); write(ln, string'(","));
                    write(ln, to_real(v_estimate), right, 0, 4); write(ln, string'(","));
                    write(ln, to_real(i_estimate), right, 0, 5); write(ln, string'(","));
                    write(ln, to_real(du), right, 0, 4); write(ln, string'(","));
                    write(ln, to_real(iload), right, 0, 5);
                    writeline(out_file, ln);
                    -- settled after the step: 10 .. 18 ms
                    if step_time > 10.0e-3 then
                        counted     := counted + 1;
                        du_sum      := du_sum + to_real(du);
                        iload_sum   := iload_sum + to_real(iload);
                        v_error_sum := v_error_sum + (to_real(v_estimate) - vfilt_mean_data(last_row))**2;
                        v_noise_sum := v_noise_sum + (vfilt_data(last_row) - vfilt_mean_data(last_row))**2;
                    end if;
                    step_time := step_time + dt;
                end loop;
                file_close(out_file);
                report_and_check;
                info("settled: du " & real'image(du_sum / real(counted)) & " V, iload " &
                     real'image(iload_sum / real(counted)) & " A; the voltage estimate " &
                     real'image(sqrt(v_error_sum / real(counted))) & " V rms from vfilt_out's period mean, " &
                     "the raw readings " & real'image(sqrt(v_noise_sum / real(counted))) & " V rms");
                -- the fit's u0 (-4.2 V) and gain (1.03 of +25 V)
                check(du_sum / real(counted) < -1.0 and du_sum / real(counted) > -6.0, "du: the bridge's offset");
                check(abs(iload_sum / real(counted)) < 0.5, "iload: no load");

            elsif run("observer_with_the_bridge_off") then
                bus_write(observer_base, to_signed(1, 32));
                bridge_on <= true;
                wait for 100 ns;
                for n in 0 to 4200 loop
                    if n = 1000 then
                        bridge_on <= false;
                    elsif n = 1001 then
                        -- the first step with the bridge off: du its integral from now on
                        du_at_stop := to_real(du);
                    elsif n = 3200 then
                        info("before the start: du " & real'image(to_real(du)) & " V (at the stop " &
                             real'image(du_at_stop) & "), current " & real'image(to_real(i_estimate)) &
                             " A, the voltage " & real'image(to_real(v_estimate)) & " V (the plant's " &
                             real'image(plant(1)) & " V)");
                        check(to_real(du) = du_at_stop, "du held while the bridge was off");
                        check(abs(to_real(i_estimate)) < 0.05, "no current while the bridge was off");
                        check(abs(to_real(v_estimate) - plant(1)) < 2.0, "the voltage followed while the bridge was off");
                        bridge_on <= true;
                        wait for 10 ns;
                    end if;
                    if bridge_on then
                        u_plant := 1.03 * 20.0 - 4.2;
                        i_load_plant := 0.0;
                    else
                        plant(0) := 0.0;
                        u_plant := plant(1);
                        i_load_plant := plant(1) / 1000.0;
                    end if;
                    i_sum_last := i_sum_this;
                    i_sum_this := 0.0;
                    if n mod 3 = 0 then readings_this_step := 3; else readings_this_step := 2; end if;
                    for m in 1 to substeps loop
                        rk4(t, plant, dt / real(substeps));
                        if not bridge_on then plant(0) := 0.0; end if;
                        t := t + dt / real(substeps);
                        i_sum_this := i_sum_this + plant(0);
                        if m mod (substeps / readings_this_step) = 0 then
                            reading(plant(1) + 15.0 * gaussian);
                        end if;
                    end loop;
                    if bridge_on then
                        step(20.0, (i_sum_this + i_sum_last) / real(2 * substeps));
                    else
                        step(0.0, 3.0 * gaussian);
                    end if;
                end loop;
                report_and_check;

            elsif run("observer_disabled_follows_the_readings") then
                for n in 1 to 100 loop
                    give(voltage_channel, to_fixed(50.0));
                    give(voltage_channel, to_fixed(50.0));
                    give(current_channel, to_fixed(3.0));
                    wait for 2 us;
                end loop;
                info("disabled: v " & real'image(to_real(v_estimate)) & " V, i " & real'image(to_real(i_estimate)) & " A");
                check(abs(to_real(v_estimate) - 50.0) < 0.01, "the voltage the readings' while disabled");
                check(i_estimate = fixed_zero, "no current while disabled");

            elsif run("closed_loop_square_wave") then
                bus_write(observer_base, to_signed(1, 32));
                bus_write(control_base, to_signed(1, 32));
                bridge_on   <= true;
                closed_loop <= true;
                run_loop(880, design);   -- 44 ms
                info(integer'image(periods_checked) & " periods; u_ref's largest difference from the functions' " &
                     real'image(max_u) & " V; peak current " & real'image(i_peak) & " A");
                check(max_u < 0.05, "u_ref the functions' within 50 mV");
                check(i_peak < 3.0, "the current as in the tuning (~2 A)");

            elsif run("closed_loop_one_ampere_limit") then
                bus_write(observer_base, to_signed(1, 32));
                bus_write(control_base, to_signed(1, 32));
                bus_write(control_base + 8, to_fixed(1.0));
                bridge_on   <= true;
                closed_loop <= true;
                run_loop(880, voltage_control_design(output_period, 0.83e-3, 18.0e-6, 0.51, 3000.0, 1000.0,
                                                     1.0, 60.0, 0.95 * 94.0, 1.0, 20.0, voltage_zero => 0.1,
                                                     prefilter_hz => 1000.0));
                info(integer'image(periods_checked) & " periods; u_ref's largest difference from the functions' " &
                     real'image(max_u) & " V; peak current " & real'image(i_peak) & " A");
                check(max_u < 0.05, "u_ref the functions' within 50 mV");
                check(i_peak < 1.3, "the current within the 1 A limit and a period's ripple");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 200 ms);

end vunit_simulation;
