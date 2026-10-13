LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.scaler_channels_pkg.all;
    use work.scope_pkg.all;

-- input_boost_control with its voltage control on output_control_processor's core (the core
-- shared: the output's observer enabled, stepping at an output current's strobe twice a period)
-- on a boost converter's model, a period at a time: a DC source of 94.6 V (the lab supply's
-- 98.6 V less the diodes) through 15 uH (the inductance the polarity test's current pulses
-- suggest), the switch's on time as the DUT drives leg 2's low side, the inductor's current rising
-- while it is on and falling at (V - Vs) / L after, stopping at zero (DCM) or not; the link a
-- 1.3 mF capacitor (the polarity test's charging) with a load. The link's voltage read three
-- times a period, with noise in noisy_readings. Checked: it holds the link at its reference
-- within the duty's limit, only leg 2's low side ever switching; a load step; a load pulsing at
-- 100 Hz (an AC output's power); +-3 V of noise on the readings; the overvoltage stop, the
-- watchdog, a trip and inhibit each stopping it.
entity input_boost_control_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of input_boost_control_tb is

    constant base : natural := 1260;
    constant t_period : real := 25.0e-6;
    constant clocks   : natural := 3125;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;
    signal scaled : fixed_array(0 to scaler_channels - 1) := (others => fixed_zero);
    signal scaled_ready : boolean := false;
    signal scaled_channel : natural range 0 to scaler_channels - 1 := 0;
    signal trip, inhibit : boolean := false;
    signal running : boolean;
    signal leg1, leg2 : dhb_leg_gates;
    signal probe : scope_sample;

    -- the boost's link to the core, the core's output side tied off
    signal b_request, b_reset, b_ready : boolean := false;
    signal b_v_sum, b_v_ref, b_ramp, b_d_half : fixed_number := fixed_zero;
    signal b_count : natural range 0 to 8 := 0;
    signal b_kp, b_ki, b_alpha : signed(31 downto 0) := (others => '0');
    signal b_duty, b_on_half, b_target, b_v : fixed_number;
    signal bus_from_boost, bus_from_core : fpga_interconnect_record := init_fpga_interconnect;
    signal u_ref : fixed_number;
    signal control_ready : boolean;
    signal observer_probes : scope_sample_array(0 to 3);
    signal control_probes  : scope_sample_array(0 to 1);
    signal noisy : boolean := false;    -- +-3 V on each reading of the link

    -- the plant, as the main process steps it
    signal v_link   : real := 94.6;
    signal i_mean   : real := 0.0;
    signal i_load   : real := 0.05;     -- a bleed
    signal pulsing  : boolean := false; -- the load 0.3 * (1 - cos(2 pi 100 Hz t)) instead
    signal other_on : natural := 0;     -- clocks with any other switch on
    signal widest   : natural := 0;     -- the longest on time, clocks

begin

    clock <= not clock after 4 ns;

    dut : entity work.input_boost_control
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_from_boost, scaled, scaled_ready, scaled_channel, trip, inhibit, running,
             leg1, leg2, b_request, b_reset, b_v_sum, b_count, b_v_ref, b_ramp, b_d_half, b_kp, b_ki, b_alpha,
             b_ready, b_duty, b_on_half, b_target, b_v, probe);

    core : entity work.output_control_processor
    generic map(g_observer_address => 1160, g_control_address => 1180,
                g_current_channel => scaled_iac_out_sampled, g_voltage_channel => scaled_vfilt_out)
    port map(clock, bus_in, bus_from_core, scaled, scaled_ready, scaled_channel,
             run => false, u => fixed_zero, closed_loop => false, v_ref => fixed_zero, period_start => false,
             u_ref => u_ref, ready => control_ready, observer_probes => observer_probes,
             control_probes => control_probes,
             boost_request => b_request, boost_reset => b_reset, boost_v_sum => b_v_sum,
             boost_count => b_count, boost_v_ref => b_v_ref, boost_ramp => b_ramp,
             boost_d_half => b_d_half, boost_kp => b_kp, boost_ki => b_ki, boost_alpha => b_alpha,
             boost_ready => b_ready, boost_duty => b_duty, boost_on_half => b_on_half,
             boost_target => b_target, boost_v => b_v);

    bus_out <= bus_from_boost when bus_from_boost /= init_fpga_interconnect else bus_from_core;

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

    -- the plant: each period's on time from the gate, then the period's currents and charge
    plant : process(clock)
        alias count is << signal .input_boost_control_tb.dut.count : natural >>;
        constant vs  : real := 94.6;
        constant l   : real := 15.0e-6;
        constant c   : real := 1.3e-3;
        variable on_clocks : natural := 0;
        variable i0, i1, i2, t_on, t_off, t2, area, charge : real := 0.0;
        variable v : real := 94.6;
        variable seed1, seed2 : positive := 7;
        variable x : real;
        procedure reading is
        begin
            uniform(seed1, seed2, x);
            if noisy then
                scaled(scaled_vdc_in) <= to_fixed(v_link + 6.0 * (x - 0.5));
            else
                scaled(scaled_vdc_in) <= to_fixed(v_link);
            end if;
            scaled_ready <= true; scaled_channel <= scaled_vdc_in;
        end procedure;
    begin
        if rising_edge(clock) then
            scaled_ready <= false;
            if leg1.high = '1' or leg1.low = '1' or leg2.high = '1' then
                other_on <= other_on + 1;
            end if;
            if leg2.low = '1' then
                on_clocks := on_clocks + 1;
            end if;
            if count = 3 then
                -- the period before: its on time
                if on_clocks > widest then widest <= on_clocks; end if;
                t_on  := real(on_clocks) * 8.0e-9;
                t_off := t_period - t_on;
                on_clocks := 0;
                v := v_link;
                i1 := i0 + vs * t_on / l;
                area := (i0 + i1) / 2.0 * t_on;
                if v > vs then
                    i2 := i1 - (v - vs) * t_off / l;
                    if i2 <= 0.0 then
                        t2 := i1 * l / (v - vs);
                        charge := i1 * t2 / 2.0;
                        i2 := 0.0;
                    else
                        charge := (i1 + i2) / 2.0 * t_off;
                    end if;
                else
                    i2 := i1;
                    charge := i1 * t_off;
                end if;
                i0 := i2;
                if pulsing then
                    v := v + (charge - 0.3 * (1.0 - cos(math_2_pi * 100.0 * real(now / 1 ns) * 1.0e-9)) * t_period) / c;
                else
                    v := v + (charge - i_load * t_period) / c;
                end if;
                -- below the source the diodes conduct: the link at least the source's
                if v < vs then v := vs; end if;
                v_link <= v;
                i_mean <= (area + charge) / t_period;
                scaled(scaled_iac_in) <= to_fixed((area + charge) / t_period);
                scaled(scaled_vac_in) <= to_fixed(vs);
            end if;
            -- the scaler's strobes: the link three times a period, the input's current and
            -- voltage; the output's current twice (the observer's steps on the shared core)
            if count = 5 or count = 1046 or count = 2087 then
                reading;
            elsif count = 6 then
                scaled_ready <= true; scaled_channel <= scaled_iac_in;
            elsif count = 7 then
                scaled_ready <= true; scaled_channel <= scaled_vac_in;
            elsif count = 1500 or count = 3060 then
                scaled_ready <= true; scaled_channel <= scaled_iac_out_sampled;
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

        -- run for a time, kicking the watchdog every 100 ms; the voltage's and current's extremes
        procedure run_for(t : time; v_low, v_high, i_high : inout real) is
            constant until_time : time := now + t;
            variable next_kick : time := now;
        begin
            while now < until_time loop
                if now >= next_kick then
                    bus_write(base + 9, to_signed(1, 32));
                    next_kick := now + 100 ms;
                end if;
                wait for 25 us;
                v_low  := realmin(v_low, v_link);
                v_high := realmax(v_high, v_link);
                i_high := realmax(i_high, i_mean);
            end loop;
        end procedure;

        variable v_low, v_high, i_high : real;
        variable status : integer;
        variable duty_low, duty_high, duty : real;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;
        -- the output's observer on: its steps share the core
        bus_write(1160, to_signed(1, 32));

        while test_suite loop
            if run("holds_the_link") then
                -- 94.6 V to 105 V: the duty within 0.05 (156 clocks), only leg 2's low side
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(800 ms, v_low, v_high, i_high);
                info("after 800 ms (the prefilter: the start ~0.5 s): the link " & real'image(v_link) & " V, the input current " &
                     real'image(i_mean) & " A; its highest " & real'image(i_high) & " A, the link's " &
                     real'image(v_high) & " V; the widest pulse " & integer'image(widest) & " clocks");
                check(abs(v_link - 105.0) < 0.3, "the link at 105 V");
                check(v_high < 106.0, "overshoot under 1 V");
                check(widest <= 157, "the duty within 0.05 (156 clocks)");
                check_equal(other_on, 0, "only leg 2's low side switched");
                -- a load step: 0.5 A
                i_load <= 0.5;
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                info("the 0.5 A load: the link down to " & real'image(v_low) & " V, after 300 ms " &
                     real'image(v_link) & " V, the input current " & real'image(i_mean) & " A");
                check(v_low > 100.0, "the dip within 5 V");
                v_low := 200.0; v_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                check(abs(v_link - 105.0) < 0.3, "back at 105 V with the load");
                check(v_low > 104.5, "and steady");
                check(i_mean > 0.5, "the input current carrying the load");
                -- the registers: the reference reached, the link's mean, the duty
                bus_read(base + 11, status);
                check(abs(real(status) / 2.0**20 - 105.0) < 0.01, "the reference ramped to 105 V");
                bus_read(base + 17, status);
                check(abs(real(status) / 2.0**20 - v_link) < 0.1, "the link's mean as the plant's");
                bus_read(base + 12, status);
                info("the duty with the 0.5 A load " & real'image(real(status) / 2.0**20));
                check(real(status) / 2.0**20 > 0.01 and real(status) / 2.0**20 < 0.05, "the duty in its range");
                bus_write(base, to_signed(2, 32));
                wait for 50 us;
                check(not running and leg2 = dhb_leg_gates_off, "stopped");
                bus_read(base, status);
                check_equal(status, 0, "stopped by stop, nothing else");

            elsif run("a_load_pulsing_at_100_hz") then
                -- an AC output's power: 0.3 * (1 - cos(2 pi 100 Hz t)) A, 0 .. 0.6 A
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                pulsing <= true;
                run_for(500 ms, v_low, v_high, i_high);
                v_low := 200.0; v_high := 0.0; duty_low := 1.0; duty_high := -1.0;
                for k in 1 to 400 loop      -- 100 ms
                    bus_write(base + 9, to_signed(1, 32));
                    wait for 250 us;
                    v_low    := realmin(v_low, v_link);
                    v_high   := realmax(v_high, v_link);
                    duty     := real(to_integer(b_duty)) / 2.0**20;
                    duty_low := realmin(duty_low, duty);
                    duty_high := realmax(duty_high, duty);
                end loop;
                info("the 100 Hz load: the link " & real'image(v_low) & " .. " & real'image(v_high) &
                     " V, the duty " & real'image(duty_low) & " .. " & real'image(duty_high));
                check(abs((v_low + v_high) / 2.0 - 105.0) < 0.5, "the link about 105 V");
                check(v_high - v_low < 2.0, "its ripple under 2 V");
                check(duty_high < 0.05, "the duty under its limit");

            elsif run("noisy_readings") then
                -- +-3 V on each reading: the period's mean and the slow loop average it out
                noisy <= true;
                i_load <= 0.3;
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(800 ms, v_low, v_high, i_high);
                v_low := 200.0; v_high := 0.0;
                run_for(200 ms, v_low, v_high, i_high);
                info("noisy readings: the link " & real'image(v_low) & " .. " & real'image(v_high) & " V");
                check(v_low > 104.5 and v_high < 105.5, "the link within 0.5 V of 105 V");

            elsif run("a_reference_step_without_the_prefilter_overshoots")
                  or run("a_reference_step_with_the_prefilter") then
                -- the board's loop: its gain a third of the model's, so kp and ki a third of the
                -- defaults here (the prefilter's alpha, ki / kp, the same); 105 V, then 112 V
                bus_write(base + 2, to_signed(integer(0.005 / 3.0 * 2.0**24), 32));
                bus_write(base + 3, to_signed(integer(1.5e-6 / 3.0 * 2.0**24), 32));
                if running_test_case = "a_reference_step_without_the_prefilter_overshoots" then
                    bus_write(base + 19, to_signed(2**24, 32));     -- 1.0: none
                end if;
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(800 ms, v_low, v_high, i_high);
                info("the start: the link's highest " & real'image(v_high) & " V, now " & real'image(v_link) & " V");
                bus_write(base + 1, to_fixed(112.0));
                v_low := 200.0; v_high := 0.0;
                run_for(1500 ms, v_low, v_high, i_high);
                info("105 V -> 112 V: the link's highest " & real'image(v_high) & " V, after 1.5 s " &
                     real'image(v_link) & " V");
                if running_test_case = "a_reference_step_without_the_prefilter_overshoots" then
                    check(v_high > 112.5, "without the prefilter the step overshoots (the test sees it)");
                else
                    check(v_high < 112.2, "with the prefilter no overshoot (within 0.2 V)");
                    check(abs(v_link - 112.0) < 0.2, "at 112 V");
                end if;

            elsif run("the_duty_at_most_0_2") then
                -- d_max written 0.5, the reference out of reach: the duty at its ceiling, 0.2
                bus_write(base + 7, to_fixed(0.5));
                bus_write(base + 1, to_fixed(200.0));
                bus_write(base + 8, to_fixed(300.0));
                bus_write(base + 16, to_fixed(0.0));
                bus_write(base + 19, to_signed(2**24, 32));     -- no prefilter: at once
                i_load <= 3.0;
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(100 ms, v_low, v_high, i_high);
                info("d_max 0.5 written: the widest pulse " & integer'image(widest) & " clocks");
                check(widest <= 625, "the duty at most 0.2 (625 clocks)");
                check(widest >= 600, "and at it");

            elsif run("the_overvoltage_stop") then
                -- the readings' noise (+-3 V) over the stop now and then: no stop
                noisy <= true;
                bus_write(base + 8, to_fixed(107.0));
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(150 ms, v_low, v_high, i_high);
                check(running, "single readings over the stop do not stop it");
                bus_write(base + 8, to_fixed(98.0));
                wait for 200 us;
                check(not running, "stopped over the stop");
                bus_read(base, status);
                check_equal(status, 2, "stopped by the overvoltage");

            elsif run("the_watchdog") then
                bus_write(base + 10, to_signed(2, 32));     -- 2 ms
                bus_write(base, to_signed(1, 32));
                for k in 1 to 5 loop
                    wait for 1 ms;
                    bus_write(base + 9, to_signed(1, 32));
                end loop;
                check(running, "kicked: running");
                wait for 2.2 ms;
                check(not running, "not kicked for 2 ms: stopped");
                bus_read(base, status);
                check_equal(status, 8, "stopped by the watchdog");

            elsif run("a_trip_and_inhibit") then
                bus_write(base, to_signed(1, 32));
                wait for 2 ms;
                trip <= true;
                wait for 30 ns;
                check(not running and leg2 = dhb_leg_gates_off, "a trip: off at once");
                trip <= false;
                bus_read(base, status);
                check_equal(status, 4, "stopped by a trip");
                bus_write(base, to_signed(1, 32));
                wait for 2 ms;
                inhibit <= true;
                wait for 30 ns;
                check(not running, "inhibit: off");
                bus_read(base, status);
                check_equal(status, 16, "stopped by inhibit");
                bus_write(base, to_signed(1, 32));
                wait for 100 ns;
                check(not running, "no start while inhibited");
                inhibit <= false;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 sec);

end vunit_simulation;
