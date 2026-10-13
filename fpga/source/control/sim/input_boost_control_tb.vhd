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
    use work.input_boost_model_pkg.all;
    use work.input_boost_plant_pkg.all;

-- input_boost_control's RTL with its voltage control on output_control_processor's core (the
-- core shared: the output's observer enabled, stepping at an output current's strobe twice a
-- period) on the converter of input_boost_plant_pkg (hVHDL_ode's rk4, a period at a time): a DC
-- source of 94.6 V through 15 uH, its positive terminal on leg 2 (on leg 1 in a_reversed_input,
-- the input's voltage then read negative), the on time as the DUT drives that leg's low side;
-- the link 1.3 mF with a load. The link read three times a period, with noise where a test
-- asks.
--
-- The loop's dynamics over seconds are input_boost_model_tb's, in real numbers. Here, in
-- milliseconds, every step the core makes is checked against input_boost_model_pkg's step fed
-- the same inputs (the readings' sum and count, the reference, the ramp, d_max, the gains, the
-- prefilter): the duty, the reference as ramped and the link's mean, within the fixed point's
-- rounding. And what only the RTL has: the gate (only the low side of the positive terminal's
-- leg, either way round), the registers, the duty's ceiling, every stop.
--
--   the_rtl_steps_as_the_model  a start ramping into the duty's limit, a reference step through
--                               the prefilter
--   a_reversed_input            leg 1's low side only, the status's leg bit
--   the_duty_at_most_0_2        d_max written 0.5: the duty at 0.2
--   the_overvoltage_stop        +-3 V of noise, single readings over the stop: no stop; under it: stopped
--   the_watchdog, a_trip_and_inhibit   each stops it at once
--   the_sequencers_start_and_at_reference   start_in starts it; at_reference
--   the_registers_defaults      the gains, the prefilter and the limits as reset
entity input_boost_control_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of input_boost_control_tb is

    constant base : natural := 1260;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;
    signal scaled : fixed_array(0 to scaler_channels - 1) := (others => fixed_zero);
    signal scaled_ready : boolean := false;
    signal scaled_channel : natural range 0 to scaler_channels - 1 := 0;
    signal trip, inhibit : boolean := false;
    signal running : boolean;
    signal start_in : boolean := false;     -- supply_sequencer's start
    signal at_reference : boolean;
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

    -- the plant
    signal v_start  : real := 94.6;     -- the link at the plant's (re)start
    signal restart  : boolean := false;
    signal v_link   : real := 94.6;
    signal noisy    : boolean := false;
    signal noise    : real := 3.0;      -- up to +-this on each reading, V
    signal reversed_source : boolean := false;  -- the source's positive terminal on leg 1
    signal other_on : natural := 0;     -- clocks with any other switch on
    signal widest   : natural := 0;     -- the longest on time, clocks

    -- the core's steps against the model's
    signal steps_compared : natural := 0;
    signal duty_error, target_error, v_error : real := 0.0;   -- the largest

begin

    clock <= not clock after 4 ns;

    dut : entity work.input_boost_control
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_from_boost, scaled, scaled_ready, scaled_channel, trip, inhibit, running,
             start_in, at_reference, leg1, leg2, b_request, b_reset, b_v_sum, b_count, b_v_ref, b_ramp, b_d_half, b_kp, b_ki, b_alpha,
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

    -- the plant: each period's on time from the gate, the period integrated at the next's start
    plant : process(clock)
        alias count is << signal .input_boost_control_tb.dut.count : natural >>;
        variable converter : boost_plant := boost_plant_init;
        variable on_clocks : natural := 0;
        variable seed1, seed2 : positive := 7;
        variable x : real;
        procedure reading is
        begin
            uniform(seed1, seed2, x);
            if noisy then
                scaled(scaled_vdc_in) <= to_fixed(converter.v + 2.0 * noise * (x - 0.5));
            else
                scaled(scaled_vdc_in) <= to_fixed(converter.v);
            end if;
            scaled_ready <= true; scaled_channel <= scaled_vdc_in;
        end procedure;
    begin
        if rising_edge(clock) then
            scaled_ready <= false;
            if restart then
                converter := boost_plant_init;
                converter.v := v_start;
            end if;
            -- the switch: the low side of the positive terminal's leg; any other one is counted
            if reversed_source then
                if leg2.high = '1' or leg2.low = '1' or leg1.high = '1' then
                    other_on <= other_on + 1;
                end if;
                if leg1.low = '1' then
                    on_clocks := on_clocks + 1;
                end if;
            else
                if leg1.high = '1' or leg1.low = '1' or leg2.high = '1' then
                    other_on <= other_on + 1;
                end if;
                if leg2.low = '1' then
                    on_clocks := on_clocks + 1;
                end if;
            end if;
            if count = 3 then
                -- the period before: its on time
                if on_clocks > widest then widest <= on_clocks; end if;
                boost_plant_period(converter, real(on_clocks) * 8.0e-9, 0.05);
                on_clocks := 0;
                v_link <= converter.v;
                scaled(scaled_iac_in) <= to_fixed(converter.i_mean);
                if reversed_source then
                    scaled(scaled_vac_in) <= to_fixed(-converter.vs);
                else
                    scaled(scaled_vac_in) <= to_fixed(converter.vs);
                end if;
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

    -- every step of the core against the model's from the same inputs
    against_the_model : process(clock)
        variable law : boost_law_state := boost_law_zero;
        variable settings : boost_law_settings;
        variable expected : boost_law_state;
        variable pending : boolean := false;
    begin
        if rising_edge(clock) then
            if b_request then
                settings := (v_ref => to_real(b_v_ref), ramp => to_real(b_ramp),
                             d_max => 2.0 * to_real(b_d_half),
                             kp => real(to_integer(b_kp)) / 2.0**24, ki => real(to_integer(b_ki)) / 2.0**24,
                             alpha => real(to_integer(b_alpha)) / 2.0**24);
                if b_reset then
                    boost_law_reset(law, b_count > 0, to_real(b_v_sum) / real(maximum(b_count, 1)));
                else
                    boost_law_step(law, b_count > 0, to_real(b_v_sum) / real(maximum(b_count, 1)), settings);
                end if;
                expected := law;
                pending := true;
            end if;
            if b_ready and pending then
                pending := false;
                duty_error   <= realmax(duty_error, abs(to_real(b_duty) - expected.duty));
                target_error <= realmax(target_error, abs(to_real(b_target) - expected.target));
                v_error      <= realmax(v_error, abs(to_real(b_v) - expected.v));
                steps_compared <= steps_compared + 1;
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

        -- the core's steps so far as the model's
        procedure check_steps(least : natural) is
        begin
            info(integer'image(steps_compared) & " steps as the model's: the duty within " &
                 real'image(duty_error) & ", the reference " & real'image(target_error) & " V, the link's mean " &
                 real'image(v_error) & " V");
            check(steps_compared >= least, "the core stepped");
            check(duty_error < 1.0e-5, "the duty as the model's");
            check(target_error < 1.0e-4, "the reference as the model's");
            check(v_error < 1.0e-4, "the link's mean as the model's");
        end procedure;

        variable status : integer;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;
        -- the output's observer on: its steps share the core
        bus_write(1160, to_signed(1, 32));

        while test_suite loop
            if run("the_rtl_steps_as_the_model") then
                -- the start without the prefilter, a fast ramp (10 V/ms) charging the link faster
                -- than the duty's limit allows (0.05: the anti-windup); then the prefilter as by
                -- default and the reference stepped from 105 V to 108 V; every step checked
                bus_write(base + 19, to_signed(2**24, 32));
                bus_write(base + 16, to_fixed(10.0));
                bus_write(base, to_signed(1, 32));
                wait for 8 ms;
                -- written after a step's result: the core takes its inputs some clocks after the
                -- request (the observer first), a write between would be taken a period earlier
                -- than the model's, which takes them at the request
                wait until rising_edge(clock) and b_ready;
                bus_write(base + 19, to_signed(integer(1.16e-5 / 0.022 * 2.0**24), 32));
                bus_write(base + 1, to_fixed(108.0));
                wait for 7 ms;
                check_steps(550);
                check(widest <= 157, "the duty within 0.05 (156 clocks)");
                check(widest >= 150, "and at it on the way");
                check_equal(other_on, 0, "only leg 2's low side switched");
                -- the registers as the core's results
                bus_read(base + 11, status);
                check(abs(real(status) / 2.0**20 - to_real(b_target)) < 1.0e-5, "the reference as ramped");
                bus_read(base + 12, status);
                check(abs(real(status) / 2.0**20 - to_real(b_duty)) < 1.0e-5, "the duty");
                bus_read(base + 17, status);
                check(abs(real(status) / 2.0**20 - to_real(b_v)) < 1.0e-5, "the link's mean");
                bus_write(base, to_signed(2, 32));
                wait for 50 us;
                check(not running and leg2 = dhb_leg_gates_off, "stopped");
                bus_read(base, status);
                check_equal(status, 0, "stopped by stop, nothing else");

            elsif run("a_reversed_input") then
                -- the source's leads the other way round: its voltage read negative, so leg 1's
                -- low side switches and nothing else
                reversed_source <= true;
                wait for 100 us;
                bus_write(base + 16, to_fixed(10.0));
                bus_write(base, to_signed(1, 32));
                wait for 5 ms;
                check_steps(190);
                check(widest > 0, "leg 1's low side switched");
                check_equal(other_on, 0, "only leg 1's low side switched");
                bus_read(base, status);
                check_equal(status, 16#21#, "running, on leg 1");
                bus_write(base, to_signed(2, 32));
                wait for 50 us;
                check(not running and leg1 = dhb_leg_gates_off and leg2 = dhb_leg_gates_off, "stopped");
                bus_read(base, status);
                check_equal(status, 16#20#, "stopped by stop, leg 1 noted");

            elsif run("the_duty_at_most_0_2") then
                -- d_max written 0.5, the reference out of reach: the duty at its ceiling, 0.2
                bus_write(base + 7, to_fixed(0.5));
                bus_write(base + 1, to_fixed(200.0));
                bus_write(base + 8, to_fixed(300.0));
                bus_write(base + 16, to_fixed(0.0));
                bus_write(base + 19, to_signed(2**24, 32));     -- no prefilter: at once
                bus_write(base, to_signed(1, 32));
                wait for 2 ms;
                info("d_max 0.5 written: the widest pulse " & integer'image(widest) & " clocks");
                check(widest <= 625, "the duty at most 0.2 (625 clocks)");
                check(widest >= 600, "and at it");
                check_steps(70);

            elsif run("the_overvoltage_stop") then
                -- the link at 105 V, its readings' noise (+-3 V) over the 107 V stop now and then
                -- (a reading in six): no stop
                v_start <= 105.0;
                restart <= true;
                wait until rising_edge(clock);
                restart <= false;
                noisy <= true;
                bus_write(base + 8, to_fixed(107.0));
                bus_write(base, to_signed(1, 32));
                wait for 10 ms;
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

            elsif run("the_sequencers_start_and_at_reference") then
                -- the link near the reference: started by start_in (no register), at the reference
                -- once the reference has ramped (0.5 V/ms) and the link's mean is within 1 V; a new
                -- reference away from it: no longer
                v_start <= 104.6;
                restart <= true;
                wait until rising_edge(clock);
                restart <= false;
                wait for 60 us;      -- a whole period's readings of it
                check(not at_reference, "stopped: not at the reference");
                start_in <= true;
                wait until rising_edge(clock);
                start_in <= false;
                wait for 3 ms;
                check(running, "started by start_in");
                bus_read(base + 11, status);
                info("the reference as ramped " & real'image(real(status) / 2.0**20));
                bus_read(base + 17, status);
                info("the link's mean " & real'image(real(status) / 2.0**20));
                check(at_reference, "at the reference");
                bus_write(base + 1, to_fixed(110.0));
                wait for 200 us;
                check(not at_reference, "a reference 5 V away: not at it");
                check_steps(110);

            elsif run("the_registers_defaults") then
                bus_read(base + 1, status);
                check_equal(status, integer(105.0 * 2.0**20), "v_ref 105 V");
                bus_read(base + 2, status);
                check_equal(status, integer(0.022 * 2.0**24), "kp 0.022");
                bus_read(base + 3, status);
                check_equal(status, integer(1.16e-5 * 2.0**24), "ki 1.16e-5");
                bus_read(base + 19, status);
                check_equal(status, integer(1.16e-5 / 0.022 * 2.0**24), "the prefilter at ki / kp");
                bus_read(base + 7, status);
                check_equal(status, integer(0.05 * 2.0**20), "d_max 0.05");
                bus_read(base + 8, status);
                check_equal(status, integer(140.0 * 2.0**20), "the overvoltage stop 140 V");
                bus_read(base + 16, status);
                check_equal(status, integer(0.5 * 2.0**20), "the ramp 0.5 V/ms");
                bus_read(base + 10, status);
                check_equal(status, 1000, "the watchdog 1000 ms");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

end vunit_simulation;
