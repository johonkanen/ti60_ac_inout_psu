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

-- input_boost_control on a boost converter's model, a period at a time: a DC source of 94.6 V
-- (the lab supply's 98.6 V less the diodes) through 15 uH (the inductance the polarity test's
-- current pulses suggest), the switch's on time as the DUT drives leg 2's low side, the inductor's
-- current rising while it is on and falling at (V - Vs) / L after, stopping at zero (DCM) or not;
-- the link a 1.3 mF capacitor (the polarity test's charging) with a load. The scaler's period
-- means of the input current and the link's voltage fed back each period, the input's voltage the
-- source's. Checked: it holds the link at its reference within the current and duty limits, only
-- leg 2's low side ever switching, its bridge's output current (iac_in * vac_in / vdc_in) the
-- load's; a load step; a load pulsing at 100 Hz (an AC output's power) hardly reaching the current
-- reference; the overvoltage stop, the watchdog, a trip and inhibit each stopping it.
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
    signal i_ref_probe, duty_probe, probe : scope_sample;

    -- the plant, as the main process steps it
    signal v_link   : real := 94.6;
    signal i_mean   : real := 0.0;
    signal i_load   : real := 0.05;     -- a bleed
    signal high_power : boolean := false;   -- the measurements 10 A, 400 V in, 420 V the link
    signal pulsing  : boolean := false; -- the load 0.3 * (1 - cos(2 pi 100 Hz t)) instead
    signal other_on : natural := 0;     -- clocks with any other switch on
    signal widest   : natural := 0;     -- the longest on time, clocks

begin

    clock <= not clock after 4 ns;

    dut : entity work.input_boost_control
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, scaled, scaled_ready, scaled_channel, trip, inhibit, running,
             leg1, leg2, i_ref_probe, duty_probe, probe);

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
                scaled(scaled_vdc_in) <= to_fixed(v);
                scaled(scaled_iac_in) <= to_fixed((area + charge) / t_period);
                scaled(scaled_vac_in) <= to_fixed(vs);
                if high_power then
                    scaled(scaled_vdc_in) <= to_fixed(420.0);
                    scaled(scaled_iac_in) <= to_fixed(10.0);
                    scaled(scaled_vac_in) <= to_fixed(400.0);
                end if;
            end if;
            -- the scaler's strobes: the link, then the current, a clock apart
            if count = 5 then
                scaled_ready <= true; scaled_channel <= scaled_vdc_in;
            elsif count = 6 then
                scaled_ready <= true; scaled_channel <= scaled_iac_in;
            elsif count = 7 then
                scaled_ready <= true; scaled_channel <= scaled_vac_in;
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
        variable status, i_bridge : integer;
        variable ref_low, ref_high : real;
        alias i_ref is << signal .input_boost_control_tb.dut.i_ref : signed(31 downto 0) >>;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("holds_the_link") then
                -- 94.6 V to 105 V: the current within 1 A, the duty within 0.3, only leg 2's low side
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(400 ms, v_low, v_high, i_high);
                info("after 400 ms: the link " & real'image(v_link) & " V, the input current " &
                     real'image(i_mean) & " A; its highest " & real'image(i_high) & " A, the link's " &
                     real'image(v_high) & " V; the widest pulse " & integer'image(widest) & " clocks");
                check(abs(v_link - 105.0) < 0.5, "the link at 105 V");
                check(v_high < 108.0, "overshoot under 3 V");
                check(i_high < 1.3, "the mean current within the 1 A limit (at the link: 1.1 A at the input)");
                check(widest <= 938, "the duty within 0.3 (938 clocks)");
                check_equal(other_on, 0, "only leg 2's low side switched");
                -- a load step: 0.5 A
                i_load <= 0.5;
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                info("the 0.5 A load: the link down to " & real'image(v_low) & " V, after 300 ms " &
                     real'image(v_link) & " V, the input current " & real'image(i_mean) & " A");
                check(v_low > 95.0, "the dip within 10 V");
                v_low := 200.0; v_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                check(abs(v_link - 105.0) < 0.5, "back at 105 V with the load");
                check(v_low > 104.0, "and steady");
                check(i_mean > 0.5, "the input current carrying the load");
                -- the bridge's output current as the controller has it: the load's, the input's
                -- current times 94.6 / 105
                bus_read(base + 17, i_bridge);
                info("the bridge's output current " & real'image(real(i_bridge) / 2.0**20) & " A, the input's " &
                     real'image(i_mean) & " A");
                check(abs(real(i_bridge) / 2.0**20 - 0.5) < 0.03, "the bridge's output current the load's");
                bus_write(base, to_signed(2, 32));
                wait for 50 us;
                check(not running and leg2 = dhb_leg_gates_off, "stopped");
                bus_read(base, status);
                check_equal(status, 0, "stopped by stop, nothing else");

            elsif run("a_load_pulsing_at_100_hz") then
                -- an AC output's power: 0.3 * (1 - cos(2 pi 100 Hz t)) A, 0 .. 0.6 A; the current
                -- reference's 100 Hz under a tenth of the load's
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(300 ms, v_low, v_high, i_high);
                pulsing <= true;
                run_for(500 ms, v_low, v_high, i_high);
                v_low := 200.0; v_high := 0.0; ref_low := 100.0; ref_high := -100.0;
                for k in 1 to 400 loop      -- 100 ms
                    bus_write(base + 9, to_signed(1, 32));
                    wait for 250 us;
                    v_low    := realmin(v_low, v_link);
                    v_high   := realmax(v_high, v_link);
                    ref_low  := realmin(ref_low, real(to_integer(i_ref)) / 2.0**20);
                    ref_high := realmax(ref_high, real(to_integer(i_ref)) / 2.0**20);
                end loop;
                info("the 100 Hz load: the link " & real'image(v_low) & " .. " & real'image(v_high) &
                     " V, the current reference " & real'image(ref_low) & " .. " & real'image(ref_high) & " A");
                check(abs((v_low + v_high) / 2.0 - 105.0) < 0.5, "the link about 105 V");
                check(ref_high - ref_low < 0.06, "the reference's ripple under a tenth of the load's 0.6 A");
                check(abs((ref_low + ref_high) / 2.0 - 0.3) < 0.05, "the reference the load's mean");

            elsif run("the_bridge_current_at_high_power") then
                -- 10 A from 400 V into 420 V: 9.52 A (current times voltage, 4000, would not fit
                -- the fixed point's 2048: the ratio first)
                high_power <= true;
                wait for 100 us;
                bus_read(base + 17, i_bridge);
                info("10 A, 400 V to 420 V: the bridge's output current " & real'image(real(i_bridge) / 2.0**20) & " A");
                check(abs(real(i_bridge) / 2.0**20 - 10.0 * 400.0 / 420.0) < 0.05, "10 * 400 / 420 A");

            elsif run("the_overvoltage_stop") then
                bus_write(base, to_signed(1, 32));
                v_low := 200.0; v_high := 0.0; i_high := 0.0;
                run_for(100 ms, v_low, v_high, i_high);
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

    test_runner_watchdog(runner, 5 sec);

end vunit_simulation;
