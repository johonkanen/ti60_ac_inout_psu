LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;

-- dhb_voltage_controller through its registers, fed like measurement_scaler feeds it: updates
-- only on a new secondary voltage, the normalized command scaled to rounded counts (1.0 a
-- quarter period, 333.25 counts), the limits from the registers, 0 disabled, and the loop
-- closed over a simple model of the secondary DC link.
entity dhb_voltage_controller_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of dhb_voltage_controller_tb is

    constant base : natural := 1100;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal scaled         : fixed_array(0 to 18) := (others => fixed_zero);
    signal scaled_ready   : boolean := false;
    signal scaled_channel : natural range 0 to 18 := 0;
    signal phase          : dhb_phase;
    signal bridge_running : boolean := true;

begin

    clock <= not clock after 4 ns;

    dut : entity work.dhb_voltage_controller
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, scaled, scaled_ready, scaled_channel, bridge_running, phase);

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

        -- the scaler's output for one channel, as it comes
        procedure scaler(channel : natural; volts : real) is
        begin
            wait until rising_edge(clock);
            scaled(channel) <= to_fixed(volts);
            scaled_ready    <= true;
            scaled_channel  <= channel;
            wait until rising_edge(clock);
            scaled_ready    <= false;
            wait for 200 ns;   -- the controller's update
        end procedure;

        variable v, before : integer;
        variable secondary : real;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            -- the gains of pi_controller_tb's loop, 40 and 0.4 counts a volt, normalized to the
            -- 333.25 counts of 1.0; the limits and the scale as they start, +-1.0 and 333.25
            bus_write(base + 1, to_fixed(40.0 / 333.25));
            bus_write(base + 2, to_fixed(0.4 / 333.25));
            bus_write(base + 9, to_signed(0, 32));   -- unfiltered unless a test filters

            if run("disabled_holds_zero") then
                scaler(9, 95.0);
                scaler(0, 50.0);
                check_equal(phase, 0, "disabled: no phase");
                bus_read(base + 7, v);
                check_equal(v, 0, "no updates");

            elsif run("held_while_the_bridge_is_not_running") then
                -- enabled, but the bridge precharging: no updates, phase 0, nothing wound up
                bus_write(base, to_signed(1, 32));
                bridge_running <= false;
                scaler(9, 95.0);
                for k in 0 to 99 loop
                    scaler(0, 20.0);
                end loop;
                check_equal(phase, 0, "phase 0 while not running");
                bus_read(base + 7, v);
                check_equal(v, 0, "no updates");
                -- running: from phase 0, the first step kp * error + ki * error only
                bridge_running <= true;
                scaler(0, 94.9);
                -- 0.1 V of error: 4.04 counts, rounded to 4 (wound up, it would be 333)
                check_equal(phase, 4, "the first step, nothing wound up before it");

            elsif run("the_filter") then
                -- n 6: after 64 readings of a step 1 - 1/e of the way, ~63 %
                bus_write(base + 9, to_signed(6, 32));
                for k in 1 to 64 loop
                    scaler(0, 95.0);
                end loop;
                bus_read(base + 11, v);
                info("after 64 readings of 95 V: " & real'image(real(v) / 2.0**20) & " V");
                check(abs(real(v) / 2.0**20 - 95.0 * (1.0 - (63.0/64.0)**64)) < 0.01, "a first order filter, 64 readings");
                -- the board's +-30 V of switching noise around 95 V: the filtered value within a
                -- few volts
                for k in 1 to 600 loop
                    scaler(0, 95.0 + 30.0 * sin(real(k) * 2.1));
                end loop;
                secondary := 0.0;
                for k in 1 to 200 loop
                    scaler(0, 95.0 + 30.0 * sin(real(k) * 2.1));
                    bus_read(base + 11, v);
                    secondary := realmax(secondary, abs(real(v) / 2.0**20 - 95.0));
                end loop;
                info("+-30 V of noise, filtered: within " & real'image(secondary) & " V of 95 V");
                check(secondary < 3.0, "within 3 V");

            elsif run("updates_on_the_secondary_only") then
                bus_write(base, to_signed(1, 32));
                scaler(9, 95.0);
                bus_read(base + 7, before);
                scaler(9, 95.1);
                scaler(16, 1.0);
                bus_read(base + 7, v);
                check_equal(v, before, "the primary and other channels: no update");
                scaler(0, 94.9);
                bus_read(base + 7, v);
                check_equal(v, before + 1, "a new secondary value: an update");
                -- 0.2 V of error: (40 + 0.4) * 0.2 / 333.25 of command, 8.08 counts, rounded to 8
                check_equal(phase, 8, "kp * error + integrator, scaled and rounded");
                bus_read(base + 6, v);
                check_equal(v, phase, "the phase read back");
                bus_read(base + 8, v);
                check(abs(real(v) / 2.0**20 - 40.4 * 0.2 / 333.25) < 1.0e-4, "the command read back");

            elsif run("phase_within_the_limit") then
                bus_write(base, to_signed(1, 32));
                scaler(9, 95.0);
                scaler(0, 0.0);
                check_equal(phase, 333, "1.0: a quarter period, 333.25 rounded");
                scaler(0, 200.0);
                check_equal(phase, -333, "-1.0");
                -- limits of 0 .. 0.5 from the registers
                bus_write(base + 3, to_fixed(0.5));
                bus_write(base + 4, to_fixed(0.0));
                scaler(0, 200.0);
                check_equal(phase, 0, "held at the lower limit 0");
                scaler(0, 0.0);
                check_equal(phase, 167, "the upper limit 0.5: 166.625 rounded");
                bus_write(base, to_signed(0, 32));
                wait for 100 ns;
                check_equal(phase, 0, "0 once disabled");

            elsif run("closes_the_loop_through_the_noise") then
                -- the DC link model with +-30 V of noise on its reading, filtered with n 6; the
                -- filter's lag (64 readings) is as long as the model's loop at the gains above,
                -- so slower ones: kp 10 counts a volt (a loop of 200 readings) and ki 0.02 (with it
                -- a damping of ~0.8)
                bus_write(base + 9, to_signed(6, 32));
                bus_write(base + 1, to_fixed(10.0 / 333.25));
                bus_write(base + 2, to_fixed(0.02 / 333.25));
                bus_write(base, to_signed(1, 32));
                -- the reference through its filter too: enough readings for it to settle
                for k in 1 to 1000 loop
                    scaler(9, 95.0);
                end loop;
                secondary := 60.0;
                for k in 0 to 3999 loop
                    scaler(0, secondary + 30.0 * sin(real(k) * 2.1));
                    secondary := secondary + 10.0e-6 / 400.0e-6 * (0.02 * real(phase) - secondary / 1000.0);
                    if k mod 500 = 499 then
                        info("step " & integer'image(k + 1) & ": " & real'image(secondary) & " V, phase " & integer'image(phase));
                    end if;
                end loop;
                info("through the noise: the secondary at " & real'image(secondary) & " V");
                check(abs(secondary - 95.0) < 1.0, "at the primary's voltage within a volt");

            elsif run("closes_the_dc_link_loop") then
                bus_write(base, to_signed(1, 32));
                scaler(9, 95.0);
                -- the secondary DC link model of pi_controller_tb, an update every 10 us
                secondary := 60.0;
                for k in 0 to 1999 loop
                    scaler(0, secondary);
                    secondary := secondary + 10.0e-6 / 400.0e-6 * (0.02 * real(phase) - secondary / 1000.0);
                end loop;
                info("the secondary at " & real'image(secondary) & " V, the phase " & integer'image(phase) & " counts");
                -- whole counts of phase: within a count's worth of volts
                check(abs(secondary - 95.0) < 0.5, "at the primary's voltage");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 100 ms);

end vunit_simulation;
