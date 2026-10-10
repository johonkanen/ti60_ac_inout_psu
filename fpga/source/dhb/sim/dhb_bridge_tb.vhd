LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.dhb_modulator_pkg.all;
    use work.dhb_bridge_pkg.all;

-- dhb_bridge's sequence at 200 MHz and 150 kHz (1333 counts), 100 ns (20 counts) of dead time
-- when running: the precharge against a simple model of the secondary DC link (a primary
-- switch on charges it towards the primary's 400 V), then both legs phase shifted; stop; and
-- a precharge that cannot charge.
entity dhb_bridge_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of dhb_bridge_tb is

    constant clock_period : time := 5 ns;
    constant period       : natural := 1333;
    constant half         : natural := period / 2;
    constant run_deadtime : natural := 20;

    signal clock : std_logic := '0';

    signal dhb_in  : dhb_input_record := dhb_input_init;
    signal dhb_out : dhb_output_record;

    -- the tests' names for the record fields
    alias start    is dhb_in.start;
    alias stop     is dhb_in.stop;
    alias phase    is dhb_in.phase;
    alias settings is dhb_in.settings;
    alias state    is dhb_out.state;
    -- both legs' gates, from the record's separate ones
    signal gates   : dhb_gates;

    constant primary_volts : real := 400.0;
    signal secondary_volts : real := 0.0;
    signal coupling        : real := 2.0e-4;

    signal both_on                 : natural := 0;   -- clocks with both switches of a leg on
    signal secondary_before_run    : natural := 0;   -- clocks with a secondary gate on before running

    function clocks(t : time) return integer is
    begin
        return t / clock_period;
    end function;

begin

    clock <= not clock after clock_period / 2;

    -- voltages in 0.1 V; the package's settings (150 kHz, 100 ns), the phase up to 600
    dhb_in.primary_voltage   <= integer(primary_volts * 10.0);
    dhb_in.secondary_voltage <= integer(secondary_volts * 10.0);

    u_bridge : entity work.dhb_bridge
    port map(clock, dhb_in, dhb_out);

    gates <= (dhb_out.primary_gates, dhb_out.secondary_gates);

    model_and_watch : process(clock)
    begin
        if rising_edge(clock) then
            if (gates(primary).high or gates(primary).low) = '1' and state = precharging then
                secondary_volts <= secondary_volts + coupling * (primary_volts - secondary_volts);
            end if;
            if (gates(primary).high and gates(primary).low) = '1' or
               (gates(secondary).high and gates(secondary).low) = '1' then
                both_on <= both_on + 1;
            end if;
            if (gates(secondary).high or gates(secondary).low) = '1' and state /= running then
                secondary_before_run <= secondary_before_run + 1;
            end if;
        end if;
    end process;

    main : process

        procedure pulse(signal strobe : out boolean) is
        begin
            strobe <= true;
            wait until rising_edge(clock);
            strobe <= false;
            wait until rising_edge(clock);
        end procedure;

        procedure measure(signal gate : std_logic; width, to_next : out integer) is
            variable rise, fall : time;
        begin
            wait until rising_edge(gate) for 50 us;
            check(gate = '1', "a gate pulse");
            rise := now;
            wait until falling_edge(gate);
            fall := now;
            wait until rising_edge(gate);
            width   := clocks(fall - rise);
            to_next := clocks(now - rise);
        end procedure;

        procedure gap(signal falling, rising : std_logic; clks : out integer) is
            variable fall : time;
        begin
            wait until falling_edge(falling);
            fall := now;
            wait until rising_edge(rising);
            clks := clocks(now - fall);
        end procedure;

        procedure shift(clks : out integer) is
            variable p : time;
        begin
            wait until rising_edge(gates(primary).high);
            p := now;
            wait until rising_edge(gates(secondary).high);
            clks := clocks(now - p);
        end procedure;

        variable width, to_next, clks : integer;

    begin
        test_runner_setup(runner, runner_cfg);
        settings.max_phase <= 600;
        phase              <= 150;

        while test_suite loop
            if run("precharge_then_both_bridges") then
                wait for 2 * period * clock_period;
                check(state = off and gates = dhb_gates_off, "off: the gates off");
                pulse(start);
                check(state = precharging, "precharging");
                wait until state /= precharging for 20 ms;
                check(state = running, "running after the precharge");
                info("precharged to " & real'image(secondary_volts) & " V");
                check(secondary_volts >= primary_volts - 5.0, "the secondary at the primary's voltage");
                check_equal(secondary_before_run, 0, "the secondary's gates off until running");

                -- both legs at half a period less 100 ns, the commanded phase shift
                wait for 3 * period * clock_period;
                measure(gates(primary).high, width, to_next);
                check_equal(to_next, period, "primary period");
                check_equal(width, half - run_deadtime, "primary high side");
                measure(gates(secondary).high, width, to_next);
                check_equal(to_next, period, "secondary period");
                check_equal(width, half - run_deadtime, "secondary high side");
                measure(gates(secondary).low, width, to_next);
                check_equal(width, period - half - run_deadtime, "secondary low side");
                gap(gates(primary).low, gates(primary).high, clks);
                check_equal(clks, run_deadtime, "primary dead time 100 ns");
                gap(gates(secondary).high, gates(secondary).low, clks);
                check_equal(clks, run_deadtime, "secondary dead time 100 ns");
                shift(clks);
                check_equal(clks, 150, "the commanded phase shift");

                -- the phase follows the command, reversed too
                phase <= -100;
                wait for 3 * period * clock_period;
                shift(clks);
                check_equal(clks, period - 100, "reversed: the secondary leads by 100");

                pulse(stop);
                wait for 1 ns;
                check(state = off, "stopped");
                wait until rising_edge(clock);
                wait for 1 ns;
                check(gates = dhb_gates_off, "all gates off a clock after stop");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("a_precharge_that_cannot_charge") then
                coupling <= 0.0;
                pulse(start);
                wait until state /= precharging for 20 ms;
                check(state = failed, "failed");
                wait for 2 * period * clock_period;
                check(gates = dhb_gates_off, "failed: the gates off");
                check_equal(secondary_before_run, 0, "the secondary never switched");
                -- a new start precharges again
                coupling <= 2.0e-4;
                pulse(start);
                check(state = precharging, "a new start from failed");
                wait until state /= precharging for 20 ms;
                check(state = running, "and runs");
                check_equal(both_on, 0, "clocks with both switches of a leg on");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 50 ms);

end vunit_simulation;
