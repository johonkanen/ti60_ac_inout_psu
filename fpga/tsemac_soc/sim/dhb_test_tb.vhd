LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.dhb_modulator_pkg.all;

-- dhb_test through its registers: the default burst (20 ms of 200 ns pulses at 150 kHz on the
-- primary alone), a shorter one, the pulse width limit, stop in the middle of a burst, and the
-- ramp: 50 ms of the 500 ms one from 200 ns pulses towards nominal, and its speed limit; and a
-- burst that ends when the secondary DC link (a simple model) is at the primary's voltage; and
-- with a run time, the bridge running after the precharge on the given phase, both legs, the
-- secondary inverted too; held on past its run time while hold is high.
entity dhb_test_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of dhb_test_tb is

    constant base : natural := 780;

    signal clock     : std_logic := '0';   -- the register bus, 125 MHz
    signal dhb_clock : std_logic := '0';   -- 200 MHz
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal probe         : scope_sample;
    -- the DC links in ADC counts: the primary at ~96 V, the secondary charged by the primary's
    -- pulses towards it (by coupling of the difference a clock, 0 for none)
    constant primary_counts : real := 1600.0;
    signal secondary_counts : real := 0.0;
    signal coupling         : real := 0.0;
    signal primary_voltage, secondary_voltage : scope_sample;
    -- switching noise on the secondary's reading: +-noise counts, a new value every 40 ns
    signal noise            : integer := 0;
    signal noise_amplitude  : natural := 0;
    signal primary_gates   : dhb_leg_gates;
    signal secondary_gates : dhb_leg_gates;
    signal phase           : dhb_phase := 0;
    signal trip            : boolean := false;
    signal bridge_running  : boolean;
    signal hold            : boolean := false;
    signal secondary_on    : natural := 0;   -- clocks with a secondary gate on

    -- pulses seen: count, the narrowest and widest high side pulse in dhb clocks
    signal high_pulses   : natural := 0;
    signal widest        : natural := 0;
    signal narrowest     : natural := natural'high;
    signal both_on       : natural := 0;
    signal first_width   : natural := 0;
    signal narrower      : natural := 0;   -- pulses narrower than the one before

begin

    clock     <= not clock after 4 ns;
    dhb_clock <= not dhb_clock after 2.5 ns;

    dut : entity work.dhb_test
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, probe, phase, trip, bridge_running, primary_voltage, secondary_voltage, dhb_clock,
             primary_gates, secondary_gates, hold);

    primary_voltage   <= std_logic_vector(to_unsigned(integer(primary_counts), 16));
    secondary_voltage <= std_logic_vector(to_unsigned(maximum(0, integer(secondary_counts) + noise), 16));

    switching_noise : process
        variable seed1, seed2 : positive := 7;
        variable r : real;
    begin
        wait for 40 ns;
        uniform(seed1, seed2, r);
        noise <= integer((2.0 * r - 1.0) * real(noise_amplitude));
    end process;

    secondary_dc_link : process(dhb_clock)
    begin
        if rising_edge(dhb_clock) then
            if (primary_gates.high or primary_gates.low) = '1' then
                secondary_counts <= secondary_counts + coupling * (primary_counts - secondary_counts);
            end if;
        end if;
    end process;

    watch : process(dhb_clock)
        variable width : natural := 0;
    begin
        if rising_edge(dhb_clock) then
            if primary_gates.high = '1' then
                width := width + 1;
            elsif width > 0 then
                high_pulses <= high_pulses + 1;
                if high_pulses = 0 then first_width <= width; end if;
                if width > widest then widest <= width; end if;
                if width < narrowest then narrowest <= width; end if;
                if width < widest then narrower <= narrower + 1; end if;
                width := 0;
            end if;
            if (primary_gates.high and primary_gates.low) = '1' or
               (secondary_gates.high and secondary_gates.low) = '1' then
                both_on <= both_on + 1;
            end if;
            if (secondary_gates.high or secondary_gates.low) = '1' then
                secondary_on <= secondary_on + 1;
            end if;
        end if;
    end process;

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
        procedure bus_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(to_unsigned(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        procedure bus_read(address : natural; data : out natural) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := to_integer(unsigned(bus_read_data(30 downto 0)));
        end procedure;

        variable v : natural;
        variable started : time;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("the_default_burst") then
                wait for 10 us;
                check_equal(high_pulses, 0, "nothing before a start");
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                check(probe(0) = '1', "the burst on the probe");
                started := now;
                wait until probe(0) = '0' for 30 ms;
                info("burst of " & time'image(now - started) & ", " & integer'image(high_pulses) &
                     " high side pulses of " & integer'image(narrowest) & " .. " & integer'image(widest) & " clocks");
                check(now - started > 19.9 ms and now - started < 20.1 ms, "20 ms");
                check_equal(high_pulses, 3000, "a pulse a period");
                check_equal(widest, 40, "200 ns pulses");
                check_equal(narrowest, 40, "all of them");
                bus_read(base + 3, v);
                check_equal(v, 3000, "3000 periods counted");
                bus_read(base, v);
                check_equal(v, 0, "off, no burst");
                wait for 50 us;
                check(primary_gates = dhb_leg_gates_off, "the gates off after it");
                check_equal(both_on, 0, "clocks with both switches on");

            elsif run("a_short_burst_and_limits") then
                bus_write(base + 1, 30);          -- 30 periods
                bus_write(base + 2, 1000);        -- past the 100 clock limit
                bus_write(base, 1);
                wait for 1 us;
                bus_read(base, v);
                check_equal(v, 256 + 1, "a burst, precharging");
                check(v / 512 = 0, "not charged");
                wait until probe(0) = '0' for 1 ms;
                check_equal(high_pulses, 30, "30 pulses");
                check_equal(widest, 100, "the pulse width limited to 100 clocks");
                bus_write(base + 1, 100000);      -- past the 75000 period limit
                bus_write(base + 2, 40);
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                started := now;
                wait until probe(0) = '0' for 700 ms;
                check(now - started > 499 ms and now - started < 501 ms, "the burst limited to 75000 periods (500 ms)");

            elsif run("a_50_ms_ramp") then
                -- a clock of dead time less every 124 periods: 606 clocks, start to nominal,
                -- in 500 ms; 7500 periods of it widen the 40 clock pulses by 60
                bus_write(base + 1, 7500);
                bus_write(base + 4, 124);
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                started := now;
                wait until probe(0) = '0' for 100 ms;
                info("burst of " & time'image(now - started) & ", " & integer'image(high_pulses) &
                     " pulses widening from " & integer'image(first_width) & " to " & integer'image(widest) & " clocks");
                check(now - started > 49.9 ms and now - started < 50.1 ms, "50 ms");
                check_equal(high_pulses, 7500, "a pulse a period");
                check_equal(first_width, 40, "from 200 ns");
                check_equal(widest, 40 + 7500 / 124, "a clock wider every 124 periods");
                check_equal(narrower, 0, "never narrower");
                check_equal(both_on, 0, "clocks with both switches on");

            elsif run("the_ramp_is_limited") then
                -- 1 period a step asked, 20 taken: 1000 periods widen the pulses by 50, not 1000
                bus_write(base + 1, 1000);
                bus_write(base + 4, 1);
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                wait until probe(0) = '0' for 20 ms;
                check_equal(widest, 40 + 1000 / 20, "at most a clock every 20 periods");

            elsif run("ends_when_charged") then
                -- the 500 ms ramp, the secondary charging; done within 84 counts (5 V)
                coupling <= 2.0e-5;
                bus_write(base + 1, 75000);
                bus_write(base + 4, 124);
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                started := now;
                wait until probe(0) = '0' for 600 ms;
                bus_read(base + 3, v);
                info("charged to " & real'image(secondary_counts) & " of " & real'image(primary_counts) &
                     " counts after " & time'image(now - started) & ", " & integer'image(v) & " periods");
                check(now - started < 490 ms, "ended before the period limit");
                -- the board compares whole ADC counts
                check(integer(secondary_counts) >= integer(primary_counts) - 84, "within the margin");
                bus_read(base, v);
                check_equal(v, 512, "off, no burst, ended charged");
                v := high_pulses;
                wait for 1 ms;
                check_equal(high_pulses, v, "no more pulses");
                check_equal(both_on, 0, "clocks with both switches on");

            elsif run("ends_when_charged_through_noise") then
                -- the board's +-15 V (250 counts) on the secondary's reading: the averages
                -- still end it only once the secondary itself is there
                noise_amplitude <= 250;
                coupling <= 2.0e-5;
                bus_write(base + 1, 75000);
                bus_write(base + 4, 124);
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                started := now;
                wait until probe(0) = '0' for 600 ms;
                info("through the noise: charged to " & real'image(secondary_counts) & " counts after " &
                     time'image(now - started));
                check(integer(secondary_counts) >= integer(primary_counts) - 84 - 10,
                      "within the margin, give or take what the averages leave of the noise");
                bus_read(base, v);
                check_equal(v, 512, "ended charged");

            elsif run("runs_after_the_precharge") then
                -- precharge on the ramp, then 3000 periods (20 ms) of both legs at a phase of 100
                coupling <= 2.0e-5;
                phase    <= 100;
                bus_write(base + 1, 75000);
                bus_write(base + 4, 124);
                bus_write(base + 6, 3000);
                bus_write(base, 1);
                wait until probe(3) = '1' for 100 ms;
                check(probe(3) = '1', "running after the precharge");
                wait for 50 ns;
                check(bridge_running, "running, for the voltage controller");
                check_equal(secondary_on, 0, "the secondary's gates off until running");
                wait for 1 ms;
                -- the primary leads by the phase, both with 100 ns of dead time
                wait until rising_edge(primary_gates.high);
                started := now;
                wait until rising_edge(secondary_gates.high);
                check_equal((now - started) / 5 ns, 100, "the secondary 100 counts behind");
                started := now;
                wait until rising_edge(secondary_gates.high);
                check_equal((now - started) / 5 ns, 1333, "a period");
                wait until probe(0) = '0' for 50 ms;
                bus_read(base + 8, v);
                check_equal(v, 3000, "3000 periods run");
                bus_read(base, v);
                check_equal(v, 512, "off, ended charged");
                wait for 50 us;
                check(primary_gates = dhb_leg_gates_off and secondary_gates = dhb_leg_gates_off, "all off");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("held_past_its_run_time") then
                -- a run time of 300 periods (2 ms), held for 10 ms after the precharge: it runs
                -- on, then stops at the first period after the hold falls; a trip still stops it
                coupling <= 2.0e-5;
                phase    <= 100;
                bus_write(base + 1, 75000);
                bus_write(base + 6, 300);
                hold <= true;
                bus_write(base, 1);
                wait until bridge_running for 100 ms;
                check(bridge_running, "running after the precharge");
                started := now;
                wait for 10 ms;
                check(bridge_running, "still running 10 ms on, held past its 2 ms");
                hold <= false;
                wait until not bridge_running for 100 us;
                check(not bridge_running, "stopped after the hold");
                check(now - started > 9.9 ms and now - started < 10.1 ms, "within a period of the hold falling");
                bus_read(base + 8, v);
                check(v > 1490 and v < 1510, "the held periods counted: " & integer'image(v));
                wait for 50 us;
                check(primary_gates = dhb_leg_gates_off and secondary_gates = dhb_leg_gates_off, "all off");
                -- held again, then a trip
                hold <= true;
                bus_write(base, 1);
                wait until bridge_running for 100 ms;
                wait for 1 ms;
                trip <= true;
                wait for 1 us;
                check(primary_gates = dhb_leg_gates_off and secondary_gates = dhb_leg_gates_off, "a trip stops it held");
                hold <= false;

            elsif run("the_secondary_inverted") then
                -- inverted: the secondary's high side where its low side was, half a period on
                coupling <= 2.0e-5;
                phase    <= 100;
                bus_write(base + 1, 75000);
                bus_write(base + 4, 124);
                bus_write(base + 6, 3000);
                bus_write(base + 7, 1);
                bus_write(base, 1);
                wait until probe(3) = '1' for 100 ms;
                wait for 1 ms;
                wait until rising_edge(primary_gates.high);
                started := now;
                wait until rising_edge(secondary_gates.high);
                check_equal((now - started) / 5 ns, 100 + 1333 / 2, "half a period further: 100 + 666");
                wait until probe(0) = '0' for 50 ms;
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("a_trip_turns_everything_off") then
                -- running after the precharge, then a trip: the gates off within a few clocks
                coupling <= 2.0e-5;
                phase    <= 100;
                bus_write(base + 1, 75000);
                bus_write(base + 4, 124);
                bus_write(base + 6, 3000);
                bus_write(base, 1);
                wait until probe(3) = '1' for 100 ms;
                wait for 100 us;
                trip <= true;
                wait until rising_edge(clock);
                wait for 30 ns;   -- two flip-flops into dhb_clock and one more
                check(primary_gates = dhb_leg_gates_off and secondary_gates = dhb_leg_gates_off,
                      "all gates off within 30 ns of the trip");
                wait for 50 us;
                bus_read(base, v);
                check_equal(v mod 512, 0, "off, no burst");
                -- no start while tripped
                v := high_pulses;
                bus_write(base, 1);
                wait for 1 ms;
                check_equal(high_pulses, v, "no start while tripped");
                -- cleared, it starts
                trip <= false;
                bus_write(base, 1);
                wait until probe(0) = '1' for 2 us;
                check(probe(0) = '1', "starts once cleared");
                check_equal(both_on, 0, "clocks with both switches of a leg on");

            elsif run("stop_in_the_middle") then
                bus_write(base, 1);
                wait for 1 ms;
                bus_write(base, 2);
                wait for 100 ns;
                check(primary_gates = dhb_leg_gates_off, "off at once");
                bus_read(base, v);
                check_equal(v, 0, "off, no burst");
                v := high_pulses;
                wait for 1 ms;
                check_equal(high_pulses, v, "no more pulses");
                check_equal(both_on, 0, "clocks with both switches on");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 2000 ms);

end vunit_simulation;
