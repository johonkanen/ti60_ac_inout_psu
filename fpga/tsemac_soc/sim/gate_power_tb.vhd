LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- gate_power: off at power up, each output on by its bit at 239.5 kHz and 44 %, the supplies
-- coming on one at a time however many bits are set at once, no two outputs switching
-- together, the duty never above half the period.
entity gate_power_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of gate_power_tb is

    constant base : natural := 760;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal pwm, previous : std_logic_vector(1 to 6) := (others => '0');

    -- the edges: clocks with edges of two outputs, and the fewest clocks between edges of
    -- different outputs
    signal clocks_since_edge : natural := 1000;
    signal last_edge_output  : natural := 0;
    signal together          : natural := 0;
    signal closest           : natural := 1000;
    signal watch_edges       : boolean := false;
    -- when each output first came on
    type time_array is array (1 to 6) of time;
    signal came_on : time_array := (others => 0 ns);

begin

    clock <= not clock after 4 ns;    -- 125 MHz

    dut : entity work.gate_power
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, pwm);

    edges : process(clock)
        variable count, output : natural;
    begin
        if rising_edge(clock) then
            previous <= pwm;
            count := 0;
            for n in 1 to 6 loop
                if pwm(n) /= previous(n) then
                    count := count + 1;
                    output := n;
                end if;
                if pwm(n) = '1' and previous(n) = '0' and came_on(n) = 0 ns then
                    came_on(n) <= now;
                end if;
            end loop;
            if count = 0 then
                clocks_since_edge <= clocks_since_edge + 1;
            else
                clocks_since_edge <= 1;
                last_edge_output  <= output;
                if watch_edges then
                    if count > 1 then
                        together <= together + 1;
                    elsif output /= last_edge_output and clocks_since_edge < closest then
                        closest <= clocks_since_edge;
                    end if;
                end if;
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

        -- the high and the whole time of output n, from a rising edge after the first, which
        -- may be partial when the output was just enabled
        procedure measure(n : natural; high, period : out time) is
            variable rise : time;
        begin
            wait until rising_edge(pwm(n)) for 20 us;
            check(pwm(n) = '1', "output " & integer'image(n) & " runs");
            wait until rising_edge(pwm(n));
            rise := now;
            wait until falling_edge(pwm(n));
            high := now - rise;
            wait until rising_edge(pwm(n));
            period := now - rise;
        end procedure;

        variable high, period, rise : time;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("off_at_power_up") then
                wait for 50 us;
                check(pwm = "000000", "all off");

            elsif run("one_output_by_its_bit") then
                bus_write(base, 2#000100#);       -- 3
                measure(3, high, period);
                info("period " & time'image(period) & ", high " & time'image(high));
                check_equal(period, 522 * 8 ns, "239.5 kHz");
                check_equal(high, 229 * 8 ns, "44 %");
                for k in 1 to 1000 loop
                    wait until rising_edge(clock);
                    check(pwm(1) = '0' and pwm(2) = '0' and pwm(4) = '0' and pwm(5) = '0' and pwm(6) = '0',
                          "the others off");
                end loop;
                bus_write(base, 0);
                wait for 5 us;
                check(pwm = "000000", "off again at once");

            elsif run("one_supply_at_a_time") then
                bus_write(base + 3, 1);           -- 1 ms apart
                bus_write(base, 2#111111#);       -- all at once
                wait for 7 ms;
                for n in 1 to 6 loop
                    check(came_on(n) /= 0 ns, "supply " & integer'image(n) & " came on");
                end loop;
                for n in 2 to 6 loop
                    info("supply " & integer'image(n) & " " & time'image(came_on(n) - came_on(n - 1))
                         & " after the previous");
                    -- a supply's first pulse comes up to a period after it is on
                    check(came_on(n) - came_on(n - 1) >= 1 ms - 522 * 8 ns, "supply " & integer'image(n)
                          & " at least 1 ms after supply " & integer'image(n - 1));
                end loop;
                bus_write(base, 0);
                wait for 5 us;
                check(pwm = "000000", "all off at once");

            elsif run("no_two_switch_together") then
                bus_write(base + 3, 1);
                bus_write(base, 2#111111#);
                wait for 6.1 ms;
                watch_edges <= true;
                wait for 100 us;
                info("edges of different outputs at least " & integer'image(closest) & " clocks apart");
                check_equal(together, 0, "clocks where two outputs switched");
                check(closest >= 32, "edges at least 32 clocks apart");
                -- output n rises (n-1) * 87 clocks after output 1
                wait until rising_edge(pwm(1));
                rise := now;
                for n in 2 to 6 loop
                    wait until rising_edge(pwm(n));
                    check_equal(now - rise, (n - 1) * 87 * 8 ns, "output " & integer'image(n) & "'s phase");
                end loop;

            elsif run("duty_at_most_half") then
                bus_write(base + 2, 400);
                bus_write(base, 2#000001#);
                wait for 10 us;
                measure(1, high, period);
                check_equal(high, 261 * 8 ns, "half of 522");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
