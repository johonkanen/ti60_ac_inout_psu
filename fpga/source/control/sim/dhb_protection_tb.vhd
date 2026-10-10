LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;

-- dhb_protection fed like measurement_scaler feeds it: overcurrent either way after 3 readings
-- in a row over the limit (not single spikes), overvoltage on either DC link only after 8
-- readings in a row over the limit (not single noisy ones), nothing from other channels,
-- latched with the value until cleared, limits from the registers.
entity dhb_protection_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of dhb_protection_tb is

    constant base : natural := 1120;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal scaled         : fixed_array(0 to 18) := (others => fixed_zero);
    signal scaled_ready   : boolean := false;
    signal scaled_channel : natural range 0 to 18 := 0;
    signal trip           : boolean;

begin

    clock <= not clock after 4 ns;

    dut : entity work.dhb_protection
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, scaled, scaled_ready, scaled_channel, trip);

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

        procedure scaler(channel : natural; value : real) is
        begin
            wait until rising_edge(clock);
            scaled(channel) <= to_fixed(value);
            scaled_ready    <= true;
            scaled_channel  <= channel;
            wait until rising_edge(clock);
            scaled_ready    <= false;
            wait for 20 ns;
        end procedure;

        procedure clear is
        begin
            bus_write(base + 2, to_signed(1, 32));
            wait for 20 ns;
        end procedure;

        variable v : integer;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("overcurrent_either_way") then
                scaler(18, 4.9);
                scaler(18, -4.9);
                check(not trip, "within 5 A");
                scaler(18, 5.1);
                scaler(18, 5.3);
                check(not trip, "2 readings over: not yet");
                scaler(18, 5.2);
                check(trip, "the third reading over 5 A trips");
                bus_read(base + 2, v);
                check_equal(v, 1, "overcurrent");
                bus_read(base + 3, v);
                check(abs(real(v) / 2.0**20 - 5.2) < 1.0e-5, "the current that tripped");
                scaler(18, 0.0);
                check(trip, "latched");
                clear;
                check(not trip, "cleared");
                for k in 1 to 3 loop
                    scaler(18, -5.3);
                end loop;
                check(trip, "-5.3 A trips too");
                clear;
                for k in 1 to 3 loop
                    scaler(17, 5.5);
                end loop;
                check(trip, "the output bridge's current at 5.5 A trips");

            elsif run("current_spikes_do_not_trip") then
                -- spikes of one or two readings over the limit, a reading within it between
                for k in 1 to 50 loop
                    scaler(17, -8.0);
                    if k mod 2 = 0 then scaler(17, -7.0); end if;
                    scaler(17, -3.0);
                end loop;
                check(not trip, "spikes of one or two readings do not trip");
                -- the other channel's readings do not count towards the output bridge's
                scaler(17, 6.0);
                scaler(18, 6.0);
                scaler(17, 6.0);
                scaler(18, 0.0);
                check(not trip, "two of each, not three of one");
                scaler(17, 6.0);
                check(trip, "the third of the output bridge's trips");

            elsif run("overvoltage_on_either_dc_link") then
                scaler(9, 119.0);
                scaler(0, 119.0);
                check(not trip, "within 120 V");
                for k in 1 to 7 loop
                    scaler(0, 121.0);
                end loop;
                check(not trip, "7 readings over: not yet");
                scaler(0, 121.0);
                check(trip, "the secondary at 121 V for 8 readings trips");
                bus_read(base + 2, v);
                check_equal(v, 2, "overvoltage");
                bus_read(base + 4, v);
                check(abs(real(v) / 2.0**20 - 121.0) < 1.0e-5, "the voltage that tripped");
                clear;
                for k in 1 to 8 loop
                    scaler(9, 125.0);
                end loop;
                check(trip, "the primary trips too");

            elsif run("noise_does_not_trip") then
                -- the board's secondary reading while switching: +-30 V around 95 V, single
                -- readings over 120 V now and then, never 8 in a row
                for k in 0 to 999 loop
                    if k mod 5 = 0 then
                        scaler(0, 121.0 + real(k mod 3));
                    else
                        scaler(0, 95.0 + 25.0 * sin(real(k)));
                    end if;
                end loop;
                check(not trip, "no trip on the noise");

            elsif run("other_channels_do_not_trip") then
                scaler(16, 50.0);    -- iac_in
                scaler(8, 300.0);    -- vac_in
                scaler(11, 300.0);   -- vfilt_in
                check(not trip, "no trip");

            elsif run("limits_from_the_registers") then
                bus_write(base + 0, to_fixed(2.0));
                bus_write(base + 1, to_fixed(100.0));
                bus_write(base + 6, to_signed(1, 32));   -- a single reading trips
                scaler(18, 2.5);
                check(trip, "2.5 A over a 2 A limit, a single reading");
                clear;
                bus_write(base + 5, to_signed(1, 32));   -- a single reading trips
                scaler(0, 101.0);
                check(trip, "101 V over a 100 V limit");
                clear;
                bus_write(base + 1, to_fixed(450.0));
                scaler(0, 400.0);
                check(not trip, "400 V under 450 V");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
