LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;

-- measurement_scaler with the board's calibrations: the conversion against value =
-- (counts - zero) / per_unit in reals, every channel ready in the same clock scaled one after
-- another to its own channel, a channel ready again while pending, saturation, readback.
entity measurement_scaler_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of measurement_scaler_tb is

    constant base     : natural := 900;
    constant channels : natural := 19;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal raw            : fixed_array(0 to channels-1) := (others => fixed_zero);
    signal ready          : std_logic_vector(0 to channels-1) := (others => '0');
    signal scaled         : fixed_array(0 to channels-1);
    signal scaled_ready   : boolean;
    signal scaled_channel : natural range 0 to channels-1;

    -- the calibration of each channel: value = (counts - zero) / per_unit
    type calibration is record
        zero, per_unit : real;
    end record;
    type calibration_array is array (0 to channels-1) of calibration;
    constant calibrations : calibration_array := (
        0  => (-15.197, 16.857480),      -- sec 0 vdc_out
        9  => (-15.197, 16.857480),      -- pri 1 vdc_in
        8  => (8137.909, 16.763636),     -- pri 0 vac_in
        11 => (8153.273, -16.327273),    -- pri 3 vfilt_in: a negative scale
        16 => (-160.2, -1024.0),         -- sdm iac_in
        18 => (-197.4, -1024.0),         -- sdm i_dab
        others => (0.0, 16.0));

    function expected(c : natural; counts : integer) return real is
    begin
        return (real(counts) - calibrations(c).zero) / calibrations(c).per_unit;
    end function;

    -- results seen: count, and the channels in the order they came
    signal results       : natural := 0;
    type order_array is array (0 to 63) of natural;
    signal order         : order_array := (others => 0);
    signal last_result_at : time := 0 ns;
    signal back_to_back  : natural := 0;   -- results a clock after the one before

begin

    clock <= not clock after 4 ns;

    dut : entity work.measurement_scaler
    generic map(g_base_address => base, g_channels => channels)
    port map(clock, bus_in, bus_out, raw, ready, scaled, scaled_ready, scaled_channel);

    watch : process(clock)
    begin
        if rising_edge(clock) then
            if scaled_ready then
                order(results mod 64) <= scaled_channel;
                results <= results + 1;
                if now - last_result_at = 8 ns then
                    back_to_back <= back_to_back + 1;
                end if;
                last_result_at <= now;
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
        procedure bus_write(address : natural; data : signed) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(data));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        procedure bus_read(address : natural; data : out signed) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := signed(bus_read_data);
        end procedure;

        -- slope with 30 fractional bits, offset with 20, as the firmware will write them
        procedure calibrate(c : natural) is
        begin
            bus_write(base + c, to_fixed(1.0 / calibrations(c).per_unit, 30));
            bus_write(base + 32 + c, to_fixed(-calibrations(c).zero / calibrations(c).per_unit));
        end procedure;

        procedure measure(c : natural; counts : integer) is
        begin
            wait until rising_edge(clock);
            raw(c)   <= to_signed(counts, 32);
            ready(c) <= '1';
            wait until rising_edge(clock);
            ready(c) <= '0';
            wait for 100 ns;
        end procedure;

        -- within a part in a million of full scale, the fixed point's resolution and the
        -- slope's rounding times the counts
        procedure check_scaled(c : natural; counts : integer) is
            constant want : real := expected(c, counts);
            constant got  : real := to_real(scaled(c));
        begin
            check(abs(got - want) < 2.0e-5, "channel " & integer'image(c) & ", " & integer'image(counts) &
                  " counts: " & real'image(got) & " for " & real'image(want));
        end procedure;

        variable v : fixed_number;
        variable first_result : natural;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("scales_like_the_calibration") then
                for c in 0 to channels-1 loop
                    calibrate(c);
                end loop;
                for counts in 0 to 16 loop
                    measure(9, counts * 1000);
                    check_scaled(9, counts * 1000);       -- 0 .. 16000 counts: -0.9 .. 950 V
                    measure(11, counts * 1000);
                    check_scaled(11, counts * 1000);      -- the negative scale
                    measure(18, -32768 + counts * 4096);
                    check_scaled(18, -32768 + counts * 4096);   -- the currents' signed counts
                end loop;
                info("16000 counts on vdc_in: " & real'image(to_real(scaled(9))) & " V");

            elsif run("all_ready_together_one_at_a_time") then
                for c in 0 to channels-1 loop
                    calibrate(c);
                end loop;
                wait until rising_edge(clock);
                first_result := results;
                for c in 0 to channels-1 loop
                    raw(c) <= to_signed(100 + 700 * c, 32);   -- a value of its own each
                end loop;
                ready <= (others => '1');
                wait until rising_edge(clock);
                ready <= (others => '0');
                wait for 1 us;
                check_equal(results - first_result, channels, "a result per channel");
                check_equal(back_to_back, channels - 1, "one a clock, back to back");
                for c in 0 to channels-1 loop
                    check_equal(order(first_result + c), c, "in channel order");
                    check_scaled(c, 100 + 700 * c);
                end loop;

            elsif run("ready_again_while_pending") then
                for c in 0 to channels-1 loop
                    calibrate(c);
                end loop;
                -- channel 18 waits behind 0..17 and gets a newer value meanwhile: one result,
                -- the newer value
                wait until rising_edge(clock);
                first_result := results;
                for c in 0 to channels-1 loop
                    raw(c) <= to_signed(1000, 32);
                end loop;
                ready <= (others => '1');
                wait until rising_edge(clock);
                ready <= (others => '0');
                raw(18)   <= to_signed(-5000, 32);
                ready(18) <= '1';
                wait until rising_edge(clock);
                ready(18) <= '0';
                wait for 1 us;
                check_equal(results - first_result, channels, "still a result per channel");
                check_scaled(18, -5000);

            elsif run("saturates") then
                bus_write(base + 2, to_fixed(1.0, 30));          -- a count a unit
                bus_write(base + 32 + 2, fixed_zero);
                measure(2, 1000);
                check_equal(to_real(scaled(2)), 1000.0, "inside the range");
                measure(2, 5000);
                check(scaled(2) = fixed_max, "+2048 saturates, not wraps");
                measure(2, -5000);
                check(scaled(2) = fixed_min, "-2048 too");

            elsif run("not_calibrated_reads_zero_and_readback") then
                measure(5, 12345);
                check(scaled(5) = fixed_zero, "zero until calibrated");
                calibrate(5);
                measure(5, 12345);
                bus_read(base + 64 + 5, v);
                check(v = scaled(5) and v /= fixed_zero, "read back over the bus");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
