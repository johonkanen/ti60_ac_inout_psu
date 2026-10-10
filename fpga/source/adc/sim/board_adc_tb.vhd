LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;

-- board_adc with an ADS7056 model behind a mux model: every mux position gives its own value.
entity board_adc_tb is
  generic (runner_cfg : string; output_delay_ns : real := 10.0);
end;

architecture vunit_simulation of board_adc_tb is

    constant base : natural := 700;

    signal clock : std_logic := '0';
    signal adc_clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal latest    : unsigned(13 downto 0);
    signal positions : scope_sample_array(0 to 7);
    signal is_new    : boolean;
    signal mux       : std_logic_vector(2 downto 0);
    signal sclk, cs_n, sdo : std_logic;
    signal value     : natural range 0 to 16383 := 0;
    signal is_calibrated : boolean;
    signal frames    : natural;
    signal mux_switched_at : time := 0 ns;
    signal cs_fell_at      : time := 0 ns;
    signal wrong_hold      : natural := 0;
    signal wrong_settling  : natural := 0;
    signal settle_ns       : real := 400.0 * 5.0;

    function mux_value(p : natural) return natural is
    begin
        return 1000 + 1700 * p + 3;   -- odd values exercise the low bits
    end function;

begin

    clock     <= not clock after 4 ns;
    adc_clock <= not adc_clock after 2.5 ns;

    u_adc : entity work.board_adc
    generic map(g_base_address => base)
    port map(clock, bus_in, bus_out, latest, positions, is_new, adc_clock, mux, sclk, cs_n, sdo);

    -- the mux output follows the selection
    value <= mux_value(to_integer(unsigned(mux)));

    u_model : entity work.ads7056_model
    generic map(g_output_delay => output_delay_ns * 1 ns)
    port map(sclk, cs_n, sdo, value, is_calibrated, frames);

    -- the mux must have settled when a sample is taken, and hold still for 100 ns after it,
    -- while the ADC's sample and hold takes it
    settling : process(mux, cs_n)
    begin
        if mux'event then
            mux_switched_at <= now;
            if now - cs_fell_at < 100 ns and now > 20 us then
                wrong_hold <= wrong_hold + 1;
            end if;
        end if;
        if falling_edge(cs_n) then
            cs_fell_at <= now;
        end if;
        if falling_edge(cs_n) and now - mux_switched_at < settle_ns * 1 ns and now > 20 us then
            wrong_settling <= wrong_settling + 1;
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

        variable v, w, first_count, passing : natural;
        variable good : boolean;
        variable window : std_logic_vector(0 to 8);
        variable started : time;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("calibrates_and_converts_all_positions") then
                wait until is_calibrated for 50 us;
                check(is_calibrated, "calibrated after power up");
                wait for 50 us;
                for p in 0 to 7 loop
                    bus_read(base + 8 + p, v);
                    check_equal(v, mux_value(p), "position " & integer'image(p));
                    check_equal(to_integer(unsigned(positions(p))), mux_value(p), "scope source of position " & integer'image(p));
                end loop;
                bus_read(base + 4, v);
                check_equal(v mod 2, 1, "calibrated status bit");
                bus_read(base + 3, v);
                check(frames - v <= 2, "conversions counted, the first after calibration dropped");
                check_equal(wrong_settling, 0, "conversions before the mux settled");
                check_equal(wrong_hold, 0, "mux switched within 100 ns of the sample");
                -- with hold and settle shorter than a frame the mux settles while the frames
                -- run, so all positions convert at the full rate
                bus_write(base + 2, 56);
                settle_ns <= 56.0 * 5.0;
                wait for 5 us;
                bus_read(base + 3, first_count);
                wait for 100 us;
                bus_read(base + 3, v);
                info(integer'image(v - first_count) & " conversions in 100 us over eight positions");
                check(v - first_count > 240, "2.5 MS/s over all positions");
                check_equal(wrong_settling, 0, "conversions before the mux settled, at the full rate");
                check_equal(wrong_hold, 0, "mux switched within 100 ns of the sample, at the full rate");
                bus_read(base + 7, v);
                check_equal(v, 0, "frames with bad zero bits");

            elsif run("single_position_back_to_back") then
                bus_write(base + 0, 16#0801#);           -- position 3 only
                wait until is_calibrated for 50 us;
                wait for 10 us;
                bus_read(base + 3, first_count);
                started := now;
                wait for 100 us;
                bus_read(base + 3, v);
                info(integer'image(v - first_count) & " conversions in 100 us");
                -- 18 SCLKs of 20 ns plus 40 ns CS high: 2.5 MS/s
                check(v - first_count > 240, "back to back conversions at 2.5 MS/s");
                check_equal(to_integer(latest), mux_value(3), "latest");
                check_equal(mux, std_logic_vector'("011"), "mux stays at 3");

            elsif run("sample_delay_window") then
                -- the window of good sample delays for the SDO delay of the model
                bus_write(base + 2, 76);
                settle_ns <= 76.0 * 5.0;
                wait until is_calibrated for 50 us;
                passing := 0;
                for delay in 0 to 8 loop
                    bus_write(base + 6, delay);
                    wait for 5 us;
                    bus_read(base + 7, first_count);
                    wait for 10 us;
                    bus_read(base + 7, v);
                    good := v = first_count;
                    for p in 0 to 7 loop
                        bus_read(base + 8 + p, w);
                        good := good and w = mux_value(p);
                    end loop;
                    window(delay) := '1' when good else '0';
                    if good then passing := passing + 1; end if;
                end loop;
                info("sample delays 0..8 passing: " & to_string(window));
                check(passing >= 2, "a window of at least two sample delays");
                check(window(1) = '1', "the default sample delay 1 passes");
                -- jumping the delay down and up again while frames run must not stop the ADC
                for i in 0 to 20 loop
                    bus_write(base + 6, 11 * (i mod 2));
                    wait for 130 ns * (1 + i mod 3);
                end loop;
                bus_write(base + 6, 1);
                bus_read(base + 3, first_count);
                wait for 10 us;
                bus_read(base + 3, v);
                check(v - first_count > 20, "conversions go on after changing the delay mid frame");

            elsif run("recalibrate_and_stop") then
                wait until is_calibrated for 50 us;
                wait for 10 us;
                bus_write(base + 0, 16#FF02#);           -- stop and recalibrate
                wait for 20 us;
                bus_read(base + 3, first_count);
                wait for 20 us;
                bus_read(base + 3, v);
                check_equal(v, first_count, "stopped");
                bus_read(base + 4, v);
                check_equal(v mod 2, 1, "calibrated again");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 ms);

end vunit_simulation;
