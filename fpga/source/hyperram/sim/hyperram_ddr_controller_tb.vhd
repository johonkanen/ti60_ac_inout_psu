LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.hyperram_pkg.all;

-- hyperram_ddr_controller at 200 MHz with models of the DDIO pins, the PLL phase shift of
-- the read clock and the HyperRAM.
--   tckd_ns      CK to DQ/RWDS delay of the memory model
--   lo_lags      pairing of the input DDIO halves, see the input register model
-- Data from the memory is valid for 1.45 ns of each 2.5 ns half period (tDV at 200 MHz),
-- hyperram_calibrator selects the read clock phase after power up like on the board.
entity hyperram_ddr_controller_tb is
  generic (runner_cfg : string; tckd_ns : real := 3.0; lo_lags : boolean := true);
end;

architecture vunit_simulation of hyperram_ddr_controller_tb is

    constant main_period : time := 8 ns;
    constant hr_period   : time := 5 ns;
    constant phase_step  : time := 250 ps;

    signal main_clock : std_logic := '0';
    signal hr_clock   : std_logic := '0';
    signal ck_clock   : std_logic := '0';   -- 270 degrees: 90 degree PLL output inverted at the CK pin
    signal read_clock : std_logic := '0';
    signal read_delay : time := 0 ns;

    signal hyperram_in  : hyperram_in_record := init_hyperram_in;
    signal command_in   : hyperram_in_record := init_hyperram_in;
    signal write_in     : hyperram_in_record := init_hyperram_in;
    signal hyperram_out : hyperram_out_record;
    signal phase        : natural range 0 to 7 := 0;
    signal lo_first     : boolean;
    signal controller_in  : hyperram_in_record;
    signal controller_out : hyperram_out_record;
    signal calibration_status : std_logic_vector(31 downto 0);
    signal phase_shift  : pll_phase_shift_record;
    signal fpga_out     : hyperram_ddr_fpga_out_record;
    signal fpga_in      : hyperram_ddr_fpga_in_record;

    -- pins
    signal ck, cs_n, reset_n : std_logic := '0';
    signal dq   : std_logic_vector(7 downto 0);
    signal rwds : std_logic;
    signal dq_out_pin  : std_logic_vector(7 downto 0) := (others => '0');
    signal rwds_out_pin, dq_oe_pin, rwds_oe_pin : std_logic := '0';

    type word_array is array (natural range <>) of std_logic_vector(15 downto 0);
    signal received : word_array(0 to 16383);
    signal number_of_received : natural := 0;
    signal write_buffer : word_array(0 to 1023);
    signal words_written : natural := 0;

    -- capture port
    signal capture_reset, capture_write, capture_flush, capture_is_full, capture_is_ready : boolean := false;
    signal capture_data    : std_logic_vector(127 downto 0) := (others => '0');
    signal capture_written : unsigned(31 downto 0);
    signal records_to_push : natural := 0;
    signal records_pushed  : natural := 0;
    constant capture_base  : natural := 16#8000#;
    constant ring_records  : natural := 128;

    function capture_word(record_number, word : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((record_number * 8 + word) * 97 mod 65536, 16) xor x"3c5a");
    end function;

    function capture_record(record_number : natural) return std_logic_vector is
        variable r : std_logic_vector(127 downto 0);
    begin
        for w in 0 to 7 loop
            r(127 - 16*w downto 112 - 16*w) := capture_word(record_number, w);
        end loop;
        return r;
    end function;

    function test_pattern(address : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(address * 37 mod 65536, 16) xor x"a5c3");
    end function;

begin

    main_clock <= not main_clock after main_period/2;
    hr_clock   <= not hr_clock after hr_period/2;
    ck_clock   <= transport hr_clock after 3*hr_period/4;
    read_clock <= transport hr_clock after read_delay;

    -- PLL dynamic phase shift: SHIFT is taken when SHIFT_ENA is high
    pll_model : process(phase_shift.shift_enable)
    begin
        if phase_shift.shift_enable = '1' and phase_shift.shift_select = "01000" then
            read_delay <= to_integer(unsigned(phase_shift.shift)) * phase_step;
        end if;
    end process;

------------------------------------------------------------------------
    -- output registers: DDIO resync, both halves taken on the rising edge, hi driven in the
    -- first half of the cycle and lo in the second
    ddio_out : process(hr_clock)
        variable dq_lo_hold   : std_logic_vector(7 downto 0);
        variable rwds_lo_hold : std_logic;
    begin
        if rising_edge(hr_clock) then
            dq_out_pin   <= fpga_out.dq_hi;
            dq_lo_hold   := fpga_out.dq_lo;
            rwds_out_pin <= fpga_out.rwds_hi;
            rwds_lo_hold := fpga_out.rwds_lo;
            dq_oe_pin    <= fpga_out.dq_oe;
            rwds_oe_pin  <= fpga_out.rwds_oe;
            cs_n         <= fpga_out.cs_n;
        end if;
        if falling_edge(hr_clock) then
            dq_out_pin   <= dq_lo_hold;
            rwds_out_pin <= rwds_lo_hold;
        end if;
    end process;

    ck_ddio : process(ck_clock)
        variable lo_hold : std_logic;
    begin
        if rising_edge(ck_clock) then
            ck      <= fpga_out.ck_hi;
            lo_hold := fpga_out.ck_lo;
        end if;
        if falling_edge(ck_clock) then
            ck <= lo_hold;
        end if;
    end process;

    reset_n <= fpga_out.reset_n;
    dq   <= dq_out_pin when dq_oe_pin = '1' else (others => 'Z');
    rwds <= rwds_out_pin when rwds_oe_pin = '1' else 'Z';

    -- input registers: hi sampled on the rising edge, lo on the falling edge, presented
    -- together on the rising edge
    ddio_in : process(read_clock)
        variable lo_dq, previous_lo_dq : std_logic_vector(7 downto 0);
        variable lo_rwds, previous_lo_rwds : std_logic;
    begin
        if rising_edge(read_clock) then
            if lo_lags then
                fpga_in <= (dq_hi => to_x01(dq), dq_lo => lo_dq, rwds_hi => to_x01(rwds), rwds_lo => lo_rwds);
            else
                -- lo of the cycle that starts here, so the halves leave one cycle later
                fpga_in <= (dq_hi => previous_lo_dq, dq_lo => lo_dq, rwds_hi => previous_lo_rwds, rwds_lo => lo_rwds);
            end if;
            previous_lo_dq   := to_x01(dq);
            previous_lo_rwds := to_x01(rwds);
        end if;
        if falling_edge(read_clock) then
            lo_dq   := to_x01(dq);
            lo_rwds := to_x01(rwds);
        end if;
    end process;

------------------------------------------------------------------------
    hyperram_in <= (
        command          => command_in.command,
        register_space   => command_in.register_space,
        address          => command_in.address,
        burst_length     => command_in.burst_length,
        write_data       => write_in.write_data,
        write_data_valid => write_in.write_data_valid,
        latency_count    => command_in.latency_count);

    u_calibrator : entity work.hyperram_calibrator
    port map(
        clock            => main_clock
        ,user_in         => hyperram_in
        ,user_out        => hyperram_out
        ,controller_in   => controller_in
        ,controller_out  => controller_out
        ,recalibrate     => false
        ,manual_set      => false
        ,manual_phase    => 0
        ,manual_lo_first => false
        ,read_phase      => phase
        ,input_lo_first  => lo_first
        ,status          => calibration_status);

    u_controller : entity work.hyperram_ddr_controller
    generic map(g_reset_low_clocks => 30, g_power_up_clocks => 100)
    port map(
        main_clock      => main_clock
        ,hyperram_in    => controller_in
        ,hyperram_out   => controller_out
        ,read_phase     => phase
        ,phase_shift    => phase_shift
        ,hyperram_clock => hr_clock
        ,fpga_out       => fpga_out
        ,read_clock     => read_clock
        ,fpga_in        => fpga_in
        ,input_lo_first => lo_first
        ,capture_reset        => capture_reset
        ,capture_write        => capture_write
        ,capture_data         => capture_data
        ,capture_flush        => capture_flush
        ,capture_is_full      => capture_is_full
        ,capture_is_ready     => capture_is_ready
        ,capture_written      => capture_written
        ,capture_base         => capture_base
        ,capture_ring_records => ring_records);

    -- pushes records_to_push records as fast as the capture FIFO takes them
    capture_source : process(main_clock)
        variable next_record : natural := 0;
    begin
        if rising_edge(main_clock) then
            next_record := records_pushed;
            if capture_write and not capture_is_full then
                next_record := records_pushed + 1;
            end if;
            records_pushed <= next_record;
            capture_write  <= next_record < records_to_push;
            capture_data   <= capture_record(next_record);
            if capture_reset then
                records_pushed <= 0;
                capture_write  <= false;
            end if;
        end if;
    end process;

    u_model : entity work.hyperram_model
    generic map(g_tckd => tckd_ns * 1 ns, g_tckdi => tckd_ns * 1 ns - 1.05 ns, g_die_crossing_is_error => false)
    port map(ck, cs_n, reset_n, dq, rwds);

    user_side : process(main_clock)
    begin
        if rising_edge(main_clock) then
            write_in.write_data_valid <= false;
            if command_in.command = write then
                words_written <= 0;
            end if;
            if hyperram_write_data_is_requested(hyperram_out) then
                send_hyperram_write_data(write_in, write_buffer(words_written));
                words_written <= words_written + 1;
            end if;
            if hyperram_read_data_is_ready(hyperram_out) then
                received(number_of_received) <= get_hyperram_read_data(hyperram_out);
                number_of_received <= number_of_received + 1;
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    main : process

        procedure run(command : hyperram_command; address : natural; length : natural; register_space : boolean := false) is
        begin
            wait until rising_edge(main_clock) and hyperram_is_ready(hyperram_out);
            command_in.command        <= command;
            command_in.address        <= address;
            command_in.burst_length   <= length;
            command_in.register_space <= register_space;
            wait until rising_edge(main_clock);
            command_in.command <= none;
            wait until rising_edge(main_clock) and hyperram_transaction_is_done(hyperram_out) for 100 us;
            check(hyperram_transaction_is_done(hyperram_out), "transaction did not finish");
        end procedure;

        procedure read_register(address : natural; expected : std_logic_vector(15 downto 0)) is
            constant first : natural := number_of_received;
        begin
            run(read, address, 1, register_space => true);
            check(not hyperram_out.read_timed_out, "register read timed out");
            check_equal(number_of_received, first + 1, "register read word count");
            check_equal(received(first), expected, "register " & to_hstring(to_unsigned(address, 28)));
        end procedure;

        -- returns the number of wrong words, checks are left to the caller
        procedure write_and_read_back(address : natural; length : natural; errors : out natural) is
            constant first : natural := number_of_received;
            variable bad : natural := 0;
        begin
            for i in 0 to length-1 loop
                write_buffer(i) <= test_pattern(address + i);
            end loop;
            run(write, address, length);
            run(read, address, length);
            if number_of_received /= first + length then
                bad := length;
            else
                for i in 0 to length-1 loop
                    if received(first + i) /= test_pattern(address + i) then
                        bad := bad + 1;
                    end if;
                end loop;
            end if;
            errors := bad;
        end procedure;

        procedure check_write_and_read_back(address : natural; length : natural) is
            variable errors : natural;
        begin
            write_and_read_back(address, length, errors);
            check_equal(errors, 0, "wrong words in a burst of " & integer'image(length) & " at " & integer'image(address));
        end procedure;

        variable errors : natural;
        variable passing, longest, run_length : natural := 0;
        variable report_line : string(1 to 8);


    begin
        test_runner_setup(runner, runner_cfg);
        wait for 1 us;

        while test_suite loop
            -- the calibrator runs after power up, the user side is ready when it is done
            wait until rising_edge(main_clock) and hyperram_is_ready(hyperram_out);
            if run("read_id_and_config_registers") then
                read_register(id_register_0, id_register_0_die0);
                read_register(id_register_0 + die1_offset, id_register_0_die1);
                read_register(id_register_1, id_register_1_value);
                read_register(config_register_0, config_register_0_default);

            elsif run("write_and_read_bursts") then
                check_write_and_read_back(16#100#, 32);
                check_write_and_read_back(16#20#, 1);
                check_write_and_read_back(16#400#, 512);

            elsif run("both_dies") then
                check_write_and_read_back(16#20#, 8);
                check_write_and_read_back(die1_offset + 16#20#, 8);
                check_write_and_read_back(16#20#, 8);

            elsif run("change_latency_to_3") then
                write_buffer(0) <= x"8F" & latency_to_config_bits(3) & x"F";
                run(write, config_register_0, 1, register_space => true);
                command_in.latency_count <= 3;
                read_register(config_register_0, x"8F" & latency_to_config_bits(3) & x"F");
                check_write_and_read_back(16#40#, 64);

            elsif run("power_up_calibration") then
                info("calibration status " & to_hstring(calibration_status) & ": phase " & integer'image(phase)
                     & ", lo first " & boolean'image(lo_first));
                check(calibration_status(0) = '1' and calibration_status(1) = '0', "calibrated without failure");
                check_equal(lo_first, lo_lags, "order of the input halves");
                -- the selected phase and its neighbours passed
                check(calibration_status(8*boolean'pos(lo_first) + 8 + phase) = '1', "selected phase passed");

            elsif run("capture_port_fills_ring") then
                -- 200 records into a ring of 128: records 128..199 overwrite slots 0..71
                capture_reset <= true;
                wait until rising_edge(main_clock);
                capture_reset <= false;
                wait until rising_edge(main_clock) and capture_is_ready for 10 us;
                check(capture_is_ready, "capture port ready after reset");
                check_equal(capture_written, 0, "written count after reset");
                records_to_push <= 150;
                -- a normal command between the capture bursts
                check_write_and_read_back(16#400#, 100);
                wait until rising_edge(main_clock) and records_pushed = 150 for 100 us;
                wait for 2 us;
                check_equal(capture_written, 128, "only whole 64 record bursts without flush");
                records_to_push <= 200;
                capture_flush <= true;
                wait until rising_edge(main_clock) and capture_written = 200 for 100 us;
                check_equal(capture_written, 200, "records written after flush");
                capture_flush <= false;
                for burst in 0 to 3 loop
                    run(read, capture_base + 256*burst, 256);
                end loop;
                check_equal(number_of_received, 100 + 1024, "words read back");
                errors := 0;
                for slot in 0 to ring_records-1 loop
                    for w in 0 to 7 loop
                        if slot < 72 then
                            if received(100 + 8*slot + w) /= capture_word(128 + slot, w) then errors := errors + 1; end if;
                        else
                            if received(100 + 8*slot + w) /= capture_word(slot, w) then errors := errors + 1; end if;
                        end if;
                    end loop;
                end loop;
                check_equal(errors, 0, "wrong capture words in the ring");

                -- a reset with records in the FIFO drops them
                capture_reset <= true;
                wait until rising_edge(main_clock);
                capture_reset <= false;
                records_to_push <= 30;
                wait until rising_edge(main_clock) and records_pushed = 30 for 10 us;
                records_to_push <= 0;
                capture_reset <= true;
                wait until rising_edge(main_clock);
                capture_reset <= false;
                check(not capture_is_ready, "not ready right after reset");
                wait until rising_edge(main_clock) and capture_is_ready for 10 us;
                records_to_push <= 64;
                wait until rising_edge(main_clock) and capture_written = 64 for 100 us;
                check_equal(capture_written, 64, "records after the second reset");
                run(read, capture_base, 512);
                errors := 0;
                for i in 0 to 511 loop
                    if received(100 + 1024 + i) /= capture_word(i/8, i mod 8) then errors := errors + 1; end if;
                end loop;
                check_equal(errors, 0, "wrong words after a reset with records in the FIFO");

            elsif run("long_bursts_after_calibration") then
                for i in 0 to 7 loop
                    check_write_and_read_back(16#1000# + 512*i, 512);
                end loop;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 ms);

end vunit_simulation;
