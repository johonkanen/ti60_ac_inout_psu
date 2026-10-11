LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.hyperram_pkg.all;
    use work.fpga_interconnect_pkg.all;

entity hyperram_tester_tb is
  generic (runner_cfg : string; stuck_dq_bit : integer := -1);
end;

architecture vunit_simulation of hyperram_tester_tb is

    constant clock_period : time := 8.333 ns;  -- 120 MHz
    constant base : natural := 200;

    signal clock : std_logic := '0';

    signal bus_to_tester   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_tester : fpga_interconnect_record := init_fpga_interconnect;

    signal hyperram_in  : hyperram_in_record;
    signal hyperram_out : hyperram_out_record;
    signal fpga_out     : hyperram_fpga_out_record;
    signal fpga_in      : hyperram_fpga_in_record;

    signal ck, cs_n, reset_n : std_logic;
    signal dq   : std_logic_vector(7 downto 0);
    signal rwds : std_logic;
    signal dq_out_reg, dq_in_reg : std_logic_vector(7 downto 0);
    signal dq_oe_reg, rwds_out_reg, rwds_oe_reg, rwds_in_reg : std_logic := '0';

    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

begin

    clock <= not clock after clock_period/2;

------------------------------------------------------------------------
    io_registers : process(clock)
    begin
        if rising_edge(clock) then
            ck           <= fpga_out.ck;
            cs_n         <= fpga_out.cs_n;
            reset_n      <= fpga_out.reset_n;
            dq_out_reg   <= fpga_out.dq_out;
            dq_oe_reg    <= fpga_out.dq_oe;
            rwds_out_reg <= fpga_out.rwds_out;
            rwds_oe_reg  <= fpga_out.rwds_oe;
            dq_in_reg    <= dq;
            rwds_in_reg  <= rwds;
        end if;
    end process;

    dq      <= dq_out_reg when dq_oe_reg = '1' else (others => 'Z');
    rwds    <= rwds_out_reg when rwds_oe_reg = '1' else 'Z';
    fpga_in <= (dq_in => dq_in_reg, rwds_in => rwds_in_reg);

------------------------------------------------------------------------
    u_tester : entity work.hyperram_tester
    generic map(g_base_address => base)
    port map(clock, bus_to_tester, bus_from_tester, hyperram_in, hyperram_out);

    u_controller : entity work.hyperram_controller
    generic map(g_reset_low_clocks => 30, g_power_up_clocks => 100)
    port map(clock, hyperram_in, hyperram_out, fpga_out, fpga_in);

    u_model : entity work.hyperram_model
    generic map(g_stuck_dq_bit => stuck_dq_bit, g_die_crossing_is_error => false)
    port map(ck, cs_n, reset_n, dq, rwds);

------------------------------------------------------------------------
    capture_bus_reads : process(clock)
    begin
        if rising_edge(clock) then
            bus_read_done <= false;
            if write_to_address_is_requested(bus_from_tester, 0) then
                bus_read_data <= get_slv_data(bus_from_tester);
                bus_read_done <= true;
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    main : process

        procedure bus_write(address : natural; data : std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_to_tester, address, data);
            wait until rising_edge(clock);
            init_bus(bus_to_tester);
        end procedure;

        procedure bus_write(address : natural; data : natural) is
        begin
            bus_write(address, std_logic_vector(to_unsigned(data, 32)));
        end procedure;

        procedure bus_read(address : natural; data : out std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_to_tester, address);
            wait until rising_edge(clock);
            init_bus(bus_to_tester);
            -- a stream read may wait for the prefetch
            wait until rising_edge(clock) and bus_read_done for 100 us;
            check(bus_read_done, "no response to bus read from " & integer'image(address));
            data := bus_read_data;
        end procedure;

        procedure wait_status_bit(bit_index : natural) is
            variable data : std_logic_vector(31 downto 0);
        begin
            for i in 0 to 100000 loop
                bus_read(base + 0, data);
                exit when data(bit_index) = '1';
                wait for 2 us;
            end loop;
            check(data(bit_index) = '1', "status bit " & integer'image(bit_index) & " never set");
        end procedure;

        procedure single_read(address : natural; register_space : boolean; expected : std_logic_vector(15 downto 0)) is
            variable data : std_logic_vector(31 downto 0);
        begin
            if register_space then
                bus_write(base + 1, '1' & std_logic_vector(to_unsigned(address, 31)));
            else
                bus_write(base + 1, address);
            end if;
            bus_write(base + 0, 1);
            wait_status_bit(0);
            bus_read(base + 3, data);
            check_equal(data(15 downto 0), expected, "single read of " & integer'image(address));
        end procedure;

        function stream_pattern(i : natural) return std_logic_vector is
            variable product : unsigned(63 downto 0);
        begin
            product := to_unsigned(i + 1, 32) * unsigned'(x"9E3779B1");
            return std_logic_vector(product(31 downto 0)) xor x"5a5a0000";
        end function;

        procedure stream_write_and_read_back(address : natural; words : natural) is
            variable data : std_logic_vector(31 downto 0);
        begin
            bus_write(base + 13, address);
            for i in 0 to words-1 loop
                bus_write(base + 14, stream_pattern(i));
            end loop;
            -- wait for the writes to reach the memory
            loop
                bus_read(base + 15, data);
                exit when data(20) = '0';
                wait for 2 us;
            end loop;
            bus_write(base + 13, address);
            for i in 0 to words-1 loop
                bus_read(base + 14, data);
                if data /= stream_pattern(i) then
                    check_equal(data, stream_pattern(i), "stream word " & integer'image(i) & " at " & integer'image(address));
                    exit;
                end if;
            end loop;
            bus_read(base + 16, data);
            check_equal(to_integer(unsigned(data)), 0, "write FIFO overflows");
        end procedure;

        variable data : std_logic_vector(31 downto 0);

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            wait_status_bit(0);

            if run("single_register_reads") then
                single_read(id_register_0, true, id_register_0_die0);
                single_read(id_register_0 + die1_offset, true, id_register_0_die1);
                single_read(config_register_0, true, config_register_0_default);

            elsif run("single_write_and_read") then
                bus_write(base + 1, 16#123#);
                bus_write(base + 2, 16#beef#);
                bus_write(base + 0, 2);
                wait_status_bit(0);
                single_read(16#123#, false, x"beef");

            elsif run("stream_write_and_read") then
                stream_write_and_read_back(16#100#, 300);

            elsif run("stream_across_die_boundary") then
                stream_write_and_read_back(2**24 - 40, 64);

            elsif run("single_access_while_prefetching") then
                bus_write(base + 13, 16#200#);   -- starts the prefetch
                bus_write(base + 1, 16#123#);
                bus_write(base + 2, 16#beef#);
                bus_write(base + 0, 2);
                wait for 20 us;
                single_read(16#123#, false, x"beef");
                single_read(id_register_0, true, id_register_0_die0);

            elsif run("pattern_test") then
                bus_write(base + 5, 256);
                bus_write(base + 6, 16#5a5a#);
                bus_write(base + 0, 3);
                wait_status_bit(2);
                bus_read(base + 10, data);
                check_equal(to_integer(unsigned(data)), 256, "words checked");
                bus_read(base + 7, data);
                if stuck_dq_bit < 0 then
                    check_equal(to_integer(unsigned(data)), 0, "error count");
                    bus_read(base + 0, data);
                    check(data(3) = '1', "pass bit");
                else
                    check(to_integer(unsigned(data)) > 0, "stuck bit was not detected");
                    bus_read(base + 0, data);
                    check(data(3) = '0', "pass bit set despite errors");
                    bus_read(base + 9, data);
                    info("first error expected/actual " & to_hstring(data));
                    check(data(31 downto 16) /= data(15 downto 0), "first error data");
                end if;
                bus_read(base + 11, data);
                check_equal(to_integer(unsigned(data)), 0, "read timeouts");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
