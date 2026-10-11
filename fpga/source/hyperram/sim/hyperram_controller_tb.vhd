LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.hyperram_pkg.all;

entity hyperram_controller_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of hyperram_controller_tb is

    constant clock_period : time := 8.333 ns;  -- 120 MHz

    signal clock : std_logic := '0';

    signal hyperram_in  : hyperram_in_record := init_hyperram_in;
    signal command_in   : hyperram_in_record := init_hyperram_in;  -- driven by main
    signal write_in     : hyperram_in_record := init_hyperram_in;  -- driven by user_side
    signal hyperram_out : hyperram_out_record;
    signal fpga_out     : hyperram_fpga_out_record;
    signal fpga_in      : hyperram_fpga_in_record;

    -- pins, outputs and inputs pass through IO registers as on the FPGA
    signal ck, cs_n, reset_n : std_logic;
    signal dq   : std_logic_vector(7 downto 0);
    signal rwds : std_logic;
    signal dq_out_reg, dq_in_reg : std_logic_vector(7 downto 0);
    signal dq_oe_reg, rwds_out_reg, rwds_oe_reg, rwds_in_reg : std_logic := '0';

    type word_array is array (natural range <>) of std_logic_vector(15 downto 0);
    signal received : word_array(0 to 63);
    signal number_of_received : natural := 0;
    signal write_buffer : word_array(0 to 63);
    signal words_written : natural := 0;

    function test_pattern(address : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(address, 16) xor x"a5c3");
    end function;

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

    hyperram_in <= (
        command          => command_in.command,
        register_space   => command_in.register_space,
        address          => command_in.address,
        burst_length     => command_in.burst_length,
        write_data       => write_in.write_data,
        write_data_valid => write_in.write_data_valid,
        latency_count    => command_in.latency_count);

------------------------------------------------------------------------
    u_controller : entity work.hyperram_controller
    generic map(g_reset_low_clocks => 30, g_power_up_clocks => 100)
    port map(clock, hyperram_in, hyperram_out, fpga_out, fpga_in);

    u_model : entity work.hyperram_model
    generic map(g_die_crossing_is_error => false)
    port map(ck, cs_n, reset_n, dq, rwds);

------------------------------------------------------------------------
    -- user side of the controller: supplies write data and collects read data
    user_side : process(clock)
    begin
        if rising_edge(clock) then
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

        procedure wait_until_ready is
        begin
            wait until rising_edge(clock) and hyperram_is_ready(hyperram_out);
        end procedure;

        procedure run(command : hyperram_command; address : natural; length : natural; register_space : boolean := false) is
        begin
            wait_until_ready;
            command_in.command        <= command;
            command_in.address        <= address;
            command_in.burst_length   <= length;
            command_in.register_space <= register_space;
            wait until rising_edge(clock);
            command_in.command <= none;
            wait until rising_edge(clock) and hyperram_transaction_is_done(hyperram_out);
            check(not hyperram_out.read_timed_out, "read timed out");
        end procedure;

        procedure read_register(address : natural; expected : std_logic_vector(15 downto 0)) is
            constant first : natural := number_of_received;
        begin
            run(read, address, 1, register_space => true);
            check_equal(number_of_received, first + 1, "register read word count");
            check_equal(received(first), expected, "register " & to_hstring(to_unsigned(address, 28)));
        end procedure;

        procedure write_and_read_back(address : natural; length : natural) is
            constant first : natural := number_of_received;
        begin
            for i in 0 to length-1 loop
                write_buffer(i) <= test_pattern(address + i);
            end loop;
            run(write, address, length);
            check_equal(words_written, length, "write words requested");
            run(read, address, length);
            check_equal(number_of_received, first + length, "read word count");
            for i in 0 to length-1 loop
                check_equal(received(first + i), test_pattern(address + i), "word " & integer'image(i));
            end loop;
        end procedure;

        procedure set_latency(latency : natural) is
            constant cr0 : std_logic_vector(15 downto 0) := x"8F" & latency_to_config_bits(latency) & x"F";
        begin
            write_buffer(0) <= cr0;
            run(write, config_register_0, 1, register_space => true);
            command_in.latency_count <= latency;
            read_register(config_register_0, cr0);
            read_register(config_register_0 + die1_offset, cr0);
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("read_id_and_config_registers") then
                read_register(id_register_0, id_register_0_die0);
                read_register(id_register_0 + die1_offset, id_register_0_die1);
                read_register(id_register_1, id_register_1_value);
                read_register(config_register_0, config_register_0_default);

            elsif run("write_and_read_burst") then
                write_and_read_back(16#100#, 32);

            elsif run("write_and_read_both_dies") then
                write_and_read_back(16#20#, 8);
                write_and_read_back(die1_offset + 16#20#, 8);
                write_and_read_back(16#20#, 1);

            elsif run("change_latency_to_3") then
                set_latency(3);
                write_and_read_back(16#40#, 32);
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 2 ms);

end vunit_simulation;
