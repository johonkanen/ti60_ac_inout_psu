LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- Three masters share a register slave through bus_arbiter: master 0 pulses its requests
-- at most every uart_spacing clocks like the UART master at 5 Mbaud, masters 1 and 2 hold them until granted like the UDP bridge. Each
-- master writes and reads back its own registers and reads an unmapped address that times
-- out. The slave answers reads after a varying delay and flags a second read while one is
-- still outstanding. Every response must reach the master that asked for it.
entity bus_arbiter_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of bus_arbiter_tb is

    constant masters        : natural := 3;
    constant timeout        : natural := 64;
    constant unmapped       : natural := 16#7ff#;
    constant operations     : natural := 300;
    constant uart_spacing   : natural := 175;  -- a 7 byte UART command at 5 Mbaud and 125 MHz

    signal clock : std_logic := '0';

    signal requests, responses : bus_array(0 to masters-1) := (others => init_fpga_interconnect);
    signal granted             : boolean_vector(0 to masters-1);
    signal to_slaves           : fpga_interconnect_record := init_fpga_interconnect;
    signal from_slave          : fpga_interconnect_record := init_fpga_interconnect;

    type natural_array is array (0 to masters-1) of natural;
    signal reads_done, errors, responses_seen, reads_expecting_response : natural_array := (others => 0);
    signal finished       : boolean_vector(0 to masters-1) := (others => false);
    signal overlapping_reads : natural := 0;
    signal busy_mode      : boolean := false;

    type register_array is array (0 to 16*masters-1) of std_logic_vector(31 downto 0);
    signal registers : register_array := (others => (others => '0'));

    -- master i owns registers 16*i .. 16*i+15
    function register_address(master, index : natural) return natural is
    begin
        return 16#100# + 16*master + index;
    end function;

    function pattern(master, n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(master, 4)) & std_logic_vector(to_unsigned((n * 7919 + master * 104729) mod 2**28, 28));
    end function;

begin

    clock <= not clock after 4 ns;

    u_arbiter : entity work.bus_arbiter
    generic map(g_masters => masters, g_timeout => timeout)
    port map(clock, requests, responses, granted, to_slaves, from_slave);

    -- registers 0x100.. read/write, everything else unmapped; reads answered 0..19 clocks later
    slave : process(clock)
        variable address      : natural;
        variable delay        : natural := 0;
        variable countdown    : natural := 0;
        variable read_pending : boolean := false;
        variable read_data    : std_logic_vector(31 downto 0);
        variable seed         : natural := 7;
    begin
        if rising_edge(clock) then
            init_bus(from_slave);
            address := to_integer(unsigned(to_slaves.address));
            if to_slaves.data_write_is_requested_with_0 = '0' and address >= 16#100# and address < 16#100# + 16*masters then
                registers(address - 16#100#) <= to_slaves.data;
            end if;
            if read_pending then
                if countdown = 0 then
                    write_data_to_address(from_slave, 0, read_data);
                    read_pending := false;
                else
                    countdown := countdown - 1;
                end if;
            end if;
            if to_slaves.data_read_is_requested_with_0 = '0' then
                if read_pending then
                    overlapping_reads <= overlapping_reads + 1;
                end if;
                if address >= 16#100# and address < 16#100# + 16*masters then
                    seed         := (seed * 75 + 74) mod 65537;
                    countdown    := seed mod 20;
                    read_data    := registers(address - 16#100#);
                    read_pending := true;
                end if;
            end if;
        end if;
    end process;

    response_counter : for i in 0 to masters-1 generate
        process(clock)
        begin
            if rising_edge(clock) then
                if write_to_address_is_requested(responses(i), 0) then
                    responses_seen(i) <= responses_seen(i) + 1;
                end if;
            end if;
        end process;
    end generate;

    master_processes : for i in 0 to masters-1 generate
        process
            variable value : std_logic_vector(31 downto 0);
            variable got_response : boolean;

            -- master 0 pulses its request for one clock, the others hold it until granted
            procedure issue(request : fpga_interconnect_record) is
            begin
                if i = 0 then
                    for n in 1 to uart_spacing loop
                        wait until rising_edge(clock);
                    end loop;
                    requests(i) <= request;
                    wait until rising_edge(clock);
                    requests(i) <= init_fpga_interconnect;
                else
                    loop
                        wait until rising_edge(clock);
                        if granted(i) then
                            requests(i) <= init_fpga_interconnect;
                            exit;
                        end if;
                        requests(i) <= request;
                    end loop;
                end if;
            end procedure;

            procedure write(address : natural; data : std_logic_vector) is
                variable request : fpga_interconnect_record := init_fpga_interconnect;
            begin
                request.address := std_logic_vector(to_unsigned(address, request.address'length));
                request.data    := data;
                request.data_write_is_requested_with_0 := '0';
                issue(request);
            end procedure;

            procedure read(address : natural; expect_response : boolean) is
                variable request : fpga_interconnect_record := init_fpga_interconnect;
            begin
                request.address := std_logic_vector(to_unsigned(address, request.address'length));
                request.data_read_is_requested_with_0 := '0';
                if expect_response then
                    reads_expecting_response(i) <= reads_expecting_response(i) + 1;
                end if;
                issue(request);
                got_response := false;
                for n in 0 to 3*timeout loop
                    if write_to_address_is_requested(responses(i), 0) then
                        got_response := true;
                        value        := get_slv_data(responses(i));
                        exit;
                    end if;
                    wait until rising_edge(clock);
                end loop;
            end procedure;

            procedure fail is
            begin
                errors(i) <= errors(i) + 1;
            end procedure;

        begin
            wait for 100 ns;
            for n in 0 to operations-1 loop
                write(register_address(i, n mod 16), pattern(i, n));
                read(register_address(i, n mod 16), true);
                if not got_response or value /= pattern(i, n) then
                    fail;
                end if;
                reads_done(i) <= reads_done(i) + 1;
                if n mod 50 = 7 and not busy_mode then
                    read(unmapped, false);
                    if got_response then
                        fail;
                    end if;
                end if;
                if not busy_mode then
                    wait for (13 + 7*i + n mod 5) * 8 ns;
                end if;
            end loop;
            finished(i) <= true;
            wait;
        end process;
    end generate;

    main : process
    begin
        test_runner_setup(runner, runner_cfg);
        if run("masters_with_gaps") then
            busy_mode <= false;
        elsif run("all_masters_busy") then
            busy_mode <= true;
        end if;
        -- round robin: when the first fast master is done, the other one is close behind
        wait until finished(1) or finished(2) for 2 ms;
        for i in 1 to masters-1 loop
            check(reads_done(i) >= operations*3/4 or not busy_mode, "master " & integer'image(i) & " starved");
        end loop;
        wait until finished = (0 to masters-1 => true) for 2 ms;
        wait for 1 us;
        for i in 0 to masters-1 loop
            check_equal(reads_done(i), operations, "master " & integer'image(i) & " did not finish");
            check_equal(errors(i), 0, "master " & integer'image(i) & " read wrong data or got a response for a timed out read");
            check_equal(responses_seen(i), reads_expecting_response(i), "master " & integer'image(i) & " responses");
        end loop;
        check_equal(overlapping_reads, 0, "a read was issued while another was outstanding");
        test_runner_cleanup(runner);
    end process;

    test_runner_watchdog(runner, 5 ms);

end vunit_simulation;
