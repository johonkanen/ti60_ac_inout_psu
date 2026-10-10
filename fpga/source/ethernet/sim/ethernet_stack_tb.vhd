LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.ethernet_pkg.all;
    use work.fpga_interconnect_pkg.all;

-- Feeds the frames written by ethernet_test_frames.py into ethernet_stack and compares
-- what it sends back with the expected replies, byte by byte including the FCS.
entity ethernet_stack_tb is
  generic (runner_cfg : string; frame_directory : string);
end;

architecture vunit_simulation of ethernet_stack_tb is

    signal clock    : std_logic := '0';
    signal rx_clock : std_logic := '0';

    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal rx_gmii         : gmii_record := gmii_idle;
    signal rx_frame_ended  : boolean := false;
    signal tx_gmii         : gmii_record;

    type byte_array is array (natural range <>) of byte;
    signal captured       : byte_array(0 to 16383);
    signal captured_count : natural := 0;
    signal capture_clear  : boolean := false;
    type start_array is array (0 to 63) of natural;
    signal frame_starts   : start_array := (others => 0);
    signal frames_captured : natural := 0;
    signal tx_was_valid   : boolean := false;

    -- bus: the bridge and a UART like master share a register slave through bus_arbiter
    signal bridge_request, bridge_response   : fpga_interconnect_record := init_fpga_interconnect;
    signal uart_request, uart_response       : fpga_interconnect_record := init_fpga_interconnect;
    signal to_slaves, from_slave, from_slave_registered : fpga_interconnect_record := init_fpga_interconnect;
    signal request_is_granted : boolean;
    signal master_granted     : boolean_vector(0 to 1);
    type register_array is array (0 to 15) of std_logic_vector(31 downto 0);
    function initial_registers return register_array is
        variable r : register_array;
    begin
        for i in 0 to 15 loop
            r(i) := x"A50000" & std_logic_vector(to_unsigned(i, 8));
        end loop;
        return r;
    end function;
    signal registers : register_array := initial_registers;
    signal uart_reads, uart_errors : natural := 0;

begin

    clock    <= not clock after 4 ns;
    rx_clock <= not rx_clock after 4.001 ns;  -- independent receive clock

    u_stack : entity work.ethernet_stack
    port map(clock => clock, bus_in => bus_in, bus_out => bus_out, rx_clock => rx_clock,
             rx_gmii_in => rx_gmii, rx_frame_is_ended => rx_frame_ended, tx_gmii_out => tx_gmii,
             tx_wants_to_send => open, tx_is_sending => open, stream_request => open,
             bridge_bus_out => bridge_request, bridge_bus_in => bridge_response,
             request_is_granted => request_is_granted);

    u_arbiter : entity work.bus_arbiter
    generic map(g_masters => 2)
    port map(clock, requests(0) => uart_request, requests(1) => bridge_request,
             responses(0) => uart_response, responses(1) => bridge_response,
             granted => master_granted, to_slaves => to_slaves, from_slaves => from_slave_registered);
    request_is_granted <= master_granted(1);

    -- registers 0x100..0x10f read/write, 0x110 read only, everything else unmapped
    register_slave : process(clock)
        variable address : natural;
    begin
        if rising_edge(clock) then
            init_bus(from_slave);
            from_slave_registered <= from_slave;  -- the bus combining register of the top level
            address := to_integer(unsigned(to_slaves.address));
            if to_slaves.data_write_is_requested_with_0 = '0' and address >= 16#100# and address < 16#110# then
                registers(address - 16#100#) <= to_slaves.data;
            end if;
            if to_slaves.data_read_is_requested_with_0 = '0' then
                if address >= 16#100# and address < 16#110# then
                    write_data_to_address(from_slave, 0, registers(address - 16#100#));
                elsif address = 16#110# then
                    write_data_to_address(from_slave, 0, std_logic_vector'(x"CAFE0110"));
                end if;
            end if;
        end if;
    end process;

    -- UART like master reading the read only register throughout the test
    uart_master : process
    begin
        wait for 1 us;
        loop
            wait until rising_edge(clock);
            request_data_from_address(uart_request, 16#110#);
            wait until rising_edge(clock);
            init_bus(uart_request);
            for i in 0 to 200 loop
                wait until rising_edge(clock);
                exit when write_to_address_is_requested(uart_response, 0);
            end loop;
            if write_to_address_is_requested(uart_response, 0) and get_slv_data(uart_response) = x"CAFE0110" then
                uart_reads <= uart_reads + 1;
            else
                uart_errors <= uart_errors + 1;
            end if;
            wait for 173 ns;
        end loop;
    end process;

    capture : process(clock)
    begin
        if rising_edge(clock) then
            tx_was_valid <= tx_gmii.is_valid;
            if capture_clear then
                captured_count  <= 0;
                frames_captured <= 0;
            elsif tx_gmii.is_valid then
                captured(captured_count) <= tx_gmii.data;
                captured_count <= captured_count + 1;
                if not tx_was_valid then
                    frame_starts(frames_captured) <= captured_count;
                    frames_captured <= frames_captured + 1;
                end if;
            end if;
        end if;
    end process;

    main : process
        file requests, replies, names : text;
        variable request_line, reply_line, name_line : line;
        variable frame, expected : byte_array(0 to 2047);
        variable frame_length, expected_length, start : natural;

        procedure parse(l : inout line; bytes : out byte_array; count : out natural) is
            variable c : character;
            variable value : natural;
            variable n : natural := 0;
            variable good : boolean;
        begin
            while l'length >= 2 loop
                value := 0;
                for i in 1 to 2 loop
                    read(l, c, good);
                    if c >= '0' and c <= '9' then value := value*16 + character'pos(c) - character'pos('0');
                    else value := value*16 + character'pos(c) - character'pos('a') + 10; end if;
                end loop;
                bytes(n) := std_logic_vector(to_unsigned(value, 8));
                n := n + 1;
            end loop;
            count := n;
        end procedure;

        procedure send_frame(bytes : byte_array; count : natural) is
        begin
            for i in 0 to count-1 loop
                wait until rising_edge(rx_clock);
                rx_gmii <= (data => bytes(i), is_valid => true, is_error => false);
            end loop;
            wait until rising_edge(rx_clock);
            rx_gmii        <= gmii_idle;
            rx_frame_ended <= true;
            wait until rising_edge(rx_clock);
            rx_frame_ended <= false;
        end procedure;

        procedure bus_read(address : natural; data : out natural) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            for i in 0 to 20 loop
                wait until rising_edge(clock);
                if write_to_address_is_requested(bus_out, 0) then
                    data := to_integer(unsigned(get_slv_data(bus_out)(30 downto 0)));
                    return;
                end if;
            end loop;
            check(false, "no response from stack register " & integer'image(address));
        end procedure;

        -- compares captured frame n (preamble and SFD included) with an expected frame
        impure function captured_frame_matches(n : natural; expected_frame : byte_array; count : natural) return boolean is
            variable start : natural := frame_starts(n);
        begin
            if captured(start + 7) /= start_of_frame_byte then
                return false;
            end if;
            for i in 0 to count-1 loop
                if captured(start + 8 + i) /= expected_frame(i) then
                    return false;
                end if;
            end loop;
            return true;
        end function;

        procedure burst_test(name : string; minimum_replies : natural) is
            file burst_requests, burst_replies : text;
            type frame_store is array (0 to 15) of byte_array(0 to 2047);
            variable request_frames, reply_frames : frame_store;
            variable request_lengths, reply_lengths : start_array;
            variable frame_count, next_expected, matched, dropped_before, dropped_after : natural := 0;
            variable l : line;
        begin
            file_open(burst_requests, frame_directory & "/" & name & "_requests.txt", read_mode);
            file_open(burst_replies,  frame_directory & "/" & name & "_replies.txt", read_mode);
            while not endfile(burst_requests) loop
                readline(burst_requests, l);
                parse(l, request_frames(frame_count), request_lengths(frame_count));
                readline(burst_replies, l);
                parse(l, reply_frames(frame_count), reply_lengths(frame_count));
                frame_count := frame_count + 1;
            end loop;

            bus_read(505, dropped_before);
            wait until rising_edge(clock);
            capture_clear <= true;
            wait until rising_edge(clock);
            capture_clear <= false;

            -- back to back: interframe gap and preamble are 20 byte times on the wire
            for i in 0 to frame_count-1 loop
                send_frame(request_frames(i), request_lengths(i));
                for gap in 1 to 19 loop
                    wait until rising_edge(rx_clock);
                end loop;
            end loop;
            wait for 200 us;

            -- every reply must belong to a request, in order, none twice
            for n in 0 to frames_captured-1 loop
                while next_expected < frame_count
                    and not captured_frame_matches(n, reply_frames(next_expected), reply_lengths(next_expected)) loop
                    next_expected := next_expected + 1;
                end loop;
                check(next_expected < frame_count, name & ": reply " & integer'image(n) & " matches no request in order");
                next_expected := next_expected + 1;
                matched := matched + 1;
            end loop;
            bus_read(505, dropped_after);
            info(name & ": " & integer'image(frame_count) & " requests, " & integer'image(matched)
                 & " replies, " & integer'image(dropped_after - dropped_before) & " dropped for a full buffer");
            check(matched >= minimum_replies, name & ": too few replies");
            check_equal(matched + dropped_after - dropped_before, frame_count, name & ": replies and drops");
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);
        while test_suite loop
        if run("three_frames_back_to_back") then
            burst_test("burst3", 3);
        elsif run("overload") then
            burst_test("overload", 3);
        elsif run("frames_one_at_a_time") then
        file_open(requests, frame_directory & "/requests.txt", read_mode);
        file_open(replies,  frame_directory & "/replies.txt", read_mode);
        file_open(names,    frame_directory & "/names.txt", read_mode);

        while not endfile(requests) loop
            readline(requests, request_line);
            readline(replies, reply_line);
            readline(names, name_line);
            parse(request_line, frame, frame_length);

            wait until rising_edge(clock);
            capture_clear <= true;
            wait until rising_edge(clock);
            capture_clear <= false;

            send_frame(frame, frame_length);
            -- wait for a reply to start and then for the transmitter to go quiet
            wait until tx_gmii.is_valid for 200 us;
            loop
                wait until tx_gmii.is_valid for 2 us;
                exit when not tx_gmii.is_valid;
            end loop;

            if reply_line.all = "none" then
                check_equal(captured_count, 0, name_line.all & ": unexpected reply");
            else
                parse(reply_line, expected, expected_length);
                -- strip preamble and SFD
                start := 0;
                while start < captured_count and captured(start) = preamble_byte loop
                    start := start + 1;
                end loop;
                check(start = 7 and captured(start) = start_of_frame_byte, name_line.all & ": preamble");
                start := start + 1;
                check_equal(captured_count - start, expected_length, name_line.all & ": reply length");
                for i in 0 to expected_length-1 loop
                    if captured(start + i) /= expected(i) then
                        check_equal(captured(start + i), expected(i), name_line.all & ": byte " & integer'image(i));
                        exit;
                    end if;
                end loop;
            end if;
            info(name_line.all & " done");
        end loop;

        info("uart reads during the test " & integer'image(uart_reads) & ", errors " & integer'image(uart_errors));
        check(uart_reads > 100, "uart master was not served");
        check_equal(uart_errors, 0, "uart reads with wrong or missing response");
        end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
