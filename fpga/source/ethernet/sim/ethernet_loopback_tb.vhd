LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.ethernet_pkg.all;
    use work.fpga_interconnect_pkg.all;
    use std.textio.all;

-- The ethernet entity at its RGMII pins, through simple DDIO and pin models.
-- With loopback the transmit pins drive the receive pins, as in PHY loopback; otherwise
-- the testbench plays the PHY: it sends a ping into the receive pins and decodes the reply
-- from the transmit pins.
-- rx_lo_lags selects whether the receive DDIO model pairs a rising edge nibble with the
-- falling edge before it (true) or after it (false).
entity ethernet_loopback_tb is
  generic (runner_cfg : string; frame_directory : string;
           rx_lo_lags : boolean := false; loop_delay_ns : real := 2.0);
end;

architecture vunit_simulation of ethernet_loopback_tb is

    constant clock_period : time := 8 ns;
    constant base : natural := 400;

    signal clock    : std_logic := '0';
    signal rx_clock : std_logic := '0';

    signal bus_to_tester   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_tester : fpga_interconnect_record := init_fpga_interconnect;

    signal rgmii_tx, tx_clock_ddio, rgmii_rx : rgmii_ddio_record := rgmii_idle;
    signal tx_pins, rx_pins : std_logic_vector(4 downto 0) := (others => '0');
    signal txc_pin : std_logic := '0';
    signal rx_hi, rx_lo, rx_lo_previous : std_logic_vector(4 downto 0) := (others => '0');

    signal fpga_in  : ethernet_fpga_in_record;
    signal fpga_out : ethernet_fpga_out_record;

    signal loopback   : boolean := true;
    signal phy_pins   : std_logic_vector(4 downto 0) := (others => '0');
    signal tx_capture_enable : boolean := false;
    type byte_array is array (natural range <>) of byte;
    signal tx_bytes   : byte_array(0 to 16383);
    signal tx_count   : natural := 0;
    type start_array is array (0 to 63) of natural;
    signal tx_frame_starts : start_array := (others => 0);
    signal tx_frames  : natural := 0;
    signal tx_was_enabled : boolean := false;

    -- data source of the UDP stream: a known word per address, three clocks latency
    signal stream_read_address : natural := 0;
    signal stream_read_data    : std_logic_vector(31 downto 0) := (others => '0');
    type word_pipeline is array (1 to 2) of std_logic_vector(31 downto 0);
    signal stream_pipeline     : word_pipeline := (others => (others => '0'));

    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

begin

    clock    <= not clock after clock_period/2;
    -- the PHY rx clock skew centres the clock in the data eye, a quarter period
    rx_clock <= transport clock after loop_delay_ns * 1 ns + clock_period/4;

------------------------------------------------------------------------
    -- DDIO output model: both halves are registered on the rising edge, hi is driven
    -- for the first half of the clock period and lo for the second
    tx_ddio : process(clock)
        variable lo_hold  : std_logic_vector(4 downto 0) := (others => '0');
        variable txc_hold : std_logic := '0';
    begin
        if rising_edge(clock) then
            tx_pins  <= fpga_out.rgmii_tx.hi;
            txc_pin  <= fpga_out.tx_clock.hi(0);
            lo_hold  := fpga_out.rgmii_tx.lo;
            txc_hold := fpga_out.tx_clock.lo(0);
        end if;
        if falling_edge(clock) then
            tx_pins <= lo_hold;
            txc_pin <= txc_hold;
        end if;
    end process;

    -- phy loopback or the testbench as the phy
    rx_pins <= transport tx_pins after loop_delay_ns * 1 ns when loopback else phy_pins;

    -- transmit pin decoder: TX_EN is high during the frame, a byte is the rising edge
    -- nibble followed by the falling edge nibble
    tx_decoder : process
        variable low_nibble : std_logic_vector(3 downto 0);
    begin
        wait until rising_edge(clock);
        wait for 2 ns;
        tx_was_enabled <= tx_pins(4) = '1';
        if tx_capture_enable and tx_pins(4) = '1' then
            if not tx_was_enabled then
                tx_frame_starts(tx_frames) <= tx_count;
                tx_frames <= tx_frames + 1;
            end if;
            low_nibble := tx_pins(3 downto 0);
            wait until falling_edge(clock);
            wait for 2 ns;
            tx_bytes(tx_count) <= tx_pins(3 downto 0) & low_nibble;
            tx_count <= tx_count + 1;
        end if;
    end process;

    -- DDIO input model with resync to the rising edge
    rx_ddio : process(rx_clock)
    begin
        if rising_edge(rx_clock) then
            rx_hi          <= rx_pins;
            rx_lo_previous <= rx_lo;
            if rx_lo_lags then
                rgmii_rx <= (hi => rx_pins, lo => rx_lo);
            else
                rgmii_rx <= (hi => rx_hi, lo => rx_lo);
            end if;
        end if;
        if falling_edge(rx_clock) then
            rx_lo <= rx_pins;
        end if;
    end process;

------------------------------------------------------------------------
    fpga_in <= (rgmii_rx => rgmii_rx, rx_pll_locked => '1', mdio_in => '1');

    stream_source : process(clock)
        variable product : unsigned(63 downto 0);
    begin
        if rising_edge(clock) then
            -- like scope_capture: two registers after the reader's address register
            product := to_unsigned(stream_read_address, 32) * unsigned'(x"9E3779B1");
            stream_pipeline(1) <= std_logic_vector(product(31 downto 0));
            stream_read_data   <= stream_pipeline(1);
        end if;
    end process;

    u_ethernet : entity work.ethernet
    generic map(g_base_address => base - 100)   -- loopback tester at base
    port map(
        clock               => clock
        ,bus_in             => bus_to_tester
        ,bus_out            => bus_from_tester
        ,bridge_bus_out     => open
        ,bridge_bus_in      => init_fpga_interconnect
        ,request_is_granted => false
        ,stream_read_address => stream_read_address
        ,stream_read_data    => stream_read_data
        ,rx_clock           => rx_clock
        ,fpga_in            => fpga_in
        ,fpga_out           => fpga_out
    );

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
        variable request, expected : byte_array(0 to 2047);
        variable request_length, expected_length : natural;
        file datagram_file : text;
        variable datagram_line : line;
        variable datagram_index, start : natural;

        procedure bus_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_to_tester, address, std_logic_vector(to_unsigned(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_to_tester);
        end procedure;

        procedure parse_hex(l : inout line; bytes : out byte_array; count : out natural) is
            variable c : character;
            variable value, n : natural := 0;
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

        procedure read_frame_file(name : string; bytes : out byte_array; count : out natural) is
            file f : text;
            variable l : line;
            variable c : character;
            variable value, n : natural := 0;
            variable good : boolean;
        begin
            file_open(f, frame_directory & "/" & name, read_mode);
            readline(f, l);
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

        -- the testbench as the phy: preamble, SFD and the frame as RGMII nibbles
        procedure phy_send(bytes : byte_array; count : natural) is
            -- each nibble is set up half a period before the edge that samples it
            procedure send_byte(b : byte) is
            begin
                wait until falling_edge(rx_clock);
                wait for 2 ns;
                phy_pins <= '1' & b(3 downto 0);   -- sampled on the rising edge
                wait until rising_edge(rx_clock);
                wait for 2 ns;
                phy_pins <= '1' & b(7 downto 4);   -- sampled on the falling edge
            end procedure;
        begin
            for i in 1 to 7 loop send_byte(preamble_byte); end loop;
            send_byte(start_of_frame_byte);
            for i in 0 to count-1 loop send_byte(bytes(i)); end loop;
            wait until falling_edge(rx_clock);
            wait for 2 ns;
            phy_pins <= (others => '0');
        end procedure;

        procedure bus_read(address : natural; data : out natural) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_to_tester, address);
            wait until rising_edge(clock);
            init_bus(bus_to_tester);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := to_integer(unsigned(bus_read_data(30 downto 0)));
        end procedure;

        procedure send_and_check(frames : natural; length : natural) is
            variable status, sent, received, good, crc_errors, payload_errors, gaps, frame_length, alignment : natural;
        begin
            bus_write(base + 0, 4);   -- clear receive counters
            wait for 200 ns;
            bus_write(base + 1, frames);
            bus_write(base + 2, length);
            bus_write(base + 0, 1);
            loop
                wait for 2 us;
                bus_read(base + 0, status);
                exit when status mod 2 = 0;
            end loop;
            wait for 2 us;
            bus_read(base + 5, received);
            bus_read(base + 6, good);
            bus_read(base + 7, crc_errors);
            bus_read(base + 8, payload_errors);
            bus_read(base + 9, gaps);
            bus_read(base + 10, frame_length);
            bus_read(base + 11, alignment);
            info("received " & integer'image(received) & " good " & integer'image(good)
                 & " crc errors " & integer'image(crc_errors) & " payload errors " & integer'image(payload_errors)
                 & " gaps " & integer'image(gaps) & " length " & integer'image(frame_length)
                 & " alignment " & integer'image(alignment));
            check_equal(received, frames, "frames received");
            check_equal(good, frames, "good frames");
            check_equal(crc_errors, 0, "crc errors");
            check_equal(payload_errors, 0, "payload errors");
            check_equal(gaps, 0, "sequence gaps");
            check_equal(frame_length, 14 + length + 4, "frame length");
        end procedure;

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("short_frames") then
                send_and_check(20, 46);
            elsif run("long_frames") then
                send_and_check(3, 1500);
            elsif run("repeated_bursts") then
                send_and_check(5, 100);
                send_and_check(5, 64);
            elsif run("stream_through_rgmii_pins") then
                loopback <= false;
                read_frame_file("stream_request.txt", request, request_length);
                tx_capture_enable <= true;
                phy_send(request, request_length);
                wait for 60 us;
                file_open(datagram_file, frame_directory & "/stream_datagrams.txt", read_mode);
                datagram_index := 0;
                while not endfile(datagram_file) loop
                    readline(datagram_file, datagram_line);
                    parse_hex(datagram_line, expected, expected_length);
                    check(datagram_index < tx_frames, "datagram " & integer'image(datagram_index) & " missing");
                    exit when datagram_index >= tx_frames;
                    start := tx_frame_starts(datagram_index);
                    check_equal(tx_bytes(start + 7), start_of_frame_byte, "SFD of datagram " & integer'image(datagram_index));
                    for i in 0 to expected_length-1 loop
                        if tx_bytes(start + 8 + i) /= expected(i) then
                            check_equal(tx_bytes(start + 8 + i), expected(i),
                                "datagram " & integer'image(datagram_index) & " byte " & integer'image(i));
                            exit;
                        end if;
                    end loop;
                    datagram_index := datagram_index + 1;
                end loop;
                info("compared " & integer'image(datagram_index) & " datagrams, " & integer'image(tx_frames) & " frames sent");
                check_equal(tx_frames, datagram_index, "number of frames sent");

            elsif run("ping_reply_through_rgmii_pins") then
                loopback <= false;
                read_frame_file("single_ping_request.txt", request, request_length);
                read_frame_file("single_ping_reply.txt", expected, expected_length);
                tx_capture_enable <= true;
                phy_send(request, request_length);
                wait for 20 us;
                info("captured " & integer'image(tx_count) & " bytes from the transmit pins");
                check_equal(tx_count, 8 + expected_length, "reply length with preamble and SFD");
                for i in 0 to 6 loop
                    check_equal(tx_bytes(i), preamble_byte, "preamble");
                end loop;
                check_equal(tx_bytes(7), start_of_frame_byte, "SFD");
                for i in 0 to expected_length-1 loop
                    if tx_bytes(8 + i) /= expected(i) then
                        check_equal(tx_bytes(8 + i), expected(i), "reply byte " & integer'image(i));
                        exit;
                    end if;
                end loop;
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 5 ms);

end vunit_simulation;
