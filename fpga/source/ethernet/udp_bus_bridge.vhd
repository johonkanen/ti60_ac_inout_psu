library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;
    use work.fpga_interconnect_pkg.all;

-- Protocol module executing fpga_interconnect commands carried in UDP datagrams to
-- g_udp_port, with the UART encoding:
--   request : sequence(15:0), then commands back to back
--             0x02 address(15:0)                     read
--             0x04 address(15:0) data(31:0)          write
--             0x05 address(15:0) count(23:0)         read the same address count times
--   reply   : sequence(15:0), then 32 bit results of the reads in order, big endian
-- Parsing stops at an unknown command, a truncated command or when the reply is full.
-- A read without a response is answered with 0xFFFFFFFF. The reply payload is written
-- into the transmit slot as the commands run, the reply header last.
entity udp_bus_bridge is
    generic(
        g_own_mac     : mac_address;
        g_own_ip      : ip_address;
        g_udp_port    : natural := 5001;
        g_max_results : natural := 367;   -- 2 + 4*367 bytes fills a 1472 byte UDP payload
        g_timeout     : natural := 2048   -- a HyperRAM stream read may wait for the prefetch
    );
    port(
        clock              : in std_logic;
        port_in            : in frame_port_in_record;
        port_out           : out frame_port_out_record;

        bus_out            : out fpga_interconnect_record;
        bus_in             : in fpga_interconnect_record;
        request_is_granted : in boolean;

        bus_timeouts       : out unsigned(31 downto 0)
    );
end entity udp_bus_bridge;

architecture rtl of udp_bus_bridge is

    constant read_command   : byte := x"02";
    constant write_command  : byte := x"04";
    constant stream_command : byte := x"05";
    constant payload_offset : natural := header_length;

    type states is (idle, capture, check_ip, decide, wait_for_tx_slot,
                    fetch, parse, bus_request, bus_wait, store, write_header, send);
    signal state : states := idle;
    type stages is (sequence_number, command, arguments);
    signal stage : stages := sequence_number;

    signal offset     : rx_frame_address := 0;
    signal length     : frame_length_type := 0;
    signal tx_offset  : tx_frame_address := 0;
    signal header     : header_array := (others => (others => '0'));
    signal index      : natural range 0 to slot_size := 0;
    signal checksum   : std_logic_vector(15 downto 0) := (others => '0');
    signal word_index : natural range 0 to 10 := 0;
    signal lengths    : reply_length_fields := no_length_change;

    type index_array is array (0 to frame_read_latency-1) of natural range 0 to slot_size-1;
    signal pending       : std_logic_vector(frame_read_latency-1 downto 0) := (others => '0');
    signal pending_index : index_array := (others => 0);

    signal payload_length : natural range 0 to slot_size := 0;
    signal position       : natural range 0 to slot_size := 0;
    signal need           : natural range 0 to 6 := 0;
    signal fetch_phase    : natural range 0 to frame_read_latency := 0;
    signal arguments_register : std_logic_vector(47 downto 0) := (others => '0');
    signal current_command    : byte := (others => '0');

    signal address     : natural range 0 to 2**16-1 := 0;
    signal write_value : std_logic_vector(31 downto 0) := (others => '0');
    signal is_read     : boolean := false;
    signal reads_left  : natural range 0 to 2**24-1 := 0;
    signal results     : natural range 0 to g_max_results := 0;
    signal timer       : natural range 0 to g_timeout := 0;
    signal timeouts    : unsigned(31 downto 0) := (others => '0');

    signal store_word   : std_logic_vector(31 downto 0) := (others => '0');
    signal store_left   : natural range 0 to 4 := 0;
    signal store_return : stages := command;
    signal reply_bytes  : natural range 0 to slot_size := 0;

    function pipeline_is_empty(p : std_logic_vector) return boolean is
    begin
        return p = (p'range => '0');
    end function;

begin

    bus_timeouts <= timeouts;

    process(clock)
        procedure read_frame_byte(i : natural) is
        begin
            port_out.read_address <= offset + i;
            pending(0)       <= '1';
            pending_index(0) <= i;
        end procedure;

        procedure fetch_bytes(count : natural; next_stage : stages) is
        begin
            need        <= count;
            stage       <= next_stage;
            fetch_phase <= 0;
            state       <= fetch;
        end procedure;

        variable datagram_length : natural;
        variable new_ip_length   : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clock) then
            init_frame_port(port_out);
            init_bus(bus_out);
            pending       <= pending(frame_read_latency-2 downto 0) & '0';
            pending_index <= 0 & pending_index(0 to frame_read_latency-2);

            CASE state is
                WHEN idle =>
                    if port_in.frame_is_ready then
                        offset <= port_in.offset;
                        length <= port_in.length;
                        index  <= 0;
                        state  <= capture;
                    end if;

                WHEN capture =>
                    if pending(frame_read_latency-1) = '1' then
                        header(pending_index(frame_read_latency-1)) <= port_in.read_data;
                    end if;
                    if index < header_length then
                        read_frame_byte(index);
                        index <= index + 1;
                    elsif pipeline_is_empty(pending) then
                        checksum   <= (others => '0');
                        word_index <= 0;
                        state      <= check_ip;
                    end if;

                WHEN check_ip =>
                    if word_index < 10 then
                        checksum   <= ones_complement_add(checksum, get_bytes(header, ip_version_ihl + 2*word_index, 2));
                        word_index <= word_index + 1;
                    else
                        state <= decide;
                    end if;

                WHEN decide =>
                    -- payload bytes, limited to what was received
                    datagram_length := to_integer(unsigned(get_bytes(header, udp_length, 2)));
                    if datagram_length < 8 then
                        payload_length <= 0;
                    elsif datagram_length - 8 > length - header_length then
                        payload_length <= length - header_length;
                    else
                        payload_length <= datagram_length - 8;
                    end if;
                    if is_ipv4_for(header, g_own_ip, checksum) and is_udp_to_port(header, g_udp_port)
                        and length >= header_length
                    then
                        state <= wait_for_tx_slot;
                    else
                        port_out.release_frame <= true;
                        state <= idle;
                    end if;

                WHEN wait_for_tx_slot =>
                    if port_in.tx_slot_is_free then
                        tx_offset   <= port_in.tx_offset;
                        position    <= 0;
                        results     <= 0;
                        reply_bytes <= 0;
                        fetch_bytes(2, sequence_number);
                    end if;

                -- read 'need' payload bytes into the low end of arguments_register
                WHEN fetch =>
                    if need = 0 then
                        state <= parse;
                    elsif position >= payload_length then
                        index <= 0;
                        state <= write_header;   -- truncated
                    elsif fetch_phase = 0 then
                        read_frame_byte(payload_offset + position);
                        fetch_phase <= 1;
                    elsif fetch_phase < frame_read_latency then
                        fetch_phase <= fetch_phase + 1;
                    else
                        arguments_register <= arguments_register(39 downto 0) & port_in.read_data;
                        position    <= position + 1;
                        need        <= need - 1;
                        fetch_phase <= 0;
                    end if;

                WHEN parse =>
                    CASE stage is
                        WHEN sequence_number =>
                            store_word   <= arguments_register(15 downto 0) & x"0000";
                            store_left   <= 2;
                            store_return <= command;
                            state        <= store;

                        WHEN command =>
                            current_command <= arguments_register(7 downto 0);
                            CASE arguments_register(7 downto 0) is
                                WHEN read_command   => fetch_bytes(2, arguments);
                                WHEN write_command  => fetch_bytes(6, arguments);
                                WHEN stream_command => fetch_bytes(5, arguments);
                                WHEN others =>
                                    index <= 0;
                                    state <= write_header;
                            end CASE;

                        WHEN arguments =>
                            CASE current_command is
                                WHEN read_command =>
                                    address    <= to_integer(unsigned(arguments_register(15 downto 0)));
                                    is_read    <= true;
                                    reads_left <= 1;
                                WHEN write_command =>
                                    address     <= to_integer(unsigned(arguments_register(47 downto 32)));
                                    write_value <= arguments_register(31 downto 0);
                                    is_read     <= false;
                                WHEN others =>  -- stream
                                    address    <= to_integer(unsigned(arguments_register(39 downto 24)));
                                    is_read    <= true;
                                    reads_left <= to_integer(unsigned(arguments_register(23 downto 0)));
                            end CASE;
                            state <= bus_request;
                    end CASE;

                WHEN bus_request =>
                    if is_read and (reads_left = 0 or results = g_max_results) then
                        if results = g_max_results then
                            index <= 0;
                            state <= write_header;
                        else
                            fetch_bytes(1, command);
                        end if;
                    elsif request_is_granted then
                        if is_read then
                            timer <= 0;
                            state <= bus_wait;
                        else
                            fetch_bytes(1, command);
                        end if;
                    elsif is_read then
                        request_data_from_address(bus_out, address);
                    else
                        write_data_to_address(bus_out, address, write_value);
                    end if;

                WHEN bus_wait =>
                    if write_to_address_is_requested(bus_in, 0) then
                        store_word <= get_slv_data(bus_in);
                    elsif timer < g_timeout then
                        timer <= timer + 1;
                    else
                        store_word <= x"FFFFFFFF";
                        timeouts   <= timeouts + 1;
                    end if;
                    if write_to_address_is_requested(bus_in, 0) or timer = g_timeout then
                        store_left   <= 4;
                        store_return <= arguments;
                        results      <= results + 1;
                        reads_left   <= reads_left - 1;
                        state        <= store;
                    end if;

                -- append store_left bytes from the top of store_word to the reply payload
                WHEN store =>
                    if store_left > 0 then
                        write_tx_byte(port_out, tx_offset + payload_offset + reply_bytes, store_word(31 downto 24));
                        store_word  <= store_word(23 downto 0) & x"00";
                        reply_bytes <= reply_bytes + 1;
                        store_left  <= store_left - 1;
                    elsif store_return = command then
                        fetch_bytes(1, command);
                    else
                        state <= bus_request;   -- next read of a stream
                    end if;

                WHEN write_header =>
                    if index = 0 then
                        new_ip_length := std_logic_vector(to_unsigned(ip_udp_header_length + reply_bytes, 16));
                        lengths <= (
                            ip_total_length => new_ip_length,
                            ip_checksum     => updated_checksum(get_bytes(header, ip_checksum, 2),
                                                                get_bytes(header, ip_total_length, 2), new_ip_length),
                            udp_length      => std_logic_vector(to_unsigned(8 + reply_bytes, 16)));
                        index <= 1;
                    elsif index <= header_length then
                        write_tx_byte(port_out, tx_offset + index - 1
                            , reply_byte(udp_bridge_reply, header, index - 1, g_own_mac, g_own_ip, x"0000", lengths));
                        index <= index + 1;
                    else
                        state <= send;
                    end if;

                WHEN send =>
                    port_out.tx_send   <= true;
                    port_out.tx_length <= header_length + reply_bytes;
                    port_out.release_frame   <= true;
                    port_out.replied   <= true;
                    state <= idle;
            end CASE;
        end if;
    end process;

end architecture rtl;
