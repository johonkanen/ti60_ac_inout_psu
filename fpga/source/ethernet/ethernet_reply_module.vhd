library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;

-- Protocol module answering one kind of request through the frame port:
--   arp_reply        ARP request for own_ip, 42 byte reply (padded by the transmitter)
--   icmp_echo_reply  ping, the request with addresses swapped and the type changed
--   udp_echo_reply   UDP datagram to g_udp_port, sent back with addresses and ports swapped
-- The 42 byte header is read into registers and checked, the reply header is written
-- into the transmit slot and the rest of the request is copied after it unchanged.
entity ethernet_reply_module is
    generic(
        g_kind     : reply_kind;
        g_own_mac  : mac_address;
        g_own_ip   : ip_address;
        g_udp_port : natural := 5000
    );
    port(
        clock    : in std_logic;
        port_in  : in frame_port_in_record;
        port_out : out frame_port_out_record
    );
end entity ethernet_reply_module;

architecture rtl of ethernet_reply_module is

    type states is (idle, capture, check_ip, decide, wait_for_tx_slot, write_header, copy, send);
    signal state : states := idle;

    signal offset    : rx_frame_address := 0;
    signal length    : frame_length_type := 0;
    signal tx_offset : tx_frame_address := 0;
    signal header    : header_array := (others => (others => '0'));
    signal index     : natural range 0 to slot_size := 0;
    signal checksum  : std_logic_vector(15 downto 0) := (others => '0');
    signal word_index : natural range 0 to 10 := 0;
    signal icmp_checksum_value : std_logic_vector(15 downto 0) := (others => '0');

    -- read pipeline: stage n holds the frame index of the read issued n+1 clocks ago
    type index_array is array (0 to frame_read_latency-1) of natural range 0 to slot_size-1;
    signal pending       : std_logic_vector(frame_read_latency-1 downto 0) := (others => '0');
    signal pending_index : index_array := (others => 0);

    function wants(h : header_array; ip_header_sum : std_logic_vector(15 downto 0)) return boolean is
    begin
        CASE g_kind is
            WHEN arp_reply       => return is_arp_request_for(h, g_own_ip);
            WHEN icmp_echo_reply => return is_ipv4_for(h, g_own_ip, ip_header_sum) and is_icmp_echo_request(h);
            WHEN udp_echo_reply  => return is_ipv4_for(h, g_own_ip, ip_header_sum) and is_udp_to_port(h, g_udp_port);
            WHEN others          => return false;
        end CASE;
    end function;

    function pipeline_is_empty(p : std_logic_vector) return boolean is
    begin
        return p = (p'range => '0');
    end function;

begin

    process(clock)
        procedure read_frame_byte(i : natural) is
        begin
            port_out.read_address <= offset + i;
            pending(0)       <= '1';
            pending_index(0) <= i;
        end procedure;
    begin
        if rising_edge(clock) then
            init_frame_port(port_out);
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
                    icmp_checksum_value <= icmp_reply_checksum(get_bytes(header, icmp_checksum, 2));
                    if wants(header, checksum) and length >= header_length then
                        state <= wait_for_tx_slot;
                    else
                        port_out.release_frame <= true;
                        state <= idle;
                    end if;

                WHEN wait_for_tx_slot =>
                    if port_in.tx_slot_is_free then
                        tx_offset <= port_in.tx_offset;
                        index     <= 0;
                        state     <= write_header;
                    end if;

                WHEN write_header =>
                    write_tx_byte(port_out, tx_offset + index
                        , reply_byte(g_kind, header, index, g_own_mac, g_own_ip, icmp_checksum_value));
                    if index < header_length-1 then
                        index <= index + 1;
                    else
                        index <= header_length;
                        if g_kind = arp_reply then
                            state <= send;
                        else
                            state <= copy;
                        end if;
                    end if;

                -- the rest of the request goes to the reply unchanged
                WHEN copy =>
                    if pending(frame_read_latency-1) = '1' then
                        write_tx_byte(port_out, tx_offset + pending_index(frame_read_latency-1), port_in.read_data);
                    end if;
                    if index < length then
                        read_frame_byte(index);
                        index <= index + 1;
                    elsif pipeline_is_empty(pending) then
                        state <= send;
                    end if;

                WHEN send =>
                    port_out.tx_send <= true;
                    if g_kind = arp_reply then
                        port_out.tx_length <= header_length;
                    else
                        port_out.tx_length <= length;
                    end if;
                    port_out.release_frame <= true;
                    port_out.replied <= true;
                    state <= idle;
            end CASE;
        end if;
    end process;

end architecture rtl;
