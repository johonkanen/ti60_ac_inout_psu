library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;
    use work.udp_stream_pkg.all;

-- Protocol module for stream requests: checks a datagram to g_udp_port, takes the
-- requester addresses and the request fields from it and passes them to udp_stream_sender.
-- Nothing is sent from here, the sender streams the data.
entity udp_stream_request_module is
    generic(
        g_own_ip   : ip_address;
        g_udp_port : natural := 5002
    );
    port(
        clock    : in std_logic;
        port_in  : in frame_port_in_record;
        port_out : out frame_port_out_record;
        request  : out stream_request_record
    );
end entity udp_stream_request_module;

architecture rtl of udp_stream_request_module is

    constant read_length : natural := stream_data_offset;  -- headers and the request fields
    type byte_array is array (0 to read_length-1) of byte;

    type states is (idle, capture, check_ip, decide);
    signal state : states := idle;

    signal offset     : rx_frame_address := 0;
    signal length     : frame_length_type := 0;
    signal bytes      : byte_array := (others => (others => '0'));
    signal index      : natural range 0 to slot_size := 0;
    signal checksum   : std_logic_vector(15 downto 0) := (others => '0');
    signal word_index : natural range 0 to 10 := 0;

    type index_array is array (0 to frame_read_latency-1) of natural range 0 to read_length-1;
    signal pending       : std_logic_vector(frame_read_latency-1 downto 0) := (others => '0');
    signal pending_index : index_array := (others => 0);

    function header_of(b : byte_array) return header_array is
        variable h : header_array;
    begin
        for i in h'range loop
            h(i) := b(i);
        end loop;
        return h;
    end function;

    function field(b : byte_array; position, count : natural) return std_logic_vector is
        variable v : std_logic_vector(8*count-1 downto 0);
    begin
        for i in 0 to count-1 loop
            v(8*(count-i)-1 downto 8*(count-i-1)) := b(position + i);
        end loop;
        return v;
    end function;

begin

    process(clock)
        variable h : header_array;
    begin
        if rising_edge(clock) then
            init_frame_port(port_out);
            request.is_requested <= false;
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
                        bytes(pending_index(frame_read_latency-1)) <= port_in.read_data;
                    end if;
                    if index < read_length then
                        port_out.read_address <= offset + index;
                        pending(0)       <= '1';
                        pending_index(0) <= index;
                        index <= index + 1;
                    elsif pending = (pending'range => '0') then
                        checksum   <= (others => '0');
                        word_index <= 0;
                        state      <= check_ip;
                    end if;

                WHEN check_ip =>
                    if word_index < 10 then
                        checksum   <= ones_complement_add(checksum, field(bytes, ip_version_ihl + 2*word_index, 2));
                        word_index <= word_index + 1;
                    else
                        state <= decide;
                    end if;

                WHEN decide =>
                    h := header_of(bytes);
                    if is_ipv4_for(h, g_own_ip, checksum) and is_udp_to_port(h, g_udp_port)
                        and length >= read_length
                    then
                        request <= (
                            is_requested     => true,
                            destination_mac  => field(bytes, eth_source, 6),
                            destination_ip   => field(bytes, ip_source, 4),
                            destination_port => field(bytes, udp_source_port, 2),
                            source_port      => field(bytes, udp_destination_port, 2),
                            request_id       => field(bytes, header_length, 2),
                            pacing           => unsigned(field(bytes, header_length + 2, 2)),
                            first_word       => unsigned(field(bytes, header_length + 4, 4)),
                            word_count       => unsigned(field(bytes, header_length + 8, 4)));
                    end if;
                    port_out.release_frame <= true;
                    state <= idle;
            end CASE;
        end if;
    end process;

end architecture rtl;
