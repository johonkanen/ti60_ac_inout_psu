library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.udp_stream_pkg.all;

-- Streams words from a data source to the requester as UDP datagrams, see udp_stream_pkg.
-- Datagrams are built while they are sent: the headers come from the request and the data
-- words are read from the source g_read_latency clocks ahead. A new request replaces the
-- running one after the datagram being sent. Requests with bit 31 of the first word set read
-- from the prefetching source: every datagram is announced through prefetch one datagram ahead
-- and is sent when data_is_ready.
entity udp_stream_sender is
    generic(
        g_own_mac      : mac_address;
        g_own_ip       : ip_address;
        g_read_latency : natural := 3     -- data valid this many clocks after read_address is set
    );
    port(
        clock         : in std_logic;
        request       : in stream_request_record;
        read_address  : out natural;
        read_data     : in std_logic_vector(31 downto 0);
        prefetch      : out stream_prefetch_record;
        data_is_ready : in boolean := true;

        may_start     : in boolean;
        wants_to_send : out boolean;
        is_sending    : out boolean;
        gmii_out      : out gmii_record;

        datagrams     : out unsigned(31 downto 0)
    );
end entity udp_stream_sender;

architecture rtl of udp_stream_sender is

    constant minimum_frame : natural := 60;

    type states is (idle, prepare, wait_for_line, preamble, frame, fcs, gap);
    signal state : states := idle;

    signal active      : stream_request_record := no_stream_request;
    signal next_request : stream_request_record := no_stream_request;
    signal first_word  : unsigned(31 downto 0) := (others => '0');
    signal words_left  : unsigned(31 downto 0) := (others => '0');
    signal words       : natural range 0 to stream_words_per_datagram := 0;
    signal ip_id       : unsigned(15 downto 0) := (others => '0');
    signal ip_length   : std_logic_vector(15 downto 0) := (others => '0');
    signal checksum    : std_logic_vector(15 downto 0) := (others => '0');
    signal sum_step    : natural range 0 to 10 := 0;

    signal index       : natural range 0 to 2047 := 0;
    signal frame_bytes : natural range 0 to 2047 := 0;
    signal gap_clocks  : natural := 0;
    signal crc         : std_logic_vector(31 downto 0) := crc32_init;
    signal data_word   : std_logic_vector(31 downto 0) := (others => '0');
    signal count       : unsigned(31 downto 0) := (others => '0');
    signal prefetching : boolean := false;
    signal prefetch_r  : stream_prefetch_record := no_stream_prefetch;

    function datagram_words(words_left : unsigned(31 downto 0)) return natural is
    begin
        if words_left > stream_words_per_datagram then
            return stream_words_per_datagram;
        end if;
        return to_integer(words_left);
    end function;

    function byte_of(v : std_logic_vector; n : natural) return byte is
        constant bytes : natural := v'length/8;
        variable x : std_logic_vector(v'length-1 downto 0) := v;
    begin
        return x(8*(bytes-n)-1 downto 8*(bytes-n-1));
    end function;

    -- the ten 16 bit words of the ip header, with the checksum field 0
    function ip_header_word(n : natural; ip_length, id : std_logic_vector(15 downto 0);
                            source, destination : ip_address) return std_logic_vector is
    begin
        CASE n is
            WHEN 0 => return x"4500";
            WHEN 1 => return ip_length;
            WHEN 2 => return id;
            WHEN 3 => return x"4000";             -- don't fragment
            WHEN 4 => return x"4011";             -- ttl 64, udp
            WHEN 5 => return x"0000";
            WHEN 6 => return source(31 downto 16);
            WHEN 7 => return source(15 downto 0);
            WHEN 8 => return destination(31 downto 16);
            WHEN others => return destination(15 downto 0);
        end CASE;
    end function;

begin

    datagrams     <= count;
    prefetch      <= prefetch_r;
    -- a datagram from the prefetching source waits for its data before taking the line
    wants_to_send <= state = wait_for_line and (data_is_ready or not prefetching);
    is_sending    <= state = preamble or state = frame or state = fcs or state = gap;

    process(clock)
        procedure output(value : byte) is
        begin
            gmii_out <= (data => value, is_valid => true, is_error => false);
        end procedure;

        variable b         : byte;
        variable data_byte : natural;
        variable udp_length : natural;
        variable last      : boolean;
        variable this_words : natural range 0 to stream_words_per_datagram;
    begin
        if rising_edge(clock) then
            gmii_out <= gmii_idle;
            prefetch_r.reset         <= false;
            prefetch_r.request       <= false;
            prefetch_r.datagram_done <= false;

            if request.is_requested then
                next_request <= request;
            end if;

            CASE state is
                WHEN idle =>
                    if next_request.is_requested then
                        active       <= next_request;
                        first_word   <= next_request.first_word;
                        words_left   <= next_request.word_count;
                        next_request.is_requested <= false;
                        prefetching <= next_request.first_word(31) = '1';
                        prefetch_r.is_active <= next_request.first_word(31) = '1';
                        if next_request.first_word(31) = '1' and next_request.word_count /= 0 then
                            -- announce the first datagram of the stream
                            prefetch_r.reset      <= true;
                            prefetch_r.request    <= true;
                            prefetch_r.first_word <= next_request.first_word(30 downto 0);
                            prefetch_r.words      <= datagram_words(next_request.word_count);
                        end if;
                        if next_request.word_count /= 0 then
                            sum_step <= 0;
                            state    <= prepare;
                        end if;
                    end if;

                -- size of this datagram and its ip header checksum
                WHEN prepare =>
                    if sum_step = 0 then
                        -- announce the datagram after this one
                        this_words := datagram_words(words_left);
                        if prefetching and words_left > this_words then
                            prefetch_r.request    <= true;
                            prefetch_r.first_word <= first_word(30 downto 0) + this_words;
                            prefetch_r.words      <= datagram_words(words_left - this_words);
                        end if;
                        if words_left > stream_words_per_datagram then
                            words <= stream_words_per_datagram;
                            udp_length := 8 + stream_header_length + 4*stream_words_per_datagram;
                        else
                            words <= to_integer(words_left);
                            udp_length := 8 + stream_header_length + 4*to_integer(words_left);
                        end if;
                        ip_length   <= std_logic_vector(to_unsigned(20 + udp_length, 16));
                        frame_bytes <= header_length + udp_length - 8;
                        if header_length + udp_length - 8 < minimum_frame then
                            frame_bytes <= minimum_frame;
                        end if;
                        checksum <= (others => '0');
                        sum_step <= 1;
                    elsif sum_step <= 10 then
                        checksum <= ones_complement_add(checksum,
                            ip_header_word(sum_step-1, ip_length, std_logic_vector(ip_id), g_own_ip, active.destination_ip));
                        if sum_step = 10 then
                            state <= wait_for_line;
                        else
                            sum_step <= sum_step + 1;
                        end if;
                    end if;

                WHEN wait_for_line =>
                    if may_start and (data_is_ready or not prefetching) then
                        index <= 0;
                        state <= preamble;
                    end if;

                WHEN preamble =>
                    if index < 7 then
                        output(preamble_byte);
                        index <= index + 1;
                    else
                        output(start_of_frame_byte);
                        crc   <= crc32_init;
                        index <= 0;
                        state <= frame;
                    end if;

                WHEN frame =>
                    last := words_left <= stream_words_per_datagram;
                    CASE index is
                        WHEN 0 to 5   => b := byte_of(active.destination_mac, index);
                        WHEN 6 to 11  => b := byte_of(g_own_mac, index - 6);
                        WHEN 12       => b := x"08";
                        WHEN 13       => b := x"00";
                        WHEN 14 to 23 => b := byte_of(ip_header_word((index-14)/2, ip_length, std_logic_vector(ip_id), g_own_ip, active.destination_ip), (index-14) mod 2);
                        WHEN 24       => b := not checksum(15 downto 8);
                        WHEN 25       => b := not checksum(7 downto 0);
                        WHEN 26 to 33 => b := byte_of(ip_header_word(6 + (index-26)/2, ip_length, std_logic_vector(ip_id), g_own_ip, active.destination_ip), (index-26) mod 2);
                        WHEN 34 to 35 => b := byte_of(active.source_port, index - 34);
                        WHEN 36 to 37 => b := byte_of(active.destination_port, index - 36);
                        WHEN 38 to 39 => b := byte_of(std_logic_vector(unsigned(ip_length) - 20), index - 38);
                        WHEN 40 to 41 => b := x"00";    -- udp checksum not used
                        WHEN 42 to 43 => b := byte_of(active.request_id, index - 42);
                        WHEN 44       => b := x"00";
                        WHEN 45       => if last then b := x"01"; else b := x"00"; end if;
                        WHEN 46 to 49 => b := byte_of(std_logic_vector(first_word), index - 46);
                        WHEN 50 to 51 => b := byte_of(std_logic_vector(to_unsigned(words, 16)), index - 50);
                        WHEN 52 to 53 => b := x"00";
                        WHEN others =>
                            data_byte := index - stream_data_offset;
                            if data_byte < 4*words then
                                b := byte_of(data_word, data_byte mod 4);
                            else
                                b := x"00";   -- padding to the minimum frame
                            end if;
                    end CASE;
                    output(b);
                    crc <= next_crc32(crc, b);

                    -- word j is read four bytes before it is needed and taken one byte before
                    if index + 4 >= stream_data_offset and (index + 4 - stream_data_offset) mod 4 = 0 then
                        read_address <= to_integer(first_word(30 downto 0)) + (index + 4 - stream_data_offset)/4;
                    end if;
                    if index + 1 >= stream_data_offset and (index + 1 - stream_data_offset) mod 4 = 0 then
                        data_word <= read_data;
                    end if;

                    if index < frame_bytes-1 then
                        index <= index + 1;
                    else
                        index <= 0;
                        state <= fcs;
                    end if;

                WHEN fcs =>
                    output(fcs_byte(crc, index));
                    if index < 3 then
                        index <= index + 1;
                    else
                        index      <= 0;
                        gap_clocks <= 12 + 16*to_integer(active.pacing);
                        count      <= count + 1;
                        ip_id      <= ip_id + 1;
                        first_word <= first_word + words;
                        words_left <= words_left - words;
                        prefetch_r.datagram_done <= prefetching;
                        state      <= gap;
                    end if;

                WHEN gap =>
                    if index < gap_clocks-1 then
                        index <= index + 1;
                    elsif next_request.is_requested or words_left = 0 then
                        state <= idle;
                    else
                        sum_step <= 0;
                        state    <= prepare;
                    end if;
            end CASE;
        end if;
    end process;

end architecture rtl;
