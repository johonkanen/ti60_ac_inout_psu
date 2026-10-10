library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Transmit checksum offload for the TSEMAC, which has none: an 8 bit AXI stream stage
-- between the transmit DMA and the MAC that fills in the IPv4 header checksum and the
-- TCP, UDP or ICMP checksum of each frame, whatever those fields hold (Zephyr leaves them
-- zero when the driver declares the offload).
--
-- Store and forward: a frame is written into a 4 KB ring while its sums are taken, byte by
-- byte, and played out once its last byte is in, with the checksum bytes replaced. Bytes
-- with tkeep low are dropped, the output has every byte kept. One byte per clock both ways,
-- a few clocks between frames.
--
-- A frame is patched when it is Ethernet II IPv4 (EtherType 0x0800, no VLAN tag), version 4
-- with a header of 20 bytes or more, and holds the whole header; its L4 checksum too when
-- the protocol is TCP (6), UDP (17) or ICMP (1), the datagram is not a fragment and the
-- frame holds all of it (bytes after the IP total length, the Ethernet padding, are not
-- summed). Anything else passes unchanged. A UDP checksum of zero is sent as 0xffff.
entity tx_checksum_inserter is
    port(
        clock    : in std_logic;
        reset    : in std_logic;
        s_tvalid : in std_logic;
        s_tready : out std_logic;
        s_tdata  : in std_logic_vector(7 downto 0);
        s_tkeep  : in std_logic;
        s_tdest  : in std_logic_vector(3 downto 0);
        s_tlast  : in std_logic;
        m_tvalid : out std_logic;
        m_tready : in std_logic;
        m_tdata  : out std_logic_vector(7 downto 0);
        m_tdest  : out std_logic_vector(3 downto 0);
        m_tlast  : out std_logic
    );
end entity tx_checksum_inserter;

architecture rtl of tx_checksum_inserter is

    constant ram_bits : natural := 12;      -- 4096 bytes, more than two full frames
    type byte_ram is array (0 to 2**ram_bits-1) of std_logic_vector(7 downto 0);
    signal ram : byte_ram;

    subtype pointer is unsigned(ram_bits downto 0);   -- one bit more than the address
    subtype frame_length is unsigned(11 downto 0);

    -- a frame that has been written, for the reader
    type frame_record is record
        start   : pointer;
        length  : frame_length;
        dest    : std_logic_vector(3 downto 0);
        ip_ok   : boolean;
        ip_sum  : unsigned(31 downto 0);
        l4_ok   : boolean;
        l4_udp  : boolean;
        l4_at   : frame_length;             -- offset of the L4 checksum in the frame
        l4_sum  : unsigned(31 downto 0);    -- with the pseudo header
    end record;
    constant no_frame : frame_record := ((others => '0'), (others => '0'), (others => '0'),
        false, (others => '0'), false, false, (others => '0'), (others => '0'));
    type frame_array is array (0 to 3) of frame_record;
    signal frames : frame_array := (others => no_frame);
    signal frames_written, frames_read : unsigned(2 downto 0) := (others => '0');

    -- writer
    signal write_pointer, read_pointer : pointer := (others => '0');
    signal ready         : std_logic;
    signal position      : frame_length := (others => '0');
    signal frame_start   : pointer := (others => '0');
    signal ethertype_hi  : std_logic_vector(7 downto 0) := (others => '0');
    signal is_ipv4       : boolean := false;
    signal header_length : unsigned(5 downto 0) := (others => '0');  -- bytes, IHL*4
    signal total_length  : unsigned(15 downto 0) := (others => '0');
    signal fragment      : boolean := false;
    signal protocol      : unsigned(7 downto 0) := (others => '0');
    signal ip_sum, l4_sum, address_sum : unsigned(31 downto 0) := (others => '0');
    signal finished      : frame_record := no_frame;
    signal push_frame    : boolean := false;

    -- reader
    type reader_state is (idle, fold_1, fold_2, fold_3, stream);
    signal state       : reader_state := idle;
    signal current     : frame_record := no_frame;
    signal ip_check, l4_check : std_logic_vector(15 downto 0) := (others => '0');
    signal offset      : frame_length := (others => '0');
    signal ram_q       : std_logic_vector(7 downto 0) := (others => '0');
    signal pending     : boolean := false;
    signal pending_replace : boolean := false;
    signal pending_byte    : std_logic_vector(7 downto 0) := (others => '0');
    signal pending_last    : std_logic := '0';
    -- two entry output queue
    type out_entry is record
        data : std_logic_vector(7 downto 0);
        last : std_logic;
        dest : std_logic_vector(3 downto 0);
    end record;
    type out_array is array (0 to 1) of out_entry;
    signal queue       : out_array := (others => ((others => '0'), '0', (others => '0')));
    signal queued      : natural range 0 to 2 := 0;

    function fold(sum : unsigned(31 downto 0)) return unsigned is
    begin
        return resize(sum(15 downto 0), 32) + resize(sum(31 downto 16), 32);
    end function;

begin

    ready    <= '1' when write_pointer - read_pointer < 2**ram_bits
                         and frames_written - frames_read < 3 and not push_frame else '0';
    s_tready <= ready;

    writer : process(clock)
        variable byte    : unsigned(7 downto 0);
        variable word    : unsigned(31 downto 0);
        variable v_ip, v_l4, v_address : unsigned(31 downto 0);
        variable l4_start : unsigned(11 downto 0);
        variable l4_end   : unsigned(16 downto 0);
        variable l4_at    : unsigned(11 downto 0);
        variable kind_ok  : boolean;
        variable is_udp   : boolean;
        variable p        : frame_length;
    begin
        if rising_edge(clock) then
            push_frame <= false;
            if push_frame then
                frames(to_integer(frames_written(1 downto 0))) <= finished;
                frames_written <= frames_written + 1;
            end if;

            if s_tvalid = '1' and ready = '1' then
                p := position;
                if s_tkeep = '1' then
                    ram(to_integer(write_pointer(ram_bits-1 downto 0))) <= s_tdata;
                    write_pointer <= write_pointer + 1;
                    if p = 0 then
                        frame_start <= write_pointer;
                    end if;
                end if;

                byte := unsigned(s_tdata);
                if p(0) = '0' then      -- the frame's 16 bit words start at even offsets
                    word := resize(byte & x"00", 32);
                else
                    word := resize(byte, 32);
                end if;
                if s_tkeep = '0' then
                    word := (others => '0');
                end if;

                -- where the L4 checksum is, once the IP header has been seen
                is_udp   := protocol = 17;
                kind_ok  := protocol = 6 or protocol = 17 or protocol = 1;
                l4_start := 14 + resize(header_length, 12);
                l4_end   := 14 + resize(total_length, 17);
                if protocol = 6 then
                    l4_at := l4_start + 16;
                elsif is_udp then
                    l4_at := l4_start + 6;
                else
                    l4_at := l4_start + 2;
                end if;

                v_ip := ip_sum;
                v_l4 := l4_sum;
                v_address := address_sum;
                if s_tkeep = '1' then
                    case to_integer(p) is
                        when 12 => ethertype_hi <= s_tdata;
                        when 13 => is_ipv4 <= ethertype_hi = x"08" and s_tdata = x"00";
                        when 14 => header_length <= unsigned(s_tdata(3 downto 0)) & "00";
                                   if s_tdata(7 downto 4) /= x"4" then
                                       header_length <= (others => '0');
                                   end if;
                        when 16 => total_length(15 downto 8) <= byte;
                        when 17 => total_length(7 downto 0) <= byte;
                        when 20 => fragment <= s_tdata(5) = '1' or s_tdata(4 downto 0) /= "00000";
                        when 21 => if s_tdata /= x"00" then fragment <= true; end if;
                        when 23 => protocol <= byte;
                        when others => null;
                    end case;
                    -- IP header, less its checksum field
                    if p = 14 or (p > 14 and p < l4_start and p /= 24 and p /= 25) then
                        v_ip := v_ip + word;
                    end if;
                    -- the addresses, for the pseudo header
                    if p >= 26 and p < 34 then
                        v_address := v_address + word;
                    end if;
                    -- the L4 data less its checksum
                    if p >= l4_start and resize(p, 17) < l4_end and p /= l4_at
                       and p /= l4_at + 1 then
                        v_l4 := v_l4 + word;
                    end if;
                    position <= p + 1;
                end if;

                if s_tlast = '1' then
                    if s_tkeep = '1' then
                        p := p + 1;     -- the frame's length
                    end if;
                    finished.start  <= frame_start;
                    if position = 0 then
                        finished.start <= write_pointer;
                    end if;
                    finished.length <= p;
                    finished.dest   <= s_tdest;
                    finished.ip_ok  <= is_ipv4 and header_length >= 20
                                       and p >= l4_start and p > 25;
                    finished.ip_sum <= v_ip;
                    finished.l4_ok  <= is_ipv4 and header_length >= 20 and not fragment
                                       and kind_ok and resize(p, 17) >= l4_end
                                       and l4_end >= resize(l4_at, 17) + 2;
                    finished.l4_udp <= is_udp;
                    finished.l4_at  <= l4_at;
                    -- pseudo header: protocol and L4 length, ICMP has none
                    if protocol = 1 then
                        finished.l4_sum <= v_l4;
                    else
                        finished.l4_sum <= v_l4 + v_address + resize(protocol, 32)
                                           + resize(total_length, 32) - resize(header_length, 32);
                    end if;
                    push_frame <= p /= 0;
                    position   <= (others => '0');
                    ip_sum     <= (others => '0');
                    l4_sum     <= (others => '0');
                    address_sum <= (others => '0');
                    total_length <= (others => '0');
                    is_ipv4    <= false;
                    fragment   <= false;
                    protocol   <= (others => '0');
                    header_length <= (others => '0');
                else
                    ip_sum <= v_ip;
                    l4_sum <= v_l4;
                    address_sum <= v_address;
                end if;
            end if;

            if reset = '1' then
                write_pointer  <= (others => '0');
                frames_written <= (others => '0');
                position       <= (others => '0');
                ip_sum         <= (others => '0');
                l4_sum         <= (others => '0');
                address_sum    <= (others => '0');
                total_length   <= (others => '0');
                is_ipv4        <= false;
                push_frame     <= false;
            end if;
        end if;
    end process;

    reader : process(clock)
        variable pop : boolean;
        variable count : natural range 0 to 3;
        variable address : pointer;
        variable replace : boolean;
        variable value   : std_logic_vector(7 downto 0);
    begin
        if rising_edge(clock) then
            pop := queued > 0 and m_tready = '1';

            -- the read issued last clock lands in the queue
            count := queued;
            if pop then
                queue(0) <= queue(1);
                count := count - 1;
            end if;
            if pending then
                if pending_replace then
                    queue(count) <= (pending_byte, pending_last, current.dest);
                else
                    queue(count) <= (ram_q, pending_last, current.dest);
                end if;
                count := count + 1;
            end if;
            queued  <= count;
            pending <= false;

            case state is
                when idle =>
                    if frames_written /= frames_read then
                        current     <= frames(to_integer(frames_read(1 downto 0)));
                        frames_read <= frames_read + 1;
                        state       <= fold_1;
                    end if;
                when fold_1 =>
                    current.ip_sum <= fold(current.ip_sum);
                    current.l4_sum <= fold(current.l4_sum);
                    state <= fold_2;
                when fold_2 =>
                    current.ip_sum <= fold(current.ip_sum);
                    current.l4_sum <= fold(current.l4_sum);
                    state <= fold_3;
                when fold_3 =>
                    ip_check <= not std_logic_vector(current.ip_sum(15 downto 0));
                    l4_check <= not std_logic_vector(current.l4_sum(15 downto 0));
                    if current.l4_udp and current.l4_sum(15 downto 0) = x"ffff" then
                        l4_check <= x"ffff";
                    end if;
                    offset <= (others => '0');
                    state  <= stream;
                when stream =>
                    -- a read lands in the queue next clock: room for it
                    if count <= 1 then
                        address := current.start + offset;
                        ram_q   <= ram(to_integer(address(ram_bits-1 downto 0)));
                        replace := false;
                        value   := (others => '0');
                        if current.ip_ok and offset = 24 then
                            replace := true; value := ip_check(15 downto 8);
                        elsif current.ip_ok and offset = 25 then
                            replace := true; value := ip_check(7 downto 0);
                        elsif current.l4_ok and offset = current.l4_at then
                            replace := true; value := l4_check(15 downto 8);
                        elsif current.l4_ok and offset = current.l4_at + 1 then
                            replace := true; value := l4_check(7 downto 0);
                        end if;
                        pending         <= true;
                        pending_replace <= replace;
                        pending_byte    <= value;
                        pending_last    <= '1' when offset = current.length - 1 else '0';
                        read_pointer    <= read_pointer + 1;
                        offset          <= offset + 1;
                        if offset = current.length - 1 then
                            state <= idle;
                        end if;
                    end if;
            end case;

            if reset = '1' then
                state       <= idle;
                frames_read <= (others => '0');
                read_pointer <= (others => '0');
                queued      <= 0;
                pending     <= false;
            end if;
        end if;
    end process;

    m_tvalid <= '1' when queued > 0 else '0';
    m_tdata  <= queue(0).data;
    m_tlast  <= queue(0).last;
    m_tdest  <= queue(0).dest;

end architecture rtl;
