library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;

-- Streaming of 32 bit words from a data source to the PC over UDP.
--
-- request : a datagram to the stream port with a 12 byte payload
--             request id(15:0), pacing(15:0), first word(31:0), word count(31:0)
--           a word count of 0 stops a running stream
-- data    : datagrams from the stream port back to the requester, each with a 12 byte header
--             request id(15:0), flags(15:0) bit0 last datagram, first word(31:0),
--             word count(15:0), reserved(15:0)
--           followed by up to stream_words_per_datagram words, big endian.
--           pacing adds pacing*16 idle clocks between datagrams.
-- With bit 31 of the first word set the words come from a prefetching source (the
-- HyperRAM): the sender announces each datagram in order through stream_prefetch_record
-- before it needs the data and waits for data_is_ready before sending it.
package udp_stream_pkg is

    constant stream_header_length      : natural := 12;
    constant stream_words_per_datagram : natural := 360;   -- 1440 bytes
    constant stream_data_offset        : natural := header_length + stream_header_length;

    type stream_request_record is record
        is_requested     : boolean;
        destination_mac  : mac_address;
        destination_ip   : ip_address;
        destination_port : std_logic_vector(15 downto 0);
        source_port      : std_logic_vector(15 downto 0);
        request_id       : std_logic_vector(15 downto 0);
        pacing           : unsigned(15 downto 0);
        first_word       : unsigned(31 downto 0);
        word_count       : unsigned(31 downto 0);
    end record;

    constant no_stream_request : stream_request_record := (
        is_requested     => false,
        destination_mac  => (others => '0'),
        destination_ip   => (others => '0'),
        destination_port => (others => '0'),
        source_port      => (others => '0'),
        request_id       => (others => '0'),
        pacing           => (others => '0'),
        first_word       => (others => '0'),
        word_count       => (others => '0'));

    type stream_prefetch_record is record
        is_active     : boolean;                -- the running stream reads the prefetching source
        reset         : boolean;                -- a new stream, forget the announced datagrams
        request       : boolean;                -- the datagram of words words from first_word comes next
        first_word    : unsigned(30 downto 0);
        words         : natural range 0 to stream_words_per_datagram;
        datagram_done : boolean;                -- the oldest announced datagram has been sent
    end record;

    constant no_stream_prefetch : stream_prefetch_record := (
        is_active     => false,
        reset         => false,
        request       => false,
        first_word    => (others => '0'),
        words         => 0,
        datagram_done => false);

end package udp_stream_pkg;
