library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;

-- Transmits numbered test frames:
--   preamble, SFD, dst ff:ff:ff:ff:ff:ff, src 02:00:00:00:00:01, ethertype 0x88B5,
--   payload: 32 bit sequence number (big endian) then bytes (seq + index) mod 256, FCS,
--   followed by a 12 byte interframe gap.
-- frame_count = 0 transmits until stopped.
entity ethernet_frame_generator is
    port(
        clock          : in std_logic;
        start          : in boolean;
        stop           : in boolean;
        frame_count    : in natural;
        payload_length : in natural range 46 to 1500;
        gmii_out       : out gmii_record;
        is_busy        : out boolean;
        frames_sent    : out unsigned(31 downto 0)
    );
end entity ethernet_frame_generator;

architecture rtl of ethernet_frame_generator is

    type states is (idle, preamble, header, payload, fcs, gap);
    signal state : states := idle;

    signal index      : natural range 0 to 2047 := 0;
    signal crc        : std_logic_vector(31 downto 0) := crc32_init;
    signal sequence_number   : unsigned(31 downto 0) := (others => '0');
    signal frames     : unsigned(31 downto 0) := (others => '0');
    signal remaining  : natural := 0;
    signal continuous : boolean := false;
    signal length     : natural range 46 to 1500 := 46;

    constant header_bytes : natural := 14;

    function header_byte(i : natural) return byte is
    begin
        CASE i is
            WHEN 0 to 5 => return x"FF";
            WHEN 6      => return x"02";
            WHEN 11     => return x"01";
            WHEN 12     => return test_frame_ethertype(15 downto 8);
            WHEN 13     => return test_frame_ethertype(7 downto 0);
            WHEN others => return x"00";
        end CASE;
    end function;

    function payload_byte(seq : unsigned(31 downto 0); i : natural) return byte is
    begin
        CASE i is
            WHEN 0      => return std_logic_vector(seq(31 downto 24));
            WHEN 1      => return std_logic_vector(seq(23 downto 16));
            WHEN 2      => return std_logic_vector(seq(15 downto 8));
            WHEN 3      => return std_logic_vector(seq(7 downto 0));
            WHEN others => return std_logic_vector(seq(7 downto 0) + to_unsigned(i mod 256, 8));
        end CASE;
    end function;

begin

    is_busy     <= state /= idle;
    frames_sent <= frames;

    process(clock)
        procedure send(data : byte) is
        begin
            gmii_out <= (data => data, is_valid => true, is_error => false);
        end procedure;

        procedure send_with_crc(data : byte) is
        begin
            send(data);
            crc <= next_crc32(crc, data);
        end procedure;
    begin
        if rising_edge(clock) then
            gmii_out <= gmii_idle;

            CASE state is
                WHEN idle =>
                    if start then
                        remaining  <= frame_count;
                        continuous <= frame_count = 0;
                        length     <= payload_length;
                        index      <= 0;
                        state      <= preamble;
                    end if;

                WHEN preamble =>
                    if index < 7 then
                        send(preamble_byte);
                        index <= index + 1;
                    else
                        send(start_of_frame_byte);
                        crc   <= crc32_init;
                        index <= 0;
                        state <= header;
                    end if;

                WHEN header =>
                    send_with_crc(header_byte(index));
                    if index < header_bytes-1 then
                        index <= index + 1;
                    else
                        index <= 0;
                        state <= payload;
                    end if;

                WHEN payload =>
                    send_with_crc(payload_byte(sequence_number, index));
                    if index < length-1 then
                        index <= index + 1;
                    else
                        index <= 0;
                        state <= fcs;
                    end if;

                WHEN fcs =>
                    send(fcs_byte(crc, index));
                    if index < 3 then
                        index <= index + 1;
                    else
                        index    <= 0;
                        sequence_number <= sequence_number + 1;
                        frames   <= frames + 1;
                        if not continuous and remaining > 0 then
                            remaining <= remaining - 1;
                        end if;
                        state <= gap;
                    end if;

                WHEN gap =>
                    if index < 11 then
                        index <= index + 1;
                    else
                        index <= 0;
                        if continuous or remaining > 0 then
                            state <= preamble;
                        else
                            state <= idle;
                        end if;
                    end if;
            end CASE;

            if stop then
                state <= idle;
            end if;
        end if;
    end process;

end architecture rtl;
