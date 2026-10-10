library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;

-- Checks frames from ethernet_frame_generator. Payload bytes are compared four bytes
-- behind the input so that the FCS at the end of the frame is never taken as payload.
entity ethernet_frame_checker is
    port(
        clock          : in std_logic;
        clear          : in boolean;
        gmii_in        : in gmii_record;
        frame_is_ended : in boolean;
        frames         : out unsigned(31 downto 0);
        good_frames    : out unsigned(31 downto 0);
        crc_errors     : out unsigned(31 downto 0);
        payload_errors : out unsigned(31 downto 0);
        sequence_gaps  : out unsigned(31 downto 0);
        last_length    : out unsigned(15 downto 0)
    );
end entity ethernet_frame_checker;

architecture rtl of ethernet_frame_checker is

    type byte_delay is array (0 to 3) of byte;

    signal index        : natural range 0 to 2047 := 0;
    signal crc          : std_logic_vector(31 downto 0) := crc32_init;
    signal delay_line   : byte_delay := (others => (others => '0'));
    signal sequence_number     : unsigned(31 downto 0) := (others => '0');
    signal last_sequence_number : unsigned(31 downto 0) := (others => '0');
    signal has_sequence_number : boolean := false;
    signal mismatch     : boolean := false;

    signal frame_count   : unsigned(31 downto 0) := (others => '0');
    signal good_count    : unsigned(31 downto 0) := (others => '0');
    signal crc_count     : unsigned(31 downto 0) := (others => '0');
    signal payload_count : unsigned(31 downto 0) := (others => '0');
    signal gap_count     : unsigned(31 downto 0) := (others => '0');
    signal length        : unsigned(15 downto 0) := (others => '0');

    function expected_header(i : natural) return byte is
    begin
        CASE i is
            WHEN 12     => return test_frame_ethertype(15 downto 8);
            WHEN 13     => return test_frame_ethertype(7 downto 0);
            WHEN others => return x"00";
        end CASE;
    end function;

begin

    frames         <= frame_count;
    good_frames    <= good_count;
    crc_errors     <= crc_count;
    payload_errors <= payload_count;
    sequence_gaps  <= gap_count;
    last_length    <= length;

    process(clock)
        variable checked_index : integer;
        variable checked_byte  : byte;
        variable payload_index : integer;
    begin
        if rising_edge(clock) then
            if gmii_in.is_valid then
                crc        <= next_crc32(crc, gmii_in.data);
                delay_line <= gmii_in.data & delay_line(0 to 2);
                index      <= index + 1;

                if gmii_in.is_error then
                    mismatch <= true;
                end if;

                -- byte entering the check four bytes ago
                checked_index := index - 4;
                checked_byte  := delay_line(3);
                if checked_index >= 12 and checked_index <= 13 then
                    if checked_byte /= expected_header(checked_index) then
                        mismatch <= true;
                    end if;
                elsif checked_index >= 14 then
                    payload_index := checked_index - 14;
                    if payload_index <= 3 then
                        sequence_number <= sequence_number(23 downto 0) & unsigned(checked_byte);
                    elsif checked_byte /= std_logic_vector(sequence_number(7 downto 0) + to_unsigned(payload_index mod 256, 8)) then
                        mismatch <= true;
                    end if;
                end if;
            end if;

            if frame_is_ended then
                frame_count <= frame_count + 1;
                length      <= to_unsigned(index, 16);
                if crc /= crc32_residue then
                    crc_count <= crc_count + 1;
                elsif mismatch or index < 64 then
                    payload_count <= payload_count + 1;
                else
                    good_count <= good_count + 1;
                    if has_sequence_number and sequence_number /= last_sequence_number + 1 then
                        gap_count <= gap_count + 1;
                    end if;
                    last_sequence_number <= sequence_number;
                    has_sequence_number  <= true;
                end if;
                index    <= 0;
                crc      <= crc32_init;
                mismatch <= false;
            end if;

            if clear then
                frame_count   <= (others => '0');
                good_count    <= (others => '0');
                crc_count     <= (others => '0');
                payload_count <= (others => '0');
                gap_count     <= (others => '0');
                has_sequence_number  <= false;
            end if;
        end if;
    end process;

end architecture rtl;
