library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_frame_port_pkg.all;

-- Transmit frame RAM with tx_slot_count slots and a send queue. A free slot is offered
-- with slot_is_free and slot_offset; the module writes its frame into it from the
-- destination MAC address on and queues it with send and length. Queued slots are sent in
-- order with preamble, SFD, FCS and the interframe gap, then freed. Frames shorter than
-- 60 bytes are padded with zeros.
entity ethernet_transmitter is
    port(
        clock         : in std_logic;
        slot_is_free  : out boolean;
        slot_offset   : out tx_frame_address;
        write_enable  : in boolean;
        write_address : in tx_frame_address;
        write_data    : in byte;
        send          : in boolean;
        length        : in frame_length_type;
        gmii_out      : out gmii_record;
        frames_sent   : out unsigned(31 downto 0);
        -- sharing the line with other frame sources, see gmii_frame_arbiter
        may_start     : in boolean := true;
        wants_to_send : out boolean;
        is_sending    : out boolean    -- from the preamble to the end of the interframe gap
    );
end entity ethernet_transmitter;

architecture rtl of ethernet_transmitter is

    constant minimum_length : natural := 60;

    type length_array is array (0 to tx_slot_count-1) of frame_length_type;
    signal is_queued    : std_logic_vector(tx_slot_count-1 downto 0) := (others => '0');
    signal lengths      : length_array := (others => 0);
    signal fill_slot    : natural range 0 to tx_slot_count-1 := 0;
    signal send_slot    : natural range 0 to tx_slot_count-1 := 0;

    type states is (idle, preamble, data, fcs, gap);
    signal state        : states := idle;
    signal index        : natural range 0 to slot_size-1 := 0;
    signal frame_length : frame_length_type := 0;
    signal crc          : std_logic_vector(31 downto 0) := crc32_init;
    signal read_address : tx_frame_address := 0;
    signal read_data    : byte;
    signal count        : unsigned(31 downto 0) := (others => '0');

    function next_slot(slot : natural) return natural is
    begin
        return (slot + 1) mod tx_slot_count;
    end function;

begin

    slot_is_free <= is_queued(fill_slot) = '0';
    slot_offset  <= fill_slot*slot_size;
    frames_sent  <= count;
    wants_to_send <= is_queued(send_slot) = '1';
    is_sending    <= state /= idle;

    process(clock)
        procedure output(value : byte) is
        begin
            gmii_out <= (data => value, is_valid => true, is_error => false);
        end procedure;

        procedure output_with_crc(value : byte) is
        begin
            output(value);
            crc <= next_crc32(crc, value);
        end procedure;
    begin
        if rising_edge(clock) then
            gmii_out <= gmii_idle;

            if send and is_queued(fill_slot) = '0' then
                is_queued(fill_slot) <= '1';
                lengths(fill_slot)   <= length;
                fill_slot            <= next_slot(fill_slot);
            end if;

            CASE state is
                WHEN idle =>
                    if is_queued(send_slot) = '1' and may_start then
                        if lengths(send_slot) < minimum_length then
                            frame_length <= minimum_length;
                        else
                            frame_length <= lengths(send_slot);
                        end if;
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
                        state <= data;
                    end if;
                    -- the first two bytes are read during the last two preamble clocks
                    if index = 7 then
                        read_address <= send_slot*slot_size + 1;
                    else
                        read_address <= send_slot*slot_size;
                    end if;

                WHEN data =>
                    if index < lengths(send_slot) then
                        output_with_crc(read_data);
                    else
                        output_with_crc(x"00");
                    end if;
                    if index + frame_read_latency < slot_size then
                        read_address <= send_slot*slot_size + index + frame_read_latency;
                    end if;
                    if index < frame_length-1 then
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
                        index <= 0;
                        state <= gap;
                    end if;

                WHEN gap =>
                    if index < 11 then
                        index <= index + 1;
                    else
                        is_queued(send_slot) <= '0';
                        send_slot <= next_slot(send_slot);
                        count     <= count + 1;
                        state     <= idle;
                    end if;
            end CASE;
        end if;
    end process;

    u_ram : entity work.dual_clock_byte_ram
    generic map(g_depth => tx_slot_count*slot_size)
    port map(clock, write_enable, write_address, write_data, clock, read_address, read_data);

end architecture rtl;
