library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;

-- Receive frame RAM with rx_slot_count fixed slots, between the RGMII receive clock and
-- the system clock. Frames with a good CRC for own_mac or broadcast are written into the
-- slots in turn. A frame is dropped when the next slot is still in use.
-- In the system clock domain the oldest received frame is offered as {offset, length}
-- while frame_is_available; take_frame moves to the next one and release_slot(i) frees
-- slot i once its module is done with it.
entity ethernet_rx_frame_buffer is
    generic(g_own_mac : mac_address);
    port(
        rx_clock           : in std_logic;
        gmii_in            : in gmii_record;
        frame_is_ended     : in boolean;

        clock              : in std_logic;
        frame_is_available : out boolean;
        frame_slot         : out natural range 0 to rx_slot_count-1;
        frame_offset       : out rx_frame_address;
        frame_length       : out frame_length_type;
        take_frame         : in boolean;
        release_slot       : in std_logic_vector(rx_slot_count-1 downto 0);  -- one clock pulse per slot
        read_address       : in rx_frame_address;
        read_data          : out byte;

        -- receive clock domain counters, cross with cdc_snapshot
        rx_dropped_busy    : out unsigned(15 downto 0);
        rx_crc_errors      : out unsigned(15 downto 0);
        rx_other_addresses : out unsigned(15 downto 0)
    );
end entity ethernet_rx_frame_buffer;

architecture rtl of ethernet_rx_frame_buffer is

    type length_array is array (0 to rx_slot_count-1) of frame_length_type;

    -- receive clock domain
    signal write_enable  : boolean := false;
    signal write_address : rx_frame_address := 0;
    signal write_data    : byte := (others => '0');
    signal index         : natural range 0 to slot_size-1 := 0;
    signal crc           : std_logic_vector(31 downto 0) := crc32_init;
    signal write_slot    : natural range 0 to rx_slot_count-1 := 0;
    signal slot_is_full  : std_logic_vector(rx_slot_count-1 downto 0) := (others => '0');
    signal is_capturing  : boolean := false;
    signal is_ours       : boolean := true;
    signal is_broadcast  : boolean := true;
    signal lengths       : length_array := (others => 0);
    signal ready_toggle  : std_logic_vector(rx_slot_count-1 downto 0) := (others => '0');
    signal release_rx_domain : std_logic_vector(rx_slot_count-1 downto 0);
    signal busy_count, crc_count, other_count : unsigned(15 downto 0) := (others => '0');

    -- system clock domain
    type sync_array is array (0 to rx_slot_count-1) of std_logic_vector(2 downto 0);
    signal ready_sync  : sync_array := (others => (others => '0'));
    signal slot_ready  : std_logic_vector(rx_slot_count-1 downto 0) := (others => '0');
    signal read_slot   : natural range 0 to rx_slot_count-1 := 0;
    attribute async_reg : boolean;
    attribute async_reg of ready_sync : signal is true;

    function mac_byte(mac : mac_address; i : natural) return byte is
    begin
        return mac(47 - 8*i downto 40 - 8*i);
    end function;

    function next_slot(slot : natural) return natural is
    begin
        return (slot + 1) mod rx_slot_count;
    end function;

begin

    rx_dropped_busy    <= busy_count;
    rx_crc_errors      <= crc_count;
    rx_other_addresses <= other_count;

------------------------------------------------------------------------
    receive : process(rx_clock)
    begin
        if rising_edge(rx_clock) then
            write_enable <= false;

            if gmii_in.is_valid then
                if index = 0 then
                    is_capturing <= slot_is_full(write_slot) = '0';
                    is_ours      <= gmii_in.data = mac_byte(g_own_mac, 0);
                    is_broadcast <= gmii_in.data = x"FF";
                elsif index < 6 then
                    if gmii_in.data /= mac_byte(g_own_mac, index) then is_ours <= false; end if;
                    if gmii_in.data /= x"FF" then is_broadcast <= false; end if;
                end if;

                if (index = 0 and slot_is_full(write_slot) = '0') or (index > 0 and is_capturing) then
                    write_enable  <= true;
                    write_address <= write_slot*slot_size + index;
                    write_data    <= gmii_in.data;
                end if;
                crc <= next_crc32(crc, gmii_in.data);
                if index < slot_size-1 then
                    index <= index + 1;
                end if;
            end if;

            if frame_is_ended and index > 0 then
                if crc /= crc32_residue then
                    crc_count <= crc_count + 1;
                elsif not (is_ours or is_broadcast) then
                    other_count <= other_count + 1;
                elsif not is_capturing or index < 64 or index > 1522 then
                    busy_count <= busy_count + 1;
                else
                    slot_is_full(write_slot) <= '1';
                    lengths(write_slot)      <= index - 4;   -- without FCS
                    ready_toggle(write_slot) <= not ready_toggle(write_slot);
                    write_slot               <= next_slot(write_slot);
                end if;
                index        <= 0;
                crc          <= crc32_init;
                is_capturing <= false;
            end if;

            for i in 0 to rx_slot_count-1 loop
                if release_rx_domain(i) = '1' then
                    slot_is_full(i) <= '0';
                end if;
            end loop;
        end if;
    end process;

    release_crossing : for i in 0 to rx_slot_count-1 generate
        signal pulse_in, pulse_out : boolean;
    begin
        pulse_in <= release_slot(i) = '1';
        u_release : entity work.cdc_pulse
        port map(clock, pulse_in, rx_clock, pulse_out);
        release_rx_domain(i) <= '1' when pulse_out else '0';
    end generate;

------------------------------------------------------------------------
    -- frames are offered in the order the slots were filled
    frame_is_available <= slot_ready(read_slot) = '1';
    frame_slot         <= read_slot;
    frame_offset       <= read_slot*slot_size;
    frame_length       <= lengths(read_slot);  -- stable while the slot is full

    offer : process(clock)
    begin
        if rising_edge(clock) then
            for i in 0 to rx_slot_count-1 loop
                ready_sync(i) <= ready_sync(i)(1 downto 0) & ready_toggle(i);
                if ready_sync(i)(2) /= ready_sync(i)(1) then
                    slot_ready(i) <= '1';
                end if;
            end loop;
            if take_frame then
                slot_ready(read_slot) <= '0';
                read_slot <= next_slot(read_slot);
            end if;
        end if;
    end process;

    u_ram : entity work.dual_clock_byte_ram
    generic map(g_depth => rx_slot_count*slot_size)
    port map(rx_clock, write_enable, write_address, write_data, clock, read_address, read_data);

end architecture rtl;
