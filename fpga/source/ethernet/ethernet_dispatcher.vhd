library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;

-- Hands received frames to protocol modules. For each frame offered by the receive buffer
-- the ethertype, ip protocol and udp destination port are read from the frame RAM and the
-- first matching entry of g_routes selects the module. The module gets the frame with
-- frame_is_ready, offset and length together with the receive RAM read port and the
-- transmit RAM write port; its release frees the receive slot. One frame is handled at a
-- time, frames that match no route are released at once.
entity ethernet_dispatcher is
    generic(g_routes : frame_route_array);
    port(
        clock              : in std_logic;

        frame_is_available : in boolean;
        frame_slot         : in natural range 0 to rx_slot_count-1;
        frame_offset       : in rx_frame_address;
        frame_length       : in frame_length_type;
        take_frame         : out boolean;
        release_slot       : out std_logic_vector(rx_slot_count-1 downto 0);
        rx_read_address    : out rx_frame_address;
        rx_read_data       : in byte;

        tx_slot_is_free    : in boolean;
        tx_slot_offset     : in tx_frame_address;
        tx_write_enable    : out boolean;
        tx_write_address   : out tx_frame_address;
        tx_write_data      : out byte;
        tx_send            : out boolean;
        tx_length          : out frame_length_type;

        ports_in           : out frame_port_in_array(g_routes'range);
        ports_out          : in frame_port_out_array(g_routes'range);

        frames             : out unsigned(31 downto 0);
        not_answered       : out unsigned(31 downto 0)
    );
end entity ethernet_dispatcher;

architecture rtl of ethernet_dispatcher is

    constant no_owner : natural := g_routes'high + 1;

    type states is (idle, classify, select_module, busy);
    signal state : states := idle;

    -- bytes needed for routing: ethertype, ip protocol, udp destination port
    type position_array is array (0 to 4) of natural;
    constant positions : position_array := (eth_type, eth_type + 1, ip_protocol, udp_destination_port, udp_destination_port + 1);
    type byte_array is array (0 to 4) of byte;
    signal fields : byte_array := (others => (others => '0'));

    signal owner        : natural range 0 to no_owner := no_owner;
    signal slot         : natural range 0 to rx_slot_count-1 := 0;
    signal offset       : rx_frame_address := 0;
    signal length       : frame_length_type := 0;
    signal read_count   : natural range 0 to 5 := 0;
    signal own_address  : rx_frame_address := 0;
    type tag_array is array (0 to frame_read_latency-1) of natural range 0 to 4;
    signal pending      : std_logic_vector(frame_read_latency-1 downto 0) := (others => '0');
    signal pending_tag  : tag_array := (others => 0);
    signal give_frame   : boolean := false;

    signal frame_count, not_answered_count : unsigned(31 downto 0) := (others => '0');

    function route_for(f : byte_array) return natural is
        variable ethertype, udp_port : std_logic_vector(15 downto 0);
    begin
        ethertype(15 downto 8) := f(0);
        ethertype(7 downto 0)  := f(1);
        udp_port(15 downto 8) := f(3);
        udp_port(7 downto 0)  := f(4);
        for i in g_routes'range loop
            if ethertype = g_routes(i).ethertype
                and (not g_routes(i).check_protocol or f(2) = g_routes(i).protocol)
                and (not g_routes(i).check_port or to_integer(unsigned(udp_port)) = g_routes(i).udp_port)
            then
                return i;
            end if;
        end loop;
        return no_owner;
    end function;

begin

    frames       <= frame_count;
    not_answered <= not_answered_count;

------------------------------------------------------------------------
    -- the owner of the current frame drives the RAM ports
    rx_read_address  <= ports_out(owner).read_address when owner /= no_owner else own_address;
    tx_write_enable  <= ports_out(owner).tx_write_enable when owner /= no_owner else false;
    tx_write_address <= ports_out(owner).tx_write_address when owner /= no_owner else 0;
    tx_write_data    <= ports_out(owner).tx_write_data when owner /= no_owner else (others => '0');
    tx_send          <= ports_out(owner).tx_send when owner /= no_owner else false;
    tx_length        <= ports_out(owner).tx_length when owner /= no_owner else 0;

    module_ports : for i in g_routes'range generate
        ports_in(i) <= (
            frame_is_ready  => give_frame and owner = i,
            offset          => offset,
            length          => length,
            read_data       => rx_read_data,
            tx_slot_is_free => tx_slot_is_free and owner = i,
            tx_offset       => tx_slot_offset);
    end generate;

------------------------------------------------------------------------
    process(clock)
    begin
        if rising_edge(clock) then
            take_frame   <= false;
            release_slot <= (others => '0');
            give_frame   <= false;

            pending     <= pending(frame_read_latency-2 downto 0) & '0';
            pending_tag <= 0 & pending_tag(0 to frame_read_latency-2);
            if pending(frame_read_latency-1) = '1' then
                fields(pending_tag(frame_read_latency-1)) <= rx_read_data;
            end if;

            CASE state is
                WHEN idle =>
                    if frame_is_available then
                        slot        <= frame_slot;
                        offset      <= frame_offset;
                        length      <= frame_length;
                        take_frame  <= true;
                        frame_count <= frame_count + 1;
                        read_count  <= 0;
                        state       <= classify;
                    end if;

                WHEN classify =>
                    if read_count < 5 then
                        own_address    <= offset + positions(read_count);
                        pending(0)     <= '1';
                        pending_tag(0) <= read_count;
                        read_count     <= read_count + 1;
                    elsif pending = (pending'range => '0') then
                        state <= select_module;
                    end if;

                WHEN select_module =>
                    if route_for(fields) = no_owner then
                        release_slot(slot) <= '1';
                        not_answered_count <= not_answered_count + 1;
                        state <= idle;
                    else
                        owner      <= route_for(fields);
                        give_frame <= true;
                        state      <= busy;
                    end if;

                WHEN busy =>
                    if ports_out(owner).release_frame then
                        release_slot(slot) <= '1';
                        if not ports_out(owner).replied then
                            not_answered_count <= not_answered_count + 1;
                        end if;
                        owner <= no_owner;
                        state <= idle;
                    end if;
            end CASE;
        end if;
    end process;

end architecture rtl;
