library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.ethernet_frame_port_pkg.all;
    use work.fpga_interconnect_pkg.all;
    use work.udp_stream_pkg.all;

-- Ethernet protocol modules on top of the RGMII receive stream:
--
--   receive frame RAM (3 slots) -> dispatcher -> ARP | ICMP echo | UDP echo | UDP bridge | stream request
--                                                   -> transmit frame RAM (2 slots) -> transmitter
--
-- Each protocol module only gets {offset, length} of its frame and access to the frame RAMs,
-- see ethernet_frame_port_pkg. The UDP bridge is a second master on the fpga_interconnect bus
-- through bus_arbiter.
--
-- register map, offsets from g_base_address, all read only
--   0  frames accepted into the receive buffer
--   1  ARP replies
--   2  ICMP echo replies
--   3  UDP echo replies
--   4  accepted frames that were not answered
--   5  receive: frames dropped because no slot was free (bits 15:0)
--   6  receive: frames with crc error (bits 15:0)
--   7  receive: frames for other MAC addresses (bits 15:0)
--   8  UDP bridge datagrams answered
--   9  UDP bridge reads without a bus response
--  10  frames sent
entity ethernet_stack is
    generic(
        g_base_address    : natural := 500;
        g_own_mac         : mac_address := x"020000000001";
        g_own_ip          : ip_address  := x"C0A8140A";   -- 192.168.20.10
        g_udp_echo_port   : natural := 5000;
        g_udp_bridge_port : natural := 5001;
        g_udp_stream_port : natural := 5002
    );
    port(
        clock             : in std_logic;
        bus_in            : in fpga_interconnect_record;
        bus_out           : out fpga_interconnect_record;
        rx_clock          : in std_logic;
        rx_gmii_in        : in gmii_record;
        rx_frame_is_ended : in boolean;
        tx_gmii_out       : out gmii_record;
        tx_may_start      : in boolean := true;
        tx_wants_to_send  : out boolean;
        tx_is_sending     : out boolean;
        stream_request    : out stream_request_record;

        -- bus master port of the UDP bridge, to bus_arbiter
        bridge_bus_out     : out fpga_interconnect_record;
        bridge_bus_in      : in fpga_interconnect_record;
        request_is_granted : in boolean
    );
end entity ethernet_stack;

architecture rtl of ethernet_stack is

    constant arp_module    : natural := 0;
    constant icmp_module   : natural := 1;
    constant echo_module   : natural := 2;
    constant bridge_module : natural := 3;
    constant stream_module : natural := 4;

    constant routes : frame_route_array(0 to 4) := (
        arp_module    => (ethertype_arp,  false, x"00", false, 0),
        icmp_module   => (ethertype_ipv4, true, ip_protocol_icmp, false, 0),
        echo_module   => (ethertype_ipv4, true, ip_protocol_udp, true, g_udp_echo_port),
        bridge_module => (ethertype_ipv4, true, ip_protocol_udp, true, g_udp_bridge_port),
        stream_module => (ethertype_ipv4, true, ip_protocol_udp, true, g_udp_stream_port));

    signal frame_is_available : boolean;
    signal frame_slot         : natural range 0 to rx_slot_count-1;
    signal frame_offset       : rx_frame_address;
    signal frame_length       : frame_length_type;
    signal take_frame         : boolean;
    signal release_slot       : std_logic_vector(rx_slot_count-1 downto 0);
    signal rx_read_address    : rx_frame_address;
    signal rx_read_data       : byte;

    signal tx_slot_is_free  : boolean;
    signal tx_slot_offset   : tx_frame_address;
    signal tx_write_enable  : boolean;
    signal tx_write_address : tx_frame_address;
    signal tx_write_data    : byte;
    signal tx_send          : boolean;
    signal tx_length        : frame_length_type;

    signal ports_in  : frame_port_in_array(routes'range);
    signal ports_out : frame_port_out_array(routes'range);

    type counter_array is array (routes'range) of unsigned(31 downto 0);
    signal replies : counter_array := (others => (others => '0'));
    signal frames, not_answered, frames_sent, bridge_timeouts : unsigned(31 downto 0);

    signal dropped_busy, crc_errors, other_addresses : unsigned(15 downto 0);
    signal rx_counters_rx_domain, rx_counters : std_logic_vector(47 downto 0) := (others => '0');

begin

    u_rx_buffer : entity work.ethernet_rx_frame_buffer
    generic map(g_own_mac => g_own_mac)
    port map(
        rx_clock            => rx_clock
        ,gmii_in            => rx_gmii_in
        ,frame_is_ended     => rx_frame_is_ended
        ,clock              => clock
        ,frame_is_available => frame_is_available
        ,frame_slot         => frame_slot
        ,frame_offset       => frame_offset
        ,frame_length       => frame_length
        ,take_frame         => take_frame
        ,release_slot       => release_slot
        ,read_address       => rx_read_address
        ,read_data          => rx_read_data
        ,rx_dropped_busy    => dropped_busy
        ,rx_crc_errors      => crc_errors
        ,rx_other_addresses => other_addresses
    );

    u_dispatcher : entity work.ethernet_dispatcher
    generic map(g_routes => routes)
    port map(
        clock               => clock
        ,frame_is_available => frame_is_available
        ,frame_slot         => frame_slot
        ,frame_offset       => frame_offset
        ,frame_length       => frame_length
        ,take_frame         => take_frame
        ,release_slot       => release_slot
        ,rx_read_address    => rx_read_address
        ,rx_read_data       => rx_read_data
        ,tx_slot_is_free    => tx_slot_is_free
        ,tx_slot_offset     => tx_slot_offset
        ,tx_write_enable    => tx_write_enable
        ,tx_write_address   => tx_write_address
        ,tx_write_data      => tx_write_data
        ,tx_send            => tx_send
        ,tx_length          => tx_length
        ,ports_in           => ports_in
        ,ports_out          => ports_out
        ,frames             => frames
        ,not_answered       => not_answered
    );

------------------------------------------------------------------------
    u_arp : entity work.ethernet_reply_module
    generic map(g_kind => arp_reply, g_own_mac => g_own_mac, g_own_ip => g_own_ip)
    port map(clock, ports_in(arp_module), ports_out(arp_module));

    u_icmp : entity work.ethernet_reply_module
    generic map(g_kind => icmp_echo_reply, g_own_mac => g_own_mac, g_own_ip => g_own_ip)
    port map(clock, ports_in(icmp_module), ports_out(icmp_module));

    u_udp_echo : entity work.ethernet_reply_module
    generic map(g_kind => udp_echo_reply, g_own_mac => g_own_mac, g_own_ip => g_own_ip, g_udp_port => g_udp_echo_port)
    port map(clock, ports_in(echo_module), ports_out(echo_module));

    u_bridge : entity work.udp_bus_bridge
    generic map(g_own_mac => g_own_mac, g_own_ip => g_own_ip, g_udp_port => g_udp_bridge_port)
    port map(
        clock               => clock
        ,port_in            => ports_in(bridge_module)
        ,port_out           => ports_out(bridge_module)
        ,bus_out            => bridge_bus_out
        ,bus_in             => bridge_bus_in
        ,request_is_granted => request_is_granted
        ,bus_timeouts       => bridge_timeouts
    );

    u_stream_request : entity work.udp_stream_request_module
    generic map(g_own_ip => g_own_ip, g_udp_port => g_udp_stream_port)
    port map(clock, ports_in(stream_module), ports_out(stream_module), stream_request);

------------------------------------------------------------------------
    u_transmitter : entity work.ethernet_transmitter
    port map(
        clock          => clock
        ,slot_is_free  => tx_slot_is_free
        ,slot_offset   => tx_slot_offset
        ,write_enable  => tx_write_enable
        ,write_address => tx_write_address
        ,write_data    => tx_write_data
        ,send          => tx_send
        ,length        => tx_length
        ,gmii_out      => tx_gmii_out
        ,frames_sent   => frames_sent
        ,may_start     => tx_may_start
        ,wants_to_send => tx_wants_to_send
        ,is_sending    => tx_is_sending
    );

------------------------------------------------------------------------
    rx_counters_rx_domain <= std_logic_vector(other_addresses) & std_logic_vector(crc_errors) & std_logic_vector(dropped_busy);

    u_rx_counters : entity work.cdc_snapshot
    generic map(48)
    port map(rx_clock, rx_counters_rx_domain, clock, rx_counters);

    process(clock)
    begin
        if rising_edge(clock) then
            for i in routes'range loop
                if ports_out(i).release_frame and ports_out(i).replied then
                    replies(i) <= replies(i) + 1;
                end if;
            end loop;

            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0 , std_logic_vector(frames));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 1 , std_logic_vector(replies(arp_module)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2 , std_logic_vector(replies(icmp_module)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3 , std_logic_vector(replies(echo_module)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4 , std_logic_vector(not_answered));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5 , x"0000" & rx_counters(15 downto 0));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6 , x"0000" & rx_counters(31 downto 16));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7 , x"0000" & rx_counters(47 downto 32));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8 , std_logic_vector(replies(bridge_module)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 9 , std_logic_vector(bridge_timeouts));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, std_logic_vector(frames_sent));
        end if;
    end process;

end architecture rtl;
