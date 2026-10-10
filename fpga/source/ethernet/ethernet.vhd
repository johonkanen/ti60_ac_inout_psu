library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.mdio_pkg.all;
    use work.fpga_interconnect_pkg.all;
    use work.udp_stream_pkg.all;

-- Gigabit ethernet over RGMII with a PEF7071 PHY:
--   MDIO driver, PHY configuration kept up by ethernet_phy_init and MDIO access from the bus
--   RGMII transmit and receive, the receive PLL is reset when it stays unlocked and when
--   the link comes up
--   optional loopback test generator and checker, which has priority on the transmitter
--   protocol modules: ARP, ICMP echo, UDP echo and the UDP register bridge
--   UDP streaming of words from a data source (the oscilloscope capture), requested on
--   g_udp_stream_port, see udp_stream_pkg
--
-- register blocks, offsets from g_base_address
--   +0    MDIO access, see mdio_bus_bridge
--   +100  RGMII loopback test, see ethernet_tester
--   +200  protocol counters, see ethernet_stack
-- The UDP register bridge is a bus master: connect bridge_bus_* to a bus_arbiter.
entity ethernet is
    generic(
        g_base_address            : natural := 300;
        g_own_mac                 : mac_address := x"020000000001";
        g_own_ip                  : ip_address  := x"C0A8140A";   -- 192.168.20.10
        g_udp_echo_port           : natural := 5000;
        g_udp_bridge_port         : natural := 5001;
        g_udp_stream_port         : natural := 5002;
        g_stream_read_latency     : natural := 3;
        g_phy_address             : natural := 0;
        g_clocks_per_half_mdc     : positive := 25;               -- 2.5 MHz MDC at 125 MHz
        g_include_loopback_tester : boolean := true
    );
    port(
        clock              : in std_logic;   -- 125 MHz, also the RGMII transmit clock
        bus_in             : in fpga_interconnect_record;
        bus_out            : out fpga_interconnect_record;

        bridge_bus_out     : out fpga_interconnect_record;
        bridge_bus_in      : in fpga_interconnect_record;
        request_is_granted : in boolean;

        -- data source of the UDP stream, data g_stream_read_latency clocks after the address
        stream_read_address : out natural;
        stream_read_data    : in std_logic_vector(31 downto 0) := (others => '0');
        -- prefetching data source for streams with bit 31 of the first word set
        stream_prefetch      : out stream_prefetch_record;
        stream_data_is_ready : in boolean := true;

        rx_clock           : in std_logic;   -- from the PLL locked to RX_CLK
        fpga_in            : in ethernet_fpga_in_record;
        fpga_out           : out ethernet_fpga_out_record
    );
end entity ethernet;

architecture rtl of ethernet is

    constant mdio_base   : natural := g_base_address;
    constant tester_base : natural := g_base_address + 100;
    constant stack_base  : natural := g_base_address + 200;

    signal bus_from_mdio, bus_from_tester, bus_from_stack : fpga_interconnect_record := init_fpga_interconnect;

    signal mdio_from_bridge : mdio_in_record;
    signal mdio_to_bridge   : mdio_out_record;
    signal phy_status       : std_logic_vector(31 downto 0);
    signal link_came_up     : boolean;
    signal mdio_in          : mdio_in_record;
    signal mdio_out         : mdio_out_record;
    signal mdio_fpga_out    : mdio_fpga_out_record;

    signal stack_tx_gmii, tester_tx_gmii, tx_gmii : gmii_record := gmii_idle;
    signal stream_gmii, line_gmii : gmii_record := gmii_idle;
    signal stream_request : stream_request_record;
    signal may_start, wants_to_send, is_sending : boolean_vector(0 to 1);
    signal frame_sources : gmii_record_array(0 to 1);
    signal tester_is_sending : boolean := false;
    signal delay_lo_half     : boolean := false;
    signal hold_rx_pll_reset : boolean := false;

    signal rx_gmii           : gmii_record;
    signal rx_frame_is_ended : boolean;
    signal rx_alignment      : natural range 0 to 3;

    -- the rx pll is reset when it has been unlocked for 1 ms, for example after the link
    -- has changed speed or come back up
    constant relock_timeout : natural := 125_000;
    constant reset_length   : natural := 1_250;
    signal rx_pll_locked_sync : std_logic_vector(1 downto 0) := (others => '0');
    signal unlocked_time      : natural range 0 to relock_timeout := 0;
    signal relock_reset       : natural range 0 to reset_length := 0;

begin

    bus_out <= bus_from_mdio and bus_from_tester and bus_from_stack;

------------------------------------------------------------------------
    u_mdio_bus_bridge : entity work.mdio_bus_bridge
    generic map(g_base_address => mdio_base)
    port map(
        clock     => clock
        ,bus_in   => bus_in
        ,bus_out  => bus_from_mdio
        ,mdio_in  => mdio_from_bridge
        ,mdio_out => mdio_to_bridge
        ,phy_status => phy_status
    );

    u_phy_init : entity work.ethernet_phy_init
    generic map(g_phy_address => g_phy_address)
    port map(
        clock      => clock
        ,bridge_in    => mdio_from_bridge
        ,bridge_out   => mdio_to_bridge
        ,mdio_in      => mdio_in
        ,mdio_out     => mdio_out
        ,is_done      => open
        ,link_is_up   => open
        ,link_came_up => link_came_up
        ,status       => phy_status
    );

    u_mdio_driver : entity work.mdio_driver
    generic map(g_clocks_per_half_mdc => g_clocks_per_half_mdc)
    port map(
        clock         => clock
        ,mdio_in      => mdio_in
        ,mdio_out     => mdio_out
        ,fpga_out     => mdio_fpga_out
        ,fpga_mdio_in => fpga_in.mdio_in
    );

    fpga_out.mdc      <= mdio_fpga_out.mdc;
    fpga_out.mdio_out <= mdio_fpga_out.mdio_out;
    fpga_out.mdio_oe  <= mdio_fpga_out.mdio_oe;

------------------------------------------------------------------------
    -- the stack replies and the stream share the line frame by frame,
    -- the loopback test generator has priority while it runs
    u_frame_arbiter : entity work.gmii_frame_arbiter
    port map(
        clock          => clock
        ,gmii_in       => frame_sources
        ,wants_to_send => wants_to_send
        ,is_sending    => is_sending
        ,may_start     => may_start
        ,gmii_out      => line_gmii
    );

    frame_sources <= (0 => stack_tx_gmii, 1 => stream_gmii);
    tx_gmii <= tester_tx_gmii when tester_is_sending else line_gmii;

    u_stream_sender : entity work.udp_stream_sender
    generic map(g_own_mac => g_own_mac, g_own_ip => g_own_ip, g_read_latency => g_stream_read_latency)
    port map(
        clock          => clock
        ,request       => stream_request
        ,read_address  => stream_read_address
        ,read_data     => stream_read_data
        ,prefetch      => stream_prefetch
        ,data_is_ready => stream_data_is_ready
        ,may_start     => may_start(1)
        ,wants_to_send => wants_to_send(1)
        ,is_sending    => is_sending(1)
        ,gmii_out      => stream_gmii
        ,datagrams     => open
    );

    u_rgmii_tx : entity work.rgmii_tx
    port map(
        clock          => clock
        ,gmii_in       => tx_gmii
        ,delay_lo_half => delay_lo_half
        ,rgmii_out     => fpga_out.rgmii_tx
        ,tx_clock_out  => fpga_out.tx_clock
    );

    u_rgmii_rx : entity work.rgmii_rx
    port map(
        clock           => rx_clock
        ,rgmii_in       => fpga_in.rgmii_rx
        ,gmii_out       => rx_gmii
        ,alignment      => rx_alignment
        ,frame_is_ended => rx_frame_is_ended
    );

    fpga_out.rx_pll_reset_n <= '0' when hold_rx_pll_reset or relock_reset > 0 else '1';

    rx_pll_relock : process(clock)
    begin
        if rising_edge(clock) then
            rx_pll_locked_sync <= rx_pll_locked_sync(0) & fpga_in.rx_pll_locked;
            if link_came_up then
                -- RX_CLK may have changed while the pll stayed locked
                relock_reset  <= reset_length;
                unlocked_time <= 0;
            elsif relock_reset > 0 then
                relock_reset  <= relock_reset - 1;
                unlocked_time <= 0;
            elsif rx_pll_locked_sync(1) = '1' then
                unlocked_time <= 0;
            elsif unlocked_time < relock_timeout then
                unlocked_time <= unlocked_time + 1;
            else
                relock_reset <= reset_length;
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    loopback_tester : if g_include_loopback_tester generate
        u_ethernet_tester : entity work.ethernet_tester
        generic map(g_base_address => tester_base)
        port map(
            clock              => clock
            ,bus_in            => bus_in
            ,bus_out           => bus_from_tester
            ,tx_gmii_out       => tester_tx_gmii
            ,tx_is_busy        => tester_is_sending
            ,delay_lo_half     => delay_lo_half
            ,hold_rx_pll_reset => hold_rx_pll_reset
            ,rx_pll_locked     => fpga_in.rx_pll_locked
            ,rx_clock          => rx_clock
            ,rx_gmii_in        => rx_gmii
            ,rx_frame_is_ended => rx_frame_is_ended
            ,rx_alignment      => rx_alignment
        );
    end generate;

------------------------------------------------------------------------
    u_ethernet_stack : entity work.ethernet_stack
    generic map(
        g_base_address     => stack_base
        ,g_own_mac         => g_own_mac
        ,g_own_ip          => g_own_ip
        ,g_udp_echo_port   => g_udp_echo_port
        ,g_udp_bridge_port => g_udp_bridge_port
        ,g_udp_stream_port => g_udp_stream_port)
    port map(
        clock               => clock
        ,bus_in             => bus_in
        ,bus_out            => bus_from_stack
        ,rx_clock           => rx_clock
        ,rx_gmii_in         => rx_gmii
        ,rx_frame_is_ended  => rx_frame_is_ended
        ,tx_gmii_out        => stack_tx_gmii
        ,tx_may_start       => may_start(0)
        ,tx_wants_to_send   => wants_to_send(0)
        ,tx_is_sending      => is_sending(0)
        ,stream_request     => stream_request
        ,bridge_bus_out     => bridge_bus_out
        ,bridge_bus_in      => bridge_bus_in
        ,request_is_granted => request_is_granted
    );

end architecture rtl;
