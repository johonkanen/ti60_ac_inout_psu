library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Board test design: UART register interface and HyperRAM tester.
--   address 1   : read only  0x0000acdc identification word
--   address 2   : read only  free running counter at main_clock
--   address 3   : read/write scratch register
--   address 4   : read only  pll_locked
--   address 200 : HyperRAM tester, see source/hyperram/hyperram_tester.vhd
--   address 300 : MDIO access to the ethernet PHY, see source/ethernet/mdio_bus_bridge.vhd
--   address 400 : RGMII frame tester, see source/ethernet/ethernet_tester.vhd
--   address 500 : ARP, ping and UDP echo (port 5000) at 192.168.20.10, see source/ethernet/ethernet_stack.vhd
-- The register bus is also reachable over UDP port 5001 with the UART command encoding;
-- bus_arbiter shares it between the UART and the UDP bridge.
-- the PHY is configured by ethernet_phy_init after power up, gigabit only
--   address 600 : oscilloscope capture, see source/scope/scope_capture.vhd, read out
--                 over UDP port 5002 (see source/ethernet/udp_stream_pkg.vhd)
--   address 700 : ADS7056 behind the 8 input analog mux, see source/adc/board_adc.vhd
-- main_clock is 125 MHz
entity board_test_top is
    port (
        main_clock  : in std_logic
        ;pll_locked : in std_logic

        ;uart_rx    : in std_logic
        ;uart_tx    : out std_logic
        -- ADS7056 and its input mux
        ;adc_sclk   : out std_logic
        ;adc_cs_n   : out std_logic
        ;adc_sdo    : in std_logic
        ;adc_mux0, adc_mux1, adc_mux2 : out std_logic

        -- HyperRAM through DDIO registers, see hyperram_ddr_controller
        ;hyperram_clock          : in std_logic     -- 200 MHz, 0 degrees
        ;hyperram_read_clock     : in std_logic     -- 200 MHz, dynamic phase
        ;hyperram_phase_shift    : out std_logic_vector(2 downto 0)
        ;hyperram_phase_select   : out std_logic_vector(4 downto 0)
        ;hyperram_phase_shift_ena : out std_logic
        ;hyperram_ck_hi, hyperram_ck_lo : out std_logic
        ;hyperram_cs_n           : out std_logic
        ;hyperram_reset_n        : out std_logic
        ;hyperram_dq_in_hi       : in std_logic_vector(7 downto 0)
        ;hyperram_dq_in_lo       : in std_logic_vector(7 downto 0)
        ;hyperram_dq_out_hi      : out std_logic_vector(7 downto 0)
        ;hyperram_dq_out_lo      : out std_logic_vector(7 downto 0)
        ;hyperram_dq_oe          : out std_logic_vector(7 downto 0)
        ;hyperram_rwds_in_hi, hyperram_rwds_in_lo   : in std_logic
        ;hyperram_rwds_out_hi, hyperram_rwds_out_lo : out std_logic
        ;hyperram_rwds_oe        : out std_logic

        ;ethernet_mdc      : out std_logic
        ;ethernet_mdio_in  : in std_logic
        ;ethernet_mdio_out : out std_logic
        ;ethernet_mdio_oe  : out std_logic

        ;ethernet_txd0_hi, ethernet_txd0_lo   : out std_logic
        ;ethernet_txd1_hi, ethernet_txd1_lo   : out std_logic
        ;ethernet_txd2_hi, ethernet_txd2_lo   : out std_logic
        ;ethernet_txd3_hi, ethernet_txd3_lo   : out std_logic
        ;ethernet_tx_ctl_hi, ethernet_tx_ctl_lo : out std_logic
        ;ethernet_tx_clk_hi, ethernet_tx_clk_lo : out std_logic

        ;ethernet_rx_clock      : in std_logic
        ;ethernet_rx_pll_locked : in std_logic
        ;ethernet_rx_pll_reset_n : out std_logic
        ;ethernet_rxd0_hi, ethernet_rxd0_lo   : in std_logic
        ;ethernet_rxd1_hi, ethernet_rxd1_lo   : in std_logic
        ;ethernet_rxd2_hi, ethernet_rxd2_lo   : in std_logic
        ;ethernet_rxd3_hi, ethernet_rxd3_lo   : in std_logic
        ;ethernet_rx_ctl_hi, ethernet_rx_ctl_lo : in std_logic
    );
end entity board_test_top;

architecture rtl of board_test_top is

    use work.fpga_interconnect_pkg.all;
    use work.hyperram_pkg.all;
    use work.ethernet_pkg.all;
    use work.scope_pkg.all;
    use work.udp_stream_pkg.all;

    signal bus_from_communications : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_to_communications   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_top            : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_hyperram       : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_ethernet       : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_to_slaves           : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_slaves         : fpga_interconnect_record := init_fpga_interconnect;
    signal bridge_request, bridge_response : fpga_interconnect_record := init_fpga_interconnect;
    signal bridge_is_granted       : boolean;
    signal master_granted          : boolean_vector(0 to 1);
    signal udp_bridge_granted      : std_logic;

    signal uart_rx_meta : std_logic_vector(1 downto 0) := (others => '1');

    constant id_word        : std_logic_vector(31 downto 0) := x"0000acdc";
    signal free_run_counter : unsigned(31 downto 0)         := (others => '0');
    signal scratch          : std_logic_vector(31 downto 0) := (others => '0');
    signal pll_status       : std_logic_vector(31 downto 0) := (others => '0');

    signal hyperram_in       : hyperram_in_record;
    signal hyperram_out      : hyperram_out_record;
    signal stream_source_in  : hyperram_in_record;
    signal stream_source_out : hyperram_out_record;
    signal shared_in         : hyperram_in_record;
    signal shared_out        : hyperram_out_record;
    signal stream_prefetch   : stream_prefetch_record;
    signal stream_data_is_ready : boolean;
    signal scope_read_data, hyperram_read_data : std_logic_vector(31 downto 0);
    signal controller_in     : hyperram_in_record;
    signal controller_out    : hyperram_out_record;
    signal calibration_set, calibration_lo_first, recalibrate : boolean;
    signal calibration_phase  : natural range 0 to 7;
    signal calibration_status : std_logic_vector(31 downto 0);
    signal hyperram_fpga_out : hyperram_ddr_fpga_out_record;
    signal hyperram_fpga_in  : hyperram_ddr_fpga_in_record;
    signal hyperram_phase    : pll_phase_shift_record;
    signal read_phase        : natural range 0 to 7;
    signal input_lo_first    : boolean;
    signal hyperram_debug    : std_logic_vector(95 downto 0);
    signal debug_hi, debug_lo, debug_transactions : unsigned(31 downto 0);

    signal bus_from_scope    : fpga_interconnect_record := init_fpga_interconnect;
    -- scope probes: add a signal with a line below and its entry in scope_probe_table
    constant scope_probe_count : natural := 16;
    signal scope_probes      : scope_sample_array(0 to scope_probe_count-1) := (others => (others => '0'));
    constant scope_probe_table : probe_table_array(0 to scope_probe_count-1) := (
        0  => probe_entry("counter 15:0", unsigned_probe),
        1  => probe_entry("counter 31:16", unsigned_probe),
        2  => probe_entry("scratch 15:0", unsigned_probe),
        3  => probe_entry("scratch 31:16", unsigned_probe),
        4  => probe_entry("locks", digital_probe),           -- pll, rx pll, hyperram cal
        5  => probe_entry("ADC latest", unsigned_probe),
        6  => probe_entry("ADC mux 0", unsigned_probe),
        7  => probe_entry("ADC mux 1", unsigned_probe),
        8  => probe_entry("ADC mux 2", unsigned_probe),
        9  => probe_entry("ADC mux 3", unsigned_probe),
        10 => probe_entry("ADC mux 4", unsigned_probe),
        11 => probe_entry("ADC mux 5", unsigned_probe),
        12 => probe_entry("ADC mux 6", unsigned_probe),
        13 => probe_entry("ADC mux 7", unsigned_probe),
        14 => probe_entry("bus", digital_probe),             -- write, read, uart rx, udp granted
        15 => probe_entry("bus address", unsigned_probe));
    signal bus_from_adc      : fpga_interconnect_record := init_fpga_interconnect;
    signal adc_latest        : unsigned(13 downto 0);
    signal adc_positions     : scope_sample_array(0 to 7);
    signal adc_mux           : std_logic_vector(2 downto 0);
    signal stream_read_address : natural;
    signal stream_read_data    : std_logic_vector(31 downto 0);
    signal capture_reset, capture_write, capture_flush, capture_is_full, capture_is_ready : boolean;
    signal capture_data      : std_logic_vector(127 downto 0);
    signal capture_written   : unsigned(31 downto 0);

    signal ethernet_fpga_in  : ethernet_fpga_in_record;
    signal ethernet_fpga_out : ethernet_fpga_out_record;

begin

------------------------------------------------------------------------
    process(main_clock) is
    begin
        if rising_edge(main_clock) then
            uart_rx_meta <= uart_rx_meta(0) & uart_rx;

            init_bus(bus_from_top);
            free_run_counter <= free_run_counter + 1;
            pll_status       <= (0 => pll_locked, others => '0');

            connect_read_only_data_to_address(bus_to_slaves , bus_from_top , 1 , id_word);
            connect_read_only_data_to_address(bus_to_slaves , bus_from_top , 2 , std_logic_vector(free_run_counter));
            connect_data_to_address(bus_to_slaves           , bus_from_top , 3 , scratch);
            connect_read_only_data_to_address(bus_to_slaves , bus_from_top , 4 , pll_status);

            bus_from_slaves <= bus_from_top and bus_from_hyperram and bus_from_ethernet and bus_from_scope and bus_from_adc;
        end if;
    end process;
------------------------------------------------------------------------
    u_bus_arbiter : entity work.bus_arbiter
    generic map(g_masters => 2, g_timeout => 2048)   -- HyperRAM stream reads may wait for the prefetch
    port map(
        clock          => main_clock
        ,requests(0)   => bus_from_communications    -- UART, pulses its requests
        ,requests(1)   => bridge_request             -- UDP bridge, holds until granted
        ,responses(0)  => bus_to_communications
        ,responses(1)  => bridge_response
        ,granted       => master_granted
        ,to_slaves     => bus_to_slaves
        ,from_slaves   => bus_from_slaves
    );
    bridge_is_granted <= master_granted(1);

    u_fpga_communications : entity work.fpga_communications
    generic map(fpga_interconnect_pkg => work.fpga_interconnect_pkg, g_clock_divider => 25) -- 5 Mbaud
        port map(
            clock                    => main_clock
            ,uart_rx                 => uart_rx_meta(1)
            ,uart_tx                 => uart_tx
            ,bus_to_communications   => bus_to_communications
            ,bus_from_communications => bus_from_communications
        );
------------------------------------------------------------------------
    u_hyperram_tester : entity work.hyperram_tester
    generic map(g_base_address => 200)
    port map(
        clock           => main_clock
        ,bus_in         => bus_to_slaves
        ,bus_out        => bus_from_hyperram
        ,hyperram_in    => hyperram_in
        ,hyperram_out   => hyperram_out
        ,calibration_set      => calibration_set
        ,calibration_phase    => calibration_phase
        ,calibration_lo_first => calibration_lo_first
        ,recalibrate          => recalibrate
        ,calibration_status   => calibration_status
        ,debug_counters       => hyperram_debug
    );

    -- the tester and the UDP stream source share the HyperRAM
    u_hyperram_user_arbiter : entity work.hyperram_user_arbiter
    port map(
        clock       => main_clock
        ,user0_in   => hyperram_in
        ,user0_out  => hyperram_out
        ,user1_in   => stream_source_in
        ,user1_out  => stream_source_out
        ,shared_in  => shared_in
        ,shared_out => shared_out
    );

    -- UDP streams with bit 31 of the first word set read the HyperRAM (deep captures)
    u_hyperram_stream_source : entity work.hyperram_stream_source
    port map(
        clock          => main_clock
        ,prefetch      => stream_prefetch
        ,data_is_ready => stream_data_is_ready
        ,read_address  => stream_read_address
        ,read_data     => hyperram_read_data
        ,hyperram_in   => stream_source_in
        ,hyperram_out  => stream_source_out
    );
    stream_read_data <= hyperram_read_data when stream_prefetch.is_active else scope_read_data;

    -- read clock phase calibration after power up, between the users and the controller
    u_hyperram_calibrator : entity work.hyperram_calibrator
    port map(
        clock            => main_clock
        ,user_in         => shared_in
        ,user_out        => shared_out
        ,controller_in   => controller_in
        ,controller_out  => controller_out
        ,recalibrate     => recalibrate
        ,manual_set      => calibration_set
        ,manual_phase    => calibration_phase
        ,manual_lo_first => calibration_lo_first
        ,read_phase      => read_phase
        ,input_lo_first  => input_lo_first
        ,status          => calibration_status
    );
    hyperram_debug <= std_logic_vector(debug_hi) & std_logic_vector(debug_lo) & std_logic_vector(debug_transactions);

    u_hyperram_controller : entity work.hyperram_ddr_controller
    port map(
        main_clock      => main_clock
        ,hyperram_in    => controller_in
        ,hyperram_out   => controller_out
        ,read_phase     => read_phase
        ,phase_shift    => hyperram_phase
        ,hyperram_clock => hyperram_clock
        ,fpga_out       => hyperram_fpga_out
        ,read_clock     => hyperram_read_clock
        ,fpga_in        => hyperram_fpga_in
        ,input_lo_first => input_lo_first
        ,debug_rwds_hi_samples => debug_hi
        ,debug_rwds_lo_samples => debug_lo
        ,debug_transactions    => debug_transactions
        -- scope deep capture into the whole memory
        ,capture_reset        => capture_reset
        ,capture_write        => capture_write
        ,capture_data         => capture_data
        ,capture_flush        => capture_flush
        ,capture_is_full      => capture_is_full
        ,capture_is_ready     => capture_is_ready
        ,capture_written      => capture_written
        ,capture_base         => 0
        ,capture_ring_records => 2**22
    );

    hyperram_phase_shift     <= hyperram_phase.shift;
    hyperram_phase_select    <= hyperram_phase.shift_select;
    hyperram_phase_shift_ena <= hyperram_phase.shift_enable;
    hyperram_ck_hi       <= hyperram_fpga_out.ck_hi;
    hyperram_ck_lo       <= hyperram_fpga_out.ck_lo;
    hyperram_cs_n        <= hyperram_fpga_out.cs_n;
    hyperram_reset_n     <= hyperram_fpga_out.reset_n;
    hyperram_dq_out_hi   <= hyperram_fpga_out.dq_hi;
    hyperram_dq_out_lo   <= hyperram_fpga_out.dq_lo;
    hyperram_dq_oe       <= (others => hyperram_fpga_out.dq_oe);
    hyperram_rwds_out_hi <= hyperram_fpga_out.rwds_hi;
    hyperram_rwds_out_lo <= hyperram_fpga_out.rwds_lo;
    hyperram_rwds_oe     <= hyperram_fpga_out.rwds_oe;
    hyperram_fpga_in     <= (dq_hi => hyperram_dq_in_hi, dq_lo => hyperram_dq_in_lo,
                             rwds_hi => hyperram_rwds_in_hi, rwds_lo => hyperram_rwds_in_lo);
------------------------------------------------------------------------
    u_ethernet : entity work.ethernet
    generic map(g_base_address => 300)   -- MDIO 300, loopback test 400, protocols 500
    port map(
        clock               => main_clock
        ,bus_in             => bus_to_slaves
        ,bus_out            => bus_from_ethernet
        ,bridge_bus_out     => bridge_request
        ,bridge_bus_in      => bridge_response
        ,request_is_granted => bridge_is_granted
        ,stream_read_address => stream_read_address
        ,stream_read_data    => stream_read_data
        ,stream_prefetch     => stream_prefetch
        ,stream_data_is_ready => stream_data_is_ready
        ,rx_clock           => ethernet_rx_clock
        ,fpga_in            => ethernet_fpga_in
        ,fpga_out           => ethernet_fpga_out
    );

    ethernet_fpga_in <= (
        rgmii_rx => (
            hi => (ethernet_rx_ctl_hi, ethernet_rxd3_hi, ethernet_rxd2_hi, ethernet_rxd1_hi, ethernet_rxd0_hi),
            lo => (ethernet_rx_ctl_lo, ethernet_rxd3_lo, ethernet_rxd2_lo, ethernet_rxd1_lo, ethernet_rxd0_lo)),
        rx_pll_locked => ethernet_rx_pll_locked,
        mdio_in       => ethernet_mdio_in);

    (ethernet_tx_ctl_hi, ethernet_txd3_hi, ethernet_txd2_hi, ethernet_txd1_hi, ethernet_txd0_hi) <= ethernet_fpga_out.rgmii_tx.hi;
    (ethernet_tx_ctl_lo, ethernet_txd3_lo, ethernet_txd2_lo, ethernet_txd1_lo, ethernet_txd0_lo) <= ethernet_fpga_out.rgmii_tx.lo;
    ethernet_tx_clk_hi      <= ethernet_fpga_out.tx_clock.hi(0);
    ethernet_tx_clk_lo      <= ethernet_fpga_out.tx_clock.lo(0);
    ethernet_mdc            <= ethernet_fpga_out.mdc;
    ethernet_mdio_out       <= ethernet_fpga_out.mdio_out;
    ethernet_mdio_oe        <= ethernet_fpga_out.mdio_oe;
    ethernet_rx_pll_reset_n <= ethernet_fpga_out.rx_pll_reset_n;
------------------------------------------------------------------------
    u_scope : entity work.scope_capture
    generic map(g_base_address => 600, g_depth => 4096, g_deep_records => 2**22, g_probes => scope_probe_count)
    port map(
        clock         => main_clock
        ,bus_in       => bus_to_slaves
        ,bus_out      => bus_from_scope
        ,probes       => scope_probes
        ,probe_table  => scope_probe_table
        ,read_address => stream_read_address
        ,read_data    => scope_read_data
        ,capture_reset   => capture_reset
        ,capture_write   => capture_write
        ,capture_data    => capture_data
        ,capture_flush   => capture_flush
        ,capture_is_full => capture_is_full
        ,capture_is_ready => capture_is_ready
        ,capture_written => capture_written
    );

    -- scope probes, sources 8 and up; all in the main clock domain except the PLL locks
    scope_probes(0)  <= to_probe(free_run_counter(15 downto 0));
    scope_probes(1)  <= to_probe(free_run_counter(31 downto 16));
    scope_probes(2)  <= to_probe(scratch(15 downto 0));
    scope_probes(3)  <= to_probe(scratch(31 downto 16));
    scope_probes(4)  <= to_digital(calibration_status(0) & ethernet_rx_pll_locked & pll_locked);
    scope_probes(5)  <= to_probe(adc_latest);
    scope_probes(6 to 13) <= adc_positions;
    udp_bridge_granted <= '1' when master_granted(1) else '0';
    scope_probes(14) <= to_digital(udp_bridge_granted & uart_rx_meta(1)
                                   & not bus_to_slaves.data_read_is_requested_with_0
                                   & not bus_to_slaves.data_write_is_requested_with_0);
    scope_probes(15) <= to_probe(bus_to_slaves.address);

------------------------------------------------------------------------
    u_board_adc : entity work.board_adc
    generic map(g_base_address => 700)
    port map(
        clock      => main_clock
        ,bus_in    => bus_to_slaves
        ,bus_out   => bus_from_adc
        ,latest    => adc_latest
        ,positions => adc_positions
        ,is_new    => open
        ,adc_clock => hyperram_clock      -- 200 MHz: 50 MHz SCLK, 2.5 MS/s
        ,mux       => adc_mux
        ,sclk      => adc_sclk
        ,cs_n      => adc_cs_n
        ,sdo       => adc_sdo
    );
    (adc_mux2, adc_mux1, adc_mux0) <= adc_mux;
------------------------------------------------------------------------
end rtl;
