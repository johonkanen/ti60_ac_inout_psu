library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;

-- The Efinix IP cores of tsemac_soc, as Efinix's Ti60F225 TSEMAC Linux example design
-- connects them (its top_ti60.v):
--   SapphireCore  the RISC-V SoC, ip/SapphireCore (gen_ips.py)
--   u_hyperram    our HyperRAM controller (hyperram_axi), the SoC's main memory on its DDR port
--   ethernet_subsystem  ethernet/ethernet_subsystem.vhd: the TSEMAC and its scatter-gather DMA
--                 on DDR master port 0, registers on APB slaves 0 (MAC, control) and 1 (DMA)
--   dma_scope     a DMA for the oscilloscope, on DDR master port 1, registers on APB slave 2:
--                 writes scope_stream into a circular buffer in the HyperRAM
-- APB slave 3 is brought out for board_test's register bus (bus_apb, apb_bus_bridge in the
-- top level): the scope's and the ADC's registers.
--
-- Clocks: ram_clk 200 MHz is also the SoC's system and memory clock, io_tseClk 125 MHz its
-- peripheral clock (Efinix's design: 100 MHz) and the clock of the scope ports. The SoC is
-- held in reset until the PLL is locked and the HyperRAM calibrated.
entity sapphire_subsystem is
    generic(g_own_ethernet_mac : boolean := false);   -- ethernet_mac in place of the TSEMAC
    port (
        pll_locked  : in std_logic
        ;ram_clk    : in std_logic     -- 200 MHz
        ;ram_clk_cal : in std_logic    -- 200 MHz, phase set by the HyperRAM calibration
        ;io_tseClk  : in std_logic     -- 125 MHz
        ;hbc_cal_SHIFT_SEL : out std_logic_vector(4 downto 0)
        ;hbc_cal_SHIFT     : out std_logic_vector(2 downto 0)
        ;hbc_cal_SHIFT_ENA : out std_logic

        -- HyperRAM, x8, through the IO registers gen_peri.py sets up
        ;hbc_dq_OE       : out std_logic_vector(7 downto 0)
        ;hbc_dq_IN_LO    : in std_logic_vector(7 downto 0)
        ;hbc_dq_IN_HI    : in std_logic_vector(7 downto 0)
        ;hbc_dq_OUT_LO   : out std_logic_vector(7 downto 0)
        ;hbc_dq_OUT_HI   : out std_logic_vector(7 downto 0)
        ;hbc_rwds_OE     : out std_logic_vector(0 downto 0)
        ;hbc_rwds_IN_LO  : in std_logic_vector(0 downto 0)
        ;hbc_rwds_IN_HI  : in std_logic_vector(0 downto 0)
        ;hbc_rwds_OUT_LO : out std_logic_vector(0 downto 0)
        ;hbc_rwds_OUT_HI : out std_logic_vector(0 downto 0)
        ;hbc_ck_p_LO     : out std_logic
        ;hbc_ck_p_HI     : out std_logic
        ;hbc_cs_n        : out std_logic
        ;hbc_rst_n       : out std_logic

        -- RGMII and MDIO of the TSEMAC
        ;rgmii_rxc       : in std_logic
        ;rgmii_txd_HI    : out std_logic_vector(3 downto 0)
        ;rgmii_txd_LO    : out std_logic_vector(3 downto 0)
        ;rgmii_tx_ctl_HI : out std_logic
        ;rgmii_tx_ctl_LO : out std_logic
        ;rgmii_txc_HI    : out std_logic
        ;rgmii_txc_LO    : out std_logic
        ;rgmii_rxd_HI    : in std_logic_vector(3 downto 0)
        ;rgmii_rxd_LO    : in std_logic_vector(3 downto 0)
        ;rgmii_rx_ctl_HI : in std_logic
        ;rgmii_rx_ctl_LO : in std_logic
        ;phy_mdi         : in std_logic
        ;phy_mdo         : out std_logic
        ;phy_mdo_en      : out std_logic
        ;phy_mdc         : out std_logic

        ;system_uart_0_io_txd : out std_logic
        ;system_uart_0_io_rxd : in std_logic
        ;system_spi_0_io_sclk_write         : out std_logic
        ;system_spi_0_io_ss                 : out std_logic
        ;system_spi_0_io_data_0_writeEnable : out std_logic
        ;system_spi_0_io_data_0_read        : in std_logic
        ;system_spi_0_io_data_0_write       : out std_logic
        ;system_spi_0_io_data_1_writeEnable : out std_logic
        ;system_spi_0_io_data_1_read        : in std_logic
        ;system_spi_0_io_data_1_write       : out std_logic

        -- FPGA JTAG TAP USER1, the SoC's RISC-V debug module
        ;jtag_inst1_TCK     : in std_logic
        ;jtag_inst1_TDI     : in std_logic
        ;jtag_inst1_TDO     : out std_logic
        ;jtag_inst1_SEL     : in std_logic
        ;jtag_inst1_CAPTURE : in std_logic
        ;jtag_inst1_SHIFT   : in std_logic
        ;jtag_inst1_UPDATE  : in std_logic
        ;jtag_inst1_RESET   : in std_logic

        -- in io_tseClk: APB slave 3, the register bus, and the scope's capture stream
        ;bus_apb_psel      : out std_logic
        ;bus_apb_penable   : out std_logic
        ;bus_apb_pwrite    : out std_logic
        ;bus_apb_paddr     : out std_logic_vector(15 downto 0)
        ;bus_apb_pwdata    : out std_logic_vector(31 downto 0)
        ;bus_apb_prdata    : in std_logic_vector(31 downto 0)
        ;bus_apb_pready    : in std_logic
        ;bus_apb_pslverror : in std_logic
        ;scope_tvalid : in std_logic
        ;scope_tready : out std_logic
        ;scope_tdata  : in std_logic_vector(63 downto 0)
        ;scope_tkeep  : in std_logic_vector(7 downto 0)
        ;scope_tlast  : in std_logic

        -- status, synchronous to io_tseClk except memory_ready (static once the SoC runs)
        ;memory_ready     : out std_logic   -- HyperRAM calibrated
        -- hyperram_axi's calibration status, read timeouts and debug word, in ram_clk
        ;hyperram_status   : out std_logic_vector(31 downto 0)
        ;hyperram_timeouts : out std_logic_vector(15 downto 0)
        ;hyperram_debug    : out std_logic_vector(63 downto 0)
        ;ethernet_speed   : out std_logic_vector(2 downto 0)
        ;ethernet_dma_interrupts : out std_logic_vector(1 downto 0)
        -- the TSEMAC control registers' PHY reset bit: no pin on this board, the top level
        -- restarts the receive PLL with it
        ;phy_reset        : out std_logic
    );
end entity sapphire_subsystem;

architecture rtl of sapphire_subsystem is

    function to_std(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;


    -- component declarations: ports sorted by name, from the IPs' Verilog (gen_ips.py)

    component SapphireCore is
    port(
        io_apbSlave_0_PADDR                : out std_logic_vector(15 downto 0);
        io_apbSlave_0_PENABLE              : out std_logic;
        io_apbSlave_0_PRDATA               : in  std_logic_vector(31 downto 0);
        io_apbSlave_0_PREADY               : in  std_logic;
        io_apbSlave_0_PSEL                 : out std_logic;
        io_apbSlave_0_PSLVERROR            : in  std_logic;
        io_apbSlave_0_PWDATA               : out std_logic_vector(31 downto 0);
        io_apbSlave_0_PWRITE               : out std_logic;
        io_apbSlave_1_PADDR                : out std_logic_vector(15 downto 0);
        io_apbSlave_1_PENABLE              : out std_logic;
        io_apbSlave_1_PRDATA               : in  std_logic_vector(31 downto 0);
        io_apbSlave_1_PREADY               : in  std_logic;
        io_apbSlave_1_PSEL                 : out std_logic;
        io_apbSlave_1_PSLVERROR            : in  std_logic;
        io_apbSlave_1_PWDATA               : out std_logic_vector(31 downto 0);
        io_apbSlave_1_PWRITE               : out std_logic;
        io_apbSlave_2_PADDR                : out std_logic_vector(15 downto 0);
        io_apbSlave_2_PENABLE              : out std_logic;
        io_apbSlave_2_PRDATA               : in  std_logic_vector(31 downto 0);
        io_apbSlave_2_PREADY               : in  std_logic;
        io_apbSlave_2_PSEL                 : out std_logic;
        io_apbSlave_2_PSLVERROR            : in  std_logic;
        io_apbSlave_2_PWDATA               : out std_logic_vector(31 downto 0);
        io_apbSlave_2_PWRITE               : out std_logic;
        io_apbSlave_3_PADDR                : out std_logic_vector(15 downto 0);
        io_apbSlave_3_PENABLE              : out std_logic;
        io_apbSlave_3_PRDATA               : in  std_logic_vector(31 downto 0);
        io_apbSlave_3_PREADY               : in  std_logic;
        io_apbSlave_3_PSEL                 : out std_logic;
        io_apbSlave_3_PSLVERROR            : in  std_logic;
        io_apbSlave_3_PWDATA               : out std_logic_vector(31 downto 0);
        io_apbSlave_3_PWRITE               : out std_logic;
        io_asyncReset                      : in  std_logic;
        io_ddrA_arw_payload_addr           : out std_logic_vector(31 downto 0);
        io_ddrA_arw_payload_burst          : out std_logic_vector(1 downto 0);
        io_ddrA_arw_payload_cache          : out std_logic_vector(3 downto 0);
        io_ddrA_arw_payload_id             : out std_logic_vector(7 downto 0);
        io_ddrA_arw_payload_len            : out std_logic_vector(7 downto 0);
        io_ddrA_arw_payload_lock           : out std_logic;
        io_ddrA_arw_payload_prot           : out std_logic_vector(2 downto 0);
        io_ddrA_arw_payload_qos            : out std_logic_vector(3 downto 0);
        io_ddrA_arw_payload_region         : out std_logic_vector(3 downto 0);
        io_ddrA_arw_payload_size           : out std_logic_vector(2 downto 0);
        io_ddrA_arw_payload_write          : out std_logic;
        io_ddrA_arw_ready                  : in  std_logic;
        io_ddrA_arw_valid                  : out std_logic;
        io_ddrA_b_payload_id               : in  std_logic_vector(7 downto 0);
        io_ddrA_b_payload_resp             : in  std_logic_vector(1 downto 0);
        io_ddrA_b_ready                    : out std_logic;
        io_ddrA_b_valid                    : in  std_logic;
        io_ddrA_r_payload_data             : in  std_logic_vector(63 downto 0);
        io_ddrA_r_payload_id               : in  std_logic_vector(7 downto 0);
        io_ddrA_r_payload_last             : in  std_logic;
        io_ddrA_r_payload_resp             : in  std_logic_vector(1 downto 0);
        io_ddrA_r_ready                    : out std_logic;
        io_ddrA_r_valid                    : in  std_logic;
        io_ddrA_w_payload_data             : out std_logic_vector(63 downto 0);
        io_ddrA_w_payload_id               : out std_logic_vector(7 downto 0);
        io_ddrA_w_payload_last             : out std_logic;
        io_ddrA_w_payload_strb             : out std_logic_vector(7 downto 0);
        io_ddrA_w_ready                    : in  std_logic;
        io_ddrA_w_valid                    : out std_logic;
        io_ddrMasters_0_ar_payload_addr    : in  std_logic_vector(31 downto 0);
        io_ddrMasters_0_ar_payload_burst   : in  std_logic_vector(1 downto 0);
        io_ddrMasters_0_ar_payload_cache   : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_ar_payload_id      : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_ar_payload_len     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_0_ar_payload_lock    : in  std_logic;
        io_ddrMasters_0_ar_payload_prot    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_0_ar_payload_qos     : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_ar_payload_region  : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_ar_payload_size    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_0_ar_ready           : out std_logic;
        io_ddrMasters_0_ar_valid           : in  std_logic;
        io_ddrMasters_0_aw_payload_addr    : in  std_logic_vector(31 downto 0);
        io_ddrMasters_0_aw_payload_burst   : in  std_logic_vector(1 downto 0);
        io_ddrMasters_0_aw_payload_cache   : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_aw_payload_id      : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_aw_payload_len     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_0_aw_payload_lock    : in  std_logic;
        io_ddrMasters_0_aw_payload_prot    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_0_aw_payload_qos     : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_aw_payload_region  : in  std_logic_vector(3 downto 0);
        io_ddrMasters_0_aw_payload_size    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_0_aw_ready           : out std_logic;
        io_ddrMasters_0_aw_valid           : in  std_logic;
        io_ddrMasters_0_b_payload_id       : out std_logic_vector(3 downto 0);
        io_ddrMasters_0_b_payload_resp     : out std_logic_vector(1 downto 0);
        io_ddrMasters_0_b_ready            : in  std_logic;
        io_ddrMasters_0_b_valid            : out std_logic;
        io_ddrMasters_0_clk                : in  std_logic;
        io_ddrMasters_0_r_payload_data     : out std_logic_vector(63 downto 0);
        io_ddrMasters_0_r_payload_id       : out std_logic_vector(3 downto 0);
        io_ddrMasters_0_r_payload_last     : out std_logic;
        io_ddrMasters_0_r_payload_resp     : out std_logic_vector(1 downto 0);
        io_ddrMasters_0_r_ready            : in  std_logic;
        io_ddrMasters_0_r_valid            : out std_logic;
        io_ddrMasters_0_reset              : out std_logic;
        io_ddrMasters_0_w_payload_data     : in  std_logic_vector(63 downto 0);
        io_ddrMasters_0_w_payload_last     : in  std_logic;
        io_ddrMasters_0_w_payload_strb     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_0_w_ready            : out std_logic;
        io_ddrMasters_0_w_valid            : in  std_logic;
        io_ddrMasters_1_ar_payload_addr    : in  std_logic_vector(31 downto 0);
        io_ddrMasters_1_ar_payload_burst   : in  std_logic_vector(1 downto 0);
        io_ddrMasters_1_ar_payload_cache   : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_ar_payload_id      : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_ar_payload_len     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_1_ar_payload_lock    : in  std_logic;
        io_ddrMasters_1_ar_payload_prot    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_1_ar_payload_qos     : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_ar_payload_region  : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_ar_payload_size    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_1_ar_ready           : out std_logic;
        io_ddrMasters_1_ar_valid           : in  std_logic;
        io_ddrMasters_1_aw_payload_addr    : in  std_logic_vector(31 downto 0);
        io_ddrMasters_1_aw_payload_burst   : in  std_logic_vector(1 downto 0);
        io_ddrMasters_1_aw_payload_cache   : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_aw_payload_id      : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_aw_payload_len     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_1_aw_payload_lock    : in  std_logic;
        io_ddrMasters_1_aw_payload_prot    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_1_aw_payload_qos     : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_aw_payload_region  : in  std_logic_vector(3 downto 0);
        io_ddrMasters_1_aw_payload_size    : in  std_logic_vector(2 downto 0);
        io_ddrMasters_1_aw_ready           : out std_logic;
        io_ddrMasters_1_aw_valid           : in  std_logic;
        io_ddrMasters_1_b_payload_id       : out std_logic_vector(3 downto 0);
        io_ddrMasters_1_b_payload_resp     : out std_logic_vector(1 downto 0);
        io_ddrMasters_1_b_ready            : in  std_logic;
        io_ddrMasters_1_b_valid            : out std_logic;
        io_ddrMasters_1_clk                : in  std_logic;
        io_ddrMasters_1_r_payload_data     : out std_logic_vector(63 downto 0);
        io_ddrMasters_1_r_payload_id       : out std_logic_vector(3 downto 0);
        io_ddrMasters_1_r_payload_last     : out std_logic;
        io_ddrMasters_1_r_payload_resp     : out std_logic_vector(1 downto 0);
        io_ddrMasters_1_r_ready            : in  std_logic;
        io_ddrMasters_1_r_valid            : out std_logic;
        io_ddrMasters_1_reset              : out std_logic;
        io_ddrMasters_1_w_payload_data     : in  std_logic_vector(63 downto 0);
        io_ddrMasters_1_w_payload_last     : in  std_logic;
        io_ddrMasters_1_w_payload_strb     : in  std_logic_vector(7 downto 0);
        io_ddrMasters_1_w_ready            : out std_logic;
        io_ddrMasters_1_w_valid            : in  std_logic;
        io_memoryClk                       : in  std_logic;
        io_memoryReset                     : out std_logic;
        io_peripheralClk                   : in  std_logic;
        io_peripheralReset                 : out std_logic;
        io_systemClk                       : in  std_logic;
        io_systemReset                     : out std_logic;
        jtagCtrl_capture                   : in  std_logic;
        jtagCtrl_enable                    : in  std_logic;
        jtagCtrl_reset                     : in  std_logic;
        jtagCtrl_shift                     : in  std_logic;
        jtagCtrl_tck                       : in  std_logic;
        jtagCtrl_tdi                       : in  std_logic;
        jtagCtrl_tdo                       : out std_logic;
        jtagCtrl_update                    : in  std_logic;
        system_spi_0_io_data_0_read        : in  std_logic;
        system_spi_0_io_data_0_write       : out std_logic;
        system_spi_0_io_data_0_writeEnable : out std_logic;
        system_spi_0_io_data_1_read        : in  std_logic;
        system_spi_0_io_data_1_write       : out std_logic;
        system_spi_0_io_data_1_writeEnable : out std_logic;
        system_spi_0_io_data_2_read        : in  std_logic;
        system_spi_0_io_data_2_write       : out std_logic;
        system_spi_0_io_data_2_writeEnable : out std_logic;
        system_spi_0_io_data_3_read        : in  std_logic;
        system_spi_0_io_data_3_write       : out std_logic;
        system_spi_0_io_data_3_writeEnable : out std_logic;
        system_spi_0_io_sclk_write         : out std_logic;
        system_spi_0_io_ss                 : out std_logic_vector(0 downto 0);
        system_uart_0_io_rxd               : in  std_logic;
        system_uart_0_io_txd               : out std_logic;
        userInterruptA                     : in  std_logic;
        userInterruptB                     : in  std_logic
    );
    end component SapphireCore;



    component dma_scope is
    port(
        clk             : in  std_logic;
        ctrl_clk        : in  std_logic;
        ctrl_interrupts : out std_logic_vector(0 downto 0);
        ctrl_PADDR      : in  std_logic_vector(13 downto 0);
        ctrl_PENABLE    : in  std_logic;
        ctrl_PRDATA     : out std_logic_vector(31 downto 0);
        ctrl_PREADY     : out std_logic;
        ctrl_PSEL       : in  std_logic;
        ctrl_PSLVERROR  : out std_logic;
        ctrl_PWDATA     : in  std_logic_vector(31 downto 0);
        ctrl_PWRITE     : in  std_logic;
        ctrl_reset      : in  std_logic;
        dat0_i_clk      : in  std_logic;
        dat0_i_reset    : in  std_logic;
        dat0_i_tdata    : in  std_logic_vector(63 downto 0);
        dat0_i_tdest    : in  std_logic_vector(3 downto 0);
        dat0_i_tkeep    : in  std_logic_vector(7 downto 0);
        dat0_i_tlast    : in  std_logic;
        dat0_i_tready   : out std_logic;
        dat0_i_tvalid   : in  std_logic;
        reset           : in  std_logic;
        write_awaddr    : out std_logic_vector(31 downto 0);
        write_awburst   : out std_logic_vector(1 downto 0);
        write_awcache   : out std_logic_vector(3 downto 0);
        write_awlen     : out std_logic_vector(7 downto 0);
        write_awlock    : out std_logic;
        write_awprot    : out std_logic_vector(2 downto 0);
        write_awqos     : out std_logic_vector(3 downto 0);
        write_awready   : in  std_logic;
        write_awregion  : out std_logic_vector(3 downto 0);
        write_awsize    : out std_logic_vector(2 downto 0);
        write_awvalid   : out std_logic;
        write_bready    : out std_logic;
        write_bresp     : in  std_logic_vector(1 downto 0);
        write_bvalid    : in  std_logic;
        write_wdata     : out std_logic_vector(63 downto 0);
        write_wlast     : out std_logic;
        write_wready    : in  std_logic;
        write_wstrb     : out std_logic_vector(7 downto 0);
        write_wvalid    : out std_logic
    );
    end component dma_scope;

    signal io_systemClk, io_memoryClk, io_peripheralClk : std_logic;
    signal io_asyncReset      : std_logic;
    signal io_systemReset     : std_logic;
    signal io_memoryReset     : std_logic;
    signal io_peripheralReset : std_logic;
    signal io_ddrMaster0Reset : std_logic;
    signal io_ddrMaster1Reset : std_logic;
    signal calibration_done   : boolean;
    signal read_timeouts      : unsigned(15 downto 0);
    signal phase_shift        : pll_phase_shift_record;
    signal hyperram_out       : hyperram_ddr_fpga_out_record;
    signal hyperram_in        : hyperram_ddr_fpga_in_record;

    -- the SoC's memory port to the HyperRAM controller
    signal ddr_arw_valid, ddr_arw_ready, ddr_arw_lock, ddr_arw_write : std_logic;
    signal ddr_arw_addr       : std_logic_vector(31 downto 0);
    signal ddr_arw_id, ddr_arw_len : std_logic_vector(7 downto 0);
    signal ddr_arw_size       : std_logic_vector(2 downto 0);
    signal ddr_arw_burst      : std_logic_vector(1 downto 0);
    signal ddr_w_valid, ddr_w_ready, ddr_w_last : std_logic;
    signal ddr_w_id           : std_logic_vector(7 downto 0);
    signal ddr_w_data         : std_logic_vector(63 downto 0);
    signal ddr_w_strb         : std_logic_vector(7 downto 0);
    signal ddr_b_valid, ddr_b_ready : std_logic;
    signal ddr_b_id           : std_logic_vector(7 downto 0);
    signal ddr_b_resp         : std_logic_vector(1 downto 0);
    signal ddr_r_valid, ddr_r_ready, ddr_r_last : std_logic;
    signal ddr_r_data         : std_logic_vector(63 downto 0);
    signal ddr_r_id           : std_logic_vector(7 downto 0);
    signal ddr_r_resp         : std_logic_vector(1 downto 0);

    -- the TSEMAC's DMA on DDR master port 0
    signal eth_arvalid, eth_arready, eth_rvalid, eth_rready, eth_rlast : std_logic;
    signal eth_araddr         : std_logic_vector(31 downto 0);
    signal eth_arregion, eth_arcache, eth_arqos : std_logic_vector(3 downto 0);
    signal eth_arlen          : std_logic_vector(7 downto 0);
    signal eth_arsize, eth_arprot : std_logic_vector(2 downto 0);
    signal eth_arburst, eth_rresp : std_logic_vector(1 downto 0);
    signal eth_arlock         : std_logic_vector(0 downto 0);
    signal eth_rdata          : std_logic_vector(63 downto 0);
    signal eth_awvalid, eth_awready, eth_wvalid, eth_wready, eth_wlast : std_logic;
    signal eth_bvalid, eth_bready : std_logic;
    signal eth_awaddr         : std_logic_vector(31 downto 0);
    signal eth_awregion, eth_awcache, eth_awqos : std_logic_vector(3 downto 0);
    signal eth_awlen          : std_logic_vector(7 downto 0);
    signal eth_awsize, eth_awprot : std_logic_vector(2 downto 0);
    signal eth_awburst, eth_bresp : std_logic_vector(1 downto 0);
    signal eth_awlock         : std_logic_vector(0 downto 0);
    signal eth_wdata          : std_logic_vector(63 downto 0);
    signal eth_wstrb          : std_logic_vector(7 downto 0);

    -- the scope DMA on DDR master port 1, write only
    signal scope_awvalid, scope_awready, scope_awlock : std_logic;
    signal scope_wvalid, scope_wready, scope_wlast, scope_bvalid, scope_bready : std_logic;
    signal scope_awaddr       : std_logic_vector(31 downto 0);
    signal scope_awregion, scope_awcache, scope_awqos : std_logic_vector(3 downto 0);
    signal scope_awlen        : std_logic_vector(7 downto 0);
    signal scope_awsize, scope_awprot : std_logic_vector(2 downto 0);
    signal scope_awburst, scope_bresp : std_logic_vector(1 downto 0);
    signal scope_wdata        : std_logic_vector(63 downto 0);
    signal scope_wstrb        : std_logic_vector(7 downto 0);

    -- APB slaves 0 (MAC and control registers), 1 (its DMA), 2 (scope DMA)
    type apb_record is record
        psel, penable, pwrite, pready, pslverror : std_logic;
        paddr  : std_logic_vector(15 downto 0);
        pwdata : std_logic_vector(31 downto 0);
        prdata : std_logic_vector(31 downto 0);
    end record;
    signal eth_apb, dma_apb, scope_dma_apb : apb_record;

    signal dma_interrupts : std_logic_vector(1 downto 0);
    signal ram_reset      : std_logic;
    signal clocks_ready   : std_logic;
    signal eth_apb_paddr32, dma_apb_paddr32 : std_logic_vector(31 downto 0);

begin

    io_systemClk     <= ram_clk;
    io_memoryClk     <= ram_clk;
    io_peripheralClk <= io_tseClk;

    -- held in reset until the clocks lock and the HyperRAM is calibrated, as top_ti60.v
    clocks_ready  <= pll_locked and to_std(calibration_done);
    io_asyncReset <= not clocks_ready;
    ram_reset     <= not pll_locked;

    eth_apb_paddr32 <= x"0000" & eth_apb.paddr;
    dma_apb_paddr32 <= x"0000" & dma_apb.paddr;

    memory_ready            <= to_std(calibration_done);
    ethernet_dma_interrupts <= dma_interrupts;

------------------------------------------------------------------------
    u_soc : SapphireCore
    port map(
        io_systemClk            => io_systemClk
        ,io_memoryClk           => io_memoryClk
        ,io_peripheralClk       => io_peripheralClk
        ,io_asyncReset          => io_asyncReset
        ,io_systemReset         => io_systemReset
        ,io_memoryReset         => io_memoryReset
        ,io_peripheralReset     => io_peripheralReset

        ,system_uart_0_io_txd   => system_uart_0_io_txd
        ,system_uart_0_io_rxd   => system_uart_0_io_rxd
        ,system_spi_0_io_sclk_write         => system_spi_0_io_sclk_write
        ,system_spi_0_io_ss(0)              => system_spi_0_io_ss
        ,system_spi_0_io_data_0_writeEnable => system_spi_0_io_data_0_writeEnable
        ,system_spi_0_io_data_0_read        => system_spi_0_io_data_0_read
        ,system_spi_0_io_data_0_write       => system_spi_0_io_data_0_write
        ,system_spi_0_io_data_1_writeEnable => system_spi_0_io_data_1_writeEnable
        ,system_spi_0_io_data_1_read        => system_spi_0_io_data_1_read
        ,system_spi_0_io_data_1_write       => system_spi_0_io_data_1_write
        ,system_spi_0_io_data_2_writeEnable => open
        ,system_spi_0_io_data_2_read        => '0'
        ,system_spi_0_io_data_2_write       => open
        ,system_spi_0_io_data_3_writeEnable => open
        ,system_spi_0_io_data_3_read        => '0'
        ,system_spi_0_io_data_3_write       => open

        ,jtagCtrl_tck            => jtag_inst1_TCK
        ,jtagCtrl_tdi            => jtag_inst1_TDI
        ,jtagCtrl_tdo            => jtag_inst1_TDO
        ,jtagCtrl_enable         => jtag_inst1_SEL
        ,jtagCtrl_capture        => jtag_inst1_CAPTURE
        ,jtagCtrl_shift          => jtag_inst1_SHIFT
        ,jtagCtrl_update         => jtag_inst1_UPDATE
        ,jtagCtrl_reset          => jtag_inst1_RESET

        ,userInterruptA          => dma_interrupts(0)
        ,userInterruptB          => dma_interrupts(1)

        ,io_apbSlave_0_PADDR     => eth_apb.paddr
        ,io_apbSlave_0_PSEL      => eth_apb.psel
        ,io_apbSlave_0_PENABLE   => eth_apb.penable
        ,io_apbSlave_0_PREADY    => eth_apb.pready
        ,io_apbSlave_0_PWRITE    => eth_apb.pwrite
        ,io_apbSlave_0_PWDATA    => eth_apb.pwdata
        ,io_apbSlave_0_PRDATA    => eth_apb.prdata
        ,io_apbSlave_0_PSLVERROR => eth_apb.pslverror
        ,io_apbSlave_1_PADDR     => dma_apb.paddr
        ,io_apbSlave_1_PSEL      => dma_apb.psel
        ,io_apbSlave_1_PENABLE   => dma_apb.penable
        ,io_apbSlave_1_PREADY    => dma_apb.pready
        ,io_apbSlave_1_PWRITE    => dma_apb.pwrite
        ,io_apbSlave_1_PWDATA    => dma_apb.pwdata
        ,io_apbSlave_1_PRDATA    => dma_apb.prdata
        ,io_apbSlave_1_PSLVERROR => dma_apb.pslverror
        ,io_apbSlave_2_PADDR     => scope_dma_apb.paddr
        ,io_apbSlave_2_PSEL      => scope_dma_apb.psel
        ,io_apbSlave_2_PENABLE   => scope_dma_apb.penable
        ,io_apbSlave_2_PREADY    => scope_dma_apb.pready
        ,io_apbSlave_2_PWRITE    => scope_dma_apb.pwrite
        ,io_apbSlave_2_PWDATA    => scope_dma_apb.pwdata
        ,io_apbSlave_2_PRDATA    => scope_dma_apb.prdata
        ,io_apbSlave_2_PSLVERROR => scope_dma_apb.pslverror
        ,io_apbSlave_3_PADDR     => bus_apb_paddr
        ,io_apbSlave_3_PSEL      => bus_apb_psel
        ,io_apbSlave_3_PENABLE   => bus_apb_penable
        ,io_apbSlave_3_PREADY    => bus_apb_pready
        ,io_apbSlave_3_PWRITE    => bus_apb_pwrite
        ,io_apbSlave_3_PWDATA    => bus_apb_pwdata
        ,io_apbSlave_3_PRDATA    => bus_apb_prdata
        ,io_apbSlave_3_PSLVERROR => bus_apb_pslverror

        ,io_ddrA_arw_valid          => ddr_arw_valid
        ,io_ddrA_arw_ready          => ddr_arw_ready
        ,io_ddrA_arw_payload_addr   => ddr_arw_addr
        ,io_ddrA_arw_payload_id     => ddr_arw_id
        ,io_ddrA_arw_payload_region => open
        ,io_ddrA_arw_payload_len    => ddr_arw_len
        ,io_ddrA_arw_payload_size   => ddr_arw_size
        ,io_ddrA_arw_payload_burst  => ddr_arw_burst
        ,io_ddrA_arw_payload_lock   => ddr_arw_lock
        ,io_ddrA_arw_payload_cache  => open
        ,io_ddrA_arw_payload_qos    => open
        ,io_ddrA_arw_payload_prot   => open
        ,io_ddrA_arw_payload_write  => ddr_arw_write
        ,io_ddrA_w_valid            => ddr_w_valid
        ,io_ddrA_w_ready            => ddr_w_ready
        ,io_ddrA_w_payload_data     => ddr_w_data
        ,io_ddrA_w_payload_strb     => ddr_w_strb
        ,io_ddrA_w_payload_last     => ddr_w_last
        ,io_ddrA_w_payload_id       => ddr_w_id
        ,io_ddrA_b_valid            => ddr_b_valid
        ,io_ddrA_b_ready            => ddr_b_ready
        ,io_ddrA_b_payload_id       => ddr_b_id
        ,io_ddrA_b_payload_resp     => ddr_b_resp
        ,io_ddrA_r_valid            => ddr_r_valid
        ,io_ddrA_r_ready            => ddr_r_ready
        ,io_ddrA_r_payload_data     => ddr_r_data
        ,io_ddrA_r_payload_id       => ddr_r_id
        ,io_ddrA_r_payload_resp     => ddr_r_resp
        ,io_ddrA_r_payload_last     => ddr_r_last

        -- the TSEMAC's DMA, ids as top_ti60.v
        ,io_ddrMasters_0_clk               => io_memoryClk
        ,io_ddrMasters_0_reset             => io_ddrMaster0Reset
        ,io_ddrMasters_0_aw_valid          => eth_awvalid
        ,io_ddrMasters_0_aw_ready          => eth_awready
        ,io_ddrMasters_0_aw_payload_addr   => eth_awaddr
        ,io_ddrMasters_0_aw_payload_id     => x"8"
        ,io_ddrMasters_0_aw_payload_region => eth_awregion
        ,io_ddrMasters_0_aw_payload_len    => eth_awlen
        ,io_ddrMasters_0_aw_payload_size   => eth_awsize
        ,io_ddrMasters_0_aw_payload_burst  => eth_awburst
        ,io_ddrMasters_0_aw_payload_lock   => eth_awlock(0)
        ,io_ddrMasters_0_aw_payload_cache  => eth_awcache
        ,io_ddrMasters_0_aw_payload_qos    => eth_awqos
        ,io_ddrMasters_0_aw_payload_prot   => eth_awprot
        ,io_ddrMasters_0_w_valid           => eth_wvalid
        ,io_ddrMasters_0_w_ready           => eth_wready
        ,io_ddrMasters_0_w_payload_data    => eth_wdata
        ,io_ddrMasters_0_w_payload_strb    => eth_wstrb
        ,io_ddrMasters_0_w_payload_last    => eth_wlast
        ,io_ddrMasters_0_b_valid           => eth_bvalid
        ,io_ddrMasters_0_b_ready           => eth_bready
        ,io_ddrMasters_0_b_payload_id      => open
        ,io_ddrMasters_0_b_payload_resp    => eth_bresp
        ,io_ddrMasters_0_ar_valid          => eth_arvalid
        ,io_ddrMasters_0_ar_ready          => eth_arready
        ,io_ddrMasters_0_ar_payload_addr   => eth_araddr
        ,io_ddrMasters_0_ar_payload_id     => x"7"
        ,io_ddrMasters_0_ar_payload_region => eth_arregion
        ,io_ddrMasters_0_ar_payload_len    => eth_arlen
        ,io_ddrMasters_0_ar_payload_size   => eth_arsize
        ,io_ddrMasters_0_ar_payload_burst  => eth_arburst
        ,io_ddrMasters_0_ar_payload_lock   => eth_arlock(0)
        ,io_ddrMasters_0_ar_payload_cache  => eth_arcache
        ,io_ddrMasters_0_ar_payload_qos    => eth_arqos
        ,io_ddrMasters_0_ar_payload_prot   => eth_arprot
        ,io_ddrMasters_0_r_valid           => eth_rvalid
        ,io_ddrMasters_0_r_ready           => eth_rready
        ,io_ddrMasters_0_r_payload_data    => eth_rdata
        ,io_ddrMasters_0_r_payload_id      => open
        ,io_ddrMasters_0_r_payload_resp    => eth_rresp
        ,io_ddrMasters_0_r_payload_last    => eth_rlast

        -- the scope DMA, which only writes
        ,io_ddrMasters_1_clk               => io_memoryClk
        ,io_ddrMasters_1_reset             => io_ddrMaster1Reset
        ,io_ddrMasters_1_aw_valid          => scope_awvalid
        ,io_ddrMasters_1_aw_ready          => scope_awready
        ,io_ddrMasters_1_aw_payload_addr   => scope_awaddr
        ,io_ddrMasters_1_aw_payload_id     => x"9"
        ,io_ddrMasters_1_aw_payload_region => scope_awregion
        ,io_ddrMasters_1_aw_payload_len    => scope_awlen
        ,io_ddrMasters_1_aw_payload_size   => scope_awsize
        ,io_ddrMasters_1_aw_payload_burst  => scope_awburst
        ,io_ddrMasters_1_aw_payload_lock   => scope_awlock
        ,io_ddrMasters_1_aw_payload_cache  => scope_awcache
        ,io_ddrMasters_1_aw_payload_qos    => scope_awqos
        ,io_ddrMasters_1_aw_payload_prot   => scope_awprot
        ,io_ddrMasters_1_w_valid           => scope_wvalid
        ,io_ddrMasters_1_w_ready           => scope_wready
        ,io_ddrMasters_1_w_payload_data    => scope_wdata
        ,io_ddrMasters_1_w_payload_strb    => scope_wstrb
        ,io_ddrMasters_1_w_payload_last    => scope_wlast
        ,io_ddrMasters_1_b_valid           => scope_bvalid
        ,io_ddrMasters_1_b_ready           => scope_bready
        ,io_ddrMasters_1_b_payload_id      => open
        ,io_ddrMasters_1_b_payload_resp    => scope_bresp
        ,io_ddrMasters_1_ar_valid          => '0'
        ,io_ddrMasters_1_ar_ready          => open
        ,io_ddrMasters_1_ar_payload_addr   => (others => '0')
        ,io_ddrMasters_1_ar_payload_id     => (others => '0')
        ,io_ddrMasters_1_ar_payload_region => (others => '0')
        ,io_ddrMasters_1_ar_payload_len    => (others => '0')
        ,io_ddrMasters_1_ar_payload_size   => (others => '0')
        ,io_ddrMasters_1_ar_payload_burst  => (others => '0')
        ,io_ddrMasters_1_ar_payload_lock   => '0'
        ,io_ddrMasters_1_ar_payload_cache  => (others => '0')
        ,io_ddrMasters_1_ar_payload_qos    => (others => '0')
        ,io_ddrMasters_1_ar_payload_prot   => (others => '0')
        ,io_ddrMasters_1_r_valid           => open
        ,io_ddrMasters_1_r_ready           => '1'
        ,io_ddrMasters_1_r_payload_data    => open
        ,io_ddrMasters_1_r_payload_id      => open
        ,io_ddrMasters_1_r_payload_resp    => open
        ,io_ddrMasters_1_r_payload_last    => open
    );
------------------------------------------------------------------------
    -- our HyperRAM controller (../source/hyperram/hyperram_axi.vhd): the whole 64 MB for the
    -- CPU and the DMAs (Efinix's controller corrupted DMA transfers on the second die)
    u_hyperram : entity work.hyperram_axi
    port map(
        clock               => io_memoryClk
        ,reset              => ram_reset = '1'
        ,arw_valid          => ddr_arw_valid
        ,arw_ready          => ddr_arw_ready
        ,arw_addr           => ddr_arw_addr
        ,arw_id             => ddr_arw_id
        ,arw_len            => ddr_arw_len
        ,arw_size           => ddr_arw_size
        ,arw_burst          => ddr_arw_burst
        ,arw_write          => ddr_arw_write
        ,w_valid            => ddr_w_valid
        ,w_ready            => ddr_w_ready
        ,w_data             => ddr_w_data
        ,w_strb             => ddr_w_strb
        ,w_last             => ddr_w_last
        ,b_valid            => ddr_b_valid
        ,b_ready            => ddr_b_ready
        ,b_id               => ddr_b_id
        ,r_valid            => ddr_r_valid
        ,r_ready            => ddr_r_ready
        ,r_data             => ddr_r_data
        ,r_id               => ddr_r_id
        ,r_resp             => ddr_r_resp
        ,r_last             => ddr_r_last
        ,calibration_done   => calibration_done
        ,calibration_passed => open
        ,status             => hyperram_status
        ,read_timeouts      => read_timeouts
        ,debug              => hyperram_debug
        ,phase_shift        => phase_shift
        ,phase_shift_output => 3                    -- ram_clk_cal is the PLL's output 3
        ,fpga_out           => hyperram_out
        ,fpga_in            => hyperram_in          -- from the input DDIO registers on ram_clk_cal
    );
    ddr_b_resp <= "00";
    hyperram_timeouts <= std_logic_vector(read_timeouts);

    hbc_cal_SHIFT     <= phase_shift.shift;
    hbc_cal_SHIFT_SEL <= phase_shift.shift_select;
    hbc_cal_SHIFT_ENA <= phase_shift.shift_enable;

    hbc_ck_p_HI     <= hyperram_out.ck_hi;
    hbc_ck_p_LO     <= hyperram_out.ck_lo;
    hbc_cs_n        <= hyperram_out.cs_n;
    hbc_rst_n       <= hyperram_out.reset_n;
    hbc_dq_OUT_HI   <= hyperram_out.dq_hi;
    hbc_dq_OUT_LO   <= hyperram_out.dq_lo;
    hbc_dq_OE       <= (others => hyperram_out.dq_oe);
    hbc_rwds_OUT_HI <= (0 => hyperram_out.rwds_hi);
    hbc_rwds_OUT_LO <= (0 => hyperram_out.rwds_lo);
    hbc_rwds_OE     <= (0 => hyperram_out.rwds_oe);
    hyperram_in <= (dq_hi => hbc_dq_IN_HI, dq_lo => hbc_dq_IN_LO,
                    rwds_hi => hbc_rwds_IN_HI(0), rwds_lo => hbc_rwds_IN_LO(0));
------------------------------------------------------------------------
    u_ethernet : entity work.ethernet_subsystem
    generic map(g_axi_width => 64, g_own_mac => g_own_ethernet_mac)
    port map(
        io_systemReset         => io_systemReset
        ,io_ddrMasterReset     => io_ddrMaster0Reset
        ,io_peripheralClk      => io_peripheralClk
        ,io_memoryClk          => io_memoryClk
        ,io_tseClk             => io_tseClk
        ,pll_locked            => clocks_ready
        ,rgmii_txd_HI          => rgmii_txd_HI
        ,rgmii_txd_LO          => rgmii_txd_LO
        ,rgmii_tx_ctl_HI       => rgmii_tx_ctl_HI
        ,rgmii_tx_ctl_LO       => rgmii_tx_ctl_LO
        ,rgmii_txc_HI          => rgmii_txc_HI
        ,rgmii_txc_LO          => rgmii_txc_LO
        ,rgmii_rxd_HI          => rgmii_rxd_HI
        ,rgmii_rxd_LO          => rgmii_rxd_LO
        ,rgmii_rx_ctl_HI       => rgmii_rx_ctl_HI
        ,rgmii_rx_ctl_LO       => rgmii_rx_ctl_LO
        ,rgmii_rxc             => rgmii_rxc
        ,phy_mdi               => phy_mdi
        ,phy_mdo               => phy_mdo
        ,phy_mdo_en            => phy_mdo_en
        ,phy_mdc               => phy_mdc
        ,phy_sw_rst            => phy_reset
        ,dma_interrupts        => dma_interrupts
        ,eth_speed             => ethernet_speed
        ,eth_apb3_paddr        => eth_apb_paddr32
        ,eth_apb3_psel         => eth_apb.psel
        ,eth_apb3_penable      => eth_apb.penable
        ,eth_apb3_pwrite       => eth_apb.pwrite
        ,eth_apb3_pwdata       => eth_apb.pwdata
        ,eth_apb3_pready       => eth_apb.pready
        ,eth_apb3_prdata       => eth_apb.prdata
        ,eth_apb3_pslverror    => eth_apb.pslverror
        ,dma_apb3_paddr        => dma_apb_paddr32
        ,dma_apb3_psel         => dma_apb.psel
        ,dma_apb3_penable      => dma_apb.penable
        ,dma_apb3_pwrite       => dma_apb.pwrite
        ,dma_apb3_pwdata       => dma_apb.pwdata
        ,dma_apb3_pready       => dma_apb.pready
        ,dma_apb3_prdata       => dma_apb.prdata
        ,dma_apb3_pslverror    => dma_apb.pslverror
        ,dmaaxi_read_arvalid   => eth_arvalid
        ,dmaaxi_read_arready   => eth_arready
        ,dmaaxi_read_araddr    => eth_araddr
        ,dmaaxi_read_arregion  => eth_arregion
        ,dmaaxi_read_arlen     => eth_arlen
        ,dmaaxi_read_arsize    => eth_arsize
        ,dmaaxi_read_arburst   => eth_arburst
        ,dmaaxi_read_arlock    => eth_arlock
        ,dmaaxi_read_arcache   => eth_arcache
        ,dmaaxi_read_arqos     => eth_arqos
        ,dmaaxi_read_arprot    => eth_arprot
        ,dmaaxi_read_rvalid    => eth_rvalid
        ,dmaaxi_read_rready    => eth_rready
        ,dmaaxi_read_rdata     => eth_rdata
        ,dmaaxi_read_rresp     => eth_rresp
        ,dmaaxi_read_rlast     => eth_rlast
        ,dmaaxi_write_awvalid  => eth_awvalid
        ,dmaaxi_write_awready  => eth_awready
        ,dmaaxi_write_awaddr   => eth_awaddr
        ,dmaaxi_write_awregion => eth_awregion
        ,dmaaxi_write_awlen    => eth_awlen
        ,dmaaxi_write_awsize   => eth_awsize
        ,dmaaxi_write_awburst  => eth_awburst
        ,dmaaxi_write_awlock   => eth_awlock
        ,dmaaxi_write_awcache  => eth_awcache
        ,dmaaxi_write_awqos    => eth_awqos
        ,dmaaxi_write_awprot   => eth_awprot
        ,dmaaxi_write_wvalid   => eth_wvalid
        ,dmaaxi_write_wready   => eth_wready
        ,dmaaxi_write_wdata    => eth_wdata
        ,dmaaxi_write_wstrb    => eth_wstrb
        ,dmaaxi_write_wlast    => eth_wlast
        ,dmaaxi_write_bvalid   => eth_bvalid
        ,dmaaxi_write_bready   => eth_bready
        ,dmaaxi_write_bresp    => eth_bresp
    );
------------------------------------------------------------------------
    u_scope_dma : dma_scope
    port map(
        clk             => io_memoryClk
        ,reset          => io_ddrMaster1Reset
        ,ctrl_clk       => io_peripheralClk
        ,ctrl_reset     => io_systemReset
        ,ctrl_PADDR     => scope_dma_apb.paddr(13 downto 0)
        ,ctrl_PSEL      => scope_dma_apb.psel
        ,ctrl_PENABLE   => scope_dma_apb.penable
        ,ctrl_PREADY    => scope_dma_apb.pready
        ,ctrl_PWRITE    => scope_dma_apb.pwrite
        ,ctrl_PWDATA    => scope_dma_apb.pwdata
        ,ctrl_PRDATA    => scope_dma_apb.prdata
        ,ctrl_PSLVERROR => scope_dma_apb.pslverror
        ,ctrl_interrupts => open
        ,write_awvalid  => scope_awvalid
        ,write_awready  => scope_awready
        ,write_awaddr   => scope_awaddr
        ,write_awregion => scope_awregion
        ,write_awlen    => scope_awlen
        ,write_awsize   => scope_awsize
        ,write_awburst  => scope_awburst
        ,write_awlock   => scope_awlock
        ,write_awcache  => scope_awcache
        ,write_awqos    => scope_awqos
        ,write_awprot   => scope_awprot
        ,write_wvalid   => scope_wvalid
        ,write_wready   => scope_wready
        ,write_wdata    => scope_wdata
        ,write_wstrb    => scope_wstrb
        ,write_wlast    => scope_wlast
        ,write_bvalid   => scope_bvalid
        ,write_bready   => scope_bready
        ,write_bresp    => scope_bresp
        ,dat0_i_clk     => io_peripheralClk
        ,dat0_i_reset   => io_peripheralReset
        ,dat0_i_tvalid  => scope_tvalid
        ,dat0_i_tready  => scope_tready
        ,dat0_i_tdata   => scope_tdata
        ,dat0_i_tkeep   => scope_tkeep
        ,dat0_i_tdest   => x"0"
        ,dat0_i_tlast   => scope_tlast
    );
------------------------------------------------------------------------
end rtl;
