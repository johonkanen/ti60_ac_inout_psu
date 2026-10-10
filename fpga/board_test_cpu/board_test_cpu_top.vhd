library ieee;
    use ieee.std_logic_1164.all;

-- board_test with a Sapphire RISC-V SoC beside it, for Zephyr (see build.sh).
--
-- board_test_top is instantiated unchanged and keeps every board pin but the UART. The
-- UART pins go to the SoC's UART0, Zephyr's console. board_test's register bus UART is
-- wired to the SoC's UART1 instead, so Zephyr reaches the register bus with the same
-- serial protocol a PC uses; the register bus also stays reachable over UDP port 5001.
--
-- The SoC runs from hyperram_clock, 200 MHz: its UART divides by 8 samples per bit, and
-- 200 MHz / 8 / 5 is board_test's 5 Mbaud exactly, where 125 MHz has no divider for it.
-- The UART lines cross between the two clock domains through the synchronisers at both
-- UART receivers.
entity board_test_cpu_top is
    port (
        main_clock  : in std_logic
        ;pll_locked : in std_logic

        ;uart_rx    : in std_logic
        ;uart_tx    : out std_logic
        ;adc_sclk   : out std_logic
        ;adc_cs_n   : out std_logic
        ;adc_sdo    : in std_logic
        ;adc_mux0, adc_mux1, adc_mux2 : out std_logic

        ;hyperram_clock          : in std_logic
        ;hyperram_read_clock     : in std_logic
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

        -- FPGA JTAG TAP USER1, the SoC's RISC-V debug module
        ;jtag_inst1_TCK     : in std_logic
        ;jtag_inst1_TDI     : in std_logic
        ;jtag_inst1_TDO     : out std_logic
        ;jtag_inst1_SEL     : in std_logic
        ;jtag_inst1_CAPTURE : in std_logic
        ;jtag_inst1_SHIFT   : in std_logic
        ;jtag_inst1_UPDATE  : in std_logic
        ;jtag_inst1_RESET   : in std_logic
    );
end entity board_test_cpu_top;

architecture rtl of board_test_cpu_top is

    component sapphire is
    port (
        io_systemClk : in std_logic;
        jtagCtrl_enable : in std_logic;
        jtagCtrl_tdi : in std_logic;
        jtagCtrl_capture : in std_logic;
        jtagCtrl_shift : in std_logic;
        jtagCtrl_update : in std_logic;
        jtagCtrl_reset : in std_logic;
        jtagCtrl_tdo : out std_logic;
        jtagCtrl_tck : in std_logic;
        io_asyncReset : in std_logic;
        io_systemReset : out std_logic;
        system_uart_0_io_txd : out std_logic;
        system_uart_0_io_rxd : in std_logic;
        system_uart_1_io_txd : out std_logic;
        system_uart_1_io_rxd : in std_logic
    );
    end component sapphire;

    -- the register bus UART between the SoC's UART1 and board_test
    signal bus_uart_to_board   : std_logic;
    signal bus_uart_from_board : std_logic;

    signal reset_meta   : std_logic_vector(4 downto 0) := (others => '1');
    signal system_reset : std_logic;

begin

------------------------------------------------------------------------
    u_board_test : entity work.board_test_top
    port map(
        main_clock  => main_clock
        ,pll_locked => pll_locked
        ,uart_rx    => bus_uart_to_board
        ,uart_tx    => bus_uart_from_board
        ,adc_sclk   => adc_sclk
        ,adc_cs_n   => adc_cs_n
        ,adc_sdo    => adc_sdo
        ,adc_mux0   => adc_mux0
        ,adc_mux1   => adc_mux1
        ,adc_mux2   => adc_mux2

        ,hyperram_clock           => hyperram_clock
        ,hyperram_read_clock      => hyperram_read_clock
        ,hyperram_phase_shift     => hyperram_phase_shift
        ,hyperram_phase_select    => hyperram_phase_select
        ,hyperram_phase_shift_ena => hyperram_phase_shift_ena
        ,hyperram_ck_hi           => hyperram_ck_hi
        ,hyperram_ck_lo           => hyperram_ck_lo
        ,hyperram_cs_n            => hyperram_cs_n
        ,hyperram_reset_n         => hyperram_reset_n
        ,hyperram_dq_in_hi        => hyperram_dq_in_hi
        ,hyperram_dq_in_lo        => hyperram_dq_in_lo
        ,hyperram_dq_out_hi       => hyperram_dq_out_hi
        ,hyperram_dq_out_lo       => hyperram_dq_out_lo
        ,hyperram_dq_oe           => hyperram_dq_oe
        ,hyperram_rwds_in_hi      => hyperram_rwds_in_hi
        ,hyperram_rwds_in_lo      => hyperram_rwds_in_lo
        ,hyperram_rwds_out_hi     => hyperram_rwds_out_hi
        ,hyperram_rwds_out_lo     => hyperram_rwds_out_lo
        ,hyperram_rwds_oe         => hyperram_rwds_oe

        ,ethernet_mdc      => ethernet_mdc
        ,ethernet_mdio_in  => ethernet_mdio_in
        ,ethernet_mdio_out => ethernet_mdio_out
        ,ethernet_mdio_oe  => ethernet_mdio_oe

        ,ethernet_txd0_hi   => ethernet_txd0_hi
        ,ethernet_txd0_lo   => ethernet_txd0_lo
        ,ethernet_txd1_hi   => ethernet_txd1_hi
        ,ethernet_txd1_lo   => ethernet_txd1_lo
        ,ethernet_txd2_hi   => ethernet_txd2_hi
        ,ethernet_txd2_lo   => ethernet_txd2_lo
        ,ethernet_txd3_hi   => ethernet_txd3_hi
        ,ethernet_txd3_lo   => ethernet_txd3_lo
        ,ethernet_tx_ctl_hi => ethernet_tx_ctl_hi
        ,ethernet_tx_ctl_lo => ethernet_tx_ctl_lo
        ,ethernet_tx_clk_hi => ethernet_tx_clk_hi
        ,ethernet_tx_clk_lo => ethernet_tx_clk_lo

        ,ethernet_rx_clock       => ethernet_rx_clock
        ,ethernet_rx_pll_locked  => ethernet_rx_pll_locked
        ,ethernet_rx_pll_reset_n => ethernet_rx_pll_reset_n
        ,ethernet_rxd0_hi   => ethernet_rxd0_hi
        ,ethernet_rxd0_lo   => ethernet_rxd0_lo
        ,ethernet_rxd1_hi   => ethernet_rxd1_hi
        ,ethernet_rxd1_lo   => ethernet_rxd1_lo
        ,ethernet_rxd2_hi   => ethernet_rxd2_hi
        ,ethernet_rxd2_lo   => ethernet_rxd2_lo
        ,ethernet_rxd3_hi   => ethernet_rxd3_hi
        ,ethernet_rxd3_lo   => ethernet_rxd3_lo
        ,ethernet_rx_ctl_hi => ethernet_rx_ctl_hi
        ,ethernet_rx_ctl_lo => ethernet_rx_ctl_lo
    );
------------------------------------------------------------------------
    -- reset while the PLL is not locked: asynchronous assert, synchronous release
    process(hyperram_clock, pll_locked) is
    begin
        if pll_locked = '0' then
            reset_meta <= (others => '1');
        elsif rising_edge(hyperram_clock) then
            reset_meta <= reset_meta(reset_meta'left-1 downto 0) & '0';
        end if;
    end process;

    u_sapphire : sapphire
    port map(
        io_systemClk          => hyperram_clock
        ,jtagCtrl_enable      => jtag_inst1_SEL
        ,jtagCtrl_tdi         => jtag_inst1_TDI
        ,jtagCtrl_capture     => jtag_inst1_CAPTURE
        ,jtagCtrl_shift       => jtag_inst1_SHIFT
        ,jtagCtrl_update      => jtag_inst1_UPDATE
        ,jtagCtrl_reset       => jtag_inst1_RESET
        ,jtagCtrl_tdo         => jtag_inst1_TDO
        ,jtagCtrl_tck         => jtag_inst1_TCK
        ,io_asyncReset        => reset_meta(reset_meta'left)
        ,io_systemReset       => system_reset
        ,system_uart_0_io_txd => uart_tx
        ,system_uart_0_io_rxd => uart_rx
        ,system_uart_1_io_txd => bus_uart_to_board
        ,system_uart_1_io_rxd => bus_uart_from_board
    );
------------------------------------------------------------------------
end rtl;
