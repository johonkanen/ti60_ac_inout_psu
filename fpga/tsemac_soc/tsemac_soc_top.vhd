library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.image_role_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.fixed_format_pkg.all;
    use work.scaler_channels_pkg.all;

-- The ti60_ac_psu control board's FPGA design, for Zephyr on the Sapphire SoC:
--   sapphire_subsystem  the Efinix IP cores: the Sapphire SoC with the HyperRAM as its main
--                       memory, the TSEMAC and its DMA on the board's PEF7071, the console
--                       on the board's UART, the configuration flash on SPI0, and a DMA for
--                       the oscilloscope's captures
--   apb_bus_bridge      the SoC's APB slave 3 as a master of board_test's register bus:
--                       register r at byte offset 4*r of the APB window
--   scope_subsystem     board_test's oscilloscope, registers at 600.., its deep capture
--                       streamed into a circular buffer in the HyperRAM
--   remote_update       the FPGA's internal reconfiguration: golden image at flash 0, the
--                       application image the software replaces at 1 MB, registers at 800..
--   ad_subsystem        the control board's two ADS7056 behind their 8 input muxes,
--                       registers at 700.. (sec, board_test's) and 720.. (pri), their
--                       results probes of the scope; and the power stage's three
--                       sigma-delta current modulators, registers at 740.., filtered
--                       results probes too
--   gate_power          the power stage's six gate driver supplies, PWMs the software turns
--                       on, registers at 760..
--   measurement_scaler  the ADC and sigma-delta readings to volts and amperes (fixed point)
--   dhb_test, dhb_voltage_controller, dhb_protection
--                       the DHB (the isolated DC/DC stage): its precharge and run,
--                       registers at 780..; the secondary DC link's voltage control (1100..);
--                       overcurrent and overvoltage trips
--   output_bridge_test  the output full bridge at 40 kHz: open loop steps, or closed loop on
--                       the control's references (1140..)
--   waveform_player     the output voltage's waveform: 512 points of (voltage, periods), the
--                       closed loop's reference in the output test's waveform mode (1200..)
--   output_control_processor
--                       the output filter's observer (1160..) and the output voltage's
--                       cascaded control (1180..) on a microprogram processor
--   input_bridge_test   the input bridge's polarity test: one switch at a time in bursts
--                       (1240..)
-- The relays are held low.
-- The transmit stream fills in IPv4, TCP, UDP and ICMP checksums on its way to the MAC
-- (ethernet/ethernet_subsystem.vhd, ethernet/tx_checksum_inserter.vhd).
-- Gigabit only: the PHY is set to advertise 1000BASE-T alone, so there is no 10/100 receive
-- clock mux.
--
-- Clocks (gen_peri.py): ram_clk 200 MHz, also the SoC's system and memory clock; io_tseClk
-- 125 MHz, also the SoC's peripheral clock and the scope's; rgmii_rxc from the receive PLL.
entity tsemac_soc_top is
    port (
        pll_locked   : in std_logic
        ;ram_clk     : in std_logic
        ;ram_clk_cal : in std_logic
        ;io_tseClk   : in std_logic
        ;hbc_cal_SHIFT_SEL : out std_logic_vector(4 downto 0)
        ;hbc_cal_SHIFT     : out std_logic_vector(2 downto 0)
        ;hbc_cal_SHIFT_ENA : out std_logic

        -- RGMII and the receive PLL
        ;rgmii_rxc       : in std_logic
        ;rx_pll_locked   : in std_logic
        ;rx_pll_rstn     : out std_logic
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

        -- HyperRAM, x8
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

        -- the two ADS7056 and their input muxes: sec (board_test's) and pri
        ;adc_sclk   : out std_logic
        ;adc_cs_n   : out std_logic
        ;adc_sdo    : in std_logic
        ;adc_mux0, adc_mux1, adc_mux2 : out std_logic
        ;adc_pri_sclk : out std_logic
        ;adc_pri_cs_n : out std_logic
        ;adc_pri_sdo  : in std_logic
        ;adc_pri_mux0, adc_pri_mux1, adc_pri_mux2 : out std_logic

        -- the power stage, held off
        ;dab_primary_hi, dab_primary_low, dab_secondary_hi, dab_secondary_low : out std_logic
        ;output_inu_leg1_hi, output_inu_leg1_low, output_inu_leg2_hi, output_inu_leg2_low : out std_logic
        ;grid_inu_leg1_hi, grid_inu_leg1_low, grid_inu_leg2_hi, grid_inu_leg2_low : out std_logic
        ;primary_bypass_relay, secondary_bypass_relay : out std_logic
        ;gate_power1_pwm, gate_power2_pwm, gate_power3_pwm : out std_logic
        ;gate_power4_pwm, gate_power5_pwm, gate_power6_pwm : out std_logic
        -- the power stage's isolated sigma-delta current modulators: a common clock, their data
        ;grid_inu_sdm_clock, output_inu_sdm_clock, dab_sdm_clock : out std_logic
        ;grid_inu_sdm_data, output_inu_sdm_data, dab_sdm_data : in std_logic

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

        -- internal reconfiguration (remote update), clocked by the internal oscillator
        ;cfg_clk    : in std_logic
        ;cfg_CBSEL  : out std_logic_vector(1 downto 0)
        ;cfg_ENA    : out std_logic
        ;cfg_CONFIG : out std_logic
        ;cfg_ERROR  : in std_logic
    );
end entity tsemac_soc_top;

architecture rtl of tsemac_soc_top is

    signal bus_psel, bus_penable, bus_pwrite, bus_pready, bus_pslverror : std_logic;
    signal bus_paddr   : std_logic_vector(15 downto 0);
    signal bus_pwdata  : std_logic_vector(31 downto 0);
    signal bus_prdata  : std_logic_vector(31 downto 0);
    signal scope_tvalid, scope_tready, scope_tlast : std_logic;
    signal scope_tdata   : std_logic_vector(63 downto 0);
    signal scope_tkeep   : std_logic_vector(7 downto 0);
    signal scope_status  : std_logic_vector(15 downto 0);

    -- board_test's register bus, mastered by the SoC
    signal bus_to_slaves   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_slaves : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_scope  : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_adc    : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_update : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_gate_power : fpga_interconnect_record := init_fpga_interconnect;
    signal gate_power_pwm  : std_logic_vector(1 to 6);
    signal bus_from_dhb    : fpga_interconnect_record := init_fpga_interconnect;
    signal dhb_probe       : scope_sample;
    signal dhb_primary_gates   : dhb_leg_gates;
    signal dhb_secondary_gates : dhb_leg_gates;

    signal sec_latest, pri_latest       : scope_sample;
    signal sec_positions, pri_positions : scope_sample_array(0 to 7);
    signal measurements       : fixed_array(0 to 25);
    signal measurements_ready : std_logic_vector(0 to 25);
    signal scaled             : fixed_array(0 to scaler_channels - 1);
    signal scaler_raw         : fixed_array(0 to scaler_channels - 1);
    signal scaler_ready       : std_logic_vector(0 to scaler_channels - 1);
    signal scaled_ready       : boolean;
    signal scaled_channel     : natural range 0 to scaler_channels - 1;
    signal bus_from_scaler    : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_dhb_control : fpga_interconnect_record := init_fpga_interconnect;
    signal dhb_phase_command    : dhb_phase;
    signal dhb_secondary_filtered : fixed_number;
    signal bus_from_protection  : fpga_interconnect_record := init_fpga_interconnect;
    signal dhb_trip             : boolean;
    signal dhb_running          : boolean;
    signal bus_from_output      : fpga_interconnect_record := init_fpga_interconnect;
    signal output_leg1, output_leg2 : dhb_leg_gates;
    signal output_probe         : scope_sample;
    signal bus_from_output_control : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_waveform    : fpga_interconnect_record := init_fpga_interconnect;
    signal output_active        : boolean;
    signal bus_from_input       : fpga_interconnect_record := init_fpga_interconnect;
    signal input_leg1, input_leg2 : dhb_leg_gates;
    signal input_probe          : scope_sample;
    signal primary_dc_voltage   : fixed_number := fixed_zero;
    signal waveform_ref         : fixed_number;
    signal waveform_finished, waveform_start, waveform_step : boolean;
    signal output_running       : boolean;
    signal output_reference     : fixed_number;
    signal observer_probes      : scope_sample_array(0 to 3);
    signal closed_loop, output_period_start, control_ready : boolean;
    signal square_reference, control_u : fixed_number;
    signal control_probes       : scope_sample_array(0 to 1);
    signal adc_mux, adc_pri_mux         : std_logic_vector(2 downto 0);
    signal sdm_results                  : scope_sample_array(0 to 2);
    signal sdm_means                    : scope_sample_array(0 to 2);
    signal sdm_sampled                  : scope_sample_array(0 to 2);
    signal output_sample                : boolean;
    signal sdm_raw_bits                 : scope_sample;
    signal sdm_midpoint                 : scope_sample;
    signal sdm_clock                    : std_logic;
    signal sdm_data                     : std_logic_vector(0 to 2);

    signal memory_ready     : std_logic;
    signal ethernet_speed   : std_logic_vector(2 downto 0);
    signal ethernet_dma_interrupts : std_logic_vector(1 downto 0);
    signal phy_reset        : std_logic;

    -- the receive PLL restart, in io_tseClk
    constant relock_timeout : natural := 125_000;   -- 1 ms
    constant relock_length  : natural := 1_000;
    signal rx_locked_sync   : std_logic_vector(1 downto 0) := (others => '0');
    signal unlocked_time    : natural range 0 to relock_timeout := 0;
    signal relock_reset     : natural range 0 to relock_length := 0;

begin

------------------------------------------------------------------------
    u_sapphire : entity work.sapphire_subsystem
    port map(
        pll_locked         => pll_locked
        ,ram_clk           => ram_clk
        ,ram_clk_cal       => ram_clk_cal
        ,io_tseClk         => io_tseClk
        ,hbc_cal_SHIFT_SEL => hbc_cal_SHIFT_SEL
        ,hbc_cal_SHIFT     => hbc_cal_SHIFT
        ,hbc_cal_SHIFT_ENA => hbc_cal_SHIFT_ENA

        ,hbc_dq_OE         => hbc_dq_OE
        ,hbc_dq_IN_LO      => hbc_dq_IN_LO
        ,hbc_dq_IN_HI      => hbc_dq_IN_HI
        ,hbc_dq_OUT_LO     => hbc_dq_OUT_LO
        ,hbc_dq_OUT_HI     => hbc_dq_OUT_HI
        ,hbc_rwds_OE       => hbc_rwds_OE
        ,hbc_rwds_IN_LO    => hbc_rwds_IN_LO
        ,hbc_rwds_IN_HI    => hbc_rwds_IN_HI
        ,hbc_rwds_OUT_LO   => hbc_rwds_OUT_LO
        ,hbc_rwds_OUT_HI   => hbc_rwds_OUT_HI
        ,hbc_ck_p_LO       => hbc_ck_p_LO
        ,hbc_ck_p_HI       => hbc_ck_p_HI
        ,hbc_cs_n          => hbc_cs_n
        ,hbc_rst_n         => hbc_rst_n

        ,rgmii_rxc         => rgmii_rxc
        ,rgmii_txd_HI      => rgmii_txd_HI
        ,rgmii_txd_LO      => rgmii_txd_LO
        ,rgmii_tx_ctl_HI   => rgmii_tx_ctl_HI
        ,rgmii_tx_ctl_LO   => rgmii_tx_ctl_LO
        ,rgmii_txc_HI      => rgmii_txc_HI
        ,rgmii_txc_LO      => rgmii_txc_LO
        ,rgmii_rxd_HI      => rgmii_rxd_HI
        ,rgmii_rxd_LO      => rgmii_rxd_LO
        ,rgmii_rx_ctl_HI   => rgmii_rx_ctl_HI
        ,rgmii_rx_ctl_LO   => rgmii_rx_ctl_LO
        ,phy_mdi           => phy_mdi
        ,phy_mdo           => phy_mdo
        ,phy_mdo_en        => phy_mdo_en
        ,phy_mdc           => phy_mdc

        ,system_uart_0_io_txd               => system_uart_0_io_txd
        ,system_uart_0_io_rxd               => system_uart_0_io_rxd
        ,system_spi_0_io_sclk_write         => system_spi_0_io_sclk_write
        ,system_spi_0_io_ss                 => system_spi_0_io_ss
        ,system_spi_0_io_data_0_writeEnable => system_spi_0_io_data_0_writeEnable
        ,system_spi_0_io_data_0_read        => system_spi_0_io_data_0_read
        ,system_spi_0_io_data_0_write       => system_spi_0_io_data_0_write
        ,system_spi_0_io_data_1_writeEnable => system_spi_0_io_data_1_writeEnable
        ,system_spi_0_io_data_1_read        => system_spi_0_io_data_1_read
        ,system_spi_0_io_data_1_write       => system_spi_0_io_data_1_write

        ,jtag_inst1_TCK     => jtag_inst1_TCK
        ,jtag_inst1_TDI     => jtag_inst1_TDI
        ,jtag_inst1_TDO     => jtag_inst1_TDO
        ,jtag_inst1_SEL     => jtag_inst1_SEL
        ,jtag_inst1_CAPTURE => jtag_inst1_CAPTURE
        ,jtag_inst1_SHIFT   => jtag_inst1_SHIFT
        ,jtag_inst1_UPDATE  => jtag_inst1_UPDATE
        ,jtag_inst1_RESET   => jtag_inst1_RESET

        ,bus_apb_psel      => bus_psel
        ,bus_apb_penable   => bus_penable
        ,bus_apb_pwrite    => bus_pwrite
        ,bus_apb_paddr     => bus_paddr
        ,bus_apb_pwdata    => bus_pwdata
        ,bus_apb_prdata    => bus_prdata
        ,bus_apb_pready    => bus_pready
        ,bus_apb_pslverror => bus_pslverror
        ,scope_tvalid        => scope_tvalid
        ,scope_tready        => scope_tready
        ,scope_tdata         => scope_tdata
        ,scope_tkeep         => scope_tkeep
        ,scope_tlast         => scope_tlast

        ,memory_ready            => memory_ready
        ,ethernet_speed          => ethernet_speed
        ,ethernet_dma_interrupts => ethernet_dma_interrupts
        ,phy_reset               => phy_reset
    );
------------------------------------------------------------------------
    u_register_bus : entity work.apb_bus_bridge
    port map(
        clock      => io_tseClk
        ,psel      => bus_psel
        ,penable   => bus_penable
        ,pwrite    => bus_pwrite
        ,paddr     => bus_paddr
        ,pwdata    => bus_pwdata
        ,prdata    => bus_prdata
        ,pready    => bus_pready
        ,pslverror => bus_pslverror
        ,bus_out   => bus_to_slaves
        ,bus_in    => bus_from_slaves
    );
    bus_from_slaves <= bus_from_scope and bus_from_adc and bus_from_update and bus_from_gate_power
                       and bus_from_dhb and bus_from_scaler and bus_from_dhb_control
                       and bus_from_protection and bus_from_output and bus_from_output_control
                       and bus_from_waveform and bus_from_input;
------------------------------------------------------------------------
    -- the golden or application image (image_role_pkg, written by build.sh), and switching
    -- between them, registers at 800..
    u_remote_update : entity work.remote_update
    generic map(g_base_address => 800, g_role => image_role)
    port map(
        clock       => io_tseClk
        ,bus_in     => bus_to_slaves
        ,bus_out    => bus_from_update
        ,cfg_CBSEL  => cfg_CBSEL
        ,cfg_ENA    => cfg_ENA
        ,cfg_CONFIG => cfg_CONFIG
        ,cfg_ERROR  => cfg_ERROR
    );
------------------------------------------------------------------------
    -- digital probe of the scope, all in io_tseClk
    scope_status <= "000000000" & ethernet_dma_interrupts & ethernet_speed & memory_ready
                    & rx_locked_sync(1);

    u_scope : entity work.scope_subsystem
    port map(
        clock          => io_tseClk
        ,bus_in        => bus_to_slaves
        ,bus_out       => bus_from_scope
        ,tvalid        => scope_tvalid
        ,tready        => scope_tready
        ,tdata         => scope_tdata
        ,tkeep         => scope_tkeep
        ,tlast         => scope_tlast
        ,status        => scope_status
        ,sec_positions => sec_positions
        ,pri_positions => pri_positions
        ,sdm_results   => sdm_results
        ,sdm_means     => sdm_means
        ,sdm_sampled   => sdm_sampled
        ,observer_probes => observer_probes
        ,control_probes  => control_probes
        ,sdm_midpoint    => sdm_midpoint
        ,dhb_probe     => dhb_probe
        ,output_probe  => output_probe
        ,input_probe   => input_probe
    );
------------------------------------------------------------------------
    u_adc : entity work.ad_subsystem
    port map(
        clock      => io_tseClk
        ,adc_clock => ram_clk      -- 200 MHz: 50 MHz SCLK, 2.5 MS/s
        ,bus_in    => bus_to_slaves
        ,bus_out   => bus_from_adc
        ,sec_latest    => sec_latest
        ,sec_positions => sec_positions
        ,pri_latest    => pri_latest
        ,pri_positions => pri_positions
        ,sec_sclk  => adc_sclk
        ,sec_cs_n  => adc_cs_n
        ,sec_sdo   => adc_sdo
        ,sec_mux   => adc_mux
        ,pri_sclk  => adc_pri_sclk
        ,pri_cs_n  => adc_pri_cs_n
        ,pri_sdo   => adc_pri_sdo
        ,pri_mux   => adc_pri_mux
        ,sdm_results => sdm_results
        ,sdm_means   => sdm_means
        ,sdm_sample  => output_sample
        ,sdm_marker  => output_leg1.high
        ,sdm_raw_bits => sdm_raw_bits
        ,sdm_period_start => output_period_start
        ,sdm_midpoint     => sdm_midpoint
        ,sdm_sampled => sdm_sampled
        ,measurements       => measurements
        ,measurements_ready => measurements_ready
        ,sdm_clock => sdm_clock
        ,sdm_data  => sdm_data
    );
    sdm_data <= (grid_inu_sdm_data, output_inu_sdm_data, dab_sdm_data);
    grid_inu_sdm_clock   <= sdm_clock;
    output_inu_sdm_clock <= sdm_clock;
    dab_sdm_clock        <= sdm_clock;
    (adc_mux2, adc_mux1, adc_mux0) <= adc_mux;
    (adc_pri_mux2, adc_pri_mux1, adc_pri_mux0) <= adc_pri_mux;
------------------------------------------------------------------------
    -- every measurement scaled to fixed point (32 bit, 20 fractional bits) as it arrives, one
    -- fused multiply-add each, slopes and offsets written by the software (registers 900..)
    -- only the measurements the controls and protections use (scaler_channels_pkg)
    scaler_inputs : for c in 0 to scaler_channels - 1 generate
        scaler_raw(c)   <= measurements(measurement_of(c));
        scaler_ready(c) <= measurements_ready(measurement_of(c));
    end generate;

    u_scaler : entity work.measurement_scaler
    generic map(g_base_address => 900, g_channels => scaler_channels)
    port map(
        clock           => io_tseClk
        ,bus_in         => bus_to_slaves
        ,bus_out        => bus_from_scaler
        ,raw            => scaler_raw
        ,ready          => scaler_ready
        ,scaled         => scaled
        ,scaled_ready   => scaled_ready
        ,scaled_channel => scaled_channel
    );

    -- the DHB's voltage control: the secondary DC link to the primary's, a PI on the scaled
    -- voltages giving the phase shift (registers 1100..) for the bridge when it runs
    u_dhb_control : entity work.dhb_voltage_controller
    generic map(g_base_address => 1100, g_primary_channel => scaled_vdc_in, g_secondary_channel => scaled_vdc_out)
    port map(
        clock           => io_tseClk
        ,bus_in         => bus_to_slaves
        ,bus_out        => bus_from_dhb_control
        ,scaled         => scaled
        ,scaled_ready   => scaled_ready
        ,scaled_channel => scaled_channel
        ,run            => dhb_running
        ,phase          => dhb_phase_command
        ,secondary_voltage => dhb_secondary_filtered
    );

    -- the DHB's trips: overcurrent and overvoltage (registers 1120..), latched until cleared
    u_dhb_protection : entity work.dhb_protection
    generic map(g_base_address => 1120, g_current_channel => scaled_i_dab, g_output_current_channel => scaled_iac_out,
                g_primary_channel => scaled_vdc_in, g_secondary_channel => scaled_vdc_out)
    port map(
        clock           => io_tseClk
        ,bus_in         => bus_to_slaves
        ,bus_out        => bus_from_protection
        ,scaled         => scaled
        ,scaled_ready   => scaled_ready
        ,scaled_channel => scaled_channel
        ,trip           => dhb_trip
    );
------------------------------------------------------------------------
    -- the gate driver supplies, which also power the sigma-delta modulators' analog side:
    -- PWMs the software turns on (gate_power.vhd, registers at 760..), off at power up
    u_gate_power : entity work.gate_power
    generic map(g_base_address => 760)
    port map(
        clock    => io_tseClk
        ,bus_in  => bus_to_slaves
        ,bus_out => bus_from_gate_power
        ,pwm     => gate_power_pwm
    );
    (gate_power1_pwm, gate_power2_pwm, gate_power3_pwm,
     gate_power4_pwm, gate_power5_pwm, gate_power6_pwm) <= gate_power_pwm;

    -- the power stage is not controlled from this design: every gate drive low and the
    -- bypass relays open
    -- the DHB's test (dhb_test.vhd, registers at 780..): its precharge, the primary leg alone,
    -- and with a run time the bridge running after it on the voltage controller's phase, both
    -- legs, stopping itself
    u_dhb_test : entity work.dhb_test
    generic map(g_base_address => 780)
    port map(
        clock          => io_tseClk
        ,bus_in        => bus_to_slaves
        ,bus_out       => bus_from_dhb
        ,probe         => dhb_probe
        ,phase         => dhb_phase_command
        ,trip          => dhb_trip
        ,bridge_running => dhb_running
        ,hold          => output_active   -- on while the output bridge runs
        ,primary_voltage   => pri_positions(1)   -- vdc_in
        ,secondary_voltage => sec_positions(0)   -- vdc_out, the same scaling
        ,dhb_clock     => ram_clk      -- 200 MHz: 150 kHz, 100 ns dead time
        ,primary_gates   => dhb_primary_gates
        ,secondary_gates => dhb_secondary_gates
    );
    -- the output bridge's open loop step test (output_bridge_test.vhd, registers at 1140..):
    -- complementary modulation at 40 kHz, only while the DHB runs, off on a trip
    u_output_test : entity work.output_bridge_test
    generic map(g_base_address => 1140)
    port map(
        clock    => io_tseClk
        ,bus_in  => bus_to_slaves
        ,bus_out => bus_from_output
        ,allowed => dhb_running
        ,trip    => dhb_trip
        ,dc_voltage => dhb_secondary_filtered
        ,leg1    => output_leg1
        ,leg2    => output_leg2
        ,sample  => output_sample
        ,running_out       => output_running
        ,applied_reference => output_reference
        ,closed_loop_out   => closed_loop
        ,active_out        => output_active
        ,voltage_reference => square_reference
        ,period_start_out  => output_period_start
        ,control_u         => control_u
        ,control_ready     => control_ready
        ,waveform_ref      => waveform_ref
        ,waveform_finished => waveform_finished
        ,waveform_start    => waveform_start
        ,waveform_step     => waveform_step
        ,probe   => output_probe
    );

    -- the primary DC link's voltage, from the measurement scaler as it comes
    process(io_tseClk)
    begin
        if rising_edge(io_tseClk) then
            if scaled_ready and scaled_channel = scaled_vdc_in then
                primary_dc_voltage <= scaled(scaled_vdc_in);
            end if;
        end if;
    end process;

    -- the input bridge's polarity test (input_bridge_test.vhd, registers at 1240..): one switch
    -- at a time pulsing for a burst, off at the overvoltage stop or a trip
    u_input_test : entity work.input_bridge_test
    generic map(g_base_address => 1240)
    port map(
        clock       => io_tseClk
        ,bus_in     => bus_to_slaves
        ,bus_out    => bus_from_input
        ,trip       => dhb_trip
        ,dc_voltage => primary_dc_voltage
        ,leg1       => input_leg1
        ,leg2       => input_leg2
        ,probe      => input_probe
    );

    -- the output voltage's arbitrary waveform (registers 1200..): a table of 512 points, the
    -- closed loop's reference in the output test's waveform mode
    u_waveform : entity work.waveform_player
    generic map(g_base_address => 1200, g_points => 512)
    port map(
        clock     => io_tseClk
        ,bus_in   => bus_to_slaves
        ,bus_out  => bus_from_waveform
        ,start    => waveform_start
        ,step     => waveform_step
        ,running  => closed_loop
        ,v_ref    => waveform_ref
        ,finished => waveform_finished
    );

    -- the output filter's observer (registers 1160..) and the output voltage's control
    -- (1180..) on a microprogram processor: an observer step at each sampled output current;
    -- closed loop, a control step after the observer's that follows the period's start, its
    -- bridge reference to the output test's modulator
    u_output_control : entity work.output_control_processor
    generic map(g_observer_address => 1160, g_control_address => 1180,
                g_current_channel => scaled_iac_out_sampled, g_voltage_channel => scaled_vfilt_out)
    port map(
        clock           => io_tseClk
        ,bus_in         => bus_to_slaves
        ,bus_out        => bus_from_output_control
        ,scaled         => scaled
        ,scaled_ready   => scaled_ready
        ,scaled_channel => scaled_channel
        ,run            => output_running
        ,u              => output_reference
        ,closed_loop    => closed_loop
        ,v_ref          => square_reference
        ,period_start   => output_period_start
        ,u_ref          => control_u
        ,ready          => control_ready
        ,observer_probes => observer_probes
        ,control_probes  => control_probes
    );
    output_inu_leg1_hi     <= output_leg1.high;
    output_inu_leg1_low    <= output_leg1.low;
    output_inu_leg2_hi     <= output_leg2.high;
    output_inu_leg2_low    <= output_leg2.low;

    dab_primary_hi         <= dhb_primary_gates.high;
    dab_primary_low        <= dhb_primary_gates.low;
    dab_secondary_hi       <= dhb_secondary_gates.high;
    dab_secondary_low      <= dhb_secondary_gates.low;
    grid_inu_leg1_hi       <= input_leg1.high;
    grid_inu_leg1_low      <= input_leg1.low;
    grid_inu_leg2_hi       <= input_leg2.high;
    grid_inu_leg2_low      <= input_leg2.low;
    primary_bypass_relay   <= '0';
    secondary_bypass_relay <= '0';
------------------------------------------------------------------------
    -- Receive PLL: the PHY's RX_CLK stops while the link is down. The PLL is restarted when
    -- it has been out of lock for 1 ms, and while software holds the PHY reset bit of the
    -- TSEMAC control registers, which no pin uses on this board: the driver pulses it when
    -- the link comes up, as board_test restarts the PLL then.
    rx_pll_relock : process(io_tseClk)
    begin
        if rising_edge(io_tseClk) then
            rx_locked_sync <= rx_locked_sync(0) & rx_pll_locked;
            if relock_reset > 0 then
                relock_reset  <= relock_reset - 1;
                unlocked_time <= 0;
            elsif rx_locked_sync(1) = '1' then
                unlocked_time <= 0;
            elsif unlocked_time < relock_timeout then
                unlocked_time <= unlocked_time + 1;
            else
                relock_reset <= relock_length;
            end if;
        end if;
    end process;

    rx_pll_rstn <= '0' when phy_reset = '1' or relock_reset > 0 else '1';
------------------------------------------------------------------------
end rtl;
