#!/usr/bin/env python3
# Generates tsemac_soc.peri.xml with the Efinity Interface Designer API.
# Pins and PLLs are board_test's (see ../board_test/gen_peri.py); the HyperRAM and SPI flash
# pins are configured the way Efinix's TSEMAC example design configures its own, for the
# Efinix HyperRAM controller and the SoC's SPI master.
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
import os
import sys

sys.path.append(os.environ['EFXPT_HOME'] + "/bin")
from api_service.design import DesignAPI

project = "tsemac_soc"
here    = os.path.dirname(os.path.abspath(__file__))

design = DesignAPI(is_verbose=False)
design.create(project, "Ti60F225", here, overwrite=True)

for bank in ("1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"):
    design.set_iobank_voltage(bank, "1.8")
for bank in ("BL", "BR", "TL", "TR"):
    design.set_iobank_voltage(bank, "3.3")

design.set_gpio_unused_state("INPUT_WEAK_PULLUP")

# 50 MHz reference -> PLL_TR0 at 2000 MHz, as board_test:
#   0 ram_clk          200 MHz  HyperRAM, also the SoC's system and memory clock
#   1 io_tseClk        125 MHz  TSEMAC and RGMII transmit, also the SoC's peripheral clock
#   2 ram_clk90        200 MHz  90 degrees, the HyperRAM CK pin
#   3 ram_clk_cal      200 MHz  dynamic phase, HyperRAM read capture (calibrated by the IP)
# The PLL's output 4 reaches IO registers only, so there is no separate 100 MHz.
design.create_pll_input_clock_gpio("ref_clock")
design.assign_pkg_pin("ref_clock", "C14")

design.create_block("main_pll", "PLL")
design.assign_resource("main_pll", "PLL_TR0", "PLL")
design.set_property("main_pll", {
    "REFCLK_SOURCE"      : "EXTERNAL",
    "EXT_CLK"            : "EXT_CLK0",
    "REFCLK_FREQ"        : "50.0",
    "M"                  : "4",
    "N"                  : "1",
    "O"                  : "2",
    "FEEDBACK_MODE"      : "LOCAL",
    "FEEDBACK_CLK"       : "CLK0",
    "LOCKED_PIN"         : "pll_locked",
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_DIV"        : "10",
    "CLKOUT0_PIN"        : "ram_clk",
    "CLKOUT0_CONN_TYPE"  : "GCLK",
    "CLKOUT1_EN"         : "1",
    "CLKOUT1_DIV"        : "16",
    "CLKOUT1_PIN"        : "io_tseClk",
    "CLKOUT2_EN"         : "1",
    "CLKOUT2_DIV"        : "10",
    "CLKOUT2_PIN"        : "ram_clk90",
    "CLKOUT2_PHASE_SETTING" : "5",
    "CLKOUT3_EN"         : "1",
    "CLKOUT3_DIV"        : "10",
    "CLKOUT3_PIN"        : "ram_clk_cal",
    "CLKOUT3_DYNPHASE_EN" : "1",
    "PHASE_SHIFT_PIN"     : "hbc_cal_SHIFT",
    "PHASE_SHIFT_SEL_PIN" : "hbc_cal_SHIFT_SEL",
    "PHASE_SHIFT_ENA_PIN" : "hbc_cal_SHIFT_ENA",
}, block_type="PLL")

# console: the SoC's UART0 on the board's UART, 3.3 V bank
design.create_input_gpio("system_uart_0_io_rxd")
design.set_property("system_uart_0_io_rxd", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("system_uart_0_io_rxd", "R4")

design.create_output_gpio("system_uart_0_io_txd")
design.set_property("system_uart_0_io_txd", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("system_uart_0_io_txd", "R3")

# HyperRAM W959D8NFYA (x8), board_test's pins, the IO registers of the Efinix controller:
# outputs on ram_clk (data and its enable on the falling edge), CK from ram_clk90 inverted,
# inputs on the calibrated ram_clk_cal
def hbc_output(name, pin, register, clock="ram_clk", inverted=False, ddio=False):
    design.create_output_gpio(name)
    properties = {"IO_STANDARD": "1.8_V_LVCMOS"}
    if register:
        properties.update({"OUT_REG": register, "OUT_CLK_PIN": clock})
    if ddio:
        properties.update({"OUT_HI_PIN": f"{name}_HI", "OUT_LO_PIN": f"{name}_LO"})
    if inverted:
        properties["IS_OUTCLK_INVERTED"] = "1"
    design.set_property(name, properties)
    design.assign_resource(name, pin)

def hbc_inout(name, pin, bus, index, out_inverted, pull):
    design.create_inout_gpio(name)
    design.set_property(name, {"IO_STANDARD": "1.8_V_LVCMOS", "IN_REG": "DDIO_RESYNC",
                               "OUT_REG": "DDIO", "OE_REG": "REG", "PULL_OPTION": pull})
    design.set_property(name, {"IN_HI_PIN": f"{bus}_IN_HI[{index}]", "IN_LO_PIN": f"{bus}_IN_LO[{index}]",
                               "IN_CLK_PIN": "ram_clk_cal",
                               "OUT_HI_PIN": f"{bus}_OUT_HI[{index}]", "OUT_LO_PIN": f"{bus}_OUT_LO[{index}]",
                               "OUT_CLK_PIN": "ram_clk", "OE_PIN": f"{bus}_OE[{index}]"})
    if out_inverted:
        design.set_property(name, {"IS_OUTCLK_INVERTED": "1"})
    design.assign_resource(name, pin)

hbc_output("hbc_ck_p", "GPIOL_N_16", "DDIO", clock="ram_clk90", inverted=True, ddio=True)
hbc_output("hbc_cs_n", "GPIOL_P_18", "REG")
hbc_output("hbc_rst_n", "GPIOL_P_17", None)
hbc_inout("hbc_rwds", "GPIOL_P_15", "hbc_rwds", 0, out_inverted=False, pull="WEAK_PULLUP")
for i, pin in enumerate(["GPIOL_P_14", "GPIOL_N_09", "GPIOL_N_17", "GPIOL_N_15",
                         "GPIOL_P_16", "GPIOL_N_14", "GPIOL_N_10", "GPIOL_P_10"]):
    hbc_inout(f"hbc_dq{i}", pin, "hbc_dq", i, out_inverted=True, pull="WEAK_PULLDOWN")

# ethernet PHY PEF7071 management interface, 3.3 V banks
design.create_output_gpio("phy_mdc")
design.set_property("phy_mdc", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("phy_mdc", "GPIOL_06")

design.create_inout_gpio("phy_mdio")
design.set_property("phy_mdio", {"IO_STANDARD": "3.3_V_LVCMOS", "PULL_OPTION": "WEAK_PULLUP",
                                 "IN_PIN": "phy_mdi", "OUT_PIN": "phy_mdo", "OE_PIN": "phy_mdo_en"})
design.assign_resource("phy_mdio", "GPIOR_15")

# RGMII as board_test: DDIO resync registers, transmit on io_tseClk with the clock from a
# DDIO pin (the PHY adds the skew), receive on the clock of the receive PLL
def ddio_output(name, pin, clock):
    design.create_output_gpio(name)
    design.set_property(name, {"IO_STANDARD": "3.3_V_LVCMOS", "OUT_REG": "DDIO_RESYNC",
                               "OUT_HI_PIN": f"{name}_HI", "OUT_LO_PIN": f"{name}_LO",
                               "OUT_CLK_PIN": clock})
    design.assign_resource(name, pin)

def ddio_input(name, pin, clock, hi, lo):
    design.create_input_gpio(name)
    design.set_property(name, {"IO_STANDARD": "3.3_V_LVCMOS", "IN_REG": "DDIO_RESYNC",
                               "IN_HI_PIN": hi, "IN_LO_PIN": lo, "IN_CLK_PIN": clock})
    design.assign_resource(name, pin)

for i, pin in enumerate(["GPIOR_19", "GPIOR_12", "GPIOR_16", "GPIOR_13"]):
    design.create_output_gpio(f"rgmii_txd{i}")
    design.set_property(f"rgmii_txd{i}", {"IO_STANDARD": "3.3_V_LVCMOS", "OUT_REG": "DDIO_RESYNC",
                                          "OUT_HI_PIN": f"rgmii_txd_HI[{i}]", "OUT_LO_PIN": f"rgmii_txd_LO[{i}]",
                                          "OUT_CLK_PIN": "io_tseClk"})
    design.assign_resource(f"rgmii_txd{i}", pin)
ddio_output("rgmii_tx_ctl", "GPIOR_18", "io_tseClk")
ddio_output("rgmii_txc", "GPIOR_20", "io_tseClk")

for i, pin in enumerate(["GPIOL_10", "GPIOL_07", "GPIOL_09", "GPIOL_03"]):
    ddio_input(f"rgmii_rxd{i}", pin, "rgmii_rxc", f"rgmii_rxd_HI[{i}]", f"rgmii_rxd_LO[{i}]")
ddio_input("rgmii_rx_ctl", "GPIOL_04", "rgmii_rxc", "rgmii_rx_ctl_HI", "rgmii_rx_ctl_LO")

# RX_CLK from the PHY into PLL_TL0, 125 MHz at 1000 Mb/s, core feedback so the capture
# clock at the IO registers follows the pin (as board_test)
design.create_pll_input_clock_gpio("rgmii_rx_clk")
design.set_property("rgmii_rx_clk", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("rgmii_rx_clk", "GPIOL_11")

design.create_block("rx_pll", "PLL")
design.assign_resource("rx_pll", "PLL_TL0", "PLL")
design.set_property("rx_pll", {
    "REFCLK_SOURCE"      : "EXTERNAL",
    "EXT_CLK"            : "EXT_CLK2",
    "REFCLK_FREQ"        : "125.0",
    "LOCKED_PIN"         : "rx_pll_locked",
    "RSTN_PIN"           : "rx_pll_rstn",
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_PIN"        : "rgmii_rxc",
    "M"                  : "1",
    "N"                  : "1",
    "O"                  : "2",
    "CLKOUT0_DIV"        : "16",
}, block_type="PLL")
design.set_property("rx_pll", {"FEEDBACK_MODE": "CORE", "FEEDBACK_CLK": "CLK0"}, block_type="PLL")

# ADS7056 behind an 8 input analog mux (ad_subsystem.vhd, sec), 1.8 V, as board_test
for name, pin in [("adc_sclk", "GPIOB_N_00"), ("adc_cs_n", "GPIOL_P_00"),
                  ("adc_mux0", "GPIOR_P_05"), ("adc_mux1", "GPIOR_N_07"), ("adc_mux2", "GPIOR_P_09")]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)
design.create_input_gpio("adc_sdo")
design.set_property("adc_sdo", "IO_STANDARD", "1.8_V_LVCMOS")
design.assign_resource("adc_sdo", "GPIOL_N_00")

# the second ADS7056 and its mux ("pri" in the power supply design), SDO on a 3.3 V bank
for name, pin in [("adc_pri_sclk", "GPIOB_P_17"), ("adc_pri_cs_n", "GPIOB_P_14"),
                  ("adc_pri_mux0", "GPIOB_N_14"), ("adc_pri_mux1", "GPIOB_P_12"),
                  ("adc_pri_mux2", "GPIOR_P_07")]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)
design.create_input_gpio("adc_pri_sdo")
design.set_property("adc_pri_sdo", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("adc_pri_sdo", "GPIOR_29")

# the power stage's outputs, as in the power supply design (ac_in_ac_out_lab_power_supply),
# all driven low by tsemac_soc_top.vhd: gate drives, gate driver supplies and bypass relays
# off instead of floating with the unused pins' weak pull-up
for name, pin in [
        ("dab_primary_hi", "GPIOB_N_07"),
        ("dab_primary_low", "GPIOB_P_02"),
        ("dab_secondary_hi", "GPIOB_P_00"),
        ("dab_secondary_low", "GPIOL_N_06"),
        ("output_inu_leg1_hi", "GPIOL_N_02"),
        ("output_inu_leg1_low", "GPIOL_P_02"),
        ("output_inu_leg2_low", "GPIOL_N_05"),
        ("grid_inu_leg1_hi", "GPIOB_P_07"),
        ("grid_inu_leg1_low", "GPIOR_N_09"),
        ("grid_inu_leg2_hi", "GPIOB_N_12"),
        ("grid_inu_leg2_low", "GPIOB_P_13"),
        ("primary_bypass_relay", "GPIOL_P_08"),
        ("secondary_bypass_relay", "GPIOB_P_08"),
        ("gate_power1_pwm", "GPIOB_N_15"),
        ("gate_power2_pwm", "GPIOB_N_17"),
        ("gate_power3_pwm", "GPIOB_P_15"),
        ("gate_power4_pwm", "GPIOB_P_01"),
        ("gate_power5_pwm", "GPIOL_N_07"),
        ("gate_power6_pwm", "GPIOB_N_01"),
]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)

# the face panel's LCD (MSP3520, ILI9488 over SPI; lcd_display.vhd), through a 1.8 V to 3.3 V
# buffer; its touch controller's pins (T_CK GPIOR_P_18, T_CS GPIOR_N_18, T_DIN GPIOR_N_19) unused
for name, pin in [
        ("lcd_cs", "GPIOR_N_06"),
        ("lcd_reset", "GPIOR_P_11"),
        ("lcd_dc", "GPIOR_P_13"),
        ("lcd_sck", "GPIOR_N_13"),
        ("lcd_mosi", "GPIOR_N_16"),
        ("lcd_led", "GPIOR_P_16"),
]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)

# a board fix: output_inu leg 2's high side is on a flywire to GPIOR_21 (a 3.3 V bank) instead of
# the schematic's GPIOL_P_09, which is left unassigned (only its weak pull-up, never driven: its
# trace may still reach the gate driver)
design.create_output_gpio("output_inu_leg2_hi")
design.set_property("output_inu_leg2_hi", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("output_inu_leg2_hi", "GPIOR_21")

# the power stage's isolated sigma-delta current modulators (ad_subsystem.vhd, sdm_adc.vhd):
# a 20 MHz clock to each, 1.8 V, and their bit streams back through the isolation, 3.3 V,
# with the power supply design's pins
for name, pin in [
        ("grid_inu_sdm_clock", "GPIOR_N_05"),
        ("output_inu_sdm_clock", "GPIOL_P_07"),
        ("dab_sdm_clock", "GPIOB_N_02"),
]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)
for name, pin in [
        ("grid_inu_sdm_data", "GPIOR_25"),
        ("output_inu_sdm_data", "GPIOR_28"),
        ("dab_sdm_data", "GPIOR_27"),
]:
    design.create_input_gpio(name)
    design.set_property(name, "IO_STANDARD", "3.3_V_LVCMOS")
    design.assign_resource(name, pin)

# configuration flash on the SPI configuration pins, the SoC's SPI0, as Efinix's design
def spi_output(name, pin):
    design.create_output_gpio(name)
    design.set_property(name, {"IO_STANDARD": "1.8_V_LVCMOS", "OUT_REG": "REG",
                               "OUT_CLK_PIN": "io_tseClk"})
    design.assign_resource(name, pin)

def spi_data(index, pin):
    name = f"system_spi_0_io_data_{index}"
    design.create_inout_gpio(name)
    design.set_property(name, {"IO_STANDARD": "1.8_V_LVCMOS", "IN_REG": "REG", "OUT_REG": "REG",
                               "OE_REG": "REG"})
    design.set_property(name, {"IN_PIN": f"{name}_read", "OUT_PIN": f"{name}_write",
                               "OE_PIN": f"{name}_writeEnable", "IN_CLK_PIN": "io_tseClk",
                               "OUT_CLK_PIN": "io_tseClk"})
    design.assign_resource(name, pin)

spi_output("system_spi_0_io_sclk_write", "GPIOL_N_01")
spi_output("system_spi_0_io_ss", "GPIOL_P_01")
spi_data(0, "GPIOL_P_03")
spi_data(1, "GPIOL_N_03")

# the SoC's RISC-V debug module on the FPGA JTAG TAP, instruction USER1
design.create_block("jtag_inst1", "JTAG")
design.assign_resource("jtag_inst1", "JTAG_USER1", "JTAG")

# internal reconfiguration (remote update, remote_update.vhd): its clock must be 100 MHz or
# less, the internal oscillator at 10 MHz; the golden image at 0 is never written remotely
design.create_block("osc", block_type="OSC")
design.assign_resource("osc", "OSC_0", block_type="OSC")
design.set_property("osc", {"FREQ": "10", "CLKOUT_PIN": "cfg_clk"}, block_type="OSC")
design.set_device_property("cfg", "RECONFIG_EN", "1", "RU")
design.set_device_property("cfg", "CLK_PIN", "cfg_clk", "RU")
design.set_device_property("cfg", "CBSEL_PIN", "cfg_CBSEL", "RU")
design.set_device_property("cfg", "ENA_PIN", "cfg_ENA", "RU")
design.set_device_property("cfg", "CONFIG_PIN", "cfg_CONFIG", "RU")
design.set_device_property("cfg", "ERROR_PIN", "cfg_ERROR", "RU")

print(design.calc_pll_clock("main_pll"))
print(design.calc_pll_clock("rx_pll"))

design.save()
