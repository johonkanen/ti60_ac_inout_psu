#!/usr/bin/env python3
# Generates uart_test.peri.xml with the Efinity Interface Designer API.
# Pinout taken from ac_in_ac_out_lab_power_supply/titanium_build.
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
import os
import sys

sys.path.append(os.environ['EFXPT_HOME'] + "/bin")
from api_service.design import DesignAPI

project = "board_test"
here    = os.path.dirname(os.path.abspath(__file__))

design = DesignAPI(is_verbose=False)
design.create(project, "Ti60F225", here, overwrite=True)

for bank in ("1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"):
    design.set_iobank_voltage(bank, "1.8")
for bank in ("BL", "BR", "TL", "TR"):
    design.set_iobank_voltage(bank, "3.3")

design.set_gpio_unused_state("INPUT_WEAK_PULLUP")

# 50 MHz reference -> PLL_TR0, PLL at 2000 MHz: main_clock 125 MHz (also the RGMII transmit
# clock) and the 200 MHz HyperRAM clocks
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
    # PLL at 2000 MHz, phase steps of 0.25 ns
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_DIV"        : "10",
    "CLKOUT0_PIN"        : "hyperram_clock",          # 200 MHz, HyperRAM controller and outputs
    "CLKOUT0_CONN_TYPE"  : "GCLK",
    "CLKOUT1_EN"         : "1",
    "CLKOUT1_DIV"        : "16",
    "CLKOUT1_PIN"        : "main_clock",              # 125 MHz
    "CLKOUT2_EN"         : "1",
    "CLKOUT2_DIV"        : "10",
    "CLKOUT2_PIN"        : "hyperram_ck_clock",       # 200 MHz, 90 degrees, inverted at the CK pin
    "CLKOUT2_PHASE_SETTING" : "5",
    "CLKOUT3_EN"         : "1",
    "CLKOUT3_DIV"        : "10",
    "CLKOUT3_PIN"        : "hyperram_read_clock",     # 200 MHz, dynamic phase for read capture
    "CLKOUT3_DYNPHASE_EN" : "1",
    "PHASE_SHIFT_PIN"     : "hyperram_phase_shift",
    "PHASE_SHIFT_SEL_PIN" : "hyperram_phase_select",
    "PHASE_SHIFT_ENA_PIN" : "hyperram_phase_shift_ena",
}, block_type="PLL")

# UART, 3.3 V bank
design.create_input_gpio("uart_rx")
design.set_property("uart_rx", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("uart_rx", "R4")

design.create_output_gpio("uart_tx")
design.set_property("uart_tx", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("uart_tx", "R3")

# HyperRAM W959D8NFYA, 1.8 V HSIO, DDIO registers at 200 MHz, see hyperram_ddr_controller
def hyperram_output(name, pin, register, clock, hi_lo=True, invert_clock=False):
    design.create_output_gpio(name)
    properties = {"IO_STANDARD": "1.8_V_LVCMOS", "OUT_REG": register, "OUT_CLK_PIN": clock}
    if hi_lo:
        properties.update({"OUT_HI_PIN": f"{name}_hi", "OUT_LO_PIN": f"{name}_lo"})
    if invert_clock:
        properties["IS_OUTCLK_INVERTED"] = "1"
    design.set_property(name, properties)
    design.assign_resource(name, pin)

def hyperram_inout(name, pin, in_hi, in_lo, out_hi, out_lo, oe):
    design.create_inout_gpio(name)
    # the DDIO register modes first: the HI/LO pin names only exist in DDIO mode
    design.set_property(name, {"IO_STANDARD": "1.8_V_LVCMOS", "IN_REG": "DDIO_RESYNC",
                               "OUT_REG": "DDIO_RESYNC", "OE_REG": "REG"})
    design.set_property(name, {"IN_HI_PIN": in_hi, "IN_LO_PIN": in_lo, "IN_CLK_PIN": "hyperram_read_clock",
                               "OUT_HI_PIN": out_hi, "OUT_LO_PIN": out_lo, "OUT_CLK_PIN": "hyperram_clock",
                               "OE_PIN": oe})
    design.assign_resource(name, pin)
    names = design.get_property(name, ["IN_HI_PIN", "IN_LO_PIN", "OUT_HI_PIN", "OUT_LO_PIN", "OE_PIN"])
    expected = {"IN_HI_PIN": in_hi, "IN_LO_PIN": in_lo, "OUT_HI_PIN": out_hi, "OUT_LO_PIN": out_lo, "OE_PIN": oe}
    if names != expected:
        raise SystemExit(f"pin names of {name} not applied: {names}")

hyperram_output("hyperram_ck", "GPIOL_N_16", "DDIO_RESYNC", "hyperram_ck_clock", invert_clock=True)
design.create_output_gpio("hyperram_cs_n")
design.set_property("hyperram_cs_n", {"IO_STANDARD": "1.8_V_LVCMOS", "OUT_REG": "REG", "OUT_CLK_PIN": "hyperram_clock"})
design.assign_resource("hyperram_cs_n", "GPIOL_P_18")
design.create_output_gpio("hyperram_reset_n")
design.set_property("hyperram_reset_n", {"IO_STANDARD": "1.8_V_LVCMOS", "OUT_REG": "REG", "OUT_CLK_PIN": "main_clock"})
design.assign_resource("hyperram_reset_n", "GPIOL_P_17")

hyperram_inout("hyperram_rwds", "GPIOL_P_15", "hyperram_rwds_in_hi", "hyperram_rwds_in_lo",
               "hyperram_rwds_out_hi", "hyperram_rwds_out_lo", "hyperram_rwds_oe")

hyperram_dq_pins = ["GPIOL_P_14", "GPIOL_N_09", "GPIOL_N_17", "GPIOL_N_15",
                    "GPIOL_P_16", "GPIOL_N_14", "GPIOL_N_10", "GPIOL_P_10"]
for i, pin in enumerate(hyperram_dq_pins):
    hyperram_inout(f"hyperram_dq{i}", pin, f"hyperram_dq_in_hi[{i}]", f"hyperram_dq_in_lo[{i}]",
                   f"hyperram_dq_out_hi[{i}]", f"hyperram_dq_out_lo[{i}]", f"hyperram_dq_oe[{i}]")

# ethernet PHY PEF7071 management interface, 3.3 V banks
design.create_output_gpio("ethernet_mdc")
design.set_property("ethernet_mdc", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("ethernet_mdc", "GPIOL_06")

design.create_inout_gpio("ethernet_mdio")
design.set_property("ethernet_mdio", {"IO_STANDARD": "3.3_V_LVCMOS", "PULL_OPTION": "WEAK_PULLUP",
                                      "IN_PIN": "ethernet_mdio_in", "OUT_PIN": "ethernet_mdio_out",
                                      "OE_PIN": "ethernet_mdio_oe"})
design.assign_resource("ethernet_mdio", "GPIOR_15")

# ethernet RGMII, 3.3 V banks, all data pins through DDIO registers
def ddio_output(name, pin, clock):
    design.create_output_gpio(name)
    design.set_property(name, {"IO_STANDARD": "3.3_V_LVCMOS", "OUT_REG": "DDIO_RESYNC",
                               "OUT_HI_PIN": f"{name}_hi", "OUT_LO_PIN": f"{name}_lo",
                               "OUT_CLK_PIN": clock})
    design.assign_resource(name, pin)

def ddio_input(name, pin, clock):
    design.create_input_gpio(name)
    design.set_property(name, {"IO_STANDARD": "3.3_V_LVCMOS", "IN_REG": "DDIO_RESYNC",
                               "IN_HI_PIN": f"{name}_hi", "IN_LO_PIN": f"{name}_lo",
                               "IN_CLK_PIN": clock})
    design.assign_resource(name, pin)

for i, pin in enumerate(["GPIOR_19", "GPIOR_12", "GPIOR_16", "GPIOR_13"]):
    ddio_output(f"ethernet_txd{i}", pin, "main_clock")
ddio_output("ethernet_tx_ctl", "GPIOR_18", "main_clock")
ddio_output("ethernet_tx_clk", "GPIOR_20", "main_clock")

for i, pin in enumerate(["GPIOL_10", "GPIOL_07", "GPIOL_09", "GPIOL_03"]):
    ddio_input(f"ethernet_rxd{i}", pin, "ethernet_rx_clock")
ddio_input("ethernet_rx_ctl", "GPIOL_04", "ethernet_rx_clock")

# RX_CLK from the PHY into PLL_TL0, 125 MHz at 1000 Mb/s, core feedback so the
# capture clock at the IO registers follows the pin
design.create_pll_input_clock_gpio("ethernet_rx_clk")
design.set_property("ethernet_rx_clk", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("ethernet_rx_clk", "GPIOL_11")

design.create_block("ethernet_rx_pll", "PLL")
design.assign_resource("ethernet_rx_pll", "PLL_TL0", "PLL")
design.set_property("ethernet_rx_pll", {
    "REFCLK_SOURCE"      : "EXTERNAL",
    "EXT_CLK"            : "EXT_CLK2",
    "REFCLK_FREQ"        : "125.0",
    "LOCKED_PIN"         : "ethernet_rx_pll_locked",
    "RSTN_PIN"           : "ethernet_rx_pll_reset_n",
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_PIN"        : "ethernet_rx_clock",
    "M"                  : "1",
    "N"                  : "1",
    "O"                  : "2",
    "CLKOUT0_DIV"        : "16",
}, block_type="PLL")
# feedback is resolved by output name, so set it after the output has been named
design.set_property("ethernet_rx_pll", {"FEEDBACK_MODE": "CORE", "FEEDBACK_CLK": "CLK0"}, block_type="PLL")

print(design.calc_pll_clock("main_pll"))
print(design.calc_pll_clock("ethernet_rx_pll"))

# ADS7056 behind an 8 input analog mux (secondary ADC of the power supply), 1.8 V
for name, pin in [("adc_sclk", "GPIOB_N_00"), ("adc_cs_n", "GPIOL_P_00"),
                  ("adc_mux0", "GPIOR_P_05"), ("adc_mux1", "GPIOR_N_07"), ("adc_mux2", "GPIOR_P_09")]:
    design.create_output_gpio(name)
    design.set_property(name, "IO_STANDARD", "1.8_V_LVCMOS")
    design.assign_resource(name, pin)
design.create_input_gpio("adc_sdo")
design.set_property("adc_sdo", "IO_STANDARD", "1.8_V_LVCMOS")
design.assign_resource("adc_sdo", "GPIOL_N_00")

design.save()
