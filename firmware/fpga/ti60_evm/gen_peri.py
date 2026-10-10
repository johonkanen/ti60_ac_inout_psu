#!/usr/bin/env python3
# Generates ti60_evm.peri.xml with the Efinity Interface Designer API.
# Pinout taken from the Sapphire IP's Ti60F225_devkit example design.
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
import os
import sys

sys.path.append(os.environ['EFXPT_HOME'] + "/bin")
from api_service.design import DesignAPI

project = "ti60_evm"
here    = os.path.dirname(os.path.abspath(__file__))

design = DesignAPI(is_verbose=False)
design.create(project, "Ti60F225", here, overwrite=True)

for bank in ("1A", "1B", "2A", "2B"):
    design.set_iobank_voltage(bank, "1.8")
for bank in ("3A", "3B", "4A", "4B"):
    design.set_iobank_voltage(bank, "1.2")
for bank in ("BL", "BR", "TL", "TR"):
    design.set_iobank_voltage(bank, "3.3")

design.set_gpio_unused_state("INPUT_WEAK_PULLUP")

# 25 MHz oscillator -> PLL_TL0, system_clock = 100 MHz
design.create_pll_input_clock_gpio("ref_clock")
design.set_property("ref_clock", "IO_STANDARD", "1.8_V_LVCMOS")
design.assign_resource("ref_clock", "GPIOL_P_18")

design.create_block("system_pll", "PLL")
design.assign_resource("system_pll", "PLL_TL0", "PLL")
design.set_property("system_pll", {
    "REFCLK_SOURCE"      : "EXTERNAL",
    "EXT_CLK"            : "EXT_CLK0",
    "REFCLK_FREQ"        : "25.0",
    "M"                  : "4",
    "N"                  : "1",
    "O"                  : "2",
    "FEEDBACK_MODE"      : "LOCAL",
    "FEEDBACK_CLK"       : "CLK0",
    "LOCKED_PIN"         : "pll_locked",
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_DIV"        : "16",
    "CLKOUT0_PIN"        : "system_clock",
    "CLKOUT0_CONN_TYPE"  : "GCLK",
}, block_type="PLL")

# push button, 1.2 V bank
design.create_input_gpio("reset_n")
design.set_property("reset_n", {"IO_STANDARD": "1.2_V_LVCMOS", "PULL_OPTION": "WEAK_PULLUP"})
design.assign_resource("reset_n", "GPIOR_P_06")

# UART to the FT4232H, 3.3 V bank
design.create_input_gpio("uart_rx")
design.set_property("uart_rx", {"IO_STANDARD": "3.3_V_LVCMOS", "PULL_OPTION": "WEAK_PULLUP"})
design.assign_resource("uart_rx", "GPIOL_01")

design.create_output_gpio("uart_tx")
design.set_property("uart_tx", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_resource("uart_tx", "GPIOL_02")

# LEDs, 1.2 V bank
for i, pin in enumerate(["GPIOR_N_06", "GPIOR_N_07", "GPIOR_N_09", "GPIOR_N_08"]):
    name = f"gpio{i}"
    design.create_inout_gpio(name)
    design.set_property(name, {"IO_STANDARD": "1.2_V_LVCMOS",
                               "IN_PIN": f"gpio_in[{i}]", "OUT_PIN": f"gpio_out[{i}]", "OE_PIN": f"gpio_oe[{i}]"})
    design.assign_resource(name, pin)

# RISC-V debugger on the FPGA JTAG TAP
design.create_block("jtag_inst1", "JTAG")
design.assign_resource("jtag_inst1", "JTAG_USER1", "JTAG")

print(design.calc_pll_clock("system_pll"))

design.save()
