#!/usr/bin/env python3
# Generates uart_test.peri.xml with the Efinity Interface Designer API.
# Pinout taken from ac_in_ac_out_lab_power_supply/titanium_build.
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
import os
import sys

sys.path.append(os.environ['EFXPT_HOME'] + "/bin")
from api_service.design import DesignAPI

project = "uart_test"
here    = os.path.dirname(os.path.abspath(__file__))

design = DesignAPI(is_verbose=False)
design.create(project, "Ti60F225", here, overwrite=True)

for bank in ("1A", "1B", "2A", "2B", "3A", "3B", "4A", "4B"):
    design.set_iobank_voltage(bank, "1.8")
for bank in ("BL", "BR", "TL", "TR"):
    design.set_iobank_voltage(bank, "3.3")

design.set_gpio_unused_state("INPUT_WEAK_PULLUP")

# 50 MHz reference -> PLL_TR0, main_clock = 120 MHz
design.create_pll_input_clock_gpio("ref_clock")
design.assign_pkg_pin("ref_clock", "C14")

design.create_block("main_pll", "PLL")
design.assign_resource("main_pll", "PLL_TR0", "PLL")
design.set_property("main_pll", {
    "REFCLK_SOURCE"      : "EXTERNAL",
    "EXT_CLK"            : "EXT_CLK0",
    "REFCLK_FREQ"        : "50.0",
    "M"                  : "2",
    "N"                  : "1",
    "O"                  : "2",
    "FEEDBACK_MODE"      : "LOCAL",
    "FEEDBACK_CLK"       : "CLK0",
    "LOCKED_PIN"         : "pll_locked",
    "CLKOUT0_EN"         : "1",
    "CLKOUT0_DIV"        : "24",
    "CLKOUT0_PIN"        : "main_pll_CLKOUT0",
    "CLKOUT0_CONN_TYPE"  : "GCLK",
    "CLKOUT1_EN"         : "1",
    "CLKOUT1_DIV"        : "20",
    "CLKOUT1_PIN"        : "main_clock",
}, block_type="PLL")

# UART, 3.3 V bank
design.create_input_gpio("uart_rx")
design.set_property("uart_rx", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("uart_rx", "R4")

design.create_output_gpio("uart_tx")
design.set_property("uart_tx", "IO_STANDARD", "3.3_V_LVCMOS")
design.assign_pkg_pin("uart_tx", "R3")

print(design.calc_pll_clock("main_pll"))

design.save()
