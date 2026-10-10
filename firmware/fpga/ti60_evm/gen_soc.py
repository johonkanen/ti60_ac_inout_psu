#!/usr/bin/env python3
# Generates the Sapphire SoC IP for the Zephyr bring-up with the Efinity IP Manager API.
# The configuration matches upstream Zephyr's efinix/sapphire_soc.dtsi:
#   rv32ima, 100 MHz, 192 KB on-chip RAM at 0xF9000000, UART0 at 0xF8010000, 4 bit GPIO0 at 0xF8015000
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
import os
import sys

sys.path.append(os.environ['EFXIPM_HOME'] + "/bin")
from ipm_api_service.design import IPMDesignAPI

here = os.path.dirname(os.path.abspath(__file__))

config = {
    "Frequency"        : "100",
    "OCRSize"          : str(192 * 1024),
    "MULDIV_EXT"       : "1'b1",
    "ATMEXT"           : "1'b1",
    "CMREXT"           : "1'b0",
    "REDUCED_CSR"      : "1'b0",
    # debug through the FPGA JTAG TAP USER1, RISC-V standard debug for stock OpenOCD
    "DEBUG"            : "1'b1",
    "SOFT_TAP"         : "1'b0",
    "RISCV_DEBUG_PRIV" : "1'b1",
    "HARD_BREAKPOINT"  : "2",
    # everything runs from on-chip RAM, no external memory or AXI
    "DDR"              : "1'b0",
    "AXISlave"         : "1'b0",
    "AXIMaster"        : "1'b0",
    "APBSlave0"        : "1'b0",
    "SPI0"             : "1'b0",
    "I2C0"             : "1'b0",
    "USER_0_INTR"      : "1'b0",
    "UART0"            : "1'b1",
    "GPIO0"            : "1'b1",
    "GPIO0Width"       : "4",
}

design = IPMDesignAPI("Ti60F225", "Titanium", here, is_verbose=True)
design.create_ip("sapphire", "efinixinc.com", "soc", "efx_soc")
design.config_ip("sapphire", config)
ok, results, params = design.validate_ip("sapphire")
if not ok:
    sys.exit("IP validation failed")
design.generate_ip("sapphire")
