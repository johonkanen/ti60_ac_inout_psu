#!/usr/bin/env python3
# Generates the Sapphire SoC IP for running Zephyr next to board_test, with the Efinity
# IP Manager API (needs java on PATH; build.sh soc sets it up).
#   rv32imac at 200 MHz (the design's hyperram_clock), no FPU
#   64 KB on-chip RAM at 0xF9000000, 4 KB instruction and data caches
#   UART0 at 0xF8010000: Zephyr's console on the board's UART pins
#   UART1: wired inside the FPGA to board_test's register bus UART, 5 Mbaud
#   debug through the FPGA JTAG TAP USER1, RISC-V standard debug for OpenOCD
# Run with Efinity's python after sourcing setup.sh.
import os
import sys

sys.path.append(os.environ['EFXIPM_HOME'] + "/bin")
from ipm_api_service.design import IPMDesignAPI

here = os.path.dirname(os.path.abspath(__file__))

config = {
    "Frequency"        : "200",
    "OCRSize"          : str(64 * 1024),
    "MULDIV_EXT"       : "1'b1",
    "ATMEXT"           : "1'b1",
    "CMREXT"           : "1'b1",
    "FPEXT"            : "1'b0",
    "REDUCED_CSR"      : "1'b0",
    "DEBUG"            : "1'b1",
    "SOFT_TAP"         : "1'b0",
    "RISCV_DEBUG_PRIV" : "1'b1",
    "HARD_BREAKPOINT"  : "2",
    # on-chip RAM only for now: no external memory or AXI
    "DDR"              : "1'b0",
    "AXISlave"         : "1'b0",
    "AXIMaster"        : "1'b0",
    "APBSlave0"        : "1'b0",
    "SPI0"             : "1'b0",
    "I2C0"             : "1'b0",
    "GPIO0"            : "1'b0",
    "USER_0_INTR"      : "1'b0",
    "UART0"            : "1'b1",
    "UART1"            : "1'b1",
}

design = IPMDesignAPI("Ti60F225", "Titanium", here, is_verbose=True)
design.create_ip("sapphire", "efinixinc.com", "soc", "efx_soc")
design.config_ip("sapphire", config)
ok, results, params = design.validate_ip("sapphire")
if not ok:
    sys.exit(f"IP validation failed: {results}")
design.generate_ip("sapphire")
