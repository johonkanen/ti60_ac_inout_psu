#!/bin/bash
# Load a Zephyr ELF over JTAG and run it from its entry point: the on-chip RAM of
# board_test_cpu or the HyperRAM of tsemac_soc. The SoC is reset through the debug module
# first: a core that has taken a fetch fault can leave the instruction bus wedged until it
# is reset.
#   ./load.sh build/psu_test/zephyr/zephyr.elf
set -e
elf=$(realpath "$1")
entry=$(${READELF:-$HOME/zephyr-sdk-1.0.1/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-readelf} -h "$elf" | awk '/Entry point/ {print $4}')
cd "$(dirname "$0")"
OPENOCD=${OPENOCD:-$HOME/efinity/2026.1/debugger/openocd/bin/openocd}
"$OPENOCD" -f openocd.cfg -c "init; reset halt; load_image $elf; verify_image $elf; resume $entry; shutdown" 2>&1 \
    | grep -E 'Error|Warn|verified' || true
