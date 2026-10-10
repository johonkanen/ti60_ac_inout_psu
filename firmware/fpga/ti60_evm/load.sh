#!/bin/bash
# Load a Zephyr ELF into the Sapphire on-chip RAM over JTAG and run it.
# The SoC is reset through the debug module first: a core that has taken a
# fetch fault can leave the instruction bus wedged until it is reset.
#   ./load.sh build/zephyr/zephyr.elf
set -e
elf=$(realpath "$1")
cd "$(dirname "$0")"
OPENOCD=${OPENOCD:-$HOME/efinity/2026.1/debugger/openocd/bin/openocd}
"$OPENOCD" -f openocd.cfg -c "init; reset halt; load_image $elf; resume 0xf9000000; shutdown" 2>&1 \
    | grep -E 'Error|Warn' || true
