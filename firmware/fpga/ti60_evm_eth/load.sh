#!/bin/bash
# Load a Zephyr ELF into HyperRAM over JTAG and start both harts at its entry.
# Both harts are held in reset-halt while loading: a running hart would execute the
# flash bootloader, which copies flash into the same HyperRAM area.
# Zephyr parks hart 1 itself.
#   ./load.sh build/zephyr/zephyr.elf
set -e
elf=$(realpath "$1")
cd "$(dirname "$0")"
OPENOCD=${OPENOCD:-$HOME/efinity/2026.1/debugger/openocd/bin/openocd}
"$OPENOCD" -f openocd.cfg -c "init; targets fpga_spinal.cpu0; reset halt; targets fpga_spinal.cpu1; halt;
    targets fpga_spinal.cpu0; load_image $elf; verify_image $elf;
    targets fpga_spinal.cpu1; resume 0x01000000; targets fpga_spinal.cpu0; resume 0x01000000; shutdown" 2>&1 \
    | grep -E 'Error|Warn|verified|MATCH' || true
