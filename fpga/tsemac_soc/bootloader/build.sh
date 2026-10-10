#!/bin/bash
# Build boot0.hex, the on-chip RAM image that gen_ips.py gives the SoC. Needs a RISC-V gcc:
# the Zephyr SDK's by default. boot0.hex is committed, so the FPGA build does not need it.
set -e
cd "$(dirname "$0")"
GCC=${GCC:-$HOME/zephyr-sdk-1.0.1/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-gcc}
"$GCC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany -mno-relax -Os \
    -ffreestanding -fno-builtin -nostdlib -nostartfiles -Wall -Wextra \
    -T boot0.ld -Wl,--gc-sections boot0.S boot0.c -o boot0.elf
"${GCC%gcc}objcopy" -O ihex boot0.elf boot0.hex
"${GCC%gcc}size" boot0.elf
