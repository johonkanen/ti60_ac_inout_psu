#!/bin/bash
# Build the second-stage loader with the Zephyr SDK's toolchain: build/loader/loader.bin
set -e
cd "$(dirname "$0")"
GCC=${GCC:-$HOME/zephyr-sdk-1.0.1/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-gcc}
OUT=../build/loader
mkdir -p $OUT
# -mno-relax: nothing sets gp. -fno-builtin: no calls into a C library there is none of.
"$GCC" -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mcmodel=medany -mno-relax -Os -g \
    -ffreestanding -fno-builtin -nostdlib -nostartfiles -Wall -Wextra \
    -T loader.ld -Wl,--gc-sections -Wl,-Map,$OUT/loader.map \
    start.S main.c -o $OUT/loader.elf
"${GCC%gcc}objcopy" -O binary $OUT/loader.elf $OUT/loader.bin
echo "$OUT/loader.bin: $(stat -c %s $OUT/loader.bin) bytes"
