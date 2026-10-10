#!/usr/bin/env python3
# Writes a firmware binary into the Sapphire on-chip RAM initialisation files.
# The RAM is four byte lanes, each read by sapphire.v with $readmemb relative to its own directory.
#   ram_init.py zephyr.bin [ram_bytes]
import sys
from pathlib import Path

binary    = Path(sys.argv[1]).read_bytes()
ram_bytes = int(sys.argv[2]) if len(sys.argv) > 2 else 64 * 1024
words     = ram_bytes // 4
if len(binary) > ram_bytes:
    sys.exit(f"{sys.argv[1]} is {len(binary)} bytes, on-chip RAM is {ram_bytes}")

binary = binary.ljust(ram_bytes, b"\0")
here   = Path(__file__).resolve().parent / "ip" / "sapphire"
for lane in range(4):
    lines = (f"{binary[4 * word + lane]:08b}\n" for word in range(words))
    (here / f"EfxSapphireSoc.v_toplevel_system_ramA_logic_ram_symbol{lane}.bin").write_text("".join(lines))
print(f"{sys.argv[1]}: {len(binary.rstrip(bytes(1)))} bytes into {ram_bytes} byte on-chip RAM")
