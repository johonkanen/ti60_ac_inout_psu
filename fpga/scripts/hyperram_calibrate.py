#!/usr/bin/env python3
# Read clock calibration of the 200 MHz HyperRAM controller, over UDP.
# Known data is written once (writes do not depend on the read clock), then read back with
# every read clock phase step and both orders of the DDIO input halves. The middle of the
# widest error free phase window is selected.
#   python3 hyperram_calibrate.py [--words N]
import argparse
import os
import random
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from udp_link import udp_link
import hyperram_stream as stream

READ_SETUP = 218
ID0 = 200 + 3

parser = argparse.ArgumentParser()
parser.add_argument('--words', type=int, default=4096)
parser.add_argument('--address', type=int, default=0x100000)
args = parser.parse_args()

link = udp_link()
data = [random.getrandbits(16) for _ in range(args.words)]
stream.write_words(link, args.address, data)

results = {}
for lo_first in (0, 1):
    line = []
    for phase in range(8):
        link.write_data_to_address(READ_SETUP, phase | lo_first << 8)
        readback = stream.read_words(link, args.address, args.words)
        errors = sum(a != b for a, b in zip(data, readback))
        results[(lo_first, phase)] = errors
        line.append('ok' if errors == 0 else f'{errors}')
    print(f"input lo first {lo_first}: " + "  ".join(f"{p}:{e:>5}" for p, e in enumerate(line)))

best = None
for lo_first in (0, 1):
    start = None
    for phase in range(9):
        good = phase < 8 and results[(lo_first, phase)] == 0
        if good and start is None:
            start = phase
        if not good and start is not None:
            width = phase - start
            if best is None or width > best[0]:
                best = (width, lo_first, start + width // 2)
            start = None

if best is None:
    print("no error free read phase found")
    sys.exit(1)
width, lo_first, phase = best
link.write_data_to_address(READ_SETUP, phase | lo_first << 8)
print(f"selected read phase {phase}, input lo first {lo_first}, window {width} steps of 0.25 ns")
