#!/usr/bin/env python3
# HyperRAM over UDP: random data written through the streaming port and read back.
#   python3 hyperram_stream_test.py <uart port> [--words N]
import argparse
import os
import random
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mdio_test import Mdio
from udp_link import udp_link
import hyperram_stream as stream

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--words', type=int, default=2**21, help='16 bit words per region')
args = parser.parse_args()

uart = Mdio(args.port).uart
udp = udp_link()
failures = 0

def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<34} {detail}")
    failures += not ok

# regions at the start, across the die boundary and at the end of the memory
regions = [(0, args.words), (2**24 - args.words // 2, args.words), (2**25 - args.words, args.words)]
for address, count in regions:
    data = [random.getrandbits(16) for _ in range(count)]
    started = time.monotonic()
    stream.write_words(udp, address, data)
    write_time = time.monotonic() - started
    started = time.monotonic()
    readback = stream.read_words(udp, address, count)
    read_time = time.monotonic() - started
    errors = sum(a != b for a, b in zip(data, readback))
    check(f"words 0x{address:07x}..0x{address + count - 1:07x}", errors == 0 and len(readback) == count,
          f"{errors} errors, write {count * 2 / write_time / 1e6:.1f} MB/s, read {count * 2 / read_time / 1e6:.1f} MB/s")

    # a few words through the UART single access path
    bad = 0
    for i in random.sample(range(count), 8):
        uart.write_data_to_address(201, address + i)
        uart.write_data_to_address(200, 1)
        while not uart.request_data_from_address(200) & 1:
            pass
        bad += uart.request_data_from_address(203) != data[i]
    check("  same words over uart", bad == 0, f"{bad} of 8 differ")

check("write fifo overflows", uart.request_data_from_address(216) == 0, str(uart.request_data_from_address(216)))
print(f"      stream reads that waited for the prefetch {uart.request_data_from_address(217)}, "
      f"udp retransmissions {udp.retransmissions}, bridge read timeouts {uart.request_data_from_address(509)}")
print("hyperram stream OK" if failures == 0 else f"{failures} check(s) failed")
sys.exit(failures != 0)
