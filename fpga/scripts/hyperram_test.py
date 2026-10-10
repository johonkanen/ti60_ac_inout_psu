#!/usr/bin/env python3
# HyperRAM test for the board_test build.
#   python3 hyperram_test.py <port> [--words N] [--baud B]
import argparse
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
    '..', 'source', 'fpga_communication', 'fpga_uart_pc_software'))
import serial
from uart_communication_functions import uart_link

# uart_link calls set_buffer_size, which pyserial only implements on Windows
if not hasattr(serial.Serial, 'set_buffer_size'):
    serial.Serial.set_buffer_size = lambda self, rx_size=None, tx_size=None: None

BASE = 200
COMMAND, ADDRESS, WRITE_DATA, READ_DATA, LATENCY, TEST_WORDS, SEED, \
    ERRORS, FIRST_ERROR_ADDRESS, FIRST_ERROR_DATA, WORDS_CHECKED, TIMEOUTS, ACCEPTED = range(13)
REGISTER_SPACE = 1 << 31
DIE1 = 1 << 24

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--baud', type=int, default=5_000_000)
parser.add_argument('--words', type=int, default=2**25, help='words in pattern test, multiple of 32')
parser.add_argument('--walk_repeats', type=int, default=1, help='repetitions of the single word walking test')
args = parser.parse_args()

uart = uart_link(args.port, args.baud, number_of_databytes=4)
failures = 0

def get(offset):
    return uart.request_data_from_address(BASE + offset)

def set(offset, value):
    uart.write_data_to_address(BASE + offset, value)

def check(name, ok, detail):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<28} {detail}")
    failures += not ok

def wait_ready(timeout=1.0):
    end = time.monotonic() + timeout
    while not get(COMMAND) & 1:
        if time.monotonic() > end:
            raise TimeoutError('hyperram controller not ready')

dropped_commands = []

def run_single(command, address):
    accepted = get(ACCEPTED)
    wait_ready()
    set(ADDRESS, address)
    set(COMMAND, command)
    wait_ready()
    if get(ACCEPTED) != (accepted + 1) & 0xffffffff:
        dropped_commands.append(f"{'read' if command == 1 else 'write'} 0x{address:08x}")

def single_read(address):
    run_single(1, address)
    timed_out = bool(get(COMMAND) & 0x10)
    return get(READ_DATA), timed_out

def single_write(address, data):
    set(WRITE_DATA, data)
    run_single(2, address)

check("uart id word", uart.request_data_from_address(1) == 0xacdc, "")

for name, address, expected in [
        ("ID0 die 0", REGISTER_SPACE | 0x0000, 0x0F86),
        ("ID0 die 1", REGISTER_SPACE | DIE1 | 0x0000, 0x4F86),
        ("ID1", REGISTER_SPACE | 0x0001, 0x0001),
        ("CR0 die 0", REGISTER_SPACE | 0x0800, 0x8F2F),
        ("CR0 die 1", REGISTER_SPACE | DIE1 | 0x0800, 0x8F2F)]:
    value, timed_out = single_read(address)
    check(name, value == expected and not timed_out,
          f"read 0x{value:04x} expected 0x{expected:04x}" + (" (read timed out)" if timed_out else ""))

for die, offset in [("die 0", 0), ("die 1", DIE1)] * args.walk_repeats:
    bad = []
    for bit in range(16):
        for data in (1 << bit, 0xffff ^ (1 << bit)):
            single_write(offset + 0x100 + bit, data)
            value, _ = single_read(offset + 0x100 + bit)
            if value != data:
                bad.append(f"wrote {data:04x} read {value:04x}")
    check(f"walking ones/zeros {die}", not bad, "; ".join(bad[:4]))
check("single commands accepted", not dropped_commands, "; ".join(dropped_commands[:4]))

for seed in (0x0000, 0xffff):
    set(TEST_WORDS, args.words)
    set(SEED, seed)
    set(COMMAND, 3)
    start = time.monotonic()
    while True:
        status = get(COMMAND)
        if status & 0x4:
            break
        time.sleep(0.5)
        print(f"\r  pattern test seed {seed:04x}: {get(WORDS_CHECKED):>9} / {args.words} words read back", end='')
    print()
    errors, checked, timeouts = get(ERRORS), get(WORDS_CHECKED), get(TIMEOUTS)
    detail = f"{checked} words in {time.monotonic() - start:.1f} s, {errors} errors, {timeouts} read timeouts"
    if errors:
        first = get(FIRST_ERROR_DATA)
        detail += (f", first at 0x{get(FIRST_ERROR_ADDRESS):07x}"
                   f" expected 0x{first >> 16:04x} read 0x{first & 0xffff:04x}")
    check(f"pattern test seed 0x{seed:04x}", errors == 0 and checked == args.words, detail)

print("hyperram OK" if failures == 0 else f"{failures} check(s) failed")
sys.exit(failures != 0)
