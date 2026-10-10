#!/usr/bin/env python3
# Scope deep capture into the HyperRAM, checked over the UART.
#   python3 scope_deep_test.py <port> [--length N] [--pre N] [--divider D]
# The capture of the test generator is read back through the HyperRAM tester streaming
# port: the sine on channel 0 must cross the trigger level at the trigger sample and the
# sample counter on channel 7 must count without gaps.
import argparse
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
    '..', 'source', 'fpga_communication', 'fpga_uart_pc_software'))
import serial
from uart_communication_functions import uart_link

if not hasattr(serial.Serial, 'set_buffer_size'):
    serial.Serial.set_buffer_size = lambda self, rx_size=None, tx_size=None: None

SCOPE = 600
STATUS, DIVIDER, TRIGGER, LEVEL, PRE, LENGTH, SOURCES, PHASE_INCREMENT, INFO, START, \
    MODE, OVERFLOWS, START_RECORD, RING_RECORDS = range(14)
CALIBRATION_STATUS, STREAM_ADDRESS, STREAM_DATA = 222, 213, 214
WORDS_PER_RECORD = 8
MEMORY_WORDS = 2**25

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--baud', type=int, default=5_000_000)
parser.add_argument('--length', type=int, default=1_000_000, help='samples in the capture')
parser.add_argument('--pre', type=int, default=400_000, help='samples before the trigger')
parser.add_argument('--divider', type=int, default=9, help='a sample every divider+1 clocks of 125 MHz')
parser.add_argument('--window', type=int, default=2000, help='samples read around the trigger and at the ends')
parser.add_argument('--rate_sweep', action='store_true', help='find the fastest sample rate without overflows')
args = parser.parse_args()

uart = uart_link(args.port, args.baud, number_of_databytes=4)
failures = 0

def get(address):
    return uart.request_data_from_address(address)

def set(address, value):
    uart.write_data_to_address(address, value & 0xffffffff)

def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<34} {detail}")
    failures += not ok

def signed16(v):
    return v - 65536 if v & 0x8000 else v

def capture(length, pre, divider, trigger=0x100, level=0, timeout=10.0):
    set(SCOPE + MODE, 1)
    set(SCOPE + DIVIDER, divider)
    set(SCOPE + SOURCES, 0x76543210)
    set(SCOPE + PHASE_INCREMENT, 2**32 // 1000)       # 1000 samples per sine period
    set(SCOPE + TRIGGER, trigger)
    set(SCOPE + LEVEL, level)
    set(SCOPE + PRE, pre)
    set(SCOPE + LENGTH, length)
    started = time.monotonic()
    set(SCOPE + STATUS, 1)
    while get(SCOPE + STATUS) & 7 != 4:
        if time.monotonic() - started > timeout:
            raise TimeoutError(f'capture not done, status 0x{get(SCOPE + STATUS):08x}')
    return time.monotonic() - started, get(SCOPE + OVERFLOWS), get(SCOPE + START_RECORD)

def read_samples(start_record, first, count):
    """samples first..first+count-1 of the capture as lists of 8 channels"""
    ring = get(SCOPE + RING_RECORDS)
    samples = []
    while count > 0:
        record = (start_record + first) % ring
        n = min(count, ring - record, 4096)
        set(STREAM_ADDRESS, record * WORDS_PER_RECORD)
        values = uart.stream_data_from_address(STREAM_DATA, n * WORDS_PER_RECORD // 2)
        words = []
        for v in values:
            words += [int(v) >> 16, int(v) & 0xffff]
        samples += [[signed16(w) for w in words[i:i + 8]] for i in range(0, len(words), 8)]
        first += n
        count -= n
    return samples

def counter_gaps(samples):
    return sum((b[7] - a[7]) % 65536 != 1 for a, b in zip(samples, samples[1:]))

check("uart id word", get(1) == 0xacdc)
status = get(CALIBRATION_STATUS)
phase, lo_first = (status >> 24) & 7, (status >> 27) & 1
check("hyperram read calibration", status & 3 == 1,
      f"status 0x{status:08x}, phase {phase} lo first {lo_first}, "
      f"pass map hi first {(status >> 8) & 0xff:08b} lo first {(status >> 16) & 0xff:08b}")

ring = get(SCOPE + RING_RECORDS)
print(f"ring of {ring} records, capture of {args.length} samples with {args.pre} before the trigger, "
      f"{125e6 / (args.divider + 1) / 1e6:.3f} MS/s")
duration, overflows, start = capture(args.length, args.pre, args.divider)
check("capture done without overflows", overflows == 0, f"{duration:.3f} s, {overflows} overflows, start record {start}")

window = min(args.window, args.pre, args.length - args.pre)
around = read_samples(start, args.pre - window, 2 * window)
check("rising crossing at trigger sample", around[window - 1][0] < 0 <= around[window][0],
      f"{around[window - 1][0]} -> {around[window][0]}")
check("counter continuous around trigger", counter_gaps(around) == 0, f"{counter_gaps(around)} gaps")
head = read_samples(start, 0, window)
tail = read_samples(start, args.length - window, window)
check("counter continuous at the start", counter_gaps(head) == 0, f"{counter_gaps(head)} gaps")
check("counter continuous at the end", counter_gaps(tail) == 0, f"{counter_gaps(tail)} gaps")
check("whole capture spans length samples", (tail[-1][7] - head[0][7]) % 65536 == (args.length - 1) % 65536,
      f"counter {head[0][7]} .. {tail[-1][7]}")

if args.rate_sweep:
    for divider in range(8, -1, -1):
        duration, overflows, start = capture(200_000, 0, divider, trigger=0)
        print(f"  {125 / (divider + 1):7.3f} MS/s  {overflows:7d} overflows")

print(f"{'all passed' if failures == 0 else f'{failures} failed'}")
sys.exit(failures != 0)
