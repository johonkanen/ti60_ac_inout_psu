#!/usr/bin/env python3
# Board ADS7056 and its input mux over the UART (registers at 700, see source/adc/board_adc.vhd).
#   python3 adc_test.py <port> [--sweep]
# Prints the conversion rate and, for every mux position, the spread of a few hundred
# readings. With the mux inputs floating the values only show that conversions arrive.
# --sweep tries every SDO sample delay and counts frames with bad zero bits.
import argparse
import os
import statistics
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
    '..', 'source', 'fpga_communication', 'fpga_uart_pc_software'))
import serial
from uart_communication_functions import uart_link

if not hasattr(serial.Serial, 'set_buffer_size'):
    serial.Serial.set_buffer_size = lambda self, rx_size=None, tx_size=None: None

BASE = 700
CONTROL, SCLK, SETTLE, CONVERSIONS, STATUS, CS_HIGH, SAMPLE_DELAY, ZERO_ERRORS = range(8)
RESULTS = 8
ADC_CLOCK_HZ = 200e6

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--baud', type=int, default=5_000_000)
parser.add_argument('--readings', type=int, default=200, help='readings per mux position')
parser.add_argument('--sweep', action='store_true', help='sweep the SDO sample delay')
args = parser.parse_args()

uart = uart_link(args.port, args.baud, number_of_databytes=4)
get = lambda offset: uart.request_data_from_address(BASE + offset)
set = lambda offset, value: uart.write_data_to_address(BASE + offset, value)
failures = 0

def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<34} {detail}")
    failures += not ok

def rate(control):
    set(CONTROL, control)
    time.sleep(0.05)
    first, started = get(CONVERSIONS), time.monotonic()
    time.sleep(0.5)
    count, seconds = get(CONVERSIONS), time.monotonic() - started
    return ((count - first) & 0xffffffff) / seconds

status = get(STATUS)
check("adc calibrated", bool(status & 1), f"status 0x{status:08x}")
sclk = get(SCLK)
period = (sclk & 15) + ((sclk >> 4) & 15)
print(f"SCLK {ADC_CLOCK_HZ / period / 1e6:.2f} MHz, CS high {get(CS_HIGH) / ADC_CLOCK_HZ * 1e9:.0f} ns, "
      f"settle {get(SETTLE) / ADC_CLOCK_HZ * 1e9:.0f} ns, sample delay {get(SAMPLE_DELAY)}")

def zero_errors():
    v = get(ZERO_ERRORS)
    return v & 0xffff, v >> 16

if args.sweep:
    default_delay = get(SAMPLE_DELAY)
    set(CONTROL, 0x7001)          # positions 4, 5, 6: floating, ground, floating
    print("\ndelay  leading/trailing zero errors in 0.2 s   position 4   5   6")
    for delay in range(12):
        set(SAMPLE_DELAY, delay)
        time.sleep(0.02)
        before = zero_errors()
        time.sleep(0.2)
        after = zero_errors()
        values = [get(RESULTS + p) for p in (4, 5, 6)]
        print(f"  {delay:2d}   {(after[0] - before[0]) & 0xffff:6d} {(after[1] - before[1]) & 0xffff:6d}"
              f"                     {values[0]:5d} {values[1]:5d} {values[2]:5d}")
    set(SAMPLE_DELAY, default_delay)

all_rate = rate(0xFF01)
# with the default 2 us mux settling time about 470 kS/s over all positions
check("conversions over all positions", all_rate > 4e5, f"{all_rate / 1e3:.0f} kS/s, {all_rate / 8e3:.0f} kS/s per position")
single_rate = rate(0x0101)
check("conversions at one position", single_rate > 2.4e6, f"{single_rate / 1e3:.0f} kS/s back to back")

# readings of each position on its own
print("\nposition  mean      stdev   min    max     (14 bit counts)")
for position in range(8):
    set(CONTROL, (1 << (8 + position)) | 1)
    time.sleep(0.01)
    values = [get(RESULTS + position) for _ in range(args.readings)]
    print(f"   {position}     {statistics.mean(values):8.1f}  {statistics.pstdev(values):6.1f}  {min(values):5d}  {max(values):5d}")
before = zero_errors()

# all positions scanned: each result must land on its own position
set(CONTROL, 0xFF01)
time.sleep(0.01)
scan = [[get(RESULTS + p) for p in range(8)] for _ in range(args.readings // 4)]
print("\nscanning all positions, mean per position:")
print("   " + "  ".join(f"{p}: {statistics.mean(r[p] for r in scan):7.1f}" for p in range(8)))

after = zero_errors()
check("zero bits of the frames", after == before, f"leading/trailing errors during the readings {after[0] - before[0]}/{after[1] - before[1]}")

print('all passed' if failures == 0 else f'{failures} failed')
sys.exit(failures != 0)
