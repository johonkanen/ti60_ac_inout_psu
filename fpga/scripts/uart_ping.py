#!/usr/bin/env python3
# Board connectivity test for uart_test build.
#   python3 uart_ping.py <port> [baud] [clock_mhz]   e.g. /dev/ttyUSB10 5e6 125, defaults 5000000 and 120
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

port = sys.argv[1]
baud = int(float(sys.argv[2])) if len(sys.argv) > 2 else 5_000_000
clock_mhz = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0   # board_test runs at 125

uart = uart_link(port, baud, number_of_databytes=4)
failures = 0

def check(name, ok, detail):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<20} {detail}")
    failures += not ok

ident = uart.request_data_from_address(1)
check("id word", ident == 0xacdc, f"0x{ident:08x}")

t0 = time.monotonic()
c0 = uart.request_data_from_address(2)
time.sleep(1.0)
t1 = time.monotonic()
c1 = uart.request_data_from_address(2)
mhz = ((c1 - c0) & 0xffffffff) / (t1 - t0) / 1e6
check("main_clock", abs(mhz - clock_mhz) < clock_mhz/100, f"{mhz:.2f} MHz (expect {clock_mhz:g})")

for pattern in (0x12345678, 0xa5a5a5a5, 0x5a5a5a5a, 0):
    uart.write_data_to_address(3, pattern)
    back = uart.request_data_from_address(3)
    check("scratch r/w", back == pattern, f"wrote 0x{pattern:08x} read 0x{back:08x}")

locked = uart.request_data_from_address(4)
check("pll locked", locked == 1, str(locked))

print("board OK" if failures == 0 else f"{failures} check(s) failed")
sys.exit(failures != 0)
