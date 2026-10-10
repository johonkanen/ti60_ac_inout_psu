#!/usr/bin/env python3
# Register bus over UDP port 5001, compared with the UART link.
#   python3 udp_bridge_test.py <uart port>
import os
import random
import sys
import threading
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mdio_test import Mdio
from udp_link import udp_link

ID, COUNTER, SCRATCH = 1, 2, 3
failures = 0

def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<30} {detail}")
    failures += not ok

uart = Mdio(sys.argv[1]).uart
udp = udp_link()

value = udp.request_data_from_address(ID)
check("id word over udp", value == 0xacdc, f"0x{value:08x}")

for _ in range(20):
    v = random.getrandbits(32)
    udp.write_data_to_address(SCRATCH, v)
    if uart.request_data_from_address(SCRATCH) != v:
        break
else:
    v = None
check("udp write, uart read", v is None)
for _ in range(20):
    v = random.getrandbits(32)
    uart.write_data_to_address(SCRATCH, v)
    # a uart write is not acknowledged and may sit in the USB serial adapter for a while,
    # a uart read after it returns only once the write has reached the FPGA
    uart.request_data_from_address(ID)
    got = udp.request_data_from_address(SCRATCH)
    if got != v:
        break
else:
    v = None
check("uart write, udp read", v is None, '' if v is None else f"wrote 0x{v:08x} read 0x{got:08x}")

results = udp.batch([('r', ID)] * 300)
check("batch of 300 reads", results == [0xacdc] * 300, f"{len(results)} results")

values = udp.batch([('w', SCRATCH, 0x11111111), ('r', SCRATCH), ('w', SCRATCH, 0x22222222), ('r', SCRATCH), ('r', ID)])
check("writes and reads in order", values == [0x11111111, 0x22222222, 0xacdc], str([hex(v) for v in values]))

stream = udp.stream_data_from_address(COUNTER, 367)
steps = [(b - a) & 0xffffffff for a, b in zip(stream, stream[1:])]
check("stream of the 125 MHz counter", len(stream) == 367 and all(0 < s < 1000 for s in steps),
      f"{len(stream)} values, {min(steps)}..{max(steps)} clocks between reads")

# throughput with full datagrams
words = 0
started = time.monotonic()
while time.monotonic() - started < 2.0:
    words += len(udp.stream_data_from_address(COUNTER, 367 * 8))
elapsed = time.monotonic() - started
print(f"      stream throughput {words / elapsed / 1e3:.0f} kwords/s = {words * 4 / elapsed / 1e6:.2f} MB/s, "
      f"single reads {1 / (lambda t: (udp.batch([('r', ID)] * 1), time.monotonic() - t)[1])(time.monotonic()) / 1e3:.1f} k/s")

# both links at the same time
uart_errors = []
def uart_traffic():
    for _ in range(200):
        if uart.request_data_from_address(ID) != 0xacdc:
            uart_errors.append(1)
thread = threading.Thread(target=uart_traffic)
thread.start()
udp_errors = 0
while thread.is_alive():
    udp_errors += sum(v != 0xacdc for v in udp.batch([('r', ID)] * 367))
thread.join()
check("uart and udp concurrently", not uart_errors and udp_errors == 0,
      f"{len(uart_errors)} uart errors, {udp_errors} udp errors, {udp.retransmissions} udp retransmissions")

print(f"fpga: bridge datagrams {uart.request_data_from_address(508)}, bus timeouts {uart.request_data_from_address(509)}")
print("udp bridge OK" if failures == 0 else f"{failures} check(s) failed")
sys.exit(failures != 0)
