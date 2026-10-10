#!/usr/bin/env python3
# RGMII test through PEF7071 PHY loopback, for the board_test build.
#   python3 ethernet_loopback_test.py <port> [--frames N] [--length L] [--rx_skew ns] [--tx_skew ns]
import argparse
import os
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mdio_test import Mdio

BASE = 400
COMMAND, FRAME_COUNT, PAYLOAD_LENGTH, OPTIONS, FRAMES_SENT, RX_FRAMES, RX_GOOD, RX_CRC_ERRORS, \
    RX_PAYLOAD_ERRORS, RX_GAPS, RX_LENGTH, RX_ALIGNMENT, RX_CYCLES = range(13)
PHY = 0
BMCR, BMSR, MIICTRL = 0, 1, 0x17

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--frames', type=int, default=1000)
parser.add_argument('--length', type=int, default=1500, help='payload bytes, 46..1500')
parser.add_argument('--rx_skew', type=float, default=2.0, help='PHY RX clock skew in ns, 0..3.5, board passes 0.5..3.0')
parser.add_argument('--tx_skew', type=float, default=2.5, help='PHY TX clock skew in ns, 0..3.5, board passes 2.0..3.0')
parser.add_argument('--delay_lo', action='store_true', help='delay the falling edge tx nibble by a clock')
parser.add_argument('--keep_loopback', action='store_true', help='leave the PHY in loopback at exit')
args = parser.parse_args()

mdio = Mdio(args.port)
uart = mdio.uart
get = lambda offset: uart.request_data_from_address(BASE + offset)
set = lambda offset, value: uart.write_data_to_address(BASE + offset, value)

print(f"PHY id 0x{mdio.read(PHY, 2):04x} 0x{mdio.read(PHY, 3):04x}, "
      f"BMCR 0x{mdio.read(PHY, BMCR):04x}, MIICTRL 0x{mdio.read(PHY, MIICTRL):04x}")

# MIICTRL: bit15 rx clock kept running, 14:12 rx skew, 11 0 = 3.3 V, 10:8 tx skew, 0.5 ns steps
miictrl = 0x8000 | (round(args.rx_skew * 2) & 7) << 12 | (round(args.tx_skew * 2) & 7) << 8
mdio.write(PHY, MIICTRL, miictrl)
# BMCR: loopback, forced 1000 Mb/s full duplex, auto negotiation off
mdio.write(PHY, BMCR, 0x4140)
time.sleep(0.1)
print(f"MIICTRL 0x{mdio.read(PHY, MIICTRL):04x} (rx skew {args.rx_skew} ns, tx skew {args.tx_skew} ns), "
      f"BMCR 0x{mdio.read(PHY, BMCR):04x}")

# restart the receive PLL now that RX_CLK follows the loopback
options = 0x1 if args.delay_lo else 0x0
set(OPTIONS, options | 0x2)
time.sleep(0.01)
set(OPTIONS, options)
time.sleep(0.05)
locked = bool(get(COMMAND) & 0x2)
c0, t0 = get(RX_CYCLES), time.monotonic()
time.sleep(0.5)
c1, t1 = get(RX_CYCLES), time.monotonic()
print(f"rx pll {'locked' if locked else 'NOT locked'}, rx clock {((c1 - c0) & 0xffffffff) / (t1 - t0) / 1e6:.2f} MHz")

set(COMMAND, 4)  # clear receive counters
time.sleep(0.01)
set(FRAME_COUNT, args.frames)
set(PAYLOAD_LENGTH, args.length)
sent_before = get(FRAMES_SENT)
set(COMMAND, 1)
while get(COMMAND) & 0x1:
    time.sleep(0.05)
time.sleep(0.05)

sent = (get(FRAMES_SENT) - sent_before) & 0xffffffff
received, good = get(RX_FRAMES), get(RX_GOOD)
crc_errors, payload_errors, gaps = get(RX_CRC_ERRORS), get(RX_PAYLOAD_ERRORS), get(RX_GAPS)
print(f"sent {sent}, received {received}, good {good}, crc errors {crc_errors}, payload errors {payload_errors}, "
      f"sequence gaps {gaps}, last length {get(RX_LENGTH)} (expect {14 + args.length + 4}), "
      f"nibble alignment {get(RX_ALIGNMENT)}")
ok = locked and sent == args.frames and received == sent and good == sent and gaps == 0

if not args.keep_loopback:
    mdio.write(PHY, BMCR, 0x1240)  # auto negotiation on and restarted, loopback off

print("loopback OK" if ok else "loopback FAILED")
sys.exit(not ok)
