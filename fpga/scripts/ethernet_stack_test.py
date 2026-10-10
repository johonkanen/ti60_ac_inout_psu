#!/usr/bin/env python3
# ARP, ping and UDP echo test of the board_test build at 192.168.20.10.
#   python3 ethernet_stack_test.py <uart port> [--count N]
# Under WSL the ping and ARP checks use the Windows tools, UDP goes through Windows NAT.
import argparse
import os
import random
import socket
import subprocess
import sys
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from mdio_test import Mdio

BOARD_IP = '192.168.20.10'
ECHO_PORT = 5000
BASE = 500
COUNTERS = ['frames', 'arp replies', 'icmp replies', 'udp replies', 'ignored',
            'rx dropped busy', 'rx crc errors', 'rx other addresses']

parser = argparse.ArgumentParser()
parser.add_argument('port')
parser.add_argument('--count', type=int, default=2000, help='udp echo datagrams')
args = parser.parse_args()

mdio = Mdio(args.port)
uart = mdio.uart
failures = 0

def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<24} {detail}")
    failures += not ok

def counters():
    return {name: uart.request_data_from_address(BASE + i) for i, name in enumerate(COUNTERS)}

def windows(command):
    exe = f'/mnt/c/Windows/System32/{command[0]}.exe'
    if not os.path.exists(exe):
        return None
    return subprocess.run([exe] + command[1:], capture_output=True, text=True).stdout.replace('\r', '')

# link state from the PHY: BMSR link bit, 1000BASE-T status
mdio.read(0, 1)
bmsr = mdio.read(0, 1)
gstat = mdio.read(0, 10)
# 0xffff is the MDIO pull-up: the PHY did not answer
check("phy link", bmsr != 0xffff and bool(bmsr & 0x4), f"BMSR 0x{bmsr:04x}, 1000BASE-T status 0x{gstat:04x}, MIICTRL 0x{mdio.read(0, 0x17):04x}")
supervisor = uart.request_data_from_address(306)
print(f"      phy supervisor: configured {supervisor & 0xffff} times, {(supervisor >> 16) & 0x7fff} link ups, "
      f"link {'up' if supervisor >> 31 else 'down'}")
rx_pll_locked = bool(uart.request_data_from_address(400) & 0x2)
check("rx pll locked", rx_pll_locked)

before = counters()

windows(['arp', '-d', BOARD_IP])
output = windows(['ping', '-n', '4', '-w', '500', BOARD_IP])
if output is not None:
    replies = output.count(f'Reply from {BOARD_IP}')
    check("windows ping", replies == 4, f"{replies}/4 replies")
    arp_table = windows(['arp', '-a']) or ''
    entry = [l.split() for l in arp_table.splitlines() if l.strip().startswith(BOARD_IP + ' ')]
    check("windows arp entry", bool(entry) and entry[0][1] == '02-00-00-00-00-01', ' '.join(entry[0]) if entry else 'missing')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(0.5)
lost = wrong = 0
started = time.monotonic()
for i in range(args.count):
    length = random.choice([1, 17, 18, 64, 512, 1000, 1472]) if i else 1472
    data = random.randbytes(length)
    sock.sendto(data, (BOARD_IP, ECHO_PORT))
    try:
        reply, source = sock.recvfrom(2048)
        if reply != data or source != (BOARD_IP, ECHO_PORT):
            wrong += 1
    except socket.timeout:
        lost += 1
elapsed = time.monotonic() - started
check("udp echo", lost == 0 and wrong == 0, f"{args.count} datagrams in {elapsed:.1f} s, {lost} lost, {wrong} wrong")

time.sleep(0.2)
after = counters()
print("fpga counters during the test: " + ", ".join(f"{k} {(after[k] - before[k]) & 0xffffffff}" for k in COUNTERS))
print("ethernet stack OK" if failures == 0 else f"{failures} check(s) failed")
sys.exit(failures != 0)
