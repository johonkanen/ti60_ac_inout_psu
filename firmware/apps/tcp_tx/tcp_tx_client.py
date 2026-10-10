#!/usr/bin/env python3
"""Receive from tcp_tx for some seconds and print the rate.
    tcp_tx_client.py address [seconds] [receive buffer bytes]"""
import socket
import sys
import time

address = sys.argv[1]
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 10
s = socket.socket()
if len(sys.argv) > 3:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(sys.argv[3]))
s.connect((address, 5003))
total, start = 0, time.monotonic()
while time.monotonic() - start < seconds:
    data = s.recv(1 << 20)
    if not data:
        break
    total += len(data)
elapsed = time.monotonic() - start
s.close()
print(f'{total / elapsed / 1e6:.2f} MB/s ({total} bytes in {elapsed:.1f} s)')
