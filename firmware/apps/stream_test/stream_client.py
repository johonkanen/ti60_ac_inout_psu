#!/usr/bin/env python3
"""Receive apps/stream_test's stream on port 5004 and check it.

stream_client.py address [seconds] [--raw]

As a WebSocket (the app's default): the handshake's accept key, every frame header (binary
opcode on the first of each 720 frame message, continuation after, FIN on the last, 1456
byte payloads) and the data: word w (32 bits, little endian) of the joined payloads is
w mod 16380, checked on a sample of the words. --raw for the app built without
CONFIG_STREAM_TEST_WEBSOCKET: the same data straight on TCP."""
import base64
import hashlib
import os
import socket
import struct
import sys
import time

args = [a for a in sys.argv[1:] if not a.startswith('--')]
address = args[0] if args else '192.168.20.11'
seconds = float(args[1]) if len(args) > 1 else 10
raw = '--raw' in sys.argv
PERIOD, PAYLOAD, MESSAGE = 16380, 1456, 720

s = socket.create_connection((address, 5004), timeout=5)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
pending = b''
if not raw:
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f'GET / HTTP/1.1\r\nHost: {address}:5004\r\nUpgrade: websocket\r\n'
               f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
               f'Sec-WebSocket-Version: 13\r\n\r\n').encode())
    while b'\r\n\r\n' not in pending:
        pending += s.recv(4096)
    reply, pending = pending.split(b'\r\n\r\n', 1)
    accept = base64.b64encode(hashlib.sha1(
        (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
    assert reply.startswith(b'HTTP/1.1 101'), reply
    assert f'Sec-WebSocket-Accept: {accept}'.encode() in reply, reply
    print('handshake ok')

received, errors, frames, messages = 0, 0, 0, 0
words_seen = 0          # data words checked so far, the position in the pattern
data = b''              # data not yet a whole number of words
start = time.monotonic()


def check(payload):
    global data, words_seen, errors
    data += payload
    n = len(data) // 4
    words = struct.unpack_from(f'<{n}I', data)
    data = data[4 * n:]
    for i in range(0, n, 97):       # checking every word is slower than the link
        if words[i] != (words_seen + i) % PERIOD:
            errors += 1
            if errors < 5:
                print(f'word {words_seen + i}: {words[i]}, expected {(words_seen + i) % PERIOD}')
    words_seen += n


while time.monotonic() - start < seconds:
    chunk = s.recv(1 << 20)
    if not chunk:
        break
    received += len(chunk)
    if raw:
        check(chunk)
        continue
    pending += chunk
    view, at, payloads = memoryview(pending), 0, []
    while len(pending) - at >= 4 + PAYLOAD:
        b0, b1, length = struct.unpack_from('>BBH', pending, at)
        place = frames % MESSAGE
        expected = (0x80 if place == MESSAGE - 1 else 0) | (0x2 if place == 0 else 0x0)
        if b0 != expected or b1 != 126 or length != PAYLOAD:
            errors += 1
            if errors < 5:
                print(f'frame {frames}: header {b0:#x} {b1} {length}, expected {expected:#x} 126 {PAYLOAD}')
        payloads.append(view[at + 4:at + 4 + PAYLOAD])
        at += 4 + PAYLOAD
        frames += 1
        messages += place == MESSAGE - 1
    check(b''.join(payloads))
    pending = pending[at:]
elapsed = time.monotonic() - start
s.close()
print(f'{received / elapsed / 1e6:.2f} MB/s ({received} bytes in {elapsed:.1f} s), '
      + ('' if raw else f'{frames} frames, {messages} messages, ') + f'{errors} errors')
