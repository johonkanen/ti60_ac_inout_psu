#!/usr/bin/env python3
"""The scope's stream (port 5004) end to end, on the PSU board's FPGA capture.

test_stream.py [address] [seconds] [divider ...]

For each divider: sets the sample rate (a sample every divider+1 clocks of 125 MHz) with
channel 0 on the test generator's counter through /ws, takes the stream for the given
seconds and checks the WebSocket frames (binary then continuation, FIN every 720 frames)
and that the counter goes up by one from every sample to the next, across frames and
messages: no sample lost, none twice. Prints the rate the stream kept. A sample rate the
link cannot keep up with ends in a reset from the board (an overrun)."""
import base64
import hashlib
import os
import socket
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_scope  # noqa: E402  the /ws client

args = sys.argv[1:]
address = args[0] if args else '192.168.20.11'
seconds = float(args[1]) if len(args) > 1 else 5
dividers = [int(d) for d in args[2:]] or [124, 49, 24]
PAYLOAD, MESSAGE, CHANNELS = 1456, 720, 8
FRAME = 4 + PAYLOAD


def stream(divider):
    control = test_scope.WebSocket(address, '/ws')
    hello = control.next_of('hello')
    control.send({'type': 'config', 'config': dict(hello['config'], divider=divider,
                                                   sources=[7, 8, 9, 10, 11, 12, 13, 14])})
    time.sleep(0.3)

    s = socket.create_connection((address, 5004), timeout=5)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f'GET / HTTP/1.1\r\nHost: {address}:5004\r\nUpgrade: websocket\r\n'
               f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
               f'Sec-WebSocket-Version: 13\r\n\r\n').encode())
    pending = b''
    while b'\r\n\r\n' not in pending:
        pending += s.recv(4096)
    reply, pending = pending.split(b'\r\n\r\n', 1)
    accept = base64.b64encode(hashlib.sha1(
        (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
    assert reply.startswith(b'HTTP/1.1 101') and accept.encode() in reply, reply

    frames, errors, samples, last = 0, 0, 0, None
    received, start, reset = 0, time.monotonic(), False
    while time.monotonic() - start < seconds:
        try:
            chunk = s.recv(1 << 20)
        except ConnectionResetError:
            reset = True        # the board's answer to an overrun
            break
        if not chunk:
            break
        received += len(chunk)
        pending += chunk
        n = len(pending) // FRAME
        if n == 0:
            continue
        buf = np.frombuffer(pending, dtype=np.uint8, count=n * FRAME).reshape(n, FRAME)
        place = (frames + np.arange(n)) % MESSAGE
        expected = np.where(place == MESSAGE - 1, 0x80, 0) | np.where(place == 0, 0x2, 0)
        bad = (buf[:, 0] != expected) | (buf[:, 1] != 126) | (buf[:, 2] != PAYLOAD >> 8) \
            | (buf[:, 3] != PAYLOAD & 0xff)
        if bad.any():
            errors += int(bad.sum())
            print(f'  frame {frames + int(np.argmax(bad))}: bad header {bytes(buf[np.argmax(bad), :4])}')
        counter = np.ascontiguousarray(buf[:, 4:]).view('<u2').reshape(-1, CHANNELS)[:, 0]
        if last is not None:
            counter = np.concatenate(([last], counter))
        steps = (np.diff(counter.astype(np.int32)) & 0xffff) != 1
        if steps.any():
            errors += int(steps.sum())
            k = int(np.argmax(steps))
            print(f'  sample {samples + k}: counter {counter[k + 1]} after {counter[k]}')
        last = int(counter[-1])
        samples += n * (PAYLOAD // 16)
        frames += n
        pending = pending[n * FRAME:]
    elapsed = time.monotonic() - start
    s.close()
    time.sleep(0.5)
    control.next_of('status', timeout=5)
    control.sock.close()
    time.sleep(1)       # the app frees the control client's slot
    rate = 125e6 / (divider + 1)
    print(f'divider {divider}: {rate / 1e6:.3f} MS/s, {16 * rate / 1e6:.1f} MB/s produced; '
          f'received {received / elapsed / 1e6:.1f} MB/s, {samples} samples in {frames} frames, '
          f'{errors} errors' + (', reset by the board after an overrun' if reset else ''))
    return errors + reset


failures = sum(stream(d) for d in dividers)
sys.exit(1 if failures else 0)
