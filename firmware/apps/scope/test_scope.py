#!/usr/bin/env python3
"""End-to-end check of the scope app from the network, standard library only.
    python3 test_scope.py [address]
Fetches the web client files, then talks to /ws like the browser does: hello, config,
run, and checks the captures: header, sample count, the trigger crossing at sample pre
on the sine channel, the generator's counter on channel 7 rising by one every sample
(every record in place, first to last), single and stop.
"""
import base64
import gzip
import json
import os
import socket
import struct
import sys
import time
import urllib.request

ADDRESS = sys.argv[1] if len(sys.argv) > 1 else '192.168.1.207'


class WebSocket:
    def __init__(self, host, path):
        self.sock = socket.create_connection((host, 80), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f'GET {path} HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n'
                           f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                           'Sec-WebSocket-Version: 13\r\n\r\n').encode())
        response = b''
        while b'\r\n\r\n' not in response:
            response += self.sock.recv(1024)
        status, self.buffer = response.split(b'\r\n\r\n', 1)
        assert b' 101 ' in status.split(b'\r\n')[0], status

    def _exact(self, n):
        while len(self.buffer) < n:
            data = self.sock.recv(65536)
            if not data:
                raise ConnectionError('closed')
            self.buffer += data
        out, self.buffer = self.buffer[:n], self.buffer[n:]
        return out

    def receive(self):
        """next message: str for text, bytes for binary"""
        message, opcode = b'', None
        while True:
            b0, b1 = self._exact(2)
            n = b1 & 0x7f
            if n == 126:
                n = struct.unpack('!H', self._exact(2))[0]
            elif n == 127:
                n = struct.unpack('!Q', self._exact(8))[0]
            opcode = opcode or b0 & 0x0f
            message += self._exact(n)
            if b0 & 0x80:
                return message.decode() if opcode == 1 else message

    def send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x81, 0x80 | n]) if n < 126 else bytes([0x81, 0x80 | 126]) + struct.pack('!H', n)
        self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def next_of(self, kind, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            m = self.receive()
            if kind == 'capture' and isinstance(m, bytes):
                return m
            if isinstance(m, str) and json.loads(m)['type'] == kind:
                return json.loads(m)
        raise TimeoutError(kind)


def parse_capture(message):
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    samples = struct.unpack(f'<{(len(message) - 4 - header_length) // 2}h', message[4 + header_length:])
    return header, samples


def main():
    for path in ['/', '/scope.js', '/scope.css', '/uPlot.iife.min.js', '/uPlot.min.css']:
        request = urllib.request.Request(f'http://{ADDRESS}{path}', headers={'Accept-Encoding': 'gzip'})
        with urllib.request.urlopen(request, timeout=5) as r:
            body = r.read()
            if r.headers.get('Content-Encoding') == 'gzip':
                body = gzip.decompress(body)
            print(f'GET {path:20} {r.status} {r.headers.get("Content-Type"):16} {len(body)} bytes')

    ws = WebSocket(ADDRESS, '/ws')
    hello = ws.next_of('hello')
    print('hello:', hello['config'], f'{len(hello["source_names"])} sources, clock {hello["clock_hz"]}')

    config = dict(hello['config'], divider=124, length=4096, pre_percent=25, mode='normal',
                  trigger_channel=0, trigger_edge=0, trigger_level=10000, generator_hz=1000.0,
                  sources=list(range(8)))
    ws.send({'type': 'config', 'config': config})
    ws.send({'type': 'run'})
    try:
        run_checks(ws, config)
    finally:
        ws.send({'type': 'stop'})
    print('ok')


def run_checks(ws, config):

    count, data, others, started = 0, 0, 0, time.monotonic()
    while time.monotonic() - started < 5:
        message = ws.next_of('capture')
        header, samples = parse_capture(message)
        n, pre = header['length'], header['pre']
        assert len(samples) == n * 8, (len(samples), n)
        if header['trigger_level'] != 10000:
            others += 1             # another client (a browser) changed the shared settings
            continue
        sine = samples[0::8]
        assert sine[pre - 1] < 10000 <= sine[pre], (sine[pre - 1], sine[pre])
        counter = samples[7::8]
        steps = [i for i in range(1, n) if (counter[i] - counter[i - 1]) & 0xffff != 1]
        assert not steps, f'the counter breaks at samples {steps[:5]}'
        assert not header['forced']
        count, data = count + 1, data + len(message)
    elapsed = time.monotonic() - started
    print(f'run: {count} captures of {n} samples in {elapsed:.1f} s, {count / elapsed:.1f}/s, '
          f'{data / elapsed / 1e6:.2f} MB/s, trigger at sample {pre} and the counter intact '
          f'on every one')
    if others:
        print(f'{others} captures had settings from another client')
    print(f'last header: {header}')

    ws.send({'type': 'stop'})
    time.sleep(0.3)
    ws.send({'type': 'config', 'config': dict(config, length=1024, mode='free')})
    ws.send({'type': 'single'})
    header, samples = parse_capture(ws.next_of('capture'))
    while header['length'] != 1024:     # sent before the stop
        header, samples = parse_capture(ws.next_of('capture'))
    assert header['length'] == 1024 and len(samples) == 1024 * 8
    status = ws.next_of('status')
    while status['running']:
        status = ws.next_of('status')
    print(f'single: {header["length"]} samples, then {status["state"]}')


if __name__ == '__main__':
    main()
