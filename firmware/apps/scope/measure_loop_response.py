#!/usr/bin/env python3
"""The output voltage's closed loop response, for the waveform page's grid compensation.
    python3 measure_loop_response.py [address] [--amplitude V] [--orders 1,3,5,...] [--update] [--yes]

Plays a tone of each order of 50 Hz (amplitude V, 10 by default) through the waveform player:
a 20 ms table of 200 points, 20 passes, the closed loop on it after the DHB's precharge, at the
board's tuning as it is. A capture of the observer's voltage gives each tone's component, against
the points' at the same instants: the closed loop from the points to the output, of which the
control's prefilter is a known part (its two stages at control_prefilter_hz); the rest, the loop
after the prefilter, is what web/waveform.js's LOOP_RESPONSE holds. --update writes the measured
table there (then upload the pages: upload_web.sh); without it the table is only printed.

This runs the power stage: the gate driver supplies on, the DHB precharged and running, the output
bridge switching, each tone for 400 ms. It asks first unless --yes. The scope's settings, the
gate driver supplies and the DHB's voltage control are put back as they were after; the control's
tuning is not touched.
Needs numpy.
"""
import argparse
import base64
import cmath
import datetime
import json
import math
import os
import re
import socket
import struct
import sys
import textwrap
import time

import numpy as np

PERIODS_A_SECOND = 40000          # the output bridge's: 25 us
TABLE_PERIODS = 800               # a 20 ms table: 50 Hz
POINTS = 200                      # its points, 4 bridge periods each
PASSES = 20                       # 400 ms a tone
HERE = os.path.dirname(os.path.abspath(__file__))


class Board:
    """the board's /ws: JSON commands; its hello, statuses and captures as they come"""

    def __init__(self, host):
        self.sock = socket.create_connection((host, 80), timeout=5)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f'GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n'
                           f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                           'Sec-WebSocket-Version: 13\r\n\r\n').encode())
        response = b''
        while b'\r\n\r\n' not in response:
            response += self.sock.recv(1024)
        status, self.buffer = response.split(b'\r\n\r\n', 1)
        assert b' 101 ' in status.split(b'\r\n')[0], status
        self.sock.settimeout(0.2)
        self.hello, self.status, self.captures = None, {}, []

    def send(self, message):
        data = json.dumps(message).encode()
        mask = os.urandom(4)
        n = len(data)
        head = bytes([0x81, 0x80 | n]) if n < 126 else bytes([0x81, 0x80 | 126]) + struct.pack('!H', n)
        self.sock.sendall(head + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def _frame(self):
        def need(n):
            while len(self.buffer) < n:
                data = self.sock.recv(1 << 20)
                if not data:
                    raise ConnectionError('closed')
                self.buffer += data
        need(2)
        n, head = self.buffer[1] & 0x7f, 2
        if n == 126:
            need(4)
            n, head = struct.unpack('!H', self.buffer[2:4])[0], 4
        elif n == 127:
            need(10)
            n, head = struct.unpack('!Q', self.buffer[2:10])[0], 10
        need(head + n)
        opcode, data = self.buffer[0] & 0x0f, self.buffer[head:head + n]
        self.buffer = self.buffer[head + n:]
        return opcode, data

    def pump(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                opcode, data = self._frame()
            except socket.timeout:
                continue
            if opcode == 1:
                message = json.loads(data)
                if message['type'] == 'hello':
                    self.hello = message
                elif message['type'] == 'status':
                    self.status = message
            elif opcode == 2:
                self.captures.append(data)

    def poll(self, seconds=0.4):
        """a status asked for; the board's watchdog sees someone is there"""
        self.send({'type': 'heartbeat'})
        self.send({'type': 'config', 'config': {}})
        self.pump(seconds)
        return self.status


def prefilter(hz, prefilter_hz):
    """the control's prefilter at a frequency: two first order stages a bridge period each"""
    if not prefilter_hz > 0:
        return 1 + 0j
    a = 1 - math.exp(-2 * math.pi * prefilter_hz / PERIODS_A_SECOND)
    w = 2 * math.pi * hz / PERIODS_A_SECOND
    h1 = a / (1 - (1 - a) * cmath.exp(-1j * w))
    return h1 * h1


def tone(board, order, amplitude, sources):
    """a tone of order times 50 Hz played; the closed loop's response at it (complex)"""
    points = [[round(amplitude * math.sin(2 * math.pi * order * (k + 0.5) / POINTS), 4),
               TABLE_PERIODS // POINTS] for k in range(POINTS)]
    board.send({'type': 'waveform_load', 'points': points, 'passes': PASSES})
    board.poll()
    # a 131 ms capture at 250 kS/s from the DHB's start, as the output's
    board.send({'type': 'config', 'config': {'divider': 499, 'length': 32768, 'pre_percent': 5,
                                             'mode': 'normal', 'trigger_channel': 6, 'trigger_edge': 0,
                                             'trigger_level': 8, 'sources': sources}})
    board.pump(0.3)
    board.send({'type': 'single'})
    board.pump(0.3)
    board.captures.clear()
    board.send({'type': 'waveform_run', 'periods': TABLE_PERIODS * PASSES + 10})
    board.send({'type': 'dhb_test', 'periods': 75000, 'pulse': 40, 'ramp': 124, 'run': 100 * 150, 'invert': 1})
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not board.captures:
        board.poll(0.3)
    for _ in range(4):       # the run over: 400 ms and the ramp down
        status = board.poll(0.3)
    if status.get('dhb_trip', [1])[0]:
        sys.exit(f'tripped at {order * 50} Hz: {status.get("dhb_trip")}')
    if not board.captures:
        sys.exit(f'no capture at {order * 50} Hz')
    message = board.captures[-1]
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n, fs = header['length'], header['sample_rate']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    output = x[:, 7].astype(int)
    running = np.flatnonzero(output & 1)
    if not len(running):
        sys.exit(f'the output did not run at {order * 50} Hz')
    r0 = running[0]
    observer_v = x[:, 0] / 64
    # whole 20 ms periods from the second on, the points at the same instants
    period = int(round(fs / 50))
    start = r0 + period
    periods = (n - start) // period
    if periods < 2:
        sys.exit('too short a capture')
    sig = observer_v[start:start + periods * period]
    t = np.arange(len(sig)) / fs + (start - r0) / fs
    steps = np.repeat([v for v, _ in points], TABLE_PERIODS // POINTS)
    reference = steps[np.floor(t * PERIODS_A_SECOND).astype(int) % TABLE_PERIODS]
    e = np.exp(-2j * np.pi * order * 50 * t)
    return np.mean(sig * e) / np.mean(reference * e)


def update_waveform_js(rows, config, path):
    text = open(path).read()
    tuning = (f"voltage loop {config['control_voltage_hz']:g} Hz, its zero {config['control_voltage_zero']:g}, "
              f"b {config['control_b']:g}, current loop {config['control_current_hz']:g} Hz, prefilter "
              f"{config['control_prefilter_hz']:g} Hz, feedforward {config['control_feedforward']:g}")
    comment = ("the closed loop after the prefilter, from the points to the output voltage (the "
               f"observer's), measured on the board on {datetime.date.today()} by measure_loop_response.py "
               f"with tones at its tuning then ({tuning}): [Hz, gain, phase in degrees]; between the tones "
               "interpolated, beyond them the nearest's")
    lines = textwrap.wrap(comment, 96)
    entries = [f'[{hz}, {gain:.3f}, {phase:.1f}]' for hz, gain, phase in rows]
    table, line = [], 'const LOOP_RESPONSE = ['
    for i, entry in enumerate(entries):
        piece = entry + (', ' if i < len(entries) - 1 else '];')
        if len(line) + len(piece) > 100:
            table.append(line.rstrip())
            line = ' ' * len('const LOOP_RESPONSE = [')
        line += piece
    table.append(line)
    block = '\n'.join(['// ' + l for l in lines] + table)
    new, count = re.subn(r'// the closed loop after the prefilter, from the points[\s\S]*?const LOOP_RESPONSE = \[[\s\S]*?\];',
                         lambda _: block, text)
    if count != 1:
        sys.exit(f'LOOP_RESPONSE not found in {path}')
    open(path, 'w').write(new)
    print(f'{path}: LOOP_RESPONSE written; upload the pages (upload_web.sh)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('address', nargs='?', default='192.168.20.11')
    parser.add_argument('--amplitude', type=float, default=10.0, help='the tones\' amplitude, V')
    parser.add_argument('--orders', default='1,3,5,7,9,11,13', help='orders of 50 Hz')
    parser.add_argument('--update', action='store_true', help='write the table into web/waveform.js')
    parser.add_argument('--yes', action='store_true', help='do not ask before running the power stage')
    args = parser.parse_args()
    orders = [int(o) for o in args.orders.split(',')]

    board = Board(args.address)
    board.pump(1.0)
    if not board.hello or not board.hello.get('waveform'):
        sys.exit('the board has no waveform player')
    config = board.hello['config']
    names = board.hello['source_names']
    channels = ['observer v', 'sec 2 vfilt_out', 'sdm iac_out mean', 'control i_ref', 'control u_ref',
                'observer i', 'dhb test', 'output test']
    sources = [names.index(c) for c in channels]
    if not args.yes:
        answer = input(f'Run the power stage: {len(orders)} tones of {args.amplitude:g} V, 400 ms each, '
                       f'the closed loop at its tuning now? [y/N] ')
        if answer.strip().lower() != 'y':
            sys.exit('not run')

    keep = {k: config[k] for k in ['divider', 'length', 'pre_percent', 'mode', 'trigger_channel', 'trigger_edge',
                                   'trigger_level', 'sources', 'dhb_control', 'gate_power']}
    board.send({'type': 'config', 'config': {'gate_power': 63, 'dhb_control': 1, 'observer': 1, 'control': 1}})
    for _ in range(20):
        if board.poll().get('gate_power_on') == 63:
            break
    else:
        sys.exit('the gate driver supplies did not come on')
    board.send({'type': 'dhb_clear_trip'})
    board.poll()
    rows = []
    try:
        print(f'{"Hz":>5} {"closed loop":>20} {"prefilter":>20} {"the loop after it":>20}')
        for order in orders:
            hz = order * 50
            h = tone(board, order, args.amplitude, sources)
            p = prefilter(hz, config['control_prefilter_hz'])
            loop = h / p
            rows.append((hz, abs(loop), math.degrees(cmath.phase(loop))))
            print(f'{hz:5d} {abs(h):8.3f} {math.degrees(cmath.phase(h)):8.1f} deg '
                  f'{abs(p):8.3f} {math.degrees(cmath.phase(p)):8.1f} deg '
                  f'{abs(loop):8.3f} {math.degrees(cmath.phase(loop)):8.1f} deg', flush=True)
    finally:
        board.send({'type': 'waveform_stop'})
        board.send({'type': 'config', 'config': keep})
        board.poll()
    if args.update:
        update_waveform_js(rows, config, os.path.join(HERE, 'web', 'waveform.js'))
    else:
        print('LOOP_RESPONSE = [' + ', '.join(f'[{hz}, {g:.3f}, {ph:.1f}]' for hz, g, ph in rows) + ']')


if __name__ == '__main__':
    main()
