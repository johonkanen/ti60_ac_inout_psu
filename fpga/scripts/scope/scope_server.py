#!/usr/bin/env python3
"""Oscilloscope server: talks UDP to the FPGA and serves the uPlot web client.

    python3 scope_server.py [--board 192.168.20.10] [--http-port 8080]

then open http://localhost:8080 in a browser. Needs numpy.

FPGA side
    registers over UDP port 5001 (UART command encoding, see scripts/udp_link.py)
    capture readout over UDP port 5002 (see source/ethernet/udp_stream_pkg.vhd), from the
    block RAM or, for deep captures, from the HyperRAM (bit 31 of the first word set)
Browser side
    WebSocket /ws, JSON commands from the browser, JSON status and binary captures to it.
    A capture message is: uint32 header length, JSON header, float64 times of the points,
    int32 values of the points, point-major (all channels of point 0, then point 1, ...),
    all little endian. Values follow the probe format of the channel's source (signed,
    unsigned or digital lanes). Long captures are sent as an envelope of the viewed range,
    two points per bin: minimum and maximum, for digital sources the AND and the OR of the
    lanes.
    The source names and formats come from the probe table of the FPGA. The browser asks for a range with a
    'view' command, the whole capture stays in the server.
"""
import argparse
import asyncio
import base64
import hashlib
import json
import os
import struct
import socket
import time

import numpy as np

SCOPE_BASE = 600
COMMAND, DIVIDER, TRIGGER, LEVEL, PRE_SAMPLES, LENGTH, SOURCES, PHASE, INFO, START, \
    MODE, OVERFLOWS, START_RECORD, RING_RECORDS, SOURCES_HI, PATTERN, PROBE_INDEX = range(17)
PROBE_NAME, PROBE_FORMAT = 17, 21
SIGNED, UNSIGNED, DIGITAL = 0, 1, 2
STATE_NAMES = ['idle', 'pre-trigger', 'waiting', 'post-trigger', 'done']
CLOCK_HZ = 125e6
CHANNELS = 8
WORDS_PER_SAMPLE = 4
BLOCK_RAM_SAMPLES = 4096
DEEP_SAMPLES = 2**22
HYPERRAM_STREAM = 0x80000000      # stream word address bit: read the HyperRAM
DISPLAY_POINTS = 8192             # longer views are sent as a min/max envelope
ENVELOPE_BINS = 2000

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')


# ---------------------------------------------------------------------------- FPGA access
class UdpEndpoint(asyncio.DatagramProtocol):
    def __init__(self):
        self.queue = asyncio.Queue()

    def datagram_received(self, data, address):
        self.queue.put_nowait(data)


async def open_endpoint(loop, address, receive_buffer=None):
    transport, protocol = await loop.create_datagram_endpoint(UdpEndpoint, remote_addr=address)
    if receive_buffer:
        sock = transport.get_extra_info('socket')
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, receive_buffer)
    return transport, protocol


class Registers:
    """register bus over UDP port 5001, same encoding as udp_link.py"""

    def __init__(self, transport, protocol):
        self.transport, self.protocol = transport, protocol
        self.sequence = 0
        self.lock = asyncio.Lock()

    async def batch(self, commands, timeout=0.2, retries=5):
        payload, expected = b'', 0
        for c in commands:
            if c[0] == 'r':
                payload += struct.pack('!BH', 2, c[1]); expected += 1
            else:
                payload += struct.pack('!BHI', 4, c[1], c[2] & 0xffffffff)
        async with self.lock:
            self.sequence = (self.sequence + 1) & 0xffff
            request = struct.pack('!H', self.sequence) + payload
            for _ in range(retries):
                while not self.protocol.queue.empty():
                    self.protocol.queue.get_nowait()
                self.transport.sendto(request)
                try:
                    while True:
                        reply = await asyncio.wait_for(self.protocol.queue.get(), timeout)
                        if struct.unpack('!H', reply[:2])[0] == self.sequence:
                            break
                except asyncio.TimeoutError:
                    continue
                return list(struct.unpack(f'!{(len(reply) - 2) // 4}I', reply[2:]))[:expected]
        raise TimeoutError('no reply from the register bridge')

    async def read(self, offset):
        return (await self.batch([('r', SCOPE_BASE + offset)]))[0]

    async def write(self, offset, value):
        await self.batch([('w', SCOPE_BASE + offset, value)])


class StreamReader:
    """capture download over UDP port 5002, lost datagrams are requested again"""

    def __init__(self, transport, protocol):
        self.transport, self.protocol = transport, protocol
        self.request_id = 0
        self.pacing = 0
        self.lost = 0

    def request(self, first_word, count):
        self.request_id = (self.request_id + 1) & 0xffff
        self.transport.sendto(struct.pack('!HHII', self.request_id, self.pacing, first_word, count))
        return self.request_id

    async def read_words(self, first_word, count, into, timeout=0.05, retries=8):
        """count stream words from first_word into the bytes of into, big endian as sent"""
        base = first_word & 0x7fffffff
        flag = first_word & HYPERRAM_STREAM
        received = np.zeros(count, dtype=bool)
        missing = (0, count)
        for attempt in range(retries):
            # a new request replaces a running one in the FPGA, so one request at a time
            request_id = self.request(flag | (base + missing[0]), missing[1])
            got = int(received.sum())
            deadline = time.monotonic() + timeout + missing[1] / 20e6
            while got < count:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                try:
                    data = await asyncio.wait_for(self.protocol.queue.get(), left)
                except asyncio.TimeoutError:
                    break
                if len(data) < 12:
                    continue
                rid, flags, first, n = struct.unpack('!HHIH', data[:10])
                first = (first & 0x7fffffff) - base
                if rid != request_id or first < 0 or first + n > count or len(data) < 12 + 4 * n:
                    continue
                into[4 * first:4 * (first + n)] = np.frombuffer(data, dtype=np.uint8, count=4 * n, offset=12)
                got += n - int(received[first:first + n].sum())
                received[first:first + n] = True
            if got == count:
                if attempt == 0:
                    self.pacing = max(0, self.pacing - 1)   # speed up again after clean downloads
                return
            # request again from the first to the last missing word
            holes = np.flatnonzero(~received)
            missing = (int(holes[0]), int(holes[-1] - holes[0] + 1))
            self.lost += len(holes)
            self.pacing = min(self.pacing + 2, 64)   # slow down when datagrams are lost
        raise TimeoutError(f'capture download incomplete after {retries} tries')


# ---------------------------------------------------------------------------- acquisition
class Scope:
    def __init__(self, registers, stream):
        self.registers, self.stream = registers, stream
        self.config = {
            'divider': 124,                  # 1 MS/s
            'length': 4096,
            'pre_percent': 50,
            'trigger_channel': 0,
            'trigger_edge': 0,               # 0 rising, 1 falling, 2 either, 3 pattern
            'trigger_mask': 0xffff,          # pattern trigger on digital sources
            'trigger_pattern': 1,
            'trigger_level': 0,
            'mode': 'auto',                  # auto, normal, single, free
            'deep': False,                   # capture into the HyperRAM
            'sources': list(range(8)),
            'generator_hz': 10000.0,
        }
        self.running = False
        self.single = False
        self.clients = set()
        self.status = {'state': 'stopped', 'captures': 0, 'forced': False, 'error': ''}
        self.config_changed = True
        self.force_now = False
        self.samples = None                  # last capture, int16 (length, CHANNELS)
        self.probes = []                     # [{'name', 'format'}] per source, from the FPGA
        self.view = None                     # viewed sample range, None for all
        self.capture_info = {}

    async def read_probe_table(self):
        """names and formats of all sources from the FPGA's probe table"""
        count = (await self.registers.read(INFO)) >> 24
        probes = []
        for source in range(count):
            values = await self.registers.batch(
                [('w', SCOPE_BASE + PROBE_INDEX, source)] + [('r', SCOPE_BASE + PROBE_NAME + i) for i in range(4)]
                + [('r', SCOPE_BASE + PROBE_FORMAT)])
            name = b''.join(v.to_bytes(4, 'big') for v in values[:4]).split(b'\0')[0].decode(errors='replace')
            probes.append({'name': name or f'source {source}', 'format': values[4]})
        self.probes = probes

    def format_of(self, source):
        return self.probes[source]['format'] if source < len(self.probes) else SIGNED

    def channel_values(self, samples):
        """int16 samples as int32 values in the format of each channel's source"""
        values = samples.astype(np.int32)
        for c, source in enumerate(self.capture_info['sources']):
            if self.format_of(source) != SIGNED:
                values[:, c] &= 0xffff
        return values

    async def apply_config(self):
        c = self.config
        deep = bool(c.get('deep'))
        length = max(16, min(DEEP_SAMPLES if deep else BLOCK_RAM_SAMPLES, int(c['length'])))
        pre = int(length * max(0, min(100, c['pre_percent'])) / 100)
        pre = min(pre, length - 1)
        sources = [int(s) & 255 for s in c['sources']]
        sample_rate = CLOCK_HZ / (int(c['divider']) + 1)
        phase = int(c['generator_hz'] / sample_rate * 2**32) & 0xffffffff
        trigger = (int(c['trigger_channel']) & 7) | ((int(c['trigger_edge']) & 3) << 4)
        if c['mode'] != 'free':
            trigger |= 0x100
        if self.format_of(sources[int(c['trigger_channel']) & 7]) == UNSIGNED:
            trigger |= 0x200                                     # compare the level unsigned
        await self.registers.batch([
            ('w', SCOPE_BASE + COMMAND, 4),                       # stop a running capture
            ('w', SCOPE_BASE + MODE, 1 if deep else 0),            ('w', SCOPE_BASE + DIVIDER, int(c['divider'])),
            ('w', SCOPE_BASE + TRIGGER, trigger),
            ('w', SCOPE_BASE + LEVEL, int(c['trigger_level']) & 0xffff),
            ('w', SCOPE_BASE + PRE_SAMPLES, pre),
            ('w', SCOPE_BASE + LENGTH, length),
            ('w', SCOPE_BASE + SOURCES, sum(s << (8 * i) for i, s in enumerate(sources[:4]))),
            ('w', SCOPE_BASE + SOURCES_HI, sum(s << (8 * i) for i, s in enumerate(sources[4:]))),
            ('w', SCOPE_BASE + PATTERN, (int(c['trigger_mask']) & 0xffff) << 16 | (int(c['trigger_pattern']) & 0xffff)),
            ('w', SCOPE_BASE + PHASE, phase)])
        self.length, self.pre, self.sample_rate, self.deep = length, pre, sample_rate, deep
        self.config_changed = False

    async def capture(self):
        """one capture: arm, wait for the trigger (forced in auto mode), download"""
        await self.registers.write(COMMAND, 1)
        record_time = self.length / self.sample_rate
        auto_timeout = max(0.1, 2 * record_time)
        started = time.monotonic()
        forced = False
        while True:
            status = await self.registers.read(COMMAND)
            state = status & 7
            if state == 4:
                break
            if self.config_changed or not self.running:
                return None
            elapsed = time.monotonic() - started
            if not forced and (self.force_now or (self.config['mode'] == 'auto' and state == 2 and elapsed > auto_timeout)):
                await self.registers.write(COMMAND, 2)
                forced, self.force_now = True, False
            self.status['state'] = STATE_NAMES[state] if state < 5 else '?'
            await asyncio.sleep(min(0.01, record_time / 4) if elapsed < 0.2 else 0.02)
        self.status['state'] = 'downloading'
        await self.broadcast_status()
        started = time.monotonic()
        # the words are sent big endian: channel 0 high byte first, so the bytes read as
        # big endian int16 are the channels in order
        raw = np.empty(self.length * WORDS_PER_SAMPLE * 4, dtype=np.uint8)
        if self.deep:
            overflows, start, ring = await self.registers.batch([
                ('r', SCOPE_BASE + OVERFLOWS), ('r', SCOPE_BASE + START_RECORD), ('r', SCOPE_BASE + RING_RECORDS)])
            self.status['overflows'] = overflows
            # the capture may wrap around the end of the ring
            first = min(self.length, ring - start)
            await self.stream.read_words(HYPERRAM_STREAM | WORDS_PER_SAMPLE * start, WORDS_PER_SAMPLE * first,
                                         raw[:16 * first])
            if first < self.length:
                await self.stream.read_words(HYPERRAM_STREAM, WORDS_PER_SAMPLE * (self.length - first),
                                             raw[16 * first:])
        else:
            self.status['overflows'] = 0
            await self.stream.read_words(0, self.length * WORDS_PER_SAMPLE, raw)
        seconds = time.monotonic() - started
        self.status['download'] = f'{len(raw) / 1e6:.1f} MB in {seconds:.2f} s'
        samples = raw.view('>i2').astype(np.int16).reshape(self.length, CHANNELS)
        self.status.update(state='done', captures=(status >> 16), forced=bool(status & 0x100))
        return samples

    async def run(self):
        display_interval = 1 / 30          # a browser redraws at most this often anyway
        last_publish = 0.0
        while True:
            if not self.running:
                await asyncio.sleep(0.05)
                continue
            try:
                if self.config_changed:
                    await self.apply_config()
                wait = last_publish + display_interval - time.monotonic()
                if wait > 0:
                    await asyncio.sleep(wait)
                samples = await self.capture()
                if samples is not None:
                    await self.publish(samples)
                    last_publish = time.monotonic()
                    if self.single:
                        self.running = False
                        self.status['state'] = 'stopped'
                self.status['error'] = ''
            except (TimeoutError, OSError) as e:
                self.status['error'] = str(e)
                await asyncio.sleep(0.5)
            await self.broadcast_status()

    async def publish(self, samples):
        """send the viewed range of a new capture"""
        self.samples = samples
        self.capture_info = {
            'length': self.length, 'pre': self.pre, 'sample_rate': self.sample_rate, 'deep': self.deep,
            'sources': list(self.config['sources']), 'forced': self.status['forced'],
            'capture': self.status['captures'], 'trigger_channel': self.config['trigger_channel'],
            'trigger_level': self.config['trigger_level']}
        await self.send_view()

    def view_message(self):
        samples, info = self.samples, self.capture_info
        length = len(samples)
        first, last = self.view if self.view else (0, length)
        first, last = max(0, min(first, length - 1)), max(1, min(last, length))
        if last - first < 2:
            first, last = max(0, first - 1), min(length, first + 2)
        rate, pre = info['sample_rate'], info['pre']
        part = self.channel_values(samples[first:last])
        digital = [self.format_of(s) == DIGITAL for s in info['sources']]
        if len(part) <= DISPLAY_POINTS:
            times = (np.arange(first, last) - pre) / rate
            points = part
            envelope = False
        else:
            # min and max of each bin, at the start and the middle of the bin
            edges = np.linspace(0, len(part), ENVELOPE_BINS + 1).astype(np.int64)
            minima = np.minimum.reduceat(part, edges[:-1], axis=0)
            maxima = np.maximum.reduceat(part, edges[:-1], axis=0)
            for c in range(CHANNELS):
                if digital[c]:
                    # lanes low in the whole bin, lanes high somewhere in it
                    minima[:, c] = np.bitwise_and.reduceat(part[:, c], edges[:-1])
                    maxima[:, c] = np.bitwise_or.reduceat(part[:, c], edges[:-1])
            points = np.empty((2 * ENVELOPE_BINS, CHANNELS), dtype=np.int32)
            points[0::2], points[1::2] = minima, maxima
            starts = first + edges[:-1]
            middles = first + (edges[:-1] + edges[1:]) / 2
            times = np.empty(2 * ENVELOPE_BINS)
            times[0::2], times[1::2] = (starts - pre) / rate, (middles - pre) / rate
            envelope = True
        values = part.astype(np.float64)
        stats = {'min': part.min(axis=0).tolist(), 'max': part.max(axis=0).tolist(),
                 'mean': values.mean(axis=0).tolist(), 'rms': np.sqrt((values ** 2).mean(axis=0)).tolist()}
        header = json.dumps({'type': 'capture', 'channels': CHANNELS, **info, 'points': len(points),
                             'view': [int(first), int(last)], 'envelope': envelope, 'stats': stats}).encode()
        return (struct.pack('<I', len(header)) + header + times.astype('<f8').tobytes()
                + points.astype('<i4').tobytes())

    async def send_view(self):
        if self.samples is None:
            return
        message = self.view_message()
        for client in list(self.clients):
            await client.send(message, binary=True)

    async def broadcast_status(self):
        message = json.dumps({'type': 'status', **self.status, 'running': self.running,
                              'lost_words': self.stream.lost, 'pacing': self.stream.pacing})
        for client in list(self.clients):
            await client.send(message)

    async def handle(self, command):
        kind = command.get('type')
        if kind == 'config':
            for key, value in command.get('config', {}).items():
                if key in self.config:
                    self.config[key] = value
            self.config_changed = True
            self.view = None
        elif kind == 'run':
            self.running, self.single = True, False
        elif kind == 'single':
            self.running, self.single = True, True
        elif kind == 'stop':
            self.running = False
            self.status['state'] = 'stopped'
        elif kind == 'force':
            self.force_now = True
        elif kind == 'view':
            # a time range in seconds from the trigger, or the whole capture
            if command.get('reset') or self.samples is None:
                self.view = None
            else:
                rate, pre = self.capture_info['sample_rate'], self.capture_info['pre']
                first = int(np.floor(command['start'] * rate)) + pre
                last = int(np.ceil(command['end'] * rate)) + pre + 1
                self.view = (first, last)
            await self.send_view()
        await self.broadcast_status()


# ---------------------------------------------------------------------------- HTTP and WebSocket
class WebSocket:
    GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    async def send(self, data, binary=False):
        if isinstance(data, str):
            data = data.encode()
        opcode = 0x2 if binary else 0x1
        n = len(data)
        if n < 126:
            head = struct.pack('!BB', 0x80 | opcode, n)
        elif n < 65536:
            head = struct.pack('!BBH', 0x80 | opcode, 126, n)
        else:
            head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
        try:
            self.writer.write(head + data)
            await self.writer.drain()
        except (ConnectionError, RuntimeError):
            pass

    async def receive(self):
        """next text message, None when the connection closes"""
        message = b''
        while True:
            first, second = await self.reader.readexactly(2)
            opcode, length = first & 0x0f, second & 0x7f
            if length == 126:
                length = struct.unpack('!H', await self.reader.readexactly(2))[0]
            elif length == 127:
                length = struct.unpack('!Q', await self.reader.readexactly(8))[0]
            mask = await self.reader.readexactly(4) if second & 0x80 else b'\0\0\0\0'
            payload = bytearray(await self.reader.readexactly(length))
            for i in range(length):
                payload[i] ^= mask[i % 4]
            if opcode == 0x8:
                return None
            if opcode == 0x9:                       # ping
                self.writer.write(struct.pack('!BB', 0x8a, len(payload)) + payload)
                continue
            message += payload
            if first & 0x80:
                return message.decode()


CONTENT_TYPES = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript', '.css': 'text/css'}


async def http_client(reader, writer, scope):
    try:
        request = await reader.readuntil(b'\r\n\r\n')
    except (asyncio.IncompleteReadError, asyncio.LimitOverrunError):
        writer.close()
        return
    lines = request.decode(errors='replace').split('\r\n')
    method, path = lines[0].split(' ')[:2]
    headers = {l.split(':', 1)[0].strip().lower(): l.split(':', 1)[1].strip() for l in lines[1:] if ':' in l}

    if path == '/ws' and headers.get('upgrade', '').lower() == 'websocket':
        accept = base64.b64encode(hashlib.sha1((headers['sec-websocket-key'] + WebSocket.GUID).encode()).digest()).decode()
        writer.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                      f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode())
        await writer.drain()
        socket = WebSocket(reader, writer)
        scope.clients.add(socket)
        if not scope.probes:
            try:
                await scope.read_probe_table()
            except TimeoutError:
                pass
        await socket.send(json.dumps({'type': 'hello', 'config': scope.config, 'probes': scope.probes,
                                      'clock_hz': CLOCK_HZ, 'block_ram_samples': BLOCK_RAM_SAMPLES,
                                      'deep_samples': DEEP_SAMPLES}))
        await scope.broadcast_status()
        try:
            while True:
                message = await socket.receive()
                if message is None:
                    break
                await scope.handle(json.loads(message))
        except (asyncio.IncompleteReadError, ConnectionError, json.JSONDecodeError):
            pass
        finally:
            scope.clients.discard(socket)
            writer.close()
        return

    name = 'index.html' if path in ('/', '/index.html') else path.lstrip('/')
    file = os.path.normpath(os.path.join(STATIC, name))
    if method == 'GET' and file.startswith(STATIC) and os.path.isfile(file):
        body = open(file, 'rb').read()
        writer.write((f'HTTP/1.1 200 OK\r\nContent-Type: {CONTENT_TYPES.get(os.path.splitext(file)[1], "application/octet-stream")}\r\n'
                      f'Content-Length: {len(body)}\r\nCache-Control: no-cache\r\nConnection: close\r\n\r\n').encode() + body)
    else:
        writer.write(b'HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n')
    await writer.drain()
    writer.close()


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--board', default='192.168.20.10')
    parser.add_argument('--http-port', type=int, default=8080)
    args = parser.parse_args()

    loop = asyncio.get_running_loop()
    registers = Registers(*await open_endpoint(loop, (args.board, 5001)))
    stream = StreamReader(*await open_endpoint(loop, (args.board, 5002), receive_buffer=64 * 2**20))
    scope = Scope(registers, stream)
    try:
        await scope.read_probe_table()
        print(f'{len(scope.probes)} sources: ' + ', '.join(p['name'] for p in scope.probes))
    except TimeoutError:
        print('no reply from the board, the probe table is read when a browser connects')
    server = await asyncio.start_server(lambda r, w: http_client(r, w, scope), '0.0.0.0', args.http_port)
    print(f'oscilloscope at http://localhost:{args.http_port}, board {args.board}')
    async with server:
        await asyncio.gather(server.serve_forever(), scope.run())


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
