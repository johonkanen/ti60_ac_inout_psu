"""Test frames for ethernet_stack_tb, with expected replies computed independently of the VHDL.

Each frame is written as one line of hex bytes, from the destination MAC to the FCS.
The expected file has the reply on the same line number, or 'none'.
"""
import struct
import zlib

OWN_MAC = bytes.fromhex('020000000001')
OWN_IP = bytes([192, 168, 20, 10])
PC_MAC = bytes.fromhex('a0b1c2d3e4f5')
PC_IP = bytes([192, 168, 20, 25])
BROADCAST = b'\xff' * 6
ECHO_PORT = 5000
BRIDGE_PORT = 5001
MAX_RESULTS = 367


class RegisterModel:
    """Registers of the bus slave in ethernet_stack_tb."""
    def __init__(self):
        self.registers = {0x100 + i: 0xA5000000 + i for i in range(16)}
        self.registers[0x110] = 0xCAFE0110

    def read(self, address):
        return self.registers.get(address, 0xFFFFFFFF)

    def write(self, address, data):
        if 0x100 <= address < 0x110:
            self.registers[address] = data

    def execute(self, payload):
        """Reply payload for a bridge request payload."""
        if len(payload) < 2:
            return b''
        reply = payload[:2]
        results = 0
        position = 2
        while position < len(payload) and results < MAX_RESULTS:
            command = payload[position]
            size = {2: 3, 4: 7, 5: 6}.get(command)
            if size is None or position + size > len(payload):
                break
            address = int.from_bytes(payload[position + 1:position + 3], 'big')
            if command == 2:
                reply += struct.pack('!I', self.read(address))
                results += 1
            elif command == 4:
                self.write(address, int.from_bytes(payload[position + 3:position + 7], 'big'))
            else:
                for _ in range(int.from_bytes(payload[position + 3:position + 6], 'big')):
                    if results == MAX_RESULTS:
                        break
                    reply += struct.pack('!I', self.read(address))
                    results += 1
            position += size
        return reply


def read_command(address):
    return struct.pack('!BH', 2, address)


def write_command(address, data):
    return struct.pack('!BHI', 4, address, data)


def stream_command(address, count):
    return struct.pack('!BH', 5, address) + count.to_bytes(3, 'big')


def checksum(data):
    if len(data) % 2:
        data += b'\0'
    total = sum(struct.unpack(f'!{len(data) // 2}H', data))
    while total > 0xffff:
        total = (total & 0xffff) + (total >> 16)
    return (~total) & 0xffff


def with_fcs(frame):
    frame = frame + b'\0' * max(0, 60 - len(frame))
    return frame + struct.pack('<I', zlib.crc32(frame) & 0xffffffff)


def ethernet(destination, source, ethertype, payload):
    return destination + source + struct.pack('!H', ethertype) + payload


def arp(operation, sender_mac, sender_ip, target_mac, target_ip):
    return struct.pack('!HHBBH', 1, 0x0800, 6, 4, operation) + sender_mac + sender_ip + target_mac + target_ip


def udp_no_checksum(source_port, destination_port, data):
    return struct.pack('!HHHH', source_port, destination_port, 8 + len(data), 0) + data


def ipv4(source, destination, protocol, payload, identification=0x1234, flags_fragment=0x4000, ttl=64):
    header = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(payload), identification, flags_fragment,
                         ttl, protocol, 0, source, destination)
    header = header[:10] + struct.pack('!H', checksum(header)) + header[12:]
    return header + payload


def icmp_echo(icmp_type, identifier, sequence, data):
    message = struct.pack('!BBHHH', icmp_type, 0, 0, identifier, sequence) + data
    return message[:2] + struct.pack('!H', checksum(message)) + message[4:]


def udp(source_ip, destination_ip, source_port, destination_port, data):
    length = 8 + len(data)
    pseudo = source_ip + destination_ip + struct.pack('!BBH', 0, 17, length)
    segment = struct.pack('!HHHH', source_port, destination_port, length, 0) + data
    value = checksum(pseudo + segment) or 0xffff
    return segment[:6] + struct.pack('!H', value) + segment[8:]


def cases():
    payload = bytes(range(256)) * 6
    yield 'arp request for us', \
        with_fcs(ethernet(BROADCAST, PC_MAC, 0x0806, arp(1, PC_MAC, PC_IP, b'\0' * 6, OWN_IP))), \
        with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0806, arp(2, OWN_MAC, OWN_IP, PC_MAC, PC_IP)))
    yield 'arp request for another ip', \
        with_fcs(ethernet(BROADCAST, PC_MAC, 0x0806, arp(1, PC_MAC, PC_IP, b'\0' * 6, bytes([192, 168, 20, 11])))), None
    for length, sequence in ((56, 1), (57, 2), (0, 3), (1400, 4)):
        data = payload[:length]
        yield f'ping {length} bytes', \
            with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 0x4242, sequence, data)))), \
            with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 1, icmp_echo(0, 0x4242, sequence, data))))
    for length in (1, 18, 19, 100, 1472):
        data = payload[3:3 + length]
        yield f'udp echo {length} bytes', \
            with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 17, udp(PC_IP, OWN_IP, 40000, ECHO_PORT, data)))), \
            with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 17, udp(OWN_IP, PC_IP, ECHO_PORT, 40000, data))))
    yield 'udp to another port', \
        with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 17, udp(PC_IP, OWN_IP, 40000, 5002, b'abc')))), None
    yield 'ping for another mac', \
        with_fcs(ethernet(bytes.fromhex('020000000002'), PC_MAC, 0x0800,
                          ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 1, 1, b'x' * 32)))), None
    bad_crc = bytearray(with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 1, 1, b'y' * 32)))))
    bad_crc[-1] ^= 0x01
    yield 'ping with crc error', bytes(bad_crc), None
    bad_ip = bytearray(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 1, 1, b'z' * 32))))
    bad_ip[14 + 10] ^= 0x01
    yield 'ping with ip checksum error', with_fcs(bytes(bad_ip)), None
    yield 'ip fragment', \
        with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 1, 1, b'f' * 32), flags_fragment=0x2000))), None
    registers = RegisterModel()
    bridge_requests = [
        ('bridge single read', struct.pack('!H', 1) + read_command(0x100)),
        ('bridge write then read', struct.pack('!H', 2) + write_command(0x101, 0x12345678) + read_command(0x101)),
        ('bridge mixed batch', struct.pack('!H', 3) + b''.join(
            write_command(0x100 + i, 0x01010101 * i) + read_command(0x100 + i) for i in range(16)) + read_command(0x110)),
        ('bridge stream', struct.pack('!H', 4) + stream_command(0x102, 5) + read_command(0x103)),
        ('bridge unmapped read', struct.pack('!H', 5) + read_command(0x300) + read_command(0x100)),
        ('bridge truncated command', struct.pack('!H', 6) + read_command(0x104) + write_command(0x105, 0)[:4]),
        ('bridge unknown command', struct.pack('!H', 7) + read_command(0x104) + b'\x07\x01\x00' + read_command(0x105)),
        ('bridge full reply', struct.pack('!H', 8) + stream_command(0x106, 400) + read_command(0x107)),
        ('bridge sequence only', struct.pack('!H', 9)),
        ('bridge empty payload', b''),
    ]
    for name, payload in bridge_requests:
        reply_payload = registers.execute(payload)
        yield name, \
            with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 17, udp(PC_IP, OWN_IP, 40001, BRIDGE_PORT, payload)))), \
            with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 17, udp_no_checksum(BRIDGE_PORT, 40001, reply_payload))))
    # ping after the frames that were dropped, to show nothing got stuck
    yield 'ping after drops', \
        with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 0x4242, 9, b'last')))), \
        with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 1, icmp_echo(0, 0x4242, 9, b'last'))))


def ping_pair(sequence, length):
    data = bytes((sequence + i) & 0xff for i in range(length))
    return (with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 1, icmp_echo(8, 0x5151, sequence, data)))),
            with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 1, icmp_echo(0, 0x5151, sequence, data)))))


def bursts():
    """frames sent back to back: three that all fit the receive buffer and an overload"""
    yield 'burst3', [ping_pair(100 + i, 1400) for i in range(3)]
    yield 'overload', [ping_pair(200 + i, 18) for i in range(8)]


STREAM_PORT = 5002


def stream_source_word(address):
    """data source of the stream test, the same function as in ethernet_loopback_tb"""
    return (address * 0x9E3779B1) & 0xffffffff


def stream_case(request_id=0x1234, first_word=10, word_count=1000, pc_port=40002):
    """a stream request and the datagrams the FPGA should send for it"""
    request = with_fcs(ethernet(OWN_MAC, PC_MAC, 0x0800, ipv4(PC_IP, OWN_IP, 17,
        udp(PC_IP, OWN_IP, pc_port, STREAM_PORT, struct.pack('!HHII', request_id, 0, first_word, word_count)))))
    datagrams = []
    word, left, ip_id = first_word, word_count, 0
    while left:
        count = min(left, 360)
        last = left <= 360
        payload = struct.pack('!HHIHH', request_id, 1 if last else 0, word, count, 0)
        payload += b''.join(struct.pack('!I', stream_source_word(word + i)) for i in range(count))
        datagrams.append(with_fcs(ethernet(PC_MAC, OWN_MAC, 0x0800, ipv4(OWN_IP, PC_IP, 17,
            udp_no_checksum(STREAM_PORT, pc_port, payload), identification=ip_id))))
        word, left, ip_id = word + count, left - count, ip_id + 1
    return request, datagrams


def write(directory):
    import os
    os.makedirs(directory, exist_ok=True)
    request, datagrams = stream_case()
    with open(os.path.join(directory, 'stream_request.txt'), 'w') as f:
        f.write(request.hex() + '\n')
    with open(os.path.join(directory, 'stream_datagrams.txt'), 'w') as f:
        for d in datagrams:
            f.write(d.hex() + '\n')
    request, reply = ping_pair(7, 56)
    with open(os.path.join(directory, 'single_ping_request.txt'), 'w') as f:
        f.write(request.hex() + '\n')
    with open(os.path.join(directory, 'single_ping_reply.txt'), 'w') as f:
        f.write(reply.hex() + '\n')
    for name, pairs in bursts():
        with open(os.path.join(directory, f'{name}_requests.txt'), 'w') as requests, \
             open(os.path.join(directory, f'{name}_replies.txt'), 'w') as replies:
            for request, reply in pairs:
                requests.write(request.hex() + '\n')
                replies.write(reply.hex() + '\n')
    with open(os.path.join(directory, 'requests.txt'), 'w') as requests, \
         open(os.path.join(directory, 'replies.txt'), 'w') as replies, \
         open(os.path.join(directory, 'names.txt'), 'w') as names:
        for name, request, reply in cases():
            requests.write(request.hex() + '\n')
            replies.write((reply.hex() if reply else 'none') + '\n')
            names.write(name + '\n')
