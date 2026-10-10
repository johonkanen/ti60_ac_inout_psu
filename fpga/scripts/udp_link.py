#!/usr/bin/env python3
"""Register bus access over UDP, same methods as uart_link of fpga_uart_pc_software.

Datagram to port 5001: sequence number (16 bit) followed by commands in the UART encoding
    0x02 address(16)                read
    0x04 address(16) data(32)       write
    0x05 address(16) count(24)      read the same address count times
The reply carries the sequence number and the 32 bit read results in order. A lost
datagram is sent again; a repeated datagram also repeats its writes.
"""
import socket
import struct

MAX_PAYLOAD = 1472
MAX_RESULTS = 367


class udp_link:
    def __init__(self, host='192.168.20.10', port=5001, timeout=0.2, retries=5):
        self.address = (host, port)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(timeout)
        self.retries = retries
        self.sequence = 0
        self.retransmissions = 0

    def _transaction(self, commands, expected_results):
        self.sequence = (self.sequence + 1) & 0xffff
        request = struct.pack('!H', self.sequence) + commands
        for _ in range(self.retries):
            self.socket.sendto(request, self.address)
            try:
                while True:
                    reply, _ = self.socket.recvfrom(2048)
                    if len(reply) >= 2 and struct.unpack('!H', reply[:2])[0] == self.sequence:
                        break   # replies to earlier retransmissions are skipped
            except socket.timeout:
                self.retransmissions += 1
                continue
            results = list(struct.unpack(f'!{(len(reply) - 2) // 4}I', reply[2:]))
            if len(results) != expected_results:
                raise RuntimeError(f'expected {expected_results} results, got {len(results)}')
            return results
        raise TimeoutError(f'no reply from {self.address} after {self.retries} tries')

    def batch(self, commands):
        """commands: ('r', address) | ('w', address, data) | ('s', address, count)
        returns the read results in order, split over as many datagrams as needed"""
        results = []
        payload, expected = b'', 0

        def flush():
            nonlocal payload, expected
            if payload:
                results.extend(self._transaction(payload, expected))
            payload, expected = b'', 0

        for command in commands:
            if command[0] == 'r':
                encoded, count = struct.pack('!BH', 2, command[1]), 1
            elif command[0] == 'w':
                encoded, count = struct.pack('!BHI', 4, command[1], command[2] & 0xffffffff), 0
            elif command[0] == 's':
                remaining = command[2]
                while remaining > 0:
                    if expected == MAX_RESULTS:
                        flush()
                    count = min(remaining, MAX_RESULTS - expected)
                    if 2 + len(payload) + 6 > MAX_PAYLOAD:
                        flush()
                    payload += struct.pack('!BH', 5, command[1]) + count.to_bytes(3, 'big')
                    expected += count
                    remaining -= count
                continue
            else:
                raise ValueError(command)
            if expected + count > MAX_RESULTS or 2 + len(payload) + len(encoded) > MAX_PAYLOAD:
                flush()
            payload += encoded
            expected += count
        flush()
        return results

    def request_data_from_address(self, address):
        return self.batch([('r', address)])[0]

    def write_data_to_address(self, address, data):
        self.batch([('w', address, data)])

    def stream_data_from_address(self, address, number_of_registers):
        return self.batch([('s', address, int(number_of_registers))])
