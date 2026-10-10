#!/usr/bin/env python3
"""HyperRAM streaming port of the board_test build, over udp_link (or any link with batch()).

Word addresses count 16 bit HyperRAM words and must be even; data moves two words per
32 bit bus word, the first word in bits 31:16.
"""
import time

BASE = 200
STREAM_ADDRESS, STREAM_DATA, STREAM_STATUS, WRITE_OVERFLOWS, READ_STALLS = 213, 214, 215, 216, 217
WRITES_PENDING = 1 << 20
WRITES_PER_DATAGRAM = 200    # 7 bytes each, stays well below one datagram


def _pack(words):
    if len(words) % 2:
        words = list(words) + [0]
    return [(words[i] << 16) | words[i + 1] for i in range(0, len(words), 2)]


def _unpack(values):
    words = []
    for v in values:
        words += [v >> 16, v & 0xffff]
    return words


def wait_for_writes(link, timeout=5.0):
    end = time.monotonic() + timeout
    while link.batch([('r', STREAM_STATUS)])[0] & WRITES_PENDING:
        if time.monotonic() > end:
            raise TimeoutError('hyperram stream writes did not finish')


def write_words(link, address, words):
    """write 16 bit words from an even word address"""
    if address % 2:
        raise ValueError('stream address must be even')
    values = _pack(words)
    link.batch([('w', STREAM_ADDRESS, address)])
    for i in range(0, len(values), WRITES_PER_DATAGRAM):
        link.batch([('w', STREAM_DATA, v) for v in values[i:i + WRITES_PER_DATAGRAM]])
    wait_for_writes(link)


def read_words(link, address, count):
    """read count 16 bit words from an even word address"""
    if address % 2:
        raise ValueError('stream address must be even')
    link.batch([('w', STREAM_ADDRESS, address)])
    return _unpack(link.batch([('s', STREAM_DATA, (count + 1) // 2)]))[:count]
