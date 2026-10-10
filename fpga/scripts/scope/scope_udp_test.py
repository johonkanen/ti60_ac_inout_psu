#!/usr/bin/env python3
"""Captures through scope_server.py's acquisition code over UDP, without a browser.

    python3 scope_udp_test.py [--board 192.168.20.10] [--length N] [--divider D]

Checks a block RAM capture and a deep HyperRAM capture of the test generator: the sine on
channel 0 crosses the trigger level at the trigger sample and the sample counter on
channel 7 counts without gaps through the whole capture.
"""
import argparse
import asyncio
import os
import sys
import time

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import scope_server as ss

failures = 0


def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<40} {detail}")
    failures += not ok


async def run_capture(scope, deep, length, divider):
    scope.config.update(deep=deep, length=length, divider=divider, mode='normal', pre_percent=25,
                        trigger_channel=0, trigger_edge=0, trigger_level=0, sources=list(range(8)),
                        generator_hz=ss.CLOCK_HZ / (divider + 1) / 1000)
    await scope.apply_config()
    scope.running = True
    started = time.monotonic()
    samples = await scope.capture()
    return samples, time.monotonic() - started


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--board', default='192.168.20.10')
    parser.add_argument('--length', type=int, default=ss.DEEP_SAMPLES)
    parser.add_argument('--divider', type=int, default=9)
    args = parser.parse_args()

    loop = asyncio.get_running_loop()
    registers = ss.Registers(*await ss.open_endpoint(loop, (args.board, 5001)))
    stream = ss.StreamReader(*await ss.open_endpoint(loop, (args.board, 5002), receive_buffer=64 * 2**20))
    scope = ss.Scope(registers, stream)

    for deep, length in ((False, ss.BLOCK_RAM_SAMPLES), (True, args.length)):
        name = 'deep' if deep else 'block RAM'
        samples, seconds = await run_capture(scope, deep, length, args.divider)
        check(f'{name} capture of {length} samples', samples is not None and len(samples) == length,
              f"{seconds:.2f} s, download {scope.status.get('download')}, overflows {scope.status.get('overflows')}, "
              f"{stream.lost} words requested again")
        counter = samples[:, 7].astype(np.int64)
        gaps = int(np.count_nonzero((np.diff(counter) % 65536) != 1))
        check(f'{name} counter continuous', gaps == 0, f'{gaps} gaps')
        pre = scope.pre
        check(f'{name} trigger at sample {pre}', samples[pre - 1, 0] < 0 <= samples[pre, 0],
              f'{samples[pre - 1, 0]} -> {samples[pre, 0]}')
        # envelope message for the browser
        scope.samples, scope.capture_info = samples, {
            'length': length, 'pre': pre, 'sample_rate': scope.sample_rate, 'deep': deep, 'sources': list(range(8)),
            'forced': False, 'capture': 0, 'trigger_channel': 0, 'trigger_level': 0}
        message = scope.view_message()
        check(f'{name} view message', len(message) < 1_000_000, f'{len(message)} bytes')

    print('all passed' if failures == 0 else f'{failures} failed')
    return failures


if __name__ == '__main__':
    sys.exit(asyncio.run(main()) != 0)
