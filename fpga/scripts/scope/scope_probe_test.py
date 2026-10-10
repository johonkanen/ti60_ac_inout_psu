#!/usr/bin/env python3
"""Probe interface of the scope over UDP, through scope_server.py's acquisition code.

    python3 scope_probe_test.py [--board 192.168.20.10]

Reads the probe table, catches a register write with a pattern trigger on the digital
'bus' probe and checks the 'bus address' probe at the trigger, then triggers on an
unsigned level of the free running counter.
"""
import argparse
import asyncio
import os
import sys

import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import scope_server as ss

failures = 0


def check(name, ok, detail=''):
    global failures
    print(f"{'PASS' if ok else 'FAIL'}  {name:<44} {detail}")
    failures += not ok


async def capture_while(scope, action):
    """arm a capture, run action once the capture waits for the trigger"""
    scope.running = True
    task = asyncio.create_task(scope.capture())
    for _ in range(100):
        if (await scope.registers.read(ss.COMMAND)) & 7 == 2:
            break
        await asyncio.sleep(0.01)
    await action()
    return await asyncio.wait_for(task, 5)


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--board', default='192.168.20.10')
    args = parser.parse_args()

    loop = asyncio.get_running_loop()
    registers = ss.Registers(*await ss.open_endpoint(loop, (args.board, 5001)))
    stream = ss.StreamReader(*await ss.open_endpoint(loop, (args.board, 5002)))
    scope = ss.Scope(registers, stream)

    await scope.read_probe_table()
    names = [p['name'] for p in scope.probes]
    print('sources: ' + ', '.join(f"{i} {p['name']}{'/u' if p['format'] == 1 else '/d' if p['format'] == 2 else ''}"
                                  for i, p in enumerate(scope.probes)))
    source = {name: i for i, name in enumerate(names)}
    check('probe table', names[0] == 'sine' and 'bus' in source and 'ADC mux 7' in source, f'{len(names)} sources')
    check('digital format of bus', scope.probes[source['bus']]['format'] == ss.DIGITAL)

    # pattern trigger: lane 0 of 'bus' (a write) goes high
    scope.config.update(deep=False, length=256, divider=0, mode='normal', pre_percent=25, trigger_channel=0,
                        trigger_edge=3, trigger_mask=0x0001, trigger_pattern=0x0001,
                        sources=[source['bus'], source['bus address'], source['scratch 15:0'], 7, 0, 1, 2, 3])
    await scope.apply_config()

    async def write_scratch():
        await registers.batch([('w', 3, 0x1234abcd)])

    samples = await capture_while(scope, write_scratch)
    scope.capture_info = {'sources': scope.config['sources']}
    values = scope.channel_values(samples)
    pre = scope.pre
    check('pattern trigger on a bus write', values[pre, 0] & 1 == 1 and values[pre - 1, 0] & 1 == 0,
          f'bus lanes {values[pre - 1, 0]:04b} -> {values[pre, 0]:04b}')
    check('bus address at the trigger', values[pre, 1] == 3, f'address {values[pre, 1]}')
    after = values[pre + 20, 2]
    check('scratch probe after the write', after == 0xabcd, f'0x{after:04x}')

    # unsigned level trigger on the counter crossing 0x9000
    scope.config.update(trigger_edge=0, trigger_level=0x9000, trigger_channel=0,
                        sources=[source['counter 15:0']] + [0] * 7, divider=99)
    await scope.apply_config()
    scope.running = True
    samples = await asyncio.wait_for(scope.capture(), 5)
    scope.capture_info = {'sources': scope.config['sources']}
    values = scope.channel_values(samples)
    check('unsigned level trigger at 0x9000', values[pre - 1, 0] < 0x9000 <= values[pre, 0],
          f'0x{values[pre - 1, 0]:04x} -> 0x{values[pre, 0]:04x}')
    steps = np.diff(values[:, 0]) % 65536
    check('counter probe steps by 100 at divider 99', bool(np.all(steps == 100)), f'steps {set(steps.tolist())}')

    print('all passed' if failures == 0 else f'{failures} failed')
    return failures


if __name__ == '__main__':
    sys.exit(asyncio.run(main()) != 0)
