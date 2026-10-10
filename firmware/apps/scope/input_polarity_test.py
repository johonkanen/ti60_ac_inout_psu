#!/usr/bin/env python3
"""The input bridge's polarity: which low side switch boosts the primary DC link.
    python3 input_polarity_test.py [address] [--duty 0.1] [--periods 400] [--limit 120]
                                   [--switches 0,1] [--yes]

With a DC source on the AC input, the low side switch of the leg on its positive terminal boosts
(input_bridge_test.vhd): the input inductor's current, built while the switch is on, flows on
through that leg's high side body diode into the primary DC link, and the link's voltage rises;
the other low side does nothing. Each switch (0 leg 1's low side, 1 leg 2's, 2 and 3 the high
sides) runs alone for a burst (duty, periods of 40 kHz), stopped at once when the link passes
limit volts or on a trip, with a 1 MS/s capture of the link, the filtered input voltage and the
input current around it. The DHB and the output bridge stay off, the relays open.

Per switch: the link's rise, its highest, the input current while switching (iac_in's change from
before the burst, 256 counts an ampere at the 0.5 mohm shunt, its sign as the sensor reads it).
The switch that charges the link is on the positive input terminal's leg.

This runs the power stage: the gate driver supplies on, the input bridge switching. It asks
first unless --yes, and puts the scope's settings and the gate supplies back after.
"""
import argparse
import json
import struct
import sys
import time

import numpy as np

from measure_loop_response import Board

SWITCHES = ["leg 1's low side", "leg 2's low side", "leg 1's high side", "leg 2's high side"]
CHANNELS = ['pri 1 vdc_in', 'pri 3 vfilt_in', 'sdm iac_in', 'sdm iac_in mean', 'pri 0 vac_in',
            'sdm i_dab mean', 'dhb test', 'input test']
# counts to volts (the scope page's scaling): (counts - zero) / per volt
VDC = (-15.197, 16.857480)
VFILT = (8153.273, -16.327273)
IAC_COUNTS_A_AMPERE = 256.0


def volts(counts, scaling):
    zero, per = scaling
    return (counts - zero) / per


def burst(board, which, duty, periods, limit, sources):
    """one switch's burst with a capture around it: (capture arrays, its sample rate, state)"""
    # 1 MS/s, 32768 samples: 33 ms, the burst (10 ms by default) 20 % in
    board.send({'type': 'config', 'config': {'divider': 124, 'length': 32768, 'pre_percent': 20,
                                             'mode': 'normal', 'trigger_channel': 7, 'trigger_edge': 0,
                                             'trigger_level': 1, 'sources': sources}})
    board.pump(0.3)
    board.captures.clear()
    board.send({'type': 'single'})
    board.pump(0.3)
    board.send({'type': 'input_test', 'switch': which, 'duty': duty, 'periods': periods, 'limit': limit})
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline and not board.captures:
        board.poll(0.3)
    status = board.poll(0.5)
    if not board.captures:
        sys.exit(f'no capture for {SWITCHES[which]}')
    message = board.captures[-1]
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n = header['length']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    return x, header['sample_rate'], status


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('address', nargs='?', default='192.168.20.11')
    parser.add_argument('--duty', type=float, default=0.1)
    parser.add_argument('--periods', type=int, default=400, help='of 40 kHz: 400 is 10 ms')
    parser.add_argument('--limit', type=float, default=120.0, help='the overvoltage stop, V')
    parser.add_argument('--switches', default='0,1', help='0 leg 1 low, 1 leg 2 low, 2 leg 1 high, 3 leg 2 high')
    parser.add_argument('--yes', action='store_true', help='do not ask before running the power stage')
    args = parser.parse_args()
    switches = [int(s) for s in args.switches.split(',')]

    board = Board(args.address)
    board.pump(1.0)
    names = board.hello['source_names']
    if 'input test' not in names:
        sys.exit('the board has no input bridge test (FPGA image without input_bridge_test)')
    sources = [names.index(c) for c in CHANNELS]
    config = board.hello['config']
    status = board.poll()
    if status['dhb'][0] & 3 or status['output'][0] & 1:
        sys.exit('the DHB or the output bridge is running')
    print(f'the primary DC link {status["dc_links"][0]:.1f} V, the secondary {status["dc_links"][1]:.1f} V')
    if not args.yes:
        answer = input(f'Run the input bridge: {", ".join(SWITCHES[s] for s in switches)}, one at a time, at '
                       f'{args.duty:g} duty for {args.periods / 40:g} ms, stopped at {args.limit:g} V? [y/N] ')
        if answer.strip().lower() != 'y':
            sys.exit('not run')

    keep = {k: config[k] for k in ['divider', 'length', 'pre_percent', 'mode', 'trigger_channel', 'trigger_edge',
                                   'trigger_level', 'sources', 'gate_power']}
    board.send({'type': 'config', 'config': {'gate_power': 63}})
    for _ in range(25):
        if board.poll().get('gate_power_on') == 63:
            break
    else:
        sys.exit('the gate driver supplies did not come on')
    board.send({'type': 'dhb_clear_trip'})
    board.poll()
    results = []
    try:
        for which in switches:
            x, fs, status = burst(board, which, args.duty, args.periods, args.limit, sources)
            running = np.flatnonzero(x[:, 7].astype(int) & 1)
            if not len(running):
                print(f'{SWITCHES[which]}: did not run (state {status.get("input")})')
                continue
            a, b = running[0], running[-1]
            before = slice(0, a)
            during = slice(a, b + 1)
            after = slice(min(b + int(0.002 * fs), len(x) - 1), len(x))
            vdc = volts(x[:, 0], VDC)
            vfilt = volts(x[:, 1], VFILT)
            i_mean = x[:, 3]
            rise = vdc[after].mean() - vdc[before].mean()
            current = (i_mean[during].mean() - i_mean[before].mean()) / IAC_COUNTS_A_AMPERE
            peak = (np.abs(x[during, 2] - x[before, 2].mean())).max() / IAC_COUNTS_A_AMPERE
            state = status.get('input', [0, 0, 0, 0])
            why = 'the overvoltage stop' if state[0] & 2 else 'a trip' if state[0] & 4 else 'its periods'
            results.append((which, rise, current))
            print(f'{SWITCHES[which]:18s}: the link {vdc[before].mean():6.1f} -> {vdc[after].mean():6.1f} V '
                  f'(rise {rise:+5.1f} V; highest {state[3]:.1f} V by the FPGA), the input '
                  f'{vfilt[before].mean():5.1f} V, iac_in {current:+6.2f} A mean while switching '
                  f'(peak {peak:.1f} A), {state[1]} periods, ended by {why}', flush=True)
            board.pump(1.0)
    finally:
        board.send({'type': 'input_stop'})
        board.send({'type': 'config', 'config': keep})
        board.poll()
    boosting = [r for r in results if r[1] > 2.0]
    if len(boosting) == 1:
        which, rise, current = boosting[0]
        leg = 1 if which in (0, 2) else 2
        print(f'{SWITCHES[which]} charges the link ({rise:+.1f} V): the positive input terminal is on leg {leg}; '
              f'iac_in reads the boost current {"positive" if current > 0 else "negative"} ({current:+.2f} A).')
    elif not boosting:
        print('no switch raised the link by 2 V or more: a longer burst or a higher duty may show it')
    else:
        print('more than one switch raised the link: look at the captures')


if __name__ == '__main__':
    main()
