#!/usr/bin/env python3
"""The supply started as its Start does, then the primary DC link raised in steps, the DAB on.
    python3 supply_raise_test.py [address] [--start 62] [--to 80] [--step 4] [--hold 3]
                                 [--d-max 0.4] [--limit 90] [--yes]

The run: the supply started to idle (supply_sequencer.vhd: the gate driver supplies, the input
boost to --start, then the DHB, its voltage control holding the secondary link at the
primary's); the boost's reference then raised by --step V every --hold seconds up to --to, held
there --hold seconds more; then everything stopped and the supplies off. Each level's link,
secondary, duty and input current printed. It stops at once when the supply leaves idle (a
trip, the boost's or the DHB's stop) or the duty stays within 0.02 of --d-max for a second (the
reference out of reach). One capture of the whole run: the DC links, the input's voltage and
current, the DAB's current, the boost's and the DHB's lanes; the boost's start settings and the
scope's put back after.

This runs the power stage: it asks first unless --yes.
"""
import argparse
import json
import math
import struct
import sys
import time

import numpy as np

from measure_loop_response import Board
from input_boost_test import boost, scaled, VDC, VAC

CHANNELS = ['pri 1 vdc_in', 'sec 0 vdc_out', 'sdm iac_in mean', 'input test', 'sdm i_dab mean', 'dhb test',
            'pri 0 vac_in', 'output test']
IAC_IN = (-160.4, 256.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('address', nargs='?', default='192.168.20.11')
    parser.add_argument('--start', type=float, default=62.0, help='the start\'s reference, V')
    parser.add_argument('--to', type=float, default=80.0, help='the last reference, V')
    parser.add_argument('--step', type=float, default=4.0, help='V a step')
    parser.add_argument('--hold', type=float, default=3.0, help='seconds at each level')
    parser.add_argument('--d-max', type=float, default=0.4, help='the duty\'s limit')
    parser.add_argument('--limit', type=float, default=90.0, help='the boost\'s overvoltage stop, V')
    parser.add_argument('--yes', action='store_true')
    args = parser.parse_args()
    if args.to >= args.limit:
        sys.exit('the references must stay under the overvoltage stop')

    board = Board(args.address)
    board.pump(1.0)
    names = board.hello['source_names']
    status = board.poll()
    if status.get('supply') not in ('off', 'stopped'):
        sys.exit(f'the supply is {status.get("supply")}: off or stopped first')
    print(f'the input {boost(status)["v_ac"]:.1f} V, the links {status["dc_links"][0]:.1f} / '
          f'{status["dc_links"][1]:.1f} V')
    levels = [args.start]
    while levels[-1] + args.step < args.to:
        levels.append(levels[-1] + args.step)
    if levels[-1] != args.to:
        levels.append(args.to)
    if not args.yes:
        answer = input(f'Start the supply to {args.start:g} V (the DAB on), then {", ".join(f"{v:g}" for v in levels[1:])} V, '
                       f'{args.hold:g} s each, the duty within {args.d_max:g}, stopped at {args.limit:g} V? [y/N] ')
        if answer.strip().lower() != 'y':
            sys.exit('not run')

    keep = {k: board.config[k] for k in ['divider', 'length', 'pre_percent', 'mode', 'trigger_channel',
                                         'trigger_edge', 'trigger_level', 'sources',
                                         'boost_v_ref', 'boost_d_max', 'boost_limit']}
    board.configure({'boost_v_ref': args.start, 'boost_d_max': args.d_max, 'boost_limit': args.limit})
    seconds = 2.0 + args.hold * (len(levels) + 1)
    divider = math.ceil(125e6 * seconds / 32768) - 1
    board.configure({'divider': divider, 'length': 32768, 'pre_percent': 2, 'mode': 'normal',
                     'trigger_channel': 3, 'trigger_edge': 0, 'trigger_level': 8,
                     'sources': [names.index(c) for c in CHANNELS]})
    board.captures.clear()
    board.send({'type': 'single'})
    if not board.wait_armed(board.config_version):
        board.configure(keep)
        sys.exit('the scope did not arm')

    def look(seconds, label):
        """the statuses for a time; False when the supply left idle or the duty sat at its limit"""
        until = time.monotonic() + seconds
        at_limit_since = None
        rows = []
        while time.monotonic() < until:
            board.send({'type': 'heartbeat'})
            s = board.poll(0.25)
            b = boost(s)
            rows.append((s['dc_links'][0], s['dc_links'][1], b['duty'], b['v_mean']))
            if s.get('supply') != 'idle':
                print(f'  {label}: the supply {s.get("supply")} (sequencer {s.get("sequencer", 0):#06x}, '
                      f'trips {s.get("dhb_trip")}, boost {s["boost"][0]})')
                return False
            if b['duty'] >= args.d_max - 0.02:
                at_limit_since = at_limit_since or time.monotonic()
                if time.monotonic() - at_limit_since > 1.0:
                    print(f'  {label}: the duty at its limit ({b["duty"]:.3f}): the reference out of reach')
                    return False
            else:
                at_limit_since = None
        r = np.array(rows[len(rows) // 2:])
        print(f'  {label}: the link {r[:, 3].mean():6.2f} V (its mean), the secondary {r[:, 1].mean():6.1f} V, '
              f'the duty {r[:, 2].mean():.3f}', flush=True)
        return True

    ok = False
    try:
        board.send({'type': 'supply_run'})
        print(f'Start: the boost to {args.start:g} V, then the DAB')
        until = time.monotonic() + 8.0
        while time.monotonic() < until:
            board.send({'type': 'heartbeat'})
            s = board.poll(0.25)
            if s.get('supply') in ('idle', 'stopped', 'off', 'tripped'):
                break
        ok = board.status.get('supply') == 'idle'
        if not ok:
            s = board.status
            print(f'  the start ended: supply {s.get("supply")}, sequencer {s.get("sequencer", 0):#06x}')
        else:
            ok = look(args.hold, f'{args.start:g} V')
        for v in levels[1:]:
            if not ok:
                break
            board.send({'type': 'boost_set', 'v_ref': v})
            ok = look(args.hold, f'{v:g} V')
        if ok:
            ok = look(args.hold, f'{levels[-1]:g} V, held')
    finally:
        board.send({'type': 'supply_stop'})
        deadline = time.monotonic() + seconds + 3
        while time.monotonic() < deadline and not board.captures:
            board.send({'type': 'heartbeat'})
            board.poll(0.3)
        board.send({'type': 'supply_power', 'on': 0})
        board.pump(0.5)
        board.configure(keep)
        board.send({'type': 'run'})
        status = board.poll(0.5)
    print(f'stopped: supply {status.get("supply")}, trips {status.get("dhb_trip")}, '
          f'the last start ended by {(status.get("sequencer", 0) >> 12) & 15} (0: a stop)')

    if not board.captures:
        print('no capture')
        return
    message = board.captures[-1]
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n, fs = header['length'], header['sample_rate']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    vdc, vsec = scaled(x[:, 0], VDC), scaled(x[:, 1], VDC)
    iin, vac = scaled(x[:, 2], IAC_IN), scaled(x[:, 6], VAC)
    lanes = x[:, 3].astype(int)
    running = np.flatnonzero(lanes & 16)
    if not len(running):
        print('the capture shows no run')
        return
    t = (np.arange(n) - running[0]) / fs
    w = max(1, int(round(0.02 * fs)))
    vm = np.convolve(vdc, np.ones(w) / w, 'same')
    k = (t >= 0) & (t <= t[running[-1]])
    print(f'capture: {fs:.0f} S/s, the run {t[running[-1]]:.1f} s; the primary highest {vm[k].max():.1f} V '
          f'(20 ms means), the input current highest {np.convolve(iin, np.ones(w) / w, "same")[k].max():.2f} A')
    np.save('supply_raise_capture.npy', np.column_stack([t, vdc, vsec, iin, lanes, x[:, 4], x[:, 5], vac]))
    print('the capture in supply_raise_capture.npy: t, vdc, vsec, iin, boost lanes, i_dab counts, dhb lanes, vac')


if __name__ == '__main__':
    main()
