#!/usr/bin/env python3
"""The input bridge as a boost converter from a DC source: the primary DC link held at a reference.
    python3 input_boost_test.py [address] [--v-ref 105] [--step 112] [--i-max 1.0] [--limit 120]
                                [--hold 1.5] [--yes]

input_boost_control.vhd switches only leg 2's low side (the input's positive terminal), so no
current can flow back into the source. Its voltage loop is slow (some Hz) on the bridge's output
current, iac_in * vac_in / vdc_in, the current loop fast within i_max at the link.

The run: the boost started from where the link is, its reference ramping to v_ref; after hold
seconds the reference stepped to step, after hold more back to v_ref, then stopped. A 10 kS/s
capture of the link, the input's voltage, the input current and the boost's lanes from the start;
the boost's state from the status messages. The DHB and the output bridge stay off (the link's
only load its bleed), the relays open.

This runs the power stage: the gate driver supplies on, the input bridge switching. It asks first
unless --yes, and puts the scope's settings and the gate supplies back after.
"""
import argparse
import json
import struct
import sys
import time

import numpy as np

from measure_loop_response import Board

# the scope's eight channels: those used first
CHANNELS = ['pri 1 vdc_in', 'pri 0 vac_in', 'sdm iac_in mean', 'input test', 'pri 3 vfilt_in', 'sdm iac_in',
            'sdm i_dab mean', 'dhb test']
# counts to volts (the scope page's scaling): (counts - zero) / per volt
VDC = (-15.197, 16.857480)
VAC = (8137.909, 16.763636)
IAC_IN = (-160.4, 256.0)        # + from the input's + terminal into the bridge (the boost's current)
STOPPED_BY = {2: 'the overvoltage stop', 4: 'a trip', 8: 'the watchdog', 16: 'inhibit'}


def scaled(counts, scaling):
    zero, per = scaling
    return (counts - zero) / per


def boost(status):
    """the status' boost state as a dict"""
    b = status.get('boost')
    if not b:
        return None
    names = ['status', 'i_ref', 'duty', 'i_in', 'v_dc', 'i_bridge', 'v_ac', 'v_ref', 'periods']
    return dict(zip(names, b))


def watch(board, seconds, log):
    """the statuses for a time: heartbeats keep the firmware's watchdog kicks going (no config
    message, which would restart the capture), the board's statuses come every 0.5 s"""
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        board.send({'type': 'heartbeat'})
        board.pump(0.5)
        state = boost(board.status)
        if state:
            log.append(state)
            print(f'  link {state["v_dc"]:6.1f} V (ref {state["v_ref"]:5.1f}), input {state["v_ac"]:5.1f} V '
                  f'{state["i_in"]:+5.2f} A, bridge {state["i_bridge"]:+5.2f} A (ref {state["i_ref"]:4.2f}), '
                  f'duty {state["duty"]:.3f}, status {state["status"]}', flush=True)
            if not state['status'] & 1:
                return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('address', nargs='?', default='192.168.20.11')
    parser.add_argument('--v-ref', type=float, default=105.0)
    parser.add_argument('--step', type=float, default=112.0, help='the reference stepped to, V')
    parser.add_argument('--i-max', type=float, default=1.0, help="the bridge's output current's limit, A")
    parser.add_argument('--limit', type=float, default=120.0, help='the overvoltage stop, V')
    parser.add_argument('--hold', type=float, default=1.5, help='seconds at each reference')
    parser.add_argument('--yes', action='store_true', help='do not ask before running the power stage')
    args = parser.parse_args()
    if max(args.v_ref, args.step) >= args.limit:
        sys.exit('the references must stay under the overvoltage stop')

    board = Board(args.address)
    board.pump(1.0)
    names = board.hello['source_names']
    status = board.poll()
    if 'boost' not in status:
        sys.exit('the board has no boost control (firmware without input_boost_control)')
    sources = [names.index(c) for c in CHANNELS]
    config = board.hello['config']
    if status['dhb'][0] & 3 or status['output'][0] & 1 or status['input'][0] & 1:
        sys.exit('the DHB, the output bridge or the input test is running')
    print(f'the primary DC link {status["dc_links"][0]:.1f} V, the input {boost(status)["v_ac"]:.1f} V')
    if not args.yes:
        answer = input(f'Run the input boost: the link to {args.v_ref:g} V, then {args.step:g} V and back, '
                       f'{args.hold:g} s each, within {args.i_max:g} A, stopped at {args.limit:g} V? [y/N] ')
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
    # 10 kS/s, 32768 samples: 3.3 s from just before the start (the input test's lanes nonzero)
    board.send({'type': 'config', 'config': {'divider': 12499, 'length': 32768, 'pre_percent': 2,
                                             'mode': 'normal', 'trigger_channel': 3, 'trigger_edge': 0,
                                             'trigger_level': 1, 'sources': sources}})
    board.pump(0.3)
    board.captures.clear()
    board.send({'type': 'single'})
    board.pump(0.3)
    log = []
    ok = True
    try:
        board.send({'type': 'boost_start', 'v_ref': args.v_ref, 'i_max': args.i_max, 'limit': args.limit})
        print(f'the reference {args.v_ref:g} V')
        ok = watch(board, args.hold, log)
        if ok:
            board.send({'type': 'boost_set', 'v_ref': args.step})
            print(f'the reference {args.step:g} V')
            ok = watch(board, args.hold, log)
        if ok:
            board.send({'type': 'boost_set', 'v_ref': args.v_ref})
            print(f'the reference {args.v_ref:g} V')
            ok = watch(board, args.hold, log)
    finally:
        board.send({'type': 'boost_stop'})
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline and not board.captures:
            board.poll(0.3)
        status = board.poll(0.5)
        board.send({'type': 'config', 'config': keep})
        board.poll()

    state = boost(status)
    why = STOPPED_BY.get(state['status'] & 30, 'stop') if state else '?'
    print(f'stopped by {why} after {state["periods"] if state else "?"} periods')
    if not board.captures:
        print('no capture')
        return
    message = board.captures[-1]
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n = header['length']
    fs = header['sample_rate']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    vdc = scaled(x[:, 0], VDC)
    vac = scaled(x[:, 1], VAC)
    iin = scaled(x[:, 2], IAC_IN)
    lanes = x[:, 3].astype(int)
    running = np.flatnonzero(lanes & 16)
    if not len(running):
        print('the capture shows no run')
        return
    a = running[0]
    t = (np.arange(n) - a) / fs
    print(f'capture: {fs:.0f} S/s, the run from t = 0 for {len(running) / fs:.2f} s')
    for t0, t1 in [(-0.01, 0.0), (0.5, args.hold), (args.hold + 0.5, 2 * args.hold),
                   (2 * args.hold + 0.5, min(3 * args.hold, t[-1]))]:
        k = (t >= t0) & (t < t1)
        if k.any():
            print(f'  {t0:5.2f} .. {t1:5.2f} s: the link {vdc[k].mean():6.1f} V (ripple {np.ptp(vdc[k]):4.1f} V '
                  f'pp), the input {vac[k].mean():5.1f} V {iin[k].mean():+5.2f} A')
    k = (t >= 0) & (t < args.hold)
    print(f'  the start: the link highest {vdc[k].max():.1f} V, the input current highest {iin[k].max():.2f} A')
    np.save('input_boost_capture.npy', np.column_stack([t, vdc, vac, iin, lanes]))
    print('the capture in input_boost_capture.npy: t, vdc, vac, iin, lanes')


if __name__ == '__main__':
    main()
