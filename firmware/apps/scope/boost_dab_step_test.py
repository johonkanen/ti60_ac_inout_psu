#!/usr/bin/env python3
"""The input boost's step with the DAB (DHB) off and on, in one capture.
    python3 boost_dab_step_test.py [address] [--v-ref 58] [--step 60] [--d-max 0.2] [--limit 70]
                                   [--hold 1.2] [--yes]

The run: the gate driver supplies on; the boost started from where the link is (the reference
ramped to v_ref, the default gains); after hold seconds the reference stepped to step and back
(the DAB off); then that boost stopped and the supply started as its Start does (supply_run,
supply_sequencer.vhd: the boost to v_ref again, then the DHB's precharge and its voltage control
holding the secondary link at the primary's: idle, the output off); hold seconds after it is idle
the same step (the DAB's capacitance on the link too). Then everything stopped and the supplies
off; the boost's start settings (boost_v_ref, boost_d_max, boost_limit) as they were. One capture of the whole run: the primary
and secondary DC links, the input's voltage and current, the DAB's current, the boost's and the
DHB's lanes. The step's overshoot and settling with the DAB off and on.

This runs the power stage: it asks first unless --yes, and puts the scope's settings back after.
"""
import argparse
import json
import math
import struct
import sys
import time

import numpy as np

from measure_loop_response import Board
from input_boost_test import boost, watch, scaled, VDC, VAC

CHANNELS = ['pri 1 vdc_in', 'sec 0 vdc_out', 'sdm iac_in mean', 'input test', 'sdm i_dab mean', 'dhb test',
            'pri 0 vac_in', 'output test']
CAPTURE_SECONDS = 7.5


def step_figures(t, v, t0, t1):
    """a step at t0 (v 20 ms means), its end at t1: the levels before and after (0.3 s means), the
    time to 90 % of it, the time it stays within 10 % of it after, the highest over the final"""
    before = v[(t >= t0 - 0.3) & (t < t0)].mean()
    after = v[(t >= t1 - 0.3) & (t < t1)].mean()
    k = (t >= t0) & (t < t1)
    tt, vv = t[k], v[k]
    reached = np.flatnonzero((vv - before) >= 0.9 * (after - before))
    off = np.flatnonzero(abs(vv - after) > 0.1 * abs(after - before))
    return (before, after, tt[reached[0]] - t0 if len(reached) else None,
            tt[off[-1]] - t0 if len(off) else 0.0, vv.max() - after)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('address', nargs='?', default='192.168.20.11')
    parser.add_argument('--v-ref', type=float, default=58.0)
    parser.add_argument('--step', type=float, default=60.0)
    parser.add_argument('--d-max', type=float, default=0.2)
    parser.add_argument('--limit', type=float, default=70.0, help='the boost\'s overvoltage stop, V')
    parser.add_argument('--hold', type=float, default=1.2, help='seconds at each reference')
    parser.add_argument('--yes', action='store_true')
    args = parser.parse_args()

    board = Board(args.address)
    board.pump(1.0)
    names = board.hello['source_names']
    status = board.poll()
    if status['dhb'][0] & 3 or status['output'][0] & 1 or status['boost'][0] & 1:
        sys.exit('the DHB, the output bridge or the boost is running')
    print(f'the primary DC link {status["dc_links"][0]:.1f} V, the secondary {status["dc_links"][1]:.1f} V, '
          f'the input {boost(status)["v_ac"]:.1f} V')
    if not args.yes:
        answer = input(f'Run the boost to {args.v_ref:g} V, a step to {args.step:g} V with the DAB off, then '
                       f'the full system (the DHB and the output at 0 V) and the step again? [y/N] ')
        if answer.strip().lower() != 'y':
            sys.exit('not run')

    keep = {k: board.config[k] for k in ['divider', 'length', 'pre_percent', 'mode', 'trigger_channel',
                                         'trigger_edge', 'trigger_level', 'sources', 'gate_power',
                                         'boost_v_ref', 'boost_d_max', 'boost_limit']}
    board.configure({'boost_v_ref': args.v_ref, 'boost_d_max': args.d_max, 'boost_limit': args.limit})
    sources = [names.index(c) for c in CHANNELS]
    board.configure({'gate_power': 63})
    for _ in range(25):
        if board.poll().get('gate_power_on') == 63:
            break
    else:
        sys.exit('the gate driver supplies did not come on')
    board.pump(0.8)                         # the trip latched while they were off cleared
    board.send({'type': 'dhb_clear_trip'})
    divider = math.ceil(125e6 * CAPTURE_SECONDS / 32768) - 1
    # the trigger: the boost's lanes rising through running (16)
    level = 8
    capture = {'divider': divider, 'length': 32768, 'pre_percent': 2, 'mode': 'normal', 'trigger_channel': 3,
               'trigger_edge': 0, 'trigger_level': level, 'sources': sources}
    board.configure(capture)
    board.captures.clear()
    board.send({'type': 'single'})
    if not board.wait_armed(board.config_version):
        board.configure(keep)
        sys.exit('the scope did not arm')

    log = []
    marks = {}
    t_start = time.monotonic()
    ok = True
    try:
        board.send({'type': 'boost_start', 'v_ref': args.v_ref, 'd_max': args.d_max, 'limit': args.limit})
        print(f'the boost: the reference {args.v_ref:g} V')
        ok = watch(board, args.hold, log)
        for name, v in [('step, the DAB off', args.step), ('back', args.v_ref)]:
            if not ok:
                break
            marks[name] = time.monotonic() - t_start
            board.send({'type': 'boost_set', 'v_ref': v})
            print(f'the reference {v:g} V ({name})')
            ok = watch(board, args.hold, log)
        if ok:
            # the supply's start: the boost again (the sequencer's), then the DAB, to idle
            marks['the supply started'] = time.monotonic() - t_start
            board.send({'type': 'boost_stop'})
            board.send({'type': 'supply_run'})
            print('the supply: the boost to its reference, then the DAB')
            until = time.monotonic() + 6.0
            while time.monotonic() < until:
                board.send({'type': 'heartbeat'})
                s = board.poll(0.3)
                if s.get('supply') in ('idle', 'stopped', 'off', 'tripped'):
                    break
            s = board.status
            print(f'  supply {s.get("supply")} (sequencer {s.get("sequencer", 0):#x}), the DHB {s["dhb"]}, '
                  f'the boost {s["boost"][0]}, the links {s["dc_links"][0]:.1f} / {s["dc_links"][1]:.1f} V')
            ok = s.get('supply') == 'idle'
            if ok:
                marks['the DAB running'] = time.monotonic() - t_start
                ok = watch(board, args.hold, log)
        if ok:
            marks['step, the DAB on'] = time.monotonic() - t_start
            board.send({'type': 'boost_set', 'v_ref': args.step})
            print(f'the reference {args.step:g} V (the DAB on)')
            ok = watch(board, args.hold, log)
    finally:
        board.send({'type': 'supply_stop'})
        board.send({'type': 'boost_stop'})
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not board.captures:
            board.poll(0.3)
        status = board.poll(0.5)
        # the supplies off: supply_run's sequencer holds them on after its run
        board.send({'type': 'supply_power', 'on': 0})
        board.pump(0.5)
        board.configure(keep)
        board.send({'type': 'run'})
        board.poll()

    s = status
    print(f'stopped: supply {s.get("supply")}, the DHB {s["dhb"]}, trips {s["dhb_trip"]}, the boost {s["boost"][0]}')
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
    vsec = scaled(x[:, 1], VDC)             # the secondary's scaling as the primary's (the pages')
    i_in = x[:, 2]
    boost_lanes = x[:, 3].astype(int)
    i_dab = x[:, 4]
    dhb_lanes = x[:, 5].astype(int)
    vac = scaled(x[:, 6], VAC)
    running = np.flatnonzero(boost_lanes & 16)
    if not len(running):
        print('the capture shows no run')
        return
    t = (np.arange(n) - running[0]) / fs
    w = max(1, int(round(0.02 * fs)))
    vm = np.convolve(vdc, np.ones(w) / w, 'same')
    sm = np.convolve(vsec, np.ones(w) / w, 'same')
    dhb_on = np.flatnonzero(dhb_lanes != 0)
    print(f'capture: {fs:.0f} S/s, the boost from t = 0 for {len(running) / fs:.2f} s; the DHB lanes from '
          f'{t[dhb_on[0]]:.2f} s' if len(dhb_on) else 'no DHB lanes')
    # the steps at the marks (the script's clock: the capture's t = 0 the boost's start, within
    # some ms), each to the next mark or the run's end
    run_end = t[running[-1]] - 0.03        # the last 20 ms means reach past the stop
    ends = {'step, the DAB off': marks.get('back', run_end), 'step, the DAB on': run_end}
    for name in ['step, the DAB off', 'step, the DAB on']:
        if name not in marks:
            continue
        t0, t1 = marks[name], ends[name]
        before, after, r90, within, over = step_figures(t, vm, t0, t1)
        k = (t >= t1 - 0.3) & (t < t1)
        print(f'{name}: {before:.2f} -> {after:.2f} V (20 ms means), 90 % at '
              f'{"-" if r90 is None else f"{r90:.2f} s"}, within 10 % of the step after {within:.2f} s, '
              f'the highest {over:+.2f} V over the final; the secondary {sm[k].mean():.2f} V '
              f'({sm[k].min():.2f} .. {sm[k].max():.2f})')
    np.save('boost_dab_capture.npy', np.column_stack([t, vdc, vsec, i_in, boost_lanes, i_dab, dhb_lanes, vac]))
    print('the capture in boost_dab_capture.npy: t, vdc, vsec, iin counts, boost lanes, i_dab counts, dhb lanes, vac')
    print('marks (s from the script\'s start):', {k: round(v, 2) for k, v in marks.items()})


if __name__ == '__main__':
    main()
