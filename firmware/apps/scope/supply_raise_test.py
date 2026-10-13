#!/usr/bin/env python3
"""The supply started as its Start does, then the primary DC link raised in steps, the DAB on.
    python3 supply_raise_test.py [address] [--start 62] [--to 80] [--step 4] [--hold 3]
                                 [--d-max 0.4] [--limit 90] [--output SECONDS] [--fast DIVIDER]
                                 [--sine VOLTS,HZ] [--yes]

The run: the supply started to idle (supply_sequencer.vhd: the gate driver supplies, the input
boost to --start, then the DHB, its voltage control holding the secondary link at the
primary's); the boost's reference then raised by --step V every --hold seconds up to --to, held
there --hold seconds more; with --output, the output then switched on (its Output on: the
loaded waveform in closed loop, none loaded: 0 V held) for that many seconds, its observed
voltage and current shown, switched off back to idle; then everything stopped and the supplies
off. With --sine the output runs a sine of that amplitude and frequency (100 points, repeated
until the output is switched off) instead of 0 V; a single 0 V point is loaded after, so a later
Output on holds 0 V. With --fast, instead of the run's capture, one capture with the output on (1.5 s after
it came on, auto triggered: 32768 samples a sample every DIVIDER + 1 clocks, 124: 1 MS/s) of the
output's filter voltage and current (raw and the period's mean), the observer's voltage, the
control's bridge reference: the output's ripple apart from the measurements' spikes. Each level's link,
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
from input_boost_test import boost, scaled, VDC

CHANNELS = ['pri 1 vdc_in', 'sec 0 vdc_out', 'sdm iac_in mean', 'input test', 'sdm i_dab mean', 'dhb test',
            'sec 2 vfilt_out', 'output test']
VFILT = (8153.273, -16.327273)
FAST_CHANNELS = ['sec 2 vfilt_out', 'sdm iac_out', 'sdm iac_out mean', 'observer v', 'control u_ref',
                 'output test', 'pri 1 vdc_in', 'iac_out sampled']
IAC_OUT = (-331.6, -256.0)
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
    parser.add_argument('--output', type=float, default=0.0, help='seconds with the output on at the top (0: none)')
    parser.add_argument('--sine', type=str, default='', help='VOLTS,HZ: the output a sine (100 points)')
    parser.add_argument('--fast', type=int, default=0, help='a capture with the output on, this divider (124: 1 MS/s)')
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
    seconds = 2.0 + args.hold * (len(levels) + 1) + (args.output + 3.0 if args.output > 0 else 0.0)
    divider = math.ceil(125e6 * seconds / 32768) - 1
    board.configure({'divider': divider, 'length': 32768, 'pre_percent': 2, 'mode': 'normal',
                     'trigger_channel': 3, 'trigger_edge': 0, 'trigger_level': 8,
                     'sources': [names.index(c) for c in CHANNELS]})
    board.captures.clear()
    if not args.fast:
        board.send({'type': 'single'})
        if not board.wait_armed(board.config_version, 10.0):
            board.configure(keep)
            sys.exit('the scope did not arm')
    fast = None

    def look(seconds, label, state='idle'):
        """the statuses for a time; False when the supply left state or the duty sat at its limit"""
        until = time.monotonic() + seconds
        at_limit_since = None
        rows = []
        while time.monotonic() < until:
            board.send({'type': 'heartbeat'})
            s = board.poll(0.25)
            b = boost(s)
            rows.append((s['dc_links'][0], s['dc_links'][1], b['duty'], b['v_mean'], s['observer'][1], s['observer'][0]))
            if s.get('supply') != state:
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
              f'the duty {r[:, 2].mean():.3f}' + (f', the output {r[:, 4].mean():+.2f} V ({r[:, 4].min():+.2f} .. '
              f'{r[:, 4].max():+.2f}), {r[:, 5].mean():+.2f} A (the observer)' if state == 'running' else ''), flush=True)
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
        if ok and args.output > 0:
            if args.sine:
                amplitude, hz = (float(v) for v in args.sine.split(','))
                hold = max(1, round(40000 / hz / 100))
                points = [[round(amplitude * math.sin(2 * math.pi * k / 100), 3), hold] for k in range(100)]
                board.send({'type': 'waveform_load', 'points': points, 'passes': 0})
                board.poll(0.5)
                print(f'the waveform: a {amplitude:g} V sine at {40000 / hold / 100:g} Hz, {board.status.get("waveform")}')
            board.send({'type': 'output_on'})
            print('Output on')
            until = time.monotonic() + 3.0
            while time.monotonic() < until:
                board.send({'type': 'heartbeat'})
                if board.poll(0.25).get('supply') != 'idle':
                    break
            if ok and args.fast and board.status.get('supply') == 'running':
                ok = look(1.5, 'the output on', 'running')
                board.configure({'divider': args.fast, 'length': 32768, 'pre_percent': 50, 'mode': 'auto',
                                 'sources': [names.index(c) for c in FAST_CHANNELS]})
                board.captures.clear()
                board.send({'type': 'single'})
                until = time.monotonic() + 3.0
                while time.monotonic() < until and not board.captures:
                    board.send({'type': 'heartbeat'})
                    board.poll(0.2)
                fast = board.captures[-1] if board.captures else None
                print('the fast capture taken' if fast else 'no fast capture')
            ok = ok and look(max(args.output - (1.5 if args.fast else 0), 0.5), 'the output on', 'running')
            if ok:
                board.send({'type': 'output_off'})
                print('Output off')
                until = time.monotonic() + 3.0
                while time.monotonic() < until:
                    board.send({'type': 'heartbeat'})
                    if board.poll(0.25).get('supply') == 'idle':
                        break
                ok = look(1.0, 'idle again')
    finally:
        board.send({'type': 'supply_stop'})
        deadline = time.monotonic() + (0 if args.fast else seconds + 3)
        while time.monotonic() < deadline and not board.captures:
            board.send({'type': 'heartbeat'})
            board.poll(0.3)
        board.send({'type': 'supply_power', 'on': 0})
        board.pump(0.5)
        if args.sine:
            board.send({'type': 'waveform_load', 'points': [[0.0, 40]], 'passes': 0})
        board.configure(keep)
        board.send({'type': 'run'})
        status = board.poll(0.5)
    print(f'stopped: supply {status.get("supply")}, trips {status.get("dhb_trip")}, '
          f'the last start ended by {(status.get("sequencer", 0) >> 12) & 15} (0: a stop)')

    if args.fast:
        if fast is not None:
            analyse_fast(fast)
        return
    if not board.captures:
        print('no capture')
        return
    message = board.captures[-1]
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n, fs = header['length'], header['sample_rate']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    vdc, vsec = scaled(x[:, 0], VDC), scaled(x[:, 1], VDC)
    iin, vfilt = scaled(x[:, 2], IAC_IN), scaled(x[:, 6], VFILT)
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
    out = np.flatnonzero(x[:, 7].astype(int) != 0)
    if len(out):
        k = (t >= t[out[0]] + 0.2) & (t <= t[out[-1]] - 0.1)
        if args.sine:
            m20 = np.convolve(vfilt, np.ones(w) / w, "same")[k]
            print(f'the output (vfilt_out, 20 ms means) {m20.min():+.1f} .. {m20.max():+.1f} V')
        print(f'the output from {t[out[0]]:.1f} s for {(out[-1] - out[0]) / fs:.1f} s: vfilt_out {vfilt[k].mean():+.2f} V '
              f'(20 ms means {np.convolve(vfilt, np.ones(w) / w, "same")[k].min():+.2f} .. '
              f'{np.convolve(vfilt, np.ones(w) / w, "same")[k].max():+.2f}), the primary {vm[k].mean():.2f} V')
    np.save('supply_raise_capture.npy', np.column_stack([t, vdc, vsec, iin, lanes, x[:, 4], x[:, 5], vfilt, x[:, 7]]))
    print('the capture in supply_raise_capture.npy: t, vdc, vsec, iin, boost lanes, i_dab counts, dhb lanes, '
          'vfilt_out, output lanes')


def analyse_fast(message):
    """the fast capture: the output's voltage and current, their ripple at the switching
    frequency and below, the observer's and the reference beside them"""
    header_length = struct.unpack('<I', message[:4])[0]
    header = json.loads(message[4:4 + header_length])
    n, fs = header['length'], header['sample_rate']
    x = np.frombuffer(message[4 + header_length:][:n * 16], '<i2').reshape(n, 8).astype(float)
    vf = scaled(x[:, 0], VFILT)
    i_raw, i_mean = scaled(x[:, 1], IAC_OUT), scaled(x[:, 2], IAC_OUT)
    v_obs, u_ref = x[:, 3] / 64.0, x[:, 4] / 64.0
    vdc = scaled(x[:, 6], VDC)
    t = np.arange(n) / fs
    period = int(round(fs / 40000))        # samples of a 40 kHz switching period
    def per_period(a):
        m = len(a) // period
        return a[:m * period].reshape(m, period).mean(axis=1)
    print(f'fast capture: {fs / 1e6:.3f} MS/s, {n / fs * 1000:.1f} ms ({n // period} switching periods)')
    for name, a in [('vfilt_out', vf), ('iac_out raw', i_raw), ('iac_out period mean', i_mean),
                    ('observer v', v_obs), ('control u_ref', u_ref), ('primary link', vdc)]:
        p = per_period(a)
        print(f'  {name:20s}: samples {a.mean():+7.2f} (std {a.std():5.2f}, {a.min():+7.1f} .. {a.max():+7.1f}); '
              f'switching period means std {p.std():5.2f} ({p.min():+6.2f} .. {p.max():+6.2f})')
    f = np.fft.rfftfreq(n, 1 / fs)
    for name, a in [('vfilt_out', vf), ('iac_out raw', i_raw)]:
        F = np.abs(np.fft.rfft((a - a.mean()) * np.hanning(n))) * 4 / n
        top = np.argsort(F[1:])[::-1][:4] + 1
        print(f'  {name} lines: ' + ', '.join(f'{f[j] / 1000:.2f} kHz {F[j]:.2f}' for j in top))
    np.save('supply_fast_capture.npy', np.column_stack([t, vf, i_raw, i_mean, v_obs, u_ref, x[:, 5], vdc]))
    print('the fast capture in supply_fast_capture.npy: t, vfilt_out, iac_out raw, its mean, observer v, u_ref, '
          'output lanes, the primary link')


if __name__ == '__main__':
    main()
