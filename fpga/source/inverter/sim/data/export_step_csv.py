#!/usr/bin/env python3
"""Export an output bridge step capture (the scope's binary capture and its JSON header, as the
scope test scripts save them) to a CSV for output_filter_fit_tb.vhd:

    ./export_step_csv.py <capture name> <reference before> <reference after> <out.csv> [ms before] [ms after]

Columns: t (s from the step, the "stepped" lane's rise), the reference (V), vdc_out, vfilt_out
(V), iac_out (A, positive into the output filter), the same three as one switching period
(22 kHz) means, and leg 1's and leg 2's high side gates. 500 kS/s captures, the board's
scaling of 2026-10-09 (plots.js)."""
import json, sys
import numpy as np

name, before, after, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
ms_before = float(sys.argv[5]) if len(sys.argv) > 5 else 2.0
ms_after = float(sys.argv[6]) if len(sys.argv) > 6 else 18.0
meta = json.load(open(name + '.json')); h = meta['header']; ch = meta['channels']
n, fs = h['length'], h['sample_rate']
x = np.frombuffer(open(name + '.bin', 'rb').read()[:n * 16], '<i2').reshape(n, 8).astype(float)
out_lane = x[:, ch.index('output test')].astype(int)
s0 = int(np.argmax(out_lane & 2))
vdc = (x[:, ch.index('sec 0 vdc_out')] + 15.197) / 16.857480
vfilt = (x[:, ch.index('sec 2 vfilt_out')] - 8153.273) / -16.327273
iac = (x[:, ch.index('sdm iac_out')] + 331.6) / -256.0
w = int(round(fs / 22000.0))                   # one switching period of samples
mean = lambda s: np.convolve(s, np.ones(w) / w, 'same')
a, b = s0 - int(ms_before * 1e-3 * fs), s0 + int(ms_after * 1e-3 * fs)
t = (np.arange(a, b) - s0) / fs
ref = np.where(t < 0, before, after)
cols = [t, ref, vdc[a:b], vfilt[a:b], iac[a:b], mean(vdc)[a:b], mean(vfilt)[a:b], mean(iac)[a:b],
        (out_lane[a:b] >> 2) & 1, (out_lane[a:b] >> 3) & 1]
with open(out, 'w') as f:
    f.write('t,vref,vdc_out,vfilt_out,iac_out,vdc_mean,vfilt_mean,iac_mean,leg1_high,leg2_high\n')
    for row in zip(*cols):
        f.write('%.7e,%.3f,%.4f,%.4f,%.5f,%.4f,%.4f,%.5f,%d,%d\n' % row)
print(f'{out}: {b - a} rows, {fs / 1e3:.0f} kS/s, {ms_before} ms before the step, {ms_after} ms after')
