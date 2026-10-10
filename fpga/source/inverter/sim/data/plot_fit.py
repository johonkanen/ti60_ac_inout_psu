#!/usr/bin/env python3
"""Plot output_filter_fit_tb's fit (output_filter_fit.csv) over the measured step:
    ./plot_fit.py [output_filter_fit.csv] [output_step_m25_p25.csv] [output_filter_fit.png]"""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
fit, data, png = (sys.argv[1:4] + ['output_filter_fit.csv', 'output_step_m25_p25.csv', 'output_filter_fit.png'][len(sys.argv[1:4]):])
d = np.genfromtxt(fit, delimiter=',', names=True); m = np.genfromtxt(data, delimiter=',', names=True)
t = d['t'] * 1e3
fig, ax = plt.subplots(2, 1, sharex=True, figsize=(11, 7))
ax[0].plot(t, m['vfilt_out'][:len(t)], lw=.3, color='0.8', label='vfilt_out')
ax[0].plot(t, d['vfilt_mean'], lw=.8, label='vfilt_out, one-period mean'); ax[0].plot(t, d['v_model'], lw=1.2, label='model')
ax[0].set_ylabel('V'); ax[0].set_ylim(-60, 80); ax[0].legend(loc='lower right')
ax[1].plot(t, d['iac_mean'], lw=.8, label='iac_out, one-period mean'); ax[1].plot(t, d['i_model'], lw=1.2, label='model')
ax[1].set_ylabel('A'); ax[1].set_xlabel('ms from the step'); ax[1].legend(loc='upper right')
fig.suptitle('output filter (L1 C1, L2 C2): averaged model fitted to the -25 V -> +25 V step'); fig.tight_layout(); fig.savefig(png, dpi=100)
print(png)
