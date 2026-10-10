#!/usr/bin/env python3
"""Plot output_voltage_control_tb's square wave (output_voltage_control.csv):
    ./plot_control.py [output_voltage_control.csv] [out.png]"""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
src, png = (sys.argv[1:3] + ['output_voltage_control.csv', 'output_voltage_control.png'][len(sys.argv[1:3]):])
d = np.genfromtxt(src, delimiter=',', names=True); t = d['t'] * 1e3
fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 10))
ax[0].plot(t, d['v_ref'], 'k', lw=.8, label='v_ref'); ax[0].plot(t, d['v_c1'], lw=1, label='C1 (vfilt_out)')
ax[0].plot(t, d['v_c2'], lw=.6, label='C2 (the output)'); ax[0].plot(t, d['v_estimate'], lw=.8, label='observer v')
ax[0].legend(loc='upper right', fontsize=8); ax[0].set_ylabel('V')
ax[1].plot(t, d['i_l1'], lw=.8, label='inductor'); ax[1].plot(t, d['i_ref'], lw=.8, label='i_ref')
ax[1].plot(t, d['i_estimate'], lw=.6, label='observer i'); ax[1].legend(loc='upper right', fontsize=8); ax[1].set_ylabel('A')
ax[2].plot(t, d['u_ref'], lw=.8); ax[2].set_ylabel('bridge reference [V]')
ax[3].plot(t, d['du'], lw=.8, label='du (slow)'); ax[3].plot(t, d['iload'] * 10, lw=.8, label='iload (slow) x10')
ax[3].legend(fontsize=8); ax[3].set_xlabel('ms')
fig.tight_layout(); fig.savefig(png, dpi=90); print(png)
