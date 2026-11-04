#!/usr/bin/env python3
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(sys.argv[1])
fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8), constrained_layout=True)
for name in ('C', 'D'):
    rows = [json.loads(line) for line in (root / name / 'timeseries.jsonl').read_text().splitlines()]
    t = [r['sim_time_s'] for r in rows]
    label = f'{name}: ' + ('no tool' if name == 'C' else 'stationary tool')
    for ax, values in zip(axes, ([r['support_force_N'][2] for r in rows],
                                 [r['residual_Ns'][2] for r in rows],
                                 [np.linalg.norm(r['cube_velocity_m_s']) for r in rows])):
        ax.plot(t, values, label=label)
    first_sleep = next((r['sim_time_s'] for r in rows if r['sleeping']), None)
    if first_sleep is not None:
        for ax in axes:
            ax.axvspan(first_sleep, t[-1], color='gray', alpha=.08)
axes[0].axhline(7.848, color='k', ls=':', label='mg')
for ax, ylabel in zip(axes, ('Reported support (N)', 'Z residual (N s / step)', 'Cube speed (m/s)')):
    ax.set_ylabel(ylabel); ax.legend(); ax.grid(alpha=.25)
axes[-1].set_xlabel('Simulation time (s); gray = sleeping')
fig.suptitle('D3 C/D: controls unmatched; see report')
fig.savefig(root / 'comparison.png', dpi=150)
