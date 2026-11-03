#!/usr/bin/env python3
"""Plot matched D2 forces and awake momentum residuals against D1."""
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path(sys.argv[1])
    read = lambda path: [json.loads(line) for line in path.read_text().splitlines()]
    a, b = (read(root / name / 'timeseries.jsonl') for name in ('A', 'B'))
    d1 = read(root.parent / 'task28_d1_support_v1/timeseries.jsonl')[:len(a)]
    t = np.array([r['sim_time_s'] for r in a])
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
    axes[0].plot(t, [r['support_force_N'][2] for r in a], label='A: table view')
    axes[0].plot(t, [r['support_force_N'][2] for r in b], '--', label='B: table + tool views')
    axes[0].plot(t, [r['matrix_force_world_N'][2] for r in d1], ':', label='D1: minimal scene')
    axes[0].axhline(7.848, color='k', alpha=.4, ls='--', label='m g')
    axes[0].set_ylabel('Support force (N)'); axes[0].legend()
    axes[1].plot(t, [r['residual_Ns'][2] for r in a], label='A')
    axes[1].plot(t, [r['residual_Ns'][2] for r in b], '--', label='B')
    axes[1].set_ylabel('Z residual (N·s/step)'); axes[1].legend()
    axes[2].plot(t, [np.linalg.norm(r['cube_velocity_m_s']) for r in a], label='Cube speed A')
    axes[2].plot(t, [np.linalg.norm(r['cube_velocity_m_s']) for r in b], '--', label='Cube speed B')
    axes[2].set_ylabel('Speed (m/s)'); axes[2].legend()
    first_sleep = next((r['sim_time_s'] for r in a if r['sleeping']), None)
    if first_sleep is not None:
        for ax in axes:
            ax.axvspan(first_sleep, t[-1], color='gray', alpha=.2)
    for ax in axes: ax.grid(alpha=.25)
    axes[-1].set_xlabel('Simulation time (s); gray = sleeping')
    fig.suptitle('D2 matched scene A/B; D1 shown for context')
    fig.savefig(root / 'comparison.png', dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
