#!/usr/bin/env python3
"""Summarize awake and sleeping support evidence without inventing sleep forces."""
import json
import sys
from collections import Counter
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    root = Path(sys.argv[1])
    summary = json.loads((root / 'summary.json').read_text())
    rows = [json.loads(x) for x in (root / 'timeseries.jsonl').read_text().splitlines()]
    events = [json.loads(x) for x in (root / 'contact_events.jsonl').read_text().splitlines()]
    if len(rows) != summary['samples'] or len(events) != summary['events'] or not rows:
        raise ValueError('incomplete evidence')
    if any(row['step'] != i+1 for i, row in enumerate(rows)):
        raise ValueError('unordered physics steps')
    t = np.array([r['sim_time_s'] for r in rows])
    awake = np.array([not r['sleeping'] for r in rows])
    force = np.array([r['matrix_force_world_N'] for r in rows])
    raw = np.array([r['matrix_impulse_world_Ns'] for r in rows])
    event = np.array([r['event_normal_impulse_world_Ns'] for r in rows])
    residual = np.array([r['unexplained_impulse_Ns'] for r in rows])
    velocity = np.array([r['velocity_world_m_s'] for r in rows])
    position = np.array([r['cube_xyz_m'] for r in rows])
    dt = np.array([r['dt_s'] for r in rows])
    before = np.array([r['pre_velocity_world_m_s'] for r in rows])
    pairs = Counter(' <-> '.join(e['colliders']) for e in events)
    awake_indices = np.flatnonzero(awake)
    sleeping_indices = np.flatnonzero(~awake)
    frame_counts = Counter(r['app_update_index'] for r in rows)
    result = {'status': 'D1_PARTIAL', 'reason': 'minimal awake support closes; earlier awake proxy-run discrepancy remains unexplained',
              'samples': len(rows), 'events': len(events), 'pairs': dict(pairs),
              'first_sleep_step': int(sleeping_indices[0] + 1) if len(sleeping_indices) else None,
              'awake_samples': int(awake.sum()), 'sleeping_samples': int((~awake).sum()),
              'awake_force_z_mean_N': float(force[awake, 2].mean()) if awake.any() else None,
              'awake_force_z_min_N': float(force[awake, 2].min()) if awake.any() else None,
              'awake_force_z_max_N': float(force[awake, 2].max()) if awake.any() else None,
              'awake_residual_sum_Ns': residual[awake].sum(axis=0).tolist(),
              'awake_residual_abs_max_Ns': np.max(np.abs(residual[awake]), axis=0).tolist() if awake.any() else None,
              'sleep_naive_residual_sum_Ns': residual[~awake].sum(axis=0).tolist(),
              'sleep_force_z_max_N': float(np.abs(force[~awake, 2]).max()) if (~awake).any() else None,
              'matrix_event_impulse_max_diff_Ns': float(np.max(np.abs(raw - event))),
              'matrix_dt_scaling_max_diff_Ns': float(np.max(np.abs(raw - force * dt[:, None]))),
              'max_speed_m_s': float(np.max(np.linalg.norm(velocity, axis=1))),
              'max_position_change_m': float(np.max(np.linalg.norm(position - position[0], axis=1))),
              'dt_min_s': float(dt.min()), 'dt_max_s': float(dt.max()),
              'pre_velocity_previous_post_max_diff_m_s': float(np.max(np.abs(before[1:] - velocity[:-1]))),
              'physics_callbacks_per_app_update': dict(Counter(frame_counts.values())),
              'raw_sleep_balance_warning': 'naively applying gravity to a sleeping body does not describe a PhysX active-body step; transition impulse is not separately exposed'}
    (root / 'analysis.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    fig, axes = plt.subplots(4, 1, figsize=(10, 11), sharex=True, constrained_layout=True)
    axes[0].plot(t, force[:, 2], label='reported Cube/table normal')
    axes[0].axhline(summary['runtime_mass_kg'] * -summary['gravity_world_m_s2'][2], ls='--', color='k', label='m g')
    axes[0].set_ylabel('Force (N)'); axes[0].legend()
    axes[1].plot(t, velocity[:, 2] * 1e3, label='Cube Z velocity'); axes[1].set_ylabel('Velocity (mm/s)'); axes[1].legend()
    axes[2].plot(t, (position[:, 2]-position[0, 2]) * 1e6, label='Cube Z displacement'); axes[2].set_ylabel('Displacement (µm)'); axes[2].legend()
    axes[3].plot(t, residual[:, 2], label='naive per-step residual'); axes[3].set_ylabel('Impulse (N·s)'); axes[3].legend()
    if len(sleeping_indices):
        for ax in axes:
            ax.axvspan(t[sleeping_indices[0]], t[-1], color='gray', alpha=.2)
    for ax in axes: ax.grid(alpha=.25)
    axes[-1].set_xlabel('Simulation time (s); gray = reported sleeping')
    fig.suptitle('D1 minimal Cube/table: awake vs sleeping')
    fig.savefig(root / 'support_curves.png', dpi=150)
    plt.close(fig)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
