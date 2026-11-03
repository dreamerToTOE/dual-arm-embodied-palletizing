#!/usr/bin/env python3
"""Check recorded pair impulses against reports and Cube momentum."""
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

CUBE = '/World/Task27/Supply/Cube_01'
TOOL = '/World/Task28ContactProbe/Cup'


def analyze(rows, events):
    if not rows or any(row['step'] != i + 1 for i, row in enumerate(rows)):
        raise ValueError('missing or out-of-order physics steps')
    if any(not np.isfinite(row['tool_cube_force_world_N']).all() for row in rows):
        raise ValueError('non-finite force')
    phases = {phase: [r for r in rows if r['phase'] == phase] for phase in
              ('SEPARATED', 'APPROACH', 'LOAD', 'RETRACT', 'SEPARATED_AFTER')}
    if any(not samples for samples in phases.values()):
        raise ValueError('missing phase')
    forces = np.array([r['tool_cube_force_world_N'] for r in rows])
    magnitudes = np.linalg.norm(forces, axis=1)
    active = np.flatnonzero(magnitudes > 1e-5)
    if not len(active):
        raise ValueError('no measured contact')
    last = int(active[-1])
    pair_impulses = defaultdict(lambda: np.zeros(3))
    other = []
    timing_ok = True
    for event in events:
        if set(event['colliders']) != {CUBE, TOOL}:
            other.append(event['colliders'])
            continue
        step = event['step_at_callback'] + 1
        if step < 1 or step > len(rows):
            raise ValueError('contact report outside sampled steps')
        timing_ok &= event['wall_ns'] <= rows[step - 1]['wall_ns']
        sign = 1 if event['colliders'][0] == CUBE else -1
        pair_impulses[step] += sign * sum((np.asarray(p['impulse_world_Ns']) for p in event['points']), np.zeros(3))
    event_error = max(np.linalg.norm(forces[i] * row['dt_s'] - pair_impulses[row['step']])
                      for i, row in enumerate(rows))
    matrix_impulse = sum(forces[i, 1] * rows[i]['dt_s'] for i in range(last + 1))
    delta_momentum = 0.8 * (rows[last]['cube_velocity_m_s'][1] - rows[0]['cube_velocity_m_s'][1])
    initial_max = max(np.linalg.norm(r['tool_cube_force_world_N']) for r in phases['SEPARATED'])
    final_max = max(np.linalg.norm(r['tool_cube_force_world_N']) for r in phases['SEPARATED_AFTER'])
    load = np.array([r['tool_cube_force_world_N'] for r in phases['LOAD']])
    retreat_end = phases['SEPARATED_AFTER'][-1]
    retreat_start = rows[last]
    checks = {
        'separated_force_below_1e-5_N': initial_max < 1e-5,
        'load_positive_y_every_step': bool(np.all(load[:, 1] > 1e-5)),
        'load_transverse_below_1_percent': bool(np.max(np.linalg.norm(load[:, [0, 2]], axis=1)) < 0.01 * np.min(load[:, 1])),
        'contact_lost_after_retraction': final_max < 1e-5 and retreat_end['tool_xyz_m'][1] < retreat_start['tool_xyz_m'][1],
        'cube_displaced_positive_y': rows[last]['cube_xyz_m'][1] > rows[0]['cube_xyz_m'][1],
        'pair_report_and_step_timing': bool(pair_impulses) and timing_ok and event_error < 1e-6,
        'no_other_cube_contact_events': not other,
        'cube_momentum_matches_pair_impulse': abs(matrix_impulse - delta_momentum) < 1e-5,
    }
    checks = {name: bool(value) for name, value in checks.items()}
    return {'status': 'PASS' if all(checks.values()) else 'PARTIAL', 'checks': checks,
            'samples': len(rows), 'first_contact_step': int(active[0] + 1), 'last_contact_step': last + 1,
            'first_contact_time_s': rows[int(active[0])]['simulation_time_s'],
            'last_contact_time_s': rows[last]['simulation_time_s'],
            'initial_max_force_N': initial_max, 'final_max_force_N': final_max,
            'peak_force_world_N': forces[int(np.argmax(magnitudes))].tolist(),
            'load_mean_force_world_N': load.mean(axis=0).tolist(),
            'load_contact_steps': int(np.sum(load[:, 1] > 1e-5)),
            'tool_y_displacement_m': retreat_end['tool_xyz_m'][1] - rows[0]['tool_xyz_m'][1],
            'cube_y_displacement_m': retreat_end['cube_xyz_m'][1] - rows[0]['cube_xyz_m'][1],
            'cube_delta_momentum_to_last_contact_Ns': delta_momentum,
            'pair_impulse_to_last_contact_Ns': matrix_impulse,
            'max_report_matrix_impulse_difference_Ns': event_error,
            'other_cube_contacts': other}


def main():
    path = Path(sys.argv[1])
    read = lambda name: [json.loads(line) for line in (path / name).read_text().splitlines()]
    result = analyze(read('timeseries.jsonl'), read('contact_events.jsonl'))
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
