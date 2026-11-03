"""Matched A/B contact-view comparison, with awake and sleeping kept separate."""
import numpy as np


def compare(a, b):
    if not a or not b:
        raise ValueError('both configurations require observations')
    for name, rows in [('A', a), ('B', b)]:
        if any(row['step'] != i + 1 for i, row in enumerate(rows)):
            raise ValueError(f'{name}: missing or unordered step')
    if len(a) != len(b):
        raise ValueError('unequal observation windows')
    for rows in (a, b):
        if any(not np.isfinite(row['support_force_N']).all() for row in rows):
            raise ValueError('nonfinite force')
    start_position = float(np.linalg.norm(np.array(a[0]['cube_xyz_m']) - b[0]['cube_xyz_m']))
    start_velocity = float(np.linalg.norm(np.array(a[0]['cube_velocity_m_s']) - b[0]['cube_velocity_m_s']))
    matched = start_position < .0001 and start_velocity < .005
    metrics = {}
    for name, rows in [('A', a), ('B', b)]:
        awake = [r for r in rows if not r['sleeping']]
        sleeping = [r for r in rows if r['sleeping']]
        forces = [r['support_force_N'][2] for r in awake]
        metrics[name] = {'samples': len(rows), 'awake_samples': len(awake), 'sleeping_samples': len(sleeping),
                         'awake_force_mean_N': float(np.mean(forces)) if forces else None,
                         'awake_force_first_N': float(forces[0]) if forces else None,
                         'awake_force_last_N': float(forces[-1]) if forces else None,
                         'awake_force_range_N': [float(min(forces)), float(max(forces))] if forces else None,
                         'awake_z_residual_sum_Ns': float(sum(r['residual_Ns'][2] for r in awake)),
                         'max_event_matrix_impulse_difference_Ns': max(float(np.linalg.norm(np.array(r['support_impulse_Ns']) - r['event_table_impulse_Ns'])) for r in rows),
                         'max_early_late_force_difference_N': max(float(np.linalg.norm(np.array(r['support_force_N']) - r['late_support_force_N'])) for r in rows),
                         'callback_order_violations': sum(not (r['pre_monotonic_ns'] <= r['early_monotonic_ns'] <= r['late_monotonic_ns'] and
                                                               all(r['pre_monotonic_ns'] <= t <= r['early_monotonic_ns'] for t in r['event_monotonic_ns'])) for r in rows),
                         'max_tool_movement_m': max(float(np.linalg.norm(np.array(r['tool_xyz_m']) - rows[0]['tool_xyz_m'])) for r in rows)}
    return {'initial_position_difference_m': start_position, 'initial_velocity_difference_m_s': start_velocity,
            'initial_states_matched': matched, 'A': metrics['A'], 'B': metrics['B']}
