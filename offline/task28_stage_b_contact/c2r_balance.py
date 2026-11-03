#!/usr/bin/env python3
"""Per-step three-axis momentum accounting from saved C2 evidence."""
import argparse
import json
from pathlib import Path

import numpy as np


def calculate(rows, mass, gravity):
    gravity = np.asarray(gravity, dtype=float)
    result = []
    for previous, row in zip(rows, rows[1:]):
        delta = mass * (np.asarray(row['cube_velocity_xyz_m_s']) - np.asarray(previous['cube_velocity_xyz_m_s']))
        tool = np.asarray(row['cube_tool_impulse_world_Ns'])
        table = np.asarray(row['cube_table_impulse_world_Ns'])
        other = np.asarray(row['cube_contact_impulse_world_Ns']) - tool - table
        weight = mass * gravity * row['dt_s']
        residual = delta - tool - table - other - weight
        friction = (np.asarray(row['cube_table_friction_force_world_N']) * row['dt_s']
                    if 'cube_table_friction_force_world_N' in row else None)
        result.append({'step': row['step'], 'time_s': row['time_s'], 'dt_s': row['dt_s'],
                       'delta_momentum_Ns': delta.tolist(), 'tool_impulse_Ns': tool.tolist(),
                       'table_impulse_Ns': table.tolist(), 'other_contact_impulse_Ns': other.tolist(),
                       'gravity_impulse_Ns': weight.tolist(), 'unexplained_residual_Ns': residual.tolist(),
                       'table_friction_impulse_Ns': friction.tolist() if friction is not None else None,
                       'residual_after_table_friction_Ns': (residual - friction).tolist() if friction is not None else None,
                       'other_external_impulse_Ns': None})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run')
    parser.add_argument('output')
    args = parser.parse_args()
    root, output = Path(args.run), Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    summary = json.loads((root / 'summary.json').read_text())
    rows = [json.loads(line) for line in (root / 'timeseries.jsonl').read_text().splitlines()]
    mass = summary.get('cube_runtime_mass_kg', .8)
    gravity = summary.get('gravity_vector_m_s2', [0, 0, -9.81])
    values = calculate(rows, mass, gravity)
    with (output / 'per_step.jsonl').open('x') as stream:
        for value in values:
            stream.write(json.dumps(value, allow_nan=False) + '\n')
    residual = np.array([v['unexplained_residual_Ns'] for v in values])
    metrics = {'samples': len(rows), 'intervals': len(values), 'mass_kg': mass, 'gravity_m_s2': gravity,
               'runtime_mass_and_gravity_recorded': 'cube_runtime_mass_kg' in summary and 'gravity_vector_m_s2' in summary,
               'raw_contact_event_file_available': (root / 'contact_events.jsonl').exists(),
               'residual_sum_Ns': residual.sum(axis=0).tolist() if len(values) else None,
               'residual_step_rms_Ns': np.sqrt(np.mean(residual ** 2, axis=0)).tolist() if len(values) else None,
               'residual_after_table_friction_sum_Ns': np.sum([v['residual_after_table_friction_Ns'] for v in values], axis=0).tolist()
                   if values and all(v['residual_after_table_friction_Ns'] is not None for v in values) else None,
               'unknown_external_force': True}
    with (output / 'metrics.json').open('x') as stream:
        json.dump(metrics, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(metrics, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
