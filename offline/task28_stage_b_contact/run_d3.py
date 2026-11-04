#!/usr/bin/env python3
"""Run one C/D job, with each configuration in a fresh simulator process."""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.d2_analysis import compare


def matched(a, b, rows_a, rows_b):
    keys = ('runtime_mass_kg', 'gravity_world_m_s2', 'warmup_app_updates', 'warmup_physics_steps',
            'solver', 'stabilization', 'view_order', 'controls', 'cube_contact_report',
            'table_contact_report', 'ros_graph_deactivated')
    differences = [key for key in keys if a[key] != b[key]]
    for key, tolerance in [('initial_cube_xyz_m', 1e-4), ('initial_cube_velocity_m_s', 1e-5),
                           ('initial_cube_quaternion_xyzw', 1e-5)]:
        if np.linalg.norm(np.array(a[key]) - b[key]) > tolerance:
            differences.append(key)
    if not np.allclose([r['dt_s'] for r in rows_a], [r['dt_s'] for r in rows_b], rtol=0, atol=1e-9):
        differences.append('physics_dt')
    if a['tool_shapes'] or len(b['tool_shapes']) != 7:
        differences.append('tool_shape_contract')
    return differences


def main():
    parent = Path(sys.argv[1]).resolve()
    if (parent / 'ros_bridge_context_live_exit.txt').read_text().strip() != '0':
        raise RuntimeError('ROS_BRIDGE_PREFLIGHT_FAILED')
    if 'ROS_BRIDGE_PROBE_OK' not in (parent / 'ros_bridge_context_live.log').read_text():
        raise RuntimeError('ROS_BRIDGE_PROBE_MISSING')
    out = parent / 'cd'
    out.mkdir(exist_ok=False)
    manifest = {'status': 'FAILED', 'reset_boundary': 'fresh Isaac process and stage per configuration', 'runs': []}
    results = {}
    try:
        for name in ('C', 'D'):
            cmd = ['bash', str(ROOT / 'isaac/scripts/task28_launch_bundled_humble.sh'),
                   str(ROOT / 'isaac/scripts/task28_d2_view_compare.py'), '--config', name, '--output', str(out / name)]
            start = time.time_ns()
            with (out / f'{name}_process.log').open('xb') as log:
                child = subprocess.Popen(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                try:
                    code = child.wait(timeout=240)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    code = child.wait()
            manifest['runs'].append({'config': name, 'exit_code': code, 'start_wall_ns': start, 'end_wall_ns': time.time_ns()})
            if code:
                raise RuntimeError(f'{name}_CHILD_FAILED:{code}')
            folder = out / name
            summary = json.loads((folder / 'summary.json').read_text())
            rows = [json.loads(line) for line in (folder / 'timeseries.jsonl').read_text().splitlines()]
            events = [json.loads(line) for line in (folder / 'contact_events.jsonl').read_text().splitlines()]
            if summary['status'] != 'RECORDED' or len(rows) != 60 or len(events) != summary['events']:
                raise RuntimeError(f'{name}_INCOMPLETE_EVIDENCE')
            results[name] = (summary, rows)
        a, ar = results['C']
        b, br = results['D']
        differences = matched(a, b, ar, br)
        comparison = compare(ar, br)
        comparison['C'] = comparison.pop('A')
        comparison['D'] = comparison.pop('B')
        comparison['control_mismatches'] = differences
        comparison['maximum_force_difference_N'] = max(abs(x['support_force_N'][2] - y['support_force_N'][2]) for x, y in zip(ar, br))
        (out / 'comparison.json').write_text(json.dumps(comparison, indent=2, allow_nan=False) + '\n')
        manifest['status'] = 'RECORDED' if not differences and comparison['initial_states_matched'] else 'UNMATCHED'
        manifest['control_mismatches'] = differences
    except Exception as exc:
        manifest['reason'] = repr(exc)
    finally:
        (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print('TASK28_D3_JOB', json.dumps(manifest), flush=True)
    return 0 if manifest['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
