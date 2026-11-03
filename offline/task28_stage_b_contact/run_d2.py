#!/usr/bin/env python3
"""A single job runs at most two independent, non-retried A/B scenes."""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.d2_analysis import compare


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        raise ValueError('invalid run ID')
    out = ROOT / 'offline/task28_stage_b_contact/evidence' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    manifest = {'status': 'FAILED', 'reset_boundary': 'separate Isaac process and fresh stage per configuration', 'runs': []}
    results = {}
    try:
        for name in ('A', 'B'):
            child_out = out / name
            command = ['/home/ubuntu2004/isaacsim-4.5.0/python.sh',
                       str(ROOT / 'isaac/scripts/task28_d2_view_compare.py'), '--config', name, '--output', str(child_out)]
            started = time.time_ns()
            with (out / f'{name}_process.log').open('xb') as log:
                child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                timed_out = False
                try:
                    exit_code = child.wait(timeout=180)
                except subprocess.TimeoutExpired:
                    timed_out = True
                    os.killpg(child.pid, signal.SIGKILL)
                    exit_code = child.wait()
            receipt = {'config': name, 'start_wall_ns': started, 'end_wall_ns': time.time_ns(),
                       'exit_code': exit_code, 'timed_out': timed_out}
            manifest['runs'].append(receipt)
            (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            if exit_code != 0 or timed_out:
                raise RuntimeError(f'{name}_PHYSICS_CHILD_FAILED:{exit_code}')
            summary = json.loads((child_out / 'summary.json').read_text())
            rows = [json.loads(line) for line in (child_out / 'timeseries.jsonl').read_text().splitlines()]
            events = [json.loads(line) for line in (child_out / 'contact_events.jsonl').read_text().splitlines()]
            if summary['status'] != 'RECORDED' or len(rows) != 60 or len(rows) != summary['samples'] or len(events) != summary['events']:
                raise RuntimeError(f'{name}_EVIDENCE_INCOMPLETE')
            results[name] = (summary, rows)
        summary_a, rows_a = results['A']
        summary_b, rows_b = results['B']
        comparison = compare(rows_a, rows_b)
        comparison['warmup_physics_steps'] = [summary_a['warmup_physics_steps'], summary_b['warmup_physics_steps']]
        comparison['warmup_matched'] = comparison['warmup_physics_steps'][0] == comparison['warmup_physics_steps'][1]
        comparison['initial_tool_position_difference_m'] = float(np.linalg.norm(
            np.array(summary_a['initial_tool_xyz_m']) - summary_b['initial_tool_xyz_m']))
        comparison['initial_orientation_difference'] = float(min(
            np.linalg.norm(np.array(summary_a['initial_cube_quaternion_xyzw']) - summary_b['initial_cube_quaternion_xyzw']),
            np.linalg.norm(np.array(summary_a['initial_cube_quaternion_xyzw']) + summary_b['initial_cube_quaternion_xyzw'])))
        comparison['initial_velocity_difference_m_s'] = float(np.linalg.norm(
            np.array(summary_a['initial_cube_velocity_m_s']) - summary_b['initial_cube_velocity_m_s']))
        comparison['initial_states_matched'] &= (comparison['warmup_matched'] and
                                                 comparison['initial_tool_position_difference_m'] < .0001 and
                                                 comparison['initial_orientation_difference'] < .001 and
                                                 comparison['initial_velocity_difference_m_s'] < .005)
        manifest['status'] = 'RECORDED' if comparison['initial_states_matched'] else 'UNMATCHED_INITIAL_STATES'
        (out / 'comparison.json').write_text(json.dumps(comparison, indent=2, allow_nan=False) + '\n')
    except Exception as exc:
        manifest['reason'] = repr(exc)
    finally:
        (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print('TASK28_D2_JOB', json.dumps(manifest), flush=True)
    return 0 if manifest['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
