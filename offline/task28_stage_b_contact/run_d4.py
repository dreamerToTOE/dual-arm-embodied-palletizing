#!/usr/bin/env python3
"""Single bounded C/D job. No sample window begins before both preflights pass."""
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.d2_analysis import compare


def validate(a, b):
    keys = ('authored', 'runtime_mass_kg', 'gravity_world_m_s2', 'warmup_physics_steps',
            'solver', 'stabilization', 'view_order', 'material_paths', 'effective_material',
            'cube_mass_properties', 'ros_graph_absent')
    differences = [k for k in keys if a[k] != b[k]]
    if (a['tool_shapes'] or len(b['tool_shapes']) != 7 or len(b['tool_separation_m']) != 7
            or any(g <= 0 for g in b['tool_separation_m'].values())):
        differences.append('tool_geometry_or_separation')
    return differences


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=False)
    manifest = {'status': 'FAILED', 'runs': [], 'gate_opened': False, 'reason': None}
    children, logs = {}, {}
    try:
        ps = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
        existing = [line.strip() for line in ps.splitlines() if '/isaacsim-4.5.0/kit/python/bin/python3' in line]
        if existing:
            raise RuntimeError(f'UNRELATED_ISAAC_PROCESS:{existing}')
        env = os.environ.copy()
        for k in ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'AMENT_PREFIX_PATH', 'LD_PRELOAD'):
            env.pop(k, None)
        env['PYTHONUNBUFFERED'] = '1'
        for name in ('C', 'D'):
            log = (out / f'{name}_process.log').open('xb')
            logs[name] = log
            cmd = ['/home/ubuntu2004/isaacsim-4.5.0/python.sh', str(ROOT / 'isaac/scripts/task28_d4_tool_isolation.py'),
                   '--config', name, '--output', str(out / name), '--gate-dir', str(out)]
            children[name] = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            if any(p.poll() is not None for p in children.values()):
                raise RuntimeError('CHILD_EXITED_BEFORE_PREFLIGHT')
            if all((out / n / 'preflight.json').is_file() for n in children):
                break
            time.sleep(.1)
        else:
            raise RuntimeError('PREFLIGHT_TIMEOUT')
        snapshots = {n: json.loads((out / n / 'preflight.json').read_text()) for n in children}
        differences = validate(snapshots['C'], snapshots['D'])
        for name, log in logs.items():
            log.flush()
            content = (out / f'{name}_process.log').read_text(errors='replace')
            if '[Error]' in content or 'ROS2 Bridge startup failed' in content:
                differences.append(f'{name}_startup_log_error')
        manifest['preflight_differences'] = differences
        if differences:
            raise RuntimeError('PREFLIGHT_FAILED')
        (out / 'go').write_text('preflight passed\n')
        manifest['gate_opened'] = True
        for name, child in children.items():
            code = child.wait(timeout=180)
            manifest['runs'].append({'config': name, 'exit_code': code})
            if code:
                raise RuntimeError(f'{name}_CHILD_FAILED:{code}')
        rows = {n: [json.loads(line) for line in (out / n / 'timeseries.jsonl').read_text().splitlines()] for n in children}
        if any(len(r) != 60 for r in rows.values()):
            raise RuntimeError('INCOMPLETE_SAMPLE')
        data = compare(rows['C'], rows['D'])
        data['C'], data['D'] = data.pop('A'), data.pop('B')
        data['max_support_difference_N'] = max(abs(x['support_force_N'][2] - y['support_force_N'][2]) for x, y in zip(rows['C'], rows['D']))
        (out / 'comparison.json').write_text(json.dumps(data, indent=2) + '\n')
        manifest['status'] = 'RECORDED'
    except Exception as exc:
        manifest['reason'] = repr(exc)
        (out / 'stop').write_text('stop\n')
    finally:
        for name, child in children.items():
            if child.poll() is None:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=10)
            if not any(row['config'] == name for row in manifest['runs']):
                manifest['runs'].append({'config': name, 'exit_code': child.returncode})
        for log in logs.values():
            log.close()
        (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        print('TASK28_D4_JOB', json.dumps(manifest), flush=True)
    return 0 if manifest['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
