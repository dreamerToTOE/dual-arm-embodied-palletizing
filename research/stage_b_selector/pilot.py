#!/usr/bin/env python3
"""Run the unchanged pre-contact planner on documented synthetic offline fixtures."""
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'offline/task28_stage_b_contact'


def half_extent(q, axis):
    w, x, y, z = q
    rot = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
           (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
           (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    return .06 * sum(abs(v) for v in rot[axis])


def cases():
    base = json.loads((SOURCE / 'fixture.json').read_text())
    park = json.loads((ROOT / 'offline/task28_task27_integration/evidence/task28_b0_poststep_probe_v2/poststep.jsonl').read_text().splitlines()[0])['snapshot']
    rows = list(csv.DictReader((SOURCE / 'evidence/precontact_v5/trajectory.csv').open()))
    result = [('S1_reference', 'S1', copy.deepcopy(base))]
    control = copy.deepcopy(base)
    control.update(cube_xyz_m=[1.08, .06021100655198097, .26007622480392456],
                   cube_quaternion_wxyz=[1, 0, 0, 0], right_q_rad=park['right_q_rad'])
    result.append(('S2_aligned_control', 'S2', copy.deepcopy(control)))
    for angle in (-6, -4, -2, 2, 4, 6):
        case = copy.deepcopy(control)
        angle_rad = math.radians(angle)
        case['cube_quaternion_wxyz'] = [math.cos(angle_rad/2), 0, 0, math.sin(angle_rad/2)]
        result.append((f'S2_yaw_{angle:+d}deg', 'S2', case))
    for y in (-.235, -.22, .22, .235):
        case = copy.deepcopy(control)
        case['cube_xyz_m'][1] = y
        result.append((f'S3_y_{y:+.3f}', 'S3', case))
    for index in (250, 550, 900):
        case = copy.deepcopy(base)
        case['left_q_rad'] = [float(rows[index][f'left_fr3_joint{j}']) for j in range(1, 8)]
        result.append((f'S4_left_path_sample_{index}', 'S4', case))
    return result


def main():
    if len(sys.argv) != 5:
        raise SystemExit('usage: pilot.py PRECONTACT URDF SRDF NEW_OUTPUT_DIR')
    binary, urdf, srdf = map(Path, sys.argv[1:4])
    output = Path(sys.argv[4])
    output.mkdir(parents=True, exist_ok=False)
    assert abs(half_extent([1, 0, 0, 0], 0) - .06) < 1e-12
    a = math.radians(6)
    assert abs(half_extent([math.cos(a/2), 0, 0, math.sin(a/2)], 0) - .06*(math.cos(a)+math.sin(a))) < 1e-12
    results = []
    env = os.environ.copy()
    env.setdefault('ROS_DOMAIN_ID', '181')
    env['ROS_LOCALHOST_ONLY'] = '1'
    for name, scenario, fixture in cases():
        fixture['description'] = ('Historical simultaneous reference, unchanged.' if name == 'S1_reference' else
                                  'Synthetic offline combination; not a simultaneous measurement or a physical recovery state.')
        xyz, quat = fixture['cube_xyz_m'], fixture['cube_quaternion_wxyz']
        gaps = {'deep_m': 1.16 - xyz[0] - half_extent(quat, 0),
                'minus_y_m': xyz[1] + .303 - half_extent(quat, 1),
                'plus_y_m': .303 - xyz[1] - half_extent(quat, 1)}
        assert min(gaps.values()) > 0, (name, gaps)
        case_dir = output / name
        case_dir.mkdir()
        fixture_file = case_dir / 'fixture.json'
        fixture_file.write_text(json.dumps(fixture, indent=2) + '\n')
        start = time.monotonic()
        try:
            p = subprocess.run([str(binary), str(urdf), str(srdf), str(fixture_file), str(case_dir / 'trajectory.csv')],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, timeout=45)
            log, rc = p.stdout, p.returncode
        except subprocess.TimeoutExpired as exc:
            log = (exc.stdout or b'').decode(errors='replace') if isinstance(exc.stdout, bytes) else (exc.stdout or '')
            log += '\nPILOT_TIMEOUT 45 s\n'
            rc = None
        elapsed = time.monotonic() - start
        (case_dir / 'planner.log').write_text(log)
        match = re.search(r'PRECONTACT_PASS (.*)', log)
        reject = re.search(r'PRECONTACT_REJECT: (.*)', log)
        result = {'case': name, 'scenario': scenario, 'cube_wall_clearance_m': gaps,
                  'elapsed_s': round(elapsed, 3), 'exit_code': rc,
                  'precontact_pass': rc == 0 and bool(match),
                  'precontact_metrics': match.group(1) if match else None,
                  'rejection': reject.group(1) if reject else ('timeout' if rc is None else None),
                  'full_insertion_and_retreat': 'NOT_IMPLEMENTED_OR_NOT_VALIDATED'}
        results.append(result)
        print(json.dumps(result), flush=True)
    digest = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (binary, urdf, srdf)}
    (output / 'results.json').write_text(json.dumps({'seed': 28, 'ros_domain_id': int(env['ROS_DOMAIN_ID']), 'sha256': digest, 'results': results}, indent=2) + '\n')


if __name__ == '__main__':
    main()
