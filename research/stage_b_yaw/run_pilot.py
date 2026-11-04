#!/usr/bin/env python3
"""Run isolated offline variants using the prior immutable pilot fixtures."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

root = Path(__file__).resolve().parents[2]
prior = root / 'research/stage_b_selector/evidence/pilot_v1'
if len(sys.argv) != 5:
    raise SystemExit('usage: run_pilot.py BUILD_DIR URDF SRDF NEW_OUTPUT_DIR')
build, urdf, srdf, out = map(Path, sys.argv[1:])
out.mkdir(parents=True, exist_ok=False)
env = os.environ.copy()
env.setdefault('ROS_DOMAIN_ID', '182')
env['ROS_LOCALHOST_ONLY'] = '1'
results = []
for name in ['S2_aligned_control', 'S2_yaw_-2deg', 'S2_yaw_+2deg', 'S2_yaw_-4deg', 'S2_yaw_+4deg', 'S2_yaw_-6deg', 'S2_yaw_+6deg']:
    for mode in ('direct', 'correct'):
        case = out / name / mode
        case.mkdir(parents=True)
        cmd = [str(build/'yaw_planner'), str(urdf), str(srdf), str(prior/name/'fixture.json'), str(prior/name/'trajectory.csv'), str(case/'trajectory.csv'), '0', mode]
        start = time.monotonic()
        try:
            run = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=90)
            output, code = run.stdout + run.stderr, run.returncode
        except subprocess.TimeoutExpired as exc:
            output = (exc.stdout or b'').decode(errors='replace') if isinstance(exc.stdout, bytes) else (exc.stdout or '')
            output += '\nPILOT_TIMEOUT\n'
            code = None
        elapsed = round(time.monotonic()-start, 3)
        (case/'planner.log').write_text(output)
        match = re.search(r'YAW_CANDIDATE_(PASS|REJECT) ([^\n]*)', output)
        results.append({'case': name, 'candidate': mode, 'return_code': code, 'wall_seconds': elapsed,
                        'result': match.group(1) if match else 'TIMEOUT_OR_ERROR',
                        'detail': match.group(2) if match else None,
                        'physical_attachment': 'UNTESTED_1MM_VIRTUAL_GAP'})
        print(json.dumps(results[-1]), flush=True)
for name in ['S3_y_-0.235', 'S3_y_-0.220', 'S3_y_+0.220', 'S3_y_+0.235']:
    case = out / name / 'left_diagnostic'
    case.mkdir(parents=True)
    start=time.monotonic()
    run=subprocess.run([str(build/'alignment_diagnostic'),str(urdf),str(srdf),str(prior/name/'fixture.json'),str(case/'trajectory.csv')],env=env,text=True,capture_output=True,timeout=90)
    log=run.stdout+run.stderr
    (case/'planner.log').write_text(log)
    debug=re.findall(r'(?:ALIGN_DEBUG|CONTACT) [^\n]*',log)
    results.append({'case':name, 'candidate':'original_left_precontact_diagnostic', 'return_code':run.returncode,
                    'wall_seconds':round(time.monotonic()-start,3),'diagnostics':debug,
                    'result':'PASS' if run.returncode==0 else 'REJECT'})
    print(json.dumps(results[-1]),flush=True)
for name in ['S3_y_-0.235', 'S3_y_-0.220']:
    case = out / name / 'right_alternative'
    case.mkdir(parents=True)
    start=time.monotonic()
    run=subprocess.run([str(build/'opposite_arm'),str(urdf),str(srdf),str(prior/name/'fixture.json'),str(case/'trajectory.csv')],env=env,text=True,capture_output=True,timeout=90)
    log=run.stdout+run.stderr
    (case/'planner.log').write_text(log)
    results.append({'case':name, 'candidate':'opposite_right_precontact', 'return_code':run.returncode,
                    'wall_seconds':round(time.monotonic()-start,3),
                    'diagnostics':re.findall(r'^(?:ALIGN_DEBUG|CONTACT|PRECONTACT_REJECT|PRECONTACT_PASS) [^\n]*',log,re.M),
                    'result':'PASS' if run.returncode==0 else 'REJECT'})
    print(json.dumps(results[-1]),flush=True)
metadata = {'ros_domain_id' :int(env['ROS_DOMAIN_ID']), 'seed':28, 'results':results,
            'sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [build/'yaw_planner',build/'alignment_diagnostic',build/'opposite_arm',urdf,srdf]}}
(out/'results.json').write_text(json.dumps(metadata,indent=2)+'\n')
assert len(results)==20
