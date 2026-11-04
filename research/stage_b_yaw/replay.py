#!/usr/bin/env python3
"""Replay every completed candidate and reject an intentionally damaged path."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
prior = root / 'research/stage_b_selector/evidence/pilot_v1'
if len(sys.argv) != 5:
    raise SystemExit('usage: replay.py VERIFY_BINARY URDF SRDF PILOT_DIR')
binary, urdf, srdf, pilot = map(Path, sys.argv[1:])
rows = json.loads((pilot/'results.json').read_text())['results']
checks=[]
for row in rows:
    if row.get('candidate') not in ('direct', 'correct') or row['result'] != 'PASS':
        continue
    name, mode = row['case'], row['candidate']
    command=[str(binary),str(urdf),str(srdf),str(prior/name/'fixture.json'),
             str(prior/name/'trajectory.csv'),str(pilot/name/mode/'trajectory.csv'),mode]
    result=subprocess.run(command,text=True,capture_output=True,timeout=30)
    if result.returncode:
        raise RuntimeError(f'{name} {mode}: {result.stdout} {result.stderr}')
    report=[line for line in result.stdout.splitlines() if line.startswith('REPLAY_PASS ')]
    assert len(report)==1
    checks.append({'case':name,'mode':mode,'result':report[0]})
assert len(checks)==10
name='S2_aligned_control'
with tempfile.TemporaryDirectory(prefix='stage_b_yaw_replay_') as directory:
    source=pilot/name/'direct/trajectory.csv'
    data=list(csv.reader(source.open()))
    data[1][8]=str(float(data[1][8])+.1)
    broken=Path(directory)/'invalid.csv'
    with broken.open('w',newline='') as f: csv.writer(f).writerows(data)
    run=subprocess.run([str(binary),str(urdf),str(srdf),str(prior/name/'fixture.json'),
                        str(prior/name/'trajectory.csv'),str(broken),'direct'],text=True,capture_output=True,timeout=30)
    assert run.returncode != 0 and 'joint step exceeded' in run.stderr
    negative={'result':'rejected', 'reason':'joint step exceeded'}
result={'passed':checks,'negative_control':negative}
with (pilot/'replay_checks.json').open('x') as f:
    json.dump(result,f,indent=2); f.write('\n')
print(json.dumps(result,indent=2))
