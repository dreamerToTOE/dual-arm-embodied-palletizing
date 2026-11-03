#!/usr/bin/env python3
import json
import math
from pathlib import Path
import subprocess
import sys

if len(sys.argv) != 3:
    raise SystemExit('usage: run_cube_sweep.py CUBE_SWEEP_BINARY PILOT_OUTPUT_DIR')
root = Path(sys.argv[2])
rows = json.loads((root / 'results.json').read_text())['results']
lines = []
for row in rows:
    if row['scenario'] not in ('S2', 'S3'):
        continue
    f = json.loads((root / row['case'] / 'fixture.json').read_text())
    q = f['cube_quaternion_wxyz']
    assert q[1] == q[2] == 0
    lines.append(' '.join(map(str, [row['case'], *f['cube_xyz_m'], 2*math.atan2(q[3], q[0])])))
run = subprocess.run([sys.argv[1]], input='\n'.join(lines) + '\n', text=True, capture_output=True, check=True)
with (root / 'cube_sweep_fcl.csv').open('x') as f:
    f.write(run.stdout)
print(run.stdout, end='')
