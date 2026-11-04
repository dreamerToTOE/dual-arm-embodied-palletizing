#!/usr/bin/env python3
"""Evaluate a geometry-derived yaw rule only on the recorded conditional plans."""
import json
import math
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit('usage: rule_compare.py PILOT_DIR')
root = Path(__file__).resolve().parents[2]
prior = root/'research/stage_b_selector/evidence/pilot_v1'
pilot = Path(sys.argv[1])
rows=json.loads((pilot/'results.json').read_text())['results']
candidates={(r['case'],r['candidate']):r for r in rows if r['candidate'] in ('direct','correct')}
threshold=math.degrees(math.asin((1+.003/.06)/math.sqrt(2))-math.pi/4)
assert 2 < threshold < 4
comparison=[]
for name in ['S2_aligned_control','S2_yaw_-2deg','S2_yaw_+2deg','S2_yaw_-4deg','S2_yaw_+4deg','S2_yaw_-6deg','S2_yaw_+6deg']:
    q=json.loads((prior/name/'fixture.json').read_text())['cube_quaternion_wxyz']
    yaw=math.degrees(2*math.atan2(q[3],q[0]))
    chosen='correct' if abs(yaw)>threshold else 'direct'
    comparison.append({'case':name,'yaw_deg':yaw,'always_direct':candidates[name,'direct']['result'],
                       'rule_action':chosen,'rule_result':candidates[name,chosen]['result'],
                       'physical_execution':'UNTESTED'})
assert sum(r['always_direct']=='PASS' for r in comparison)==3
assert all(r['rule_result']=='PASS' for r in comparison)
result={'derived_yaw_threshold_deg':threshold,'derivation':'0.06*(cos(yaw)+sin(|yaw|)-1) > original 0.003 m nominal acceptance',
        'comparison':comparison, 'scope':'conditional virtual attachment plans only'}
with (pilot/'rule_comparison.json').open('x') as f:
    json.dump(result,f,indent=2); f.write('\n')
print(json.dumps(result,indent=2))
