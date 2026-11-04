#!/usr/bin/env python3
import json, pathlib, sys
root = pathlib.Path(sys.argv[1])
d = json.loads((root/'results.json').read_text())
assert d['scenario_count'] == 36
assert d['workers'] == 8
rows = d['results']
assert sum(r['m2_two_level']['result'] == 'PASS' for r in rows) == 28
assert sum(r['m2_two_level']['result'] == 'PASS' for r in rows if r['split']=='dev') == 12
assert sum(r['m2_two_level']['result'] == 'PASS' for r in rows if r['split']=='test') == 16
assert all(0.39 <= r['width_m'] <= 0.43 and 0.27 <= r['depth_m'] <= 0.32 for r in rows)
assert all(r[k]['result'] != 'ERROR' for r in rows for k in ('m0_fixed','m1_rule','m2_two_level'))
assert sum(r['level2_decision'] == 'ADVANTAGE_OVER_RULE' for r in rows) == 0
logs = list(root.glob('cases/*/contact_*/*.log'))
yaw_logs = [p for p in logs if 'YAW_CANDIDATE_' in p.read_text(errors='ignore')]
assert len(yaw_logs) == 168
assert all('CUP_FOOTPRINT_PASS' in p.read_text(errors='ignore') for p in yaw_logs)
print('FINAL_EVIDENCE_PASS scenes=36 valid_m2=28 dev=12 test=16 yaw_candidates=168')
