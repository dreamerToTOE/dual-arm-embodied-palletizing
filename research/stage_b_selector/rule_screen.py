#!/usr/bin/env python3
"""Replay a partial deterministic screen; never label it a complete plan."""
import json
from pathlib import Path
import sys

if len(sys.argv) != 2:
    raise SystemExit('usage: rule_screen.py PILOT_OUTPUT_DIR')
root = Path(sys.argv[1])
rows = json.loads((root / 'results.json').read_text())['results']
sweep = json.loads((root / 'translation_geometry.json').read_text())
results = []
for row in rows:
    name = row['case']
    if name in sweep and sweep[name]['wall_intersection_before_acceptance']:
        action = 'request_yaw_correction_UNIMPLEMENTED'
    elif not row['precontact_pass']:
        action = 'request_alternative_contact_or_replan_UNVALIDATED'
    else:
        action = 'continue_UNVALIDATED_FULL_SEQUENCE'
    results.append({'case': name, 'partial_screen_action': action,
                    'full_sequence_feasible': None,
                    'candidate_sequences_evaluated': 0})
assert sum(r['partial_screen_action'] == 'request_yaw_correction_UNIMPLEMENTED' for r in results) == 4
assert sum(r['partial_screen_action'] == 'request_alternative_contact_or_replan_UNVALIDATED' for r in results) == 2
with (root / 'rule_screen.json').open('x') as f:
    json.dump(results, f, indent=2)
    f.write('\n')
print(json.dumps(results, indent=2))
