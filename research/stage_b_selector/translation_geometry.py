#!/usr/bin/env python3
"""Analyze a hypothetical fixed-yaw +X translation, without moving a robot."""
import json
from pathlib import Path
import sys
from pilot import half_extent


def analyze(fixture):
    xyz, quat = fixture['cube_xyz_m'], fixture['cube_quaternion_wxyz']
    # Existing nominal deep-wall acceptance is 1.100 - x <= 0.003 m.
    acceptance_x = 1.100 - .003
    deep_wall_inner_x = 1.160
    assert xyz[0] <= acceptance_x
    first_wall_x = deep_wall_inner_x - half_extent(quat, 0)
    initial_clearance = first_wall_x - xyz[0]
    target_clearance = first_wall_x - acceptance_x
    return {'nominal_acceptance_boundary_x_m': acceptance_x,
            'initial_deep_clearance_m': initial_clearance,
            'deep_clearance_at_acceptance_m': target_clearance,
            'first_deep_wall_touch_x_m': first_wall_x,
            'nominal_gap_at_first_touch_m': 1.100 - first_wall_x,
            'wall_intersection_before_acceptance': target_clearance < -1e-12}


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: translation_geometry.py PILOT_OUTPUT_DIR')
    root = Path(sys.argv[1])
    rows = json.loads((root / 'results.json').read_text())['results']
    out = {}
    for row in rows:
        if row['scenario'] not in ('S2', 'S3'):
            continue
        fixture = json.loads((root / row['case'] / 'fixture.json').read_text())
        out[row['case']] = analyze(fixture)
    assert not out['S2_aligned_control']['wall_intersection_before_acceptance']
    assert out['S2_yaw_+6deg']['wall_intersection_before_acceptance']
    output = root / 'translation_geometry.json'
    with output.open('x') as f:
        json.dump(out, f, indent=2)
        f.write('\n')
    print(json.dumps(out, indent=2))


if __name__ == '__main__':
    main()
