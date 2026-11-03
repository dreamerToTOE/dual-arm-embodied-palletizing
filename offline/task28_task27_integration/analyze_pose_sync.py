"""Join the executor's exact acceptance message to its simulation publication."""
import argparse
import json
from pathlib import Path
import re

from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace


SEAT = re.compile(r'TASK28_B0_SEAT_SAMPLE header_ns=(\d+) received_wall_ns=(\d+) acceptance_wall_ns=(\d+) revision=(\d+) frame=(\S+) xyz=\(([^,]+),([^,]+),([^\)]+)\) gap_mm=(\S+)')


def join_evidence(executor, publications):
    match = SEAT.search(executor)
    if not match:
        raise ValueError('executor acceptance sample was not recorded')
    header, received, accepted, revision = map(int, match.group(1, 2, 3, 4))
    matches = [row for row in publications if row['header_ns'] == header]
    if len(matches) != 1:
        raise ValueError(f'acceptance message publication matches: {len(matches)}')
    row = matches[0]
    if match.group(5) != row['frame_id']:
        raise ValueError('acceptance frame does not match publication')
    if not (row['wall_time_ns'] <= received <= accepted):
        raise ValueError('publication, reception, and acceptance timestamps are out of order')
    xyz = list(map(float, match.group(6, 7, 8)))
    if max(abs(a-b) for a, b in zip(xyz, row['ros_usd_xyz_m'])) > 1e-8:
        raise ValueError('acceptance pose does not match published pose')
    return {'header_ns': header, 'received_wall_ns': received, 'acceptance_wall_ns': accepted,
        'revision': revision, 'frame_id': match.group(5), 'executor_xyz_m': xyz,
        'executor_gap_mm': float(match.group(9)), 'publication': row,
        'publish_to_receive_ms': (received - row['wall_time_ns']) / 1e6,
        'receive_to_accept_ms': (accepted - received) / 1e6,
        'usd_minus_rigid_xyz_mm': [1000*(a-b) for a,b in zip(row['ros_usd_xyz_m'], row['rigid_prim_xyz_m'])],
        'rigid_geometry_gap_mm_at_publication': 1000*(1.1-row['rigid_prim_xyz_m'][0])}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with open_trace(args.run) as stream:
        publications = [row for line in stream if (row := json.loads(line)).get('event') == 'B0_CUBE_POSE_PUBLICATION']
    executor = (args.run / 'task27_executor.log').read_text(errors='replace')
    result = join_evidence(executor, publications)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'acceptance_pose_sync.json').write_text(json.dumps(result, indent=2) + '\n')
    accept_wall = result['acceptance_wall_ns']
    with (args.output / 'acceptance_pose_window.jsonl').open('x') as stream:
        for row in publications:
            if abs(row['wall_time_ns']-accept_wall) <= 10_000_000_000:
                stream.write(json.dumps(row) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
