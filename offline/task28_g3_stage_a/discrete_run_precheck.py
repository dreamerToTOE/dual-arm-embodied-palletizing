"""Screen a recorded motion-start reference using frozen discrete FCL samples."""
import argparse
import hashlib
import json
import lzma
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.collision import CollisionValidator

parser = argparse.ArgumentParser()
parser.add_argument('run_dir', type=Path)
args = parser.parse_args()
BASE = args.run_dir / 'task27_torque_bridge.jsonl'
OUT = args.run_dir / 'discrete_run_precheck.json'
SIDES = ('left', 'right')
trace = BASE if BASE.exists() else Path(str(BASE) + '.xz')
reader = lzma.open if trace.suffix == '.xz' else open
start = frame = None
with reader(trace, 'rt') as stream:
    for line in stream:
        row = json.loads(line)
        if row.get('event') == 'SUSPENDED_PROBE_MOTION_START':
            if start is not None:
                raise RuntimeError('MULTIPLE_MOTION_START_EVENTS')
            start = row
        elif start and row.get('event') == 'TORQUE_FRAME' and row.get('physics_step') == start['physics_step']:
            frame = row
            break
if start is None or frame is None:
    raise RuntimeError('MOTION_START_OR_MATCHING_PHYSICS_STEP_MISSING')
if abs(frame['time_s'] - start['time_s']) > 1e-8:
    raise RuntimeError('TIME_MISMATCH')
model = RobotModel()
collision = CollisionValidator(model.fixture_dir)
q0 = {s: np.asarray(start['ik'][s]['q_start'], float) for s in SIDES}
q1 = {s: np.asarray(start['ik'][s]['q_end'], float) for s in SIDES}
T0 = {s: model.fk(s, q0[s]) for s in SIDES}
rel0 = np.linalg.inv(T0['left']) @ T0['right']
cube = frame['cube_runtime']['pose_world']
Tc0 = np.eye(4)
Tc0[:3, 3] = cube['position_m']
# Isaac get_world_pose returns wxyz, although this historic log field says xyzw.
Tc0[:3, :3] = Rotation.from_quat(np.asarray(cube['orientation_xyzw'])[[1, 2, 3, 0]]).as_matrix()
if np.linalg.norm(Tc0[:3, 3] - start['cube_start_world_m']) > 1e-7:
    raise RuntimeError('CUBE_POSE_MISMATCH')
amp = float(start['amplitude_world_x_m'])
solutions = {}
for side in SIDES:
    if np.max(np.abs(q0[side] - np.asarray(frame['q_rad'][side]))) > 1e-7:
        raise RuntimeError('START_JOINT_MISMATCH:' + side)
    solutions[side] = {'q_start_rad': q0[side].tolist(), 'q_end_rad': q1[side].tolist(),
        'max_joint_delta_rad': float(np.max(np.abs(q1[side] - q0[side]))),
        'model_vs_live_tcp_translation_m': float(np.linalg.norm(T0[side][:3, 3] - frame['tcp_world_m'][side])),
        'ik_translation_error_m': start['ik'][side]['translation_error_m'],
        'ik_rotation_error_rad': start['ik'][side]['rotation_error_rad']}
rows = []
for alpha in np.linspace(0, 1, 11):
    qs = {s: (1-alpha)*q0[s] + alpha*q1[s] for s in SIDES}
    Ts = {s: model.fk(s, qs[s]) for s in SIDES}
    rel = np.linalg.inv(Ts['left']) @ Ts['right']
    Tc = Tc0.copy()
    Tc[0, 3] += amp * alpha
    check = collision.validate(model.forward(qs['left'], qs['right']), Tc, 'bilateral')
    rows.append({'fraction': float(alpha), 'cube_world_position_m': Tc[:3,3].tolist(),
        'joint_limits_valid': all(model.within_limits(qs[s]) for s in SIDES),
        'collision_valid': check.valid, 'collision_reasons': list(check.reasons),
        'accepted_collision_pairs': len(check.accepted_attachment_contacts),
        'tcp_vs_ideal_translation_error_m': {s: float(np.linalg.norm(Ts[s][:3,3] - T0[s][:3,3] - np.array([amp*alpha,0,0]))) for s in SIDES},
        'relative_tcp_translation_change_m': float(np.linalg.norm(rel[:3,3] - rel0[:3,3])),
        'relative_tcp_rotation_change_rad': float(np.linalg.norm(Rotation.from_matrix(rel0[:3,:3].T @ rel[:3,:3]).as_rotvec()))})
hasher = hashlib.sha256()
with trace.open('rb') as stream:
    for block in iter(lambda: stream.read(1024 * 1024), b''):
        hasher.update(block)
result = {'scope': 'actual recorded reference start; 11 frozen-model discrete FCL samples, not continuous CCD or actual execution safety',
    'source_trace': str(trace), 'source_trace_sha256': hasher.hexdigest(),
    'source_physics_step': start['physics_step'], 'source_time_s': start['time_s'],
    'method': 'actual recorded q_start/q_end interpolate simultaneously; translate same-step Cube pose by +X target amplitude times fraction',
    'amplitude_m': amp, 'fixture': str(model.fixture_dir), 'solutions': solutions, 'rows': rows,
    'all_joint_limits_valid': all(r['joint_limits_valid'] for r in rows),
    'all_discrete_collision_valid': all(r['collision_valid'] for r in rows),
    'limitations': ['frozen Task28 single Cube and table; excludes other Task27 objects, truck and live contacts',
        'bilateral cup contact uses strict frozen geometry and may not match PhysX offset',
        'only 11 static samples, not continuous collision detection, nor measured dynamics',
        'Cube motion here is an assumed target, not a measured trajectory']}
OUT.write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps({'output': str(OUT), 'physics_step': start['physics_step'],
    'all_discrete_collision_valid': result['all_discrete_collision_valid'],
    'all_joint_limits_valid': result['all_joint_limits_valid'],
    'max_tcp_ideal_error_um': max(x for r in rows for x in r['tcp_vs_ideal_translation_error_m'].values())*1e6,
    'max_relative_translation_change_um': max(r['relative_tcp_translation_change_m'] for r in rows)*1e6,
    'max_relative_rotation_change_rad': max(r['relative_tcp_rotation_change_rad'] for r in rows)}, indent=2))
