"""Archive-bound bilateral motion references; no controller or actuator sink.

q_d is the accepted interval-end q_next; qdot_d is the QP velocity computed at
interval start and used for Euler integration. It is NOT a certified derivative
of a smooth actuator trajectory. Only the registered five-step archive is in
scope. New trajectories require an explicit source-binding revision.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping
import numpy as np
from offline.task28_g1_1.task28_validator.state import expected_joint_names
from offline.task28_g1_1.task28_validator.model import verify_fixture
from offline.task28_g1_1.task28_validator.se3 import checked_transform

ROOT = Path(__file__).resolve().parents[2]
BINDING = Path(__file__).with_name('source_binding.json')
QP3_PATH = 'offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json'
QP3_RECORD = ROOT / QP3_PATH
FIXTURE_ID = 'task28_g1_0_new_single_cube_v0'
FIXTURE = ROOT / 'fixtures' / FIXTURE_ID
CUBE_ID, FRAME_ID, WORLD_FRAME = 'Cube_01', 'task28_g1_0_dual_sync', 'world'
LEFT_JOINTS = tuple(expected_joint_names('left'))
RIGHT_JOINTS = tuple(expected_joint_names('right'))
JOINT_ORDER = LEFT_JOINTS + RIGHT_JOINTS
IDENTITY = 'task28_qp3_seed28_transport40_5step'
ACCEPTED = 'COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED'
TIME = 'synthetic_monotonic_qp3_candidate_time_not_ros_or_servo_time'
PROVENANCE = 'NR0_SEED28_MIDPOINT40_INITIAL_NODE_THEN_QP2_SYNTHETIC_FIXED_TWIST'
SAFETY = 'NOT_CERTIFIED_FINITE_SAMPLING_ONLY'


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def source_hashes(root: Path = ROOT) -> dict[str, str]:
    """Verify every pinned file; missing sources cannot be silently skipped."""
    binding = json.loads(BINDING.read_text())
    for name, digest in binding['source_hashes'].items():
        p = root / name
        _require(p.is_file(), 'MISSING_SOURCE:' + name)
        _require(sha256(p) == digest, 'SOURCE_HASH_MISMATCH:' + name)
    verify_fixture(root / 'fixtures' / FIXTURE_ID)
    return dict(binding['source_hashes'])


def load_archive(path: Path | str = QP3_RECORD) -> dict[str, Any]:
    hashes = source_hashes()
    p = Path(path)
    _require(p.is_file(), 'MISSING_QP3_ARCHIVE')
    _require(sha256(p) == hashes[QP3_PATH], 'QP3_ARCHIVE_HASH_MISMATCH')
    return json.loads(p.read_text())


def load_qp3_positive(path: Path | str = QP3_RECORD) -> dict[str, Any]:
    return load_archive(path)['positive']


@dataclass(frozen=True)
class ReferenceContract:
    fixture_id: str = FIXTURE_ID
    cube_id: str = CUBE_ID
    world_frame: str = WORLD_FRAME
    reference_frame: str = FRAME_ID  # provenance only, never a bridge route
    left_joint_names: tuple[str, ...] = LEFT_JOINTS
    right_joint_names: tuple[str, ...] = RIGHT_JOINTS
    joint_units: str = 'radians_and_radians_per_second'
    timestamp_semantics: str = TIME
    acceleration_status: str = 'UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION'
    execution_authorized: bool = False
    command_semantics: str = 'motion_reference_only_no_torque_no_position_publish'

    def validate(self):
        defaults = asdict(ReferenceContract())
        for key, value in asdict(self).items():
            _require(_canonical(value) == _canonical(defaults[key]), 'CONTRACT_MISMATCH:' + key)


@dataclass(frozen=True)
class ReferenceFrame:
    sequence: int
    source_step_index: int
    time_s: float
    stamp_s: float
    cube_id: str
    phase: str
    contact_mode: str
    frame_id: str
    object_frame: str
    fixture_id: str
    source_model: Mapping[str, Any]
    left_joint_names: tuple[str, ...]
    right_joint_names: tuple[str, ...]
    left_q_rad: tuple[float, ...]
    right_q_rad: tuple[float, ...]
    left_qdot_rad_s: tuple[float, ...]
    right_qdot_rad_s: tuple[float, ...]
    acceleration_rad_s2: None
    object_pose_world: tuple[tuple[float, ...], ...]
    source_identity: str
    provenance: str
    synchronization: Mapping[str, Any]
    geometry_status: Mapping[str, Any]
    safety_status: str
    execution_authorized: bool = False


def _array(values, shape, label):
    a = np.asarray(values)
    _require(a.shape == shape and a.dtype.kind in 'iuf' and np.isfinite(a).all(), label + '_INVALID')
    return a.astype(float)


def _pose(value):
    a = checked_transform(value)
    return tuple(tuple(float(x) for x in row) for row in a)


def _num(value, label):
    _require(type(value) in (int, float) and math.isfinite(value), label + '_INVALID')
    return float(value)


def _integer(value, expected, label):
    _require(type(value) is int and value == expected, label + '_MISMATCH')


def _build(positive, contract, hashes):
    """Semantic checks before an exact comparison with the pinned source."""
    contract.validate()
    _require(isinstance(positive, Mapping), 'UPSTREAM_MAPPING_REQUIRED')
    _require(positive.get('status') == ACCEPTED, 'UPSTREAM_STATUS_NOT_ACCEPTED')
    _require(positive.get('execution_authorized') is False, 'UPSTREAM_EXECUTION_AUTHORIZED')
    _require(positive.get('identity') == IDENTITY, 'SOURCE_IDENTITY_MISMATCH')
    _require(positive.get('failure_stop') is False and positive.get('rejections') == [], 'UPSTREAM_FAILURE_PRESENT')
    _require(positive.get('timestamp_scope') == 'SYNTHETIC_NUMERICAL_TIME_NOT_ROS_OR_SERVO_TIME', 'UPSTREAM_TIME_SEMANTICS')
    raw = positive.get('raw_qp2', {})
    _require(raw.get('status') == ACCEPTED and raw.get('failure') is None, 'QP2_STATUS_NOT_ACCEPTED')
    _require(raw.get('execution_authorized') is False, 'QP2_EXECUTION_AUTHORIZED')
    _require(raw.get('phase_confirmation_observations_generated') == 0, 'SYNTHETIC_PHASE_CONFIRMATION_FORBIDDEN')
    request = raw.get('request', {})
    _require(request.get('cube_id') == CUBE_ID, 'INCORRECT_CUBE_ID')
    _require(request.get('frame_id') == WORLD_FRAME, 'INCORRECT_REFERENCE_FRAME')
    targets, steps = positive.get('position_targets'), raw.get('steps')
    _require(isinstance(targets, list) and isinstance(steps, list) and len(targets) == len(steps) == 5, 'UPSTREAM_STEP_COUNT_INVALID')
    _integer(raw.get('accepted_step_count'), 5, 'ACCEPTED_COUNT')
    _integer(raw.get('attempted_step_count'), 5, 'ATTEMPTED_COUNT')
    _integer(positive.get('target_count'), 10, 'TARGET_COUNT')
    previous_q = _array(request.get('q14'), (14,), 'INITIAL_Q')
    previous_pose = _pose(request.get('object_pose'))
    previous_stamp = 0.
    binding = json.loads(BINDING.read_text())
    source_model = {k: hashes[v] for k, v in binding['model_hash_paths'].items()}
    source_model.update(fixture_id=FIXTURE_ID, model_path=binding['model_hash_paths']['robot_urdf_sha256'],
                        qp3_record_sha256=hashes[QP3_PATH], nr0_seed28_sha256=hashes[binding['nr0_path']],
                        source_binding_sha256=sha256(BINDING))
    result = []
    for i, (target, step) in enumerate(zip(targets, steps)):
        _integer(step.get('step_index'), i, 'SOURCE_STEP')
        _require(step.get('accepted') is True and step.get('reasons') == [], 'UPSTREAM_STEP_NOT_ACCEPTED')
        _require(step.get('execution_authorized') is False and target.get('execution_authorized') is False, 'EXECUTION_MUST_REMAIN_DISABLED')
        qp = step.get('qp', {})
        _require(qp.get('status') == 'FEASIBLE_TRACKING_OK' and qp.get('execution_authorized') is False, 'UPSTREAM_QP_NOT_ACCEPTED')
        _require(target.get('predicted_state_valid') is True, 'UPSTREAM_STATE_INVALID')
        _require(target.get('sampled_edge_status') == 'sampled_edge_valid' and step.get('sampled_edge', {}).get('status') == 'sampled_edge_valid', 'UPSTREAM_EDGE_INVALID')
        for field in ('current_state', 'predicted_state'):
            state = step.get(field, {})
            _require(state.get('valid') is True and state.get('collision', {}).get('valid') is True and state.get('closure', {}).get('valid') is True, 'UPSTREAM_GEOMETRY_INVALID:' + field)
            _require(state.get('waypoint', {}).get('mode') == 'bilateral_transitioning', 'UPSTREAM_CONTACT_MODE_MISMATCH')
        _require(step.get('frozen_bounds', {}).get('valid') is True, 'UPSTREAM_BOUNDS_INVALID')
        t = _num(step.get('time_s'), 'TIME')
        stamp = _num(step.get('target_time_s'), 'STAMP')
        dt = _num(step.get('dt_s'), 'DT')
        _require(t >= 0 and stamp > t and t == previous_stamp and dt == .02 and abs(stamp-t-dt) <= 1e-15, 'STALE_OR_OUT_OF_ORDER_REFERENCE')
        q0, q, qd = (_array(step.get(k), (14,), k.upper()) for k in ('q_initial', 'q_next', 'qdot14'))
        _require(np.array_equal(q0, previous_q), 'DISCONTINUOUS_Q_INITIAL')
        _require(np.array_equal(qd, _array(qp.get('qdot'), (14,), 'QP_QDOT')), 'QP_QDOT_MISMATCH')
        _require(np.allclose(q, q0+dt*qd, rtol=0, atol=1e-10), 'Q_RECONSTRUCTION_MISMATCH')
        _require(_pose(step.get('object_pose_initial')) == previous_pose, 'DISCONTINUOUS_OBJECT_REFERENCE')
        pose = _pose(step.get('desired_object_pose_next'))
        for side, names, section in (('left', LEFT_JOINTS, slice(0,7)), ('right', RIGHT_JOINTS, slice(7,14))):
            arm = target.get(side, {})
            _require(arm.get('side') == side, 'SIDE_IDENTITY_MISMATCH')
            _require(arm.get('execution_authorized') is False, 'EXECUTION_MUST_REMAIN_DISABLED')
            _require(arm.get('frame_id') == FRAME_ID, 'INCORRECT_REFERENCE_FRAME')
            _require(tuple(arm.get('joint_names', ())) == names, 'JOINT_ORDER_MISMATCH')
            _integer(arm.get('sequence'), i, 'PAIR_SEQUENCE')
            _integer(arm.get('source_step_index'), i, 'PAIR_SOURCE_STEP')
            _require(_num(arm.get('stamp_s'), 'PAIR_STAMP') == stamp and _num(arm.get('time_s'), 'PAIR_TIME') == t, 'ASYNC_REFERENCE_PAIR')
            _require(np.array_equal(_array(arm.get('positions_rad'), (7,), 'PAIR_Q'), q[section]), 'Q_TARGET_MISMATCH')
            _require(np.array_equal(_array(arm.get('source_qdot_rad_s'), (7,), 'PAIR_QDOT'), qd[section]), 'QDOT_TARGET_MISMATCH')
        result.append(ReferenceFrame(
            i, i, t, stamp, CUBE_ID, 'A_STAGE_TRANSPORT_REFERENCE_UNCONFIRMED',
            'bilateral_transitioning', FRAME_ID, WORLD_FRAME, FIXTURE_ID, source_model.copy(),
            LEFT_JOINTS, RIGHT_JOINTS, tuple(q[:7]), tuple(q[7:]), tuple(qd[:7]), tuple(qd[7:]),
            None, pose, IDENTITY, PROVENANCE,
            {'pair_sequence': i, 'left_stamp_s': stamp, 'right_stamp_s': stamp,
             'source_computation_time_s': t, 'time_semantics': TIME,
             'velocity_semantics': 'interval_start_qdot_used_for_Euler_update_not_smooth_endpoint_derivative',
             'phase_confirmation': 'NOT_RUN', 'release_authorized': False},
            {'current_state_valid': True, 'predicted_state_valid': True,
             'sampled_edge_status': 'sampled_edge_valid', 'qp_status': qp['status'],
             'scope': 'HASH_BOUND_ARCHIVED_ACCEPTANCE_NOT_LIVE_FEEDBACK',
             'closure': step['predicted_state']['closure']}, SAFETY, False))
        previous_q, previous_stamp, previous_pose = q, stamp, pose
    return result


def build_reference_sequence(positive, *, contract=ReferenceContract()):
    hashes = source_hashes()
    result = _build(positive, contract, hashes)
    archive = json.loads(QP3_RECORD.read_text())
    _require(_canonical(positive) == _canonical(archive['positive']), 'UPSTREAM_ARCHIVE_CONTENT_MISMATCH')
    return result


def _validate(ref, canonical):
    _require(type(ref) is ReferenceFrame, 'TYPED_REFERENCE_REQUIRED')
    _require(ref.cube_id == CUBE_ID, 'INCORRECT_CUBE_ID')
    _require(ref.frame_id == FRAME_ID and ref.object_frame == WORLD_FRAME, 'INCORRECT_REFERENCE_FRAME')
    _require(ref.left_joint_names == LEFT_JOINTS and ref.right_joint_names == RIGHT_JOINTS, 'JOINT_ORDER_MISMATCH')
    _require(ref.acceleration_rad_s2 is None, 'ACCELERATION_MUST_BE_UNAVAILABLE')
    _require(ref.execution_authorized is False, 'EXECUTION_MUST_REMAIN_DISABLED')
    t, stamp = _num(ref.time_s, 'TIME'), _num(ref.stamp_s, 'STAMP')
    _require(t >= 0 and stamp > t, 'INVALID_INTERVAL')
    _pose(ref.object_pose_world)
    for field in ('left_q_rad', 'right_q_rad', 'left_qdot_rad_s', 'right_qdot_rad_s'):
        _array(getattr(ref, field), (7,), field.upper())
    _require(isinstance(ref.synchronization, Mapping), 'SYNC_METADATA_INVALID')
    _require(ref.synchronization.get('left_stamp_s') == stamp == ref.synchronization.get('right_stamp_s'), 'ASYNC_REFERENCE_PAIR')
    _require(type(ref.sequence) is int and 0 <= ref.sequence < len(canonical), 'REFERENCE_SEQUENCE_GAP')
    _require(_canonical(asdict(ref)) == _canonical(asdict(canonical[ref.sequence])), 'REFERENCE_SOURCE_MISMATCH')


def validate_reference(ref, *, contract=ReferenceContract()):
    expected = build_reference_sequence(load_qp3_positive(), contract=contract)
    _validate(ref, expected)
    return {'valid': True, 'sequence': ref.sequence, 'stamp_s': ref.stamp_s, 'execution_authorized': False}


def validate_sequence(refs: Iterable[ReferenceFrame], *, contract=ReferenceContract(), now_s=None, max_age_s=None):
    """Replay cursor freshness only. No actuator-feedback age is measured here.

    If supplied, now_s/max_age_s MUST both use the synthetic reference clock.
    No physical freshness threshold is set or inferred by this adapter.
    """
    refs = list(refs)
    _require(bool(refs), 'EMPTY_REFERENCE_SEQUENCE')
    expected = build_reference_sequence(load_qp3_positive(), contract=contract)
    _require((now_s is None) == (max_age_s is None), 'FRESHNESS_CLOCK_AND_BUDGET_REQUIRED')
    if now_s is not None:
        now_s, max_age_s = _num(now_s, 'NOW'), _num(max_age_s, 'MAX_AGE')
        _require(max_age_s >= 0, 'NEGATIVE_AGE_BUDGET')
    prev = None
    for i, ref in enumerate(refs):
        _integer(ref.sequence, i, 'REFERENCE_SEQUENCE')
        stamp = _num(ref.stamp_s, 'STAMP')
        _require(prev is None or stamp > prev, 'STALE_OR_OUT_OF_ORDER_REFERENCE')
        _validate(ref, expected)
        prev = stamp
    if now_s is not None:
        _require(0 <= now_s - refs[-1].stamp_s <= max_age_s, 'STALE_OR_FUTURE_REFERENCE')
    _require(len(refs) == len(expected), 'INCOMPLETE_REFERENCE_SEQUENCE')
    return {'valid': True, 'count': len(refs), 'first_stamp_s': refs[0].stamp_s,
            'last_stamp_s': refs[-1].stamp_s, 'execution_authorized': False}


def as_jsonable(refs):
    return [asdict(x) for x in refs]
