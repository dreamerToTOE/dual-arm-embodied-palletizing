"""Bilateral virtual-contact static integration; never a live force observer.

T_A_B maps column coordinates from B to A. The source-specific R1 adapter
estimates each on-robot wrench about its TCP in world axes. Action/reaction
negates that wrench, shifts its reference from the measured actual TCP to the
nominal TCP, and rotates it to the frozen object-to-TCP local axes; P2-1 then
shifts its moment from the nominal TCP to the Cube center. No applied-load oracle enters
this module. A calibration claim and live FK are evidence supplied by the
caller; this module checks identity and numerical consistency, not their origin.
Separate baseline and loaded trials must not be represented as synchronous.
"""
from dataclasses import dataclass
import numpy as np

from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import checked_transform, residual
from offline.task28_p2_1.grasp import GraspModel
from offline.task28_p2_2.estimator import Contract
from offline.task28_p2_2_r1.adapter import StaticWindow, StaticLimits, SignCalibration, static_actuator_pair_to_wrench

JOINT_NAMES = tuple(f'fr3_joint{i}' for i in range(1, 8))

@dataclass(frozen=True)
class ArmPair:
    side: str
    articulation_id: str
    unloaded: StaticWindow
    loaded: StaticWindow
    unloaded_trial_id: str
    loaded_trial_id: str
    unloaded_steps: tuple[int, ...]
    loaded_steps: tuple[int, ...]
    live_world_tcp: object
    calibration: SignCalibration
    calibration_articulation_id: str
    calibration_verified_for_articulation: bool
    calibration_provenance: str
    live_tcp_provenance: str

@dataclass(frozen=True)
class IntegrationContract:
    left_articulation_id: str
    right_articulation_id: str
    cube_id: str
    left_tcp_id: str
    right_tcp_id: str
    source_kind: str
    max_sync_s: float
    max_fk_translation_m: float
    max_fk_rotation_rad: float
    max_grasp_translation_m: float
    max_grasp_rotation_rad: float
    min_jacobian_singular_value: float
    # The scene has no object contact. This must not be raised to real contact.
    observation_scope: str = 'VIRTUAL_CONTACT_STATIC'


def _positive_limits(contract):
    try:
        a = np.asarray([contract.max_sync_s, contract.max_fk_translation_m,
                        contract.max_fk_rotation_rad, contract.max_grasp_translation_m,
                        contract.max_grasp_rotation_rad, contract.min_jacobian_singular_value], dtype=float)
    except (TypeError, ValueError):
        raise ValueError('INVALID_INTEGRATION_LIMITS') from None
    if not np.isfinite(a).all() or np.any(a < 0):
        raise ValueError('INVALID_INTEGRATION_LIMITS')


def _check_pair(left, right, contract):
    if not isinstance(contract, IntegrationContract):
        raise ValueError('INVALID_INTEGRATION_CONTRACT')
    _positive_limits(contract)
    if contract.cube_id != 'Cube_01' or (contract.left_tcp_id, contract.right_tcp_id) != (
            'left_fr3_side_suction_tcp', 'right_fr3_side_suction_tcp'):
        raise ValueError('INVALID_CUBE_OR_GRASP_FRAME')
    if contract.observation_scope != 'VIRTUAL_CONTACT_STATIC':
        raise ValueError('REAL_CONTACT_NOT_ESTABLISHED')
    if contract.source_kind not in ('SYNTHETIC_TEST', 'ISAAC_TWO_ARTICULATIONS_STANDALONE'):
        raise ValueError('UNSUPPORTED_SOURCE_KIND')
    identities = (contract.left_articulation_id, contract.right_articulation_id)
    if (not all(isinstance(s, str) and s.strip() for s in identities)
            or identities[0] == identities[1]):
        raise ValueError('INVALID_ARTICULATION_IDENTITIES')
    for obs, side, expected in zip((left, right), ('left', 'right'), identities):
        if not isinstance(obs, ArmPair) or obs.side != side or obs.articulation_id != expected:
            raise ValueError('ARM_IDENTITY_MISMATCH')
        if (obs.calibration_articulation_id != expected
                or obs.calibration_verified_for_articulation is not True
                or not isinstance(obs.calibration_provenance, str) or not obs.calibration_provenance.strip()):
            raise ValueError('UNCALIBRATED_ARM_SOURCE')
        if not isinstance(obs.live_tcp_provenance, str) or not obs.live_tcp_provenance.strip():
            raise ValueError('MISSING_LIVE_TCP_PROVENANCE')
        if not all(isinstance(t, str) and t.strip() for t in (obs.unloaded_trial_id, obs.loaded_trial_id)):
            raise ValueError('MISSING_TRIAL_ID')
        if obs.unloaded_trial_id == obs.loaded_trial_id or obs.unloaded is obs.loaded:
            raise ValueError('DUPLICATE_PAIR_EVIDENCE')
        for label in ('unloaded', 'loaded'):
            window = getattr(obs, label)
            steps = getattr(obs, label + '_steps')
            if not isinstance(window, StaticWindow) or not isinstance(steps, tuple) or len(steps) < 2:
                raise ValueError('MISSING_OBSERVATION_WINDOW')
            if (len(steps) != len(window.timestamps_s) or
                    any(type(n) is not int or n < 0 for n in steps) or
                    any(b <= a for a, b in zip(steps, steps[1:]))):
                raise ValueError('INVALID_OBSERVATION_STEPS')
    for label in ('unloaded', 'loaded'):
        if getattr(left, label + '_trial_id') != getattr(right, label + '_trial_id'):
            raise ValueError('MISMATCHED_BILATERAL_TRIAL')
        if getattr(left, label + '_steps') != getattr(right, label + '_steps'):
            raise ValueError('ASYNCHRONOUS_BILATERAL_STEPS')
        l, r = getattr(left, label), getattr(right, label)
        try:
            lt, rt = np.asarray(l.timestamps_s, dtype=float), np.asarray(r.timestamps_s, dtype=float)
            if (lt.shape != rt.shape or not np.isfinite(lt).all() or not np.isfinite(rt).all()
                    or np.max(np.abs(lt - rt)) > contract.max_sync_s):
                raise ValueError('ASYNCHRONOUS_BILATERAL_TIMESTAMPS')
        except (TypeError, OverflowError):
            raise ValueError('INVALID_BILATERAL_TIMESTAMPS') from None


def integrate_virtual_pair(left, right, world_object, contract, static_limits):
    """Estimate/decompose bilateral virtual loads from paired static windows.

    Both arms in a given trial must share an observation window. Baseline and
    loaded trials may have different run IDs, and have no claimed synchrony.
    Jacobians come from the frozen model at each loaded-window mean q. Live TCP
    poses and grasp geometry are independently checked before wrench estimation.
    The returned results are virtual only, regardless of source kind.
    """
    _check_pair(left, right, contract)
    if not isinstance(static_limits, StaticLimits):
        raise ValueError('INVALID_STATIC_LIMITS')
    obj = checked_transform(world_object)
    model = RobotModel()
    grasp = GraspModel.frozen(obj)
    # Enforce the full frozen G1 bilateral closure, including the relative
    # pose of the two independently implied object frames, before estimating.
    from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
    frozen_frames = {}
    for pair in (left, right):
        q_samples = np.asarray(pair.loaded.q_rad, dtype=float)
        if q_samples.ndim != 2 or q_samples.shape[1] != 7 or not np.isfinite(q_samples).all():
            raise ValueError('INVALID_Q')
        q_mean = q_samples.mean(axis=0)
        if not model.within_limits(q_mean):
            raise ValueError('JOINT_LIMIT_VIOLATION')
        frozen = model.fk(pair.side, q_mean)
        expected = obj @ (grasp.object_left if pair.side == 'left' else grasp.object_right)
        grasp_t, grasp_r = residual(frozen, expected)
        if grasp_t > contract.max_grasp_translation_m or grasp_r > contract.max_grasp_rotation_rad:
            raise ValueError('FROZEN_GRASP_MISMATCH:' + pair.side)
        frozen_frames[pair.side] = frozen
    closure = evaluate_closure(obj, frozen_frames['left'], frozen_frames['right'])
    if not closure.valid:
        raise ValueError('FROZEN_G1_CLOSURE_MISMATCH:' + ','.join(closure.reasons))
    unloaded_frames = {}
    for pair in (left, right):
        q_samples = np.asarray(pair.unloaded.q_rad, dtype=float)
        if q_samples.ndim != 2 or q_samples.shape[1] != 7 or not np.isfinite(q_samples).all():
            raise ValueError('INVALID_UNLOADED_Q')
        q_mean = q_samples.mean(axis=0)
        if not model.within_limits(q_mean):
            raise ValueError('UNLOADED_JOINT_LIMIT_VIOLATION')
        unloaded_frames[pair.side] = model.fk(pair.side, q_mean)
    unloaded_closure = evaluate_closure(obj, unloaded_frames['left'], unloaded_frames['right'])
    if not unloaded_closure.valid:
        raise ValueError('UNLOADED_G1_CLOSURE_MISMATCH:' + ','.join(unloaded_closure.reasons))
    estimates = {}
    local = []
    diagnostics = {}
    for pair in (left, right):
        try:
            q = np.asarray(pair.loaded.q_rad, dtype=float)
            if q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all():
                raise ValueError('INVALID_Q')
            mean_q = q.mean(axis=0)
        except (TypeError, OverflowError):
            raise ValueError('INVALID_Q') from None
        if not model.within_limits(mean_q):
            raise ValueError('JOINT_LIMIT_VIOLATION')
        tcp = pair.side + '_fr3_side_suction_tcp'
        live = checked_transform(pair.live_world_tcp)
        frozen = model.fk(pair.side, mean_q)
        fk_t, fk_r = residual(live, frozen)
        if fk_t > contract.max_fk_translation_m or fk_r > contract.max_fk_rotation_rad:
            raise ValueError('LIVE_FK_MISMATCH:' + pair.side)
        expected = obj @ (grasp.object_left if pair.side == 'left' else grasp.object_right)
        grasp_t, grasp_r = residual(frozen, expected)
        if grasp_t > contract.max_grasp_translation_m or grasp_r > contract.max_grasp_rotation_rad:
            raise ValueError('FROZEN_GRASP_MISMATCH:' + pair.side)
        record = static_actuator_pair_to_wrench(
            pair.unloaded, pair.loaded, model.jacobian(pair.side, mean_q),
            contract=Contract(JOINT_NAMES, 'world', tcp, 0., 0.), limits=static_limits,
            calibration=pair.calibration, jacobian_q=mean_q, jacobian_joint_names=JOINT_NAMES,
            frame='world', tcp_point=tcp, jacobian_provenance='frozen G1-1 RobotModel world TCP Jacobian')
        estimate = record['estimate']
        if estimate.rank != 6 or estimate.status != 'FULL_RANK':
            raise ValueError('JACOBIAN_RANK_DEFICIENT:' + pair.side)
        if min(estimate.singular_values) < contract.min_jacobian_singular_value:
            raise ValueError('JACOBIAN_SINGULAR_VALUE_BELOW_THRESHOLD:' + pair.side)
        on_robot_world = np.asarray(estimate.wrench_minimum_norm)
        on_cube_world_actual = -on_robot_world
        reference_shift = live[:3, 3] - expected[:3, 3]
        on_cube_world_nominal = np.r_[on_cube_world_actual[:3],
                                    on_cube_world_actual[3:] + np.cross(reference_shift, on_cube_world_actual[:3])]
        on_cube_local = grasp.world_to_contact(pair.side, on_cube_world_nominal)
        estimates[pair.side] = {'adapter': record, 'on_robot_world_tcp': on_robot_world,
                                'on_cube_world_nominal_tcp': on_cube_world_nominal,
                                'reference_shift_actual_minus_nominal_m': reference_shift,
                                'on_cube_local_tcp': on_cube_local}
        local.append(on_cube_local)
        diagnostics[pair.side] = {'fk_translation_m': fk_t, 'fk_rotation_rad': fk_r,
                                   'grasp_translation_m': grasp_t, 'grasp_rotation_rad': grasp_r}
    decomposition = grasp.decompose(np.concatenate(local))
    return {'scope': 'VIRTUAL_CONTACT_STATIC', 'execution_authorized': False,
            'physical_contact_measured': False, 'same_trial_arms_synchronous': True,
            'unloaded_loaded_physically_synchronous': False,
            'left': estimates['left'], 'right': estimates['right'],
            'local_contacts_on_cube': np.concatenate(local),
            'net_cube_wrench_world_center': decomposition.net_wrench,
            'weighted_external_local': decomposition.external,
            'weighted_internal_local': decomposition.internal,
            'grasp_null_residual': decomposition.null_residual,
            'decomposition_reconstruction_residual': decomposition.reconstruction_residual,
            'weighted_orthogonality': decomposition.metric_orthogonality,
            'diagnostics': diagnostics}
