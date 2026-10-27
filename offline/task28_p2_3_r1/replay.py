"""Archived static bilateral *virtual* wrench replay, never a contact observer.

T_A_B maps coordinates B -> A. Each Jacobian is evaluated at the measured
loaded-window mean q and maps to world TCP twist [linear, angular]. The R1
adapter produces the on-robot wrench at that measured TCP. Action/reaction
negates it; moving its moment reference from actual TCP p_a to fixed nominal
TCP p_n adds (p_a - p_n) cross force. P2-1 rotates into the nominal contact
axes and shifts the moment to the unchanged object center. No known applied
load, simulated attachment, or inferred physical contact is an input.

Caller must independently verify calibration provenance, raw file integrity,
identity and actual live pose provenance. This API enforces the old pair and
static contracts and records G1 closure rather than using it as a replay gate.
"""
import numpy as np

from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import checked_transform, residual
from offline.task28_p2_1.grasp import GraspModel
from offline.task28_p2_2.estimator import Contract
from offline.task28_p2_2_r1.adapter import StaticLimits, static_actuator_pair_to_wrench
from offline.task28_p2_3.integration import JOINT_NAMES, _check_pair


def replay_virtual_pair(left, right, world_object, contract, static_limits):
    """Return a mathematical virtual decomposition and G1 diagnostics.

    `live_world_tcp` is separately measured at the loaded-window tail. The
    caller must verify pose/source provenance; this function independently
    compares it to the frozen model at the measured loaded mean q. Each trial
    must be synchronous across arms, and paired trials remain asynchronous.
    """
    _check_pair(left, right, contract)
    if not isinstance(static_limits, StaticLimits):
        raise ValueError('INVALID_STATIC_LIMITS')
    obj = checked_transform(world_object)
    # Archived P2-3 preregistration fixes this reference, not an observed Cube.
    # A caller cannot silently relocate or rotate the virtual object to make
    # the nonclosing measured robot poses appear to describe a rigid grasp.
    if contract.source_kind == 'ISAAC_TWO_ARTICULATIONS_STANDALONE':
        fixed_object = np.eye(4)
        fixed_object[:3, 3] = [0.65, 0., 0.4]
        if not np.allclose(obj, fixed_object, atol=1e-12, rtol=0):
            raise ValueError('ARCHIVED_FIXED_OBJECT_REFERENCE_MISMATCH')
    model = RobotModel()
    grasp = GraspModel.frozen(obj)
    frames = {}
    for kind in ('unloaded', 'loaded'):
        frames[kind] = {}
        for pair in (left, right):
            q = np.asarray(getattr(pair, kind).q_rad, dtype=float)
            if q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all():
                raise ValueError('INVALID_Q:' + pair.side + ':' + kind)
            mean = q.mean(axis=0)
            if not model.within_limits(mean):
                raise ValueError('JOINT_LIMIT_VIOLATION:' + pair.side + ':' + kind)
            frames[kind][pair.side] = model.fk(pair.side, mean)
    geometry = {}
    for kind in ('unloaded', 'loaded'):
        c = evaluate_closure(obj, frames[kind]['left'], frames[kind]['right'])
        geometry[kind] = {
            'valid': c.valid, 'reasons': c.reasons,
            'left_right_translation_m': c.translation_error_m,
            'left_right_rotation_rad': c.rotation_error_rad,
            'left_object_translation_m': c.left_object_translation_error_m,
            'right_object_translation_m': c.right_object_translation_error_m,
            'left_object_rotation_rad': c.left_object_rotation_error_rad,
            'right_object_rotation_rad': c.right_object_rotation_error_rad,
        }
    estimates = {}
    diagnostics = {}
    local = []
    for pair in (left, right):
        q = np.asarray(pair.loaded.q_rad, dtype=float).mean(axis=0)
        live = checked_transform(pair.live_world_tcp)
        fk_t, fk_r = residual(live, frames['loaded'][pair.side])
        if fk_t > contract.max_fk_translation_m or fk_r > contract.max_fk_rotation_rad:
            raise ValueError('LIVE_FK_MISMATCH:' + pair.side)
        nominal = obj @ (grasp.object_left if pair.side == 'left' else grasp.object_right)
        grasp_t, grasp_r = residual(frames['loaded'][pair.side], nominal)
        tcp = pair.side + '_fr3_side_suction_tcp'
        record = static_actuator_pair_to_wrench(
            pair.unloaded, pair.loaded, model.jacobian(pair.side, q),
            contract=Contract(JOINT_NAMES, 'world', tcp, 0., 0.), limits=static_limits,
            calibration=pair.calibration, jacobian_q=q, jacobian_joint_names=JOINT_NAMES,
            frame='world', tcp_point=tcp, jacobian_provenance='frozen G1-1 RobotModel world TCP Jacobian')
        estimate = record['estimate']
        if estimate.rank != 6 or estimate.status != 'FULL_RANK':
            raise ValueError('JACOBIAN_RANK_DEFICIENT:' + pair.side)
        if min(estimate.singular_values) < contract.min_jacobian_singular_value:
            raise ValueError('JACOBIAN_SINGULAR_VALUE_BELOW_THRESHOLD:' + pair.side)
        on_robot = np.asarray(estimate.wrench_minimum_norm)
        on_cube = -on_robot
        shift = live[:3, 3] - nominal[:3, 3]
        nominal_world = np.r_[on_cube[:3], on_cube[3:] + np.cross(shift, on_cube[:3])]
        local_wrench = grasp.world_to_contact(pair.side, nominal_world)
        local.append(local_wrench)
        estimates[pair.side] = {
            'adapter': record, 'on_robot_world_tcp': on_robot,
            'actual_world_tcp': live, 'nominal_world_tcp': nominal,
            'on_cube_world_nominal_tcp': nominal_world,
            'reference_shift_actual_minus_nominal_m': shift,
            'on_cube_local_tcp': local_wrench,
        }
        diagnostics[pair.side] = {'fk_translation_m': fk_t, 'fk_rotation_rad': fk_r,
                                  'grasp_translation_m': grasp_t, 'grasp_rotation_rad': grasp_r}
    decomposition = grasp.decompose(np.concatenate(local))
    return {
        'scope': 'VIRTUAL_CONTACT_STATIC', 'execution_authorized': False,
        'physical_contact_measured': False, 'same_trial_arms_synchronous': True,
        'unloaded_loaded_physically_synchronous': False,
        'frozen_g1_geometry': geometry,
        'left': estimates['left'], 'right': estimates['right'],
        'local_contacts_on_cube': np.concatenate(local),
        'net_cube_wrench_world_center': decomposition.net_wrench,
        'weighted_external_local': decomposition.external,
        'weighted_internal_local': decomposition.internal,
        'grasp_null_residual': decomposition.null_residual,
        'decomposition_reconstruction_residual': decomposition.reconstruction_residual,
        'weighted_orthogonality': decomposition.metric_orthogonality,
        'diagnostics': diagnostics,
    }
