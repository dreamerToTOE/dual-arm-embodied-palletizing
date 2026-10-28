"""Independent archive calculations; no controller, optimizer, or actuator calls."""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure


def array(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError('NONFINITE_OR_WRONG_SHAPE')
    return result


def angle(rotation):
    """Independent SO(3) angle using skew norm and trace (including pi)."""
    r = np.asarray(rotation)
    sine = np.linalg.norm([r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1]]) / 2
    return float(np.arctan2(sine, (np.trace(r)-1)/2))


def sample_original(steps, t):
    """Reconstruct raw QP-3 intervals directly, without QP-5's sampler.

    This is a diagnostic reproduction of its piecewise linear q and geodesic
    object rotation, not a differentiable actuator trajectory.
    """
    if not np.isfinite(t) or t < 0:
        raise ValueError('INVALID_TIME')
    if t >= steps[-1]['target_time_s']:
        return np.asarray(steps[-1]['q_next']), np.zeros(14), np.asarray(steps[-1]['desired_object_pose_next'])
    step = next(s for s in steps if t < s['target_time_s'])
    u = (t-step['time_s'])/step['dt_s']
    q = np.asarray(step['q_initial']) + (t-step['time_s'])*np.asarray(step['qdot14'])
    a, b = np.asarray(step['object_pose_initial']), np.asarray(step['desired_object_pose_next'])
    pose = np.eye(4)
    pose[:3, 3] = a[:3, 3]+u*(b[:3, 3]-a[:3, 3])
    pose[:3, :3] = a[:3, :3] @ Rotation.from_rotvec(u*Rotation.from_matrix(a[:3, :3].T@b[:3, :3]).as_rotvec()).as_matrix()
    return q, np.asarray(step['qdot14']), pose


def closure_diagnostics(obj, left, right):
    """Explicit frozen grasp inverse, checked against unchanged G1.

    Translation decomposition is exact in world axes. Rotation averages are
    small-error diagnostics in object-reference axes, not a global SE(3) mean.
    """
    obj, left, right = (array(t, (4, 4)) for t in (obj, left, right))
    implied = []
    for tcp, rotation, offset in (
        (left, [[0,1,0],[1,0,0],[0,0,-1]], [0,-.061,0]),
        (right, [[0,-1,0],[-1,0,0],[0,0,-1]], [0,.061,0]),
    ):
        rotation = tcp[:3, :3] @ np.asarray(rotation).T
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = tcp[:3, 3]-rotation@offset
        implied.append(transform)
    l, r = implied
    el, er = l[:3, 3]-obj[:3, 3], r[:3, 3]-obj[:3, 3]
    common, diff = (el+er)/2, el-er
    metrics = dict(left_absolute_translation_m=float(np.linalg.norm(el)), right_absolute_translation_m=float(np.linalg.norm(er)),
                   relative_translation_m=float(np.linalg.norm(diff)), left_absolute_rotation_rad=angle(l[:3, :3].T@obj[:3, :3]),
                   right_absolute_rotation_rad=angle(r[:3, :3].T@obj[:3, :3]), relative_rotation_rad=angle(l[:3, :3].T@r[:3, :3]))
    frozen = evaluate_closure(obj, left, right)
    original = [frozen.left_object_translation_error_m, frozen.right_object_translation_error_m, frozen.translation_error_m,
                frozen.left_object_rotation_error_rad, frozen.right_object_rotation_error_rad, frozen.rotation_error_rad]
    disagreement = float(np.max(np.abs(np.array(list(metrics.values()))-original)))
    if disagreement > 1e-12:
        raise ValueError('INDEPENDENT_GRASP_CHECK_DISAGREEMENT')
    rl, rr = [Rotation.from_matrix(obj[:3, :3].T@t[:3, :3]).as_rotvec() for t in implied]
    return dict(metrics=metrics, valid=frozen.valid, reasons=list(frozen.reasons), implied_left_object=l.tolist(), implied_right_object=r.tolist(),
                common_translation_vector_m=common.tolist(), common_translation_m=float(np.linalg.norm(common)),
                differential_translation_vector_m=diff.tolist(), differential_translation_m=float(np.linalg.norm(diff)),
                common_local_rotvec_rad=((rl+rr)/2).tolist(), differential_local_rotvec_rad=(rl-rr).tolist(),
                translation_decomposition_identity_residual_m2=float(abs(np.dot(el,el)+np.dot(er,er)-2*np.dot(common,common)-.5*np.dot(diff,diff))),
                independent_vs_g1_max_abs=disagreement)


def validate_row(row, steps, config, *, r1):
    t = float(row['sim_time_s'])
    q, dq, obj = sample_original(steps, t)
    command_t = float(row['controller_time_s']) if r1 else max(0., t-config['physics_dt_s'])
    cq, cdq, _ = sample_original(steps, command_t)
    if r1:
        if abs(row['post_step_time_s']-t)>1e-10 or abs(t-command_t-config['physics_dt_s'])>1e-7 or abs(row['pre_step']['time_s']-command_t)>1e-10:
            raise ValueError('TIME_ALIGNMENT')
        for field, expected in [('reference_q14', cq), ('reference_qdot14', cdq)]:
            if not np.allclose(array(row['pre_step'][field], (14,)), expected, atol=1e-12, rtol=0):
                raise ValueError('PRE_REFERENCE_MISMATCH')
        if row['reference']['source_identity'] != 'task28_qp3_seed28_transport40_5step':
            raise ValueError('SOURCE_IDENTITY')
    for field, expected in [('q_ref_14', q if r1 else cq), ('qdot_ref_14', dq if r1 else cdq)]:
        if not np.allclose(array(row['reference'][field], (14,)), expected, atol=1e-12, rtol=0):
            raise ValueError('REFERENCE_MISMATCH')
    if not np.allclose(array(row['object_pose_world'], (4,4)), obj, atol=1e-12, rtol=0):
        raise ValueError('OBJECT_REFERENCE_MISMATCH')
    for side in ('left','right'):
        arm = row['arms'][side]
        if arm['dof_names'] != config['expected_all_dof_names']:
            raise ValueError('DOF_ORDER')
        for field in ('q_rad','dq_rad_s','raw_effort','commanded_effort','applied_effort','gravity_pre'):
            array(arm[field], (9,))
    return q, dq, obj, cq, cdq
