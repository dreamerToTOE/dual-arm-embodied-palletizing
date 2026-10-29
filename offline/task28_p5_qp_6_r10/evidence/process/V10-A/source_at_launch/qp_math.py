"""R10 closed-loop cooperative-QP mathematics.

This module is CPU-only math.  It does not authorize actuation, contact,
suction, or inverse-dynamics claims.  Twists use the world spatial ordering
``[linear velocity, angular velocity]`` and every pose error is transformed
explicitly before it enters a task or constraint.
"""
from __future__ import annotations

import time
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import linprog

from offline.task28_g1_1.task28_validator.se3 import checked_transform, inverse
from offline.task28_p5_qp_0.solver import solve_box_equality_qp
from offline.task28_p5_qp_6_r9.qp_math import object_to_tcp_map, stack_jacobians
from .virtual_frames import VirtualCooperativeFrame


def skew(v):
    x, y, z = np.asarray(v, float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def adjoint(T):
    """Spatial SE(3) adjoint for twist ordering ``[v; omega]``."""
    T = checked_transform(T)
    R, p = T[:3, :3], T[:3, 3]
    return np.block([[R, skew(p) @ R], [np.zeros((3, 3)), R]])


def quintic_profile(t_s, duration_s, amplitude_m):
    """Return ``(s, ds/dt, d2s/dt2)`` for a genuine C2 rest-to-rest move."""
    if not (np.isfinite(t_s) and np.isfinite(duration_s) and np.isfinite(amplitude_m)):
        raise ValueError("INVALID_QUINTIC_INPUT")
    if duration_s <= 0 or amplitude_m < 0:
        raise ValueError("INVALID_QUINTIC_INPUT")
    u = float(np.clip(t_s / duration_s, 0.0, 1.0))
    s = 10.0*u**3 - 15.0*u**4 + 6.0*u**5
    ds_du = 30.0*u**2 - 60.0*u**3 + 30.0*u**4
    d2s_du2 = 60.0*u - 180.0*u**2 + 120.0*u**3
    return float(s), float(amplitude_m * ds_du / duration_s), float(amplitude_m * d2s_du2 / duration_s**2)


def virtual_twist_quintic(t_s, duration_s, amplitude_m, axis="x"):
    """C2 world translation twist and scalar displacement."""
    if axis not in ("x", "z"):
        raise ValueError("INVALID_VIRTUAL_TRAJECTORY_AXIS")
    s, ds, d2s = quintic_profile(t_s, duration_s, amplitude_m)
    out = np.zeros(6)
    out[0 if axis == "x" else 2] = ds
    return out, s, d2s


def virtual_pose_quintic(frame: VirtualCooperativeFrame, t_s, duration_s,
                         amplitude_m, axis="x"):
    _, s, _ = virtual_twist_quintic(t_s, duration_s, amplitude_m, axis)
    T = frame.T_WV.copy()
    T[0 if axis == "x" else 2, 3] += amplitude_m * s
    return T


# Named aliases used by the R10 runtime/evidence schema.  Keeping these
# aliases local to R10 avoids changing the historical R9 cubic implementation.
def quintic_progress(t_s, duration_s):
    s, ds, d2s = quintic_profile(t_s, duration_s, 1.0)
    return s, ds, d2s


def virtual_pose(frame: VirtualCooperativeFrame, t_s, duration_s, amplitude_m,
                 axis="x"):
    return virtual_pose_quintic(frame, t_s, duration_s, amplitude_m, axis)


def native_velocity_handoff(t_s, dq0, transition_s=0.15):
    """C2 zero-net-area transition from observed native dq to zero.

    The quintic polynomial starts with position derivative one and ends with
    both first and second derivatives zero; its position integral is zero at
    both boundaries, so it cannot imprint the R9 cubic handoff drift.
    """
    dq0 = np.asarray(dq0, float)
    if dq0.shape != (14,) or not np.isfinite(dq0).all() or transition_s <= 0:
        raise ValueError("INVALID_HANDOFF_INPUT")
    s = float(np.clip(t_s / transition_s, 0.0, 1.0))
    f = s - 6*s**3 + 8*s**4 - 3*s**5
    df = 1 - 18*s**2 + 32*s**3 - 15*s**4
    d2f = -36*s + 96*s**2 - 60*s**3
    return dq0 * transition_s * f, dq0 * df, dq0 * d2f / transition_s


def _so3_left_jacobian_inverse(phi):
    phi = np.asarray(phi, float)
    theta = float(np.linalg.norm(phi))
    K = skew(phi)
    if theta < 1e-7:
        return np.eye(3) - 0.5*K + (1.0/12.0)*K@K
    # Stable away from the pi branch, which is explicitly rejected below.
    if theta >= np.pi - 1e-7:
        raise ValueError("SE3_LOG_NEAR_PI_BRANCH")
    a = 1.0/theta**2 - (1.0 + np.cos(theta))/(2.0*theta*np.sin(theta))
    return np.eye(3) - 0.5*K + a*K@K


def se3_log(T):
    """Principal logarithm as body-coordinate twist ``[rho; phi]``.

    The translational component uses the inverse SO(3) left Jacobian, so this
    is a proper SE(3) logarithm rather than a translation-plus-rotation norm.
    """
    T = checked_transform(T)
    phi = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    rho = _so3_left_jacobian_inverse(phi) @ T[:3, 3]
    return np.r_[rho, phi]


def pose_error_world(T_actual, T_desired):
    """Body-log pose error mapped to the actual pose's world spatial frame."""
    actual, desired = checked_transform(T_actual), checked_transform(T_desired)
    e_body = se3_log(inverse(actual) @ desired)
    return adjoint(actual) @ e_body


def relative_pose_error(frame: VirtualCooperativeFrame, T_WL, T_WR):
    """Return actual/desired TCP relative pose and body error.

    Both transforms map the right TCP frame into the left TCP frame.  The
    nominal relative transform is ``inv(T_VL) @ T_VR``; reversing this order is
    a frame-convention error and is intentionally covered by negative tests.
    """
    T_WL, T_WR = checked_transform(T_WL), checked_transform(T_WR)
    actual = inverse(T_WL) @ T_WR
    desired = inverse(frame.T_VL) @ frame.T_VR
    return actual, desired, se3_log(inverse(actual) @ desired)


def lookahead_velocity_bounds(model, q14, dt, qdot_limit=0.12, position_margin=0.0):
    """Intersect velocity limits with frozen joint-position lookahead bounds."""
    q = np.asarray(q14, float)
    if q.shape != (14,) or not np.isfinite(q).all() or dt <= 0:
        raise ValueError("INVALID_LOOKAHEAD_STATE")
    if not np.isfinite(position_margin) or position_margin < 0:
        raise ValueError("INVALID_LOOKAHEAD_MARGIN")
    qlo = np.r_[model.joint_lower, model.joint_lower] + float(position_margin)
    qhi = np.r_[model.joint_upper, model.joint_upper] - float(position_margin)
    if np.any(qlo > qhi):
        raise ValueError("INVALID_LOOKAHEAD_MARGIN")
    base_lo, base_hi = np.full(14, -float(qdot_limit)), np.full(14, float(qdot_limit))
    lower = np.maximum(base_lo, (qlo - q) / float(dt))
    upper = np.minimum(base_hi, (qhi - q) / float(dt))
    return lower, upper


def _pair_world_relative_matrix(T_WL, T_WR):
    """Map stacked world TCP twists to relative body twist in the left TCP."""
    # First obtain the relative spatial twist expressed in L, then map it to
    # the relative transform's body coordinates.
    TL = checked_transform(T_WL)
    TR = checked_transform(T_WR)
    TLR = inverse(TL) @ TR
    D_spatial_L = np.hstack((-adjoint(inverse(TL)), adjoint(inverse(TL))))
    return adjoint(inverse(TLR)) @ D_spatial_L


def _solve_affine_box_qp(H, f, C, b, lower, upper):
    """Solve ``min .5*x'Hx+f'x`` with ``C*x=b`` and a box.

    The repository solver intentionally exposes homogeneous equalities only.
    A separately solved LP first obtains an in-box affine-feasible point ``x0``;
    the strict QP is then solved in ``y=x-x0`` with ``C*y=0``.  The returned
    diagnostics retain the affine feasibility and residuals explicitly.
    """
    n = len(f)
    feasible = linprog(np.zeros(n), A_eq=C, b_eq=b,
                       bounds=list(zip(lower, upper)), method="highs",
                       options={"time_limit": 10.})
    if not feasible.success or feasible.x is None:
        return None, {"status": "QP_INFEASIBLE", "affine_feasibility": {
            "success": bool(feasible.success), "exit_status": int(feasible.status),
            "message": str(feasible.message)}, "execution_authorized": False}
    x0 = np.asarray(feasible.x, float)
    shifted = solve_box_equality_qp(H, H@x0 + f, C, lower-x0, upper-x0,
                                    primal_tolerance=2e-8, kkt_tolerance=2e-7)
    if shifted.x is None:
        return None, dict(shifted.diagnostics, status=shifted.status,
                          affine_feasibility={"success": True, "seed": x0.tolist()})
    x = x0 + np.asarray(shifted.x)
    diag = dict(shifted.diagnostics)
    diag.update({"status": shifted.status,
                 "affine_feasibility": {"success": True, "seed": x0.tolist()},
                 "affine_equality_residual_inf": float(np.max(np.abs(C@x-b), initial=0.)),
                 "execution_authorized": False})
    return x, diag


def solve_pose_aware_qp(model, q14, frame, *args, **kwargs):
    """Solve a centralized 14-DOF QP with measured pose feedback.

    The relative-pose correction is a hard equality.  If it cannot satisfy the
    current box/lookahead bounds, the solver returns no command and the caller
    must fail closed.  This function remains a kinematic QP and marks every
    output as unauthorized for execution until external source/geometry/effort
    gates pass.
    """
    # Accept the explicit R10 form and the preregistered runtime's equivalent
    # positional form.  The latter is converted here so frame conventions and
    # gains have one implementation, rather than two subtly different paths.
    if len(args) >= 3 and all(np.asarray(args[i]).shape == (4, 4) for i in range(3)):
        T_WL_actual, T_WR_actual, T_WV_des = args[:3]
        virtual_feedforward = kwargs.pop("virtual_feedforward", None)
        previous = kwargs.pop("previous", None)
        dt = kwargs.pop("dt", 1/60); qdot_limit = kwargs.pop("qdot_limit", .12)
        position_margin = kwargs.pop("position_margin", .02)
        object_gain = kwargs.pop("object_gain", None); relative_gain = kwargs.pop("relative_gain", None)
    elif len(args) >= 6 and "measured_tcp" in kwargs:
        # Legacy runtime call: (T_des, t_ref, duration, amplitude, axis,
        # previous_qdot, measured_tcp=..., scalar feedback gains...).
        T_WV_des, tref, duration, amplitude, axis, previous = args[:6]
        measured = kwargs.pop("measured_tcp")
        T_WL_actual, T_WR_actual = measured["left"], measured["right"]
        virtual_feedforward = virtual_twist_quintic(float(tref), float(duration),
                                                    float(amplitude), str(axis))[0]
        kt = float(kwargs.pop("common_translation_gain", 0.0))
        kr = float(kwargs.pop("common_rotation_gain", 0.0))
        rt = float(kwargs.pop("relative_translation_gain", 0.0))
        rr = float(kwargs.pop("relative_rotation_gain", 0.0))
        object_gain = np.diag([kt, kt, kt, kr, kr, kr])
        relative_gain = np.diag([rt, rt, rt, rr, rr, rr])
        dt = float(kwargs.pop("dt", 1/60)); qdot_limit = float(kwargs.pop("qdot_limit", .12))
        position_margin = float(kwargs.pop("position_margin", .02))
    else:
        raise TypeError("R10_QP_ARGUMENT_CONVENTION")
    correction_linear_limit = kwargs.pop("correction_linear_limit", None)
    correction_angular_limit = kwargs.pop("correction_angular_limit", None)
    if kwargs:
        raise TypeError("R10_QP_UNKNOWN_KWARGS:" + ",".join(sorted(kwargs)))
    start = time.perf_counter()
    q = np.asarray(q14, float)
    if q.shape != (14,) or not np.isfinite(q).all() or previous is not None and np.asarray(previous).shape != (14,):
        raise ValueError("INVALID_QP_STATE")
    T_WL = checked_transform(T_WL_actual)
    T_WR = checked_transform(T_WR_actual)
    T_des = checked_transform(T_WV_des)
    previous = np.zeros(14) if previous is None else np.asarray(previous, float)
    object_gain = np.diag([4., 4., 4., 2., 2., 2.]) if object_gain is None else np.asarray(object_gain, float)
    relative_gain = np.diag([8., 8., 8., 4., 4., 4.]) if relative_gain is None else np.asarray(relative_gain, float)
    if object_gain.shape != (6, 6) or relative_gain.shape != (6, 6):
        raise ValueError("INVALID_FEEDBACK_GAIN")
    if virtual_feedforward is None:
        virtual_feedforward = np.zeros(6)
    virtual_feedforward = np.asarray(virtual_feedforward, float)
    if virtual_feedforward.shape != (6,) or not np.isfinite(virtual_feedforward).all():
        raise ValueError("INVALID_VIRTUAL_FEEDFORWARD")

    J = stack_jacobians(model, q)
    T_WVL = T_WL @ inverse(frame.T_VL)
    T_WVR = T_WR @ inverse(frame.T_VR)
    eL = pose_error_world(T_WVL, T_des)
    eR = pose_error_world(T_WVR, T_des)
    e_obj = 0.5*(eL + eR)
    correction = object_gain @ e_obj
    if correction_linear_limit is not None:
        correction[:3] = np.clip(correction[:3], -float(correction_linear_limit), float(correction_linear_limit))
    if correction_angular_limit is not None:
        correction[3:] = np.clip(correction[3:], -float(correction_angular_limit), float(correction_angular_limit))
    v_obj = virtual_feedforward + correction
    H_L = object_to_tcp_map(T_des, frame.T_VL)
    H_R = object_to_tcp_map(T_des, frame.T_VR)
    target = np.r_[H_L @ v_obj, H_R @ v_obj]

    _, _, e_rel = relative_pose_error(frame, T_WL, T_WR)
    D_rel = _pair_world_relative_matrix(T_WL, T_WR)
    C = D_rel @ J
    b_raw = relative_gain @ e_rel
    b = b_raw.copy()
    if correction_linear_limit is not None:
        b[:3] = np.clip(b[:3], -float(correction_linear_limit), float(correction_linear_limit))
    if correction_angular_limit is not None:
        b[3:] = np.clip(b[3:], -float(correction_angular_limit), float(correction_angular_limit))
    Wt = np.eye(12) * 50.
    Ws = np.eye(14) * 0.25
    H = J.T @ Wt.T @ Wt @ J + Ws.T @ Ws + np.eye(14)*1e-8
    f = -J.T @ Wt.T @ Wt @ target - Ws.T @ Ws @ previous
    lower, upper = lookahead_velocity_bounds(model, q, dt, qdot_limit, position_margin)
    x, solve_diag = _solve_affine_box_qp(H, f, C, b, lower, upper)
    diag = dict(solve_diag)
    diag.update({"status": solve_diag.get("status"), "dimension": 14,
                 "hard_constraint_rows": int(C.shape[0]),
                 "hard_relative_pose_target": b.tolist(),
                 "hard_relative_pose_target_raw": b_raw.tolist(),
                 "object_error_left_world": eL.tolist(),
                 "object_error_right_world": eR.tolist(),
                 "object_error_common_world": e_obj.tolist(),
                 "common_target_twist_world": target.tolist(),
                 "lookahead_lower": lower.tolist(), "lookahead_upper": upper.tolist(),
                 "execution_authorized": False, "inverse_dynamics_claim": False,
                 "calculation_time_s": time.perf_counter() - start})
    if x is None:
        return None, diag
    diag.update({"hard_relative_pose_residual_inf": float(np.max(np.abs(C@x-b), initial=0.)),
                 "hard_relative_residual_inf": float(np.max(np.abs(C@x-b), initial=0.)),
                 "common_task_residual_inf": float(np.max(np.abs(J@x-target), initial=0.)),
                 "qdot_inf": float(np.max(np.abs(x), initial=0.)),
                 "qdot_lookahead_feasible": bool(np.all(x >= lower-2e-8) and np.all(x <= upper+2e-8)),
                 "position_lookahead_min_margin_rad": float(np.min(np.minimum(
                     np.r_[model.joint_upper, model.joint_upper] - float(position_margin) - (q + dt*x),
                     (q + dt*x) - (np.r_[model.joint_lower, model.joint_lower] + float(position_margin))))),
                 "hard_constraints": ["finite_source", "14D_joint_velocity_box",
                                       "within_q_position_lookahead", "relative_pose_feedback"]})
    return x, diag

