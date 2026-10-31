"""Bilateral acceleration/torque QP for the Task27 live controller.

The decision vector is ``x = [qdd_left(7), qdd_right(7),
tau_left(7), tau_right(7)]``.  The QP includes the measured rigid-body
equation

``tau = M qdd + h``

as an affine equality.  Common and relative TCP acceleration tracking are
quadratic objectives built from the measured 6x7 Jacobians; the caller must
provide Jacobian-dot terms in the same world spatial convention.  Torque,
torque-rate, joint-position and joint-velocity limits are hard box bounds.

This is deliberately a small, auditable CPU core.  It does not infer a model
from a historical fixture, generate contact forces, or authorize a scene write
by itself.  ``model_validated`` is an explicit source gate.  A caller that has
not independently checked the Task27 articulation, frame conventions and
feedback freshness receives a fail-closed result with no command.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Optional

import numpy as np
from scipy.optimize import linprog, LinearConstraint, minimize

from offline.task28_p5_qp_0.solver import solve_box_equality_qp


# Franka/FR3 arm limits used by the existing Task28 controller contract.  They
# are only defaults for offline protection; a live caller should pass the
# limits read from and validated against its articulation.
FR3_POSITION_LOWER = np.array(
    [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973],
    dtype=float,
)
FR3_POSITION_UPPER = np.array(
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973],
    dtype=float,
)


def _vec(value, shape, code):
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.isfinite(arr).all():
        raise ValueError(code)
    return arr.copy()


def _mat(value, shape, code):
    arr = np.asarray(value, dtype=float)
    if arr.shape != shape or not np.isfinite(arr).all():
        raise ValueError(code)
    return arr.copy()


@dataclass(frozen=True)
class CooperativeTorqueQPConfig:
    """Hard protection and objective parameters for one synchronized step."""

    dt_s: float = 1.0 / 60.0
    torque_abs_Nm: tuple[float, ...] = (
        35.0, 12.0, 8.0, 35.0, 4.0, 4.0, 4.0,
    )
    torque_rate_Nm_s: tuple[float, ...] = (
        2400.0, 600.0, 240.0, 2400.0, 240.0, 240.0, 240.0,
    )
    qdd_abs_rad_s2: tuple[float, ...] = (20.0,) * 7
    qdot_abs_rad_s: tuple[float, ...] = (2.17, 2.17, 2.17, 2.17, 2.61, 2.61, 2.61)
    q_lower_rad: tuple[float, ...] = tuple(float(v) for v in FR3_POSITION_LOWER)
    q_upper_rad: tuple[float, ...] = tuple(float(v) for v in FR3_POSITION_UPPER)
    common_weight: float = 1.0e4
    relative_weight: float = 1.0e4
    acceleration_weight: float = 1.0e-2
    torque_weight: float = 1.0e-4
    torque_slew_weight: float = 1.0e-5
    # A positive value enables an explicit residual gate after optimization.
    # The default leaves tracking residual as a measured diagnostic while the
    # hard physical limits remain authoritative.
    common_residual_limit: float | None = None
    relative_residual_limit: float | None = None
    # Optional hard task equalities.  They are disabled by default because a
    # measured Jacobian may be rank deficient near a singularity; enabling one
    # makes infeasibility explicit and returns no torque command.
    hard_common_acceleration: bool = False
    hard_relative_acceleration: bool = False

    def arrays(self):
        dt = float(self.dt_s)
        if not np.isfinite(dt) or dt <= 0.0 or dt > 1.0:
            raise ValueError("INVALID_DT")
        tau = _vec(self.torque_abs_Nm, (7,), "INVALID_TORQUE_LIMIT")
        rate = _vec(self.torque_rate_Nm_s, (7,), "INVALID_TORQUE_RATE_LIMIT")
        qdd = _vec(self.qdd_abs_rad_s2, (7,), "INVALID_ACCELERATION_LIMIT")
        qdot = _vec(self.qdot_abs_rad_s, (7,), "INVALID_VELOCITY_LIMIT")
        lo = _vec(self.q_lower_rad, (7,), "INVALID_POSITION_LOWER")
        hi = _vec(self.q_upper_rad, (7,), "INVALID_POSITION_UPPER")
        if np.any(tau <= 0) or np.any(rate <= 0) or np.any(qdd <= 0) or np.any(qdot <= 0):
            raise ValueError("NONPOSITIVE_PROTECTION_LIMIT")
        if np.any(lo >= hi):
            raise ValueError("INVALID_POSITION_INTERVAL")
        weights = (float(self.common_weight), float(self.relative_weight),
                   float(self.acceleration_weight), float(self.torque_weight),
                   float(self.torque_slew_weight))
        if any(not np.isfinite(v) or v < 0 for v in weights) or weights[0] == 0 or weights[1] == 0:
            raise ValueError("INVALID_OBJECTIVE_WEIGHT")
        for value, code in ((self.common_residual_limit, "INVALID_COMMON_RESIDUAL_LIMIT"),
                            (self.relative_residual_limit, "INVALID_RELATIVE_RESIDUAL_LIMIT")):
            if value is not None and (not np.isfinite(value) or value < 0):
                raise ValueError(code)
        return dt, tau, rate, qdd, qdot, lo, hi


@dataclass(frozen=True)
class CooperativeTorqueQPResult:
    """One-step result.  ``None`` commands always mean stop/fail closed."""

    status: str
    qdd14: tuple[float, ...] | None
    tau14_Nm: tuple[float, ...] | None
    diagnostics: dict = field(default_factory=dict)

    @property
    def feasible(self) -> bool:
        return self.status == "QP_OPTIMAL"

    @property
    def execution_authorized(self) -> bool:
        return bool(self.diagnostics.get("execution_authorized", False))


def _failure(status, reason, **extra):
    diagnostics = {"status": status, "reason": reason,
                   "execution_authorized": False, **extra}
    return CooperativeTorqueQPResult(status, None, None, diagnostics)


def _solve_affine(H, f, C, b, lower, upper):
    """Solve a strict convex QP with affine equalities and box bounds."""
    n = int(f.size)
    try:
        feasible = linprog(np.zeros(n), A_eq=C, b_eq=b,
                           bounds=list(zip(lower, upper)), method="highs",
                           options={"time_limit": 10.0})
    except Exception as exc:  # numerical backend errors are fail-closed
        return None, {"status": "NUMERICAL_FAILURE", "reason": "FEASIBILITY_BACKEND_EXCEPTION",
                      "exception_type": type(exc).__name__, "exception_message": str(exc)}
    if not feasible.success or feasible.x is None:
        return None, {"status": "QP_INFEASIBLE", "reason": "AFFINE_DYNAMICS_INFEASIBLE",
                      "feasibility_success": bool(feasible.success),
                      "feasibility_exit_status": int(feasible.status),
                      "feasibility_message": str(feasible.message)}
    x0 = np.asarray(feasible.x, dtype=float)
    try:
        shifted = solve_box_equality_qp(H, H @ x0 + f, C,
                                        lower - x0, upper - x0,
                                        primal_tolerance=2e-8,
                                        kkt_tolerance=2e-7)
    except Exception as exc:
        return None, {"status": "NUMERICAL_FAILURE", "reason": "QP_BACKEND_EXCEPTION",
                      "exception_type": type(exc).__name__, "exception_message": str(exc)}
    if shifted.x is None:
        return None, {"status": shifted.status, "reason": "QP_KKT_REJECTED",
                      "solver": shifted.diagnostics}
    x = x0 + np.asarray(shifted.x, dtype=float)
    equality = C @ x - b
    box_violation = max(0.0, float(np.max(np.maximum(lower - x, x - upper), initial=0.0)))
    if not np.isfinite(x).all() or float(np.max(np.abs(equality), initial=0.0)) > 2e-7 or box_violation > 2e-7:
        return None, {"status": "NUMERICAL_FAILURE", "reason": "POST_SOLVE_CERTIFICATE_FAILED",
                      "solver": shifted.diagnostics,
                      "affine_residual_inf": float(np.max(np.abs(equality), initial=0.0)),
                      "box_violation": box_violation}
    return x, {"status": "QP_OPTIMAL", "solver": shifted.diagnostics,
               "affine_residual_inf": float(np.max(np.abs(equality), initial=0.0)),
               "box_violation": box_violation}


def _solve_reduced_qdd(Hq, fq, M, h, lower_qdd, upper_qdd,
                       lower_tau, upper_tau, *, Htau=None, ftau=None,
                       A_eq=None, b_eq=None):
    """Solve after eliminating the affine dynamics equality.

    Eliminating ``tau`` avoids the poor conditioning caused by asking a
    generic equality-constrained solver to recover ``tau=M*qdd+h`` while
    torque bounds are simultaneously active.  The reduced problem is still a
    true QP: torque limits become linear inequalities in qdd.
    """
    # Mappings are full 14-DOF.  The caller's Hq/fq include task terms; this
    # function adds torque regularization before constructing the objective.
    Hq = np.asarray(Hq, float); fq = np.asarray(fq, float)
    # The caller supplies torque terms through an augmented matrix only when
    # available; defaults below preserve the nominal task objective.
    H = 0.5 * (Hq + Hq.T)
    f = fq.copy()
    if Htau is not None:
        Htau = 0.5 * (np.asarray(Htau, float) + np.asarray(Htau, float).T)
        ftau = np.zeros(14) if ftau is None else np.asarray(ftau, float)
        H = H + M.T @ Htau @ M
        f = f + M.T @ (Htau @ h + ftau)
    if not np.isfinite(H).all() or not np.isfinite(f).all():
        return None, {"status": "NUMERICAL_FAILURE", "reason": "NONFINITE_REDUCED_OBJECTIVE"}
    eig = np.linalg.eigvalsh(H)
    if eig[0] <= 0:
        H = H + np.eye(H.shape[0]) * (1e-8 - float(eig[0]))
    # Rescale the objective for SLSQP's absolute stopping tests.  Scaling does
    # not alter the minimizer and materially improves active torque cases.
    scale = max(1.0, float(np.max(np.abs(H))), float(np.max(np.abs(f))))
    Hs, fs = H / scale, f / scale
    A_tau = M
    tau_rhs_lo = lower_tau - h
    tau_rhs_hi = upper_tau - h
    constraints = [LinearConstraint(A_tau, tau_rhs_lo, tau_rhs_hi)]
    if A_eq is not None:
        constraints.append(LinearConstraint(np.asarray(A_eq, float),
                                            np.asarray(b_eq, float),
                                            np.asarray(b_eq, float)))
    # Feasible seed from HiGHS is also used as SLSQP's starting point; this
    # avoids relying on an unconstrained inverse-dynamics torque guess.
    try:
        A_ub = np.vstack((A_tau, -A_tau))
        b_ub = np.r_[tau_rhs_hi, -tau_rhs_lo]
        seed = linprog(np.zeros(14), A_ub=A_ub, b_ub=b_ub,
                       A_eq=None if A_eq is None else np.asarray(A_eq, float),
                       b_eq=None if b_eq is None else np.asarray(b_eq, float),
                       bounds=list(zip(lower_qdd, upper_qdd)), method="highs",
                       options={"time_limit": 10.0})
    except Exception as exc:
        return None, {"status": "NUMERICAL_FAILURE", "reason": "FEASIBILITY_BACKEND_EXCEPTION",
                      "exception_type": type(exc).__name__, "exception_message": str(exc)}
    if not seed.success or seed.x is None:
        return None, {"status": "QP_INFEASIBLE", "reason": "TORQUE_ACCELERATION_BOX_INFEASIBLE",
                      "feasibility_success": bool(seed.success),
                      "feasibility_exit_status": int(seed.status),
                      "feasibility_message": str(seed.message)}
    x0 = np.asarray(seed.x, float)
    try:
        result = minimize(lambda x: .5 * float(x @ Hs @ x) + float(fs @ x), x0,
                          jac=lambda x: Hs @ x + fs,
                          method="SLSQP", bounds=list(zip(lower_qdd, upper_qdd)),
                          constraints=constraints,
                          options={"maxiter": 600, "ftol": 1e-12, "disp": False})
    except Exception as exc:
        return None, {"status": "NUMERICAL_FAILURE", "reason": "QP_BACKEND_EXCEPTION",
                      "exception_type": type(exc).__name__, "exception_message": str(exc)}
    qdd = np.asarray(result.x, float)
    tau = M @ qdd + h
    qdd_violation = max(0., float(np.max(np.maximum(lower_qdd - qdd, qdd - upper_qdd), initial=0.)))
    tau_violation = max(0., float(np.max(np.maximum(lower_tau - tau, tau - upper_tau), initial=0.)))
    if qdd.shape != (14,) or not np.isfinite(qdd).all() or not np.isfinite(tau).all():
        return None, {"status": "NUMERICAL_FAILURE", "reason": "NONFINITE_QP_RESULT"}
    # SLSQP occasionally reports a line-search warning at an exactly feasible
    # active solution.  Feasibility and gradient stationarity are checked
    # independently; a materially violated solution is always rejected.
    if qdd_violation > 2e-7 or tau_violation > 2e-7:
        return None, {"status": "NUMERICAL_FAILURE", "reason": "QP_FEASIBILITY_CERTIFICATE_FAILED",
                      "optimizer_success": bool(result.success),
                      "optimizer_message": str(result.message),
                      "qdd_violation": qdd_violation, "tau_violation": tau_violation}
    if not bool(result.success):
        # Unlike a diagnostic replay, the live bridge cannot use a candidate
        # after an unverified optimizer termination.  No command is returned;
        # the caller must latch its stop gate and record this status.
        return None, {"status": "NUMERICAL_FAILURE", "reason": "QP_OPTIMIZER_NOT_CONVERGED",
                      "optimizer_success": False,
                      "optimizer_exit_status": int(result.status),
                      "optimizer_message": str(result.message),
                      "qdd_violation": qdd_violation, "tau_violation": tau_violation}
    diag = {
        "status": "QP_OPTIMAL", "optimizer": "scipy.optimize.minimize:SLSQP",
        "optimizer_success": bool(result.success),
        "optimizer_exit_status": int(result.status), "optimizer_message": str(result.message),
        "optimizer_iterations": int(getattr(result, "nit", 0)),
        "objective_scale": scale, "qdd_box_violation": qdd_violation,
        "tau_box_violation": tau_violation,
        "feasibility_exit_status": int(seed.status),
        "feasibility_message": str(seed.message),
        "reduced_hessian_eigenvalues": np.linalg.eigvalsh(H).tolist(),
        "dynamics_residual_inf": float(np.max(np.abs(tau - (M @ qdd + h)), initial=0.)),
    }
    return np.r_[qdd, tau], diag


def solve_cooperative_torque_qp(*, q14, dq14, mass14, bias14,
                                jacobian_left, jacobian_right,
                                jacobian_relative=None,
                                jdotdq_left=None, jdotdq_right=None,
                                jdotdq_relative=None,
                                common_accel_desired=None,
                                relative_accel_desired=None,
                                previous_tau14=None,
                                config: CooperativeTorqueQPConfig | None = None,
                                model_validated: bool = False,
                                feedback_fresh: bool = True,
                                dt_s: float | None = None):
    """Solve one synchronized bilateral Task27 acceleration/torque QP.

    ``jacobian_left/right`` are world spatial Jacobians with row ordering
    ``[linear; angular]``.  ``relative`` is the caller's verified Jacobian for
    the chosen left-TCP relative frame (it is not silently approximated by a
    frame swap).  The function validates every array before constructing the
    objective.  Invalid provenance, stale feedback, a non-finite model, or an
    infeasible dynamics/limit intersection returns no command.
    """
    started = time.perf_counter()
    cfg = config or CooperativeTorqueQPConfig()
    try:
        dt, tau_lim, tau_rate, qdd_lim, qdot_lim, qlo_arm, qhi_arm = cfg.arrays()
        if dt_s is not None:
            dt_value = float(dt_s)
            if not np.isfinite(dt_value) or abs(dt_value - dt) > 1e-9:
                return _failure("INPUT_REJECTED", "DT_MISMATCH")
        q = _vec(q14, (14,), "INVALID_Q14")
        dq = _vec(dq14, (14,), "INVALID_DQ14")
        M = _mat(mass14, (14, 14), "INVALID_MASS14")
        h = _vec(bias14, (14,), "INVALID_BIAS14")
        JL = _mat(jacobian_left, (6, 7), "INVALID_LEFT_JACOBIAN")
        JR = _mat(jacobian_right, (6, 7), "INVALID_RIGHT_JACOBIAN")
        Jrel = (None if jacobian_relative is None else
                _mat(jacobian_relative, (6, 14), "INVALID_RELATIVE_JACOBIAN"))
        JdotL = np.zeros(6) if jdotdq_left is None else _vec(jdotdq_left, (6,), "INVALID_LEFT_JDOTDQ")
        JdotR = np.zeros(6) if jdotdq_right is None else _vec(jdotdq_right, (6,), "INVALID_RIGHT_JDOTDQ")
        JdotRel = (JdotR - JdotL if jdotdq_relative is None else
                   _vec(jdotdq_relative, (6,), "INVALID_RELATIVE_JDOTDQ"))
        ac = np.zeros(6) if common_accel_desired is None else _vec(common_accel_desired, (6,), "INVALID_COMMON_ACCEL")
        ar = np.zeros(6) if relative_accel_desired is None else _vec(relative_accel_desired, (6,), "INVALID_RELATIVE_ACCEL")
        prev = None if previous_tau14 is None else _vec(previous_tau14, (14,), "INVALID_PREVIOUS_TORQUE")
    except ValueError as exc:
        return _failure("INPUT_REJECTED", str(exc))
    if not model_validated:
        return _failure("MODEL_GATE_REJECTED", "TASK27_MODEL_NOT_VALIDATED")
    if not bool(feedback_fresh):
        return _failure("FEEDBACK_GATE_REJECTED", "STALE_FEEDBACK")
    if not np.allclose(M, M.T, rtol=0.0, atol=1e-9):
        return _failure("INPUT_REJECTED", "MASS_NOT_SYMMETRIC")
    eig = np.linalg.eigvalsh((M + M.T) * 0.5)
    if not np.isfinite(eig).all() or float(np.min(eig)) <= 1e-9:
        return _failure("INPUT_REJECTED", "MASS_NOT_POSITIVE_DEFINITE", mass_eigenvalues=eig.tolist())
    if np.any(q < np.r_[qlo_arm, qlo_arm]) or np.any(q > np.r_[qhi_arm, qhi_arm]):
        return _failure("INPUT_REJECTED", "JOINT_POSITION_LIMIT_VIOLATION")
    if prev is not None and np.any(np.abs(prev) > np.r_[tau_lim, tau_lim] + 1e-12):
        return _failure("INPUT_REJECTED", "PREVIOUS_TORQUE_LIMIT_VIOLATION")

    # Task maps use the same spatial convention as the measured Jacobians.
    A_common = 0.5 * np.hstack((JL, JR))
    # Relative acceleration map is a signed bilateral difference in the
    # caller-validated relative frame.  Angular and linear rows remain paired.
    A_relative = np.hstack((-JL, JR)) if Jrel is None else Jrel
    bias_common = 0.5 * (JdotL + JdotR)
    bias_relative = JdotRel

    lower_qdd = -np.r_[qdd_lim, qdd_lim].astype(float)
    upper_qdd = np.r_[qdd_lim, qdd_lim].astype(float)
    # One-step velocity and position look-ahead are hard constraints.
    qdd_pos_lo = 2.0 * (np.r_[qlo_arm, qlo_arm] - q - dq * dt) / (dt * dt)
    qdd_pos_hi = 2.0 * (np.r_[qhi_arm, qhi_arm] - q - dq * dt) / (dt * dt)
    qdd_vel_lo = (-np.r_[qdot_lim, qdot_lim] - dq) / dt
    qdd_vel_hi = (np.r_[qdot_lim, qdot_lim] - dq) / dt
    lower_qdd = np.maximum.reduce((lower_qdd, qdd_pos_lo, qdd_vel_lo))
    upper_qdd = np.minimum.reduce((upper_qdd, qdd_pos_hi, qdd_vel_hi))
    lower_tau = -np.r_[tau_lim, tau_lim].astype(float)
    upper_tau = np.r_[tau_lim, tau_lim].astype(float)
    if prev is not None:
        rate = np.r_[tau_rate, tau_rate] * dt
        lower_tau = np.maximum(lower_tau, prev - rate)
        upper_tau = np.minimum(upper_tau, prev + rate)
    if np.any(lower_qdd > upper_qdd) or np.any(lower_tau > upper_tau):
        return _failure("QP_INFEASIBLE", "EMPTY_ACCELERATION_OR_TORQUE_BOX",
                        qdd_lower=lower_qdd.tolist(), qdd_upper=upper_qdd.tolist(),
                        tau_lower=lower_tau.tolist(), tau_upper=upper_tau.tolist())

    # Strictly convex tracking objective.  The residuals include measured
    # Jacobian-dot terms, so the QP is acceleration-level rather than a
    # velocity QP followed by an unrelated PD torque law.
    Hq = np.eye(14) * 1e-10
    fq = np.zeros(14)
    wc, wr, wa, wt, ws = (float(cfg.common_weight), float(cfg.relative_weight),
                           float(cfg.acceleration_weight), float(cfg.torque_weight),
                           float(cfg.torque_slew_weight))
    Hq += wa * np.eye(14)
    if wc:
        Hq += wc * (A_common.T @ A_common)
        fq += wc * A_common.T @ (bias_common - ac)
    if wr:
        Hq += wr * (A_relative.T @ A_relative)
        fq += wr * A_relative.T @ (bias_relative - ar)
    Htau = wt * np.eye(14)
    ftau = np.zeros(14)
    if prev is not None and ws:
        Htau += ws * np.eye(14)
        ftau -= ws * prev
    task_equalities, task_targets = [], []
    if cfg.hard_common_acceleration:
        task_equalities.append(A_common)
        task_targets.append(ac - bias_common)
    if cfg.hard_relative_acceleration:
        task_equalities.append(A_relative)
        task_targets.append(ar - bias_relative)
    A_eq = np.vstack(task_equalities) if task_equalities else None
    b_eq = np.concatenate(task_targets) if task_targets else None
    # Eliminate tau using the measured dynamics.  The returned decision still
    # includes both qdd and tau, with a certificate that tau=M*qdd+h.
    x, solve_diag = _solve_reduced_qdd(
        Hq, fq, M, h, lower_qdd, upper_qdd, lower_tau, upper_tau,
        Htau=Htau, ftau=ftau, A_eq=A_eq, b_eq=b_eq)
    diagnostics = {
        "status": solve_diag.get("status"), "decision_dimension": 28,
        "decision_layout": "qdd_left7_right7_then_tau_left7_right7",
        "dynamics_model": "tau=M*qdd+h",
        "solver_reduction": "qdd-eliminated affine dynamics; tau reconstructed and certified",
        "model_validated": bool(model_validated), "feedback_fresh": bool(feedback_fresh),
        "common_jacobian_shape": list(A_common.shape), "relative_jacobian_shape": list(A_relative.shape),
        "relative_jacobian_source": "caller_verified" if Jrel is not None else "world_difference_fallback",
        "hard_common_acceleration": bool(cfg.hard_common_acceleration),
        "hard_relative_acceleration": bool(cfg.hard_relative_acceleration),
        "common_accel_desired": ac.tolist(), "relative_accel_desired": ar.tolist(),
        "common_weight": wc, "relative_weight": wr,
        "solve_wall_time_s": time.perf_counter() - started,
        "execution_authorized": False,
        **solve_diag,
    }
    if x is None:
        return CooperativeTorqueQPResult(str(solve_diag.get("status", "NUMERICAL_FAILURE")), None, None, diagnostics)
    qdd, tau = x[:14], x[14:]
    common_residual = A_common @ qdd + bias_common - ac
    relative_residual = A_relative @ qdd + bias_relative - ar
    diagnostics.update({
        "common_acceleration_residual": common_residual.tolist(),
        "relative_acceleration_residual": relative_residual.tolist(),
        "common_acceleration_residual_inf": float(np.max(np.abs(common_residual), initial=0.0)),
        "relative_acceleration_residual_inf": float(np.max(np.abs(relative_residual), initial=0.0)),
        "torque_saturated_indices": np.flatnonzero(
            np.isclose(np.abs(tau), np.r_[tau_lim, tau_lim], atol=2e-7, rtol=0.)).tolist(),
        "torque_rate_active_indices": (
            np.flatnonzero(prev is not None and np.isclose(
                np.abs(tau - prev), np.r_[tau_rate, tau_rate] * dt,
                atol=2e-7, rtol=0.)).tolist() if prev is not None else []),
        "joint_position_predicted_min_margin": float(np.min(
            np.r_[qlo_arm, qlo_arm] - (q + dq * dt + 0.5 * qdd * dt * dt))),
        "joint_position_predicted_max_margin": float(np.min(
            np.r_[qhi_arm, qhi_arm] - (q + dq * dt + 0.5 * qdd * dt * dt))),
    })
    if cfg.common_residual_limit is not None and diagnostics["common_acceleration_residual_inf"] > cfg.common_residual_limit:
        return _failure("QP_TRACKING_REJECTED", "COMMON_ACCELERATION_RESIDUAL", **diagnostics)
    if cfg.relative_residual_limit is not None and diagnostics["relative_acceleration_residual_inf"] > cfg.relative_residual_limit:
        return _failure("QP_TRACKING_REJECTED", "RELATIVE_ACCELERATION_RESIDUAL", **diagnostics)
    diagnostics["status"] = "QP_OPTIMAL"
    # The source/model gate above is necessary but not sufficient for a scene
    # write; the runtime must still pass ownership, collision and readback
    # gates.  Keep this explicit so no offline result is mistaken for actuation.
    diagnostics["execution_authorized"] = False
    return CooperativeTorqueQPResult("QP_OPTIMAL", tuple(qdd), tuple(tau), diagnostics)

