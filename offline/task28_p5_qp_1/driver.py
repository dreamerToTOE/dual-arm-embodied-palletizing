"""Bounded synthetic Euler integration with the unchanged QP-0 and G1 gates.

Every desired object pose is obtained from the initial reference and a fixed
world-at-Cube-center twist. Only jointly accepted numerical and sampled
geometric steps are fed forward. Neither NR projection nor temporal observation
generation is performed. The optional unbounded comparison is a mathematical
reference, never a safe or executable controller.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from enum import Enum
import math
import time
from typing import Any

import numpy as np

from offline.task28_p5_qp_0.core import (
    CentralizedVelocityQP, VelocityInput, VelocityResult, _positive, _vector,
    velocity_maps,
)
from offline.task28_p5_qp_0.integration import integrate_object_world_center
from offline.task28_p5_qp_0.bootstrap import checked_transform, evaluate_closure
from offline.task28_p4_nr_0.validation import CandidateValidator
from task28_edge.types import EdgeStatus, Motion


def _jsonable(value: Any) -> Any:
    """Preserve rejected numeric inputs explicitly without emitting JSON NaN."""
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return {"invalid_numeric_literal": repr(value)}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    return {"unserializable_input_type": type(value).__name__, "repr": repr(value)}


@dataclass(frozen=True)
class MultiStepRequest:
    q14: object
    object_pose: object
    desired_object_twist: object
    dt_s: float
    max_steps: int
    identity: str
    cube_id: str = "Cube_01"
    frame_id: str = "world"
    observation_provenance: dict | None = None


def _closure(model, q, X, grasps):
    return evaluate_closure(X, model.fk("left", q[:7]),
                            model.fk("right", q[7:]), grasps)


class EqualityUnboundedReference:
    """Independent nullspace/Cholesky optimum with NO box constraints.

    The equality and the weighted regularized objective match QP-0. Input and
    full SE(3) closure checks retain the frozen contract. Velocity/lookahead
    limits are explicitly *not* optimization constraints here; MultiStepDriver
    checks them independently and halts before any violating point is reused.
    """
    def __init__(self, controller=None):
        self.controller = controller if controller is not None else CentralizedVelocityQP()
        self.model = self.controller.model
        self.config = self.controller.config

    def solve(self, request):
        started = time.perf_counter()
        if type(request) is not VelocityInput:
            return VelocityResult("INPUT_REJECTED", None, ("INVALID_VELOCITY_INPUT_TYPE",))
        try:
            self.config.__post_init__()
            q = _vector(request.q14, 14, "JOINT_STATE")
            X = checked_transform(request.object_pose)
            desired = _vector(request.desired_object_twist, 6, "DESIRED_OBJECT_TWIST")
            dt = _positive(request.dt_s, "DT")
            if request.cube_id != "Cube_01":
                raise ValueError("INCORRECT_CUBE_ID")
            if request.frame_id != "world":
                raise ValueError("INCORRECT_FRAME_ID")
            if not self.model.within_limits(q[:7]) or not self.model.within_limits(q[7:]):
                raise ValueError("INITIAL_JOINT_LIMIT_VIOLATION")
            grasps = self.controller.grasps if request.grasps is None else request.grasps
            if not (np.array_equal(checked_transform(grasps.left), self.controller.grasps.left)
                    and np.array_equal(checked_transform(grasps.right), self.controller.grasps.right)):
                raise ValueError("FROZEN_GRASP_MISMATCH")
            if request.previous is not None or self.config.acceleration_caps_rad_s2 is not None:
                raise ValueError("ACCELERATION_NOT_SUPPORTED_BY_UNBOUNDED_COMPARISON")
            closure = _closure(self.model, q, X, grasps)
            if not closure.valid:
                return VelocityResult("INPUT_REJECTED", None,
                                      ("INITIAL_SE3_CLOSURE_REJECTED",) + closure.reasons)
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            return VelocityResult("INPUT_REJECTED", None, (str(exc),))
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                maps = velocity_maps(self.model, q, X, grasps)
                B, C = maps["B"], maps["C"]
                cfg = self.config
                W = np.diag([cfg.translation_weight] * 3 +
                            [cfg.translation_weight * cfg.rotation_characteristic_length_m**2] * 3)
                H = B.T @ W @ B + cfg.regularization * np.eye(14)
                f = -B.T @ W @ desired
                # Independent mathematical reference: eliminate the equality,
                # then solve the positive-definite reduced problem by Cholesky.
                _, singular, vt = np.linalg.svd(C, full_matrices=True)
                cutoff = 1e-12 * (float(singular[0]) if len(singular) else 0.)
                rank = int(np.sum(singular > cutoff))
                N = vt[rank:].T
                if N.shape[1]:
                    reduced = N.T @ H @ N
                    L = np.linalg.cholesky(reduced)
                    z = np.linalg.solve(L.T, np.linalg.solve(L, -N.T @ f))
                    x = N @ z
                else:
                    x = np.zeros(14)
                gradient = H @ x + f
                multipliers = np.linalg.lstsq(C.T, -gradient, rcond=1e-12)[0]
                equality_inf = float(np.linalg.norm(C @ x, ord=np.inf))
                stationarity_inf = float(np.linalg.norm(gradient + C.T @ multipliers, ord=np.inf))
                if (not np.isfinite(x).all() or not np.isfinite(multipliers).all()
                        or equality_inf > 1e-8 or stationarity_inf > 1e-7):
                    return VelocityResult("NUMERICAL_FAILURE", None,
                        ("UNBOUNDED_ORIGINAL_EQUALITY_OR_KKT_CHECK_FAILED",),
                        dict(original_equality_inf=equality_inf, stationarity_inf=stationarity_inf))
                actual = B @ x
                error = actual - desired
                t_error = float(np.linalg.norm(error[:3]))
                r_error = float(np.linalg.norm(error[3:]))
                tracking_ok = (t_error <= cfg.tracking_translation_tolerance_m_s
                               and r_error <= cfg.tracking_rotation_tolerance_rad_s)
                diagnostics = dict(
                    comparison_scope="MATHEMATICAL_EQUALITY_ONLY_UNBOUNDED_REFERENCE_NOT_A_CONTROLLER",
                    box_constraints_in_optimization=False, acceleration_constraints_in_optimization=False,
                    B=B.tolist(), C=C.tolist(), weight_matrix=W.tolist(),
                    regularization=cfg.regularization, H=H.tolist(), f=f.tolist(),
                    equality_rank=rank, equality_rank_relative_cutoff=1e-12,
                    singular_values=singular.tolist(), equality_multipliers=multipliers.tolist(),
                    original_equality_inf=equality_inf, stationarity_inf=stationarity_inf,
                    achieved_object_twist=actual.tolist(), desired_object_twist=desired.tolist(),
                    closure_velocity_residual=(C @ x).tolist(),
                    left_implied_object_twist=(maps["B_left"] @ x[:7]).tolist(),
                    right_implied_object_twist=(maps["B_right"] @ x[7:]).tolist(),
                    tracking_translation_error_m_s=t_error,
                    tracking_rotation_error_rad_s=r_error,
                    tracking_translation_tolerance_m_s=cfg.tracking_translation_tolerance_m_s,
                    tracking_rotation_tolerance_rad_s=cfg.tracking_rotation_tolerance_rad_s,
                    soft_tracking_objective=.5*float(error @ W @ error)
                                            + .5*cfg.regularization*float(x @ x),
                    q_initial=q.tolist(), q_lookahead=(q+dt*x).tolist(), dt_s=dt,
                    solve_wall_time_s=time.perf_counter()-started,
                    execution_authorized=False,
                )
                return VelocityResult("FEASIBLE_TRACKING_OK" if tracking_ok else "FEASIBLE_TRACKING_REJECTED",
                    tuple(float(v) for v in x),
                    ("EQUALITY_ONLY_OPTIMUM_VERIFIED_NO_BOX_CONSTRAINTS",) if tracking_ok
                    else ("SOFT_TRACKING_TOLERANCE_EXCEEDED",), diagnostics)
        except Exception as exc:
            return VelocityResult("NUMERICAL_FAILURE", None, ("UNBOUNDED_NUMERICAL_EXCEPTION",),
                dict(exception_type=type(exc).__name__, exception_message=str(exc),
                     solve_wall_time_s=time.perf_counter()-started, execution_authorized=False))


class MultiStepDriver:
    """Retain all attempted points; never continue from a rejected candidate.

    Optional injection exists only for deterministic software controls. Any
    supplied dependency is conservatively labelled as injected scope; it cannot
    serve as the default frozen-FR3 evidence merely by returning valid receipts.
    """
    def __init__(self, controller=None, validator=None):
        self.injected = controller is not None or validator is not None
        self.controller = controller if controller is not None else CentralizedVelocityQP()
        self.validator = validator if validator is not None else CandidateValidator()
        self.reference = EqualityUnboundedReference(self.controller)

    def _state_diagnostic(self, q, X):
        closure = _closure(self.controller.model, q, X, self.controller.grasps)
        return dict(
            closure=asdict(closure),
            achieved_object_pose_left=closure.implied_left_object,
            achieved_object_pose_right=closure.implied_right_object,
            accumulated_drift_semantics="FULL_SE3_RESIDUAL_AGAINST_CURRENT_FIXED_TWIST_REFERENCE_NOT_SUM_OF_ERRORS",
            minimum_current_position_margin_rad=float(np.min(np.minimum(
                q-self.controller.lower, self.controller.upper-q))),
        )

    def _frozen_bounds(self, q, qdot, dt):
        candidate = q + dt*qdot
        position_lower = (self.controller.interior_lower-q)/dt
        position_upper = (self.controller.interior_upper-q)/dt
        lower = np.maximum(-self.controller.velocity, position_lower)
        upper = np.minimum(self.controller.velocity, position_upper)
        velocity_margin = self.controller.velocity - np.abs(qdot)
        position_margin = np.minimum(candidate-self.controller.lower, self.controller.upper-candidate)
        reasons = []
        if np.any(velocity_margin < -1e-10):
            reasons.append("FROZEN_VELOCITY_LIMIT_VIOLATION")
        if not (np.all(candidate > self.controller.lower) and np.all(candidate < self.controller.upper)):
            reasons.append("FROZEN_LOOKAHEAD_JOINT_LIMIT_VIOLATION")
        # The same original QP lookahead box, independently reconstructed.
        if np.any(qdot < lower-1e-8) or np.any(qdot > upper+1e-8):
            reasons.append("FROZEN_LOOKAHEAD_BOX_VIOLATION")
        # Match QP-0's report-only active-set classification. This does not
        # enlarge the hard box or any safety acceptance tolerance.
        active_tolerance = 1e-7
        return dict(valid=not reasons, reasons=reasons, lower_rad_s=lower, upper_rad_s=upper,
            active_lower_indices=np.flatnonzero(np.abs(qdot-lower) <= active_tolerance),
            active_upper_indices=np.flatnonzero(np.abs(qdot-upper) <= active_tolerance),
            active_bound_reporting_tolerance_rad_s=active_tolerance,
            minimum_position_margin_rad=float(np.min(position_margin)),
            minimum_velocity_margin_rad_s=float(np.min(velocity_margin)),
            velocity_violating_indices=np.flatnonzero(velocity_margin < -1e-10),
            position_violating_indices=np.flatnonzero(position_margin <= 0),
            lower_bound_sources=[
                "FROZEN_VELOCITY" if -self.controller.velocity[j] > position_lower[j]
                else "POSITION_LOOKAHEAD" if position_lower[j] > -self.controller.velocity[j]
                else "FROZEN_VELOCITY_AND_POSITION_LOOKAHEAD" for j in range(14)],
            upper_bound_sources=[
                "FROZEN_VELOCITY" if self.controller.velocity[j] < position_upper[j]
                else "POSITION_LOOKAHEAD" if position_upper[j] < self.controller.velocity[j]
                else "FROZEN_VELOCITY_AND_POSITION_LOOKAHEAD" for j in range(14)],
            velocity_max_rad_s=self.controller.velocity,
            position_lower_rad=self.controller.lower, position_upper_rad=self.controller.upper,
            evaluation_scope="INDEPENDENT_FROZEN_BOUND_DIAGNOSTIC_NO_CLIPPING_OR_PROJECTION")

    def run(self, request: MultiStepRequest, *, method="bounded", site=None):
        started = time.perf_counter()
        record = dict(
            status="INPUT_REJECTED", method=method, request=_jsonable(request),
            steps=[], accepted_step_count=0, attempted_step_count=0, retained_state=None,
            failure=None, execution_authorized=False,
            continuous_safety="NOT_CERTIFIED_FINITE_SAMPLING_ONLY", continuous_verifier_run=False,
            temporal_confirmation="NOT_RUN_NO_PHASE_CHANGE_OR_RELEASE_AUTHORIZATION",
            phase_confirmation_observations_generated=0,
            collision_gradient_constraint_implemented=False, nr_projection_performed=False,
            observation_source="SYNTHETIC_OFFLINE_COMPUTATION_NOT_MEASURED",
            time_semantics="NUMERICAL_TIMESTEP_NOT_MEASURED_SERVO_OR_ROS_TIMING",
            injected_backend_scope=("ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE"
                                    if self.injected else "DEFAULT_UNCHANGED_FROZEN_FR3_QP0_G1_B0"),
            reference_scope=("MATHEMATICAL_UNBOUNDED_COMPARISON_NOT_SAFE_OR_EXECUTABLE"
                             if method == "unbounded_reference" else "AUTHENTICATED_FROZEN_POSITION_VELOCITY_BOX_QP"),
        )
        try:
            if type(request) is not MultiStepRequest:
                raise ValueError("INVALID_MULTI_STEP_REQUEST_TYPE")
            if method not in ("bounded", "unbounded_reference"):
                raise ValueError("UNKNOWN_METHOD")
            if site is not None:
                raise ValueError("SUPPORT_SITE_SWITCHING_NOT_SUPPORTED_GENERIC_TRANSITIONING_ONLY")
            if type(request.max_steps) is not int or request.max_steps <= 0:
                raise ValueError("INVALID_BOUNDED_MAX_STEPS")
            if type(request.identity) is not str or not request.identity.strip():
                raise ValueError("MISSING_TRAJECTORY_IDENTITY")
            if request.observation_provenance is not None and type(request.observation_provenance) is not dict:
                raise ValueError("INVALID_OBSERVATION_PROVENANCE")
            if request.cube_id != "Cube_01":
                raise ValueError("INCORRECT_CUBE_ID")
            if request.frame_id != "world":
                raise ValueError("INCORRECT_FRAME_ID")
            dt = _positive(request.dt_s, "DT")
            q = _vector(request.q14, 14, "JOINT_STATE")
            X_initial = checked_transform(request.object_pose)
            desired = _vector(request.desired_object_twist, 6, "DESIRED_OBJECT_TWIST")
            if self.controller.config.acceleration_caps_rad_s2 is not None:
                raise ValueError("ILLUSTRATIVE_ACCELERATION_NOT_ENABLED_BY_THIS_MULTISTEP_DRIVER")
        except (ValueError, TypeError, AttributeError) as exc:
            record["failure"] = dict(step_index=None, reasons=[str(exc)])
            record["wall_time_s"] = time.perf_counter()-started
            return _jsonable(record)

        X = X_initial.copy()
        record["retained_state"] = dict(q14=q.copy(), object_pose=X.copy(), time_s=0.)
        previous_qdot = None
        # Prove the initial current state under the original geometry gate
        # before any integration. These synthetic waypoints are never R1 frames.
        try:
            initial_trajectory = self.validator.trajectory(
                [(X, q), (X, q)], Motion.GENERIC, request.identity+":initial",
                time_step_s=dt, enforce_temporal_confirmation=False)
            initial = self.validator.checker.evaluate(initial_trajectory.waypoints[0])
            record["initial_state"] = initial
            record["initial_kinematics"] = self._state_diagnostic(q, X)
            if not initial["valid"]:
                record["status"] = "STOPPED_AT_FIRST_REJECTION"
                record["failure"] = dict(step_index=0, reasons=["INITIAL_STATE_GEOMETRY_REJECTED"])
                record["wall_time_s"] = time.perf_counter()-started
                return _jsonable(record)
        except Exception as exc:
            record["status"] = "STOPPED_AT_FIRST_REJECTION"
            record["failure"] = dict(step_index=0, reasons=["INITIAL_STATE_DIAGNOSTIC_FAILURE"],
                exception_type=type(exc).__name__, exception_message=str(exc))
            record["wall_time_s"] = time.perf_counter()-started
            return _jsonable(record)

        for index in range(request.max_steps):
            step_started = time.perf_counter()
            incoming = VelocityInput(q.copy(), X.copy(), desired.copy(), dt,
                                     cube_id=request.cube_id, frame_id=request.frame_id)
            step = dict(step_index=index, time_s=index*dt, identity=request.identity+f":step:{index}",
                q_initial=q.copy(), object_pose_initial=X.copy(), desired_object_twist=desired.copy(),
                accepted=False, reasons=[], execution_authorized=False,
                qdot14=None, q_next=None, desired_object_pose_next=None,
                current_state=None, predicted_state=None, sampled_edge=None, trajectory=None,
                frozen_bounds=None, velocity_change_rad_s=None,
                velocity_change_rate_rad_s2=None,
                velocity_change_semantics="NUMERICAL_DIFFERENCE_ONLY_NO_AUTHENTICATED_ACCELERATION_LIMIT",
                observation_source="SYNTHETIC_OFFLINE_COMPUTATION_NOT_MEASURED")
            try:
                solver = self.controller if method == "bounded" else self.reference
                result = solver.solve(incoming)
                step["qp"] = asdict(result)
                if result.status != "FEASIBLE_TRACKING_OK":
                    step["reasons"].append("QP_"+result.status)
                    step["reasons"].extend(result.reasons)
                if result.qdot is not None:
                    qdot = _vector(result.qdot, 14, "QP_CANDIDATE")
                    q_next = q + dt*qdot
                    X_next = integrate_object_world_center(X_initial, desired, (index+1)*dt)
                    step.update(qdot14=qdot, q_next=q_next, desired_object_pose_next=X_next,
                                frozen_bounds=self._frozen_bounds(q, qdot, dt))
                    step["reasons"].extend(step["frozen_bounds"]["reasons"])
                    # Recompute hard equality and tracking from the frozen
                    # kinematics rather than trusting a status or receipt.
                    maps = velocity_maps(self.controller.model, q, X, self.controller.grasps)
                    actual_twist = maps["B"] @ qdot
                    closure_velocity = maps["C"] @ qdot
                    tracking = actual_twist-desired
                    t_error = float(np.linalg.norm(tracking[:3]))
                    r_error = float(np.linalg.norm(tracking[3:]))
                    equality_inf = float(np.linalg.norm(closure_velocity, ord=np.inf))
                    step["independent_velocity_diagnostics"] = dict(
                        achieved_object_twist=actual_twist,
                        closure_velocity_residual=closure_velocity,
                        original_equality_inf=equality_inf,
                        tracking_translation_error_m_s=t_error,
                        tracking_rotation_error_rad_s=r_error,
                        tracking_translation_tolerance_m_s=self.controller.config.tracking_translation_tolerance_m_s,
                        tracking_rotation_tolerance_rad_s=self.controller.config.tracking_rotation_tolerance_rad_s,
                        original_equality_tolerance=1e-8,
                        evaluation_scope="RECOMPUTED_FROM_FROZEN_MAPS_NOT_SOLVER_REPORTED_STATUS")
                    if not (np.isfinite(actual_twist).all() and np.isfinite(closure_velocity).all()):
                        step["reasons"].append("INDEPENDENT_NONFINITE_VELOCITY_MAP_RESULT")
                    if equality_inf > 1e-8:
                        step["reasons"].append("INDEPENDENT_VELOCITY_CLOSURE_REJECTED")
                    if (t_error > self.controller.config.tracking_translation_tolerance_m_s
                            or r_error > self.controller.config.tracking_rotation_tolerance_rad_s):
                        step["reasons"].append("INDEPENDENT_TASK_TRACKING_REJECTED")
                    step["current_kinematics"] = self._state_diagnostic(q, X)
                    step["predicted_kinematics"] = self._state_diagnostic(q_next, X_next)
                    if previous_qdot is not None:
                        step["velocity_change_rad_s"] = qdot-previous_qdot
                        step["velocity_change_rate_rad_s2"] = (qdot-previous_qdot)/dt
                    trajectory = self.validator.trajectory(
                        [(X, q), (X_next, q_next)], Motion.GENERIC, step["identity"],
                        start_time_s=index*dt, time_step_s=dt,
                        enforce_temporal_confirmation=False)
                    current = self.validator.checker.evaluate(trajectory.waypoints[0])
                    predicted = self.validator.checker.evaluate(trajectory.waypoints[1])
                    # Original default sampling policy: no configurable or
                    # weakened gate is introduced by this experimental driver.
                    edge = self.validator.checker.validate(trajectory)
                    step.update(current_state=current, predicted_state=predicted,
                                sampled_edge=asdict(edge), trajectory=asdict(trajectory))
                    if not current["valid"]:
                        step["reasons"].append("CURRENT_STATE_GEOMETRY_REJECTED")
                    if not predicted["valid"]:
                        step["reasons"].append("PREDICTED_STATE_GEOMETRY_REJECTED")
                    if edge.status is not EdgeStatus.SAMPLED_EDGE_VALID:
                        step["reasons"].append("SAMPLED_EDGE_"+edge.status.value.upper())
                        step["reasons"].extend(edge.reasons)
                    step["accepted"] = not step["reasons"]
                else:
                    if not step["reasons"]:
                        step["reasons"].append("MISSING_NUMERICAL_CANDIDATE")
            except Exception as exc:
                step["reasons"].append("STEP_DIAGNOSTIC_OR_BACKEND_FAILURE")
                step["exception_type"] = type(exc).__name__
                step["exception_message"] = str(exc)
            step["reasons"] = list(dict.fromkeys(step["reasons"]))
            step["wall_time_s"] = time.perf_counter()-step_started
            record["steps"].append(step)
            record["attempted_step_count"] += 1
            if not step["accepted"]:
                record["status"] = "STOPPED_AT_FIRST_REJECTION"
                record["failure"] = dict(step_index=index, reasons=step["reasons"],
                    retained_failing_attempt=True,
                    retained_failing_candidate=step["q_next"] is not None)
                break
            q, X = q_next, X_next
            previous_qdot = qdot.copy()
            record["accepted_step_count"] += 1
            record["retained_state"] = dict(q14=q.copy(), object_pose=X.copy(), time_s=(index+1)*dt)
        else:
            record["status"] = "COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED"
        record["wall_time_s"] = time.perf_counter()-started
        return _jsonable(record)


def run_multistep(q14, object_pose, desired_object_twist, dt_s, max_steps, identity,
                  *, method="bounded", controller=None, validator=None, site=None,
                  cube_id="Cube_01", frame_id="world", observation_provenance=None):
    """Convenience API; all configuration and experiment budgets remain caller-owned."""
    request = MultiStepRequest(q14, object_pose, desired_object_twist, dt_s, max_steps,
                              identity, cube_id, frame_id, observation_provenance)
    return MultiStepDriver(controller, validator).run(request, method=method, site=site)
