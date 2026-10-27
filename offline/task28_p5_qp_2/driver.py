"""Matched offline Euler rollouts with unchanged QP1 and original G1/B0 gates.

No trajectory is sent to any actuator. The reference is the same prescribed
world-Cube-centre twist for both methods. A failed candidate is never reused.
"""
from dataclasses import asdict
import time
import numpy as np

from offline.task28_p5_qp_0.core import VelocityInput, _positive, _vector, velocity_maps
from offline.task28_p5_qp_0.integration import integrate_object_world_center
from offline.task28_p5_qp_0.bootstrap import checked_transform
from offline.task28_p5_qp_1.driver import MultiStepDriver, MultiStepRequest, _jsonable
from offline.task28_p4_nr_0.validation import CandidateValidator
from task28_edge.types import EdgeStatus, Motion


class FeedbackDriver:
    """Use a separate feedback controller; the original validators stay authoritative.

    Optional injections support software fault controls only, and are labeled.
    """
    def __init__(self, controller=None, validator=None):
        self.injected = controller is not None or validator is not None
        if controller is None:
            from .feedback import FeedbackVelocityQP
            controller = FeedbackVelocityQP()
        self.controller = controller
        self.validator = validator if validator is not None else CandidateValidator()
        # Reuse the original independent limit and closure diagnostic, not its
        # homogeneous-constraint solve or its feedback-incompatible run loop.
        self.diagnostics = MultiStepDriver(controller=self.controller, validator=self.validator)

    def run(self, request, *, site=None):
        start = time.perf_counter()
        out = dict(status="INPUT_REJECTED", method="feedback", request=_jsonable(request),
            steps=[], accepted_step_count=0, attempted_step_count=0, retained_state=None,
            failure=None, execution_authorized=False,
            continuous_safety="NOT_CERTIFIED_FINITE_SAMPLING_ONLY", continuous_verifier_run=False,
            temporal_confirmation="NOT_RUN_NO_PHASE_CHANGE_OR_RELEASE_AUTHORIZATION",
            phase_confirmation_observations_generated=0, collision_gradient_constraint_implemented=False,
            nr_projection_performed=False, observation_source="SYNTHETIC_OFFLINE_COMPUTATION_NOT_MEASURED",
            time_semantics="NUMERICAL_TIMESTEP_NOT_MEASURED_SERVO_OR_ROS_TIMING",
            injected_backend_scope=("ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE" if self.injected
                                    else "DEFAULT_UNCHANGED_FROZEN_FR3_G1_B0_WITH_NEW_FEEDBACK_QP"))
        try:
            if type(request) is not MultiStepRequest:
                raise ValueError("INVALID_MULTI_STEP_REQUEST_TYPE")
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
            initial_X = checked_transform(request.object_pose)
            desired = _vector(request.desired_object_twist, 6, "DESIRED_OBJECT_TWIST")
            if self.controller.config.acceleration_caps_rad_s2 is not None:
                raise ValueError("ILLUSTRATIVE_ACCELERATION_NOT_ENABLED_BY_THIS_MULTISTEP_DRIVER")
        except (ValueError, TypeError, AttributeError) as exc:
            out["failure"] = dict(step_index=None, reasons=[str(exc)])
            out["wall_time_s"] = time.perf_counter()-start
            return _jsonable(out)

        X = initial_X.copy()
        out["retained_state"] = dict(q14=q.copy(), object_pose=X.copy(), time_s=0.)
        previous = None
        try:
            initial_trajectory = self.validator.trajectory(
                [(X, q), (X, q)], Motion.GENERIC, request.identity+":initial",
                time_step_s=dt, enforce_temporal_confirmation=False)
            initial = self.validator.checker.evaluate(initial_trajectory.waypoints[0])
            out["initial_state"] = initial
            out["initial_kinematics"] = self.diagnostics._state_diagnostic(q, X)
            if not initial["valid"]:
                out.update(status="STOPPED_AT_FIRST_REJECTION", failure=dict(
                    step_index=0, reasons=["INITIAL_STATE_GEOMETRY_REJECTED"]))
                out["wall_time_s"] = time.perf_counter()-start
                return _jsonable(out)
        except Exception as exc:
            out.update(status="STOPPED_AT_FIRST_REJECTION", failure=dict(step_index=0,
                reasons=["INITIAL_STATE_DIAGNOSTIC_FAILURE"], exception_type=type(exc).__name__,
                exception_message=str(exc)))
            out["wall_time_s"] = time.perf_counter()-start
            return _jsonable(out)

        for index in range(request.max_steps):
            step_start = time.perf_counter()
            incoming = VelocityInput(q.copy(), X.copy(), desired.copy(), dt,
                                     cube_id=request.cube_id, frame_id=request.frame_id)
            step = dict(step_index=index, time_s=index*dt, target_time_s=(index+1)*dt,
                dt_s=dt, identity=request.identity+f":step:{index}", q_initial=q.copy(),
                object_pose_initial=X.copy(), desired_object_twist=desired.copy(), accepted=False,
                reasons=[], execution_authorized=False, qdot14=None, q_next=None,
                desired_object_pose_next=None, current_state=None, predicted_state=None,
                sampled_edge=None, trajectory=None, frozen_bounds=None,
                velocity_change_rad_s=None, velocity_change_rate_rad_s2=None,
                velocity_change_semantics="NUMERICAL_DIFFERENCE_ONLY_NO_AUTHENTICATED_ACCELERATION_LIMIT",
                observation_source="SYNTHETIC_OFFLINE_COMPUTATION_NOT_MEASURED")
            try:
                result = self.controller.solve(incoming)
                step["qp"] = asdict(result)
                if result.status != "FEASIBLE_TRACKING_OK":
                    step["reasons"].append("QP_"+result.status)
                    step["reasons"].extend(result.reasons)
                if result.qdot is not None:
                    qdot = _vector(result.qdot, 14, "QP_CANDIDATE")
                    q_next = q+dt*qdot
                    X_next = integrate_object_world_center(initial_X, desired, (index+1)*dt)
                    step.update(qdot14=qdot, q_next=q_next, desired_object_pose_next=X_next,
                                frozen_bounds=self.diagnostics._frozen_bounds(q, qdot, dt))
                    step["reasons"].extend(step["frozen_bounds"]["reasons"])
                    maps = velocity_maps(self.controller.model, q, X, self.controller.grasps)
                    achieved = maps["B"]@qdot
                    original_C = maps["C"]@qdot
                    tracking = achieved-desired
                    trans, rot = float(np.linalg.norm(tracking[:3])), float(np.linalg.norm(tracking[3:]))
                    # Reconstruct affine geometry and target from the current state;
                    # do not trust a controller-returned equality receipt.
                    A, b = self._affine_equality(incoming)
                    A, b = np.asarray(A), np.asarray(b)
                    if A.shape != (6, 14) or b.shape != (6,) or not np.isfinite(A).all() or not np.isfinite(b).all():
                        raise ValueError("INVALID_INDEPENDENT_AFFINE_EQUALITY")
                    affine = A@qdot-b
                    affine_inf = float(np.linalg.norm(affine, ord=np.inf))
                    step["independent_velocity_diagnostics"] = dict(
                        achieved_object_twist=achieved, original_C_residual=original_C,
                        original_C_inf=float(np.linalg.norm(original_C, ord=np.inf)),
                        original_C_semantics="COMPARATIVE_ONLY_NOT_FEEDBACK_HARD_CONSTRAINT",
                        tracking_translation_error_m_s=trans, tracking_rotation_error_rad_s=rot,
                        tracking_translation_tolerance_m_s=self.controller.config.tracking_translation_tolerance_m_s,
                        tracking_rotation_tolerance_rad_s=self.controller.config.tracking_rotation_tolerance_rad_s,
                        independent_affine_A=A, independent_affine_b=b,
                        affine_residual=affine, affine_residual_inf=affine_inf,
                        affine_tolerance=1e-8,
                        evaluation_scope="RECONSTRUCTED_FROM_FROZEN_MAPS_AND_CURRENT_STATE")
                    if not (np.isfinite(achieved).all() and np.isfinite(original_C).all() and np.isfinite(affine).all()):
                        step["reasons"].append("INDEPENDENT_NONFINITE_VELOCITY_MAP_RESULT")
                    if affine_inf > 1e-8:
                        step["reasons"].append("INDEPENDENT_AFFINE_EQUALITY_REJECTED")
                    if (trans > self.controller.config.tracking_translation_tolerance_m_s or
                            rot > self.controller.config.tracking_rotation_tolerance_rad_s):
                        step["reasons"].append("INDEPENDENT_TASK_TRACKING_REJECTED")
                    step["current_kinematics"] = self.diagnostics._state_diagnostic(q, X)
                    step["predicted_kinematics"] = self.diagnostics._state_diagnostic(q_next, X_next)
                    if previous is not None:
                        step["velocity_change_rad_s"] = qdot-previous
                        step["velocity_change_rate_rad_s2"] = (qdot-previous)/dt
                    trajectory = self.validator.trajectory(
                        [(X, q), (X_next, q_next)], Motion.GENERIC, step["identity"],
                        start_time_s=index*dt, time_step_s=dt, enforce_temporal_confirmation=False)
                    current = self.validator.checker.evaluate(trajectory.waypoints[0])
                    predicted = self.validator.checker.evaluate(trajectory.waypoints[1])
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
                elif not step["reasons"]:
                    step["reasons"].append("MISSING_NUMERICAL_CANDIDATE")
            except Exception as exc:
                step["reasons"].append("STEP_DIAGNOSTIC_OR_BACKEND_FAILURE")
                step["exception_type"] = type(exc).__name__
                step["exception_message"] = str(exc)
            step["reasons"] = list(dict.fromkeys(step["reasons"]))
            step["wall_time_s"] = time.perf_counter()-step_start
            out["steps"].append(step)
            out["attempted_step_count"] += 1
            if not step["accepted"]:
                out.update(status="STOPPED_AT_FIRST_REJECTION", failure=dict(
                    step_index=index, reasons=step["reasons"], retained_failing_attempt=True,
                    retained_failing_candidate=step["q_next"] is not None))
                break
            q, X, previous = q_next, X_next, qdot.copy()
            out["accepted_step_count"] += 1
            out["retained_state"] = dict(q14=q.copy(), object_pose=X.copy(), time_s=(index+1)*dt)
        else:
            out["status"] = "COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED"
        out["wall_time_s"] = time.perf_counter()-start
        return _jsonable(out)

    def _affine_equality(self, request):
        """Independently reconstruct the world-centre pose-error equations.

        Compute each implied object centre from TCP FK and its fixed grasp,
        then shift the TCP Jacobian with an explicit centre-offset cross
        product. No optimizer receipt or feedback-constructor helper is used.
        """
        import math
        from scipy.spatial.transform import Rotation

        q = _vector(request.q14, 14, "JOINT_STATE")
        cfg = self.controller.feedback_config
        gains = (cfg.relative_translation_gain_s, cfg.relative_rotation_gain_s)
        if any(isinstance(v, bool) or not isinstance(v, (int, float))
               or not np.isfinite(v) or v < 0 for v in gains):
            raise ValueError("INVALID_INDEPENDENT_FEEDBACK_GAIN")
        exclusion = cfg.log_pi_exclusion_rad
        if (isinstance(exclusion, bool) or not isinstance(exclusion, (int, float))
                or not np.isfinite(exclusion) or not 0 < exclusion < .1):
            raise ValueError("INVALID_INDEPENDENT_LOG_PI_EXCLUSION")
        positions, rotations, maps = [], [], []
        for side, joints, grasp in (("left", q[:7], self.controller.grasps.left),
                                    ("right", q[7:], self.controller.grasps.right)):
            tcp = checked_transform(self.controller.model.fk(side, joints))
            G = checked_transform(grasp)
            R = tcp[:3, :3] @ G[:3, :3].T
            offset = R @ G[:3, 3]  # object centre -> TCP, expressed in world
            p = tcp[:3, 3] - offset
            J = np.asarray(self.controller.model.jacobian(side, joints))
            if J.shape != (6, 7) or not np.isfinite(J).all():
                raise ValueError("INVALID_INDEPENDENT_TCP_JACOBIAN")
            x, y, z = offset
            cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            centre_map = np.vstack((J[:3] + cross @ J[3:], J[3:]))
            if not (np.isfinite(R).all() and np.isfinite(p).all()
                    and np.isfinite(centre_map).all()):
                raise ValueError("NONFINITE_INDEPENDENT_CENTER_MAP")
            positions.append(p)
            rotations.append(R)
            maps.append(centre_map)
        relative_R = rotations[0] @ rotations[1].T
        phi = Rotation.from_matrix(relative_R).as_rotvec()
        if not np.isfinite(phi).all():
            raise ValueError("NONFINITE_INDEPENDENT_ROTATION_LOG")
        if np.linalg.norm(phi) >= math.pi-exclusion:
            raise ValueError("INDEPENDENT_SO3_LOG_NEAR_PI_UNSUPPORTED")
        left, right = maps
        A = np.vstack((np.hstack((left[:3], -right[:3])),
                       np.hstack((left[3:], -relative_R @ right[3:]))))
        b = np.r_[-gains[0]*(positions[0]-positions[1]), -gains[1]*phi]
        if not np.isfinite(A).all() or not np.isfinite(b).all():
            raise ValueError("NONFINITE_INDEPENDENT_AFFINE_EQUALITY")
        return A, b


def run_comparison(q14, object_pose, desired_object_twist, dt_s, max_steps, identity,
                   *, method="baseline", controller=None, validator=None, site=None,
                   cube_id="Cube_01", frame_id="world", observation_provenance=None):
    request = MultiStepRequest(q14, object_pose, desired_object_twist, dt_s, max_steps,
                               identity, cube_id, frame_id, observation_provenance)
    if method == "baseline":
        return MultiStepDriver(controller=controller, validator=validator).run(request, site=site)
    if method == "feedback":
        return FeedbackDriver(controller=controller, validator=validator).run(request, site=site)
    return dict(status="INPUT_REJECTED", method=method, steps=[], accepted_step_count=0,
                attempted_step_count=0, failure=dict(step_index=None, reasons=["UNKNOWN_METHOD"]),
                execution_authorized=False)
