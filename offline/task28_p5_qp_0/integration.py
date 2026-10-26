"""Offline Euler prediction and original G1 geometry diagnostics, never a driver.

The object reference is advanced with its requested world-at-center twist.
Joint prediction is q+dt*qdot. A velocity-compatible solution need not preserve
finite SE(3) closure; the unchanged state and sampled-edge validators decide.
No interpolated state is converted to temporal attachment/support evidence.
"""
from dataclasses import asdict
import numpy as np
from scipy.spatial.transform import Rotation
from . import bootstrap  # noqa: F401
from .core import CentralizedVelocityQP, VelocityInput, _positive, _vector
from task28_validator.se3 import checked_transform
from offline.task28_p4_nr_0.validation import CandidateValidator
from task28_edge.types import EdgeStatus, Motion


def integrate_object_world_center(pose, twist, dt_s):
    """Constant world linear velocity of Cube CENTER and spatial angular rate.

    This is a numerical reference increment, not exp(t*se3) about world origin.
    It must never be sent to the current position-only actuator bridge.
    """
    pose = checked_transform(pose)
    twist = _vector(twist, 6, 'TWIST')
    dt = _positive(dt_s, 'DT')
    result = pose.copy()
    result[:3, 3] += dt * twist[:3]
    result[:3, :3] = Rotation.from_rotvec(dt * twist[3:]).as_matrix() @ pose[:3, :3]
    return result


class OfflineStepDiagnostics:
    def __init__(self, controller=None, validator=None):
        self.controller = controller or CentralizedVelocityQP()
        self.validator = validator or CandidateValidator()

    def evaluate(self, request, identity, *, site=None):
        if not isinstance(identity, str) or not identity.strip():
            raise ValueError('MISSING_STEP_IDENTITY')
        result = self.controller.solve(request)
        record = dict(identity=identity, qp=asdict(result),
                      execution_authorized=False,
                      continuous_safety='NOT_CERTIFIED_FINITE_SAMPLING_ONLY',
                      continuous_verifier_run=False,
                      collision_gradient_constraint_implemented=False,
                      temporal_confirmation='NOT_RUN_NO_PHASE_CHANGE_OR_RELEASE_AUTHORIZATION',
                      phase_confirmation_observations_generated=0,
                      scope='ONE_OFFLINE_PREDICTED_STEP_NO_ACTUATOR_ADAPTER')
        if result.qdot is None:
            record.update(status=result.status, sampled_tracking_candidate=False,
                          current_state=None, predicted_state=None, sampled_edge=None)
            return record
        X = checked_transform(request.object_pose)
        q = _vector(request.q14, 14, 'JOINT_STATE')
        q_next = q + request.dt_s * np.asarray(result.qdot)
        X_next = integrate_object_world_center(X, request.desired_object_twist, request.dt_s)
        trajectory = self.validator.trajectory(
            [(X, q), (X_next, q_next)], Motion.GENERIC, identity,
            site=site, time_step_s=request.dt_s,
            enforce_temporal_confirmation=False)
        current = self.validator.checker.evaluate(trajectory.waypoints[0])
        predicted = self.validator.checker.evaluate(trajectory.waypoints[1])
        edge = self.validator.checker.validate(trajectory)
        eligible = (result.status == 'FEASIBLE_TRACKING_OK' and current['valid']
                    and predicted['valid'] and edge.status is EdgeStatus.SAMPLED_EDGE_VALID)
        if not current['valid']:
            status = 'CURRENT_STATE_GEOMETRY_REJECTED'
        elif result.status != 'FEASIBLE_TRACKING_OK':
            status = 'FEASIBLE_BUT_TASK_TRACKING_REJECTED'
        elif not eligible:
            status = 'PREDICTED_STEP_GEOMETRY_REJECTED'
        else:
            status = 'OFFLINE_SAMPLED_STEP_VALID_UNCERTIFIED'
        record.update(status=status, sampled_tracking_candidate=bool(eligible),
                      current_state=current, predicted_state=predicted,
                      sampled_edge=asdict(edge), trajectory=asdict(trajectory),
                      q_next=q_next.tolist(), desired_object_pose_next=X_next.tolist(),
                      achieved_object_pose_increment_diagnostic=integrate_object_world_center(
                          X, result.diagnostics['achieved_object_twist'], request.dt_s).tolist())
        return record
