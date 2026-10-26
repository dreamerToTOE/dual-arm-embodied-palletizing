"""Object-conditioned, full-coordinate SVD Newton projection for frozen FR3s.

This is the P4 full-Jacobian Moore--Penrose update, adapted to a fixed shared
object target and 14 FR3 coordinates.  The residual has twelve rows (position
and world-frame SO(3) logarithm for each frozen TCP grasp).  The analytic log
Jacobian is valid away from the principal-log cut at pi.  A global L2 step cap
and residual-decreasing, limit-preserving backtracking are explicit engineering
adaptations; joint proposals are never clipped.  This module performs no search,
collision query, command, or call to the pre-existing least-squares IK solver.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from task28_validator.closed_chain import nominal_grasps
from task28_validator.se3 import checked_transform, real_array


@dataclass(frozen=True)
class ProjectionResult:
    success: bool
    q14: tuple
    reason: str
    iterations: int
    trace: tuple
    translation_error_m: float | None
    rotation_error_rad: float | None
    minimum_joint_limit_margin_rad: float


def _skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def _inverse_left_log_jacobian(phi):
    theta = float(np.linalg.norm(phi))
    if math.pi - theta < 1e-7:
        raise ValueError("NR_SO3_PRINCIPAL_CHART_NEAR_PI")
    matrix = _skew(phi)
    # The half-angle form stays conditioned near pi; the chart itself is still
    # rejected at pi, rather than silently choosing a discontinuous derivative.
    coefficient = (1./12. + theta**2/720. + theta**4/30240.) if theta < 1e-4 else (
        1./theta**2 - (1./math.tan(theta/2.))/(2.*theta))
    return np.eye(3) - .5*matrix + coefficient*(matrix @ matrix)


class NRProjector:
    """One bounded, deterministic seed per call; q14 is left7 then right7."""

    translation_tolerance_m = 1e-6
    rotation_tolerance_rad = 1e-5

    def __init__(self, model, max_iterations=60, max_step_rad=.2,
                 backtracks=8, relative_svd_cutoff=1e-12):
        if (type(max_iterations) is not int or not 1 <= max_iterations <= 2000
                or type(backtracks) is not int or not 0 <= backtracks <= 20
                or isinstance(max_step_rad, bool) or not isinstance(max_step_rad, (int, float))
                or not math.isfinite(max_step_rad) or not 0 < max_step_rad <= math.pi
                or isinstance(relative_svd_cutoff, bool)
                or not isinstance(relative_svd_cutoff, (int, float))
                or not math.isfinite(relative_svd_cutoff) or not 0 < relative_svd_cutoff < 1):
            raise ValueError("NR_INVALID_PROJECTION_BUDGET")
        lower = real_array(model.joint_lower)
        upper = real_array(model.joint_upper)
        if (lower.shape != (7,) or upper.shape != (7,) or not np.isfinite(lower).all()
                or not np.isfinite(upper).all() or not np.all(lower < upper)):
            raise ValueError("NR_INVALID_MODEL_LIMITS")
        self.model = model
        self.lower = np.tile(lower, 2)
        self.upper = np.tile(upper, 2)
        self.max_iterations = max_iterations
        self.max_step_rad = float(max_step_rad)
        self.backtracks = backtracks
        self.relative_svd_cutoff = float(relative_svd_cutoff)
        self._grasps = nominal_grasps()

    @staticmethod
    def _q14(value):
        value = real_array(value)
        if value.shape != (14,) or not np.isfinite(value).all():
            raise ValueError("NR_EXPECTED_FINITE_Q14_LEFT7_RIGHT7")
        return value.copy()

    def _margin(self, q):
        return float(min(np.min(q-self.lower), np.min(self.upper-q)))

    def _residual_jacobian(self, object_pose, q, jacobian=True):
        values, blocks = [], []
        for index, side in enumerate(("left", "right")):
            joints = q[index*7:(index+1)*7]
            actual = checked_transform(self.model.fk(side, joints))
            target = object_pose @ getattr(self._grasps, side)
            error_rotation = actual[:3, :3] @ target[:3, :3].T
            phi = Rotation.from_matrix(error_rotation).as_rotvec()
            values.extend(actual[:3, 3]-target[:3, 3])
            values.extend(phi)
            if jacobian:
                # Residual values exist at pi, but an NR derivative there is
                # not well-defined.  Never silently select a derivative chart.
                inverse_left = _inverse_left_log_jacobian(phi)
                geometric = real_array(self.model.jacobian(side, joints))
                if geometric.shape != (6, 7) or not np.isfinite(geometric).all():
                    raise ValueError("NR_INVALID_MODEL_JACOBIAN")
                block = np.zeros((6, 14))
                block[:3, index*7:(index+1)*7] = geometric[:3]
                block[3:, index*7:(index+1)*7] = inverse_left @ geometric[3:]
                blocks.append(block)
        residual = np.asarray(values, dtype=float)
        if not np.isfinite(residual).all():
            raise ValueError("NR_NONFINITE_RESIDUAL")
        return residual, np.vstack(blocks) if jacobian else None

    def residual_jacobian(self, object_pose4x4, q14):
        """Return analytic r12/J12x14; FK algebra does not accept the state."""
        return self._residual_jacobian(checked_transform(object_pose4x4), self._q14(q14))

    @staticmethod
    def _errors(values):
        return (float(max(np.linalg.norm(values[:3]), np.linalg.norm(values[6:9]))),
                float(max(np.linalg.norm(values[3:6]), np.linalg.norm(values[9:12]))))

    def project(self, object_pose4x4, seed_q14):
        object_pose = checked_transform(object_pose4x4)
        q = self._q14(seed_q14)
        trace = []
        translation_error = rotation_error = None
        accepted_updates = 0

        def finish(success, reason):
            return ProjectionResult(success, tuple(float(value) for value in q), reason,
                                    accepted_updates, tuple(trace), translation_error,
                                    rotation_error, self._margin(q))

        try:
            initial_values, _ = self._residual_jacobian(object_pose, q, False)
            translation_error, rotation_error = self._errors(initial_values)
        except ValueError as error:
            return finish(False, str(error))
        if self._margin(q) < 0:
            return finish(False, "NR_SEED_JOINT_LIMIT_VIOLATION")
        # A loose necessary reach sphere is a valid rejection certificate, not an
        # IK solver.  Keep it optional for mathematical mock models in tests.
        if hasattr(self.model, "reach_bounds") and hasattr(self.model, "manifest"):
            for side in ("left", "right"):
                target = object_pose @ getattr(self._grasps, side)
                base = np.asarray(self.model.manifest["bases_world_m"][side])
                if np.linalg.norm(target[:3, 3]-base) > self.model.reach_bounds[side]+1e-12:
                    return finish(False, "NR_TARGET_OUTSIDE_REACH_BOUND")

        for iteration in range(self.max_iterations+1):
            try:
                values, jacobian = self._residual_jacobian(object_pose, q)
            except ValueError as error:
                return finish(False, str(error))
            translation_error, rotation_error = self._errors(values)
            if (translation_error <= self.translation_tolerance_m
                    and rotation_error <= self.rotation_tolerance_rad):
                return finish(True, "NR_CONVERGED")
            if iteration == self.max_iterations:
                return finish(False, "NR_ITERATION_LIMIT")
            try:
                u, spectrum, vh = np.linalg.svd(jacobian, full_matrices=False)
            except np.linalg.LinAlgError:
                return finish(False, "NR_SVD_FAILURE")
            cutoff = float(spectrum[0]*self.relative_svd_cutoff)
            qualified = spectrum > cutoff
            inverse = np.zeros_like(spectrum)
            inverse[qualified] = 1./spectrum[qualified]
            direction = -vh.T @ (inverse*(u.T @ values))
            if not np.isfinite(direction).all():
                return finish(False, "NR_NONFINITE_UPDATE")
            length = float(np.linalg.norm(direction))
            scale = min(1., self.max_step_rad/length) if length > 0 else 1.
            row = {"iteration": iteration, "q14_before": q.tolist(),
                   "residual12_before": values.tolist(),
                   "translation_error_m_before": translation_error,
                   "rotation_error_rad_before": rotation_error,
                   "residual_norm_before": float(np.linalg.norm(values)),
                   "singular_values": spectrum.tolist(), "svd_cutoff": cutoff,
                   "numerical_rank": int(np.count_nonzero(qualified)),
                   "uncapped_newton_direction14_rad": direction.tolist(),
                   "global_l2_step_cap_scale": scale, "trials": []}
            legal_trial = False
            accepted = False
            for backtrack in range(self.backtracks+1):
                alpha = scale*2.**(-backtrack)
                proposal = q + alpha*direction
                margin = self._margin(proposal)
                trial = {"backtrack": backtrack, "alpha": alpha,
                         "minimum_joint_limit_margin_rad": margin}
                if not np.isfinite(proposal).all():
                    trial["reason"] = "NR_NONFINITE_UPDATE"
                elif margin < 0:
                    trial["reason"] = "NR_UPDATE_JOINT_LIMIT_VIOLATION"
                else:
                    legal_trial = True
                    try:
                        candidate_values, _ = self._residual_jacobian(object_pose, proposal, False)
                        candidate_norm = float(np.linalg.norm(candidate_values))
                        trial["residual_norm_after"] = candidate_norm
                        trial["reason"] = "ACCEPTED" if candidate_norm < row["residual_norm_before"] else "NR_NO_RESIDUAL_DECREASE"
                        if trial["reason"] == "ACCEPTED":
                            q = proposal
                            translation_error, rotation_error = self._errors(candidate_values)
                            accepted_updates += 1
                            row["q14_after"] = q.tolist()
                            row["accepted_delta14_rad"] = (alpha*direction).tolist()
                            accepted = True
                    except ValueError as error:
                        trial["reason"] = str(error)
                row["trials"].append(trial)
                if accepted:
                    break
            trace.append(row)
            if not accepted:
                reason = "NR_NO_DECREASING_STEP_WITHIN_BACKTRACK_BUDGET" if legal_trial else "NR_UPDATE_JOINT_LIMIT_VIOLATION"
                return finish(False, reason)
        raise AssertionError("NR_UNREACHABLE_LOOP_EXIT")
