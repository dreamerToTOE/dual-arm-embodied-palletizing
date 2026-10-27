"""World-coordinate pose feedback for the isolated offline 14-DOF QP.

X_i = FK_i G_i^-1. All twists are [world velocity at the *implied*
object centre; world angular velocity]. With A=R_L R_R^T and
phi=Log(A), dphi=J_l(phi)^-1(w_L-A w_R). Thus the affine rotational
row is w_L-A w_R = -k_r phi, for scalar k_r. This is not C*qdot=0.
No commanded state, attachment evidence or collision safety is generated.
"""
from dataclasses import dataclass
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_p5_qp_0.bootstrap import checked_transform, evaluate_closure, inverse
from offline.task28_p5_qp_0.core import (CentralizedVelocityQP, QPConfig, VelocityInput,
    VelocityResult, _positive, _vector, tcp_twist_transport, velocity_maps)
from .affine_solver import solve_affine_box_qp


@dataclass(frozen=True)
class FeedbackConfig:
    relative_translation_gain_s: float = 2.0
    relative_rotation_gain_s: float = 2.0
    object_translation_gain_s: float = 0.5
    object_rotation_gain_s: float = 0.5
    # Fixed research domain of principal logarithm. Not a physical threshold.
    log_pi_exclusion_rad: float = 1e-5

    def __post_init__(self):
        for name in ('relative_translation_gain_s', 'relative_rotation_gain_s',
                     'object_translation_gain_s', 'object_rotation_gain_s'):
            v = getattr(self, name)
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
                raise ValueError('INVALID_FEEDBACK_GAIN:' + name)
        value = self.log_pi_exclusion_rad
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not (0 < value < .1):
            raise ValueError('INVALID_LOG_PI_EXCLUSION')


def log_rotation(matrix, pi_exclusion=1e-5):
    """World principal SO(3) logarithm; refuse the ambiguous pi neighborhood."""
    T = np.eye(4)
    T[:3, :3] = matrix
    r = checked_transform(T)[:3, :3]
    if not isinstance(pi_exclusion, (int, float)) or isinstance(pi_exclusion, bool) or not math.isfinite(pi_exclusion) or not 0 < pi_exclusion < .1:
        raise ValueError('INVALID_LOG_PI_EXCLUSION')
    phi = Rotation.from_matrix(r).as_rotvec()
    if np.linalg.norm(phi) >= math.pi - pi_exclusion:
        raise ValueError('SO3_LOG_NEAR_PI_UNSUPPORTED')
    return phi


def pose_feedback_terms(X_left, X_right, X_reference, desired_twist, config):
    """Return independently usable world-coordinate error and twist targets."""
    if type(config) is not FeedbackConfig:
        raise ValueError('INVALID_FEEDBACK_CONFIG')
    config.__post_init__()
    left, right, reference = map(checked_transform, (X_left, X_right, X_reference))
    desired = _vector(desired_twist, 6, 'DESIRED_OBJECT_TWIST')
    Rl, Rr, Rd = left[:3, :3], right[:3, :3], reference[:3, :3]
    A = Rl @ Rr.T
    phi = log_rotation(A, config.log_pi_exclusion_rad)
    delta_p = left[:3, 3] - right[:3, 3]
    targets = []
    reference_errors = []
    for X in (left, right):
        D = Rd @ X[:3, :3].T
        psi = log_rotation(D, config.log_pi_exclusion_rad)
        dp = reference[:3, 3] - X[:3, 3]
        # psi_dot = J_l(psi)^-1 (w_ref - D w_i), so this target
        # gives psi_dot = -k psi exactly for scalar gain.
        target = np.r_[desired[:3] + config.object_translation_gain_s * dp,
                       D.T @ (desired[3:] + config.object_rotation_gain_s * psi)]
        targets.append(target)
        reference_errors.append((dp, psi))
    b = np.r_[-config.relative_translation_gain_s * delta_p,
              -config.relative_rotation_gain_s * phi]
    return {'relative_rotation_matrix': A, 'relative_translation': delta_p,
            'relative_rotation_log': phi, 'b': b, 'targets': targets,
            'reference_errors': reference_errors}


def feedback_maps(model, q14, grasps):
    """Actual implied object transforms and their own world-centre Jacobians."""
    q = _vector(q14, 14, 'JOINT_STATE')
    poses, B = [], []
    for side, part, G in (('left', q[:7], grasps.left), ('right', q[7:], grasps.right)):
        X = checked_transform(model.fk(side, part) @ inverse(G))
        H = tcp_twist_transport(X, G)
        poses.append(X)
        B.append(np.linalg.solve(H, model.jacobian(side, part)))
    return poses, B


class FeedbackVelocityQP:
    """Frozen QP0 joint and tracking gates plus separately checked affine QP."""
    def __init__(self, feedback_config=FeedbackConfig(), model=None):
        if type(feedback_config) is not FeedbackConfig:
            raise ValueError('INVALID_FEEDBACK_CONFIG')
        feedback_config.__post_init__()
        base = CentralizedVelocityQP(model=model, config=QPConfig())
        for name in ('model', 'config', 'lower', 'upper', 'velocity', 'interior_lower',
                     'interior_upper', 'grasps', 'binding'):
            setattr(self, name, getattr(base, name))
        self.feedback_config = feedback_config

    def solve(self, request):
        start = time.perf_counter()
        try:
            with np.errstate(over='raise', invalid='raise', divide='raise'):
                return self._solve(request, start)
        except Exception as exc:
            return VelocityResult('NUMERICAL_FAILURE', None, ('NUMERICAL_BACKEND_EXCEPTION',),
                {'exception_type': type(exc).__name__, 'exception_message': str(exc),
                 'solve_wall_time_s': time.perf_counter()-start, 'execution_authorized': False})

    def _solve(self, request, started):
        if type(request) is not VelocityInput:
            return VelocityResult('INPUT_REJECTED', None, ('INVALID_VELOCITY_INPUT_TYPE',))
        try:
            self.feedback_config.__post_init__()
            q = _vector(request.q14, 14, 'JOINT_STATE')
            obj = checked_transform(request.object_pose)
            desired = _vector(request.desired_object_twist, 6, 'DESIRED_OBJECT_TWIST')
            dt = _positive(request.dt_s, 'DT')
            if request.cube_id != 'Cube_01': raise ValueError('INCORRECT_CUBE_ID')
            if request.frame_id != 'world': raise ValueError('INCORRECT_FRAME_ID')
            if request.previous is not None: raise ValueError('PREVIOUS_COMMAND_NOT_SUPPORTED_ACCELERATION_DISABLED')
            if not self.model.within_limits(q[:7]) or not self.model.within_limits(q[7:]):
                raise ValueError('INITIAL_JOINT_LIMIT_VIOLATION')
            grasps = self.grasps if request.grasps is None else request.grasps
            if not (np.array_equal(checked_transform(grasps.left), self.grasps.left) and
                    np.array_equal(checked_transform(grasps.right), self.grasps.right)):
                raise ValueError('FROZEN_GRASP_MISMATCH')
            closure = evaluate_closure(obj, self.model.fk('left', q[:7]), self.model.fk('right', q[7:]), grasps)
            if not closure.valid:
                return VelocityResult('INPUT_REJECTED', None, ('INITIAL_SE3_CLOSURE_REJECTED',)+closure.reasons,
                    {'initial_closure_translation_m': closure.translation_error_m,
                     'initial_closure_rotation_rad': closure.rotation_error_rad})
            poses, maps = feedback_maps(self.model, q, grasps)
            terms = pose_feedback_terms(poses[0], poses[1], obj, desired, self.feedback_config)
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            return VelocityResult('INPUT_REJECTED', None, (str(exc),))
        lower = np.maximum(-self.velocity, (self.interior_lower-q)/dt)
        upper = np.minimum(self.velocity, (self.interior_upper-q)/dt)
        L, R = maps
        mean_map = .5 * np.hstack([L, R])
        A = np.vstack([np.hstack([L[:3], -R[:3]]),
                       np.hstack([L[3:], -terms['relative_rotation_matrix'] @ R[3:]])])
        b = terms['b']
        cfg = self.config
        W = np.diag([cfg.translation_weight]*3 + [cfg.translation_weight*cfg.rotation_characteristic_length_m**2]*3)
        # Keep the original mean object-twist objective, with each actual
        # implied centre and its independently derived reference correction.
        target = .5 * (terms['targets'][0] + terms['targets'][1])
        H = mean_map.T @ W @ mean_map + cfg.regularization*np.eye(14)
        f = -mean_map.T @ W @ target
        reference_maps = velocity_maps(self.model, q, obj, grasps)
        solved = solve_affine_box_qp(H, f, A, b, lower, upper)
        diag = {'binding': self.binding, 'q_initial': q.tolist(), 'dt_s': dt,
                'desired_object_twist': desired.tolist(), 'implied_left_pose': poses[0].tolist(),
                'implied_right_pose': poses[1].tolist(), 'relative_translation_m': terms['relative_translation'].tolist(),
                'relative_rotation_log_rad': terms['relative_rotation_log'].tolist(),
                'relative_rotation_matrix': terms['relative_rotation_matrix'].tolist(),
                'reference_errors': [{'translation_m': p.tolist(), 'rotation_log_rad': r.tolist()} for p,r in terms['reference_errors']],
                'actual_center_B_left': L.tolist(), 'actual_center_B_right': R.tolist(),
                'actual_center_mean_B': mean_map.tolist(), 'corrected_mean_target': target.tolist(),
                'B': reference_maps['B'].tolist(), 'C': reference_maps['C'].tolist(),
                'affine_A': A.tolist(), 'affine_b': b.tolist(), 'corrected_twist_targets': [v.tolist() for v in terms['targets']],
                'constraint_lower_rad_s': lower.tolist(), 'constraint_upper_rad_s': upper.tolist(),
                'frozen_velocity_max_rad_s': self.velocity.tolist(), 'solver': solved.diagnostics,
                'feedback_config': self.feedback_config.__dict__, 'collision_checked_in_core': False,
                'physical_limits_authenticated': False, 'execution_authorized': False}
        if solved.x is None:
            return VelocityResult(solved.status, None, (solved.diagnostics.get('reason', solved.status),), diag)
        x = np.asarray(solved.x, dtype=float)
        lookahead = q+dt*x
        old_actual = reference_maps['B'] @ x
        actual_left, actual_right = L @ x[:7], R @ x[7:]
        actual_mean = .5*(actual_left+actual_right)
        old_err = old_actual-desired
        actual_err = actual_mean-desired
        equality = A @ x-b
        t_err, r_err = float(np.linalg.norm(old_err[:3])), float(np.linalg.norm(old_err[3:]))
        diag.update(achieved_object_twist=old_actual.tolist(), actual_implied_mean_twist=actual_mean.tolist(),
            left_implied_object_twist=actual_left.tolist(), right_implied_object_twist=actual_right.tolist(),
            actual_implied_mean_tracking_error=actual_err.tolist(),
            corrected_twist_errors=[(actual_left-terms['targets'][0]).tolist(), (actual_right-terms['targets'][1]).tolist()],
            closure_velocity_residual=(reference_maps['C'] @ x).tolist(),
            affine_equality_residual=equality.tolist(), affine_equality_residual_inf=float(np.linalg.norm(equality, ord=np.inf)),
            tracking_translation_error_m_s=t_err, tracking_rotation_error_rad_s=r_err,
            tracking_translation_tolerance_m_s=cfg.tracking_translation_tolerance_m_s,
            tracking_rotation_tolerance_rad_s=cfg.tracking_rotation_tolerance_rad_s,
            q_lookahead=lookahead.tolist(), minimum_position_margin_rad=float(np.min(np.minimum(lookahead-self.lower,self.upper-lookahead))),
            minimum_velocity_margin_rad_s=float(np.min(self.velocity-np.abs(x))),
            solve_wall_time_s=time.perf_counter()-started)
        if (x.shape != (14,) or not np.isfinite(x).all() or not np.isfinite(equality).all()
            or np.linalg.norm(equality, ord=np.inf) > 1e-8
            or np.any(x < lower-1e-8) or np.any(x > upper+1e-8)
            or not (np.all(lookahead > self.lower) and np.all(lookahead < self.upper))
            or np.any(np.abs(x) > self.velocity+1e-10)):
            return VelocityResult('NUMERICAL_FAILURE', None, ('INDEPENDENT_HARD_GATE_FAILED',), diag)
        ok = t_err <= cfg.tracking_translation_tolerance_m_s and r_err <= cfg.tracking_rotation_tolerance_rad_s
        return VelocityResult('FEASIBLE_TRACKING_OK' if ok else 'FEASIBLE_TRACKING_REJECTED',
            tuple(float(v) for v in x), ('HARD_CONSTRAINTS_VERIFIED' if ok else 'SOFT_TRACKING_TOLERANCE_EXCEEDED',), diag)
