"""Centralized 14-DOF object-velocity QP with unchanged frozen G1 kinematics.

All twists are world expressed [linear velocity at the Cube center, angular
velocity]. The hard velocity equality makes the two arms imply the same object
velocity; it does not by itself certify a finite Euler step or collision safety.
The soft tracking objective is an explicit engineering adaptation of P5.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
import time
import numpy as np
from .bootstrap import RobotModel, checked_transform, evaluate_closure, nominal_grasps
from .solver import solve_box_equality_qp


def _vector(value, n, name):
    raw = np.asarray(value)
    if (raw.shape != (n,) or not np.issubdtype(raw.dtype, np.number)
            or np.issubdtype(raw.dtype, np.complexfloating)
            or np.issubdtype(raw.dtype, np.bool_) or not np.isfinite(raw).all()):
        raise ValueError("INVALID_" + name)
    return raw.astype(float)


def _positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError("INVALID_" + name)
    return float(value)


def skew(v3):
    x, y, z = _vector(v3, 3, "SKEW_VECTOR")
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def reexpress_twist(rotation_A_B, twist_B):
    """Reexpress a twist at the SAME physical origin, without origin shift."""
    R = np.asarray(rotation_A_B)
    if (R.shape != (3, 3) or not np.issubdtype(R.dtype, np.number)
            or np.issubdtype(R.dtype, np.complexfloating) or not np.isfinite(R).all()
            or not np.allclose(R.T @ R, np.eye(3), rtol=0, atol=1e-10)
            or not np.isclose(np.linalg.det(R), 1., rtol=0, atol=1e-10)):
        raise ValueError("INVALID_TWIST_ROTATION")
    v = _vector(twist_B, 6, "TWIST")
    return np.r_[R @ v[:3], R @ v[3:]]


def tcp_twist_transport(object_pose, grasp_transform):
    """H maps world Cube-center twist to world TCP-origin twist.

    v_TCP = v_object + omega cross r; r=R_WO*t_O_TCP. This is an origin
    shift in world coordinates; grasp rotation affects FK/closure, not this H.
    """
    obj, grasp = checked_transform(object_pose), checked_transform(grasp_transform)
    r = obj[:3, :3] @ grasp[:3, 3]
    H = np.eye(6)
    H[:3, 3:] = -skew(r)
    return H


def velocity_maps(model, q14, object_pose, grasps=None):
    """Return centralized B,C and per-arm transport/Jacobian matrices."""
    q = _vector(q14, 14, "JOINT_STATE")
    grasps = nominal_grasps() if grasps is None else grasps
    H_left = tcp_twist_transport(object_pose, grasps.left)
    H_right = tcp_twist_transport(object_pose, grasps.right)
    J_left, J_right = model.jacobian("left", q[:7]), model.jacobian("right", q[7:])
    B_left, B_right = np.linalg.solve(H_left, J_left), np.linalg.solve(H_right, J_right)
    return {"H_left": H_left, "H_right": H_right, "J_left": J_left, "J_right": J_right,
            "B_left": B_left, "B_right": B_right,
            "B": .5 * np.hstack([B_left, B_right]), "C": np.hstack([B_left, -B_right])}


@dataclass(frozen=True)
class PreviousCommand:
    """Accepted *numerical* command provenance, not a geometric safety certificate.

    dt_s is the elapsed interval from this command to the proposed command and
    must equal the request lookahead interval. No physical acceleration claim.
    """
    qdot14: object
    dt_s: float
    source_id: str
    accepted: bool


@dataclass(frozen=True)
class QPConfig:
    regularization: float = 1e-6
    translation_weight: float = 1.
    rotation_characteristic_length_m: float = .1
    tracking_translation_tolerance_m_s: float = 1e-4
    tracking_rotation_tolerance_rad_s: float = 1e-3
    acceleration_caps_rad_s2: object | None = None
    acceleration_label: str | None = None

    def __post_init__(self):
        for name in ("regularization", "translation_weight", "rotation_characteristic_length_m",
                     "tracking_translation_tolerance_m_s", "tracking_rotation_tolerance_rad_s"):
            _positive(getattr(self, name), name.upper())
        if self.acceleration_caps_rad_s2 is not None:
            caps = _vector(self.acceleration_caps_rad_s2, 14, "ACCELERATION_CAPS")
            if np.any(caps <= 0) or self.acceleration_label != "ILLUSTRATIVE_OFFLINE":
                raise ValueError("UNAUTHENTICATED_ACCELERATION_CONFIGURATION")
        elif self.acceleration_label is not None:
            raise ValueError("ACCELERATION_LABEL_WITHOUT_CAPS")


@dataclass(frozen=True)
class VelocityInput:
    q14: object
    object_pose: object
    desired_object_twist: object
    dt_s: float
    cube_id: str = "Cube_01"
    frame_id: str = "world"
    grasps: object | None = None
    previous: PreviousCommand | None = None


@dataclass(frozen=True)
class VelocityResult:
    status: str
    qdot: tuple | None
    reasons: tuple
    diagnostics: dict = field(default_factory=dict)
    execution_authorized: bool = False


class CentralizedVelocityQP:
    """One SPD QP over [left7,right7]; state/FCL integration lives separately."""
    def __init__(self, model=None, config=QPConfig()):
        if type(config) is not QPConfig:
            raise ValueError("INVALID_QP_CONFIG_TYPE")
        config.__post_init__()
        self.model = RobotModel() if model is None else model
        self.config = config
        limits_path = self.model.fixture_dir / "moveit/task28_single_cube_description/config/effective_joint_limits.json"
        limits = json.loads(limits_path.read_text())
        lower7 = _vector(limits["position_lower_rad"], 7, "FROZEN_LOWER")
        upper7 = _vector(limits["position_upper_rad"], 7, "FROZEN_UPPER")
        velocity7 = _vector(limits["velocity_max_rad_s"], 7, "FROZEN_VELOCITY")
        if (not np.array_equal(lower7, self.model.joint_lower)
                or not np.array_equal(upper7, self.model.joint_upper) or np.any(velocity7 <= 0)):
            raise ValueError("FROZEN_MODEL_LIMIT_MISMATCH")
        for side in ("left", "right"):
            urdf_velocity = np.array([float(self.model.root.find(f"joint[@name='{side}_fr3_joint{i+1}']/limit").attrib["velocity"]) for i in range(7)])
            if not np.array_equal(urdf_velocity, velocity7):
                raise ValueError("FROZEN_URDF_VELOCITY_MISMATCH")
        self.lower, self.upper, self.velocity = (np.tile(v, 2) for v in (lower7, upper7, velocity7))
        # Strict lookahead interior: use representable adjacent floats, never
        # enlarge position bounds. The final candidate is explicitly rechecked.
        self.interior_lower = np.nextafter(self.lower, np.inf)
        self.interior_upper = np.nextafter(self.upper, -np.inf)
        self.grasps = nominal_grasps()
        self.binding = {"fixture_id": self.model.manifest["fixture_id"],
                        "effective_joint_limits_sha256": hashlib.sha256(limits_path.read_bytes()).hexdigest(),
                        "velocity_limit_source": str(limits_path.relative_to(self.model.fixture_dir)),
                        "urdf_velocity_equality_verified_both_arms": True,
                        "g1_fixture_hash_verification": "RobotModel.verify_fixture on construction",
                        "position_bounds_policy": "STRICT_INTERIOR_NEXTAFTER_NO_EXPANSION"}

    def solve(self, request):
        """Return a typed numerical failure on backend exceptions or overflow.

        This exception boundary does not convert KeyboardInterrupt/SystemExit
        into scientific outcomes. No failed numerical candidate emits qdot.
        """
        start = time.perf_counter()
        try:
            with np.errstate(over="raise", invalid="raise", divide="raise"):
                return self._solve_checked(request)
        except Exception as exc:
            return VelocityResult("NUMERICAL_FAILURE", None, ("NUMERICAL_BACKEND_EXCEPTION",),
                                  {"exception_type": type(exc).__name__, "exception_message": str(exc),
                                   "failure_scope": "OFFLINE_NUMERICAL_COMPUTATION",
                                   "solve_wall_time_s": time.perf_counter()-start,
                                   "execution_authorized": False})

    def _solve_checked(self, request):
        """Reject invalid inputs before QP; never emit a failed numerical action.

        A feasible soft-tracking miss is FEASIBLE_TRACKING_REJECTED, not QP
        infeasibility. Even FEASIBLE_TRACKING_OK remains execution unauthorized.
        """
        if type(request) is not VelocityInput:
            return VelocityResult("INPUT_REJECTED", None, ("INVALID_VELOCITY_INPUT_TYPE",))
        cfg = self.config
        try:
            cfg.__post_init__()  # Revalidate caller-owned mutable cap arrays.
            q = _vector(request.q14, 14, "JOINT_STATE")
            obj = checked_transform(request.object_pose)
            desired = _vector(request.desired_object_twist, 6, "DESIRED_OBJECT_TWIST")
            dt = _positive(request.dt_s, "DT")
            if request.cube_id != "Cube_01":
                raise ValueError("INCORRECT_CUBE_ID")
            if request.frame_id != "world":
                raise ValueError("INCORRECT_FRAME_ID")
            if not self.model.within_limits(q[:7]) or not self.model.within_limits(q[7:]):
                raise ValueError("INITIAL_JOINT_LIMIT_VIOLATION")
            grasps = self.grasps if request.grasps is None else request.grasps
            if not (np.array_equal(checked_transform(grasps.left), self.grasps.left)
                    and np.array_equal(checked_transform(grasps.right), self.grasps.right)):
                raise ValueError("FROZEN_GRASP_MISMATCH")
            closure = evaluate_closure(obj, self.model.fk("left", q[:7]), self.model.fk("right", q[7:]), grasps)
            if not closure.valid:
                return VelocityResult("INPUT_REJECTED", None, ("INITIAL_SE3_CLOSURE_REJECTED",) + closure.reasons,
                                      {"initial_closure_translation_m": closure.translation_error_m,
                                       "initial_closure_rotation_rad": closure.rotation_error_rad})
            lower = np.maximum(-self.velocity, (self.interior_lower - q) / dt)
            upper = np.minimum(self.velocity, (self.interior_upper - q) / dt)
            acceleration = {"enabled": False, "physical_limits_authenticated": False}
            if cfg.acceleration_caps_rad_s2 is not None:
                prev = request.previous
                if type(prev) is not PreviousCommand:
                    raise ValueError("MISSING_OR_INVALID_PREVIOUS_COMMAND")
                previous = _vector(prev.qdot14, 14, "PREVIOUS_COMMAND")
                previous_dt = _positive(prev.dt_s, "PREVIOUS_DT")
                if (type(prev.accepted) is not bool or not prev.accepted
                        or not isinstance(prev.source_id, str) or not prev.source_id.strip()
                        or np.any(np.abs(previous) > self.velocity)):
                    raise ValueError("UNACCEPTED_OR_INVALID_PREVIOUS_COMMAND")
                if not math.isclose(previous_dt, dt, rel_tol=0, abs_tol=1e-12):
                    raise ValueError("PREVIOUS_INTERVAL_DT_MISMATCH")
                caps = _vector(cfg.acceleration_caps_rad_s2, 14, "ACCELERATION_CAPS")
                lower = np.maximum(lower, previous - caps * dt)
                upper = np.minimum(upper, previous + caps * dt)
                acceleration = {"enabled": True, "label": cfg.acceleration_label,
                                "caps_rad_s2": caps.tolist(), "previous_qdot": previous.tolist(),
                                "interval_s": dt, "source_id": prev.source_id,
                                "previous_command_numerically_accepted": True,
                                "physical_limits_authenticated": False,
                                "previous_geometric_validation_claimed": False}
            elif request.previous is not None:
                raise ValueError("UNUSED_PREVIOUS_COMMAND_WITH_ACCELERATION_DISABLED")
        except (ValueError, TypeError, AttributeError, KeyError) as exc:
            return VelocityResult("INPUT_REJECTED", None, (str(exc),))
        maps = velocity_maps(self.model, q, obj, grasps)
        for name, matrix in maps.items():
            if not np.isfinite(matrix).all():
                raise FloatingPointError("NONFINITE_KINEMATIC_MATRIX:" + name)
        W = np.diag([cfg.translation_weight] * 3 + [cfg.translation_weight * cfg.rotation_characteristic_length_m ** 2] * 3)
        B, C = maps["B"], maps["C"]
        H = B.T @ W @ B + cfg.regularization * np.eye(14)
        f = -B.T @ W @ desired
        solved = solve_box_equality_qp(H, f, C, lower, upper)
        diag = {"binding": self.binding, "joint_order": "left_joint1..7,right_joint1..7",
                "twist_convention": "WORLD_EXPRESSED_LINEAR_AT_CUBE_CENTER_THEN_ANGULAR",
                "desired_object_twist": desired.tolist(), "q_initial": q.tolist(), "dt_s": dt,
                "initial_closure_translation_m": closure.translation_error_m,
                "initial_closure_rotation_rad": closure.rotation_error_rad,
                "B": B.tolist(), "C": C.tolist(), "weight_matrix": W.tolist(),
                "regularization": cfg.regularization, "rotation_characteristic_length_m": cfg.rotation_characteristic_length_m,
                "objective_units": "weighted (m/s)^2 plus lambda*(rad/s)^2; numerical engineering weights",
                "constraint_lower_rad_s": lower.tolist(), "constraint_upper_rad_s": upper.tolist(),
                "frozen_velocity_max_rad_s": self.velocity.tolist(), "acceleration": acceleration,
                "solver": solved.diagnostics, "collision_checked_in_core": False,
                "finite_step_geometric_safety_certified": False,
                "execution_authorized": False}
        if solved.x is None:
            return VelocityResult(solved.status, None, (solved.diagnostics["reason"],), diag)
        qdot = np.asarray(solved.x)
        lookahead = q + dt * qdot
        implied_left, implied_right = maps["B_left"] @ qdot[:7], maps["B_right"] @ qdot[7:]
        actual = B @ qdot
        err = actual - desired
        translational_error = float(np.linalg.norm(err[:3]))
        rotational_error = float(np.linalg.norm(err[3:]))
        diag.update(achieved_object_twist=actual.tolist(), left_implied_object_twist=implied_left.tolist(),
                    right_implied_object_twist=implied_right.tolist(),
                    closure_velocity_residual=(implied_left - implied_right).tolist(),
                    tracking_translation_error_m_s=translational_error,
                    tracking_rotation_error_rad_s=rotational_error,
                    tracking_translation_tolerance_m_s=cfg.tracking_translation_tolerance_m_s,
                    tracking_rotation_tolerance_rad_s=cfg.tracking_rotation_tolerance_rad_s,
                    q_lookahead=lookahead.tolist(), minimum_position_margin_rad=float(np.min(np.minimum(lookahead-self.lower, self.upper-lookahead))),
                    minimum_velocity_margin_rad_s=float(np.min(self.velocity-np.abs(qdot))),
                    soft_tracking_objective=.5*float(err @ W @ err)+.5*cfg.regularization*float(qdot @ qdot))
        if not (np.all(lookahead > self.lower) and np.all(lookahead < self.upper)
                and np.all(np.abs(qdot) <= self.velocity + 1e-10)):
            return VelocityResult("NUMERICAL_FAILURE", None, ("INDEPENDENT_LOOKAHEAD_OR_VELOCITY_CHECK_FAILED",), diag)
        tracking_ok = (translational_error <= cfg.tracking_translation_tolerance_m_s
                       and rotational_error <= cfg.tracking_rotation_tolerance_rad_s)
        return VelocityResult("FEASIBLE_TRACKING_OK" if tracking_ok else "FEASIBLE_TRACKING_REJECTED",
                              tuple(float(v) for v in qdot),
                              ("HARD_CONSTRAINTS_VERIFIED" if tracking_ok else "SOFT_TRACKING_TOLERANCE_EXCEEDED",), diag)
