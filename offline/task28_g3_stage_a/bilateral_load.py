"""Minimal bilateral load observation/decomposition for Stage A.

The P2-4 observer produces a TCP-equivalent *candidate* wrench from
``M*qdd+c+g-tau``.  This module keeps that provenance, converts the wrench
from robot-on-TCP to Cube-on-contact convention, and applies the existing
P2-1 grasp algebra using measured runtime geometry.  It deliberately exposes
no physical contact-force claim and has no command sink.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np

from offline.task28_p2_1.grasp import GraspModel, Decomposition

SIDES = ("left", "right")
_CANDIDATE_SOURCE_STATUS = {"SYNTHETIC_ONLY", "SOURCE_UNVALIDATED_CANDIDATE"}


def _vec(value, shape, code):
    try:
        raw = np.asarray(value)
    except (TypeError, ValueError):
        raise ValueError(code) from None
    if raw.shape != shape or not np.issubdtype(raw.dtype, np.number):
        raise ValueError(code)
    if np.issubdtype(raw.dtype, np.complexfloating) or np.issubdtype(raw.dtype, np.bool_):
        raise ValueError(code)
    out = raw.astype(float)
    if not np.isfinite(out).all():
        raise ValueError(code)
    return out


def _transform(value, code):
    t = _vec(value, (4, 4), code)
    if not np.allclose(t[3], [0., 0., 0., 1.], atol=1e-9, rtol=0.):
        raise ValueError(code)
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0.) or np.linalg.det(r) <= 0:
        raise ValueError(code)
    return t


@dataclass(frozen=True)
class RuntimeGeometry:
    """Measured simulation geometry at one physics step.

    ``tcp_world`` are the actual side-suction TCP poses.  The provenance is
    required to prevent the historical nominal +/-61 mm fixture from entering
    this runtime path unnoticed.
    """

    cube_world: object
    tcp_world: dict[str, object]
    provenance: str
    evidence_id: str

    def __post_init__(self):
        _transform(self.cube_world, "INVALID_CUBE_TRANSFORM")
        if not isinstance(self.tcp_world, dict) or set(self.tcp_world) != set(SIDES):
            raise ValueError("INVALID_TCP_TRANSFORMS")
        for side in SIDES:
            _transform(self.tcp_world[side], "INVALID_TCP_TRANSFORM")
        if self.provenance != "RUNTIME_MEASURED_SIMULATION":
            raise ValueError("GEOMETRY_PROVENANCE_NOT_RUNTIME")
        if not isinstance(self.evidence_id, str) or not self.evidence_id.strip():
            raise ValueError("MISSING_GEOMETRY_EVIDENCE")

    @property
    def model(self) -> GraspModel:
        cube = _transform(self.cube_world, "INVALID_CUBE_TRANSFORM")
        contacts = {
            side: np.linalg.inv(cube) @ _transform(self.tcp_world[side], "INVALID_TCP_TRANSFORM")
            for side in SIDES
        }
        model = GraspModel(cube, contacts["left"], contacts["right"])
        # GraspModel's historical default is intentionally abstract.  Override
        # only the provenance label on this immutable runtime-built instance.
        model.geometry_provenance = self.provenance
        return model


@dataclass(frozen=True)
class ArmObserverOutput:
    """One P2-4 candidate result plus its explicit frame/time contract."""

    side: str
    observer_result: object
    frame: str
    tcp_point: str
    evaluation_time_s: float

    def __post_init__(self):
        if self.side not in SIDES:
            raise ValueError("INVALID_ARM_SIDE")
        if self.frame != "world" or not isinstance(self.tcp_point, str) or not self.tcp_point.strip():
            raise ValueError("UNSUPPORTED_WRENCH_FRAME")
        try:
            t = float(self.evaluation_time_s)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("INVALID_EVALUATION_TIME") from None
        if not math.isfinite(t) or t < 0:
            raise ValueError("INVALID_EVALUATION_TIME")
        result = self.observer_result
        if result is None or getattr(result, "status", None) != "CANDIDATE_ESTIMATE":
            raise ValueError("OBSERVER_RESULT_NOT_CANDIDATE")
        if getattr(result, "source_status", None) not in _CANDIDATE_SOURCE_STATUS:
            raise ValueError("OBSERVER_SOURCE_NOT_CANDIDATE")
        wrench = getattr(result, "wrench", None)
        values = None if wrench is None else getattr(wrench, "wrench_minimum_norm", None)
        _vec(values, (6,), "INVALID_OBSERVER_WRENCH")
        history = getattr(result, "history", None)
        if history is None or not isinstance(getattr(history, "evidence_id", None), str) or not history.evidence_id.strip():
            raise ValueError("MISSING_OBSERVER_EVIDENCE")

    @property
    def wrench_on_robot_world(self) -> np.ndarray:
        return _vec(self.observer_result.wrench.wrench_minimum_norm, (6,), "INVALID_OBSERVER_WRENCH")

    @property
    def evidence_id(self) -> str:
        return self.observer_result.history.evidence_id


@dataclass(frozen=True)
class BilateralLoadResult:
    status: str
    geometry_provenance: str
    geometry_evidence_id: str
    source_status: dict[str, str]
    observer_evidence_id: dict[str, str]
    contact_wrench_on_cube_local: tuple[float, ...]
    net_wrench_on_cube_world: tuple[float, ...]
    external_contact_wrench_local: tuple[float, ...]
    internal_contact_wrench_local: tuple[float, ...]
    internal_axis_object: tuple[float, ...]
    internal_axis_basis_local: tuple[float, ...]
    decomposition: Decomposition
    residuals: dict[str, float]
    torque_feasibility: dict[str, object] | None
    diagnostics: tuple[str, ...]

    @property
    def physical_contact_wrench_validated(self) -> bool:
        return False

    def as_record(self) -> dict[str, object]:
        """Return JSON-safe evidence without promoting the candidate result."""
        d = self.decomposition
        return {
            "status": self.status,
            "geometry_provenance": self.geometry_provenance,
            "geometry_evidence_id": self.geometry_evidence_id,
            "source_status": dict(self.source_status),
            "observer_evidence_id": dict(self.observer_evidence_id),
            "contact_wrench_on_cube_local": list(self.contact_wrench_on_cube_local),
            "net_wrench_on_cube_world": list(self.net_wrench_on_cube_world),
            "external_contact_wrench_local": list(self.external_contact_wrench_local),
            "internal_contact_wrench_local": list(self.internal_contact_wrench_local),
            "internal_axis_object": list(self.internal_axis_object),
            "internal_axis_basis_local": list(self.internal_axis_basis_local),
            "rank": int(d.rank),
            "singular_values": np.asarray(d.singular_values, dtype=float).tolist(),
            "residuals": dict(self.residuals),
            "torque_feasibility": self.torque_feasibility,
            "physical_contact_wrench_validated": False,
            "diagnostics": list(self.diagnostics),
        }


class BilateralLoadObserver:
    """Compose two candidate arm outputs without creating a sensor claim."""

    def __init__(self, *, max_time_skew_s: float = 0.002):
        if not math.isfinite(float(max_time_skew_s)) or float(max_time_skew_s) < 0:
            raise ValueError("INVALID_TIME_SKEW_LIMIT")
        self.max_time_skew_s = float(max_time_skew_s)

    def observe(self, left: ArmObserverOutput, right: ArmObserverOutput,
                geometry: RuntimeGeometry, *,
                jacobians: dict[str, object] | None = None,
                torque_limits_Nm: dict[str, object] | None = None) -> BilateralLoadResult:
        if not isinstance(left, ArmObserverOutput) or not isinstance(right, ArmObserverOutput):
            raise ValueError("INVALID_ARM_OBSERVER_OUTPUT")
        if left.side != "left" or right.side != "right":
            raise ValueError("ARM_SIDE_ORDER_MISMATCH")
        if not isinstance(geometry, RuntimeGeometry):
            raise ValueError("INVALID_RUNTIME_GEOMETRY")
        skew = abs(left.evaluation_time_s - right.evaluation_time_s)
        if skew > self.max_time_skew_s:
            raise ValueError("BILATERAL_TIME_SKEW")
        if left.tcp_point != right.tcp_point:
            raise ValueError("TCP_POINT_MISMATCH")
        model = geometry.model
        # P2-4 is a wrench on the robot at the TCP.  The equal/opposite wrench
        # acts on Cube; re-express it in each measured contact frame before G.
        left_cube_world = -left.wrench_on_robot_world
        right_cube_world = -right.wrench_on_robot_world
        contacts = np.r_[model.world_to_contact("left", left_cube_world),
                         model.world_to_contact("right", right_cube_world)]
        decomposition = model.decompose(contacts)
        net = model.net(contacts, "world")
        axis_object = self._grasp_axis_object(geometry)
        axis_basis = self._internal_axis_basis(model, axis_object)
        feasibility = None
        if jacobians is not None:
            feasibility = self._torque_feasibility(
                decomposition.internal, jacobians, torque_limits_Nm)
        diagnostics = (
            "simulation_candidate_only",
            "observer_wrench_is_TCP_equivalent_model_residual",
            "contact_wrench_uses_equal_opposite_robot_wrench_sign",
            "internal_component_is_metric_projection_nullspace",
            "no_independent_PhysX_contact_oracle_used",
        )
        return BilateralLoadResult(
            status="CANDIDATE_BILATERAL_DECOMPOSITION",
            geometry_provenance=geometry.provenance,
            geometry_evidence_id=geometry.evidence_id,
            source_status={"left": left.observer_result.source_status,
                           "right": right.observer_result.source_status},
            observer_evidence_id={"left": left.evidence_id, "right": right.evidence_id},
            contact_wrench_on_cube_local=tuple(map(float, contacts)),
            net_wrench_on_cube_world=tuple(map(float, net)),
            external_contact_wrench_local=tuple(map(float, decomposition.external)),
            internal_contact_wrench_local=tuple(map(float, decomposition.internal)),
            internal_axis_object=tuple(map(float, axis_object)),
            internal_axis_basis_local=tuple(map(float, axis_basis)),
            decomposition=decomposition,
            residuals={"null_space_N_Nm": decomposition.null_residual,
                       "reconstruction_N_Nm": decomposition.reconstruction_residual,
                       "time_skew_s": skew},
            torque_feasibility=feasibility,
            diagnostics=diagnostics,
        )

    @staticmethod
    def _grasp_axis_object(geometry: RuntimeGeometry) -> np.ndarray:
        cube = _transform(geometry.cube_world, "INVALID_CUBE_TRANSFORM")
        inv = np.linalg.inv(cube)
        points = [inv @ np.r_[np.asarray(_transform(geometry.tcp_world[s], "INVALID_TCP_TRANSFORM")[:3, 3]), 1.]
                  for s in SIDES]
        axis = points[1][:3] - points[0][:3]
        norm = float(np.linalg.norm(axis))
        if norm <= 1e-9:
            raise ValueError("DEGENERATE_GRASP_AXIS")
        return axis / norm

    @staticmethod
    def _internal_axis_basis(model: GraspModel, axis_object: np.ndarray) -> np.ndarray:
        # Opposing forces along the measured bilateral axis are the supported
        # one-dimensional internal target.  Project into the exact G nullspace
        # so small measured asymmetries cannot leak into net object wrench.
        left_r = model.object_left[:3, :3]
        right_r = model.object_right[:3, :3]
        trial = np.r_[left_r.T @ axis_object, np.zeros(3),
                      -right_r.T @ axis_object, np.zeros(3)]
        projected = model.decompose(trial).internal
        norm = float(np.linalg.norm(projected))
        if norm <= 1e-12:
            raise ValueError("INTERNAL_AXIS_UNOBSERVABLE")
        return projected / norm

    @staticmethod
    def _torque_feasibility(internal_local: np.ndarray, jacobians: dict[str, object], limits: dict[str, object] | None):
        if set(jacobians) != set(SIDES):
            raise ValueError("JACOBIAN_SIDE_ORDER_MISMATCH")
        jl = _vec(jacobians["left"], (6, 7), "INVALID_LEFT_JACOBIAN")
        jr = _vec(jacobians["right"], (6, 7), "INVALID_RIGHT_JACOBIAN")
        tau = np.r_[jl.T @ internal_local[:6], jr.T @ internal_local[6:]]
        out = {"internal_joint_torque_Nm": tuple(map(float, tau)),
               "within_limits": None, "limit_source": None}
        if limits is not None:
            if set(limits) != set(SIDES):
                raise ValueError("TORQUE_LIMIT_SIDE_ORDER_MISMATCH")
            ll = _vec(limits["left"], (7,), "INVALID_LEFT_TORQUE_LIMITS")
            lr = _vec(limits["right"], (7,), "INVALID_RIGHT_TORQUE_LIMITS")
            bound = np.r_[ll, lr]
            if np.any(bound <= 0):
                raise ValueError("INVALID_TORQUE_LIMITS")
            out.update(within_limits=bool(np.all(np.abs(tau) <= bound + 1e-12)),
                       limit_source="caller_declared_joint_abs_limits_Nm",
                       max_normalized_abs=float(np.max(np.abs(tau) / bound)))
        return out

    @staticmethod
    def bounded_internal_target(result: BilateralLoadResult, target_force_N: float) -> np.ndarray:
        """Return a bounded algebraic internal target; no actuator write."""
        if not isinstance(result, BilateralLoadResult):
            raise ValueError("INVALID_BILATERAL_RESULT")
        force = float(target_force_N)
        if not math.isfinite(force) or abs(force) > 50.0:
            raise ValueError("INTERNAL_TARGET_OUT_OF_BOUNDS")
        return force * np.asarray(result.internal_axis_basis_local, dtype=float)
