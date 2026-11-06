"""Causal, uncertainty-aware E0/E1 contact recognition for Stage B."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping

try:
    from ..observer.contract import StageBObserverFrame
except ImportError:  # tests also import ``estimator`` as a top-level package
    from observer.contract import StageBObserverFrame


class ContactEdge(str, Enum):
    LEFT_TOOL_CUBE = "LEFT_TOOL_CUBE"
    RIGHT_TOOL_CUBE = "RIGHT_TOOL_CUBE"
    CUBE_TABLE = "CUBE_TABLE"
    CUBE_DEEP_WALL = "CUBE_DEEP_WALL"
    CUBE_SIDE_WALL = "CUBE_SIDE_WALL"


class ContactState(str, Enum):
    FREE_OR_RECONTACT = "FREE_OR_RECONTACT"
    SINGLE_ARM_PUSHING = "SINGLE_ARM_PUSHING"
    PUSH_PROGRESS = "PUSH_PROGRESS"
    DEEP_WALL_NEAR = "DEEP_WALL_NEAR"
    DEEP_WALL_CONTACT_POSSIBLE = "DEEP_WALL_CONTACT_POSSIBLE"
    SIDE_CONTACT_POSSIBLE = "SIDE_CONTACT_POSSIBLE"
    STALLED_OR_AMBIGUOUS = "STALLED_OR_AMBIGUOUS"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class ContactEstimate:
    physics_step: int
    time_s: float
    state: ContactState
    edge_probabilities: Mapping[str, float]
    contact_possible: bool
    contact_confirmed_from_available_evidence: bool
    confidence: float
    uncertainty: float
    causes: tuple[str, ...] = ()
    sensor_age_s: float | None = None


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


def _proximity(gap: float | None, threshold: float, scale: float) -> float:
    if gap is None or not math.isfinite(float(gap)):
        return 0.5
    return _clamp(1.0 / (1.0 + math.exp(max(-50.0, min(50.0, (float(gap) - threshold) / scale)))))


def _features(frame: StageBObserverFrame | Mapping[str, Any]) -> dict[str, Any]:
    if isinstance(frame, Mapping):
        return dict(frame)
    visual = frame.visual_cube
    camera_stamp = frame.source_stamps.get("camera")
    age = None if camera_stamp is None else camera_stamp.age_s
    return {
        "physics_step": frame.physics_step,
        "time_s": frame.sim_time_s,
        "cube_position_m": visual.position_m,
        "visual_available": visual.position_m is not None and visual.confidence > 0.0,
        "visual_confidence": visual.confidence,
        "visual_age_s": age,
        "tcp_world_m": frame.tcp_world_m,
        "commanded_advancement_m": frame.commanded_advancement_m,
        "observed_cube_velocity_x_m_s": frame.controller_reference.get("observed_cube_velocity_x_m_s"),
        "suction_state": frame.suction_state,
        "sensor_health": frame.sensor_health,
    }


class E0ContactEstimator:
    """Geometry and motion baseline; no force/contact oracle is consumed."""

    def __init__(
        self,
        *,
        tool_threshold_m: float = 0.085,
        deep_wall_near_m: float = 0.020,
        deep_wall_face_x_m: float = 1.100,
        side_wall_inner_y_m: float = 0.243,
        table_center_z_m: float = 0.260,
        max_visual_age_s: float = 0.50,
    ) -> None:
        self.tool_threshold_m = tool_threshold_m
        self.deep_wall_near_m = deep_wall_near_m
        self.deep_wall_face_x_m = deep_wall_face_x_m
        self.side_wall_inner_y_m = side_wall_inner_y_m
        self.table_center_z_m = table_center_z_m
        self.max_visual_age_s = max_visual_age_s

    def estimate(self, frame: StageBObserverFrame | Mapping[str, Any]) -> ContactEstimate:
        f = _features(frame)
        step = int(f.get("physics_step", 0))
        time_s = float(f.get("time_s", 0.0))
        age = f.get("visual_age_s")
        available = bool(f.get("visual_available", f.get("cube_position_m") is not None))
        confidence = float(f.get("visual_confidence", 0.0) or 0.0)
        if not available or confidence < 0.30 or (age is not None and float(age) > self.max_visual_age_s):
            uncertainty = _clamp(0.85 + (0.10 if age is not None and float(age) > self.max_visual_age_s else 0.0))
            return ContactEstimate(
                step, time_s, ContactState.UNKNOWN,
                {edge.value: 0.5 for edge in ContactEdge}, False, False,
                1.0 - uncertainty, uncertainty, ("visual_missing_or_stale",), age,
            )

        position = f.get("cube_position_m")
        if position is None or len(position) < 3:
            return ContactEstimate(step, time_s, ContactState.UNKNOWN, {edge.value: 0.5 for edge in ContactEdge}, False, False, 0.0, 1.0, ("cube_pose_missing",), age)
        x, y, z = (float(v) for v in position[:3])
        tcp = f.get("tcp_world_m") or {}
        sensor_health = f.get("sensor_health") or {}
        tcp_health_value = sensor_health.get("tcp")
        tcp_health = getattr(tcp_health_value, "value", tcp_health_value)
        valid_tcp = {
            side: tcp.get(side) is not None and len(tcp.get(side)) >= 3
            for side in ("left", "right")
        }
        if tcp_health in ("UNKNOWN", "STALE", "FAILED") or not all(valid_tcp.values()):
            return ContactEstimate(
                step,
                time_s,
                ContactState.UNKNOWN,
                {edge.value: 0.5 for edge in ContactEdge},
                False,
                False,
                0.0,
                1.0,
                ("tcp_missing_or_stale",),
                age,
            )
        distances = {}
        for side in ("left", "right"):
            point = tcp.get(side)
            distances[side] = None if point is None else math.dist(tuple(float(v) for v in point[:3]), (x, y, z))
        edges = {
            ContactEdge.LEFT_TOOL_CUBE.value: _proximity(distances["left"], self.tool_threshold_m, 0.010),
            ContactEdge.RIGHT_TOOL_CUBE.value: _proximity(distances["right"], self.tool_threshold_m, 0.010),
            ContactEdge.CUBE_TABLE.value: _proximity(abs(z - self.table_center_z_m), 0.004, 0.003),
            ContactEdge.CUBE_DEEP_WALL.value: _proximity(self.deep_wall_face_x_m - x, self.deep_wall_near_m, 0.010),
            ContactEdge.CUBE_SIDE_WALL.value: _proximity(self.side_wall_inner_y_m - abs(y), self.deep_wall_near_m, 0.010),
        }
        tool_near = max(edges[ContactEdge.LEFT_TOOL_CUBE.value], edges[ContactEdge.RIGHT_TOOL_CUBE.value]) > 0.55
        command = f.get("commanded_advancement_m")
        velocity = f.get("observed_cube_velocity_x_m_s")
        pushing = command is not None and float(command) > 0.0 and tool_near
        near_wall = (self.deep_wall_face_x_m - x) <= self.deep_wall_near_m
        side_possible = (self.side_wall_inner_y_m - abs(y)) <= self.deep_wall_near_m
        causes: list[str] = []
        if near_wall:
            if pushing and velocity is not None and float(velocity) < 0.005:
                state = ContactState.DEEP_WALL_CONTACT_POSSIBLE
                causes.append("commanded_push_without_visual_progress")
            else:
                state = ContactState.DEEP_WALL_NEAR
                causes.append("geometric_wall_proximity_only")
        elif side_possible and tool_near:
            state = ContactState.SIDE_CONTACT_POSSIBLE
            causes.append("side_gap_and_tool_proximity")
        elif pushing and velocity is not None and float(velocity) > 0.005:
            state = ContactState.PUSH_PROGRESS
            causes.append("measured_visual_progress_during_command")
        elif pushing and velocity is not None and float(velocity) <= 0.005:
            state = ContactState.STALLED_OR_AMBIGUOUS
            causes.append("commanded_motion_without_measured_progress")
        elif tool_near:
            state = ContactState.SINGLE_ARM_PUSHING
            causes.append("tool_cube_proximity")
        else:
            state = ContactState.FREE_OR_RECONTACT
            causes.append("no_tool_contact_geometry")
        # Proximity is evidence for a possible edge. Without force or an
        # equivalent measured contact signal, it is never confirmed contact.
        uncertainty = _clamp(0.10 + (1.0 - confidence) * 0.60 + (0.10 if age is not None and age > 0.20 else 0.0))
        return ContactEstimate(
            step, time_s, state, edges, tool_near or near_wall or side_possible,
            False, _clamp(1.0 - uncertainty), uncertainty, tuple(causes), age,
        )


class E1TemporalContactEstimator:
    """Causal temporal wrapper with bounded prediction and state hysteresis."""

    def __init__(self, base: E0ContactEstimator | None = None, *, dwell_frames: int = 3, max_prediction_s: float = 0.25) -> None:
        self.base = base or E0ContactEstimator()
        self.dwell_frames = max(1, int(dwell_frames))
        self.max_prediction_s = float(max_prediction_s)
        self._last_position: tuple[float, float, float] | None = None
        self._last_time: float | None = None
        self._velocity_x = 0.0
        self._state = ContactState.UNKNOWN
        self._pending: ContactState | None = None
        self._pending_count = 0

    def estimate(self, frame: StageBObserverFrame | Mapping[str, Any]) -> ContactEstimate:
        f = _features(frame)
        position = f.get("cube_position_m")
        time_s = float(f.get("time_s", 0.0))
        available = bool(f.get("visual_available", position is not None))
        causes: list[str] = []
        if available and position is not None:
            pos = tuple(float(v) for v in position[:3])
            if self._last_position is not None and self._last_time is not None and time_s > self._last_time:
                self._velocity_x = (pos[0] - self._last_position[0]) / (time_s - self._last_time)
            self._last_position, self._last_time = pos, time_s
            f["observed_cube_velocity_x_m_s"] = f.get("observed_cube_velocity_x_m_s", self._velocity_x)
        else:
            age = None if self._last_time is None else max(0.0, time_s - self._last_time)
            if self._last_position is not None and age is not None and age <= self.max_prediction_s:
                predicted = (self._last_position[0] + self._velocity_x * age, self._last_position[1], self._last_position[2])
                f["cube_position_m"] = predicted
                f["visual_available"] = True
                f["visual_confidence"] = max(0.35, min(float(f.get("visual_confidence", 0.0) or 0.0), 0.55))
                f["visual_age_s"] = age
                causes.append("short_horizon_causal_prediction")
            else:
                self._state = ContactState.UNKNOWN
                self._pending = None
                self._pending_count = 0
                return ContactEstimate(int(f.get("physics_step", 0)), time_s, ContactState.UNKNOWN, {edge.value: 0.5 for edge in ContactEdge}, False, False, 0.0, 1.0, ("visual_dropout_exceeded_prediction_horizon",), age)
        raw = self.base.estimate(f)
        if raw.state == ContactState.UNKNOWN:
            self._state = ContactState.UNKNOWN
            self._pending = None
            self._pending_count = 0
            return raw
        if raw.state == self._state:
            self._pending = None
            self._pending_count = 0
            return raw if not causes else ContactEstimate(**{**raw.__dict__, "causes": raw.causes + tuple(causes)})
        if raw.state != self._pending:
            self._pending = raw.state
            self._pending_count = 1
        else:
            self._pending_count += 1
        if self._pending_count >= self.dwell_frames:
            self._state = raw.state
            self._pending = None
            self._pending_count = 0
            transition_causes = raw.causes + ("hysteresis_transition",) + tuple(causes)
            return ContactEstimate(**{**raw.__dict__, "causes": transition_causes})
        return ContactEstimate(**{**raw.__dict__, "state": self._state, "uncertainty": _clamp(raw.uncertainty + 0.08), "confidence": _clamp(raw.confidence - 0.08), "causes": raw.causes + ("candidate_pending",) + tuple(causes)})
