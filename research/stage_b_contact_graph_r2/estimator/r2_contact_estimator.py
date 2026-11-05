"""Causal contact-state baselines for R2.

The estimators consume only per-frame geometry/motion observations.  PhysX
contact edges are kept in a separate reference field by the replay script and
are never read by E0/E1.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping, Optional


class ObservationState(str, Enum):
    OBSERVED_ACTIVE = "OBSERVED_ACTIVE"
    OBSERVED_INACTIVE = "OBSERVED_INACTIVE"
    UNKNOWN = "UNKNOWN"
    STALE = "STALE"


class ContactEdge(str, Enum):
    LEFT_TOOL_CUBE = "LEFT_TOOL_CUBE"
    RIGHT_TOOL_CUBE = "RIGHT_TOOL_CUBE"
    CUBE_TABLE = "CUBE_TABLE"
    CUBE_DEEP_WALL = "CUBE_DEEP_WALL"
    CUBE_SIDE_WALL = "CUBE_SIDE_WALL"


class TaskState(str, Enum):
    UNKNOWN = "UNKNOWN"
    FREE_SPACE_APPROACH = "FREE_SPACE_APPROACH"
    TOOL_CONTACT = "TOOL_CONTACT"
    NORMAL_ADVANCEMENT = "NORMAL_ADVANCEMENT"
    DEEP_WALL_ARRIVAL = "DEEP_WALL_ARRIVAL"
    SUSPECTED_PREMATURE_BLOCKAGE = "SUSPECTED_PREMATURE_BLOCKAGE"
    TRACKING_FAILURE = "TRACKING_FAILURE"
    OBSERVATION_UNAVAILABLE = "OBSERVATION_UNAVAILABLE"


@dataclass(frozen=True)
class FrameObservation:
    step: int
    time_s: float
    cube_position_m: tuple[float, float, float]
    cube_velocity_m_s: Optional[tuple[float, float, float]]
    left_tcp_m: Optional[tuple[float, float, float]]
    right_tcp_m: Optional[tuple[float, float, float]]
    command_advance_m: Optional[float] = None
    observation_state: ObservationState = ObservationState.OBSERVED_ACTIVE
    edge_observations: Mapping[ContactEdge, ObservationState] = field(default_factory=dict)
    features: Mapping[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class Estimate:
    step: int
    time_s: float
    state: TaskState
    confidence: float
    reason: str
    observation_state: ObservationState
    edges: Mapping[ContactEdge, ObservationState]


def _finite_vec(value):
    return value is not None and len(value) == 3 and all(math.isfinite(float(x)) for x in value)


def _edge_active(obs: FrameObservation, edge: ContactEdge) -> bool:
    return obs.edge_observations.get(edge) is ObservationState.OBSERVED_ACTIVE


class E0GeometricMotionRules:
    """Single-frame causal rules; no future frames or oracle fields."""

    def __init__(self, *, tool_distance_threshold_m: float = 0.085,
                 deep_wall_gap_threshold_m: float = 0.012,
                 table_gap_threshold_m: float = 0.004,
                 advancement_speed_threshold_m_s: float = 0.010,
                 tracking_error_threshold_m: float = 0.030):
        self.tool_distance_threshold_m = tool_distance_threshold_m
        self.deep_wall_gap_threshold_m = deep_wall_gap_threshold_m
        self.table_gap_threshold_m = table_gap_threshold_m
        self.advancement_speed_threshold_m_s = advancement_speed_threshold_m_s
        self.tracking_error_threshold_m = tracking_error_threshold_m

    def _inferred_edges(self, obs: FrameObservation):
        # These are geometric hypotheses, not contact truth; UNKNOWN stays UNKNOWN.
        edges = dict(obs.edge_observations)
        for edge, key in ((ContactEdge.LEFT_TOOL_CUBE, "left_tool_cube_distance_m"),
                          (ContactEdge.RIGHT_TOOL_CUBE, "right_tool_cube_distance_m")):
            value = obs.features.get(key)
            if value is not None:
                edges[edge] = (ObservationState.OBSERVED_ACTIVE
                               if value <= self.tool_distance_threshold_m
                               else ObservationState.OBSERVED_INACTIVE)
        gap = obs.features.get("cube_bottom_gap_m")
        if gap is not None:
            edges[ContactEdge.CUBE_TABLE] = (ObservationState.OBSERVED_ACTIVE
                                              if abs(gap) <= self.table_gap_threshold_m
                                              else ObservationState.OBSERVED_INACTIVE)
        gap = obs.features.get("deep_wall_gap_m")
        if gap is not None:
            edges[ContactEdge.CUBE_DEEP_WALL] = (ObservationState.OBSERVED_ACTIVE
                                                  if gap <= self.deep_wall_gap_threshold_m
                                                  else ObservationState.OBSERVED_INACTIVE)
        return edges

    def estimate(self, obs: FrameObservation) -> Estimate:
        edges = self._inferred_edges(obs)
        if obs.observation_state in (ObservationState.UNKNOWN, ObservationState.STALE):
            return Estimate(obs.step, obs.time_s, TaskState.OBSERVATION_UNAVAILABLE,
                            0.0, "required state is unknown or stale",
                            obs.observation_state, edges)
        if not _finite_vec(obs.cube_position_m):
            return Estimate(obs.step, obs.time_s, TaskState.OBSERVATION_UNAVAILABLE,
                            0.0, "Cube pose is nonfinite", ObservationState.UNKNOWN,
                            edges)
        if obs.features.get("tracking_error_m", 0.0) > self.tracking_error_threshold_m:
            return Estimate(obs.step, obs.time_s, TaskState.TRACKING_FAILURE,
                            0.9, "measured tool motion diverges from command",
                            obs.observation_state, edges)
        deep_gap = obs.features.get("deep_wall_gap_m")
        if deep_gap is not None and deep_gap <= self.deep_wall_gap_threshold_m:
            return Estimate(obs.step, obs.time_s, TaskState.DEEP_WALL_ARRIVAL,
                            0.85, "Cube is within the nominal deep-wall gap",
                            obs.observation_state, edges)
        tool_distance = obs.features.get("tool_cube_distance_m")
        near_tool = tool_distance is not None and tool_distance <= self.tool_distance_threshold_m
        if near_tool:
            velocity = obs.cube_velocity_m_s[0] if _finite_vec(obs.cube_velocity_m_s) else None
            if obs.command_advance_m is not None and velocity is not None:
                if obs.command_advance_m > 0.0 and velocity < self.advancement_speed_threshold_m_s:
                    return Estimate(obs.step, obs.time_s,
                                    TaskState.SUSPECTED_PREMATURE_BLOCKAGE,
                                    0.65, "commanded advancement with low Cube velocity",
                                    obs.observation_state, edges)
            if velocity is not None and velocity >= self.advancement_speed_threshold_m_s:
                return Estimate(obs.step, obs.time_s, TaskState.NORMAL_ADVANCEMENT,
                                0.70, "tool is near Cube and Cube advances",
                                obs.observation_state, edges)
            return Estimate(obs.step, obs.time_s, TaskState.TOOL_CONTACT,
                            0.55, "tool is geometrically near Cube",
                            obs.observation_state, edges)
        return Estimate(obs.step, obs.time_s, TaskState.FREE_SPACE_APPROACH,
                        0.60, "no tool proximity or deep-wall condition",
                        obs.observation_state, edges)


class E1TemporalHysteresis:
    """Causal E0 wrapper with activation/release dwell and unknown handling."""

    def __init__(self, base: E0GeometricMotionRules | None = None,
                 *, activation_frames: int = 3, release_frames: int = 3):
        self.base = base or E0GeometricMotionRules()
        self.activation_frames = max(1, int(activation_frames))
        self.release_frames = max(1, int(release_frames))
        self.state = TaskState.UNKNOWN
        self._candidate = TaskState.UNKNOWN
        self._candidate_count = 0

    def estimate(self, obs: FrameObservation) -> Estimate:
        raw = self.base.estimate(obs)
        if raw.state in (TaskState.OBSERVATION_UNAVAILABLE, TaskState.TRACKING_FAILURE,
                          TaskState.SUSPECTED_PREMATURE_BLOCKAGE):
            self.state = raw.state
            self._candidate = raw.state
            self._candidate_count = 0
            return raw
        if raw.state == self.state:
            self._candidate = raw.state
            self._candidate_count = 0
            return raw
        if raw.state != self._candidate:
            self._candidate = raw.state
            self._candidate_count = 1
        else:
            self._candidate_count += 1
        needed = self.release_frames if raw.state == TaskState.FREE_SPACE_APPROACH else self.activation_frames
        if self._candidate_count >= needed:
            previous = self.state
            self.state = raw.state
            self._candidate_count = 0
            return Estimate(raw.step, raw.time_s, self.state,
                            min(1.0, raw.confidence + 0.10),
                            f"causal hysteresis transition {previous.value}->{self.state.value}",
                            raw.observation_state, raw.edges)
        return Estimate(raw.step, raw.time_s, self.state,
                        max(0.0, raw.confidence - 0.15),
                        f"candidate {raw.state.value} awaiting {needed} frames",
                        raw.observation_state, raw.edges)
