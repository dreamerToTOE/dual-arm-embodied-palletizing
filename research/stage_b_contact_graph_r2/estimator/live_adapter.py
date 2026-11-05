"""Narrow adapter for a future runtime observer.

It accepts the same trace-shaped row produced by PhysicalTrace and deliberately
ignores ``oracle_reference``.  Wiring this adapter into Isaac is deferred until
command timestamps, suction feedback, and side-wall monitoring are available.
"""
from __future__ import annotations
from typing import Mapping
from .r2_contact_estimator import FrameObservation, ObservationState, ContactEdge, E0GeometricMotionRules, Estimate


def frame_from_runtime_row(row: Mapping) -> FrameObservation:
    ii = row.get("estimator_input", {})
    features = {k: v for k, v in ii.get("features", {}).items() if v is not None}
    edges = {ContactEdge(k): ObservationState(v) for k, v in ii.get("edge_observations", {}).items()}
    return FrameObservation(
        step=int(row["physics_step"]), time_s=float(row["time_s"]),
        cube_position_m=tuple(row["cube_world_m"]), cube_velocity_m_s=tuple(row["cube_velocity_m_s"]),
        left_tcp_m=tuple(row["tcp_world_m"]["left"]), right_tcp_m=tuple(row["tcp_world_m"]["right"]),
        command_advance_m=ii.get("command_advance_m"),
        observation_state=ObservationState(ii.get("observation_state", "UNKNOWN")),
        edge_observations=edges, features=features)


def estimate_runtime_row(row: Mapping, estimator=None) -> Estimate:
    return (estimator or E0GeometricMotionRules()).estimate(frame_from_runtime_row(row))
