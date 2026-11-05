"""Step-aligned Stage B observation contract.

The contract keeps online estimator inputs separate from privileged simulator
labels.  A frame may carry an oracle diagnostic record, but ``estimator_payload``
never serializes it.
"""
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from enum import Enum
import math
from typing import Any, Mapping, Optional

class Health(str, Enum):
    PRESENT = "PRESENT"
    UNKNOWN = "UNKNOWN"
    STALE = "STALE"
    FAILED = "FAILED"

@dataclass(frozen=True)
class SourceStamp:
    source: str
    stamp_s: Optional[float]
    age_s: Optional[float]
    provenance: str
    health: Health

@dataclass(frozen=True)
class VisualPose:
    position_m: Optional[tuple[float, float, float]]
    quaternion_wxyz: Optional[tuple[float, float, float, float]]
    confidence: float
    reprojection_error_px: Optional[float]
    latency_s: Optional[float]
    marker_id: Optional[int]
    source: str = "camera_aruco_pnp"

@dataclass(frozen=True)
class StageBObserverFrame:
    physics_step: int
    sim_time_s: float
    dt_s: float
    source_stamps: Mapping[str, SourceStamp]
    q_actual_rad: Mapping[str, tuple[float, ...]]
    dq_actual_rad_s: Mapping[str, tuple[float, ...]]
    tcp_world_m: Mapping[str, Optional[tuple[float, float, float]]]
    controller_reference: Mapping[str, Any]
    command_id: Optional[str]
    command_stamp_s: Optional[float]
    commanded_advancement_m: Optional[float]
    visual_cube: VisualPose
    suction_command: Mapping[str, Optional[bool]]
    suction_state: Mapping[str, Optional[bool]]
    oracle_contact_pairs: tuple[Mapping[str, Any], ...] = ()
    sensor_health: Mapping[str, Health] = field(default_factory=dict)
    uncertainty: Mapping[str, float] = field(default_factory=dict)

    def validate(self) -> None:
        if self.physics_step < 1 or not math.isfinite(self.sim_time_s) or self.dt_s <= 0:
            raise ValueError("invalid physics time")
        for side in ("left", "right"):
            if side not in self.q_actual_rad or side not in self.dq_actual_rad_s:
                raise ValueError(f"missing q/dq for {side}")
            if len(self.q_actual_rad[side]) != len(self.dq_actual_rad_s[side]):
                raise ValueError(f"q/dq length mismatch for {side}")
        if self.commanded_advancement_m is not None and not math.isfinite(self.commanded_advancement_m):
            raise ValueError("commanded advancement must be finite or None")
        if self.visual_cube.confidence < 0.0 or self.visual_cube.confidence > 1.0:
            raise ValueError("visual confidence outside [0,1]")

    def estimator_payload(self) -> dict[str, Any]:
        """Return deployable fields only; oracle contacts are intentionally absent."""
        self.validate()
        return {
            "physics_step": self.physics_step,
            "sim_time_s": self.sim_time_s,
            "dt_s": self.dt_s,
            "source_stamps": {k: asdict(v) for k, v in self.source_stamps.items()},
            "q_actual_rad": {k: list(v) for k, v in self.q_actual_rad.items()},
            "dq_actual_rad_s": {k: list(v) for k, v in self.dq_actual_rad_s.items()},
            "tcp_world_m": {k: (None if v is None else list(v)) for k, v in self.tcp_world_m.items()},
            "controller_reference": dict(self.controller_reference),
            "command_id": self.command_id,
            "command_stamp_s": self.command_stamp_s,
            "commanded_advancement_m": self.commanded_advancement_m,
            "visual_cube": asdict(self.visual_cube),
            "suction_command": dict(self.suction_command),
            "suction_state": dict(self.suction_state),
            "sensor_health": {k: v.value for k, v in self.sensor_health.items()},
            "uncertainty": dict(self.uncertainty),
        }

    def to_dict(self) -> dict[str, Any]:
        payload = self.estimator_payload()
        payload["oracle_contact_pairs"] = [dict(v) for v in self.oracle_contact_pairs]
        payload["schema"] = "task28.r2b.observer_frame.v1"
        return payload

class StageBFrameBuffer:
    """Single-owner monotonic buffer; no future-frame lookup."""
    def __init__(self):
        self.owner: Optional[str] = None
        self.last_step = 0
        self.frames: list[StageBObserverFrame] = []

    def append(self, frame: StageBObserverFrame, owner: str) -> None:
        if self.owner is None:
            self.owner = owner
        if owner != self.owner:
            raise RuntimeError("observer owner conflict")
        frame.validate()
        if frame.physics_step <= self.last_step:
            raise ValueError("non-monotonic or duplicate physics step")
        self.frames.append(frame)
        self.last_step = frame.physics_step
