"""Join camera observations to the latest causal runtime receipt.

The join is step/time bounded: a future robot-state sample is never selected,
and an old sample remains explicitly STALE for downstream abstention.
"""
from __future__ import annotations

from typing import Any, Iterable, Mapping

from .stage_b_adapter import frame_from_records


def runtime_receipt_from_trace(
    visual_record: Mapping[str, Any],
    runtime_record: Mapping[str, Any] | None,
    *,
    max_age_s: float = 0.25,
) -> dict[str, Any] | None:
    """Normalize one physical-trace row for ``frame_from_records``."""
    if runtime_record is None:
        return None
    runtime = dict(runtime_record)
    visual_time = float(visual_record.get("time_s", 0.0))
    runtime_time = float(runtime.get("time_s", 0.0))
    age = visual_time - runtime_time
    if age < -1e-9:
        raise ValueError("runtime receipt is from the future")
    health = "PRESENT" if age <= max_age_s else "STALE"
    provenance = "physical_trace.v1.latest_past_receipt"
    field_present = {
        "physics": True,
        "q": runtime.get("q_actual_rad") is not None,
        "dq": runtime.get("dq_actual_rad_s") is not None,
        "tcp": runtime.get("tcp_world_m") is not None,
        "suction": runtime.get("suction_command") is not None and runtime.get("suction_state") is not None,
        "command": runtime.get("controller_command") is not None,
    }
    runtime["source_stamps"] = {
        name: {
            "source": name,
            "stamp_s": runtime_time,
            "age_s": age,
            "health": health if field_present[name] else "UNKNOWN",
            "provenance": provenance if field_present[name] else f"physical_trace.v1.missing_{name}",
        }
        for name in ("physics", "q", "dq", "tcp", "suction", "command")
    }
    runtime["controller_reference"] = {
        "control_owner": runtime.get("control_owner"),
        "control_mode": runtime.get("control_mode"),
        "controller_command": runtime.get("controller_command"),
    }
    velocity = runtime.get("cube_velocity_m_s")
    if velocity is not None and len(velocity) >= 1:
        runtime["observed_cube_velocity_x_m_s"] = float(velocity[0])
    runtime["runtime_age_s"] = age
    runtime["runtime_health"] = health
    command = runtime.get("controller_command")
    if command is not None:
        runtime["command_id"] = f"sequence:{command.get('sequence', 'unknown')}"
    # The trace has no measured attachment confidence; preserve UNKNOWN.
    runtime.setdefault("attachment_confidence", {"left": None, "right": None})
    return runtime


def latest_past_runtime(
    visual_record: Mapping[str, Any],
    runtime_records: Iterable[Mapping[str, Any]],
) -> Mapping[str, Any] | None:
    """Select the latest record with both step and timestamp no later than RGB."""
    visual_step = int(visual_record.get("physics_step", -1))
    visual_time = float(visual_record.get("time_s", 0.0))
    candidates = []
    for record in runtime_records:
        step = int(record.get("physics_step", -1))
        time_s = float(record.get("time_s", 0.0))
        if step <= visual_step and time_s <= visual_time + 1e-9:
            candidates.append((step, time_s, record))
    if not candidates:
        return None
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def frame_from_synchronized_records(
    visual_record: Mapping[str, Any],
    runtime_records: Iterable[Mapping[str, Any]],
    *,
    timeline_record: Mapping[str, Any] | None = None,
    max_age_s: float = 0.25,
):
    """Build a causal observer frame from camera + latest-past runtime data."""
    runtime = latest_past_runtime(visual_record, runtime_records)
    return frame_from_records(
        visual_record,
        timeline_record=timeline_record,
        runtime_record=runtime_receipt_from_trace(visual_record, runtime, max_age_s=max_age_s),
    )
