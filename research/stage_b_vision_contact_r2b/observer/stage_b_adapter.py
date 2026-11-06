"""Build causal Stage-B observer frames from runtime receipts.

The adapter deliberately leaves missing q/dq/TCP/command fields missing.  It
never fills them from Cube USD, PhysX contacts, or an evaluation trace.
"""
from __future__ import annotations

from typing import Any, Mapping

from .contract import Health, SourceStamp, StageBObserverFrame, VisualPose


def _health(value: Any) -> Health:
    try:
        return Health(str(value))
    except ValueError:
        return Health.UNKNOWN


def _stamp(name: str, raw: Mapping[str, Any] | None, default_s: float) -> SourceStamp:
    raw = raw or {}
    health = _health(raw.get("health", "UNKNOWN"))
    stamp = raw.get("stamp_s", default_s)
    age = raw.get("age_s")
    return SourceStamp(
        source=str(raw.get("source", name)),
        stamp_s=None if stamp is None else float(stamp),
        age_s=None if age is None else float(age),
        provenance=str(raw.get("provenance", "runtime_receipt")),
        health=health,
    )


def frame_from_records(
    visual_record: Mapping[str, Any],
    *,
    timeline_record: Mapping[str, Any] | None = None,
    runtime_record: Mapping[str, Any] | None = None,
) -> StageBObserverFrame:
    """Convert one RGB detection plus optional runtime receipt into a frame.

    ``runtime_record`` may contain q/dq, TCP, controller and suction receipts.
    If it does not, the returned frame carries empty/None values and UNKNOWN
    source health, so downstream estimators abstain instead of using an oracle.
    """
    runtime = runtime_record or {}
    timeline = timeline_record or {}
    time_s = float(visual_record.get("time_s", timeline.get("physics_state_time_s", 0.0)))
    step = int(visual_record.get("physics_step", timeline.get("physics_step", 0)))
    if step < 1:
        step = 1
    status = str(visual_record.get("status", "UNKNOWN"))
    position = visual_record.get("estimated_cube_world_m")
    quaternion = visual_record.get("estimated_quaternion_wxyz")
    present = status == "PRESENT" and position is not None
    visual = VisualPose(
        None if not present else tuple(float(v) for v in position),
        None if quaternion is None else tuple(float(v) for v in quaternion),
        float(visual_record.get("confidence", 0.0) or 0.0),
        None if visual_record.get("reprojection_error_px") is None else float(visual_record["reprojection_error_px"]),
        None if visual_record.get("latency_s") is None else float(visual_record["latency_s"]),
        None if visual_record.get("marker_id") is None else int(visual_record["marker_id"]),
        source=str(visual_record.get("source", "camera_aruco_pnp_pixels")),
    )

    q = runtime.get("q_actual_rad") or {"left": (), "right": ()}
    dq = runtime.get("dq_actual_rad_s") or {"left": (), "right": ()}
    tcp = runtime.get("tcp_world_m") or {"left": None, "right": None}
    suction_command = runtime.get("suction_command") or {"left": None, "right": None}
    suction_state = runtime.get("suction_state") or {"left": None, "right": None}
    attachment = runtime.get("attachment_confidence") or {}
    camera_health = "PRESENT" if present else "UNKNOWN"
    stamps = {
        "physics": _stamp("physics", runtime.get("source_stamps", {}).get("physics"), time_s),
        "camera": _stamp(
            "camera",
            runtime.get("source_stamps", {}).get("camera"),
            time_s,
        ),
        "q": _stamp("q", runtime.get("source_stamps", {}).get("q"), time_s),
        "dq": _stamp("dq", runtime.get("source_stamps", {}).get("dq"), time_s),
        "tcp": _stamp("tcp", runtime.get("source_stamps", {}).get("tcp"), time_s),
    }
    stamps["camera"] = SourceStamp(
        stamps["camera"].source,
        stamps["camera"].stamp_s,
        stamps["camera"].age_s,
        stamps["camera"].provenance if stamps["camera"].provenance != "runtime_receipt" else str(visual_record.get("source", "camera_pixels")),
        _health(camera_health),
    )
    sensor_health = {
        "camera": _health(camera_health),
        "q": stamps["q"].health,
        "dq": stamps["dq"].health,
        "tcp": stamps["tcp"].health,
    }
    uncertainty = dict(runtime.get("uncertainty") or {})
    uncertainty.setdefault("visual_confidence", max(0.0, 1.0 - visual.confidence))
    if stamps["camera"].age_s is not None:
        uncertainty.setdefault("camera_age_s", max(0.0, stamps["camera"].age_s))
    if not present:
        uncertainty.setdefault("missing_visual", 1.0)
    controller_reference = dict(runtime.get("controller_reference") or {})
    if "observed_cube_velocity_x_m_s" in runtime:
        controller_reference["observed_cube_velocity_x_m_s"] = runtime["observed_cube_velocity_x_m_s"]
    return StageBObserverFrame(
        physics_step=step,
        sim_time_s=time_s,
        dt_s=float(runtime.get("dt_s", 1.0 / 60.0)),
        source_stamps=stamps,
        q_actual_rad={k: tuple(float(v) for v in q.get(k, ())) for k in ("left", "right")},
        dq_actual_rad_s={k: tuple(float(v) for v in dq.get(k, ())) for k in ("left", "right")},
        tcp_world_m={k: (None if tcp.get(k) is None else tuple(float(v) for v in tcp[k])) for k in ("left", "right")},
        controller_reference=controller_reference,
        command_id=runtime.get("command_id"),
        command_stamp_s=runtime.get("command_stamp_s"),
        commanded_advancement_m=runtime.get("commanded_advancement_m"),
        visual_cube=visual,
        suction_command=dict(suction_command),
        suction_state=dict(suction_state),
        oracle_contact_pairs=tuple(runtime.get("oracle_contact_pairs") or ()),
        sensor_health=sensor_health,
        uncertainty=uncertainty,
        attachment_confidence={
            side: None if attachment.get(side) is None else float(attachment[side])
            for side in ("left", "right")
        },
    )


def causal_payload(frame: StageBObserverFrame) -> dict[str, Any]:
    """Return the online payload and assert the oracle channel is excluded."""
    payload = frame.estimator_payload()
    if "oracle_contact_pairs" in payload:
        raise AssertionError("oracle contact leaked into causal payload")
    return payload
