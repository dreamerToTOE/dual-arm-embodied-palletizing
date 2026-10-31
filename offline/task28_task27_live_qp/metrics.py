"""Metrics for live Task27 cooperative QP JSONL receipts.

The Isaac bridge deliberately writes one JSON object per event.  This module
keeps the experiment comparison independent from the simulator process: it
only consumes recorded actual feedback and solver diagnostics, and never
classifies a run as successful.  The returned dictionary is suitable for
embedding in a report or writing as ``metrics.json``.
"""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
from typing import Iterable

import numpy as np


def _norm(values) -> float:
    arr = np.asarray(values, dtype=float)
    return float(np.linalg.norm(arr)) if arr.size else 0.0


def _percentile(values, percentile: float):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), percentile))


def _read_events(source: Path | str | Iterable[dict]):
    if isinstance(source, (str, Path)):
        with Path(source).open("r", encoding="utf-8") as stream:
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(event, dict):
                    yield event
        return
    yield from source


def _pose(event, side: str):
    pose = event.get("tcp_pose_world", {}).get(side)
    if pose is not None:
        matrix = np.asarray(pose, dtype=float)
        if matrix.shape == (4, 4) and np.isfinite(matrix).all():
            return matrix
    point = event.get("tcp_world_m", {}).get(side)
    if point is None:
        return None
    point = np.asarray(point, dtype=float).reshape(-1)
    if point.shape != (3,) or not np.isfinite(point).all():
        return None
    matrix = np.eye(4, dtype=float)
    matrix[:3, 3] = point
    return matrix


def summarize_events(events: Iterable[dict]) -> dict:
    """Summarize actual motion and QP diagnostics from bridge events.

    ``TORQUE_FRAME`` entries are the only samples used for motion metrics;
    boot, readback and fail-closed records are retained as status evidence.
    Distances are metres, angles are radians, torques are N*m and solver times
    are seconds.  Empty or incomplete logs produce a valid summary with
    ``sample_count=0`` rather than an inferred PASS.
    """
    frames = []
    statuses = Counter()
    fail_closed = []
    readback_count = 0
    stop_events = []
    active_events = 0
    all_events = 0
    for event in _read_events(events):
        all_events += 1
        if event.get("event") == "TORQUE_FRAME":
            frames.append(event)
            qp = event.get("qp") or {}
            status = str(qp.get("status", "UNKNOWN"))
            statuses[status] += 1
        elif event.get("event") == "FAIL_CLOSED":
            fail_closed.append(event)
        elif event.get("event") == "IMMEDIATE_READBACK":
            readback_count += 1
        elif event.get("event") == "TASK27_TORQUE_BRIDGE_ACTIVE":
            active_events += 1
        elif event.get("event") == "TASK27_TORQUE_BRIDGE_STOPPED":
            stop_events.append(event)

    pose_frames = []
    for event in frames:
        left, right = _pose(event, "left"), _pose(event, "right")
        if left is not None and right is not None:
            pose_frames.append((event, left, right))

    common_disp = []
    relative_disp = []
    left_disp = []
    right_disp = []
    common_error_translation = []
    common_error_rotation = []
    relative_error_translation = []
    relative_error_rotation = []
    if pose_frames:
        _, left0, right0 = pose_frames[0]
        common0 = 0.5 * (left0[:3, 3] + right0[:3, 3])
        relative0 = np.linalg.inv(left0) @ right0
        for event, left, right in pose_frames:
            common = 0.5 * (left[:3, 3] + right[:3, 3])
            relative = np.linalg.inv(left) @ right
            common_disp.append(_norm(common - common0))
            relative_disp.append(_norm(relative[:3, 3] - relative0[:3, 3]))
            left_disp.append(_norm(left[:3, 3] - left0[:3, 3]))
            right_disp.append(_norm(right[:3, 3] - right0[:3, 3]))
            qp = event.get("qp") or {}
            ce = np.asarray(qp.get("common_tcp_error", []), dtype=float).reshape(-1)
            re = np.asarray(qp.get("relative_tcp_error", []), dtype=float).reshape(-1)
            if ce.shape == (6,) and np.isfinite(ce).all():
                common_error_translation.append(_norm(ce[:3]))
                common_error_rotation.append(_norm(ce[3:]))
            if re.shape == (6,) and np.isfinite(re).all():
                relative_error_translation.append(_norm(re[:3]))
                relative_error_rotation.append(_norm(re[3:]))

    solve_times = []
    common_residuals = []
    relative_residuals = []
    optimizer_iterations = []
    saturated = 0
    rate_active = 0
    dq_inf = []
    for event in frames:
        qp = event.get("qp") or {}
        for field, target in (("solve_wall_time_s", solve_times),
                              ("common_acceleration_residual_inf", common_residuals),
                              ("relative_acceleration_residual_inf", relative_residuals),
                              ("optimizer_iterations", optimizer_iterations)):
            value = qp.get(field)
            if isinstance(value, (int, float)) and np.isfinite(value):
                target.append(float(value))
        if qp.get("torque_saturated_indices"):
            saturated += 1
        if qp.get("torque_rate_active_indices"):
            rate_active += 1
        qpdq = event.get("dq_rad_s") or {}
        for side in ("left", "right"):
            values = qpdq.get(side)
            if values is not None:
                arr = np.asarray(values, dtype=float).reshape(-1)
                if arr.size and np.isfinite(arr).all():
                    dq_inf.append(float(np.max(np.abs(arr))))

    def extrema(values):
        if not values:
            return {"max": None, "mean": None, "p95": None}
        return {"max": float(np.max(values)), "mean": float(np.mean(values)),
                "p95": _percentile(values, 95.0)}

    duration = None
    if len(frames) > 1:
        times = [float(frame["time_s"]) for frame in frames if isinstance(frame.get("time_s"), (int, float))]
        if len(times) > 1:
            duration = float(max(times) - min(times))
    return {
        "schema": "task28.task27.live_qp_metrics.v1",
        "event_count": all_events,
        "sample_count": len(frames),
        "pose_sample_count": len(pose_frames),
        "duration_s": duration,
        "qp_status_counts": dict(statuses),
        "qp_optimal_count": int(statuses.get("QP_OPTIMAL", 0)),
        "fail_closed_count": len(fail_closed),
        "last_failure": fail_closed[-1].get("error") if fail_closed else None,
        "active_event_count": active_events,
        "immediate_readback_count": readback_count,
        "torque_write_count": (stop_events[-1].get("torque_writes")
                                if stop_events else None),
        "position_arm_write_count": (stop_events[-1].get("position_arm_write_count")
                                      if stop_events else None),
        "common_displacement_m": extrema(common_disp),
        "relative_displacement_m": extrema(relative_disp),
        "left_displacement_m": extrema(left_disp),
        "right_displacement_m": extrema(right_disp),
        "common_error_translation_m": extrema(common_error_translation),
        "common_error_rotation_rad": extrema(common_error_rotation),
        "relative_error_translation_m": extrema(relative_error_translation),
        "relative_error_rotation_rad": extrema(relative_error_rotation),
        "dq_inf_rad_s": extrema(dq_inf),
        "solve_wall_time_s": extrema(solve_times),
        "common_acceleration_residual_inf": extrema(common_residuals),
        "relative_acceleration_residual_inf": extrema(relative_residuals),
        "optimizer_iterations": extrema(optimizer_iterations),
        "frames_with_torque_saturation": saturated,
        "frames_with_torque_rate_active": rate_active,
    }


def summarize_log(path: Path | str, output: Path | str | None = None) -> dict:
    """Summarize a JSONL bridge log and optionally write JSON output."""
    result = summarize_events(path)
    if output is not None:
        output_path = Path(output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    print(json.dumps(summarize_log(args.log, args.output), indent=2, sort_keys=True))
