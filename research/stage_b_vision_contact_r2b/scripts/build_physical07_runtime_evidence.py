#!/usr/bin/env python3
"""Build compact causal observations and offline PhysX contact labels.

The large raw trace stays in its source worktree.  This artifact records its
hashes, the latest-past state join, and labels in a separate oracle channel.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median
from typing import Any

from research.stage_b_vision_contact_r2b.observer.runtime_adapter import (
    frame_from_synchronized_records,
    latest_past_runtime,
    runtime_receipt_from_trace,
)


def _jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _pair_has(colliders: list[str], token: str) -> bool:
    return any(token in path for path in colliders)


EDGE_TOKENS = {
    "LEFT_TOOL_CUBE": "/left_fr3/fr3_link8/side_suction_tool/",
    "RIGHT_TOOL_CUBE": "/right_fr3/fr3_link8/side_suction_tool/",
    "CUBE_TABLE": "/World/Table",
    "CUBE_DEEP_WALL": "/World/Task27/TruckBox/WallDeep",
}


def _edge_events(events: list[dict[str, Any]], token: str) -> list[dict[str, Any]]:
    return [e for e in events if _pair_has(e["colliders"], token) and _pair_has(e["colliders"], "/World/Task27/Supply/Cube_01")]


def _label_stats(events: list[dict[str, Any]]) -> dict[str, Any]:
    impulses = [sum(float(v) ** 2 for v in e.get("impulse_world_Ns", [])) ** 0.5 for e in events]
    return {
        "event_count": len(events),
        "max_impulse_Ns": max(impulses) if impulses else 0.0,
        "min_separation_m": min((float(p["separation_m"]) for e in events for p in e.get("points", [])), default=None),
    }


def _edge_label(events: list[dict[str, Any]], token: str, persistence_steps: int) -> dict[str, Any]:
    matched = _edge_events(events, token)
    stats = _label_stats(matched)
    active = stats["max_impulse_Ns"] > 1e-8 or (stats["min_separation_m"] is not None and stats["min_separation_m"] < -1e-4)
    if not matched:
        status = "SUPPORTED_ABSENT"
    elif active and persistence_steps >= 2:
        status = "LOADED_SUPPORTED"
    else:
        status = "POSSIBLE_PAIR"
    return {"status": status, "pair_present": bool(matched), "persistence_steps": persistence_steps, **stats}


def build(visual_root: Path, runtime_trace: Path, contact_trace: Path, out_dir: Path) -> dict[str, Any]:
    visual = list(_jsonl(visual_root / "visual_detections.jsonl"))
    timeline = {int(row["physics_step"]): row for row in _jsonl(visual_root / "r2b_camera/camera_timeline.jsonl")}
    runtime = list(_jsonl(runtime_trace))
    runtime_by_step = {int(row["physics_step"]): row for row in runtime}
    events_by_step: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for event in _jsonl(contact_trace):
        events_by_step[int(event["physics_step"])].append(event)
    edge_presence = {edge: {step for step, events in events_by_step.items() if _edge_events(events, token)} for edge, token in EDGE_TOKENS.items()}

    def persistence(edge: str, step: int) -> int:
        present = edge_presence[edge]
        if step not in present:
            return 0
        count = 0
        while step - count in present:
            count += 1
        return count

    out_dir.mkdir(parents=True, exist_ok=True)
    observation_path = out_dir / "causal_observations.jsonl"
    label_path = out_dir / "contact_labels_oracle.jsonl"
    ages = []
    join_counts = Counter()
    with observation_path.open("w", encoding="utf-8") as observations, label_path.open("w", encoding="utf-8") as labels:
        for row in visual:
            step = int(row.get("physics_step", -1))
            tl = timeline.get(step, {})
            runtime_row = latest_past_runtime(row, runtime)
            frame = frame_from_synchronized_records(row, runtime, timeline_record=tl)
            payload = frame.estimator_payload()
            runtime_normalized = runtime_receipt_from_trace(row, runtime_row)
            age = None if runtime_normalized is None else float(runtime_normalized["runtime_age_s"])
            if age is not None:
                ages.append(age)
            join_counts["joined" if runtime_row is not None else "missing"] += 1
            if age is not None and age > 0.25:
                join_counts["stale"] += 1
            observations.write(json.dumps({
                "physics_step": step,
                "camera_time_s": float(row.get("time_s", 0.0)),
                "visual_status": row.get("status", "UNKNOWN"),
                "visual_confidence": row.get("confidence", 0.0),
                "runtime_step": None if runtime_row is None else runtime_row.get("physics_step"),
                "runtime_time_s": None if runtime_normalized is None else runtime_normalized.get("time_s"),
                "runtime_age_s": age,
                "runtime_health": None if runtime_normalized is None else runtime_normalized.get("runtime_health"),
                "causal_payload": payload,
            }, ensure_ascii=False, allow_nan=False) + "\n")

            label_events = events_by_step.get(step, [])
            label = {
                "physics_step": step,
                "time_s": float(row.get("time_s", 0.0)),
                "source": "PhysX_simulation_contact_report",
                "status": "KNOWN_PARTIAL" if step in events_by_step else "UNKNOWN",
                "edges": {
                    edge: _edge_label(label_events, token, persistence(edge, step))
                    for edge, token in EDGE_TOKENS.items()
                },
                "edge_support": {
                    "LEFT_TOOL_CUBE": "SUPPORTED",
                    "RIGHT_TOOL_CUBE": "SUPPORTED",
                    "CUBE_TABLE": "SUPPORTED",
                    "CUBE_DEEP_WALL": "SUPPORTED",
                    "CUBE_SIDE_WALL": "UNAVAILABLE_NO_REPORTED_PAIR",
                },
                "event_summary": _label_stats(label_events),
                "label_policy": "LOADED_SUPPORTED requires nonzero impulse or >0.1 mm penetration and two-step persistence; raw pair presence remains POSSIBLE_PAIR",
            }
            labels.write(json.dumps(label, ensure_ascii=False, allow_nan=False) + "\n")

    trace_steps = [int(row["physics_step"]) for row in runtime if "physics_step" in row]
    summary = {
        "schema": "task28.r2b.physical07_runtime_integration.v1",
        "visual_frames": len(visual),
        "runtime_rows": len(runtime),
        "runtime_step_first_last": [min(trace_steps), max(trace_steps)] if trace_steps else None,
        "contact_event_rows": sum(len(v) for v in events_by_step.values()),
        "contact_event_steps": len(events_by_step),
        "join_counts": dict(join_counts),
        "runtime_age_s_median": median(ages) if ages else None,
        "runtime_age_s_max": max(ages) if ages else None,
        "causal_future_samples_selected": 0,
        "attachment_confidence_source": "absent_in_physical_trace; preserved UNKNOWN",
        "oracle_channel_separate": True,
        "source_files": {
            "visual_detections": str(visual_root / "visual_detections.jsonl"),
            "camera_timeline": str(visual_root / "r2b_camera/camera_timeline.jsonl"),
                "visual_detections_sha256": _sha256(visual_root / "visual_detections.jsonl"),
                "camera_timeline_sha256": _sha256(visual_root / "r2b_camera/camera_timeline.jsonl"),
            "runtime_trace": str(runtime_trace),
            "contact_trace": str(contact_trace),
            "runtime_trace_sha256": _sha256(runtime_trace),
            "contact_trace_sha256": _sha256(contact_trace),
        },
        "output_files": {
            "causal_observations": str(observation_path),
            "contact_labels_oracle": str(label_path),
        },
    }
    (out_dir / "runtime_integration_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--visual-root", type=Path, required=True)
    parser.add_argument("--runtime-trace", type=Path, required=True)
    parser.add_argument("--contact-trace", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.visual_root, args.runtime_trace, args.contact_trace, args.out_dir), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
