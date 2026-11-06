#!/usr/bin/env python3
"""Evaluate causal E0/E1 and read-only shadow proposals on physical07.

The physical07 RGB stream is legacy single-marker data.  This script reports
its coverage and abstention behaviour, but does not turn Cube/PhysX evaluation
fields into online labels or claim a contact confusion matrix.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from statistics import median

from research.stage_b_vision_contact_r2b.estimator.stage_b_contact import E0ContactEstimator, E1TemporalContactEstimator
from research.stage_b_vision_contact_r2b.observer.stage_b_adapter import causal_payload, frame_from_records
from research.stage_b_vision_contact_r2b.shadow.mode_selector import ShadowModeSelector


def _jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def _phase_for(time_s: float, receipts: list[dict]) -> str | None:
    for receipt in receipts:
        if float(receipt["start_sim_time_s"]) <= time_s <= float(receipt["end_sim_time_s"]):
            return str(receipt["skill_id"])
    return None


def evaluate(evidence: Path) -> dict:
    detections = list(_jsonl(evidence / "visual_detections.jsonl"))
    timeline = {int(row["physics_step"]): row for row in _jsonl(evidence / "r2b_camera/camera_timeline.jsonl")}
    receipts = json.loads((evidence / "execution_phase_receipts.json").read_text(encoding="utf-8"))["receipts"]
    e0, e1, selector = E0ContactEstimator(), E1TemporalContactEstimator(), ShadowModeSelector()
    e0_counts, e1_counts, mode_counts = Counter(), Counter(), Counter()
    phase_stats = defaultdict(lambda: {"frames": 0, "detections": 0, "e0_states": Counter(), "e1_states": Counter(), "shadow_modes": Counter()})
    causal_frames = 0
    timing_age = []
    writer_latency = []
    transition_rows = []
    previous_e1 = None
    for row in detections:
        tl = timeline.get(int(row.get("physics_step", -1)), {})
        age_s = None
        if tl.get("physics_time_after_render_wait_s") is not None:
            age_s = max(0.0, float(tl["physics_time_after_render_wait_s"]) - float(tl.get("physics_state_time_s", row.get("time_s", 0.0))))
            timing_age.append(age_s)
        if tl.get("writer_complete_wall_s") is not None and tl.get("physics_state_receipt_wall_s") is not None:
            writer_latency.append(float(tl["writer_complete_wall_s"]) - float(tl["physics_state_receipt_wall_s"]))
        runtime = {
            "source_stamps": {
                "camera": {"source": "physical07_camera_timeline", "stamp_s": row.get("time_s"), "age_s": age_s, "health": "PRESENT" if row.get("status") == "PRESENT" else "UNKNOWN", "provenance": "legacy_single_marker_id23"},
            },
            "uncertainty": {"writer_latency_s": writer_latency[-1] if writer_latency else None},
        }
        frame = frame_from_records(row, timeline_record=tl, runtime_record=runtime)
        causal_payload(frame)
        if row.get("status") == "PRESENT":
            causal_frames += 1
        a, b = e0.estimate(frame), e1.estimate(frame)
        decision = selector.select(b)
        e0_counts[a.state.value] += 1
        e1_counts[b.state.value] += 1
        mode_counts[decision.mode.value] += 1
        phase = _phase_for(float(row.get("time_s", 0.0)), receipts)
        if phase is not None:
            p = phase_stats[phase]
            p["frames"] += 1
            p["detections"] += int(row.get("status") == "PRESENT")
            p["e0_states"][a.state.value] += 1
            p["e1_states"][b.state.value] += 1
            p["shadow_modes"][decision.mode.value] += 1
        if previous_e1 != b.state.value:
            transition_rows.append({"time_s": row.get("time_s"), "from": previous_e1, "to": b.state.value, "reason": list(b.causes)})
            previous_e1 = b.state.value

    phase_output = {}
    for name, p in phase_stats.items():
        phase_output[name] = {
            "frames": p["frames"],
            "detections": p["detections"],
            "detection_rate": None if not p["frames"] else p["detections"] / p["frames"],
            "e0_states": dict(p["e0_states"]),
            "e1_states": dict(p["e1_states"]),
            "shadow_modes": dict(p["shadow_modes"]),
        }
    return {
        "schema": "task28.r2b.stage_b_contact_evaluation.v1",
        "evidence": str(evidence),
        "input_policy": "causal RGB detection + camera timing only; q/dq/TCP/commands/suction absent from physical07 export are UNKNOWN",
        "marker_configuration": "legacy single marker ID23; incompatible with cal24 four-marker board",
        "frames": len(detections),
        "visual_present_frames": causal_frames,
        "visual_availability": causal_frames / len(detections) if detections else 0.0,
        "e0_state_counts": dict(e0_counts),
        "e1_state_counts": dict(e1_counts),
        "shadow_mode_counts": dict(mode_counts),
        "phase_statistics": phase_output,
        "e1_transition_rows": transition_rows,
        "timing": {
            "camera_age_sim_s_median": median(timing_age) if timing_age else None,
            "camera_age_sim_s_max": max(timing_age) if timing_age else None,
            "writer_latency_wall_s_median": median(writer_latency) if writer_latency else None,
            "writer_latency_wall_s_max": max(writer_latency) if writer_latency else None,
        },
        "independent_contact_labels": {
            "status": "UNAVAILABLE",
            "reason": "execution phase receipts identify executed intervals but do not label per-frame loaded contact; Cube/PhysX fields remain offline evaluation only",
            "confusion_matrix": None,
            "precision_recall_f1": None,
            "false_positive_contact_events": None,
            "false_stalled_events": None,
            "transition_detection_delay_s": None,
        },
        "safety": {
            "selector_is_read_only": True,
            "controller_switching": False,
            "motion_commands_issued": False,
            "side_preflight_required": True,
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, default=Path("research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical07"))
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = evaluate(args.evidence)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
