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
from research.stage_b_vision_contact_r2b.observer.runtime_adapter import frame_from_synchronized_records
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


def _edge_metrics(rows: list[tuple[dict, object]], edge: str, truth: str) -> dict:
    counts = Counter()
    for label, estimate in rows:
        edge_label = label.get("edges", {}).get(edge, {})
        if edge_label.get("status", "").startswith("UNAVAILABLE"):
            continue
        expected = bool(edge_label.get("pair_present")) if truth == "possible" else edge_label.get("status") == "LOADED_SUPPORTED"
        predicted = float(estimate.edge_probabilities.get(edge, 0.5)) > 0.55
        counts[(expected, predicted)] += 1
    tp, fp, fn, tn = (counts[(True, True)], counts[(False, True)], counts[(True, False)], counts[(False, False)])
    precision = tp / (tp + fp) if tp + fp else None
    recall = tp / (tp + fn) if tp + fn else None
    return {"samples": sum(counts.values()), "tp": tp, "fp": fp, "fn": fn, "tn": tn, "precision": precision, "recall": recall, "f1": None if precision is None or recall is None or precision + recall == 0 else 2 * precision * recall / (precision + recall)}


def evaluate(evidence: Path, runtime_trace: Path | None = None, contact_labels: Path | None = None) -> dict:
    detections = list(_jsonl(evidence / "visual_detections.jsonl"))
    timeline = {int(row["physics_step"]): row for row in _jsonl(evidence / "r2b_camera/camera_timeline.jsonl")}
    receipts = json.loads((evidence / "execution_phase_receipts.json").read_text(encoding="utf-8"))["receipts"]
    runtime_records = list(_jsonl(runtime_trace)) if runtime_trace else []
    labels = {int(row["physics_step"]): row for row in _jsonl(contact_labels)} if contact_labels else {}
    e0, e1, selector = E0ContactEstimator(), E1TemporalContactEstimator(), ShadowModeSelector()
    e0_counts, e1_counts, mode_counts = Counter(), Counter(), Counter()
    phase_stats = defaultdict(lambda: {"frames": 0, "detections": 0, "e0_states": Counter(), "e1_states": Counter(), "shadow_modes": Counter()})
    causal_frames = 0
    timing_age = []
    writer_latency = []
    transition_rows = []
    edge_rows_e0 = defaultdict(list)
    edge_rows_e1 = defaultdict(list)
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
        frame = frame_from_synchronized_records(row, runtime_records, timeline_record=tl) if runtime_trace else frame_from_records(row, timeline_record=tl, runtime_record=runtime)
        causal_payload(frame)
        if row.get("status") == "PRESENT":
            causal_frames += 1
        a, b = e0.estimate(frame), e1.estimate(frame)
        if int(row.get("physics_step", -1)) in labels:
            label = labels[int(row["physics_step"])]
            for edge in label.get("edges", {}):
                edge_rows_e0[edge].append((label, a))
                edge_rows_e1[edge].append((label, b))
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
        "input_policy": "causal RGB plus latest-past physical trace; controller command receipts retain UNKNOWN when absent and PhysX labels stay in the offline oracle channel",
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
            "status": "AVAILABLE_OFFLINE_ONLY" if contact_labels else "UNAVAILABLE",
            "reason": "per-step PhysX labels are scored only after causal estimation; they are never included in estimator_payload",
            "e0_possible_pair": {edge: _edge_metrics(rows, edge, "possible") for edge, rows in edge_rows_e0.items()} if contact_labels else None,
            "e1_possible_pair": {edge: _edge_metrics(rows, edge, "possible") for edge, rows in edge_rows_e1.items()} if contact_labels else None,
            "e0_loaded_supported": {edge: _edge_metrics(rows, edge, "loaded") for edge, rows in edge_rows_e0.items()} if contact_labels else None,
            "e1_loaded_supported": {edge: _edge_metrics(rows, edge, "loaded") for edge, rows in edge_rows_e1.items()} if contact_labels else None,
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
    parser.add_argument("--runtime-trace", type=Path)
    parser.add_argument("--contact-labels", type=Path)
    args = parser.parse_args()
    result = evaluate(args.evidence, args.runtime_trace, args.contact_labels)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
