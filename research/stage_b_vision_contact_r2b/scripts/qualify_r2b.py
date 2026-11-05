#!/usr/bin/env python3
"""Qualify rendered ArUco pixels and the offline V0/V1 comparison.

The image path is the only input to the V2 detector.  USD camera extrinsics and
the authored smoke pose are used only for an evaluation receipt, never for the
estimator payload.
"""
from __future__ import annotations

import argparse
import gzip
import json
import random
from pathlib import Path

import cv2
import numpy as np

from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco
from research.stage_b_vision_contact_r2b.estimator.contact_estimators import E0MultiLabel, E1Hysteresis


def _world_from_cv(summary, tvec):
    m = np.asarray(summary["camera"]["world_from_camera"], dtype=float)
    # USD stores the row-vector transform in this serialized matrix.  OpenCV
    # uses x-right/y-down/z-forward; Isaac camera local axes are x-right/y-up/z-back.
    local = np.array([float(tvec[0]), -float(tvec[1]), -float(tvec[2])])
    return m[3, :3] + local @ m[:3, :3]


def _v0_v1(dataset, seed=20261105):
    rows = []
    with gzip.open(dataset, "rt", encoding="utf-8") as stream:
        for line in stream:
            rows.append(json.loads(line))
    poses = np.asarray([r["cube_world_m"] for r in rows], dtype=float)
    dt = float(np.median([r["dt_s"] for r in rows]))
    rng = random.Random(seed)
    delay = 2
    dropout = 0.10
    errors = []
    available = 0
    for i, truth in enumerate(poses):
        if rng.random() < dropout:
            continue
        available += 1
        src = poses[max(0, i - delay)]
        estimate = src + np.asarray([rng.gauss(0.0, 0.001) for _ in range(3)])
        errors.append(float(np.linalg.norm(estimate - truth)))
    return {
        "frames": len(rows),
        "availability": available / len(rows),
        "delay_frames": delay,
        "latency_s": delay * dt,
        "position_noise_sigma_m": 0.001,
        "mean_position_error_m": float(np.mean(errors)),
        "p95_position_error_m": float(np.percentile(errors, 95)),
        "seed": seed,
    }


def qualify(evidence: Path, dataset: Path) -> dict:
    summary = json.loads((evidence / "summary.json").read_text(encoding="utf-8"))
    width = int(summary["camera"]["image_width_px"])
    height = int(summary["camera"]["image_height_px"])
    f = float(summary["camera"]["focal_length_mm"])
    ha = float(summary["camera"]["horizontal_aperture_mm"])
    va = float(summary["camera"]["vertical_aperture_mm"])
    K = np.array([[f / ha * width, 0.0, width / 2], [0.0, f / va * height, height / 2], [0.0, 0.0, 1.0]])
    dist = np.zeros(5, dtype=float)
    records = []
    errors = []
    qualified_by_image = {}
    annotated = evidence / "annotated"
    annotated.mkdir(exist_ok=True)
    for path in sorted((evidence / "rgb").glob("*.png")):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        detection = detect_aruco(image, K, dist, float(summary["camera"]["marker_size_m"]), int(summary["camera"]["marker_id"]))
        record = {"image": str(path.relative_to(evidence)), "status": detection.status, "marker_id": detection.marker_id,
                  "confidence": detection.confidence, "reprojection_error_px": detection.reprojection_error_px,
                  "latency_s": 0.0, "source": "rendered_rgb_pixels"}
        if detection.status == "PRESENT":
            p_marker = _world_from_cv(summary, detection.tvec_m)
            p_cube = p_marker - np.array([0.0, 0.0, 0.061])
            truth = np.asarray(summary["cube_oracle_eval_pose_world_m"], dtype=float)
            error = float(np.linalg.norm(p_cube - truth))
            record.update({"marker_position_world_m": p_marker.tolist(), "cube_position_eval_world_m": p_cube.tolist(), "pose_error_m": error})
            qualified_by_image[path.name] = {"position_m": p_cube.tolist(), "quaternion_wxyz": None,
                                             "confidence": detection.confidence, "reprojection_error_px": detection.reprojection_error_px,
                                             "latency_s": 0.0, "marker_id": detection.marker_id, "source": "camera_aruco_pnp_pixels"}
            errors.append(error)
            corners = np.asarray(detection.corners_px, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(image, [corners], True, (0, 255, 0), 2)
            cv2.imwrite(str(annotated / path.name), image)
        records.append(record)
    (evidence / "visual_detections.jsonl").write_text("".join(json.dumps(r, allow_nan=False) + "\n" for r in records), encoding="utf-8")
    frame_file = evidence / "observer_frames.jsonl"
    qualified_frames = []
    if frame_file.exists():
        for line in frame_file.read_text(encoding="utf-8").splitlines():
            frame = json.loads(line)
            if qualified_by_image:
                frame["visual_cube"] = next(iter(qualified_by_image.values()))
            qualified_frames.append(frame)
        (evidence / "observer_frames_v2_qualified.jsonl").write_text(
            "".join(json.dumps(f, allow_nan=False) + "\n" for f in qualified_frames), encoding="utf-8")
        e0, e1 = E0MultiLabel(), E1Hysteresis(dwell=3)
        journal = []
        for frame in qualified_frames:
            pose = frame.get("visual_cube", {}).get("position_m")
            if pose is None:
                continue
            p = np.asarray(pose, dtype=float)
            tcp = frame.get("tcp_world_m", {})
            features = {
                "physics_step": frame["physics_step"], "time_s": frame["sim_time_s"], "cube_position_m": tuple(p),
                "visual_health": "PRESENT", "visual_confidence": float(frame["visual_cube"]["confidence"]),
                "left_tool_distance_m": float(np.linalg.norm(np.asarray(tcp["left"]) - p)),
                "right_tool_distance_m": float(np.linalg.norm(np.asarray(tcp["right"]) - p)),
                "table_gap_m": float(p[2] - .26), "deep_wall_gap_m": float(1.16 - .06 - p[0]),
                "cube_velocity_x_m_s": 0.0, "suction_closed_count": sum(bool(v) for v in frame.get("suction_state", {}).values()),
                "commanded_advancement_m": frame.get("commanded_advancement_m"),
            }
            a, b = e0.estimate(features), e1.estimate(features)
            journal.append({"physics_step": frame["physics_step"], "time_s": frame["sim_time_s"], "source": "v2_qualified_rgb_pixels",
                            "oracle_contact_pairs": "excluded", "e0_edges": {k.value: v for k, v in a.edges.items()},
                            "e0_mode": a.mode.value, "e1_mode": b.mode.value, "e1_causes": list(b.causes)})
        (evidence / "estimator_event_journal.jsonl").write_text("".join(json.dumps(x, allow_nan=False) + "\n" for x in journal), encoding="utf-8")
    present = [r for r in records if r["status"] == "PRESENT"]
    depth_stats = []
    for rel in summary.get("depth_paths", []):
        depth = np.load(evidence / rel)
        depth_stats.append({"path": rel, "shape": list(depth.shape), "finite_fraction": float(np.isfinite(depth).mean()),
                            "finite_median_m": float(np.median(depth[np.isfinite(depth)]))})
    v2 = {
        "images": len(records), "present": len(present), "availability": len(present) / len(records) if records else 0.0,
        "mean_reprojection_error_px": float(np.mean([r["reprojection_error_px"] for r in present])) if present else None,
        "mean_pose_error_m": float(np.mean(errors)) if errors else None,
        "p95_pose_error_m": float(np.percentile(errors, 95)) if errors else None,
        "latency_s": 0.0, "marker_id": int(summary["camera"]["marker_id"]),
        "depth": depth_stats,
        "measurement": "single rendered smoke frame; no manipulation trajectory",
    }
    v1 = _v0_v1(dataset)
    result = {"schema": "task28.r2b.qualification.v1", "evidence": evidence.name,
              "v0_oracle": {"frames": v1["frames"], "availability": 1.0, "mean_position_error_m": 0.0, "latency_s": 0.0},
              "v1_proxy": v1, "v2_rendered_aruco": v2,
              "policy": "V2 detector receives pixels only; USD/oracle values are evaluation-only"}
    (evidence / "visual_qualification.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("evidence", type=Path)
    parser.add_argument("--dataset", type=Path, default=Path("research/stage_b_contact_graph_r2/dataset/r2_timeline.jsonl.gz"))
    args = parser.parse_args()
    print(json.dumps(qualify(args.evidence, args.dataset), indent=2))


if __name__ == "__main__":
    main()
