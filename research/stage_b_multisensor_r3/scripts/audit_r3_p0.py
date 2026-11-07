#!/usr/bin/env python3
"""Build the reproducible R3-P0 sensor/causal lineage manifest."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from statistics import median
import numpy as np
import cv2


def jsonl(path: Path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def count_files(path: Path, suffix: str | None = None) -> int:
    return sum(1 for item in path.glob("*") if item.is_file() and (suffix is None or item.suffix == suffix)) if path is not None and path.exists() else 0


def audit_run(name: str, compact: Path, raw: Path | None) -> dict:
    detections_path = compact / "visual_detections.jsonl"
    timeline_path = compact / "r2b_camera/camera_timeline.jsonl"
    detections = list(jsonl(detections_path)) if detections_path.exists() else []
    timeline = list(jsonl(timeline_path)) if timeline_path.exists() else []
    raw_trace = None if raw is None else raw / "physical_trace/physical_trace.jsonl"
    raw_contacts = None if raw is None else raw / "physical_trace/contact_events.jsonl"
    trace_rows = list(jsonl(raw_trace)) if raw_trace and raw_trace.exists() else []
    trace_count = len(trace_rows) if trace_rows else None
    trace_steps = {r["physics_step"]: r for r in trace_rows}
    intervals = np.diff([r["time_s"] for r in timeline]).tolist()
    contact_count = sum(1 for _ in jsonl(raw_contacts)) if raw_contacts and raw_contacts.exists() else None
    rgb_dir = None if raw is None else raw / "r2b_camera/rgb"
    depth_dir = None if raw is None else raw / "r2b_camera/depth"
    latencies = [float(row["writer_complete_wall_s"]) - float(row["physics_state_receipt_wall_s"]) for row in timeline if row.get("writer_complete_wall_s") is not None and row.get("physics_state_receipt_wall_s") is not None]
    samples = []
    for index in sorted({0, len(timeline)//2, len(timeline)-1}) if timeline and raw is not None else []:
        t = timeline[index]
        sample = {"physics_step": t["physics_step"], "depth_ok_receipt": t.get("depth_ok"), "frame_pair_ok_receipt": t.get("frame_pair_ok")}
        for kind in ("rgb", "depth"):
            path = raw / "r2b_camera" / t[kind] if t.get(kind) else None
            item = {"path": str(path) if path is not None else None, "exists": path is not None and path.exists()}
            if item["exists"]:
                array = cv2.imread(str(path), cv2.IMREAD_UNCHANGED) if kind == "rgb" else np.load(path, allow_pickle=False)
                item.update({"sha256": sha256(path), "shape": list(array.shape), "dtype": str(array.dtype)})
                if kind == "depth": item["finite_fraction"] = float(np.isfinite(array).mean())
            sample[kind] = item
        samples.append(sample)
    return {
        "run": name,
        "sample_file_checks": samples,
        "compact_root": str(compact),
        "raw_root": None if raw is None else str(raw),
        "visual_detection_rows": len(detections),
        "visual_present_rows": sum(row.get("status") == "PRESENT" for row in detections),
        "camera_timeline_rows": len(timeline),
        "raw_runtime_rows": trace_count,
        "runtime_fields_present_rows": {key: sum(r.get(key) is not None for r in trace_rows) for key in ("q_actual_rad", "dq_actual_rad_s", "tcp_world_m", "suction_command", "suction_state", "controller_command", "cube_world_m", "cube_velocity_m_s")},
        "runtime_trace_error_rows": sum("trace_error" in r for r in trace_rows),
        "camera_receipt_exact_runtime_step_matches": sum(r["physics_step"] in trace_steps for r in timeline),
        "camera_receipt_exact_runtime_time_matches": sum(r["physics_step"] in trace_steps and abs(trace_steps[r["physics_step"]]["time_s"]-r["time_s"])<1e-6 for r in timeline),
        "camera_receipt_interval_s_median_max": [median(intervals), max(intervals)] if intervals else None,
        "runtime_dt_s_median": median(r["dt_s"] for r in trace_rows if r.get("dt_s") is not None) if trace_rows else None,
        "raw_contact_rows": contact_count,
        "raw_rgb_files": count_files(rgb_dir),
        "raw_depth_files": count_files(depth_dir),
        "runtime_sha256": None if not raw_trace or not raw_trace.exists() else sha256(raw_trace),
        "contact_sha256": None if not raw_contacts or not raw_contacts.exists() else sha256(raw_contacts),
        "visual_sha256": None if not detections_path.exists() else sha256(detections_path),
        "timeline_sha256": None if not timeline_path.exists() else sha256(timeline_path),
        "time_range_s": None if not timeline else [float(timeline[0]["time_s"]), float(timeline[-1]["time_s"])],
        "physics_step_range": None if not timeline else [int(timeline[0]["physics_step"]), int(timeline[-1]["physics_step"])],
        "writer_latency_wall_s_median": None if not latencies else median(latencies),
    }


def build(args) -> dict:
    runs = []
    for name, compact, raw in args.run:
        runs.append(audit_run(name, Path(compact), None if raw == "NONE" else Path(raw)))
    manifest = {
        "schema": "task28.r3.p0_sensor_lineage.v1",
        "baseline_commit": "8af7ee236f7b06afe893ca1efc7b0126b798f4c7",
        "generated_by": "research/stage_b_multisensor_r3/scripts/audit_r3_p0.py",
        "runs": runs,
        "field_lineage": [
            {"field": "RGB image", "availability": "ONLINE_OBSERVABLE", "producer": "isaac/scripts/task28_task27_one_pass.py::R2BPhysicalCamera.capture_if_due -> capture_viewport_to_file", "consumer": "research/stage_b_vision_contact_r2b/visual/aruco_pose.py and qualify_physical_camera.py", "time": "camera_timeline.physics_state_time_s/physics_step plus writer wall stamps", "oracle": False},
            {"field": "Depth image", "availability": "ONLINE_OPTIONAL_SIM_SENSOR", "producer": "isaac/scripts/task28_task27_one_pass.py::R2BPhysicalCamera depth capture", "consumer": "physical-camera qualification only; no synchronized depth fusion in R3-P1", "time": "camera_timeline depth_acquisition_wall_s; exposure-level equality unproven", "oracle": False},
            {"field": "ArUco corners", "availability": "ONLINE_OBSERVABLE", "producer": "research/stage_b_vision_contact_r2b/visual/aruco_pose.py::detect_aruco", "consumer": "qualify_physical_camera.py and R3-P1 V0/V1", "time": "visual_detections.time_s/physics_step", "oracle": False},
            {"field": "K / distortion", "availability": "ONLINE_CONFIGURED", "producer": "camera_metadata authored aperture/focal values; calibration_result.json is evaluation artifact", "consumer": "visual/aruco_pose.py and R3-P1", "time": "static configuration; no per-frame timestamp", "oracle": False},
            {"field": "T_world_camera", "availability": "ONLINE_CONFIGURED", "producer": "camera_metadata.world_from_camera", "consumer": "qualify_physical_camera.py::world_pose and R3-P1", "time": "static configuration; camera reapplication recorded after initialize", "oracle": False},
            {"field": "q_actual / dq_actual", "availability": "ONLINE_OBSERVABLE_IN_RUNTIME_TRACE", "producer": "isaac/scripts/task28_task27_one_pass.py::PhysicalTrace._post_step", "consumer": "runtime_adapter.py and controller telemetry; not Cube-GT", "time": "physical_trace.physics_step/time_s", "oracle": False},
            {"field": "FK TCP", "availability": "ONLINE_OBSERVABLE_IN_RUNTIME_TRACE", "producer": "PhysicalTrace Xformable TCP readback", "consumer": "Stage-B observer/E0/E1", "time": "physical_trace.physics_step/time_s", "oracle": False},
            {"field": "suction command/state", "availability": "ONLINE_OBSERVABLE_WITH_SEMANTIC_LIMIT", "producer": "feed bridge _last_commanded and Surface_Gripper.is_closed() captured by PhysicalTrace", "consumer": "observer; closed is not independent attachment proof", "time": "runtime row time_s; initial booleans may be default state", "oracle": False},
            {"field": "controller command/reference", "availability": "ONLINE_PARTIAL", "producer": "PhysicalTrace latest command_receipts[-1]", "consumer": "runtime_adapter/controller_reference and phase receipts", "time": "command contains wall time but no authoritative exposure/physics-step timestamp", "oracle": False},
            {"field": "Cube GT pose/velocity", "availability": "EVALUATION_ORACLE", "producer": "PhysicalTrace Cube USD/PhysX readback", "consumer": "offline scoring in R3; legacy Task26/27 controller also consumes /cube_poses GT", "time": "physical_trace.physics_step/time_s", "oracle": True},
            {"field": "PhysX contact pairs/points/impulse/separation", "availability": "EVALUATION_ORACLE", "producer": "task28_task27_one_pass.py contact callback", "consumer": "offline label sidecar only; never estimator_payload", "time": "contact_events.physics_step/time_s", "oracle": True},
            {"field": "measured effort/force for physical07", "availability": "UNAVAILABLE_IN_PHYSICAL07_TRACE", "producer": "none in physical07 PhysicalTrace; separate task28 torque bridge exposes optional simulated channels", "consumer": "not used by R3-P1", "time": None, "oracle": None},
            {"field": "independent attachment confidence", "availability": "UNAVAILABLE", "producer": "none in physical07", "consumer": "must remain UNKNOWN", "time": None, "oracle": None},
        ],
        "causal_policy": {"join": "latest runtime row with physics_step <= visual receipt step and time_s <= visual receipt time_s; receipt alignment only, not exposure alignment", "future_samples": "rejected", "oracle_leakage": "Cube GT and PhysX contacts excluded from causal estimator inputs", "dynamic_limit": "camera writer wall latency is recorded; exposure-level RGB/depth/command synchronization is not established"},
        "audit_findings": [
            {"id": "GT_VELOCITY_LEAK", "source": "research/stage_b_vision_contact_r2b/observer/runtime_adapter.py::runtime_receipt_from_trace", "finding": "cube_velocity_m_s is copied to observed_cube_velocity_x_m_s and enters controller_reference; old causal payload excludes contact pairs but not this GT velocity. Prior R2 E0/E1 causal claim is qualified. R3 does not import that adapter or field."},
            {"id": "GT_PHASE_LABELS", "source": "research/stage_b_vision_contact_r2b/scripts/qualify_physical_camera.py::phase", "finding": "visual_detections.phase is derived from Cube GT and suction. R3 discards it and only uses executor event intervals when available."},
            {"id": "K_ASPECT_MISMATCH", "source": "Isaac Sim 4.5 Camera.get_vertical_aperture/get_intrinsics_matrix", "finding": "SDK effective vertical aperture derives from horizontal aperture times height/width. Fixed 36x24 authored metadata produces legacy fy=600, SDK square-pixel K gives fy=711.111. Bounded readback confirms."},
            {"id": "MARKER_OFFSET", "source": "r2b_camera_observer_smoke.py::make_marker", "finding": "Legacy rendered cells have front plane +61.5mm, whereas old evaluator used +61mm. R3 freezes 61.5mm for both V0 and V1."},
            {"id": "TCP_NOT_INDEPENDENT_FK", "source": "PhysicalTrace._tcp", "finding": "tcp_world_m reads the simulated TCP prim transform, not a fresh independent q-to-URDF FK calculation. Treat as simulated robot-link pose; FK fusion validation belongs to P2."},
            {"id": "CALLBACK_TIMING", "source": "PhysicalTrace._contact_report", "finding": "Contact reports use current self.step/time_s; subscription ordering relative to post-step is not certified, possible one-step assignment ambiguity. Impulse and penetration proxies are not measured loaded force."},
            {"id": "LEGACY_CONTROLLER_GT", "source": "Task26BatchedFeedBridge._publish -> /task27/cube_poses -> C++ CubeBuffer", "finding": "Existing Task27 controller consumes simulator Cube GT for planning/push; R3 A/B is a separate non-actuating path, not a GT-free Task27 controller."}
        ],
        "raw_data_policy": "large RGB/depth/JSONL files remain in local sensor-pipeline worktrees and are not copied into Git",
    }
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", nargs=3, action="append", metavar=("NAME", "COMPACT_ROOT", "RAW_ROOT"), required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    manifest = build(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
