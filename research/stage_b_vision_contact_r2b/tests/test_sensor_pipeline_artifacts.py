import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence"


def test_calibration_manifest_and_holdout_are_independent():
    manifest = json.loads((EVIDENCE / "calibration_static/manifest.json").read_text())
    for row in manifest["items"]:
        data = (EVIDENCE / "calibration_static" / row["path"]).read_bytes()
        assert hashlib.sha256(data).hexdigest() == row["sha256"]
    fit = json.loads((EVIDENCE / "r2b_rear_wall_physical05/multiframe_calibration.json").read_text())
    assert set(fit["fit_policy"]["train_ids"]).isdisjoint(fit["fit_policy"]["heldout_ids"])
    assert fit["fit_policy"]["runtime_use"] == "NEVER; evaluation only"
    assert fit["static_holdout"]["frames"] == len(fit["static_holdout"]["by_frame"])
    assert fit["physical05_holdout"]["detections"] == fit["physical05_holdout"]["frames"]
    assert fit["physical05_holdout"]["mean_pose_error_m"] > 0.03


def test_physical05_timing_distinguishes_blind_gap_from_dropout():
    data = json.loads((EVIDENCE / "r2b_rear_wall_physical05/visual_qualification.json").read_text())
    timing = data["observation_timing"]
    assert data["frames"] == data["detections"] == 55
    assert data["longest_visual_dropout_s"] == 0.0
    assert timing["max_intercapture_gap_s"] > 90.0
    assert timing["unsampled_intervals"]


def test_physical07_camera_pairs_are_complete_and_timestamped():
    root = EVIDENCE / "r2b_rear_wall_physical07"
    summary = json.loads((root / "r2b_camera/camera_acquisition_summary.json").read_text())
    rows = [json.loads(line) for line in (root / "r2b_camera/camera_timeline.jsonl").read_text().splitlines() if line]
    assert summary["scheduler"] == "physics_step_threshold"
    assert summary["failed_frames"] == 0
    assert summary["successful_frames"] == len(rows) > 1000
    assert summary["actual_physics_receipt_rate_hz"] > 5.0
    assert summary["max_physics_receipt_gap_s"] <= 0.25
    assert all(row["frame_pair_ok"] and row["depth_ok"] for row in rows)
    assert all(row["time_s"] == row["physics_state_time_s"] for row in rows)
    assert all(row["scheduled_physics_step"] <= row["physics_step"] for row in rows)
