#!/usr/bin/env python3
"""Offline diagnosis for the six saved single-marker pose images.

Ground-truth USD geometry is used only to score hypotheses.  No result from
this script is used to alter the online estimator.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_EVIDENCE = ROOT / "research/stage_b_vision_contact_r2b/evidence/r2b_cube_marker_transfer_probe03"
DEFAULT_K = ROOT / "research/stage_b_vision_contact_r2b/evidence/r2b_camera_precision_cal24/calibration_result.json"
OBJ = np.asarray([[-.03, .03, 0.0], [.03, .03, 0.0], [.03, -.03, 0.0], [-.03, -.03, 0.0]], dtype=np.float32)
OFFSET_M = 0.061


def _cube_rotation(matrix: np.ndarray) -> np.ndarray:
    rows = np.asarray(matrix, dtype=float)[:3, :3]
    return (rows / np.linalg.norm(rows, axis=1, keepdims=True)).T


def _camera_axes(world_from_camera: np.ndarray) -> np.ndarray:
    # USD camera is Y-up/Z-back; OpenCV is X-right/Y-down/Z-forward.
    return np.column_stack((world_from_camera[0, :3], -world_from_camera[1, :3], -world_from_camera[2, :3]))


def _rotation_error(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip((np.trace(a @ b.T) - 1.0) / 2.0, -1.0, 1.0))))


def _detect(image: np.ndarray, dictionary, params) -> np.ndarray | None:
    corners, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), dictionary, parameters=params)
    if ids is None:
        return None
    for corner, marker_id in zip(corners, ids.reshape(-1)):
        if int(marker_id) == 23:
            return corner.reshape(4, 2).astype(np.float64)
    return None


def _project(world_points: np.ndarray, K: np.ndarray, camera_axes: np.ndarray, camera_center: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    camera_points = (world_points - camera_center) @ camera_axes
    pixels = np.column_stack((K[0, 0] * camera_points[:, 0] / camera_points[:, 2] + K[0, 2], K[1, 1] * camera_points[:, 1] / camera_points[:, 2] + K[1, 2]))
    return pixels, camera_points


def _pose_metrics(rvec: np.ndarray, tvec: np.ndarray, view: dict, K: np.ndarray, camera_axes: np.ndarray, camera_center: np.ndarray) -> dict:
    R_cv, _ = cv2.Rodrigues(rvec)
    center_marker = -R_cv.T @ np.asarray(tvec, dtype=float).reshape(3)
    cube_world = np.asarray(view["cube_world_from_local"], dtype=float)
    cube_R = _cube_rotation(cube_world)
    marker_origin = cube_world[3, :3] + cube_R @ np.array([0.0, 0.0, OFFSET_M])
    expected_center_marker = cube_R.T @ (camera_center - marker_origin)
    marker_to_world = camera_axes @ R_cv
    estimated_origin = camera_center - marker_to_world @ center_marker
    estimated_cube_center = estimated_origin - marker_to_world[:, 2] * OFFSET_M
    projection, _ = cv2.projectPoints(OBJ, rvec, tvec, K, np.zeros(5))
    observed = np.asarray(view["detected_corners"], dtype=float)
    reproj_residual = projection.reshape(4, 2) - observed
    return {
        "reprojection_mean_px": float(np.mean(np.linalg.norm(reproj_residual, axis=1))),
        "reprojection_max_px": float(np.max(np.linalg.norm(reproj_residual, axis=1))),
        "translation_error_m": float(np.linalg.norm(estimated_cube_center - cube_world[3, :3])),
        "translation_error_xyz_m": (estimated_cube_center - cube_world[3, :3]).tolist(),
        "rotation_error_deg": _rotation_error(marker_to_world, cube_R),
        "depth_error_m": float(center_marker[2] - expected_center_marker[2]),
        "camera_center_marker_error_m": float(np.linalg.norm(center_marker - expected_center_marker)),
        "rvec": np.asarray(rvec, dtype=float).reshape(3).tolist(),
        "tvec": np.asarray(tvec, dtype=float).reshape(3).tolist(),
    }


def _solve_hypotheses(points: np.ndarray, K: np.ndarray) -> dict[str, list[tuple[np.ndarray, np.ndarray]]]:
    dist = np.zeros(5)
    hypotheses: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {}
    for name, flag in (("ITERATIVE", cv2.SOLVEPNP_ITERATIVE), ("IPPE_SQUARE", cv2.SOLVEPNP_IPPE_SQUARE), ("SQPNP", cv2.SOLVEPNP_SQPNP)):
        try:
            ok, rvec, tvec = cv2.solvePnP(OBJ, points.astype(np.float32), K, dist, flags=flag)
            hypotheses[name] = [(rvec, tvec)] if ok else []
        except cv2.error:
            hypotheses[name] = []
    try:
        result = cv2.solvePnPGeneric(OBJ, points.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        hypotheses["IPPE_GENERIC"] = list(zip(result[1], result[2])) if result[0] else []
    except cv2.error:
        hypotheses["IPPE_GENERIC"] = []
    try:
        result = cv2.solvePnPGeneric(OBJ, points.astype(np.float32), K, dist, flags=cv2.SOLVEPNP_SQPNP)
        hypotheses["SQPNP_GENERIC"] = list(zip(result[1], result[2])) if result[0] else []
    except cv2.error:
        hypotheses["SQPNP_GENERIC"] = []
    return hypotheses


def _incidence(view: dict, camera_center: np.ndarray) -> float:
    cube_world = np.asarray(view["cube_world_from_local"], dtype=float)
    R = _cube_rotation(cube_world)
    origin = cube_world[3, :3] + R @ np.array([0.0, 0.0, OFFSET_M])
    view_direction = camera_center - origin
    return math.degrees(math.acos(float(np.clip(abs(np.dot(R[:, 2], view_direction / np.linalg.norm(view_direction))), -1.0, 1.0))))


def diagnose(evidence: Path, calibration_path: Path, perturbations: int = 32) -> dict:
    manifest = json.loads((evidence / "manifest.json").read_text(encoding="utf-8"))
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    K = np.asarray(calibration["K_calibrated"], dtype=float)
    camera_world = np.asarray(manifest["camera"]["world_from_camera"], dtype=float)
    camera_axes, camera_center = _camera_axes(camera_world), camera_world[3, :3]
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    views = []
    method_scores: dict[str, list[dict]] = {}
    rng = np.random.default_rng(20261106)
    for raw in manifest["views"]:
        image = cv2.imread(str(evidence / raw["image"]))
        detected = _detect(image, dictionary, params)
        if detected is None:
            views.append({"name": raw["name"], "detected": False})
            continue
        world = np.asarray(raw["expected_world_corners"], dtype=float)
        expected_pixels, camera_points = _project(world, K, camera_axes, camera_center)
        observed_center, expected_center = detected.mean(axis=0), expected_pixels.mean(axis=0)
        view = dict(raw)
        view["detected_corners"] = detected.tolist()
        per_view = {
            "name": raw["name"],
            "detected": True,
            "corner_residual_vectors_px": (detected - expected_pixels).tolist(),
            "corner_residual_mean_px": float(np.mean(np.linalg.norm(detected - expected_pixels, axis=1))),
            "corner_residual_max_px": float(np.max(np.linalg.norm(detected - expected_pixels, axis=1))),
            "marker_width_px": float(np.linalg.norm(detected[1] - detected[0])),
            "marker_height_px": float(np.linalg.norm(detected[0] - detected[3])),
            "camera_to_marker_distance_m": float(np.linalg.norm(camera_points.mean(axis=0))),
            "camera_to_marker_optical_depth_m": float(camera_points[:, 2].mean()),
            "incidence_angle_deg": _incidence(raw, camera_center),
            "projected_center_bias_px": (observed_center - expected_center).tolist(),
            "expected_projected_corners_px": expected_pixels.tolist(),
            "detected_corners_px": detected.tolist(),
            "methods": {},
        }
        hypotheses = _solve_hypotheses(detected, K)
        for method, solutions in hypotheses.items():
            entries = []
            for index, (rvec, tvec) in enumerate(solutions):
                metrics = _pose_metrics(rvec, tvec, view, K, camera_axes, camera_center)
                metrics["hypothesis_index"] = index
                entries.append(metrics)
                method_scores.setdefault(method, []).append(metrics)
            per_view["methods"][method] = entries
        perturbation_errors = []
        for _ in range(perturbations):
            noisy = detected + rng.normal(0.0, 0.25, detected.shape)
            try:
                ok, rvec, tvec = cv2.solvePnP(OBJ, noisy.astype(np.float32), K, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    perturbation_errors.append(_pose_metrics(rvec, tvec, view, K, camera_axes, camera_center)["translation_error_m"])
            except cv2.error:
                pass
        per_view["iterative_0_25px_perturbation_translation_p50_p95_m"] = [float(np.percentile(perturbation_errors, 50)), float(np.percentile(perturbation_errors, 95))] if perturbation_errors else None
        views.append(per_view)
    summary = {}
    for method, rows in method_scores.items():
        summary[method] = {
            "solutions": len(rows),
            "translation_error_median": float(np.median([r["translation_error_m"] for r in rows])),
            "translation_error_p95": float(np.percentile([r["translation_error_m"] for r in rows], 95)),
            "rotation_error_deg_median": float(np.median([r["rotation_error_deg"] for r in rows])),
            "depth_error_median": float(np.median([r["depth_error_m"] for r in rows])),
            "reprojection_mean_px_median": float(np.median([r["reprojection_mean_px"] for r in rows])),
        }
    result = {
        "schema": "task28.r2b.single_marker_pose_diagnosis.v1",
        "evidence": str(evidence),
        "calibration": str(calibration_path),
        "camera_world_position_m": camera_center.tolist(),
        "camera_axes_world_from_opencv": camera_axes.tolist(),
        "intrinsics": K.tolist(),
        "marker_size_m": 0.060,
        "diagnostic_only_ground_truth_scoring": True,
        "depth_evidence": {"status": "UNAVAILABLE_FOR_PROBE03", "reason": "probe03 saved RGB only; physical07 depth belongs to a different legacy episode and was not fused"},
        "method_summary": summary,
        "views": views,
    }
    (evidence / "pose_diagnosis.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_K)
    parser.add_argument("--perturbations", type=int, default=32)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = diagnose(args.evidence, args.calibration, args.perturbations)
    if args.out:
        args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result["method_summary"], indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
