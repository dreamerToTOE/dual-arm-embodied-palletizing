#!/usr/bin/env python3
"""Bounded multi-pose calibration at the fixed rear_11 camera.

The visual target is a four-marker ArUco board shown at twelve independently
authored rigid poses. Isaac renders the RGB images and all calibration/scoring
happens afterwards from image points and authored target geometry. No Cube pose
is used as an online measurement or per-frame correction.
"""
from __future__ import annotations

import argparse
import cv2
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
SCENE = REPO / "isaac/scripts/task27_five_cube_center_insert_scene.py"
OUT_ROOT = REPO / "research/stage_b_vision_contact_r2b/evidence"
MARKER_IDS = (10, 11, 12, 13)
MARKER_SIZE_M = 0.060
# The visible code cells are raised from the target backing by 8 mm.
MARKER_FACE_X_M = 0.008
IMAGE_SIZE = (1280, 720)
CAMERA_EYE = np.array([1.70, -0.35, 0.80], dtype=float)
CAMERA_TARGET = np.array([0.82, 0.0, 0.30], dtype=float)
_GRID_ARCHIVE = np.load(REPO / "research/stage_b_vision_contact_r2b/config/precision_marker_grids.npz")
MARKER_GRIDS = {int(key.split("_")[1]): np.asarray(_GRID_ARCHIVE[key], dtype=bool)
                for key in _GRID_ARCHIVE.files}
MARKER_OFFSETS = {10: (0.0, -0.10, 0.10), 11: (0.0, 0.10, 0.10),
                  12: (0.0, -0.10, -0.10), 13: (0.0, 0.10, -0.10)}


def marker_corners(marker_id: int) -> np.ndarray:
    """Return marker-local corners in OpenCV order, with local +z normal."""
    h = MARKER_SIZE_M / 2.0
    if marker_id not in MARKER_OFFSETS:
        raise ValueError(f"unsupported marker id: {marker_id}")
    center = np.asarray(MARKER_OFFSETS[marker_id], dtype=float)
    center[0] = MARKER_FACE_X_M
    # The fixed rear_11 camera is on +X, so the target's front plane is X=0.
    return center + np.array([
        [0.0, -h, h], [0.0, h, h], [0.0, h, -h], [0.0, -h, -h]
    ])


def pose_sets() -> list[dict]:
    """Deterministic train/evaluation poses with strong multi-view diversity."""
    train = [
        ([1.22, -0.16, 0.60], [0.0, 0.0, 0.0]),
        ([1.25, -0.17, 0.62], [0.0, 3.0, 0.0]),
        ([1.28, -0.18, 0.64], [2.0, -3.0, 1.0]),
        ([1.31, -0.19, 0.66], [-2.0, 3.0, -2.0]),
        ([1.24, -0.15, 0.58], [3.0, -2.0, 3.0]),
        ([1.27, -0.16, 0.68], [-2.0, 4.0, -3.0]),
        ([1.30, -0.20, 0.62], [2.0, -4.0, 2.0]),
        ([1.34, -0.21, 0.70], [-3.0, 2.0, -2.0]),
    ]
    holdout = [
        ([1.23, -0.17, 0.66], [3.0, 3.0, 2.0]),
        ([1.29, -0.18, 0.60], [-3.0, -3.0, -3.0]),
        ([1.33, -0.20, 0.68], [4.0, 2.0, -3.0]),
        ([1.35, -0.21, 0.64], [-3.0, -2.0, 3.0]),
    ]
    return [
        {"split": "train", "translation_m": t, "rotation_xyz_deg": r}
        for t, r in train
    ] + [
        {"split": "holdout", "translation_m": t, "rotation_xyz_deg": r}
        for t, r in holdout
    ]


def _load_scene():
    namespace = {"__name__": "__r2b_camera_precision_scene__", "_SIDE_SUCTION_SCENARIO": "task27"}
    source = SCENE.read_text(encoding="utf-8")
    exec(compile(source, str(SCENE), "exec"), namespace, namespace)
    return namespace


def _make_marker(stage, target_path, marker_id):
    from pxr import Gf, UsdGeom

    group = UsdGeom.Xform.Define(stage, f"{target_path}/marker_{marker_id}")
    center = tuple(marker_corners(marker_id).mean(axis=0))
    scale = (0.0015, MARKER_SIZE_M / 6.0, MARKER_SIZE_M / 6.0)
    offsets = lambda row, col: (0.0, (col - 2.5) * scale[1],
                                (2.5 - row) * scale[2])
    group.AddTranslateOp(opSuffix="marker_center").Set(Gf.Vec3d(*center))
    # ArUco's black border needs a contrasting white quiet zone. Without it,
    # the rendered border merges into the black viewport and has no contour.
    backing = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/white_backing")
    backing.CreateSizeAttr(1.0)
    backing.CreateDisplayColorAttr([Gf.Vec3f(0.98, 0.98, 0.98)])
    backing_xf = UsdGeom.Xformable(backing.GetPrim())
    # Put the white field on the camera-facing side of the target; the code
    # below raises each marker cell slightly farther toward +X.
    backing_xf.AddTranslateOp().Set(Gf.Vec3d(0.0040, 0.0, 0.0))
    backing_xf.AddScaleOp().Set(Gf.Vec3f(0.0015, MARKER_SIZE_M * 0.70,
                                          MARKER_SIZE_M * 0.70))
    # An explicit camera-facing quad makes the white quiet zone independent of
    # cube face shading and renderer back-face behavior.
    quiet = UsdGeom.Mesh.Define(stage, f"{group.GetPath()}/white_quiet_mesh")
    x, half = 0.0040, MARKER_SIZE_M * 0.70
    quiet.CreatePointsAttr([Gf.Vec3f(x, -half, -half), Gf.Vec3f(x, half, -half),
                            Gf.Vec3f(x, half, half), Gf.Vec3f(x, -half, half)])
    quiet.CreateFaceVertexCountsAttr([4])
    quiet.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    quiet.CreateDisplayColorAttr([Gf.Vec3f(0.98, 0.98, 0.98)])
    # Keep the quiet zone explicit on the camera-facing side. This avoids
    # relying on renderer lighting for a very thin backing cube.
    def quiet_strip(name, translation, strip_scale):
        strip = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/{name}")
        strip.CreateSizeAttr(1.0)
        strip.CreateDisplayColorAttr([Gf.Vec3f(0.98, 0.98, 0.98)])
        xf = UsdGeom.Xformable(strip.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(*translation))
        xf.AddScaleOp().Set(Gf.Vec3f(*strip_scale))
    quiet_strip("quiet_top", (0.0040, 0.0, 0.055), (0.0007, 0.065, 0.010))
    quiet_strip("quiet_bottom", (0.0040, 0.0, -0.055), (0.0007, 0.065, 0.010))
    quiet_strip("quiet_left", (0.0040, -0.055, 0.0), (0.0007, 0.010, 0.065))
    quiet_strip("quiet_right", (0.0040, 0.055, 0.0), (0.0007, 0.010, 0.065))
    grid = MARKER_GRIDS[marker_id]
    for row in range(6):
        for col in range(6):
            cell = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/cell_{row}_{col}")
            cell.CreateSizeAttr(1.0)
            color = (0.02, 0.02, 0.02) if not grid[row, col] else (0.98, 0.98, 0.98)
            cell.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            xf = UsdGeom.Xformable(cell.GetPrim())
            x, y, z = offsets(row, col)
            xf.AddTranslateOp().Set(Gf.Vec3d(x + 0.0080, y, z))
            xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def _set_pose(ops, translation, rotation_xyz_deg):
    from pxr import Gf

    ops[0].Set(Gf.Vec3d(*translation))
    ops[1].Set(Gf.Vec3f(*rotation_xyz_deg))


def _camera_matrix(camera):
    focal = float(camera.GetFocalLengthAttr().Get())
    horizontal = float(camera.GetHorizontalApertureAttr().Get())
    vertical = float(camera.GetVerticalApertureAttr().Get())
    width, height = IMAGE_SIZE
    return np.array([[focal / horizontal * width, 0.0, width / 2.0],
                     [0.0, focal / vertical * height, height / 2.0],
                     [0.0, 0.0, 1.0]], dtype=float)


def _capture_views(run_id: str) -> Path:
    out = OUT_ROOT / run_id
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    image_dir = out / "views"
    image_dir.mkdir()
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaacsim import SimulationApp

    print("R2B_PRECISION_APP_STARTED", flush=True)
    app = SimulationApp({"headless": False, "create_new_stage": True,
                         "renderer": "RaytracedLighting", "multi_gpu": False,
                         "anti_aliasing": 2, "sync_loads": True})
    try:
        import omni.kit.commands
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        from pxr import Usd, UsdGeom

        print("R2B_PRECISION_BEFORE_SCENE", flush=True)
        _load_scene()
        print("R2B_PRECISION_AFTER_SCENE", flush=True)
        stage = omni.usd.get_context().get_stage()
        target_path = "/World/R2B_CalibrationTarget"
        target = UsdGeom.Xform.Define(stage, target_path)
        print("R2B_PRECISION_TARGET", flush=True)
        translation_op = target.AddTranslateOp(opSuffix="calibration_translation")
        rotation_op = target.AddRotateXYZOp(opSuffix="calibration_rotation")
        for marker_id in MARKER_IDS:
            _make_marker(stage, target_path, marker_id)

        camera_path = "/World/R2B_RearWallCamera"
        camera = UsdGeom.Camera.Define(stage, camera_path)
        camera.GetFocalLengthAttr().Set(20.0)
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(24.0)
        camera.GetClippingRangeAttr().Set((0.01, 100.0))
        set_camera_view(eye=CAMERA_EYE.tolist(), target=CAMERA_TARGET.tolist(),
                        camera_prim_path=camera_path)
        viewport = get_active_viewport()
        viewport.set_active_camera(camera_path)
        try:
            viewport.render_mode = "RaytracedLighting"
        except Exception:
            pass
        try:
            omni.kit.commands.execute("SetLightingMenuModeCommand",
                                      lighting_mode="Default", usd_context_name="")
        except Exception:
            pass
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()

        matrix = np.array(UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        K0 = _camera_matrix(camera)
        manifest = {
            "schema": "task28.r2b.camera_precision_calibration.v1",
            "camera": {"prim": camera_path, "eye_world_m": CAMERA_EYE.tolist(),
                       "target_world_m": CAMERA_TARGET.tolist(),
                       "resolution_px": list(IMAGE_SIZE),
                       "focal_length_mm": 20.0, "horizontal_aperture_mm": 36.0,
                       "vertical_aperture_mm": 24.0,
                       "world_from_camera": matrix.tolist(), "K_authored": K0.tolist()},
            "marker_ids": list(MARKER_IDS), "marker_size_m": MARKER_SIZE_M,
            "target_geometry": {str(i): marker_corners(i).tolist() for i in MARKER_IDS},
            "views": [],
        }
        for index, view in enumerate(pose_sets()):
            _set_pose((translation_op, rotation_op), view["translation_m"], view["rotation_xyz_deg"])
            for _ in range(30):
                app.update()
            world_from_target = np.array(
                UsdGeom.Xformable(target.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
                dtype=float,
            )
            world_corners = {
                str(marker_id): (np.asarray(marker_corners(marker_id)) @ world_from_target[:3, :3]
                                 + world_from_target[3, :3]).tolist()
                for marker_id in MARKER_IDS
            }
            image_path = image_dir / f"view_{index:02d}.png"
            capture_viewport_to_file(viewport, file_path=str(image_path), is_hdr=False)
            for _ in range(40):
                app.update()
                if image_path.is_file() and image_path.stat().st_size > 0:
                    break
            manifest["views"].append({**view, "index": index,
                                      "image": str(image_path.relative_to(out)),
                                      "world_from_target": world_from_target.tolist(),
                                      "world_corners": world_corners,
                                      "file_present": image_path.is_file()})
        (out / "calibration_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return out
    except BaseException as exc:
        (out / "failure.json").write_text(json.dumps({"status": "FAILED", "error": repr(exc)}, indent=2) + "\n")
        raise
    finally:
        try:
            app.close()
        except Exception:
            pass


def _detect_all(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None:
        return []
    return [np.asarray(item, dtype=float).reshape(4, 2) for item in corners]


def _associate(view, detected, K, world_from_camera):
    """Assign the single repeated-id detection to the known target location."""
    if not detected:
        return {}
    M = np.asarray(world_from_camera, dtype=float)
    centers = np.asarray([np.mean(np.asarray(view["world_corners"][str(mid)]), axis=0)
                          for mid in MARKER_IDS], dtype=float)
    rel = centers - M[3, :3]
    cv = np.asarray([[np.dot(x, M[0, :3]), -np.dot(x, M[1, :3]), -np.dot(x, M[2, :3])] for x in rel])
    expected = np.column_stack((K[0, 0] * cv[:, 0] / cv[:, 2] + K[0, 2],
                                K[1, 1] * cv[:, 1] / cv[:, 2] + K[1, 2]))
    found = np.asarray([np.mean(x, axis=0) for x in detected])
    result = {}
    remaining = set(range(len(detected)))
    for marker_index in np.argsort(expected[:, 1]):
        if not remaining:
            break
        candidate = min(remaining, key=lambda i: float(np.linalg.norm(found[i] - expected[marker_index])))
        if float(np.linalg.norm(found[candidate] - expected[marker_index])) <= 90.0:
            result[MARKER_IDS[marker_index]] = detected[candidate]
            remaining.remove(candidate)
    return result


def _rotation_error_deg(a, b):
    r = np.asarray(a, dtype=float) @ np.asarray(b, dtype=float).T
    angle = math.acos(float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
    return float(np.degrees(angle))


def _camera_pose_error(rvec, tvec, known_world_from_camera):
    R_cv, _ = cv2.Rodrigues(np.asarray(rvec, dtype=float))
    A_est = R_cv.T
    C_est = -A_est @ np.asarray(tvec, dtype=float).reshape(3)
    known = np.asarray(known_world_from_camera, dtype=float)
    A_known = np.column_stack((known[0, :3], -known[1, :3], -known[2, :3]))
    C_known = known[3, :3]
    return {"camera_center_error_xyz_m": (C_est - C_known).tolist(),
            "camera_center_error_m": float(np.linalg.norm(C_est - C_known)),
            "camera_rotation_error_deg": _rotation_error_deg(A_est, A_known)}


def _score_view(view, observations, K, dist, known_world_from_camera):
    object_points = []
    image_points = []
    by_marker = []
    for marker_id in MARKER_IDS:
        if marker_id not in observations:
            continue
        obj = np.asarray(view["world_corners"][str(marker_id)], dtype=np.float32)
        img = np.asarray(observations[marker_id], dtype=np.float32)
        object_points.append(obj)
        image_points.append(img)
        by_marker.append({"marker_id": marker_id, "corners": img.tolist()})
    if len(object_points) == 0:
        return {"status": "NO_MARKERS", "marker_count": 0}
    obj = np.concatenate(object_points).astype(np.float32)
    img = np.concatenate(image_points).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return {"status": "PNP_FAILED", "marker_count": len(object_points)}
    projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    error = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
    score = {"status": "PRESENT", "marker_count": len(object_points),
             "corner_count": int(len(obj)),
             "reprojection_error_px_mean": float(np.mean(error)),
             "reprojection_error_px_p95": float(np.percentile(error, 95)),
             "reprojection_error_px_max": float(np.max(error)),
             "markers": by_marker}
    score.update(_camera_pose_error(rvec, tvec, known_world_from_camera))
    return score


def _calibrate(out: Path):
    manifest = json.loads((out / "calibration_manifest.json").read_text())
    K0 = np.asarray(manifest["camera"]["K_authored"], dtype=float)
    observations = {}
    for view in manifest["views"]:
        image = cv2.imread(str(out / view["image"]))
        observations[view["index"]] = _associate(view, _detect_all(image), K0,
                                                   manifest["camera"]["world_from_camera"]) if image is not None else {}
    (out / "observations.json").write_text(json.dumps(
        {str(k): {str(mid): corners.tolist() for mid, corners in v.items()}
         for k, v in observations.items()}, indent=2) + "\n")
    object_points = []
    image_points = []
    for view in manifest["views"]:
        if view["split"] != "train":
            continue
        ids = observations[view["index"]]
        obj = [np.asarray(view["world_corners"][str(mid)], dtype=np.float32)
               for mid in MARKER_IDS if mid in ids]
        img = [np.asarray(ids[mid], dtype=np.float32) for mid in MARKER_IDS if mid in ids]
        if obj:
            object_points.append(np.concatenate(obj))
            image_points.append(np.concatenate(img))
    dist0 = np.zeros(5, dtype=float)
    if len(object_points) < 3:
        raise RuntimeError("insufficient multi-marker training views")
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT |
             cv2.CALIB_ZERO_TANGENT_DIST |
             cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
             cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5 | cv2.CALIB_FIX_K6)
    rms, Kfit, Dfit, _, _ = cv2.calibrateCamera(object_points, image_points,
                                                  IMAGE_SIZE, K0.copy(), dist0.copy(), flags=flags)
    scores = []
    baseline_scores = []
    for view in manifest["views"]:
        score = _score_view(view, observations[view["index"]], Kfit, Dfit,
                            manifest["camera"]["world_from_camera"])
        baseline = _score_view(view, observations[view["index"]], K0, dist0,
                               manifest["camera"]["world_from_camera"])
        scores.append({"index": view["index"], "split": view["split"], **score})
        baseline_scores.append({"index": view["index"], "split": view["split"], **baseline})

    def aggregate(rows):
        valid = [r for r in rows if r["status"] == "PRESENT"]
        if not valid:
            return {"views": 0}
        trans = [r["camera_center_error_m"] for r in valid]
        rot = [r["camera_rotation_error_deg"] for r in valid]
        reproj = [r["reprojection_error_px_mean"] for r in valid]
        axis = np.asarray([r["camera_center_error_xyz_m"] for r in valid], dtype=float)
        return {"views": len(valid), "camera_center_error_median": float(np.median(trans)),
                "camera_center_error_p95": float(np.percentile(trans, 95)),
                "camera_rotation_error_deg_median": float(np.median(rot)),
                "camera_rotation_error_deg_p95": float(np.percentile(rot, 95)),
                "reprojection_error_px_median": float(np.median(reproj)),
                "reprojection_error_px_p95": float(np.percentile(reproj, 95)),
                "camera_center_error_axis_mean_m": np.mean(axis, axis=0).tolist(),
                "camera_center_error_axis_median_abs_m": np.median(np.abs(axis), axis=0).tolist()}

    result = {"schema": "task28.r2b.camera_precision_calibration_result.v1",
              "run_id": out.name, "calibration_model": "pinhole_with_5_distortion_coefficients",
              "train_views": [v["index"] for v in manifest["views"] if v["split"] == "train"],
              "holdout_views": [v["index"] for v in manifest["views"] if v["split"] == "holdout"],
              "marker_ids": list(MARKER_IDS), "marker_size_m": MARKER_SIZE_M,
              "calibration_rms_px": float(rms), "K_authored": K0.tolist(),
              "distortion_authored": dist0.tolist(), "K_calibrated": Kfit.tolist(),
              "distortion_calibrated": Dfit.reshape(-1).tolist(),
              "baseline": {"train": aggregate([x for x in baseline_scores if x["split"] == "train"]),
                           "holdout": aggregate([x for x in baseline_scores if x["split"] == "holdout"])},
              "calibrated": {"train": aggregate([x for x in scores if x["split"] == "train"]),
                             "holdout": aggregate([x for x in scores if x["split"] == "holdout"])},
              "per_view_baseline": baseline_scores, "per_view_calibrated": scores,
              "policy": "rendered multi-marker target; no Cube ground truth or per-frame correction"}
    (out / "calibration_result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-id", default="r2b_camera_precision_cal01")
    ap.add_argument("--analyze-only", type=Path)
    args = ap.parse_args()
    try:
        out = args.analyze_only or _capture_views(args.run_id)
        _calibrate(out)
    except BaseException as exc:
        out = OUT_ROOT / args.run_id
        out.mkdir(parents=True, exist_ok=True)
        (out / "failure.json").write_text(json.dumps({"status": "FAILED", "error": repr(exc)}, indent=2) + "\n")
        raise


if __name__ == "__main__":
    main()
