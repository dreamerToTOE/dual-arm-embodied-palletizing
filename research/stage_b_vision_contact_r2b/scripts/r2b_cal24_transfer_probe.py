#!/usr/bin/env python3
"""One bounded, static Cube-mounted cal24 board transfer probe.

This does not run Task27 manipulation.  It verifies that the four-marker board
can be rigidly mounted to Cube_01 and recovered with the rear_11 camera and
cal24 K, while keeping Cube/scene truth only in the offline score.
"""
from __future__ import annotations

import argparse
import cv2
import json
import math
import os
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
SCENE = REPO / "isaac/scripts/task27_five_cube_center_insert_scene.py"
OUT_ROOT = REPO / "research/stage_b_vision_contact_r2b/evidence"
K_RESULT = REPO / "research/stage_b_vision_contact_r2b/evidence/r2b_camera_precision_cal24/calibration_result.json"
MARKER_IDS = (10, 11, 12, 13)
MARKER_SIZE_M = 0.060
PARENT_SIZE_M = 0.120
MARKER_OFFSETS = {10: (0.0, -0.10, 0.10), 11: (0.0, 0.10, 0.10), 12: (0.0, -0.10, -0.10), 13: (0.0, 0.10, -0.10)}
MARKER_GRIDS = {int(k.split("_")[1]): np.asarray(v, dtype=bool) for k, v in np.load(REPO / "research/stage_b_vision_contact_r2b/config/precision_marker_grids.npz").items()}
CAMERA_EYE = np.array([1.70, -0.35, 0.80], dtype=float)
CAMERA_TARGET = np.array([0.82, 0.0, 0.30], dtype=float)


def _load_scene():
    namespace = {"__name__": "__r2b_cal24_transfer_scene__", "_SIDE_SUCTION_SCENARIO": "task27"}
    source = SCENE.read_text(encoding="utf-8")
    exec(compile(source, str(SCENE), "exec"), namespace, namespace)


def _set_cube_pose(prim, value):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpName().startswith("xformOp:translate:task26_pose"):
            op.Set(Gf.Vec3d(*value))
            return
    xf.AddTranslateOp(opSuffix="r2b_transfer_pose").Set(Gf.Vec3d(*value))


def marker_corners_local(marker_id: int) -> np.ndarray:
    h = MARKER_SIZE_M / 2.0
    _, y, z = MARKER_OFFSETS[marker_id]
    # Cell front: cube +X face + 8 mm board offset + half cell thickness.
    x = PARENT_SIZE_M / 2.0 + 0.0080 + 0.0015
    return np.asarray([[x, y - h, z + h], [x, y + h, z + h], [x, y + h, z - h], [x, y - h, z - h]], dtype=float)


def _make_board(stage, cube_path):
    from pxr import Gf, UsdGeom
    board = UsdGeom.Xform.Define(stage, f"{cube_path}/R2B_Cal24_RigidPlate")
    # The plate is rigidly attached to the +X cube face; all child coordinates
    # are in the Cube's 120 mm local units and inherit its scale.
    for marker_id in MARKER_IDS:
        group = UsdGeom.Xform.Define(stage, f"{board.GetPath()}/marker_{marker_id}")
        _, oy, oz = MARKER_OFFSETS[marker_id]
        group.AddTranslateOp().Set(Gf.Vec3d(0.0, oy / PARENT_SIZE_M, oz / PARENT_SIZE_M))
        cell = MARKER_SIZE_M / 6.0 / PARENT_SIZE_M
        face_center_x = 0.5 + 0.0080 / PARENT_SIZE_M
        quiet_x = 0.5 + 0.0040 / PARENT_SIZE_M
        backing = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/white_backing")
        backing.CreateSizeAttr(1.0)
        backing.CreateDisplayColorAttr([Gf.Vec3f(0.98, 0.98, 0.98)])
        backing_xf = UsdGeom.Xformable(backing.GetPrim())
        backing_xf.AddTranslateOp().Set(Gf.Vec3d(quiet_x, 0.0, 0.0))
        backing_xf.AddScaleOp().Set(Gf.Vec3f(0.0015 / PARENT_SIZE_M, MARKER_SIZE_M * 0.70 / PARENT_SIZE_M, MARKER_SIZE_M * 0.70 / PARENT_SIZE_M))
        for row in range(6):
            for col in range(6):
                prim = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/cell_{row}_{col}")
                prim.CreateSizeAttr(1.0)
                color = (0.02, 0.02, 0.02) if not MARKER_GRIDS[marker_id][row, col] else (0.98, 0.98, 0.98)
                prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
                xf = UsdGeom.Xformable(prim.GetPrim())
                xf.AddTranslateOp().Set(Gf.Vec3d(face_center_x, (col - 2.5) * cell, (2.5 - row) * cell))
                xf.AddScaleOp().Set(Gf.Vec3f(0.0015 / PARENT_SIZE_M, cell, cell))


def _capture(run_id: str) -> Path:
    out = OUT_ROOT / run_id
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    (out / "views").mkdir()
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": False, "create_new_stage": True, "renderer": "RaytracedLighting", "multi_gpu": False, "anti_aliasing": 2, "sync_loads": True})
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
        from pxr import Usd, UsdGeom
        _load_scene()
        stage = omni.usd.get_context().get_stage()
        cube_path = "/World/Task27/Supply/Cube_01"
        cube = stage.GetPrimAtPath(cube_path)
        _set_cube_pose(cube, (0.500, 0.0, 0.260))
        _make_board(stage, cube_path)
        cam_path = "/World/R2B_RearWallCamera"
        camera = UsdGeom.Camera.Define(stage, cam_path)
        camera.GetFocalLengthAttr().Set(20.0)
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(24.0)
        camera.GetClippingRangeAttr().Set((0.01, 100.0))
        set_camera_view(eye=CAMERA_EYE.tolist(), target=CAMERA_TARGET.tolist(), camera_prim_path=cam_path)
        viewport = get_active_viewport()
        viewport.set_active_camera(cam_path)
        try:
            viewport.render_mode = "RaytracedLighting"
        except Exception:
            pass
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        matrix = np.array(UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        cube_world = np.array(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        corners_world = {str(mid): (marker_corners_local(mid) @ cube_world[:3, :3] + cube_world[3, :3]).tolist() for mid in MARKER_IDS}
        manifest = {"schema": "task28.r2b.cal24_transfer_probe.v1", "camera": {"prim": cam_path, "world_from_camera": matrix.tolist(), "eye_world_m": CAMERA_EYE.tolist(), "target_world_m": CAMERA_TARGET.tolist(), "resolution_px": [1280, 720]}, "cube_path": cube_path, "cube_world_from_local": cube_world.tolist(), "marker_ids": list(MARKER_IDS), "marker_size_m": MARKER_SIZE_M, "marker_plate_to_cube": "rigid +X face plate; marker front x=+0.0695 m in Cube local world units", "world_corners": corners_world, "views": []}
        for index in range(12):
            for _ in range(12):
                app.update()
            path = out / "views" / f"view_{index:02d}.png"
            capture_viewport_to_file(viewport, file_path=str(path), is_hdr=False)
            for _ in range(30):
                app.update()
                if path.is_file() and path.stat().st_size > 0:
                    break
            manifest["views"].append({"index": index, "physics_time_s": float(timeline.get_current_time()), "image": str(path.relative_to(out)), "file_present": path.is_file()})
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return out
    finally:
        try:
            app.close()
        except Exception:
            pass


def _pose_error(rvec, tvec, world_from_camera):
    rotation, _ = cv2.Rodrigues(rvec)
    camera_axes = rotation.T
    center = -camera_axes @ np.asarray(tvec, dtype=float).reshape(3)
    known = np.asarray(world_from_camera, dtype=float)
    known_axes = np.column_stack((known[0, :3], -known[1, :3], -known[2, :3]))
    rot = math.degrees(math.acos(float(np.clip((np.trace(camera_axes @ known_axes.T) - 1.0) / 2.0, -1.0, 1.0))))
    return float(np.linalg.norm(center - known[3, :3])), float(rot)


def _analyze(out: Path) -> dict:
    manifest = json.loads((out / "manifest.json").read_text())
    calibration = json.loads(K_RESULT.read_text())
    K = np.asarray(calibration["K_calibrated"], dtype=float)
    dist = np.zeros(5, dtype=float)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    counts = {str(mid): 0 for mid in MARKER_IDS}
    errors = []
    lumas = []
    for view in manifest["views"]:
        image = cv2.imread(str(out / view["image"]))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        lumas.append(float(gray.mean()))
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        if ids is None:
            continue
        objects, images = [], []
        for item, marker_id in zip(corners, ids.reshape(-1)):
            key = str(int(marker_id))
            if key not in manifest["world_corners"]:
                continue
            counts[key] += 1
            objects.extend(manifest["world_corners"][key])
            images.extend(item.reshape(4, 2).tolist())
        if len(objects) < 8:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.asarray(objects, dtype=np.float32), np.asarray(images, dtype=np.float32), K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if ok:
            err_m, err_deg = _pose_error(rvec, tvec, manifest["camera"]["world_from_camera"])
            projection, _ = cv2.projectPoints(np.asarray(objects, dtype=np.float32), rvec, tvec, K, dist)
            reproj = float(np.mean(np.linalg.norm(projection.reshape(-1, 2) - np.asarray(images), axis=1)))
            errors.append({"index": view["index"], "marker_count": len(objects) // 4, "camera_center_error_m": err_m, "camera_rotation_error_deg": err_deg, "reprojection_error_px": reproj})
    result = {"schema": "task28.r2b.cal24_transfer_probe_result.v1", "run_id": out.name, "board": "four-marker rigid plate mounted to Cube_01 +X face", "marker_detection_counts": counts, "views": len(manifest["views"]), "readable_views": len(lumas), "mean_luma_median": None if not lumas else float(np.median(lumas)), "mean_luma_max": None if not lumas else float(max(lumas)), "usable_pose_views": len(errors), "pose_errors": errors, "cal24_k_source": str(K_RESULT), "online_status": "STATIC_TRANSFER_ONLY", "manipulation_episode": False, "contact_confirmation": False}
    if errors:
        result["camera_center_error_median"] = float(np.median([x["camera_center_error_m"] for x in errors]))
        result["camera_center_error_p95"] = float(np.percentile([x["camera_center_error_m"] for x in errors], 95))
        result["camera_rotation_error_deg_median"] = float(np.median([x["camera_rotation_error_deg"] for x in errors]))
        result["reprojection_error_px_median"] = float(np.median([x["reprojection_error_px"] for x in errors]))
    (out / "transfer_result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="r2b_cal24_transfer_probe01")
    parser.add_argument("--analyze-only", type=Path)
    args = parser.parse_args()
    out = args.analyze_only or _capture(args.run_id)
    _analyze(out)


if __name__ == "__main__":
    main()
