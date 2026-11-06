#!/usr/bin/env python3
"""Bounded Cube-marker transfer probe with a positive RGB smoke image.

The cal24 four-marker board remains an external calibration target.  This
probe uses one 60 mm ID23 marker on the Cube top face, where a 72 mm quiet
zone fits inside the 120 mm Cube face and does not interfere with suction.
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
MARKER_ID = 23
MARKER_SIZE_M = 0.060
QUIET_SIZE_M = 0.072
CUBE_SIZE_M = 0.120
CELL_THICKNESS_M = 0.001
MARKER_FRONT_OFFSET_M = CUBE_SIZE_M / 2.0 + CELL_THICKNESS_M
MARKER_GRID = np.asarray(np.load(REPO / "research/stage_b_vision_contact_r2b/config/marker23_grid.npy"), dtype=bool)
CAMERA_EYE = np.array([1.70, -0.35, 0.80], dtype=float)
CAMERA_TARGET = np.array([0.82, 0.0, 0.30], dtype=float)
VIEWS = (
    ("single_arm_recontact", (0.771, 0.060, 0.260), 0.0),
    ("insertion_entry", (0.790, 0.060, 0.260), 0.0),
    ("insertion_mid", (0.900, 0.060, 0.260), 2.0),
    ("deep_wall_near", (1.040, 0.060, 0.260), -2.0),
    ("deep_wall_arrival", (1.090, 0.060, 0.260), 0.0),
    ("side_contact_approach", (0.850, 0.180, 0.260), 4.0),
)


def _load_scene():
    namespace = {"__name__": "__r2b_cal24_transfer_scene__", "_SIDE_SUCTION_SCENARIO": "task27"}
    exec(compile(SCENE.read_text(encoding="utf-8"), str(SCENE), "exec"), namespace, namespace)


def marker_corners_metric() -> np.ndarray:
    h = MARKER_SIZE_M / 2.0
    return np.asarray([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]], dtype=float)


def metric_to_cube_local(points: np.ndarray) -> np.ndarray:
    return np.asarray(points, dtype=float) / CUBE_SIZE_M


def _set_cube_pose(prim, value, rotation_deg: float = 0.0):
    from pxr import Gf, UsdGeom
    xf = UsdGeom.Xformable(prim)
    translate = next((op for op in xf.GetOrderedXformOps() if op.GetOpName().startswith("xformOp:translate:task26_pose")), None)
    (translate or xf.AddTranslateOp(opSuffix="r2b_transfer_pose")).Set(Gf.Vec3d(*value))
    rotate = next((op for op in xf.GetOrderedXformOps() if op.GetOpName().endswith("r2b_transfer_rotation")), None)
    (rotate or xf.AddRotateZOp(opSuffix="r2b_transfer_rotation")).Set(float(rotation_deg))


def _make_cube_marker(stage, cube_path):
    from pxr import Gf, UsdGeom
    group = UsdGeom.Xform.Define(stage, f"{cube_path}/R2B_CubeTopMarker")
    cell = MARKER_SIZE_M / 6.0 / CUBE_SIZE_M
    thickness = CELL_THICKNESS_M / CUBE_SIZE_M
    z = 0.5 + thickness / 2.0
    backing = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/quiet_backing")
    backing.CreateSizeAttr(1.0)
    backing.CreateDisplayColorAttr([Gf.Vec3f(0.98, 0.98, 0.98)])
    backing_xf = UsdGeom.Xformable(backing.GetPrim())
    # Keep the quiet backing behind the raised cells; coincident front faces
    # make the backing occlude the black ArUco cells under RTX rasterization.
    backing_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))
    backing_xf.AddScaleOp().Set(Gf.Vec3f(QUIET_SIZE_M / CUBE_SIZE_M, QUIET_SIZE_M / CUBE_SIZE_M, thickness))
    for row in range(6):
        for col in range(6):
            prim = UsdGeom.Cube.Define(stage, f"{group.GetPath()}/cell_{row}_{col}")
            prim.CreateSizeAttr(1.0)
            color = (0.02, 0.02, 0.02) if not MARKER_GRID[row, col] else (0.98, 0.98, 0.98)
            prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            xf = UsdGeom.Xformable(prim.GetPrim())
            xf.AddTranslateOp().Set(Gf.Vec3d((col - 2.5) * cell, (2.5 - row) * cell, z))
            xf.AddScaleOp().Set(Gf.Vec3f(cell, cell, thickness))


def _cube_rotation(cube_world: np.ndarray) -> np.ndarray:
    rows = np.asarray(cube_world, dtype=float)[:3, :3]
    return (rows / np.linalg.norm(rows, axis=1, keepdims=True)).T


def expected_world_corners(cube_world: np.ndarray) -> np.ndarray:
    rotation = _cube_rotation(cube_world)
    origin = np.asarray(cube_world, dtype=float)[3, :3] + rotation @ np.array([0.0, 0.0, MARKER_FRONT_OFFSET_M])
    return origin + marker_corners_metric() @ rotation.T


def _actual_world_corners(stage, cube_path: str) -> np.ndarray:
    from pxr import Gf, Usd, UsdGeom
    names = ((0, 0, (-.5, .5, .5)), (0, 5, (.5, .5, .5)), (5, 5, (.5, -.5, .5)), (5, 0, (-.5, -.5, .5)))
    points = []
    for row, col, local in names:
        prim = stage.GetPrimAtPath(f"{cube_path}/R2B_CubeTopMarker/cell_{row}_{col}")
        matrix = np.asarray(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        p = np.asarray(local, dtype=float) @ matrix[:3, :3] + matrix[3, :3]
        points.append(p)
    return np.asarray(points)


def _enable_lighting():
    import omni.kit.commands
    result = omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default", usd_context_name="")
    if not result or result[0] is not True:
        raise RuntimeError(f"VIEWPORT_LIGHTING_DEFAULT_REJECTED:{result!r}")
    return repr(result)


def _capture(app, viewport, path):
    from omni.kit.viewport.utility import capture_viewport_to_file
    capture_viewport_to_file(viewport, file_path=str(path), is_hdr=False)
    for _ in range(20):
        app.update()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"VIEWPORT_CAPTURE_MISSING:{path}")


def _image_stats(path: Path) -> dict:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        return {"readable": False}
    gray = cv2.cvtColor(image[:, :, :3], cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    return {"readable": True, "shape": list(image.shape), "dtype": str(image.dtype), "min": int(image.min()), "max": int(image.max()), "mean": float(image.mean()), "std": float(image.std()), "gray_mean": float(gray.mean()), "gray_histogram_16": np.histogram(gray, bins=16, range=(0, 256))[0].astype(int).tolist()}


def _capture_run(run_id: str) -> Path:
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
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Usd, UsdGeom
        _load_scene()
        stage = omni.usd.get_context().get_stage()
        cube_path = "/World/Task27/Supply/Cube_01"
        cube = stage.GetPrimAtPath(cube_path)
        _set_cube_pose(cube, (0.500, 0.0, 0.260), 0.0)
        cam_path = "/World/R2B_RearWallCamera"
        camera_prim = UsdGeom.Camera.Define(stage, cam_path)
        camera_prim.GetFocalLengthAttr().Set(20.0)
        camera_prim.GetHorizontalApertureAttr().Set(36.0)
        camera_prim.GetVerticalApertureAttr().Set(24.0)
        camera_prim.GetClippingRangeAttr().Set((0.01, 100.0))
        set_camera_view(eye=CAMERA_EYE.tolist(), target=CAMERA_TARGET.tolist(), camera_prim_path=cam_path)
        viewport = get_active_viewport()
        viewport.set_active_camera(cam_path)
        viewport.render_mode = "RaytracedLighting"
        lighting = _enable_lighting()
        from isaacsim.sensors.camera import Camera
        sensor = Camera(prim_path=cam_path, resolution=(1280, 720), render_product_path=viewport.get_render_product_path())
        sensor.initialize()
        set_camera_view(eye=CAMERA_EYE.tolist(), target=CAMERA_TARGET.tolist(), camera_prim_path=cam_path)
        viewport.set_active_camera(cam_path)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(120):
            app.update()
        warmup_time_s = float(timeline.get_current_time())
        timeline.stop()
        smoke_path = out / "scene_smoke.png"
        _capture(app, viewport, smoke_path)
        smoke = _image_stats(smoke_path)
        if not smoke.get("readable") or smoke.get("gray_mean", 0.0) < 2.0:
            raise RuntimeError(f"POSITIVE_RGB_SMOKE_FAILED:{smoke}")
        smoke_meta = {"active_camera": cam_path, "render_product_path": viewport.get_render_product_path(), "renderer": "RaytracedLighting", "lighting": lighting, "resolution_px": [1280, 720], "physics_time_s": warmup_time_s, "timeline_state": "stopped_after_warmup", "warmup_app_updates": 120, "world_from_camera": np.asarray(UsdGeom.Xformable(camera_prim.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float).tolist(), "image": smoke, "cube_world_from_local": np.asarray(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float).tolist()}
        (out / "scene_smoke_metadata.json").write_text(json.dumps(smoke_meta, indent=2) + "\n")
        _make_cube_marker(stage, cube_path)
        cube_world = np.asarray(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
        expected = expected_world_corners(cube_world)
        actual = _actual_world_corners(stage, cube_path)
        geometry_error = float(np.max(np.linalg.norm(actual - expected, axis=1)))
        if geometry_error > 1e-5:
            raise RuntimeError(f"MARKER_GEOMETRY_MISMATCH:{geometry_error}")
        manifest = {"schema": "task28.r2b.cube_marker_transfer_probe.v2", "external_cal24_target": {"path": str(K_RESULT), "board": "separate external four-marker board; not authored on Cube", "size_m": 0.260}, "camera": {"prim": cam_path, "world_from_camera": smoke_meta["world_from_camera"], "render_product_path": smoke_meta["render_product_path"], "resolution_px": [1280, 720], "lighting": lighting}, "cube_path": cube_path, "cube_size_m": CUBE_SIZE_M, "marker_id": MARKER_ID, "marker_size_m": MARKER_SIZE_M, "quiet_zone_size_m": QUIET_SIZE_M, "marker_front_offset_m": MARKER_FRONT_OFFSET_M, "expected_world_corners": expected.tolist(), "actual_world_corners": actual.tolist(), "geometry_max_error_m": geometry_error, "views": []}
        for index, (name, pose, yaw) in enumerate(VIEWS):
            _set_cube_pose(cube, pose, yaw)
            for _ in range(8):
                app.update()
            path = out / "views" / f"view_{index:02d}_{name}.png"
            _capture(app, viewport, path)
            cube_world = np.asarray(UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()), dtype=float)
            expected = expected_world_corners(cube_world)
            actual = _actual_world_corners(stage, cube_path)
            manifest["views"].append({"index": index, "name": name, "pose_world_m": list(pose), "yaw_deg": yaw, "physics_time_s": float(timeline.get_current_time()), "image": str(path.relative_to(out)), "image_stats": _image_stats(path), "expected_world_corners": expected.tolist(), "actual_world_corners": actual.tolist(), "geometry_max_error_m": float(np.max(np.linalg.norm(actual - expected, axis=1))), "cube_world_from_local": cube_world.tolist()})
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        return out
    finally:
        try:
            app.close()
        except Exception:
            pass


def _rotation_error_deg(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip((np.trace(a @ b.T) - 1.0) / 2.0, -1.0, 1.0))))


def _analyze(out: Path) -> dict:
    manifest = json.loads((out / "manifest.json").read_text())
    calibration = json.loads(K_RESULT.read_text())
    K = np.asarray(calibration["K_calibrated"], dtype=float)
    dist = np.zeros(5, dtype=float)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    detected = 0
    errors = []
    for view in manifest["views"]:
        image = cv2.imread(str(out / view["image"]))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
        ids_flat = [] if ids is None else [int(value) for value in ids.reshape(-1)]
        matches = [item.reshape(4, 2) for item, marker_id in zip(corners or [], ids_flat) if marker_id == MARKER_ID]
        if not matches:
            continue
        detected += 1
        image_points = np.asarray(matches[0], dtype=np.float32)
        ok, rvec, tvec = cv2.solvePnP(marker_corners_metric().astype(np.float32), image_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        R_cv, _ = cv2.Rodrigues(rvec)
        # solvePnP returns marker/object -> OpenCV-camera rotation.  The
        # transpose is only needed when converting the camera center back to
        # marker coordinates; using it as marker->world rotation flips the
        # pose and produces metre-scale errors.
        camera_center_marker = -R_cv.T @ np.asarray(tvec, dtype=float).reshape(3)
        camera_world = np.asarray(manifest["camera"]["world_from_camera"], dtype=float)
        camera_axes_world = np.column_stack((camera_world[0, :3], -camera_world[1, :3], -camera_world[2, :3]))
        cube_world = np.asarray(view["cube_world_from_local"], dtype=float)
        cube_R = _cube_rotation(cube_world)
        marker_origin_world = cube_world[3, :3] + cube_R @ np.array([0.0, 0.0, MARKER_FRONT_OFFSET_M])
        expected_camera_marker = cube_R.T @ (camera_world[3, :3] - marker_origin_world)
        estimated_marker_R_world = camera_axes_world @ R_cv
        estimated_marker_origin_world = camera_world[3, :3] - estimated_marker_R_world @ camera_center_marker
        estimated_cube_center = estimated_marker_origin_world - estimated_marker_R_world @ np.array([0.0, 0.0, MARKER_FRONT_OFFSET_M])
        projection, _ = cv2.projectPoints(marker_corners_metric().astype(np.float32), rvec, tvec, K, dist)
        reproj = float(np.mean(np.linalg.norm(projection.reshape(-1, 2) - image_points, axis=1)))
        errors.append({"index": view["index"], "name": view["name"], "position_error_m": float(np.linalg.norm(estimated_cube_center - cube_world[3, :3])), "orientation_error_deg": _rotation_error_deg(estimated_marker_R_world, cube_R), "camera_center_marker_error_m": float(np.linalg.norm(camera_center_marker - expected_camera_marker)), "reprojection_error_px": reproj, "distance_m": float(np.linalg.norm(tvec))})
    result = {"schema": "task28.r2b.cube_marker_transfer_probe_result.v2", "run_id": out.name, "marker_id": MARKER_ID, "marker_size_m": MARKER_SIZE_M, "quiet_zone_size_m": QUIET_SIZE_M, "views": len(manifest["views"]), "detected_views": detected, "detection_rate": detected / len(manifest["views"]) if manifest["views"] else 0.0, "geometry_max_error_m": manifest["geometry_max_error_m"], "scene_smoke": json.loads((out / "scene_smoke_metadata.json").read_text()), "pose_errors": errors, "online_status": "STATIC_TRANSFER_ONLY", "manipulation_episode": False, "contact_confirmation": False}
    if errors:
        result.update({"position_error_median": float(np.median([x["position_error_m"] for x in errors])), "position_error_p95": float(np.percentile([x["position_error_m"] for x in errors], 95)), "orientation_error_deg_median": float(np.median([x["orientation_error_deg"] for x in errors])), "reprojection_error_px_median": float(np.median([x["reprojection_error_px"] for x in errors])), "distance_error_median": float(np.median([x["camera_center_marker_error_m"] for x in errors]))})
    (out / "transfer_result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="r2b_cube_marker_transfer_probe01")
    parser.add_argument("--analyze-only", type=Path)
    args = parser.parse_args()
    _analyze(args.analyze_only or _capture_run(args.run_id))


if __name__ == "__main__":
    main()
