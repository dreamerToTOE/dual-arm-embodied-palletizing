"""Pixel-only ArUco pose estimation for the R2-B marker."""
from __future__ import annotations
from dataclasses import dataclass
import cv2
import numpy as np

@dataclass(frozen=True)
class ArucoDetection:
    status: str
    marker_id: int | None
    rvec: tuple[float, float, float] | None
    tvec_m: tuple[float, float, float] | None
    confidence: float
    reprojection_error_px: float | None
    corners_px: tuple[tuple[float, float], ...] | None


def detect_aruco(image_bgr, camera_matrix, dist_coeffs, marker_size_m, marker_id=None):
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY) if image_bgr.ndim == 3 else image_bgr
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None or len(ids) == 0:
        return ArucoDetection("UNKNOWN", None, None, None, 0.0, None, None)
    chosen = 0
    if marker_id is not None:
        matches = np.where(ids.reshape(-1) == int(marker_id))[0]
        if len(matches) == 0:
            return ArucoDetection("UNKNOWN", None, None, None, 0.0, None, None)
        chosen = int(matches[0])
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        [corners[chosen]], float(marker_size_m), np.asarray(camera_matrix, dtype=np.float64), np.asarray(dist_coeffs, dtype=np.float64))
    pts = corners[chosen].reshape(-1, 2).astype(np.float64)
    projected, _ = cv2.projectPoints(np.zeros((4, 3), dtype=np.float64), rvecs[0], tvecs[0], camera_matrix, dist_coeffs)
    # The marker-corner reprojection check uses the known square corners.
    half = float(marker_size_m) / 2.0
    object_corners = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    projected, _ = cv2.projectPoints(object_corners, rvecs[0], tvecs[0], camera_matrix, dist_coeffs)
    error = float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - pts, axis=1)))
    conf = float(max(0.0, min(1.0, (1.0 - error / 8.0) * min(1.0, cv2.contourArea(pts.astype(np.float32)) / 400.0))))
    return ArucoDetection("PRESENT", int(ids[chosen, 0]), tuple(float(v) for v in rvecs[0, 0]), tuple(float(v) for v in tvecs[0, 0]), conf, error, tuple(tuple(float(v) for v in p) for p in pts))
