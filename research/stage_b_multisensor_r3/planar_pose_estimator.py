"""R3-P1 pixel-only V0 (6DoF) and table-constrained V1 (x,y,yaw)."""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

import cv2
import numpy as np
from scipy.optimize import least_squares


@dataclass(frozen=True)
class PoseEstimate:
    status: str
    cube_xyz_m: tuple[float, float, float] | None = None
    yaw_rad: float | None = None
    reprojection_error_px: float | None = None
    support_residual_m: float | None = None
    tilt_deg: float | None = None
    runtime_ms: float = 0.0
    uncertainty_xy_m: float | None = None
    reason: str | None = None

    def to_dict(self):
        return asdict(self)


def marker_corners(marker_size_m=0.06):
    h = float(marker_size_m) / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def camera_axes_from_usd(world_from_camera):
    m = np.asarray(world_from_camera, dtype=float)
    return np.column_stack((m[0, :3], -m[1, :3], -m[2, :3]))


def rotation_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def project(world_points, K, world_from_camera):
    camera = (np.asarray(world_points) - world_from_camera[3, :3]) @ camera_axes_from_usd(world_from_camera)
    if np.any(camera[:, 2] <= 0):
        raise ValueError('marker behind camera')
    pixels = camera @ K.T
    return pixels[:, :2] / pixels[:, 2, None]


def cube_marker_world_corners(x, y, yaw, *, table_height_m=0.20, cube_size_m=0.12, marker_offset_m=0.0615, marker_size_m=0.06):
    origin = np.array([x, y, table_height_m + cube_size_m / 2 + marker_offset_m])
    return origin + marker_corners(marker_size_m) @ rotation_z(yaw).T


def _validate(corners_px, K, world_from_camera):
    corners, K, M = (np.asarray(v, dtype=float) for v in (corners_px, K, world_from_camera))
    if corners.shape != (4, 2) or K.shape != (3, 3) or M.shape != (4, 4):
        raise ValueError('invalid corner/K/camera shape')
    if not all(np.isfinite(v).all() for v in (corners, K, M)):
        raise ValueError('nonfinite observation/configuration')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or not np.allclose(K[2], [0, 0, 1]):
        raise ValueError('invalid intrinsics')
    if abs(cv2.contourArea(corners.astype(np.float32))) < 4:
        raise ValueError('degenerate or too small corner polygon')
    A = camera_axes_from_usd(M)
    if not np.allclose(A.T @ A, np.eye(3), atol=1e-5) or np.linalg.det(A) < 0.99:
        raise ValueError('camera extrinsics must be rigid, with USD row-vector convention')
    return corners, K, M


def angle_error_rad(a, b):
    return float((a - b + math.pi) % (2 * math.pi) - math.pi)


def yaw_from_quaternion_wxyz(quaternion):
    q = np.asarray(quaternion, dtype=float)
    w, x, y, z = q / np.linalg.norm(q)
    return float(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def estimate_v0(corners_px, K, world_from_camera, *, marker_size_m=0.06, marker_offset_m=0.0615):
    start = time.perf_counter()
    corners, K, M = _validate(corners_px, K, world_from_camera)
    ok, rv, tv = cv2.solvePnP(marker_corners(marker_size_m), corners, K, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tv[2, 0] <= 0:
        return PoseEstimate('UNKNOWN', runtime_ms=(time.perf_counter()-start)*1000, reason='pnp_failed')
    Rcv, _ = cv2.Rodrigues(rv)
    A = camera_axes_from_usd(M)
    R = A @ Rcv
    origin = M[3, :3] + A @ tv.reshape(3)
    cube = origin - R[:, 2] * marker_offset_m
    pixels, _ = cv2.projectPoints(marker_corners(marker_size_m), rv, tv, K, np.zeros(5))
    error = float(np.linalg.norm(pixels.reshape(4, 2)-corners, axis=1).mean())
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
    return PoseEstimate('PRESENT', tuple(cube), math.atan2(R[1, 0], R[0, 0]), error, None, tilt, (time.perf_counter()-start)*1000)


def estimate_v1(corners_px, K, world_from_camera, *, table_height_m=0.20, cube_size_m=0.12, marker_offset_m=0.0615, marker_size_m=0.06, support_context=None, support_tolerance_m=0.06, tilt_limit_deg=10.0, residual_limit_px=2.0):
    """Fit x/y/yaw; reject known carry, apparent lift/tilt and model mismatch.

    support_context=False is an executor/observable contradiction, never GT.
    A passing residual is consistency evidence, not proof of physical support.
    """
    start = time.perf_counter()
    corners, K, M = _validate(corners_px, K, world_from_camera)
    if support_context is False:
        return PoseEstimate('UNCERTAIN', runtime_ms=(time.perf_counter()-start)*1000, reason='executor_support_context_invalid')
    v0 = estimate_v0(corners, K, M, marker_size_m=marker_size_m, marker_offset_m=marker_offset_m)
    height_residual = None if v0.cube_xyz_m is None else v0.cube_xyz_m[2] - (table_height_m+cube_size_m/2)
    if v0.status != 'PRESENT' or abs(height_residual) > support_tolerance_m or v0.tilt_deg > tilt_limit_deg:
        return PoseEstimate('UNCERTAIN', support_residual_m=height_residual, tilt_deg=v0.tilt_deg, runtime_ms=(time.perf_counter()-start)*1000, reason='unconstrained_pose_inconsistent_with_support')

    def residual(params):
        world = cube_marker_world_corners(*params, table_height_m=table_height_m, cube_size_m=cube_size_m, marker_offset_m=marker_offset_m, marker_size_m=marker_size_m)
        return (project(world, K, M) - corners).reshape(-1)

    fit = least_squares(residual, [*v0.cube_xyz_m[:2], v0.yaw_rad], loss='huber', f_scale=1.0, max_nfev=100)
    error = float(np.linalg.norm(residual(fit.x).reshape(4, 2), axis=1).mean())
    present = fit.success and np.isfinite(fit.x).all() and error <= residual_limit_px
    # Local pixel-noise covariance excludes K/T/model bias; never a certified error bound.
    covariance = np.linalg.pinv(fit.jac.T @ fit.jac) * max(1.0, float(np.mean(fit.fun**2)))
    uncertainty = math.sqrt(max(0.0, float(np.linalg.eigvalsh(covariance[:2, :2]).max())))
    return PoseEstimate('PRESENT' if present else 'UNCERTAIN', (float(fit.x[0]), float(fit.x[1]), table_height_m+cube_size_m/2) if present else None, angle_error_rad(float(fit.x[2]), 0.0), error, height_residual, v0.tilt_deg, (time.perf_counter()-start)*1000, uncertainty, None if present else 'planar_reprojection_residual')
