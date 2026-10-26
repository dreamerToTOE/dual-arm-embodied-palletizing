"""Column vectors: T_A_B maps coordinates in B into A; products compose leftward.

Translations use metres, angles radians, input quaternions are xyzw. RPY means
Rz(yaw) @ Ry(pitch) @ Rx(roll). Invalid poses are rejected, never repaired.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def real_array(value):
    raw = np.asarray(value)
    if not np.issubdtype(raw.dtype, np.number) or np.issubdtype(raw.dtype, np.complexfloating) or np.issubdtype(raw.dtype, np.bool_):
        raise ValueError("EXPECTED_REAL_NUMERIC_ARRAY")
    return raw.astype(float)


def transform(rotation=None, translation=(0., 0., 0.)):
    r = np.eye(3) if rotation is None else real_array(rotation)
    p = real_array(translation)
    if r.shape != (3,3) or p.shape != (3,):
        raise ValueError("INVALID_ROTATION_OR_TRANSLATION_SHAPE")
    result = np.eye(4)
    result[:3, :3] = r
    result[:3, 3] = p
    return checked_transform(result)


def checked_transform(value):
    value = real_array(value)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError("INVALID_SE3_SHAPE_OR_NONFINITE")
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-12, rtol=0):
        raise ValueError("INVALID_SE3_BOTTOM_ROW")
    r = value[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-9, rtol=0) or abs(np.linalg.det(r) - 1) > 1e-9:
        raise ValueError("INVALID_SE3_ROTATION")
    return value.copy()


def pose_matrix(position_m, quaternion_xyzw):
    p, q = real_array(position_m), real_array(quaternion_xyzw)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError("INVALID_POSE_SHAPE_OR_NONFINITE")
    if abs(np.linalg.norm(q) - 1.) > 1e-6:
        raise ValueError("INVALID_QUATERNION_NORM")
    return transform(Rotation.from_quat(q).as_matrix(), p)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def axis_rotation(axis, angle):
    axis = real_array(axis)
    if axis.shape != (3,) or not np.isfinite(axis).all() or not math.isfinite(angle) or abs(np.linalg.norm(axis)-1) > 1e-9:
        raise ValueError("INVALID_AXIS_OR_ANGLE")
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*(skew @ skew)


def inverse(value):
    value = checked_transform(value)
    return transform(value[:3, :3].T, -value[:3, :3].T @ value[:3, 3])


def residual(actual, expected):
    """Return invariant relative-pose translation norm and SO(3) geodesic angle."""
    relative = inverse(expected) @ checked_transform(actual)
    return float(np.linalg.norm(relative[:3, 3])), float(np.linalg.norm(Rotation.from_matrix(relative[:3, :3]).as_rotvec()))
