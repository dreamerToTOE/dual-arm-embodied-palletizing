import cv2
import numpy as np

from research.stage_b_vision_contact_r2b.scripts.diagnose_single_marker_pose import _camera_axes, _project


def test_usd_camera_to_opencv_axes_are_right_handed():
    world_from_camera = np.eye(4)
    axes = _camera_axes(world_from_camera)
    assert np.allclose(axes, np.diag([1.0, -1.0, -1.0]))
    assert np.isclose(np.linalg.det(axes), 1.0)


def test_projection_and_pnp_round_trip_with_known_rigid_transform():
    K = np.asarray([[700.0, 0.0, 640.0], [0.0, 700.0, 360.0], [0.0, 0.0, 1.0]])
    axes = np.eye(3)
    center = np.asarray([0.2, -0.1, -1.2])
    world = np.asarray([[-.03, .03, 0.0], [.03, .03, 0.0], [.03, -.03, 0.0], [-.03, -.03, 0.0]]) + np.asarray([0.2, -0.1, 0.0])
    pixels, _ = _project(world, K, axes, center)
    ok, rvec, tvec = cv2.solvePnP((world - np.asarray([0.2, -0.1, 0.0])).astype(np.float32), pixels.astype(np.float32), K, np.zeros(5), flags=cv2.SOLVEPNP_ITERATIVE)
    assert ok
    reprojection, _ = cv2.projectPoints((world - np.asarray([0.2, -0.1, 0.0])).astype(np.float32), rvec, tvec, K, np.zeros(5))
    assert float(np.max(np.linalg.norm(reprojection.reshape(4, 2) - pixels, axis=1))) < 1e-3
