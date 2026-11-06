import math

import numpy as np

from research.stage_b_vision_contact_r2b.scripts.r2b_cal24_transfer_probe import (
    CUBE_SIZE_M,
    MARKER_SIZE_M,
    QUIET_SIZE_M,
    expected_world_corners,
    marker_corners_metric,
    metric_to_cube_local,
)


def _world_matrix(yaw_deg=0.0, translation=(0.0, 0.0, 0.0), scale=CUBE_SIZE_M):
    a = math.radians(yaw_deg)
    c, s = math.cos(a), math.sin(a)
    return np.asarray([[scale * c, scale * s, 0.0, 0.0], [-scale * s, scale * c, 0.0, 0.0], [0.0, 0.0, scale, 0.0], [*translation, 1.0]])


def test_parent_scale_is_applied_once_and_marker_stays_metric():
    local = metric_to_cube_local(marker_corners_metric())
    assert np.allclose(local.ptp(axis=0)[:2], MARKER_SIZE_M / CUBE_SIZE_M)
    world = expected_world_corners(_world_matrix())
    assert np.isclose(np.linalg.norm(world[1] - world[0]), MARKER_SIZE_M)


def test_translation_and_rotation_preserve_corner_order():
    world = expected_world_corners(_world_matrix(90.0, (0.4, -0.2, 0.25)))
    assert np.allclose(world[1] - world[0], [0.0, MARKER_SIZE_M, 0.0], atol=1e-8)
    assert np.allclose(world.mean(axis=0), [0.4, -0.2, 0.25 + 0.061], atol=1e-8)


def test_quiet_zone_fits_cube_face_with_margin():
    assert QUIET_SIZE_M < CUBE_SIZE_M
    assert (CUBE_SIZE_M - QUIET_SIZE_M) / 2.0 == 0.024
