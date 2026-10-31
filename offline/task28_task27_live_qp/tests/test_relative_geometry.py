import numpy as np
from scipy.spatial.transform import Rotation

from offline.task28_p5_qp_6_r12.qp_math import pair_tcp_geometric_relative_matrix, se3_log


def _pose(rotation, translation):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    T[:3, 3] = translation
    return T


def test_geometric_relative_map_matches_se3_finite_difference():
    left = _pose([0.1, -0.2, 0.05], [1.0, -0.4, 0.6])
    right = _pose([-0.15, 0.1, 0.2], [0.75, 0.5, 0.62])
    stacked = np.array([0.03, -0.02, 0.01, 0.04, -0.03, 0.02,
                        -0.01, 0.02, 0.04, -0.02, 0.01, 0.03])
    dt = 1e-7
    next_left = left.copy()
    next_right = right.copy()
    for T, v, w in ((next_left, stacked[:3], stacked[3:6]),
                    (next_right, stacked[6:9], stacked[9:12])):
        T[:3, 3] += v * dt
        T[:3, :3] = Rotation.from_rotvec(w * dt).as_matrix() @ T[:3, :3]
    rel = np.linalg.inv(left) @ right
    rel_next = np.linalg.inv(next_left) @ next_right
    finite_difference = se3_log(np.linalg.inv(rel) @ rel_next) / dt
    predicted = pair_tcp_geometric_relative_matrix(left, right) @ stacked
    np.testing.assert_allclose(predicted, finite_difference, atol=2e-6, rtol=2e-6)
