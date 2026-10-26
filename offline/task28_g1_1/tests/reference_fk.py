"""Independent FK oracle from the *frozen USD inspection*, without USD runtime.

The implementation under test parses the frozen URDF and rotates its joint Z
axes. This reference deliberately does neither: it composes the two USD joint
frames and their authored X-axis rotation. The local USD coordinates were
captured in G1-0B and remain float32-derived; comparison tolerance is 1e-6 m/rad.

Transforms are active, column-vector T_A_B matrices (coordinates in B to A).
No task28_validator helpers are imported here.
"""

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def _frame(position, quaternion_wxyz):
    quaternion_wxyz = np.asarray(quaternion_wxyz, dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(
        quaternion_wxyz[[1, 2, 3, 0]]
    ).as_matrix()
    matrix[:3, 3] = np.asarray(position, dtype=float)
    return matrix


def _inverse_frame(matrix):
    result = np.eye(4)
    result[:3, :3] = matrix[:3, :3].T
    result[:3, 3] = -matrix[:3, :3].T.dot(matrix[:3, 3])
    return result


def _joint_frame(joint, angle=0.0):
    if joint["axis"] not in ("X", None):
        raise ValueError("Frozen USD oracle requires the authored X axis")
    local0 = _frame(joint["local_pos0_m"], joint["local_rot0_wxyz"])
    local1 = _frame(joint["local_pos1_m"], joint["local_rot1_wxyz"])
    motion = np.eye(4)
    motion[:3, :3] = Rotation.from_rotvec([angle, 0.0, 0.0]).as_matrix()
    return local0.dot(motion).dot(_inverse_frame(local1))


def frozen_data(fixture_dir):
    fixture_dir = Path(fixture_dir)
    audit = json.loads(
        (fixture_dir / "assets/fr3_offline_asset_inspection.json").read_text()
    )
    scene = json.loads(
        (fixture_dir / "scene/task28_g1_0_scene_contract.json").read_text()
    )
    controller = json.loads(
        (fixture_dir / "controller/task28_g1_0_controller_contract.json").read_text()
    )
    return audit, scene, controller


def frozen_bounds(fixture_dir):
    """Read the independently documented USD/MoveIt intersection."""
    audit, _, _ = frozen_data(fixture_dir)
    limits = np.array([
        audit["joint_comparison"][f"joint{i}"][
            "proposed_future_effective_intersection_rad"
        ] for i in range(1, 8)
    ])
    return limits[:, 0], limits[:, 1]


def usd_link_frames(fixture_dir, side, q):
    """Return world link0...link8 frames from USD's joint-frame equation."""
    if side not in ("left", "right"):
        raise ValueError("side must be left or right")
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.isfinite(q).all():
        raise ValueError("Expected seven finite joint angles")
    audit, scene, _ = frozen_data(fixture_dir)
    world = np.eye(4)
    world[:3, 3] = scene["bases_world_m"][side]
    frames = {f"{side}_fr3_link0": world.copy()}
    for index, angle in enumerate(q, start=1):
        world = world.dot(_joint_frame(
            audit["joint_comparison"][f"joint{index}"], angle
        ))
        frames[f"{side}_fr3_link{index}"] = world.copy()
    world = world.dot(_joint_frame(audit["link8_fixed_joint"]))
    frames[f"{side}_fr3_link8"] = world.copy()
    return frames


def usd_fk(fixture_dir, side, q):
    """Return world TCP pose; local TCP follows the frozen tool contract."""
    _, _, controller = frozen_data(fixture_dir)
    contract = controller["tcp_contract"]
    local = np.eye(4)
    local[:3, 3] = contract[f"{side}_link8_translation_m"]
    local[:3, :3] = Rotation.from_euler(
        "xyz", contract[f"{side}_link8_rpy_rad"]
    ).as_matrix()
    return usd_link_frames(fixture_dir, side, q)[f"{side}_fr3_link8"].dot(local)


def analytic_zero_tcp(side):
    """Hand-derived ideal-coordinate q=0 reference, independent of any files.

    At q=0 the link8 orientation is Rx(pi). Its displacement from link0 is
    (0.088, 0, 0.333 + 0.316 + 0.384 - 0.107) = (0.088, 0, 0.926).
    Apply each base and the local TCP offset/orientation algebraically. This
    q=0 FK control intentionally is not a valid FR3 joint-limit state.
    """
    if side == "left":
        return np.array([
            [0.0, 1.0, 0.0, 0.738],
            [1.0, 0.0, 0.0, -0.445],
            [0.0, 0.0, -1.0, 0.846],
            [0.0, 0.0, 0.0, 1.0],
        ])
    if side == "right":
        return np.array([
            [0.0, -1.0, 0.0, 0.738],
            [-1.0, 0.0, 0.0, 0.445],
            [0.0, 0.0, -1.0, 0.846],
            [0.0, 0.0, 0.0, 1.0],
        ])
    raise ValueError("side must be left or right")


def residuals(actual, expected):
    """Euclidean translation and SO(3) geodesic angle in m/rad.

    SciPy's quaternion-based log is stable near identity, unlike a raw arccos
    of a rounded trace. A 180-degree orientation mismatch remains observable.
    """
    actual, expected = np.asarray(actual), np.asarray(expected)
    translation_m = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    rotation_rad = float(np.linalg.norm(Rotation.from_matrix(
        expected[:3, :3].T.dot(actual[:3, :3])
    ).as_rotvec()))
    return translation_m, rotation_rad
