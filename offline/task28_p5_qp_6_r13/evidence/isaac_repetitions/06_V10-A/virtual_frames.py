"""Typed separation of the physical Cube, nominal grasps, and virtual object.

All transforms are active ``T_world_frame`` column-vector transforms.  The
virtual object is a kinematic bookkeeping frame only; it never changes the
fixture or authorizes suction/contact.
"""
from dataclasses import dataclass
import numpy as np
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
from offline.task28_g1_1.task28_validator.se3 import checked_transform, inverse, residual


def _T(value, name):
    try:
        return checked_transform(value).copy()
    except Exception as exc:
        raise ValueError(f"INVALID_{name}") from exc


@dataclass(frozen=True)
class PhysicalCubeFrame:
    T_WCube: np.ndarray
    identity: str = "ATTACHED_PHYSICAL_CUBE_CANDIDATE"

    def __post_init__(self):
        object.__setattr__(self, "T_WCube", _T(self.T_WCube, "PHYSICAL_CUBE"))
        if self.identity != "ATTACHED_PHYSICAL_CUBE_CANDIDATE":
            raise ValueError("PHYSICAL_CUBE_IDENTITY")


@dataclass(frozen=True)
class NominalSuctionGrasps:
    T_Cube_GL: np.ndarray
    T_Cube_GR: np.ndarray
    gap_m: float = 0.001
    identity: str = "PRE_CONTACT_ORIGINAL_CUBE"

    def __post_init__(self):
        object.__setattr__(self, "T_Cube_GL", _T(self.T_Cube_GL, "LEFT_GRASP"))
        object.__setattr__(self, "T_Cube_GR", _T(self.T_Cube_GR, "RIGHT_GRASP"))
        if self.gap_m != 0.001 or self.identity != "PRE_CONTACT_ORIGINAL_CUBE":
            raise ValueError("NOMINAL_GRASP_IDENTITY")


@dataclass(frozen=True)
class VirtualCooperativeFrame:
    T_WV: np.ndarray
    T_VL: np.ndarray
    T_VR: np.ndarray
    identity: str = "FREE_HOME_VIRTUAL_COOPERATION"

    def __post_init__(self):
        for name in ("T_WV", "T_VL", "T_VR"):
            object.__setattr__(self, name, _T(getattr(self, name), name))
        if self.identity != "FREE_HOME_VIRTUAL_COOPERATION":
            raise ValueError("VIRTUAL_OBJECT_IDENTITY")

    def tcp_goals(self, T_WV=None):
        obj = self.T_WV if T_WV is None else _T(T_WV, "VIRTUAL_OBJECT_POSE")
        return obj @ self.T_VL, obj @ self.T_VR


def physical_cube_pose():
    T = np.eye(4)
    T[:3, 3] = [0.65, 0.0, 0.26]
    return PhysicalCubeFrame(T)


def nominal_cube_grasps():
    pair = nominal_grasps(gap_m=0.001)
    return NominalSuctionGrasps(pair.left, pair.right)


def make_virtual_frame(T_WL, T_WR):
    """Use a world-oriented midpoint origin and preserve the measured TCP pair."""
    left, right = _T(T_WL, "LEFT_TCP"), _T(T_WR, "RIGHT_TCP")
    origin = np.eye(4)
    origin[:3, 3] = 0.5 * (left[:3, 3] + right[:3, 3])
    return VirtualCooperativeFrame(origin, inverse(origin) @ left, inverse(origin) @ right)


def virtual_relative_residual(frame, T_WL, T_WR):
    goals = frame.tcp_goals()
    e_l = residual(_T(T_WL, "LEFT_TCP"), goals[0])
    e_r = residual(_T(T_WR, "RIGHT_TCP"), goals[1])
    return {"left_translation_m": e_l[0], "left_rotation_rad": e_l[1],
            "right_translation_m": e_r[0], "right_rotation_rad": e_r[1],
            "relative_transform_error": residual(_T(T_WL, "LEFT_TCP") @ inverse(_T(T_WR, "RIGHT_TCP")), goals[0] @ inverse(goals[1]))}


def reject_virtual_as_physical(frame, physical, grasps):
    """Return a fail-closed promotion decision; virtual and physical are distinct."""
    if frame.identity != "FREE_HOME_VIRTUAL_COOPERATION":
        raise ValueError("VIRTUAL_IDENTITY_REQUIRED")
    left_goal, right_goal = frame.tcp_goals()
    implied_l = left_goal @ inverse(grasps.T_Cube_GL)
    implied_r = right_goal @ inverse(grasps.T_Cube_GR)
    e_l = residual(implied_l, physical.T_WCube)
    e_r = residual(implied_r, physical.T_WCube)
    return {"promotion_allowed": False, "reason": "VIRTUAL_FRAME_NEVER_PROMOTES_TO_PHYSICAL_CUBE",
            "left_physical_mismatch_m": e_l[0], "right_physical_mismatch_m": e_r[0],
            "left_physical_rotation_rad": e_l[1], "right_physical_rotation_rad": e_r[1]}
