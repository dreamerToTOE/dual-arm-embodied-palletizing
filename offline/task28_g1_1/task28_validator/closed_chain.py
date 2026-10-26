"""Full SE(3) bilateral grasp closure; no planning or state projection."""
from dataclasses import dataclass
import math
import numpy as np
from .se3 import checked_transform, inverse, residual, transform


@dataclass(frozen=True)
class ClosurePolicy:
    translation_tolerance_m: float = 1e-4
    rotation_tolerance_rad: float = 1e-3

    def __post_init__(self):
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or x <= 0 or x > maximum
               for x,maximum in ((self.translation_tolerance_m,1e-4),(self.rotation_tolerance_rad,1e-3))):
            raise ValueError("INVALID_CLOSURE_POLICY")


@dataclass(frozen=True)
class GraspPair:
    """T_O_GL and T_O_GR: object-coordinate grasp frames (TCP identities)."""
    left: np.ndarray
    right: np.ndarray

    def __post_init__(self):
        for name in ("left", "right"):
            value = checked_transform(getattr(self,name))
            value.setflags(write=False)
            object.__setattr__(self,name,value)


@dataclass(frozen=True)
class ClosureResult:
    valid: bool
    reasons: tuple
    translation_error_m: float
    rotation_error_rad: float
    left_object_translation_error_m: float
    left_object_rotation_error_rad: float
    right_object_translation_error_m: float
    right_object_rotation_error_rad: float
    implied_left_object: np.ndarray
    implied_right_object: np.ndarray


def nominal_grasps(gap_m=0.001):
    """NEW G1-1 derived grasp contract, without editing the G1-0B fixture.

    60 mm half Cube + the existing 1 mm nominal contact-command gap, within the
    unchanged 3 mm capture threshold. gap=0 is reserved for exact contact controls.
    A fixed physical attachment is represented by a constant full object->TCP
    transform. This is a kinematic abstraction, not evidence that suction holds.
    """
    if gap_m not in (0., .001):
        raise ValueError("UNFROZEN_GRASP_GAP")
    # Exactly defined rotations, independent of approximate sqrt(1/2) serialization.
    left_rotation = np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
    right_rotation = np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    return GraspPair(transform(left_rotation,(0.,-.06-gap_m,0.)),transform(right_rotation,(0.,.06+gap_m,0.)))


def desired_tcps(world_object, grasps=None):
    grasps = nominal_grasps() if grasps is None else grasps
    world_object = checked_transform(world_object)
    return world_object @ grasps.left, world_object @ grasps.right


def evaluate_closure(world_object, world_left_tcp, world_right_tcp, grasps=None, policy=ClosurePolicy()):
    if type(policy) is not ClosurePolicy:
        raise ValueError("INVALID_CLOSURE_POLICY_TYPE")
    policy.__post_init__()
    grasps = nominal_grasps() if grasps is None else grasps
    obj, left, right = map(checked_transform,(world_object,world_left_tcp,world_right_tcp))
    implied_left, implied_right = left @ inverse(grasps.left), right @ inverse(grasps.right)
    closure_pos, closure_rot = residual(implied_left,implied_right)
    left_pos,left_rot = residual(implied_left,obj)
    right_pos,right_rot = residual(implied_right,obj)
    reasons = []
    for prefix,pos,rot in (("LEFT_RIGHT_OBJECT",closure_pos,closure_rot),("LEFT_GRASP",left_pos,left_rot),("RIGHT_GRASP",right_pos,right_rot)):
        if pos > policy.translation_tolerance_m:
            reasons.append(prefix+"_TRANSLATION_MISMATCH")
        if rot > policy.rotation_tolerance_rad:
            reasons.append(prefix+"_ROTATION_MISMATCH")
    return ClosureResult(not reasons,tuple(reasons),closure_pos,closure_rot,left_pos,left_rot,right_pos,right_rot,implied_left,implied_right)
