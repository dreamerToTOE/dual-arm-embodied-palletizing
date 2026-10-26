"""Single-state pipeline: input validity -> limits -> FK/SE(3) -> collision.

A return value is a verdict on this one state only. It does not check a trajectory,
propagate a grasp, solve forces, project a manifold, or issue robot commands.
"""
from dataclasses import dataclass
import math
from .closed_chain import ClosurePolicy, evaluate_closure
from .se3 import pose_matrix, residual
from .state import AttachmentMode, StatePolicy, validate_state


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reasons: tuple
    closure: object = None
    collision: object = None
    fk_observation_errors: tuple = ()


class ClosedChainValidator:
    def __init__(self, model, collision_validator, state_policy=StatePolicy(), closure_policy=ClosurePolicy()):
        if collision_validator is None:
            raise ValueError("COLLISION_BACKEND_REQUIRED")
        if type(state_policy) is not StatePolicy or type(closure_policy) is not ClosurePolicy:
            raise ValueError("INVALID_VALIDATOR_POLICY_TYPE")
        if validate_state(None,0.,state_policy).reasons == ("INVALID_POLICY",):
            raise ValueError("INVALID_VALIDATOR_STATE_POLICY")
        # Recheck immutable policies as a guard against forged/tampered instances.
        closure_policy.__post_init__()
        self.model,self.collision_validator = model,collision_validator
        self.state_policy,self.closure_policy = state_policy,closure_policy

    def validate(self, state, now_s):
        validity=validate_state(state,now_s,self.state_policy)
        if not validity.valid:
            return ValidationResult(False,validity.reasons)
        if state.attachment_mode is not AttachmentMode.BILATERAL:
            return ValidationResult(False,("CLOSED_CHAIN_REQUIRES_BILATERAL_ATTACHMENT",))
        ql,qr=state.left_joints.positions_rad,state.right_joints.positions_rad
        reasons=[]
        for side,q in (("LEFT",ql),("RIGHT",qr)):
            if not self.model.within_limits(q):
                reasons.append(side+"_JOINT_LIMIT_VIOLATION")
        if reasons:
            return ValidationResult(False,tuple(reasons))
        frames=self.model.forward(ql,qr)
        obj=pose_matrix(state.cube.position_m,state.cube.quaternion_xyzw)
        errors=[]
        for side,observation in (("left",state.left_tcp),("right",state.right_tcp)):
            fk=frames[f"{side}_fr3_side_suction_tcp"]
            observed=pose_matrix(observation.position_m,observation.quaternion_xyzw)
            pos,rot=residual(fk,observed)
            errors.append((side,pos,rot))
            if pos>self.closure_policy.translation_tolerance_m or rot>self.closure_policy.rotation_tolerance_rad:
                reasons.append(side.upper()+"_FK_OBSERVATION_MISMATCH")
        closure=evaluate_closure(obj,frames["left_fr3_side_suction_tcp"],frames["right_fr3_side_suction_tcp"],policy=self.closure_policy)
        reasons.extend(closure.reasons)
        # The collision gate is never bypassed merely because an earlier numeric
        # closure test failed. Only structurally invalid states stop before FK.
        collision=self.collision_validator.validate(frames,obj,attachment_mode=state.attachment_mode.value)
        reasons.extend(collision.reasons)
        if not collision.valid and not collision.reasons:
            reasons.append("COLLISION_REJECTED")
        return ValidationResult(not reasons,tuple(reasons),closure,collision,tuple(errors))
