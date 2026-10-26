"""Mode-aware single-state validation and explicit endpoint transition guards.

A valid transition validates only its two snapshots and the declared mode event.
It computes no interpolated edge, swept volume, speed or continuous path safety.
"""
from dataclasses import dataclass
import numpy as np
from task28_validator.closed_chain import ClosurePolicy,evaluate_closure
from task28_validator.collision import CollisionValidator
from task28_validator.model import RobotModel,DEFAULT_FIXTURE
from task28_validator.se3 import pose_matrix,residual
from task28_validator.state import StatePolicy,StateValidationResult,validate_state
from .collision import PhaseCollisionValidator
from .modes import OperationMode,SupportSite,Snapshot,TransitionEvent,validate_snapshot_metadata
from .support import SupportPolicy


@dataclass(frozen=True)
class ModeValidationResult:
    valid: bool
    reasons: tuple
    metadata: object
    closure: object = None
    collision: object = None
    fk_observation_errors: tuple = ()


@dataclass(frozen=True)
class TransitionResult:
    valid: bool
    reasons: tuple
    event: object
    source: ModeValidationResult
    target: ModeValidationResult
    scope: str = 'MODE_AND_ENDPOINT_GUARDS_ONLY_NO_EDGE_SAFETY'


class ContactModeValidator:
    def __init__(self,fixture_dir=DEFAULT_FIXTURE,state_policy=StatePolicy(),closure_policy=ClosurePolicy()):
        if type(state_policy) is not StatePolicy or type(closure_policy) is not ClosurePolicy:
            raise ValueError('INVALID_CONTACT_MODE_POLICY')
        if validate_state(None,0.,state_policy).reasons==('INVALID_POLICY',):
            raise ValueError('INVALID_CONTACT_STATE_POLICY')
        closure_policy.__post_init__()
        self.model=RobotModel(fixture_dir)
        self.collision=PhaseCollisionValidator(CollisionValidator(fixture_dir),SupportPolicy(fixture_dir))
        self.state_policy,self.closure_policy=state_policy,closure_policy

    def validate(self,snapshot,now_s):
        try:
            if type(self.state_policy) is not StatePolicy or type(self.closure_policy) is not ClosurePolicy:
                raise ValueError('INVALID_CONTACT_MODE_POLICY')
            self.closure_policy.__post_init__()
        except (ValueError,TypeError):
            gate=StateValidationResult(False,('INVALID_CONTACT_MODE_POLICY',))
            return ModeValidationResult(False,gate.reasons,gate)
        metadata=validate_snapshot_metadata(snapshot,now_s,self.state_policy)
        if not metadata.valid:return ModeValidationResult(False,metadata.reasons,metadata)
        state=snapshot.input_state
        ql,qr=state.left_joints.positions_rad,state.right_joints.positions_rad
        reasons=[]
        for side,q in (('LEFT',ql),('RIGHT',qr)):
            if not self.model.within_limits(q):reasons.append(side+'_JOINT_LIMIT_VIOLATION')
        if reasons:return ModeValidationResult(False,tuple(reasons),metadata)
        frames=self.model.forward(ql,qr)
        obj=pose_matrix(state.cube.position_m,state.cube.quaternion_xyzw)
        errors=[]
        for side,observation in (('left',state.left_tcp),('right',state.right_tcp)):
            fk=frames[f'{side}_fr3_side_suction_tcp']
            observed=pose_matrix(observation.position_m,observation.quaternion_xyzw)
            pos,rot=residual(fk,observed);errors.append((side,pos,rot))
            if pos>self.closure_policy.translation_tolerance_m or rot>self.closure_policy.rotation_tolerance_rad:
                reasons.append(side.upper()+'_FK_OBSERVATION_MISMATCH')
        closure=None
        if snapshot.mode in (OperationMode.BILATERAL_SUPPORTED,OperationMode.BILATERAL_AIRBORNE):
            closure=evaluate_closure(obj,frames['left_fr3_side_suction_tcp'],frames['right_fr3_side_suction_tcp'],policy=self.closure_policy)
            reasons.extend(closure.reasons)
        collision=self.collision.validate(frames,obj,snapshot.mode,snapshot.support_site)
        reasons.extend(collision.reasons)
        if not collision.valid and not collision.reasons:reasons.append('PHASE_COLLISION_REJECTED')
        return ModeValidationResult(not reasons,tuple(reasons),metadata,closure,collision,tuple(errors))

    def validate_transition(self,source,target,event,source_now_s,target_now_s):
        before=self.validate(source,source_now_s);after=self.validate(target,target_now_s)
        reasons=['SOURCE_STATE:'+reason for reason in before.reasons]+['TARGET_STATE:'+reason for reason in after.reasons]
        if type(event) is not TransitionEvent:reasons.append('INVALID_TRANSITION_EVENT')
        if reasons:return TransitionResult(False,tuple(reasons),event,before,after)
        graph={
            TransitionEvent.ATTACH:(OperationMode.DETACHED_SUPPORTED,OperationMode.BILATERAL_SUPPORTED),
            TransitionEvent.LIFT:(OperationMode.BILATERAL_SUPPORTED,OperationMode.BILATERAL_AIRBORNE),
            TransitionEvent.TRANSPORT_SNAPSHOT:(OperationMode.BILATERAL_AIRBORNE,OperationMode.BILATERAL_AIRBORNE),
            TransitionEvent.PLACE:(OperationMode.BILATERAL_AIRBORNE,OperationMode.BILATERAL_SUPPORTED),
            TransitionEvent.RELEASE:(OperationMode.BILATERAL_SUPPORTED,OperationMode.RELEASED_SUPPORTED),
        }
        if (source.mode,target.mode)!=graph[event]:
            return TransitionResult(False,('ILLEGAL_MODE_TRANSITION',),event,before,after)
        if target_now_s<=source_now_s:reasons.append('NONINCREASING_SNAPSHOT_CLOCK')
        if source.observed.evidence_id==target.observed.evidence_id:reasons.append('REUSED_ATTACHMENT_EVIDENCE')
        observations=('left_joints','right_joints','left_tcp','right_tcp','cube')
        if any(getattr(target.input_state,key).stamp_s<=getattr(source.input_state,key).stamp_s for key in observations) or target.observed.stamp_s<=source.observed.stamp_s:
            reasons.append('OBSERVATIONS_NOT_ADVANCED')
        old_object=pose_matrix(source.input_state.cube.position_m,source.input_state.cube.quaternion_xyzw)
        new_object=pose_matrix(target.input_state.cube.position_m,target.input_state.cube.quaternion_xyzw)
        pos,rot=residual(new_object,old_object)
        if event in (TransitionEvent.ATTACH,TransitionEvent.RELEASE):
            expected_site=SupportSite.SUPPLY if event is TransitionEvent.ATTACH else SupportSite.TARGET
            if source.support_site is not expected_site or target.support_site is not expected_site:reasons.append('INCORRECT_ATTACH_RELEASE_SITE')
            if source.intent.request_id==target.intent.request_id:reasons.append('MISSING_EXPLICIT_ATTACHMENT_REQUEST')
            if not source.observed.stamp_s<target.intent.issued_at_s<=target.observed.stamp_s:reasons.append('ATTACH_RELEASE_CAUSAL_ORDER')
            if pos>1e-6 or rot>1e-6:reasons.append('OBJECT_MOVED_DURING_ATTACH_RELEASE')
            for side in ('left','right'):
                old_q=np.asarray(getattr(source.input_state,side+'_joints').positions_rad)
                new_q=np.asarray(getattr(target.input_state,side+'_joints').positions_rad)
                if np.max(np.abs(new_q-old_q))>1e-6:reasons.append(side.upper()+'_JOINT_MOVED_DURING_ATTACH_RELEASE')
        else:
            if source.intent!=target.intent:reasons.append('ATTACHMENT_INTENT_CHANGED_WITHOUT_ATTACH_RELEASE')
        if event in (TransitionEvent.LIFT,TransitionEvent.PLACE):
            expected_site=SupportSite.SUPPLY if event is TransitionEvent.LIFT else SupportSite.TARGET
            actual_site=source.support_site if event is TransitionEvent.LIFT else target.support_site
            if actual_site is not expected_site:reasons.append('INCORRECT_LIFT_PLACE_SITE')
            if np.max(np.abs(new_object[:2,3]-old_object[:2,3]))>1e-6 or rot>1e-6:reasons.append('LIFT_PLACE_NOT_VERTICAL_ENDPOINTS')
            dz=float(new_object[2,3]-old_object[2,3])
            if event is TransitionEvent.LIFT and dz<=1e-6:reasons.append('LIFT_HEIGHT_NOT_INCREASED')
            if event is TransitionEvent.PLACE and dz>=-1e-6:reasons.append('PLACE_HEIGHT_NOT_DECREASED')
        return TransitionResult(not reasons,tuple(reasons),event,before,after)
