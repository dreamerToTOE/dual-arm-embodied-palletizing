"""Deterministic finite edge evaluation; no continuous certificate or repair."""
from dataclasses import asdict,dataclass
import math,time
import numpy as np
from scipy.spatial.transform import Rotation
from task28_validator.state import (InputState,JointObservation,PoseObservation,AttachmentMode,expected_joint_names)
from task28_validator.closed_chain import evaluate_closure
from task28_validator.se3 import pose_matrix,residual
from task28_contact.validator import ContactModeValidator
from task28_contact.modes import (Snapshot,OperationMode as M,SupportSite,TransitionEvent,AttachmentIntent,ObservedAttachment)
from task28_contact.clearance import (TemporalClearanceConfirmer,ClearanceFrame,CubePoseSource,ConfirmationEvent as E,ClearancePhase)
from task28_contact.clearance_geometry import ClearanceGeometry
from .types import (Trajectory,Waypoint,Provenance,Source,SamplingPolicy,Interpolation,Motion,
                    JointInterpolation,TranslationInterpolation,EdgeStatus,TemporalObservation)
from .interpolation import subdivisions,reconstruct
from .oracle import load_oracle


@dataclass(frozen=True)
class EdgeResult:
    status:EdgeStatus
    endpoints_valid:bool
    endpoint_results:tuple
    samples:tuple
    rejection_locations:tuple
    reasons:tuple
    planned_sample_count:int
    evaluated_sample_count:int
    resolution:dict
    maxima:dict
    temporal_evidence:tuple
    computation_time_s:float
    continuous_safety:str='NOT_ESTABLISHED_FINITE_SAMPLING_ONLY'


def finite(x):
    try:return type(x) in (float,int) and math.isfinite(x)
    except (OverflowError,TypeError,ValueError):return False
def numeric_tuple(x,n):return type(x) is tuple and len(x)==n and all(finite(v) for v in x)


def rejected_status(reasons):
    uncertain=('COLLISION_BACKEND_FAILURE','SUPPORT_POLICY_CONFIGURATION_CHANGED',
               'REFERENCE_REJECTED_WITHOUT_REASON','TERMINAL_PHASE_NOT_CONFIRMED')
    return EdgeStatus.SAMPLED_EDGE_REJECTED if any(not any(marker in reason for marker in uncertain) for reason in reasons) else EdgeStatus.EDGE_INCONCLUSIVE


def contract(trajectory,policy):
    if type(trajectory) is not Trajectory:return ('INVALID_TRAJECTORY_TYPE',)
    if (type(policy) is not SamplingPolicy or any(not finite(x) or x<=0 for x in (
        policy.max_dt_s,policy.max_joint_step_rad,policy.max_object_step_m,policy.max_rotation_step_rad))
        or type(policy.max_samples) is not int or not 2<=policy.max_samples<=50000):return ('INVALID_SAMPLING_POLICY',)
    if (type(trajectory.motion) is not Motion or type(trajectory.identity) is not str or not trajectory.identity.strip()
        or type(trajectory.waypoints) is not tuple or not 2<=len(trajectory.waypoints)<=10000
        or type(trajectory.interpolation) is not Interpolation):return ('INVALID_TRAJECTORY_CONTRACT',)
    spec=trajectory.interpolation
    if (type(spec.joints) is not JointInterpolation or type(spec.translation) is not TranslationInterpolation
        or spec.rotation!='shortest_so3_geodesic_xyzw'):return ('UNSUPPORTED_INTERPOLATION',)
    previous=None;cube_ids=set();feedback_ids=set()
    for index,w in enumerate(trajectory.waypoints):
        if type(w) is not Waypoint:return (f'INVALID_WAYPOINT_TYPE:{index}',)
        if not finite(w.time_s) or (previous is not None and w.time_s<=previous):return (f'NONINCREASING_WAYPOINT_TIME:{index}',)
        previous=w.time_s
        if not (numeric_tuple(w.left_q,7) and numeric_tuple(w.right_q,7) and numeric_tuple(w.object_position_m,3)
                and numeric_tuple(w.object_quaternion_xyzw,4)):return (f'INVALID_WAYPOINT_NUMBERS:{index}',)
        if abs(math.hypot(*w.object_quaternion_xyzw)-1)>1e-6:return (f'NONUNIT_WAYPOINT_QUATERNION:{index}',)
        if type(w.mode) is not M or (w.support_site is not None and type(w.support_site) is not SupportSite):return (f'INVALID_CONTACT_MODE:{index}',)
        p=w.provenance
        if (type(p) is not Provenance or p.source is not Source.EXPLICIT_SYNTHETIC or
            any(type(x) is not str or not x.strip() for x in (p.cube_evidence_id,p.attachment_evidence_id,p.description))):return (f'INVALID_WAYPOINT_PROVENANCE:{index}',)
        if p.cube_evidence_id in cube_ids or p.attachment_evidence_id in feedback_ids:return (f'REUSED_WAYPOINT_EVIDENCE:{index}',)
        cube_ids.add(p.cube_evidence_id);feedback_ids.add(p.attachment_evidence_id)
        if type(w.event) is not E:return (f'INVALID_WAYPOINT_EVENT:{index}',)
        if (type(w.intent) is not AttachmentIntent or type(w.observed) is not ObservedAttachment
            or type(w.left_attached) is not bool or type(w.right_attached) is not bool
            or (w.observation_stamp_s is not None and not finite(w.observation_stamp_s))):return (f'INVALID_WAYPOINT_METADATA_TYPE:{index}',)
        if getattr(w.observed,'evidence_id',None)!=p.attachment_evidence_id:return (f'FEEDBACK_ID_MISMATCH:{index}',)
        if spec.joints is JointInterpolation.CUBIC_HERMITE and not (numeric_tuple(w.left_velocity_rad_s,7) and numeric_tuple(w.right_velocity_rad_s,7)):
            return (f'MISSING_HERMITE_JOINT_TANGENT:{index}',)
        if spec.translation is TranslationInterpolation.CUBIC_HERMITE and not numeric_tuple(w.object_velocity_m_s,3):return (f'MISSING_HERMITE_OBJECT_TANGENT:{index}',)
    if type(trajectory.enforce_temporal_confirmation) is not bool or type(trajectory.temporal_prelude) is not tuple:return ('INVALID_TEMPORAL_CONTRACT',)
    if trajectory.enforce_temporal_confirmation and type(trajectory.required_terminal_phase) is not ClearancePhase:return ('REQUIRED_TERMINAL_PHASE_MISSING_OR_INVALID',)
    if not trajectory.enforce_temporal_confirmation and (trajectory.required_terminal_phase is not None or trajectory.temporal_prelude):return ('TERMINAL_PHASE_REQUIRES_TEMPORAL_CONFIRMATION',)
    if len(trajectory.temporal_prelude)>5000:return ('TEMPORAL_CONTEXT_BUDGET_EXCEEDED',)
    for obs in trajectory.temporal_prelude:
        if (type(obs) is not TemporalObservation or type(obs.frame) is not ClearanceFrame or not finite(obs.now_s)
            or type(obs.event) is not E or type(obs.frame.snapshot) is not Snapshot
            or type(obs.frame.snapshot.input_state) is not InputState
            or type(obs.frame.snapshot.intent) is not AttachmentIntent or type(obs.frame.snapshot.observed) is not ObservedAttachment
            or obs.frame.cube_pose_source is not CubePoseSource.SYNTHETIC_TEST_INPUT
            or type(obs.frame.cube_evidence_id) is not str or not obs.frame.cube_evidence_id.strip()):return ('INVALID_TEMPORAL_PRELUDE_CONTRACT',)
    return ()


class SampledEdgeValidator:
    def __init__(self,fixture_dir=None):
        self.contact=ContactModeValidator() if fixture_dir is None else ContactModeValidator(fixture_dir)
        self.model=self.contact.model
        self.oracle=load_oracle()
        self.geometry=ClearanceGeometry(self.contact.collision.support)

    def snapshot(self,w):
        stamp=w.time_s if w.observation_stamp_s is None else w.observation_stamp_s
        fields={}
        for side,q in (('left',w.left_q),('right',w.right_q)):
            tcp=self.oracle.usd_fk(self.model.fixture_dir,side,q)
            fields[side+'_joints']=JointObservation(side,'world',stamp,expected_joint_names(side),q)
            fields[side+'_tcp']=PoseObservation('world',stamp,tuple(float(x) for x in tcp[:3,3]),
                                               tuple(float(x) for x in Rotation.from_matrix(tcp[:3,:3]).as_quat()))
        fields['cube']=PoseObservation('world',stamp,w.object_position_m,w.object_quaternion_xyzw,'Cube_01')
        bilateral=w.mode in (M.BILATERAL_SUPPORTED,M.BILATERAL_TRANSITIONING,M.BILATERAL_AIRBORNE)
        state=InputState(**fields,attachment_mode=AttachmentMode.BILATERAL if bilateral else AttachmentMode.DETACHED,
                         left_attached=w.left_attached,right_attached=w.right_attached)
        return Snapshot(state,w.mode,w.support_site,w.intent,w.observed)

    def evaluate(self,w):
        start=time.perf_counter()
        snapshot=self.snapshot(w)
        result=self.contact.validate(snapshot,w.time_s)
        reasons=list(result.reasons)
        ql,qr=np.array(w.left_q),np.array(w.right_q)
        margin=float(min(np.min(q-self.model.joint_lower) for q in (ql,qr)))
        margin=min(margin,float(min(np.min(self.model.joint_upper-q) for q in (ql,qr))))
        frames=self.model.forward(ql,qr)
        independent=[]
        for side,q in (('left',ql),('right',qr)):
            oracle=self.oracle.usd_link_frames(self.model.fixture_dir,side,q)
            oracle[side+'_fr3_side_suction_tcp']=self.oracle.usd_fk(self.model.fixture_dir,side,q)
            for link,expected in oracle.items():
                pos,rot=self.oracle.residuals(frames[link],expected)
                independent.append({'link':link,'translation_error_m':pos,'rotation_error_rad':rot})
                if pos>1e-6 or rot>1e-6:reasons.append('INDEPENDENT_FK_MISMATCH:'+link)
        obj=pose_matrix(w.object_position_m,w.object_quaternion_xyzw)
        bilateral=w.mode in (M.BILATERAL_SUPPORTED,M.BILATERAL_TRANSITIONING,M.BILATERAL_AIRBORNE)
        closure=result.closure
        if closure is None and bilateral:
            closure=evaluate_closure(obj,frames['left_fr3_side_suction_tcp'],frames['right_fr3_side_suction_tcp'],policy=self.contact.closure_policy)
            reasons.extend(closure.reasons)
        # Even an out-of-limit but structurally finite mathematical input gets
        # explicit FK/closure/FCL diagnostics; it is never accepted or executed.
        collision=result.collision
        if collision is None:
            collision=self.contact.collision.validate(frames,obj,w.mode,w.support_site)
            reasons.extend(collision.reasons)
        geometry=self.geometry.evaluate(obj)
        reasons.extend(geometry.reasons)
        closure_data=None if closure is None else asdict(closure)
        return {'time_s':w.time_s,'valid':not reasons,'reasons':tuple(dict.fromkeys(reasons)),
                'waypoint':asdict(w),'minimum_joint_limit_margin_rad':margin,
                'closure':closure_data,'independent_fk':independent,
                'collision':asdict(collision),'geometry':asdict(geometry),
                'world_tcp_poses':{side:frames[side+'_fr3_side_suction_tcp'] for side in ('left','right')},
                'tcp_reconstruction':'hash_bound_USD_frame_oracle_not_measured_observation',
                'computation_time_s':time.perf_counter()-start}

    def mode_schedule(self,trajectory):
        reasons=[];waypoints=trajectory.waypoints
        separated=(M.BILATERAL_TRANSITIONING,M.BILATERAL_AIRBORNE)
        event_counts={}
        for i,w in enumerate(waypoints):
            if w.event is not E.OBSERVE:
                event_counts[w.event]=event_counts.get(w.event,0)+1
                expected={E.ATTACH:Motion.ATTACH,E.BEGIN_LIFT:Motion.LIFT,E.BEGIN_PLACE:Motion.PLACE,E.RELEASE:Motion.RELEASE}[w.event]
                if trajectory.motion is not expected or i>1:
                    reasons.append((i,'PHASE_EVENT_IN_WRONG_MOTION_OR_LOCATION'))
                if event_counts[w.event]>1:reasons.append((i,'REPEATED_PHASE_EVENT'))
            permitted={Motion.LIFT:(M.BILATERAL_SUPPORTED,*separated),Motion.PLACE:(M.BILATERAL_SUPPORTED,*separated),
                       Motion.TRANSPORT:separated,Motion.ATTACH:(M.DETACHED_SUPPORTED,M.BILATERAL_SUPPORTED),
                       Motion.RELEASE:(M.BILATERAL_SUPPORTED,M.RELEASED_SUPPORTED)}.get(trajectory.motion)
            if permitted is not None and w.mode not in permitted:reasons.append((i,'MODE_NOT_ALLOWED_FOR_MOTION'))
            if w.mode is M.BILATERAL_SUPPORTED and trajectory.motion in (Motion.LIFT,Motion.PLACE):
                site=SupportSite.SUPPLY if trajectory.motion is Motion.LIFT else SupportSite.TARGET
                if w.support_site is not site:reasons.append((i,'INCORRECT_MOTION_SUPPORT_SITE'))
            if trajectory.motion in (Motion.LIFT,Motion.PLACE):
                a=waypoints[0]
                pos,rot=residual(pose_matrix(a.object_position_m,a.object_quaternion_xyzw),pose_matrix(w.object_position_m,w.object_quaternion_xyzw))
                if max(abs(w.object_position_m[j]-a.object_position_m[j]) for j in (0,1))>1e-6 or rot>1e-6:
                    reasons.append((i,'LIFT_PLACE_NOT_VERTICAL_WAYPOINTS'))
            if i==0:continue
            a=waypoints[i-1]
            if w.event in (E.ATTACH,E.RELEASE):
                wanted=Motion.ATTACH if w.event is E.ATTACH else Motion.RELEASE
                if trajectory.motion is not wanted:reasons.append((i,'ATTACHMENT_EVENT_IN_WRONG_MOTION'))
                event=TransitionEvent.ATTACH if w.event is E.ATTACH else TransitionEvent.RELEASE
                gate=self.contact.validate_transition(self.snapshot(a),self.snapshot(w),event,a.time_s,w.time_s)
                reasons.extend((i,'BOUNDARY_EVENT:'+reason) for reason in gate.reasons)
            else:
                if w.intent!=a.intent or (w.left_attached,w.right_attached)!=(a.left_attached,a.right_attached):
                    reasons.append((i,'ATTACHMENT_CHANGED_WITHOUT_EXPLICIT_EVENT'))
                if w.mode!=a.mode:
                    allowed=(trajectory.motion in (Motion.LIFT,Motion.PLACE) and
                             a.mode in (M.BILATERAL_SUPPORTED,*separated) and w.mode in (M.BILATERAL_SUPPORTED,*separated)) or (
                             trajectory.motion is Motion.TRANSPORT and a.mode in separated and w.mode in separated)
                    if not allowed:reasons.append((i,'ILLEGAL_INTERIOR_CONTACT_MODE_TRANSITION'))
        return reasons

    def validate(self,trajectory,policy=SamplingPolicy(),endpoint_only=False):
        start=time.perf_counter();bad=contract(trajectory,policy)
        resolution=asdict(policy) if type(policy) is SamplingPolicy else {}
        if type(trajectory) is Trajectory:resolution['temporal_confirmation_requested']=trajectory.enforce_temporal_confirmation
        def finish(status,endpoints,samples,locations,reasons,planned,temporal=()):
            closure=[r['closure'] for r in samples if r['closure'] is not None]
            maxima={'max_closure_translation_m':max((max(c['translation_error_m'],c['left_object_translation_error_m'],c['right_object_translation_error_m']) for c in closure),default=None),
                    'max_closure_rotation_rad':max((max(c['rotation_error_rad'],c['left_object_rotation_error_rad'],c['right_object_rotation_error_rad']) for c in closure),default=None),
                    'minimum_joint_limit_margin_rad':min((r['minimum_joint_limit_margin_rad'] for r in samples),default=None)}
            return EdgeResult(status,bool(endpoints) and all(x['valid'] for x in endpoints),tuple(endpoints),tuple(samples),tuple(locations),tuple(dict.fromkeys(reasons)),
                              planned,len(samples),resolution,maxima,tuple(temporal),time.perf_counter()-start)
        if bad:return finish(EdgeStatus.EDGE_INCONCLUSIVE,(),(),(),bad,0)
        waypoints=trajectory.waypoints
        endpoints=(self.evaluate(waypoints[0]),self.evaluate(waypoints[-1]))
        endpoint_reasons=[reason for row in endpoints for reason in row['reasons']]
        if endpoint_only:
            return finish(EdgeStatus.ENDPOINT_VALID if all(x['valid'] for x in endpoints) else rejected_status(endpoint_reasons),
                          endpoints,endpoints,(),endpoint_reasons,2)
        try:
            counts=[subdivisions(a,b,trajectory.interpolation,policy) for a,b in zip(waypoints,waypoints[1:])]
        except (ValueError,OverflowError) as error:
            return finish(EdgeStatus.EDGE_INCONCLUSIVE,endpoints,endpoints,(),('SAMPLING_BOUND_ERROR:'+str(error),),2)
        planned=1+sum(counts);resolution['span_subdivisions']=counts
        if planned>policy.max_samples:
            status=rejected_status(endpoint_reasons) if endpoint_reasons else EdgeStatus.EDGE_INCONCLUSIVE
            return finish(status,endpoints,endpoints,(),(*endpoint_reasons,'SAMPLE_BUDGET_EXCEEDED'),planned)
        samples=[];locations=[];reasons=[]
        for span,(a,b,n) in enumerate(zip(waypoints,waypoints[1:],counts)):
            for k in range(n+1):
                if span and k==0:continue
                u=k/n
                w=reconstruct(a,b,u,trajectory.interpolation,trajectory.motion,self.contact.collision.support,trajectory.identity)
                if span==0 and k==0:row=dict(endpoints[0])
                elif span==len(counts)-1 and k==n:row=dict(endpoints[1])
                else:row=self.evaluate(w)
                row.update(span_index=span,u=u,explicit_waypoint_index=span if k==0 else span+1 if k==n else None)
                samples.append(row)
                if not row['valid']:
                    locations.append({'sample_index':len(samples)-1,'span_index':span,'u':u,'time_s':w.time_s,'reasons':row['reasons']})
                    reasons.extend(row['reasons'])
        for index,reason in self.mode_schedule(trajectory):
            locations.append({'waypoint_index':index,'time_s':waypoints[index].time_s,'reasons':(reason,)})
            reasons.append(reason)
        temporal=[]
        if trajectory.enforce_temporal_confirmation:
            confirmer=TemporalClearanceConfirmer(self.contact)
            observations=list(trajectory.temporal_prelude)
            for w in waypoints:
                frame=ClearanceFrame(self.snapshot(w),w.provenance.cube_evidence_id,CubePoseSource.SYNTHETIC_TEST_INPUT)
                observations.append(TemporalObservation(frame,w.time_s,w.event))
            for index,obs in enumerate(observations):
                if type(obs) is not TemporalObservation or type(obs.frame) is not ClearanceFrame:
                    reasons.append('INVALID_TEMPORAL_CONTEXT');break
                result=confirmer.observe(obs.frame,obs.now_s,obs.event)
                temporal.append({'observation_index':index,'prelude':index<len(trajectory.temporal_prelude),
                                 'now_s':obs.now_s,'event':obs.event.value,'frame':asdict(obs.frame),'result':asdict(result)})
                if not result.valid:
                    reasons.extend('TEMPORAL:'+r for r in result.reasons)
                    locations.append({'temporal_observation_index':index,'time_s':obs.now_s,'reasons':result.reasons})
        resolution['temporal_observation_checks']=len(temporal)
        resolution['interpolated_points_used_for_temporal_confirmation']=0
        if temporal:
            resolution['temporal_end_phase']=temporal[-1]['result']['phase'].value
            resolution['temporal_end_confirmed']=temporal[-1]['result']['phase_confirmed']
            resolution['required_terminal_phase']=trajectory.required_terminal_phase.value
            if temporal[-1]['result']['phase'] is not trajectory.required_terminal_phase or not temporal[-1]['result']['phase_confirmed']:
                reasons.append('TERMINAL_PHASE_NOT_CONFIRMED')
        if reasons:return finish(rejected_status(reasons),endpoints,samples,locations,reasons,planned,temporal)
        return finish(EdgeStatus.SAMPLED_EDGE_VALID,endpoints,samples,(),(),planned,temporal)
