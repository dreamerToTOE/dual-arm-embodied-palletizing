"""Versioned actual-model controls and prescribed A-stage inputs; no planner."""
from dataclasses import replace
import json
from pathlib import Path
from task28_contact.modes import (OperationMode as M,SupportSite,AttachmentIntent,ObservedAttachment,EvidenceSource)
from task28_contact.clearance import ConfirmationEvent as E,ClearanceFrame,CubePoseSource,ClearancePhase
from .types import (Waypoint,Provenance,Source,Trajectory,Motion,Interpolation,
                    JointInterpolation,TemporalObservation)

HERE=Path(__file__).resolve().parents[1]
REPO=HERE.parents[1]


def waypoint(config,time_s,identity,mode=M.BILATERAL_TRANSITIONING,site=None,attached=True,request='held-attachment'):
    intent=AttachmentIntent(request,-1.,attached,attached)
    evidence=identity+':attachment'
    return Waypoint(float(time_s),tuple(config['left_q']),tuple(config['right_q']),tuple(config['object_position_m']),
                    tuple(config.get('object_quaternion_xyzw',(0.,0.,0.,1.))),mode,site,attached,attached,intent,
                    ObservedAttachment(evidence,float(time_s),request,attached,attached,EvidenceSource.INDEPENDENT_SYNTHETIC),
                    Provenance(Source.EXPLICIT_SYNTHETIC,identity+':cube',evidence,'Prescribed synthetic computational input; not a measured observation.'))


def controls(model):
    configurations=json.loads((REPO/'offline/task28_g1_2a/examples/r1_configurations.json').read_text())['configurations']
    supply=configurations['supply_gap_4000um'];target=configurations['target_gap_4000um']
    a=waypoint(supply,0.,'hold-a');b=waypoint(supply,.02,'hold-b')
    positive=Trajectory('positive_stationary_bilateral', (a,b),Motion.GENERIC)
    closure=Trajectory('negative_A_linear_independent_arms',
                       (waypoint(supply,0.,'a-start'),waypoint(target,1.,'a-end')),Motion.GENERIC,
                       scope='Real frozen FK endpoints; independent joint and object linear interpolation deliberately need not preserve closure.')
    raw=json.loads((HERE/'examples/collision_control.json').read_text())
    collision=Trajectory('negative_B_payload_table',tuple(waypoint(row['configuration'],row['time_s'],'b-'+str(i))
                         for i,row in enumerate(raw['waypoints'])),Motion.GENERIC,
                         scope='Real frozen FK and bounded IK interior waypoint; deliberately unsafe table penetration, not executable motion.')
    aa=waypoint(supply,0.,'c-start');bb=waypoint(supply,1.,'c-end')
    tangent=4*(float(model.joint_upper[0])-aa.left_q[0]+.01)
    zeros=(0.,)*7
    aa=replace(aa,left_velocity_rad_s=(tangent,*zeros[1:]),right_velocity_rad_s=zeros)
    bb=replace(bb,left_velocity_rad_s=(-tangent,*zeros[1:]),right_velocity_rad_s=zeros)
    limits=Trajectory('negative_C_Hermite_limit_overshoot',(aa,bb),Motion.GENERIC,
                      Interpolation(joints=JointInterpolation.CUBIC_HERMITE),
                      scope='Mathematical cubic overshoot; endpoints valid, intermediate out of hardware range, never an executable command.')
    cfg=configurations['target_gap_0000um']
    da=waypoint(cfg,0.,'d-start',M.BILATERAL_SUPPORTED,SupportSite.TARGET)
    dm=waypoint(cfg,.01,'d-middle',M.RELEASED_SUPPORTED,SupportSite.TARGET,False,'unannounced-release')
    db=waypoint(cfg,.02,'d-end',M.BILATERAL_SUPPORTED,SupportSite.TARGET)
    mode=Trajectory('negative_D_illegal_interior_release',(da,dm,db),Motion.HOLD,
                    scope='All point geometries valid; unannounced attachment/contact mode changes without explicit release/attach events.')
    return {'positive_hold':positive,'A_closure':closure,'B_payload_collision':collision,'C_limit':limits,'D_mode':mode}


def a_stage(checker):
    raw=json.loads((HERE/'examples/a_stage_waypoints.json').read_text())
    mapping={'grasp_boundary':Motion.HOLD,'attachment_confirmation':Motion.ATTACH,'vertical_lift':Motion.LIFT,
             'bilateral_transport':Motion.TRANSPORT,'vertical_placement':Motion.PLACE,'support_confirmation':Motion.HOLD,'release_boundary':Motion.RELEASE}
    final_phases={'grasp_boundary':ClearancePhase.DETACHED_SUPPORTED_CONFIRMED,
                  'attachment_confirmation':ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED,
                  'vertical_lift':ClearancePhase.AIRBORNE_CONFIRMED,
                  'bilateral_transport':ClearancePhase.AIRBORNE_CONFIRMED,
                  'vertical_placement':ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED,
                  'support_confirmation':ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED,
                  'release_boundary':ClearancePhase.RELEASED_SUPPORTED_CONFIRMED}
    all_previous=[];output={}
    for name,segment in raw['segments'].items():
        rows=[]
        for row in segment['waypoints']:
            time_s=row['time_s'];stamp=row['observation_stamp_s'];intent=AttachmentIntent(**row['intent'])
            w=Waypoint(time_s,tuple(row['left_q']),tuple(row['right_q']),tuple(row['object_position_m']),tuple(row['object_quaternion_xyzw']),
                M(row['mode']),SupportSite(row['support_site']) if row['support_site'] else None,
                row['attached'],row['attached'],intent,
                ObservedAttachment(row['attachment_evidence_id'],stamp,intent.request_id,row['attached'],row['attached'],EvidenceSource.INDEPENDENT_SYNTHETIC),
                Provenance(Source.EXPLICIT_SYNTHETIC,row['cube_evidence_id'],row['attachment_evidence_id'],row['provenance']),
                E(row['confirmation_event']),observation_stamp_s=stamp)
            rows.append(w)
        prelude=[]
        prior=all_previous[:-1] if all_previous else []
        for w in prior:
            frame=ClearanceFrame(checker.snapshot(w),w.provenance.cube_evidence_id,CubePoseSource.SYNTHETIC_TEST_INPUT)
            prelude.append(TemporalObservation(frame,w.time_s,w.event))
        waypoints=tuple(([all_previous[-1]] if all_previous else [])+rows)
        output[name]=Trajectory('a_stage_'+name,waypoints,mapping[name],
                               temporal_prelude=tuple(prelude),enforce_temporal_confirmation=True,
                               required_terminal_phase=final_phases[name],
                               scope='Prescribed frozen-model single-state IK continuation; sampled joint-linear edges, not a planner or execution.')
        all_previous.extend(rows)
    return output


def placement_with_support_hold(checker):
    from dataclasses import replace
    segments=a_stage(checker)
    placement=segments['vertical_placement']
    hold=segments['support_confirmation']
    return replace(placement,identity='a_stage_placement_with_explicit_support_hold',
                   waypoints=placement.waypoints+hold.waypoints[1:])
