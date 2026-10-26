"""Bounded deterministic offline experiments for R1 evidence, not a planner."""
from dataclasses import asdict, replace
from task28_contact.clearance import (TemporalClearanceConfirmer, ClearanceParameters,
    ClearancePhase, ConfirmationEvent, GeometricContact)
from task28_contact.clearance_examples import SyntheticFrames, representative_sequence


def trace_row(label, frame, now, event, result):
    row={key:value for key,value in asdict(result).items() if key not in ('geometry','validation')}
    row.update(label=label, now_s=now, event=event.value,
               cube_evidence_id=frame.cube_evidence_id,
               cube_pose_source=frame.cube_pose_source.value,
               feedback_evidence_id=frame.snapshot.observed.evidence_id,
               feedback_source=frame.snapshot.observed.source.value,
               observation_stamps_s={key:getattr(frame.snapshot.input_state,key).stamp_s
                                     for key in ('left_joints','right_joints','left_tcp','right_tcp','cube')},
               feedback_stamp_s=frame.snapshot.observed.stamp_s,
               object_position_m=frame.snapshot.input_state.cube.position_m,
               object_quaternion_xyzw=frame.snapshot.input_state.cube.quaternion_xyzw,
               operation_mode=frame.snapshot.mode.value)
    if result.geometry is not None:
        row['geometry']=asdict(result.geometry)
    if result.validation is not None:
        validation=result.validation
        row['validation']={'valid':validation.valid,'reasons':validation.reasons,
                           'fk_observation_errors':validation.fk_observation_errors,
                           'closure':None if validation.closure is None else asdict(validation.closure),
                           'checked_shape_pairs':None if validation.collision is None else validation.collision.reference.checked_shape_pairs,
                           'skipped_acm_shape_pairs':None if validation.collision is None else validation.collision.reference.skipped_acm_shape_pairs,
                           'accepted_support_records':0 if validation.collision is None else len(validation.collision.accepted_support_contacts)}
    return row


def bootstrap(validator, stop, parameters=None):
    gate=TemporalClearanceConfirmer(validator, parameters)
    trace=[]
    for label,frame,now,event in representative_sequence(validator.model):
        result=gate.observe(frame,now,event)
        assert result.valid,(label,result.reasons)
        trace.append(trace_row(label,frame,now,event,result))
        if label==stop:
            factory=SyntheticFrames(validator.model)
            factory.index=int(round((now-20.)/.01));factory.intent=frame.snapshot.intent
            return gate,factory,trace
    raise ValueError('UNKNOWN_SEQUENCE_LABEL')


def run_representative(validator):
    return bootstrap(validator,'release_3')[2]


def sensitivity(validator):
    """Finite grid; frame index is counted after bootstrap, never latency."""
    output=[]
    for high in (.0025,.003,.0035):
        for amplitude in (0.,.0001,.0005):
            gate,factory,_=bootstrap(validator,'lift_initiation',ClearanceParameters(d_high_m=high))
            gaps=[.003+amplitude,.003-amplitude]*3+[.003+amplitude]*3+[.004]*3
            first=None;rows=[]
            for i,gap in enumerate(gaps,1):
                frame,now=factory.make(gap_m=gap);r=gate.observe(frame,now)
                assert r.valid,r.reasons
                if r.phase is ClearancePhase.AIRBORNE_CONFIRMED and first is None:first=i
                rows.append({'frame':i,'gap_m':gap,'high_count':r.high_count,'phase':r.phase.value,
                             'contact':r.contact.value,'support_records':len(r.validation.collision.accepted_support_contacts)})
            output.append({'direction':'lift','d_low_m':.001,'d_high_m':high,'noise_amplitude_m':amplitude,
                           'first_confirmation_frame':first,'frames':rows})
    for low in (.00075,.001,.00125):
        for amplitude in (0.,.0001):
            gate,factory,_=bootstrap(validator,'place_initiation',ClearanceParameters(d_low_m=low))
            gaps=[.001-amplitude,.001+amplitude]*3+[.001-amplitude]*3+[.0005]*3
            first=None;rows=[]
            for i,gap in enumerate(gaps,1):
                frame,now=factory.make('target',gap);r=gate.observe(frame,now)
                assert r.valid,r.reasons
                if r.phase is ClearancePhase.NEAR_TABLE_CONFIRMED and first is None:first=i
                assert not r.can_release and r.contact is GeometricContact.SEPARATED
                rows.append({'frame':i,'gap_m':gap,'low_count':r.low_count,'phase':r.phase.value,
                             'contact':r.contact.value,'can_release':r.can_release})
            output.append({'direction':'place_approach','d_low_m':low,'d_high_m':.003,'noise_amplitude_m':amplitude,
                           'first_confirmation_frame':first,'frames':rows})
    return output


def support_noise(validator):
    rows=[]
    for site in ('supply','target'):
        # This tests Cube-only observation perturbations at the fixed supported
        # q. Millimetre phase parameters never enlarge the support gate.
        template=SyntheticFrames(validator.model)
        frame,now=template.make(site,request=('bilateral-test',True))
        for delta in (-.000002,-.0000005,0.,.0000005,.000002,.0001,.0005):
            state=frame.snapshot.input_state;xyz=state.cube.position_m
            state=replace(state,cube=replace(state.cube,position_m=(*xyz[:2],xyz[2]+delta)))
            result=validator.validate(replace(frame.snapshot,input_state=state),now)
            rows.append({'site':site,'cube_observation_z_noise_m':delta,'valid_supported_state':result.valid,
                         'reasons':result.reasons,'accepted_support_records':0 if result.collision is None else len(result.collision.accepted_support_contacts)})
    return rows


def negative_controls(validator):
    rows=[]
    for name in ('penetration','wrong_support_site','wrong_table_footprint','stale','asynchronous',
                 'duplicate_cube','duplicate_feedback','out_of_order','attachment_loss','closure_mismatch',
                 'illegal_place','unsupported_release'):
        gate,factory,_=bootstrap(validator,'lift_initiation')
        first_frame,first_now=factory.make(gap_m=.003)
        assert gate.observe(first_frame,first_now).valid
        frame,now=factory.make(gap_m=.003);s=frame.snapshot;state=s.input_state;event=ConfirmationEvent.OBSERVE
        if name=='penetration':state=replace(state,cube=replace(state.cube,position_m=(.65,0,.259998)))
        elif name=='wrong_support_site':
            from task28_contact.modes import OperationMode,SupportSite
            s=replace(s,mode=OperationMode.BILATERAL_SUPPORTED,support_site=SupportSite.SUPPLY)
            state=replace(state,cube=replace(state.cube,position_m=(.9,.2,.26)))
        elif name=='wrong_table_footprint':state=replace(state,cube=replace(state.cube,position_m=(1.14,0,.263)))
        elif name=='stale':state=replace(state,cube=replace(state.cube,stamp_s=now-.2))
        elif name=='asynchronous':state=replace(state,right_tcp=replace(state.right_tcp,stamp_s=now-.04))
        elif name=='duplicate_cube':frame=replace(frame,cube_evidence_id=first_frame.cube_evidence_id)
        elif name=='duplicate_feedback':s=replace(s,observed=replace(s.observed,evidence_id=first_frame.snapshot.observed.evidence_id))
        elif name=='out_of_order':state=replace(state,cube=replace(state.cube,stamp_s=first_frame.snapshot.input_state.cube.stamp_s))
        elif name=='attachment_loss':s=replace(s,observed=replace(s.observed,left=False))
        elif name=='closure_mismatch':state=replace(state,cube=replace(state.cube,position_m=(.66,0,.263)))
        elif name=='illegal_place':event=ConfirmationEvent.BEGIN_PLACE
        elif name=='unsupported_release':event=ConfirmationEvent.RELEASE
        frame=replace(frame,snapshot=replace(s,input_state=state))
        result=gate.observe(frame,now,event)
        assert not result.valid,(name,result)
        assert not result.phase_confirmed and not result.can_release
        rows.append(trace_row(name,frame,now,event,result))
    return rows
