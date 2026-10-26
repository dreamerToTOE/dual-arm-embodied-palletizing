"""R1 sequence/negative controls with real frozen FK, full closure and FCL.

All Cube/TCP/attachment observations here are independent synthetic test inputs.
No physical timing, support force or attachment measurement is claimed.
"""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest
from task28_contact.clearance import (TemporalClearanceConfirmer, ClearanceParameters,
    ClearancePhase as P, ConfirmationEvent as E, GeometricContact as C, CubePoseSource)
from task28_contact.clearance_examples import SyntheticFrames, representative_sequence, CONFIG_PATH
from task28_contact.modes import OperationMode as M, SupportSite
from task28_contact.validator import ContactModeValidator
from task28_validator.state import AttachmentMode


@pytest.fixture(scope='module')
def validator():
    return ContactModeValidator()


def at(validator, stop):
    gate=TemporalClearanceConfirmer(validator)
    rows=representative_sequence(validator.model)
    for label,frame,now,event in rows:
        result=gate.observe(frame,now,event)
        assert result.valid,(label,result.reasons)
        if label==stop:
            factory=SyntheticFrames(validator.model)
            factory.index=int(round((now-20.)/.01))
            factory.intent=frame.snapshot.intent
            return gate,factory,result
    raise AssertionError(stop)


def apply(gate,factory,site='supply',gap=0.,event=E.OBSERVE,**kwargs):
    frame,now=factory.make(site,gap,**kwargs)
    return gate.observe(frame,now,event)


def test_full_sequence_has_independently_expected_phases_and_contact(validator):
    gate=TemporalClearanceConfirmer(validator)
    results={}
    for label,frame,now,event in representative_sequence(validator.model):
        r=gate.observe(frame,now,event); results[label]=r
        assert r.valid,(label,r.reasons)
        assert r.physical_support_observed is False
        assert r.validation.collision.reference.checked_shape_pairs==414
        assert r.validation.collision.reference.skipped_acm_shape_pairs==40
        assert len(r.validation.collision.accepted_support_contacts)==(1 if r.contact is C.SUPPORT_CANDIDATE else 0)
        if frame.snapshot.mode in (M.BILATERAL_TRANSITIONING,M.BILATERAL_SUPPORTED):
            assert r.validation.closure.valid
    assert results['initial_support_2'].phase is P.INITIALIZING
    assert results['initial_support_3'].phase is P.DETACHED_SUPPORTED_CONFIRMED
    assert results['attach_2'].phase is P.ATTACHING
    assert results['attach_3'].phase is P.BILATERAL_SUPPORTED_CONFIRMED
    for label in ('lift_0.5mm','lift_1mm','lift_2mm'):
        assert results[label].contact is C.SEPARATED
        assert results[label].phase is P.LIFTING
        assert not results[label].phase_confirmed
    assert results['high_3mm_1'].high_count==1
    assert results['high_3mm_2'].high_count==2
    assert results['high_3mm_3'].phase is P.AIRBORNE_CONFIRMED
    assert results['descend_19'].phase is P.NEAR_TABLE_CONFIRMED
    assert results['descend_19'].contact is C.SEPARATED
    assert not results['descend_19'].can_release
    assert not results['target_strict_support_2'].can_release
    assert results['target_strict_support_3'].can_release
    assert results['release_2'].phase is P.RELEASING
    assert results['release_3'].phase is P.RELEASED_SUPPORTED_CONFIRMED


@pytest.mark.parametrize('gap',[.0005,.001,.002])
def test_subhigh_separation_never_gets_support_or_air_confirmation(validator,gap):
    gate,factory,_=at(validator,'lift_initiation')
    for _ in range(4):
        r=apply(gate,factory,gap=gap)
        assert r.valid and r.contact is C.SEPARATED
        assert r.phase is P.LIFTING and not r.phase_confirmed
        assert not r.can_release and r.high_count==0
        assert not r.validation.collision.accepted_support_contacts
        assert r.validation.closure.valid


def test_three_high_are_consecutive_and_threshold_equality_qualifies(validator):
    gate,factory,_=at(validator,'lift_initiation')
    for gap,expected in ((.003,1),(.003,2),(.0029,0),(.003,1),(.003,2),(.003,3)):
        r=apply(gate,factory,gap=gap)
        assert r.valid and r.high_count==expected
        assert r.phase_confirmed==(expected==3)


def test_air_latch_does_not_chatter_with_threshold_noise(validator):
    gate,factory,_=at(validator,'high_3mm_3')
    for gap in (.0029,.0031,.002,.0011,.0009,.0011,.0009,.0011):
        r=apply(gate,factory,gap=gap)
        assert r.valid and r.phase is P.AIRBORNE_CONFIRMED and r.phase_confirmed
        assert r.contact is C.SEPARATED


def test_unannounced_sustained_descent_is_not_an_implicit_place(validator):
    gate,factory,_=at(validator,'high_3mm_3')
    assert apply(gate,factory,gap=.0005).valid
    assert apply(gate,factory,gap=.0005).valid
    r=apply(gate,factory,gap=.0005)
    assert not r.valid and r.reasons==('DESCENT_REQUIRES_BEGIN_PLACE_EVENT',)
    assert not r.phase_confirmed and not r.can_release


def test_low_three_confirms_near_table_only_and_latches_in_dead_band(validator):
    gate,factory,_=at(validator,'place_initiation')
    for i in range(3):
        r=apply(gate,factory,'target',.001)
        assert r.valid and r.contact is C.SEPARATED
        assert r.low_count==i+1
    assert r.phase is P.NEAR_TABLE_CONFIRMED and not r.can_release
    for gap in (.0011,.002,.0011,.0009,.0011):
        r=apply(gate,factory,'target',gap)
        assert r.valid and r.phase is P.NEAR_TABLE_CONFIRMED
        assert not r.can_release and r.contact is C.SEPARATED
    for i in range(3): r=apply(gate,factory,'target',.003)
    assert r.valid and r.phase is P.PLACING and not r.phase_confirmed


@pytest.mark.parametrize('stage',['lift_initiation','place_initiation'])
@pytest.mark.parametrize('bad',['stale','async','duplicate_cube','duplicate_feedback','frozen_cube_stamp',
                                'out_of_order','invalid_quaternion','wrong_frame','attachment_loss',
                                'joint_limit','closure','collision'])
def test_invalid_breaks_runs_and_never_contributes(validator,stage,bad):
    gate,factory,_=at(validator,stage)
    placing=stage=='place_initiation'; site='target' if placing else 'supply'
    gap=.0005 if placing else .003
    first_frame,first_now=factory.make(site,gap)
    first=gate.observe(first_frame,first_now)
    assert first.valid
    second=apply(gate,factory,site,gap);assert second.valid
    frame,now=factory.make(site,gap);s=frame.snapshot;state=s.input_state
    if bad=='stale': state=replace(state,cube=replace(state.cube,stamp_s=now-.2))
    elif bad=='async': state=replace(state,cube=replace(state.cube,stamp_s=now-.04))
    elif bad=='duplicate_cube': frame=replace(frame,cube_evidence_id=first_frame.cube_evidence_id)
    elif bad=='duplicate_feedback': s=replace(s,observed=replace(s.observed,evidence_id=first_frame.snapshot.observed.evidence_id))
    elif bad=='frozen_cube_stamp': state=replace(state,cube=replace(state.cube,stamp_s=gate._last_stamps[4]))
    elif bad=='out_of_order': state=replace(state,left_joints=replace(state.left_joints,stamp_s=gate._last_stamps[0]-.001))
    elif bad=='invalid_quaternion': state=replace(state,cube=replace(state.cube,quaternion_xyzw=(0.,0.,0.,0.)))
    elif bad=='wrong_frame': state=replace(state,cube=replace(state.cube,frame_id='map'))
    elif bad=='attachment_loss': s=replace(s,observed=replace(s.observed,left=False))
    elif bad=='joint_limit':
        q=list(state.left_joints.positions_rad);q[0]=float(validator.model.joint_upper[0]+.001)
        state=replace(state,left_joints=replace(state.left_joints,positions_rad=tuple(q)))
    elif bad=='closure': state=replace(state,cube=replace(state.cube,position_m=(state.cube.position_m[0]+.01,*state.cube.position_m[1:])))
    elif bad=='collision': state=replace(state,cube=replace(state.cube,position_m=(.65,0,.1)))
    s=replace(s,input_state=state);frame=replace(frame,snapshot=s)
    r=gate.observe(frame,now)
    assert not r.valid and not r.phase_confirmed
    assert (r.high_count,r.low_count,r.support_count,r.attachment_count,r.release_count)==(0,0,0,0,0)
    r=apply(gate,factory,site,gap)
    assert r.valid
    assert (r.low_count if placing else r.high_count)==1
    assert not r.phase_confirmed


def test_replayed_nonadjacent_ids_are_rejected(validator):
    gate,factory,_=at(validator,'lift_initiation')
    frames=[factory.make(gap_m=.003) for _ in range(3)]
    for f,n in frames[:2]: assert gate.observe(f,n).valid
    replay=replace(frames[2][0],cube_evidence_id=frames[0][0].cube_evidence_id)
    r=gate.observe(replay,frames[2][1])
    assert not r.valid and r.reasons==('REUSED_OBSERVATION_EVIDENCE',)


def test_individually_fresh_but_discontinuous_time_resets_run(validator):
    gate,factory,_=at(validator,'lift_initiation')
    for _ in range(2): assert apply(gate,factory,gap=.003).valid
    factory.index+=20
    r=apply(gate,factory,gap=.003)
    assert r.valid and r.high_count==1 and not r.phase_confirmed
    assert r.sequence_reset_reason=='OBSERVATION_GAP_RESET'


@pytest.mark.parametrize('stage,event',[('initial_support_1',E.BEGIN_LIFT),('initial_support_3',E.BEGIN_PLACE),
    ('attach_2',E.BEGIN_LIFT),('lift_2mm',E.BEGIN_PLACE),('high_3mm_2',E.BEGIN_PLACE),
    ('descend_19',E.RELEASE),('target_strict_support_2',E.RELEASE)])
def test_illegal_phase_jumps_do_not_confirm(validator,stage,event):
    gate,factory,_=at(validator,stage);previous=gate.phase
    site='target' if stage.startswith(('descend','target')) else 'supply'
    gap=.0005 if stage=='descend_19' else 0.
    r=apply(gate,factory,site,gap,event)
    assert not r.valid and not r.phase_confirmed
    assert gate.phase is previous


def test_release_cannot_supply_the_third_support_confirmation(validator):
    gate,factory,_=at(validator,'target_strict_support_2')
    r=apply(gate,factory,'target',0,E.RELEASE,request=('release-x',False),mode=M.RELEASED_SUPPORTED)
    assert not r.valid and 'RELEASE_REQUIRES_PRIOR_CONFIRMED_TARGET_SUPPORT' in r.reasons


def test_invalid_after_support_revokes_release_until_three_fresh_supports(validator):
    gate,factory,_=at(validator,'target_strict_support_3')
    assert gate.can_release
    frame,now=factory.make('target');s=frame.snapshot
    r=gate.observe(replace(frame,snapshot=replace(s,observed=replace(s.observed,left=False))),now)
    assert not r.valid and not gate.can_release
    for i in range(2):
        r=apply(gate,factory,'target'); assert r.valid and not r.can_release
    r=apply(gate,factory,'target'); assert r.can_release
    r=apply(gate,factory,'target',0,E.RELEASE,request=('release-recovered',False),mode=M.RELEASED_SUPPORTED)
    assert r.valid and r.phase is P.RELEASING


def test_invalid_after_airborne_revokes_place_until_three_highs(validator):
    gate,factory,_=at(validator,'high_3mm_3')
    frame,now=factory.make(gap_m=.003)
    r=gate.observe(replace(frame,cube_pose_source='synthetic'),now)
    assert not r.valid
    for i in range(3):
        r=apply(gate,factory,gap=.003)
        assert r.valid and r.phase_confirmed==(i==2)


@pytest.mark.parametrize('key',['supply_roll_0p01_gap_3000um','supply_roll_0p1_gap_3000um'])
def test_rotated_full_geometry_closure_and_no_support_exemption(validator,key):
    gate,factory,_=at(validator,'lift_initiation')
    for i in range(3):
        frame,now=factory.make(gap_m=.003,key=key)
        r=gate.observe(frame,now)
        assert r.valid,r.reasons
        assert r.contact is C.SEPARATED
        assert r.clearance_m==pytest.approx(.003,abs=1e-12)
        assert r.validation.closure.valid and not r.validation.collision.accepted_support_contacts
    assert r.phase is P.AIRBORNE_CONFIRMED


def test_wrong_designated_support_footprint_and_penetration(validator):
    for xyz,code in (((.9,.2,.26),'SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT'),((.65,0,.259998),'CLEARANCE_TABLE_PENETRATION')):
        gate,factory,_=at(validator,'lift_initiation')
        frame,now=factory.make();s=frame.snapshot
        state=replace(s.input_state,cube=replace(s.input_state.cube,position_m=xyz))
        r=gate.observe(replace(frame,snapshot=replace(s,input_state=state)),now)
        assert not r.valid and code in r.reasons


def test_separated_cube_cannot_be_labeled_supported(validator):
    gate,factory,_=at(validator,'lift_initiation')
    frame,now=factory.make(gap_m=.002);s=frame.snapshot
    r=gate.observe(replace(frame,snapshot=replace(s,mode=M.BILATERAL_SUPPORTED,support_site=SupportSite.SUPPLY)),now)
    assert not r.valid and 'SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE' in r.reasons


def test_transitional_mode_at_table_cannot_get_support_exception(validator):
    frame,now=SyntheticFrames(validator.model).make(request=('attach-test',True))
    s=replace(frame.snapshot,mode=M.BILATERAL_TRANSITIONING,support_site=None)
    result=validator.validate(s,now)
    assert not result.valid and not result.collision.accepted_support_contacts
    assert any(reason.startswith('COLLISION:Cube_01') for reason in result.reasons)


@pytest.mark.parametrize('parameters',[dict(d_low_m=0),dict(d_low_m=.004),dict(d_high_m=float('nan')),
    dict(confirmation_frames=2),dict(confirmation_frames=True),dict(d_low_m=True),dict(max_observation_gap_s=1.)])
def test_invalid_research_parameters_rejected(parameters):
    with pytest.raises(ValueError): ClearanceParameters(**parameters)


@pytest.mark.parametrize('change',[lambda f:replace(f,cube_evidence_id=[]),
    lambda f:replace(f,cube_pose_source='external'),
    lambda f:replace(f,snapshot=replace(f.snapshot,observed=replace(f.snapshot.observed,evidence_id=[]))),
    lambda f:replace(f,snapshot=replace(f.snapshot,observed=None))])
def test_malformed_input_returns_rejection_not_exception(validator,change):
    gate=TemporalClearanceConfirmer(validator);frame,now=SyntheticFrames(validator.model).make()
    assert not gate.observe(change(frame),now).valid


def test_external_pose_provenance_not_manufactured_from_synthetic_attachment(validator):
    gate=TemporalClearanceConfirmer(validator);frame,now=SyntheticFrames(validator.model).make()
    assert frame.cube_pose_source is CubePoseSource.SYNTHETIC_TEST_INPUT
    # External pose provenance is a caller declaration for future adapters;
    # it does not change the synthetic attachment source or claim force sensing.
    frame=replace(frame,cube_pose_source=CubePoseSource.EXTERNAL_OBSERVATION)
    r=gate.observe(frame,now)
    assert r.valid and not r.physical_support_observed


def test_provenance_cannot_mix_or_change_during_three_frame_run(validator):
    gate,factory,_=at(validator,'lift_initiation')
    assert apply(gate,factory,gap=.003).high_count==1
    frame,now=factory.make(gap_m=.003)
    r=gate.observe(replace(frame,cube_pose_source=CubePoseSource.EXTERNAL_OBSERVATION),now)
    assert not r.valid and r.reasons==('CUBE_PROVENANCE_CHANGED_MID_SESSION',)
    assert apply(gate,factory,gap=.003).high_count==1


def test_active_parameters_cannot_change_mid_confirmation(validator):
    gate,factory,_=at(validator,'lift_initiation')
    assert apply(gate,factory,gap=.003).high_count==1
    gate.parameters=ClearanceParameters(d_high_m=.002)
    r=apply(gate,factory,gap=.002)
    assert not r.valid and r.reasons==('INVALID_CLEARANCE_PARAMETERS',)


def test_invalid_attempt_identity_cannot_be_reused_as_a_fresh_frame(validator):
    gate=TemporalClearanceConfirmer(validator);frame,now=SyntheticFrames(validator.model).make()
    r=gate.observe(replace(frame,cube_pose_source='invalid'),now)
    assert not r.valid
    r=gate.observe(frame,now)
    assert not r.valid and r.reasons==('REUSED_OBSERVATION_EVIDENCE',)


def test_asynchronous_but_legal_source_keeps_its_actual_validation_clock(validator):
    gate=TemporalClearanceConfirmer(validator);factory=SyntheticFrames(validator.model)
    for _ in range(3):
        frame,now=factory.make();s=frame.snapshot
        frame=replace(frame,snapshot=replace(s,observed=replace(s.observed,stamp_s=now-.015)))
        assert gate.observe(frame,now).valid
    frame,now=factory.make(request=('attach-new',True))
    r=gate.observe(frame,now,E.ATTACH)
    assert r.valid,r.reasons


@pytest.mark.parametrize('kind',['payload_arm','payload_manifold','arm_arm','self','robot_table'])
def test_transition_mode_never_waives_payload_or_robot_collision(validator,kind):
    from test_contact_pipeline import separated_frames
    frames=separated_frames(validator)
    def pose(x,y,z):
        value=np.eye(4);value[:3,3]=[x,y,z];return value
    obj=pose(.65,0,.262)
    if kind=='payload_arm':frames['left_fr3_link0']=pose(.65,0,.21)
    elif kind=='payload_manifold':
        frames['left_fr3_side_suction']=pose(.65,-.138,.342)
        frames['left_fr3_side_suction'][:3,:3]=np.diag([1.,-1.,-1.])
    elif kind=='arm_arm':frames['left_fr3_link7']=frames['right_fr3_link7']=pose(5,0,2)
    elif kind=='self':frames['left_fr3_link0']=frames['left_fr3_link7']=pose(5,0,2)
    else:frames['right_fr3_link0']=pose(.65,0,.05)
    r=validator.collision.validate(frames,obj,M.BILATERAL_TRANSITIONING,None)
    assert not r.valid and r.remaining_collisions and not r.accepted_support_contacts
