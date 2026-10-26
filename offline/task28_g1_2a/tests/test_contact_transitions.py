"""Endpoint and mode guards only: never an interpolated trajectory test."""
from dataclasses import replace
import pytest
from task28_contact.examples import load_sequence
from task28_contact.modes import TransitionEvent
from task28_contact.validator import ContactModeValidator


@pytest.fixture(scope='module')
def gate():return ContactModeValidator()


@pytest.fixture
def sequence(gate):return load_sequence(gate.model)[0]


def transition(gate,before,after,event):
    return gate.validate_transition(before.snapshot,after.snapshot,event,before.now_s,after.now_s)


def test_all_five_declared_a_stage_events_pass(gate,sequence):
    events=load_sequence(gate.model)[1]
    for before,after,event in zip(sequence,sequence[1:],events):
        result=transition(gate,before,after,event)
        assert result.valid,result.reasons
        assert result.source.valid and result.target.valid
        assert result.scope=='MODE_AND_ENDPOINT_GUARDS_ONLY_NO_EDGE_SAFETY'


@pytest.mark.parametrize('first,last,event',[(0,2,TransitionEvent.LIFT),(3,5,TransitionEvent.RELEASE),(1,3,TransitionEvent.PLACE),(5,1,TransitionEvent.ATTACH)])
def test_illegal_mode_jumps_rejected_even_when_each_snapshot_valid(gate,sequence,first,last,event):
    result=transition(gate,sequence[first],sequence[last],event)
    assert result.source.valid and result.target.valid
    assert not result.valid and result.reasons==('ILLEGAL_MODE_TRANSITION',)


def test_attach_needs_a_new_explicit_request(gate,sequence):
    before,after=sequence[:2]
    intent=replace(after.snapshot.intent,request_id=before.snapshot.intent.request_id)
    observed=replace(after.snapshot.observed,request_id=intent.request_id)
    after=replace(after,snapshot=replace(after.snapshot,intent=intent,observed=observed))
    result=transition(gate,before,after,TransitionEvent.ATTACH)
    assert result.source.valid and result.target.valid
    assert not result.valid and 'MISSING_EXPLICIT_ATTACHMENT_REQUEST' in result.reasons


def test_attach_cannot_reuse_the_old_feedback_identity(gate,sequence):
    before,after=sequence[:2]
    after=replace(after,snapshot=replace(after.snapshot,observed=replace(after.snapshot.observed,evidence_id=before.snapshot.observed.evidence_id)))
    result=transition(gate,before,after,TransitionEvent.ATTACH)
    assert result.target.valid
    assert not result.valid and 'REUSED_ATTACHMENT_EVIDENCE' in result.reasons


def test_attach_command_must_follow_source_observation(gate,sequence):
    before,after=sequence[:2]
    after=replace(after,snapshot=replace(after.snapshot,intent=replace(after.snapshot.intent,issued_at_s=9.99)))
    result=transition(gate,before,after,TransitionEvent.ATTACH)
    assert result.target.valid
    assert not result.valid and 'ATTACH_RELEASE_CAUSAL_ORDER' in result.reasons


def test_attach_cannot_hide_a_small_joint_jump_inside_looser_fk_tolerance(gate,sequence):
    before,after=sequence[:2]
    state=after.snapshot.input_state;q=list(state.left_joints.positions_rad);q[0]+=.000002
    after=replace(after,snapshot=replace(after.snapshot,input_state=replace(state,left_joints=replace(state.left_joints,positions_rad=tuple(q)))))
    result=transition(gate,before,after,TransitionEvent.ATTACH)
    assert result.target.valid,result.target.reasons
    assert not result.valid and 'LEFT_JOINT_MOVED_DURING_ATTACH_RELEASE' in result.reasons


def test_attach_cannot_move_supported_cube_across_the_numerical_band(gate,sequence):
    before,after=sequence[:2]
    def shifted(row,dx):
        state=row.snapshot.input_state;cube=replace(state.cube,position_m=(.65+dx,0,.26))
        return replace(row,snapshot=replace(row.snapshot,input_state=replace(state,cube=cube)))
    result=transition(gate,shifted(before,-.0000009),shifted(after,.0000009),TransitionEvent.ATTACH)
    assert result.source.valid and result.target.valid
    assert not result.valid and 'OBJECT_MOVED_DURING_ATTACH_RELEASE' in result.reasons


def test_lift_and_place_require_vertical_endpoint_alignment(gate,sequence):
    for before,after,event in ((sequence[1],sequence[3],TransitionEvent.LIFT),(sequence[2],sequence[4],TransitionEvent.PLACE)):
        result=transition(gate,before,after,event)
        assert result.source.valid and result.target.valid
        assert not result.valid and 'LIFT_PLACE_NOT_VERTICAL_ENDPOINTS' in result.reasons


def test_transport_cannot_silently_replace_latched_attachment_request(gate,sequence):
    before,after=sequence[2:4]
    intent=replace(after.snapshot.intent,request_id='unannounced-attach')
    observed=replace(after.snapshot.observed,request_id=intent.request_id)
    after=replace(after,snapshot=replace(after.snapshot,intent=intent,observed=observed))
    result=transition(gate,before,after,TransitionEvent.TRANSPORT_SNAPSHOT)
    assert result.target.valid
    assert not result.valid and 'ATTACHMENT_INTENT_CHANGED_WITHOUT_ATTACH_RELEASE' in result.reasons


def test_transition_requires_new_synchronized_observations_and_clock(gate,sequence):
    before,after=sequence[2:4];stamp=before.snapshot.observed.stamp_s
    fields={key:replace(getattr(after.snapshot.input_state,key),stamp_s=stamp) for key in ('left_joints','right_joints','left_tcp','right_tcp','cube')}
    state=replace(after.snapshot.input_state,**fields)
    after=replace(after,now_s=before.now_s,snapshot=replace(after.snapshot,input_state=state,observed=replace(after.snapshot.observed,stamp_s=stamp)))
    result=transition(gate,before,after,TransitionEvent.TRANSPORT_SNAPSHOT)
    assert result.target.valid
    assert not result.valid and 'OBSERVATIONS_NOT_ADVANCED' in result.reasons and 'NONINCREASING_SNAPSHOT_CLOCK' in result.reasons


def test_release_requires_target_site_not_teleport_from_supply(gate,sequence):
    result=transition(gate,sequence[1],sequence[5],TransitionEvent.RELEASE)
    assert result.source.valid and result.target.valid
    assert not result.valid and 'INCORRECT_ATTACH_RELEASE_SITE' in result.reasons


def test_release_request_and_independent_feedback_must_be_explicit(gate,sequence):
    before,after=sequence[4:6]
    after=replace(after,snapshot=replace(after.snapshot,intent=replace(after.snapshot.intent,request_id=before.snapshot.intent.request_id),
                                        observed=replace(after.snapshot.observed,request_id=before.snapshot.intent.request_id)))
    result=transition(gate,before,after,TransitionEvent.RELEASE)
    assert result.target.valid
    assert not result.valid and 'MISSING_EXPLICIT_ATTACHMENT_REQUEST' in result.reasons


def test_release_while_unsupported_cannot_pass_as_a_transition(gate,sequence):
    before,after=sequence[4:6];state=after.snapshot.input_state
    after=replace(after,snapshot=replace(after.snapshot,input_state=replace(state,cube=replace(state.cube,position_m=(.82,-.075,.4)))))
    result=transition(gate,before,after,TransitionEvent.RELEASE)
    assert not result.valid and any(reason=='TARGET_STATE:SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE' for reason in result.reasons)


def test_event_string_cannot_impersonate_typed_transition(gate,sequence):
    result=transition(gate,sequence[0],sequence[1],'attach')
    assert not result.valid and 'INVALID_TRANSITION_EVENT' in result.reasons
