"""Actual frozen FK poses plus isolated negative FCL diagnostics."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from task28_validator.state import AttachmentMode,StatePolicy
from task28_validator.validator import ClosedChainValidator
from task28_contact.examples import load_sequence,input_state
from task28_contact.modes import OperationMode,SupportSite
from task28_contact.validator import ContactModeValidator


@pytest.fixture(scope='module')
def gate():return ContactModeValidator()


@pytest.fixture
def sequence(gate):return load_sequence(gate.model)[0]


def object_pose(snapshot):
    from task28_validator.se3 import pose_matrix
    cube=snapshot.input_state.cube
    return pose_matrix(cube.position_m,cube.quaternion_xyzw)


def test_actual_frozen_initial_target_and_all_modes_pass(gate,sequence):
    acm=gate.collision.reference.allowed_pairs
    for row in sequence:
        result=gate.validate(row.snapshot,row.now_s)
        assert result.valid,result.reasons
        assert result.collision.reference.checked_shape_pairs==414
        assert result.collision.reference.skipped_acm_shape_pairs==40
        if row.snapshot.mode is OperationMode.BILATERAL_AIRBORNE:
            assert result.collision.reference.valid
            assert not result.collision.accepted_support_contacts
            assert result.collision.airborne_clearance_m==pytest.approx(.14)
        else:
            assert not result.collision.reference.valid  # unchanged G1-1 still detects table touching
            assert result.collision.support.valid
            assert result.collision.support.bottom_gap_m==pytest.approx(0,abs=1e-14)
            assert len(result.collision.accepted_support_contacts)==1
            assert len(result.collision.accepted_support_contacts[0]['record']['contacts'])==4
        assert (result.closure is not None)==(row.snapshot.mode in (OperationMode.BILATERAL_SUPPORTED,OperationMode.BILATERAL_AIRBORNE))
    assert gate.collision.reference.allowed_pairs==acm
    assert sequence[0].snapshot.input_state.cube.position_m==(.65,0.,.26)
    assert sequence[-1].snapshot.input_state.cube.position_m==(.82,-.075,.26)


def test_unchanged_airborne_reference_retains_supported_rejection(gate,sequence):
    old=ClosedChainValidator(gate.model,gate.collision.reference)
    assert not old.validate(sequence[1].snapshot.input_state,sequence[1].now_s).valid
    for row in sequence[2:4]:
        assert old.validate(row.snapshot.input_state,row.now_s).valid
        assert gate.validate(row.snapshot,row.now_s).valid


def test_detached_home_skips_closure_but_preserves_other_gates(gate,sequence):
    row=sequence[0]
    configuration={'object_position_m':[.65,0,.26],'object_yaw_rad':0.,'left_q':list(gate.model.home),'right_q':list(gate.model.home)}
    state=input_state(gate.model,configuration,9.995,False,False,AttachmentMode.DETACHED)
    snapshot=replace(row.snapshot,input_state=state)
    result=gate.validate(snapshot,row.now_s)
    assert result.valid,result.reasons
    assert result.closure is None
    moved=replace(state.left_tcp,position_m=(state.left_tcp.position_m[0]+.01,*state.left_tcp.position_m[1:]))
    result=gate.validate(replace(snapshot,input_state=replace(state,left_tcp=moved)),row.now_s)
    assert not result.valid and 'LEFT_FK_OBSERVATION_MISMATCH' in result.reasons


@pytest.mark.parametrize('change,expected',[
    (lambda s:replace(s.input_state.cube,position_m=(.65,0,.24)),'SUPPORT_TABLE_PENETRATION'),
    (lambda s:replace(s.input_state.cube,position_m=(.65,0,.259998)),'SUPPORT_TABLE_PENETRATION'),
    (lambda s:replace(s.input_state.cube,position_m=(.90,.20,.26)),'SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT'),
])
def test_supported_mode_does_not_open_every_cube_table_contact(gate,sequence,change,expected):
    row=sequence[0]
    snapshot=replace(row.snapshot,input_state=replace(row.snapshot.input_state,cube=change(row.snapshot)))
    result=gate.validate(snapshot,row.now_s)
    assert not result.valid and expected in result.reasons
    assert not result.collision.accepted_support_contacts
    assert any('Cube_01/box|task28_table/box' in reason for reason in result.reasons)


def test_airborne_cannot_claim_supported_or_intersecting_cube(gate,sequence):
    row=sequence[1]
    for dz in (0.,-.02,.0000005):
        cube=replace(row.snapshot.input_state.cube,position_m=(.65,0,.26+dz))
        snapshot=replace(row.snapshot,mode=OperationMode.BILATERAL_AIRBORNE,support_site=None,input_state=replace(row.snapshot.input_state,cube=cube))
        result=gate.validate(snapshot,row.now_s)
        assert not result.valid and 'AIRBORNE_CLEARANCE_NOT_ESTABLISHED' in result.reasons
        assert not result.collision.accepted_support_contacts


def test_release_while_unsupported_is_rejected(gate,sequence):
    row=sequence[-1]
    snapshot=replace(row.snapshot,input_state=replace(row.snapshot.input_state,cube=replace(row.snapshot.input_state.cube,position_m=(.82,-.075,.40))))
    result=gate.validate(snapshot,row.now_s)
    assert not result.valid and 'SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE' in result.reasons


def test_bilateral_flags_and_feedback_are_not_commands(gate,sequence):
    row=sequence[1]
    candidates=[replace(row.snapshot,input_state=replace(row.snapshot.input_state,left_attached=False,right_attached=False)),
                replace(row.snapshot,observed=replace(row.snapshot.observed,left=False,right=False)),
                replace(row.snapshot,observed=None)]
    for snapshot in candidates:
        result=gate.validate(snapshot,row.now_s)
        assert not result.valid and result.collision is None


def test_stale_and_async_state_or_attachment_feedback_rejected(gate,sequence):
    row=sequence[2]
    candidates=[replace(row.snapshot,input_state=replace(row.snapshot.input_state,cube=replace(row.snapshot.input_state.cube,stamp_s=9.5))),
                replace(row.snapshot,input_state=replace(row.snapshot.input_state,right_joints=replace(row.snapshot.input_state.right_joints,stamp_s=9.97))),
                replace(row.snapshot,observed=replace(row.snapshot.observed,stamp_s=9.5))]
    for snapshot in candidates:
        result=gate.validate(snapshot,row.now_s)
        assert not result.valid and result.collision is None


def test_invalid_airborne_full_closure_rejected(gate,sequence):
    row=sequence[2]
    cube=replace(row.snapshot.input_state.cube,position_m=(.66,0,.40))
    result=gate.validate(replace(row.snapshot,input_state=replace(row.snapshot.input_state,cube=cube)),row.now_s)
    assert not result.valid and 'LEFT_GRASP_TRANSLATION_MISMATCH' in result.reasons
    assert result.closure.left_object_translation_error_m==pytest.approx(.01)
    assert result.collision is not None


def test_limits_still_gate_both_detached_and_bilateral(gate,sequence):
    for row in sequence[:2]:
        q=list(row.snapshot.input_state.left_joints.positions_rad);q[0]=float(gate.model.joint_upper[0]+.001)
        state=replace(row.snapshot.input_state,left_joints=replace(row.snapshot.input_state.left_joints,positions_rad=tuple(q)))
        result=gate.validate(replace(row.snapshot,input_state=state),row.now_s)
        assert not result.valid and result.reasons==('LEFT_JOINT_LIMIT_VIOLATION',)


def separated_frames(gate):
    frames={}
    for index,name in enumerate(sorted(gate.collision.reference.required_links)):
        matrix=np.eye(4);matrix[:3,3]=[10+2*index,0,2];frames[name]=matrix
    return frames


@pytest.mark.parametrize('kind',['payload_arm','payload_manifold','arm_arm','self','robot_table'])
def test_support_exception_does_not_hide_other_collisions(gate,kind):
    frames=separated_frames(gate);obj=np.eye(4);obj[:3,3]=[.65,0,.26]
    def pose(x,y,z):
        value=np.eye(4);value[:3,3]=[x,y,z];return value
    if kind=='payload_arm':frames['left_fr3_link0']=pose(.65,0,.21)
    elif kind=='payload_manifold':
        frames['left_fr3_side_suction']=pose(.65,-.138,.34);frames['left_fr3_side_suction'][:3,:3]=np.diag([1.,-1.,-1.])
    elif kind=='arm_arm':frames['left_fr3_link7']=frames['right_fr3_link7']=pose(5,0,2)
    elif kind=='self':frames['left_fr3_link0']=frames['left_fr3_link7']=pose(5,0,2)
    elif kind=='robot_table':frames['right_fr3_link0']=pose(.65,0,.05)
    result=gate.collision.validate(frames,obj,OperationMode.BILATERAL_SUPPORTED,SupportSite.SUPPLY)
    assert not result.valid
    assert len(result.accepted_support_contacts)==1
    assert result.remaining_collisions
    assert any('Cube_01/box|task28_table/box' not in reason for reason in result.reasons)


def test_backend_failure_is_never_reclassified_as_support(gate,monkeypatch,tmp_path):
    monkeypatch.setattr(gate.collision.reference,'backend_path',tmp_path/'missing')
    obj=np.eye(4);obj[:3,3]=[.65,0,.26]
    result=gate.collision.validate(separated_frames(gate),obj,OperationMode.DETACHED_SUPPORTED,SupportSite.SUPPLY)
    assert not result.valid and not result.accepted_support_contacts
    assert any(reason.startswith('COLLISION_BACKEND_FAILURE') for reason in result.reasons)


def test_forged_policy_cannot_weaken_detached_fk_gate(sequence):
    gate=ContactModeValidator();gate.closure_policy=SimpleNamespace(translation_tolerance_m=float('nan'),rotation_tolerance_rad=float('nan'))
    result=gate.validate(sequence[0].snapshot,sequence[0].now_s)
    assert not result.valid and result.reasons==('INVALID_CONTACT_MODE_POLICY',)


def test_support_policy_tampering_rejects_airborne_too(sequence):
    gate=ContactModeValidator();object.__setattr__(gate.collision.support,'PLANE_TOLERANCE_M',float('nan'))
    result=gate.validate(sequence[2].snapshot,sequence[2].now_s)
    assert not result.valid and 'SUPPORT_POLICY_CONFIGURATION_CHANGED' in result.reasons
