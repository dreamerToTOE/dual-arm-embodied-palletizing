from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from offline.task28_p5_qp_5_r2.analysis import ROOT, HERE, build_diagnostics, verify_sources
from offline.task28_p5_qp_5_r2.calculations import angle, closure_diagnostics, sample_original, validate_row


@pytest.fixture(scope='module')
def result():
    return build_diagnostics()


@pytest.fixture(scope='module')
def steps():
    return json.loads((ROOT/'offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json').read_text())['positive']['raw_qp2']['steps']


def test_all_frames_and_first_failure(result):
    original=result['archives']['task28_p5_qp_5']
    r1=result['archives']['task28_p5_qp_5_r1']
    assert len(original['rows']) == 75 and len(r1['rows']) == 2
    assert original['summary']['first_geometry_rejection']['index']==2
    assert r1['summary']['first_geometry_rejection']['index']==1
    first=r1['rows'][1]
    assert first['physics_step_index']==244
    assert first['closure']['metrics']['left_absolute_translation_m']==pytest.approx(.00015406338914891624,abs=1e-12)
    assert first['closure']['metrics']['relative_translation_m']<.0001
    assert first['archived_runtime_gate']['reasons']==['LEFT_ABSOLUTE_TRANSLATION']


def test_independent_reference_interval_and_endpoint(steps):
    for s in steps:
        t=(s['time_s']+s['target_time_s'])/2
        q,dq,_=sample_original(steps,t)
        np.testing.assert_allclose(q,(np.asarray(s['q_initial'])+s['q_next'])/2,atol=1e-14,rtol=0)
        np.testing.assert_array_equal(dq,s['qdot14'])
    q,dq,obj=sample_original(steps,.1)
    np.testing.assert_array_equal(q,steps[-1]['q_next'])
    np.testing.assert_array_equal(dq,np.zeros(14))
    np.testing.assert_array_equal(obj,steps[-1]['desired_object_pose_next'])


@pytest.mark.parametrize('time',[-1.,float('nan'),float('inf')])
def test_bad_reference_time(steps,time):
    with pytest.raises(ValueError,match='INVALID_TIME'):
        sample_original(steps,time)


def test_initial_and_final_discontinuity(steps,result):
    jumps=result['reference']['velocity_jumps']
    assert len(jumps)==6
    assert jumps[0]['inf_rad_s']>.036 and jumps[-1]['inf_rad_s']>.036
    assert max(j['inf_rad_s'] for j in jumps[1:-1])<.000022
    # At .1 the left derivative remains nonzero but right derivative is zero.
    q_before,_,_=sample_original(steps,.1-1e-7)
    q_at,_,_=sample_original(steps,.1)
    q_after,_,_=sample_original(steps,.1+1e-7)
    assert np.max(np.abs((q_at-q_before)/1e-7))>.036
    np.testing.assert_array_equal(q_after,q_at)
    assert result['reference']['acceleration_status']=='UNAVAILABLE_NO_VALID_TIME_PARAMETERIZATION'


def test_kinematic_reference_pass_is_not_moving_physics_pass(result):
    assert all(r['closure']['valid'] for r in result['reference']['sampled_reference_geometry'])
    assert not result['archives']['task28_p5_qp_5_r1']['rows'][-1]['closure']['valid']


def fixture_tcps():
    # Explicit analytic fixture, independent of production desired_tcps().
    left=np.eye(4);left[:3,:3]=[[0,1,0],[1,0,0],[0,0,-1]];left[:3,3]=[0,-.061,0]
    right=np.eye(4);right[:3,:3]=[[0,-1,0],[-1,0,0],[0,0,-1]];right[:3,3]=[0,.061,0]
    return left,right


def test_common_translation_control():
    l,r=fixture_tcps();l[0,3]+=.0002;r[0,3]+=.0002
    d=closure_diagnostics(np.eye(4),l,r)
    assert d['common_translation_m']==pytest.approx(.0002)
    assert d['differential_translation_m']==pytest.approx(0.)
    assert not d['valid']  # relative closure alone must not pass absolute grasp


def test_opposite_translation_control():
    l,r=fixture_tcps();l[0,3]+=.00006;r[0,3]-=.00006
    d=closure_diagnostics(np.eye(4),l,r)
    assert d['common_translation_m']==pytest.approx(0.)
    assert d['differential_translation_m']==pytest.approx(.00012)
    assert d['metrics']['left_absolute_translation_m']<.0001
    assert not d['valid']


def test_rotation_and_world_frame_invariance():
    l,r=fixture_tcps();world=np.eye(4);world[:3,:3]=Rotation.from_rotvec([.2,.3,-.1]).as_matrix();world[:3,3]=[.3,-.2,.1]
    d=closure_diagnostics(world,world@l,world@r)
    assert d['valid'] and d['metrics']['relative_rotation_rad']<1e-12
    assert angle(Rotation.from_rotvec([0,0,np.pi]).as_matrix())==pytest.approx(np.pi)


def test_effort_reconstruction_and_mixed_units(result):
    for archive in result['archives'].values():
        for row in archive['rows']:
            for arm in row['arms'].values():
                assert arm['raw_reconstruction_error_arm_Nm']<1e-12
                assert arm['readback_error_arm_Nm']<1e-5
                assert not arm['saturated'] and not arm['rate_limited']
                assert arm['effort_units']==['Nm']*7+['N']*2
                assert len(arm['P_feedback'])==9 and len(arm['q_error_rad'])==7


def test_r1_gravity_finger_field_has_PD_logging_contamination(result):
    for row in result['archives']['task28_p5_qp_5_r1']['rows']:
        for arm in row['arms'].values():
            np.testing.assert_allclose(arm['gravity_field_minus_inferred'][:7],0,atol=1e-12,rtol=0)
            np.testing.assert_allclose(arm['gravity_field_minus_inferred'][7:],arm['P_feedback'][7:],atol=1e-12,rtol=0)
    assert any(np.max(np.abs(a['P_feedback'][7:]))>1e-7 for r in result['archives']['task28_p5_qp_5_r1']['rows'] for a in r['arms'].values())


@pytest.mark.parametrize('index',range(7))
def test_fault_injection_rejected(result,index):
    c=result['negative_controls'][index]
    assert c['passed'] and c['actual_reason']==c['expected_reason']


def test_missing_source_rejected(tmp_path):
    with pytest.raises(ValueError,match='MISSING_SOURCE'):
        verify_sources(tmp_path)


def test_tampered_source_rejected(tmp_path):
    first=next(iter(json.loads((HERE/'source_binding.json').read_text())['files']))
    p=tmp_path/first;p.parent.mkdir(parents=True);p.write_text('tampered')
    with pytest.raises(ValueError,match='SOURCE_HASH_MISMATCH'):
        verify_sources(tmp_path)


def test_dynamics_unavailable_at_required_state(result):
    d=result['dynamic_availability']
    assert d['M_c_missing_in_QP5_and_R1'] and d['r3_has_M_c_g']
    assert min(d['r3_min_posture_difference_inf_rad'].values())>.5
    assert not d['physical_bounds_authenticated']
    assert result['derived_reference_identity'] is None and not result['new_physics_run']
