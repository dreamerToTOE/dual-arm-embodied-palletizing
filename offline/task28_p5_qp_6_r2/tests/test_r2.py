import json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p5_qp_6_r2.common import DOF_NAMES, HERE
from offline.task28_p5_qp_6_r2.scene_identity import inspect_static
from offline.task28_p5_qp_6_r2.torque_adapter import (AdapterRejected, ActuatorOwnership,
    coordinated_effort, synchronized_effort_messages, validate_effort_payload, validate_snapshot)
from offline.task28_p5_qp_6_r2.suction_interface import SuctionStateInterface


def sample():
    return dict(names=DOF_NAMES, effort_modes=['force']*9, drive_kp=[0]*9,drive_kd=[0]*9,
                q=[0]*9,dq=[0]*9,M=np.eye(9),c=[0]*9,g=[0]*9,time_s=0.,physics_step=0)


def test_scene_source_and_runtime_prims():
    assert inspect_static()['status']=='SCENE_IDENTITY_VERIFIED'
    d=json.loads((HERE/'evidence/runtime_scene_v2.json').read_text())
    assert d['status']=='SCENE_IDENTITY_VERIFIED' and d['physics_steps']==0 and d['robot_commands']==0
    assert d['source_hashes_match'] and d['scene']['physics']['time_steps_per_second']==60
    for s,sign in [('left',-1),('right',1)]:
        t=d['scene']['tools'][s]
        assert np.allclose(t['local_translation_m'],[0,sign*.155,.08]) and t['local_rz_deg']==sign*90
        assert np.allclose(t['vertical_scale_m'],[.028,.028,.08],atol=1e-7)
        assert np.allclose(t['lateral_scale_m'],[.018,.13,.018],atol=1e-7)
        assert np.allclose(t['manifold_scale_m'],[.07,.014,.07],atol=1e-7)
        assert len(t['cups'])==4


def test_local_transform_is_not_object_grasp():
    from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps, ClosurePolicy
    g=nominal_grasps()
    assert np.allclose(g.left[:3,3],[0,-.061,0])
    assert np.allclose(g.right[:3,3],[0,.061,0])
    assert ClosurePolicy().translation_tolerance_m==.0001 and ClosurePolicy().rotation_tolerance_rad==.001


def test_torque_never_activates_with_unqualified_source_or_position_sink():
    with pytest.raises(AdapterRejected,match='POSITION_SINK_ACTIVE'):
        ActuatorOwnership(position_sink_active=True).activate_torque()
    with pytest.raises(AdapterRejected,match='ARM_DYNAMICS_SOURCE_UNQUALIFIED'):
        ActuatorOwnership().activate_torque()
    forged=ActuatorOwnership(True,False,'task28_qp6_r2_torque_adapter')
    with pytest.raises(AdapterRejected,match='ARM_DYNAMICS_SOURCE_UNQUALIFIED'):
        coordinated_effort(sample(),np.zeros(7),ownership=forged)


@pytest.mark.parametrize('mutation', ['name','nan','mode','gain','time','matrix'])
def test_reject_bad_snapshot(mutation):
    x=sample()
    if mutation=='name': x['names']=list(reversed(DOF_NAMES))
    if mutation=='nan': x['q'][1]=float('nan')
    if mutation=='mode': x['effort_modes'][0]='position'
    if mutation=='gain': x['drive_kp']=[0]*8
    if mutation=='time': x['time_s']=1.
    if mutation=='matrix': x['M'][0,0]=-1
    with pytest.raises(AdapterRejected): validate_snapshot(x,expected_time=0)


def test_effort_message_never_contains_position():
    p=synchronized_effort_messages(np.zeros(9),np.ones(9),stamp=(0,1))
    assert validate_effort_payload(p)['valid']
    assert 'position' not in p['left'] and 'position' not in p['right']
    p['position_field_present']=True
    with pytest.raises(AdapterRejected): validate_effort_payload(p)


def test_suction_contract_and_runtime_lock():
    s=SuctionStateInterface(); assert s.validate_contract()['valid']
    assert not s.request_state(False,False).left_closed
    with pytest.raises(RuntimeError,match='NOT_AUTHORIZED'): s.request_state(True,True)


def test_target_frames_use_g1_and_reject_actual_preapproach():
    from offline.task28_p5_qp_6_r2.targets import desired_suction_frames, frozen_closure_gate
    d=json.loads((HERE/'evidence/runtime_scene_v2.json').read_text())['scene']
    cube=np.eye(4); cube[:3,3]=np.asarray(d['cube_world_transform_usd_row_vector'])[3,:3]
    desired=desired_suction_frames(cube)
    assert np.allclose(desired['left'][:3,3],[.65,-.061,.26])
    assert np.allclose(desired['right'][:3,3],[.65,.061,.26])
    actual={s:np.asarray(d['tools'][s]['tcp_world_transform_usd_row_vector']).T for s in ['left','right']}
    result=frozen_closure_gate(cube,actual['left'],actual['right'])
    assert not result.valid and result.left_object_translation_error_m > .7
    assert frozen_closure_gate(cube,desired['left'],desired['right']).valid
