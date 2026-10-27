"""Offline controls for the independent evaluator; no simulated sensor claims."""
from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p2_3.evaluate import (CONFIG, R1, COLLECTOR, ROOT, _live_check, _oracle,
                                          _runtime, evaluate, sha256)


def cfg(): return json.loads(CONFIG.read_text())


def runtime_fixture():
    c=cfg(); prior=json.loads(R1.read_text())['runtime']
    arms={}
    for side in ('left','right'):
        arms[side]={k:deepcopy(prior[k]) for k in ('all_dof_names','ordered_joint_names','ordered_dof_indices',
            'dof_stiffness','dof_damping','asset_physx_articulation_properties')}
        arms[side].update(articulation_path='/World/Probe_'+side,link7_path='/World/Probe_'+side+'/fr3_link7',
            base_world_position_m=c['base_world_m'][side],base_world_orientation_wxyz=[1,0,0,0])
    runtime={'arms':arms,'observation_source':'SingleArticulation.get_measured_joint_efforts',
             'force_application_api':prior['force_application_api'],'force_frame':'world','torque_frame':'world',
             'physics_dt_s':prior['physics_dt_s'],'physics_gravity':prior['physics_gravity']}
    return {'status':'COLLECTION_COMPLETE_NOT_EVALUATED','source_kind':'ISAAC_TWO_ARTICULATIONS_STANDALONE',
       'config_sha256':sha256(CONFIG),'collector_sha256':sha256(COLLECTOR),'asset_sha256':c['fixture_usd_sha256'],
       'g1_source_state_sha256':sha256(ROOT/'offline/task28_g1_1/tests/control_states.json'),
       'completed_condition_files':[f"{x['id']}_{kind}.json" for x in c['cases'] for kind in ('unloaded','loaded')],
       'actual_physics_steps':10818,'runtime':runtime}


def test_missing_collector_cannot_pass_and_preserves_all_case_ids(tmp_path):
    result=evaluate(tmp_path)
    assert result['status']=='INCOMPLETE' and result['global_failures']
    assert len(result['cases']) == 9 and all(x['status']=='INCOMPLETE' for x in result['cases'])
    assert result['execution_authorized'] is False and result['physical_contact_measured'] is False


@pytest.mark.parametrize('mutation', ['wrong_gains','wrong_arm','wrong_order','wrong_asset','wrong_frame'])
def test_actual_source_identity_and_r1_runtime_reject(mutation):
    c=cfg(); status=runtime_fixture()
    _runtime(status,c,sha256(CONFIG))
    if mutation=='wrong_gains': status['runtime']['arms']['right']['dof_stiffness'][0]+=1
    if mutation=='wrong_arm': status['runtime']['arms']['right']['articulation_path']='/World/Probe_left'
    if mutation=='wrong_order': status['runtime']['arms']['right']['ordered_dof_indices'].reverse()
    if mutation=='wrong_asset': status['asset_sha256']='0'*64
    if mutation=='wrong_frame': status['runtime']['force_frame']='local'
    with pytest.raises(ValueError): _runtime(status,c,sha256(CONFIG))


@pytest.mark.parametrize('side', ['left','right'])
def test_live_link7_fk_checks_actual_observed_pose_and_tcp(side):
    c=cfg(); m=RobotModel(); q=np.asarray(c['state'][side+'_q']); frames=m._chain(side,q)[0]
    link=frames[side+'_fr3_link7']; tcp=m.fk(side,q)
    quat=Rotation.from_matrix(link[:3,:3]).as_quat()[[3,0,1,2]]
    a={'link7_position_world_m':link[:3,3].tolist(),'link7_orientation_world_wxyz':quat.tolist(),
       'q_rad':q.tolist(),'tcp_world':tcp.tolist()}
    raw={'observations':[{'arms':{side:a}}]}; status=runtime_fixture()
    status['runtime']['arms'][side]['link7_to_frozen_virtual_tcp']=(np.linalg.inv(link)@tcp).tolist()
    _, errors=_live_check(raw,side,m,status['runtime'],c)
    assert errors['max_translation_m'] < 1e-10
    corrupted=deepcopy(raw); corrupted['observations'][0]['arms'][side]['link7_position_world_m'][0]+=.01
    with pytest.raises(ValueError): _live_check(corrupted,side,m,status['runtime'],c)


def record(force, point, pure=(0,0,0)):
    a={'load_oracle':{'applied_force_world_n':force,'applied_pure_torque_world_nm':pure,
       'application_point_world_m':point,'tcp_pre_step_world':np.eye(4).tolist()}}
    return {'observations':[{'arms':{'left':a}}]}


def test_independent_force_and_offset_moment_signs():
    center=np.array([.65,0,.4]); point=np.array([.65,-.061,.4]); f=[1.,0,0]
    robot, cube=_oracle(record(f,point), 'left',center)
    np.testing.assert_allclose(robot[:3], [1,0,0])
    np.testing.assert_allclose(cube, [-1,0,0,0,0,-.061], atol=1e-15)


def test_all_preregistered_cases_are_required(tmp_path):
    changed=cfg(); changed['cases']=changed['cases'][:-1]
    f=tmp_path/'bad.json'; f.write_text(json.dumps(changed))
    result=evaluate(tmp_path,f)
    assert 'FORMAL_PREREGISTRATION_HASH_OR_MATRIX_MISMATCH' in result['global_failures']
    assert result['status']=='INCOMPLETE'


def test_independent_weighted_kkt_internal_force_and_net():
    from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
    from offline.task28_p2_3.evaluate import _independent_weighted_oracle
    obj=np.eye(4); obj[:3,3]=[.65,0,.4]; nominal=nominal_grasps()
    arms={}
    for side,force in (('left',[0,-1,0]),('right',[0,1,0])):
        point=(obj@getattr(nominal,side))[:3,3]
        arms[side]={'load_oracle':{'applied_force_world_n':force,'applied_pure_torque_world_nm':[0,0,0],
                    'application_point_world_m':point.tolist()}}
    got=_independent_weighted_oracle({'observations':[{'arms':arms}]},obj,nominal)
    np.testing.assert_allclose(got['net_world_center'],0.,atol=1e-12)
    np.testing.assert_allclose(got['weighted_external_local'],0.,atol=1e-12)
    np.testing.assert_allclose(got['weighted_internal_local'],got['local_on_cube'],atol=1e-12)
    assert got['null_residual'] < 1e-12
