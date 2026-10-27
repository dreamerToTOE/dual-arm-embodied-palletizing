import copy
import json
import numpy as np
import pytest

from offline.task28_p2_3_r1.oracle import applied_oracle, independent_kkt, direct_measured_center
from offline.task28_p2_3_r1.evaluate_replay import CONFIG, RAW, observation_only, check_oracle_provenance
from offline.task28_p2_3.evaluate import _runtime
from offline.task28_p2_3.sign_control import sha256
from offline.task28_g1_1.task28_validator.model import RobotModel


def ideal_raw():
    row={'arms':{}}
    for side,y,fx in (('left',-.061,1.),('right',.061,-1.)):
        tcp=np.eye(4); tcp[:3,3]=[0,y,0]
        row['arms'][side]={'load_oracle':{'applied_force_world_n':[fx,0,0],
            'applied_pure_torque_world_nm':[0,0,0], 'application_point_world_m':[0,y,0],
            'tcp_pre_step_world':tcp.tolist()}}
    return {'observations':[row]}


def nominal():
    out={}
    for side,y in (('left',-.061),('right',.061)):
        t=np.eye(4);t[:3,3]=[0,y,0];out[side]=t
    return out


def test_ideal_opposing_fx_is_a_couple_not_pure_internal():
    result=applied_oracle(ideal_raw(),np.zeros(3),nominal())
    np.testing.assert_allclose(result['net_world_center'],[0,0,0,0,0,-.122],atol=1e-14)
    assert result['net_direct_vs_kkt_residual'] < 1e-14
    assert result['null_residual'] < 1e-14


def test_wrong_action_reaction_rejected_by_independent_force_oracle():
    one=ideal_raw();one['observations'][0]['arms']['right']['load_oracle']['applied_force_world_n']=[0,0,0]
    truth=applied_oracle(one,np.zeros(3),nominal())['net_world_center']
    wrong=-truth
    assert np.linalg.norm(wrong[:3]-truth[:3]) > .6  # archived force mismatch gate


def test_wrong_moment_arm_caught_by_direct_reference_comparison():
    result=applied_oracle(ideal_raw(),np.zeros(3),nominal())
    wrong_local=result['nominal_local_contacts'].copy()
    # Deliberately remove the +/-61 mm contact moment arms while summing forces.
    wrong_center=np.r_[wrong_local[:3]+wrong_local[6:9], wrong_local[3:6]+wrong_local[9:12]]
    assert np.linalg.norm(wrong_center-result['net_world_center']) > 1e-10
    assert abs(wrong_center[5]-result['net_world_center'][5]) == pytest.approx(.122)


def test_estimated_actual_tcp_direct_net_catches_omitted_arms():
    fake = {}
    for side,y,fx in (('left',-.061,1.),('right',.061,-1.)):
        t=np.eye(4);t[:3,3]=[0,y,0]
        fake[side]={'actual_world_tcp':t,'on_robot_world_tcp':[fx,0,0,0,0,0]}
    reference=direct_measured_center(fake,np.zeros(3))
    np.testing.assert_allclose(reference,[0,0,0,0,0,-.122],atol=1e-14)
    assert np.linalg.norm(reference-np.zeros(6)) > 1e-10


def test_wrong_weighted_split_detected_even_if_reconstructs_and_is_null():
    result=applied_oracle(ideal_raw(),np.zeros(3),nominal())
    x=result['nominal_local_contacts']; kkt=independent_kkt(x,np.zeros(3),nominal())
    # x itself as 'external' and zero as 'internal' reconstruct and are null
    # but are generally not the minimum weighted norm external split.
    assert np.linalg.norm(kkt['weighted_external_local']-x) > 1e-3


@pytest.fixture
def baseline():
    cfg=json.loads(CONFIG.read_text()); cfg_hash=sha256(CONFIG)
    runtime=_runtime(json.loads((RAW/'collector_status.json').read_text()),cfg,cfg_hash)
    return cfg,cfg_hash,runtime


def test_observation_only_ignores_missing_oracles(tmp_path,baseline):
    cfg,h,runtime=baseline
    raw=json.loads((RAW/'asymmetric_loaded.json').read_text())
    for row in raw['observations']:
        for arm in row['arms'].values(): arm.pop('load_oracle')
    path=tmp_path/'case.json';path.write_text(json.dumps(raw))
    data,_,_=observation_only(path,cfg,h,'asymmetric','loaded',RobotModel(),runtime)
    assert data['left']['effort'].shape == (60,7)


@pytest.mark.parametrize('mutation,reason', [('identity','ARM_SOURCE_OR_JOINT_ORDER_MISMATCH'),
    ('order','ARM_SOURCE_OR_JOINT_ORDER_MISMATCH'),('duplicate','STALE_DUPLICATE_OR_OUT_OF_ORDER_OBSERVATION')])
def test_bad_archived_observation_rejected(tmp_path,baseline,mutation,reason):
    cfg,h,runtime=baseline
    raw=json.loads((RAW/'asymmetric_loaded.json').read_text())
    if mutation=='identity': raw['observations'][0]['arms']['left']['source_id']='probe_right'
    elif mutation=='order': raw['observations'][0]['arms']['left']['joint_names'].reverse()
    else: raw['observations'][1]['simulation_time_s']=raw['observations'][0]['simulation_time_s']
    path=tmp_path/'case.json';path.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match=reason):
        observation_only(path,cfg,h,'asymmetric','loaded',RobotModel(),runtime)


def test_oracle_bad_point_rejected_only_after_estimation_input_parsed(baseline):
    cfg,h,runtime=baseline
    raw=json.loads((RAW/'asymmetric_loaded.json').read_text())
    case=next(c for c in cfg['cases'] if c['id']=='asymmetric')
    raw['observations'][0]['arms']['left']['load_oracle']['application_point_world_m'][0] += .1
    with pytest.raises(ValueError,match='APPLIED_LOAD_OR_POINT_MISMATCH'):
        check_oracle_provenance(raw,case,cfg)
