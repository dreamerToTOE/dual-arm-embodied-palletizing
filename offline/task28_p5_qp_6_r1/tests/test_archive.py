from copy import deepcopy
import json
import numpy as np
import pytest
from offline.task28_p5_qp_6_r1.common import HERE,config
from offline.task28_p5_qp_6_r1.dynamics import constrained_prediction
from offline.task28_p5_qp_6_r1.evaluate import active_face_oracle
from offline.task28_p5_qp_6_r1.safety import check
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6.profiles import Profile


@pytest.mark.parametrize('seed',range(5))
def test_constraint_QP_against_independent_active_faces(seed):
    rng=np.random.default_rng(seed); A=rng.normal(size=(9,9)); M=A.T@A+.1*np.eye(9)
    q=np.zeros(9);q[7:]=.04 if seed%2 else .02; dq=np.zeros(9); tau=rng.normal(size=9)*.01
    actual,_=constrained_prediction(M,np.zeros(9),tau,q,dq,.01,[0,0],[.04,.04])
    oracle=active_face_oracle(M,np.zeros(9),tau,q,dq,.01,[0,0],[.04,.04])
    assert actual is not None and oracle is not None
    assert np.max(abs(actual-oracle))<2e-6


@pytest.mark.parametrize('cfg,steps,status',[('A',19,'REJECTED'),('B',30,'COMPLETE'),('C',140,'REJECTED'),('D',20,'COMPLETE')])
def test_independent_source_counts_and_immediate_stop(cfg,steps,status):
    ev=json.loads((HERE/f'evidence/independent_v2/configuration_{cfg}.json').read_text())
    assert ev['controlled_steps']==steps and ev['status']==status
    assert ev['first_failure_is_last_recorded_step']
    assert ev['maximum_K_oracle_difference_inf']<2e-6
    raw=json.loads((HERE/f'evidence/source_{cfg}_v2/raw.json').read_text())['rows']
    assert len(raw)==steps and all(r['executed'] for r in raw)
    assert all(r['post_safety_gate']['valid'] for r in raw)
    if status=='REJECTED':
        assert not raw[-1]['registered_model_gate']['valid']
        assert all(r['registered_model_gate']['valid'] for r in raw[:-1])


@pytest.mark.parametrize('fault',['names','time','step','nan','torque','rate','drive','geometry'])
def test_precommand_common_rejection(fault):
    r=json.loads((HERE/'evidence/source_C_v2/raw.json').read_text())['rows'][0]
    states=deepcopy(r['pre']);u=np.r_[r['arms']['left']['proposed_effort'],r['arms']['right']['proposed_effort']];prev=u.copy(); obj=Profile('S05').sample(0)['pose']
    if fault=='names': states['right']['names']=list(reversed(states['right']['names']))
    if fault=='time': states['right']['time_s']+=.01
    if fault=='step': states['right']['physics_step']+=1
    if fault=='nan': states['right']['g'][0]=np.nan
    if fault=='torque': u[0]=61
    if fault=='rate': prev[0]-=101
    if fault=='drive': states['right']['drive_kp'][0][0]=1
    if fault=='geometry': states['right']['tcp'][0][3]+=.001
    t=states['left']['time_s'];step=states['left']['physics_step']
    g=check(states,obj,t,step,config(),RobotModel(),u,prev)
    assert not g['valid'] and g['reasons']


def test_no_interior_or_arm_only_promotion_and_no_formal_steps():
    q=json.loads((HERE/'evidence/independent_v2/source_qualification.json').read_text())
    assert not q['formal_eligible']
    assert q['source_configuration_statuses']['B']['status']=='COMPLETE'
    assert q['source_configuration_statuses']['D']['status']=='COMPLETE'
    assert q['source_configuration_statuses']['C']['status']=='REJECTED'
    cells=json.loads((HERE/'evidence/independent_v2/formal_matrix.json').read_text())
    assert len(cells)==4 and all(r['status']=='SKIPPED_WITH_REASON' and r['moving_steps']==0 for r in cells)


def test_historical_failure_and_new_C_failure_are_separate():
    old=json.loads((HERE/'evidence/forensic_v1/summary.json').read_text())['first_failure']
    assert (old['row'],old['side'],old['physics_step_post'])==(18,'right',21)
    assert np.max(abs(np.array(old['observed_qdd'])[7:]+.0044159387))<1e-9
    new=json.loads((HERE/'evidence/independent_v2/configuration_C.json').read_text())['first_failure']
    assert (new['row'],new['side'],new['physics_step_post'])==(139,'right',142)
    assert not new['R_independent']['checks']['CONSTRAINT_RESIDUAL_STRUCTURE']
    assert all(v for k,v in new['R_independent']['checks'].items() if k!='CONSTRAINT_RESIDUAL_STRUCTURE')
