from copy import deepcopy
import json
import numpy as np
import pytest
from scipy.linalg import block_diag
from offline.task28_p5_qp_6.common import HERE,NAMES,config
from offline.task28_p5_qp_6.gates import RuntimeGate
from offline.task28_p5_qp_6.model import object_dynamics,contact_maps
from offline.task28_p5_qp_6.controllers import no_contact
from offline.task28_p5_qp_6.coupled import solve
from offline.task28_p5_qp_6.profiles import Profile,full_joint_reference
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p2_1.grasp import GraspModel,metric_matrix


def state_pair():
    m=RobotModel(); ref=Profile('S05').sample(0); q,_,_=full_joint_reference(ref)
    states={}
    for side,q9 in [('left',q[:9]),('right',q[9:])]:
        T=m.fk(side,q9[:7])
        states[side]={'names':NAMES.copy(),'time_s':0.,'step':2,'source_identity':'/World/QP6_'+side,'q':q9,'dq':np.zeros(9),'g':np.zeros(9),'M':np.eye(9),'c':np.zeros(9),'tcp':T,'model_tcp':T,'effort_modes':['force']*9,'drive_kp':np.zeros((1,9)),'drive_kd':np.zeros((1,9))}
    return m,ref,states


@pytest.mark.parametrize('fault',['nan_q','nan_g','wrong_names','async_time','async_step','wrong_articulation','position_drive','nonzero_drive','closure','over_torque','effort_rate','velocity','finger_position'])
def test_precommand_fault_rejection_latches(fault):
    m,ref,s=state_pair(); u=np.zeros(18); previous=np.zeros(18); r=s['right']
    if fault=='nan_q': r['q'][0]=np.nan
    if fault=='nan_g': r['g'][0]=np.nan
    if fault=='wrong_names': r['names']=list(reversed(NAMES))
    if fault=='async_time': r['time_s']=.01
    if fault=='async_step': r['step']=3
    if fault=='wrong_articulation': r['source_identity']='/World/other'
    if fault=='position_drive': r['effort_modes']=['acceleration']*9
    if fault=='nonzero_drive': r['drive_kp'][0,0]=1
    if fault=='closure': r['tcp']=r['tcp'].copy(); r['tcp'][0,3]+=.001
    if fault=='over_torque': u[0]=61
    if fault=='effort_rate': previous[0]=-101
    if fault=='velocity': r['dq'][0]=10
    if fault=='finger_position': r['q'][7]=.05
    gate=RuntimeGate(config(),m); result=gate.check(s,ref,0.,2,u,previous)
    assert not result['valid'] and result['reasons']
    _,ref,restored=state_pair()
    assert gate.check(restored,ref,0.,2)['reasons']==['LATCHED_REJECTION']


@pytest.mark.parametrize('kind',['B','C'])
def test_no_arm_only_inverse_dynamics_bypass(kind):
    m,ref,s=state_pair(); q,dq,_=full_joint_reference(ref)
    with pytest.raises(ValueError,match='SOURCE_BLOCKED_FULL_18_DOF'):
        no_contact(kind,q,dq,np.eye(18),{'g':np.zeros(18),'c':np.zeros(18),'source_full_pass':False},ref,None,config(),m)


def fixture():
    T=np.eye(4); G=GraspModel.frozen(T).matrix(); H,hd=contact_maps(T,np.zeros(6))
    J=block_diag(np.c_[np.eye(6),np.zeros((6,3))],np.c_[np.eye(6),np.zeros((6,3))])
    Mo,ho=object_dynamics(T,np.zeros(6),.8,np.eye(3)*.00192)
    return [np.eye(18),np.zeros(18),J,np.zeros(12),Mo,ho,G,H,hd,np.array([0,0,.1,0,0,0]),metric_matrix(),np.full(54,-100.),np.full(54,100.),np.zeros(18)]


def test_coupled_D_E_abstract_allocation_and_contact_negative():
    args=fixture(); internal=np.zeros(12); internal[0]=internal[6]=.5
    D,dd=solve(*args); E,de=solve(*args,internal_ref=internal)
    assert D is not None and E is not None
    assert max(dd['equality_residual_inf'],de['equality_residual_inf'])<2e-7
    assert np.max(abs(D[48:]-E[48:]))<2e-7
    assert np.linalg.norm(de['internal_prediction']-internal)<np.linalg.norm(dd['internal_prediction']-internal)
    assert de['internal_null_residual']<1e-10
    # Abstract finite restriction: all f=0 and a=0; incompatible with gravity.
    args[9]=np.zeros(6); args[11][48:]=0.; args[12][48:]=0.
    x,d=solve(*args,contact_feasibility=(np.eye(12),np.zeros(12),np.zeros(12)))
    assert x is None and d['status']=='QP_REJECTED'


@pytest.mark.parametrize('fault',['rankG','rankJ','power','metric','mass','nan'])
def test_coupled_source_rank_sign_metric_rejections(fault):
    args=fixture()
    if fault=='rankG': args[6][0]=0
    if fault=='rankJ': args[2][0]=0
    if fault=='power': args[6]*=-1
    if fault=='metric': args[10][0,0]=-1
    if fault=='mass': args[0][0,0]=-1
    if fault=='nan': args[1][0]=np.nan
    x,d=solve(*args)
    assert x is None and 'REJECTED' in d['status']


def test_registration_preserves_all_ten_statuses_and_blocks_sources():
    reg=json.loads((HERE/'formal_registration.json').read_text())
    assert len(reg['cell_eligibility'])==10
    assert not reg['full_dynamics_source_pass']
    assert all(not v['eligible'] for k,v in reg['cell_eligibility'].items() if not k.startswith('A/'))


@pytest.mark.parametrize('kind',['B','C'])
def test_synthetic_only_B_C_full_dynamics_and_no_contact_wrench(kind):
    model=RobotModel(); ref=Profile('S05').sample(.037); q,dq,ddq=full_joint_reference(ref)
    M=np.diag(np.linspace(.5,2,18)); h={'g':np.zeros(18),'c':np.zeros(18),'source_full_pass':True}
    tau,d=no_contact(kind,q,dq,M,h,ref,None,config(),model)
    assert tau.shape==(18,) and d['f_contact'] is None
    assert np.max(np.abs(M@d['qdd_model_prediction']-tau))<2e-7
    if kind=='B': assert np.max(np.abs(tau-M@ddq))<1e-12
    else:
        assert len(d['qp'])==3
        assert all(r['feasibility_inf']<2e-7 for r in d['qp'])
