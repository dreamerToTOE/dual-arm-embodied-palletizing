import json
from pathlib import Path
import numpy as np
import pytest
from research.stage_b_multisensor_r3.offline_cube_estimator import ConnectionEstimator,OfflineCubeEstimator

CFG=json.loads((Path(__file__).parents[1]/'config/p2b1_estimator_frozen.json').read_text())


def robot(t=0.,x=0.):
    T=np.eye(4);T[0,3]=x
    return {'time_s':t,'tcp_fk':{'left':T.tolist()},'suction_observation':{'left':True}}


def visual(t=0.,x=.2,interval=None):
    return {'cube_pose':{'status':'PRESENT','cube_xyz_m':[x,0.,.26]},'image_epoch_accepted':True,
        'admissible_interval_s':interval or [t,t],'sdk_time_s':t}


def test_software_closed_and_matching_motion_do_not_prove_connection():
    c=ConnectionEstimator(CFG)
    assert c.observe([0.,0.],[[0.,0.]],True)['state']=='UNKNOWN'
    assert c.observe(None,[],True)['state']=='UNKNOWN'


def test_rigid_relation_contradiction_requires_repeated_independent_visual_evidence():
    c=ConnectionEstimator(CFG)
    assert c.observe([.1,0.],[[0.,0.]],True)['state']=='UNKNOWN'
    assert c.observe([.1,0.],[[0.,0.]],True)['state']=='DISCONNECTED'


def test_k1_is_explicitly_conditional_while_f1_suspends_rigid_constraint():
    for method in ('K1','F1'):
        e=OfflineCubeEstimator(method,CFG);e.step(robot(),[visual()],[robot()])
        p=e.step(robot(1.,.1),[],[robot(),robot(1.,.1)])
        if method=='K1':
            assert p['xy_m'][0]==pytest.approx(.3) and p['connection']['state']=='CONNECTED'
            assert p['connection']['reason'].startswith('EXPLICIT_INITIAL')
        else:
            assert p['xy_m'][0]==pytest.approx(.2) and p['rigid_constraint']=='SUSPENDED'
            assert p['connection']['state']=='UNKNOWN'


def test_unknown_before_anchor_and_no_future_vision_no_reset_leakage():
    e=OfflineCubeEstimator('F1',CFG)
    assert e.step(robot(),[],[])['status']=='UNKNOWN'
    with pytest.raises(ValueError):e.step(robot(),[visual(1.)],[robot()])
    e=OfflineCubeEstimator('F1',CFG);e.step(robot(),[visual()],[robot()])
    e.step(robot(1.),[],[robot()])
    with pytest.raises(ValueError):e.step(robot(.5),[],[robot()])


def test_time_interval_remains_uncertain_covariance_psd_and_older_observation_is_not_now():
    e=OfflineCubeEstimator('F1',CFG);e.step(robot(),[visual()],[robot()])
    p=e.step(robot(2.),[visual(2.,interval=[0.,1.])],[robot(),robot(1.)])
    assert p['measurement_time_s'] is None and p['observation_age_interval_s']==[1.,2.]
    assert min(np.linalg.eigvalsh(p['covariance_xy_m2']))>=0 and not p['online_fusion_eligible']


def test_gt_injection_rejected_and_no_camera_never_implies_attachment():
    e=OfflineCubeEstimator('F1',CFG)
    with pytest.raises(ValueError):e.step(dict(robot(),cube_gt=[0,0,0]),[],[])
    for i in range(20):assert e.step(robot(i),[],[])['status']=='UNKNOWN'


def test_delayed_measurement_operator_does_not_reuse_velocity_as_position_feedback():
    from research.stage_b_multisensor_r3.offline_cube_estimator_v0 import OfflineCubeEstimator as Failed
    errors=[]
    for cls in (Failed,OfflineCubeEstimator):
        estimator=cls('F1',CFG);history=[];error=[]
        for i in range(301):
            t=i/15.;dx=.015*(1-np.cos(2*np.pi*max(0,t-2)/9));r=robot(t,dx);history.append(r)
            p=estimator.step(r,[visual(t,.2+dx,[0.,t])],history)
            error.append(abs(p['xy_m'][0]-(.2+dx)))
        errors.append(max(error))
    assert errors[0]>.15 and errors[1]<.05
