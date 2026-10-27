"""Independent central-difference controls: offline algebra, no commands."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from offline.task28_p5_qp_2.feedback import (FeedbackConfig, feedback_maps,
    log_rotation, pose_feedback_terms)
from offline.task28_p5_qp_0.bootstrap import RobotModel, nominal_grasps

ROOT = Path(__file__).resolve().parents[3]


def X(R, p):
    out = np.eye(4); out[:3, :3] = R; out[:3, 3] = p; return out


def independent_inverse(T):
    return X(T[:3, :3].T, -T[:3, :3].T @ T[:3, 3])


def rot(v):
    return Rotation.from_rotvec(v).as_matrix()


def independent_log(R):
    return Rotation.from_matrix(R).as_rotvec()


def hat(x):
    return np.array([[0.,-x[2],x[1]],[x[2],0.,-x[0]],[-x[1],x[0],0.]])


def jl(phi):
    t = np.linalg.norm(phi); P = hat(phi)
    if t < 1e-6: return np.eye(3)+.5*P+(P@P)/6
    return np.eye(3)+(1-np.cos(t))/t**2*P+(t-np.sin(t))/t**3*(P@P)


def evolved(T,v,w,h):
    return X(rot(h*w) @ T[:3, :3], T[:3, 3]+h*v)


def test_actual_center_jacobians_against_fk_finite_difference():
    model = RobotModel(); grasps = nominal_grasps()
    control = json.loads((ROOT/'offline/task28_g1_1/tests/control_states.json').read_text())['states'][1]
    q = np.r_[control['left_q'],control['right_q']]
    poses, maps = feedback_maps(model,q,grasps)
    rng=np.random.default_rng(280201); h=1e-6; worst=0.
    for side, offset, G, center, B in zip(('left','right'),(0,7),(grasps.left,grasps.right),poses,maps):
        for _ in range(5):
            direction=rng.normal(size=7); base=q[offset:offset+7]
            plus=model.fk(side,base+h*direction) @ independent_inverse(G)
            minus=model.fk(side,base-h*direction) @ independent_inverse(G)
            dp=(plus[:3,3]-minus[:3,3])/(2*h)
            dR=(plus[:3,:3]-minus[:3,:3])/(2*h) @ center[:3,:3].T
            skew=.5*(dR-dR.T); omega=np.array([skew[2,1],skew[0,2],skew[1,0]])
            error=np.max(np.abs(np.r_[dp,omega]-B@direction)); worst=max(worst,error)
    print(json.dumps({'qp2_actual_center_fk_fd_max_abs':worst,'seed':280201,'cases':10,'h':h}))
    assert worst < 2e-8


def test_relative_log_differential_and_feedback_direction_noncommuting():
    rng=np.random.default_rng(280202); cfg=FeedbackConfig(1.3,1.7,.8,.9); h=1e-6
    errors=[]; convergence=[]; wrong_sign=[]
    for _ in range(100):
        right=Rotation.random(random_state=rng).as_matrix()
        phi=rng.normal(size=3); phi*=rng.uniform(.001,2.8)/np.linalg.norm(phi)
        left=rot(phi)@right; pL=rng.normal(size=3); pR=rng.normal(size=3)
        XL,XR=X(left,pL),X(right,pR); wr=rng.normal(size=3); wl=rng.normal(size=3)
        vl=rng.normal(size=3); vr=rng.normal(size=3)
        terms=pose_feedback_terms(XL,XR,np.eye(4),np.zeros(6),cfg)
        A=terms['relative_rotation_matrix']; phi=terms['relative_rotation_log']
        f=lambda s: independent_log((rot(s*wl)@left)@(rot(s*wr)@right).T)
        numerical=(f(h)-f(-h))/(2*h)
        predicted=np.linalg.solve(jl(phi),wl-A@wr)
        errors.append(np.max(np.abs(numerical-predicted)))
        # Pick commands satisfying each production affine equation and compare
        # independently measured pose-error derivatives to the required decay.
        target_w=A@wr+terms['b'][3:]; target_v=vr+terms['b'][:3]
        def rel(s,w,v):
            l=evolved(XL,v,w,s); r=evolved(XR,vr,wr,s)
            return np.r_[l[:3,3]-r[:3,3],independent_log(l[:3,:3]@r[:3,:3].T)]
        derivative=(rel(h,target_w,target_v)-rel(-h,target_w,target_v))/(2*h)
        expected=np.r_[-cfg.relative_translation_gain_s*(pL-pR),-cfg.relative_rotation_gain_s*phi]
        convergence.append(np.max(np.abs(derivative-expected)))
        wrong=(rel(h,A@wr-terms['b'][3:],vr-terms['b'][:3])-rel(-h,A@wr-terms['b'][3:],vr-terms['b'][:3]))/(2*h)
        wrong_sign.append(np.linalg.norm(wrong-expected))
    print(json.dumps({'qp2_relative_log_fd_max_abs':max(errors),'qp2_relative_feedback_fd_max_abs':max(convergence),'wrong_sign_min_direction_difference':min(wrong_sign),'seed':280202,'cases':100,'h':h}))
    assert max(errors)<2e-8 and max(convergence)<2e-8
    assert min(wrong_sign)>1e-3


def test_noncoaxial_world_reference_rotation_feedback_finite_difference():
    rng=np.random.default_rng(280203); cfg=FeedbackConfig(1.,1.,.8,1.1); h=1e-6; errors=[]
    for _ in range(100):
        current=Rotation.random(random_state=rng).as_matrix()
        psi=rng.normal(size=3); psi*=rng.uniform(.01,2.5)/np.linalg.norm(psi)
        ref=rot(psi)@current
        p=rng.normal(size=3); p_ref=rng.normal(size=3)
        vref=rng.normal(size=3); wref=rng.normal(size=3)
        desired=np.r_[vref,wref]; obj=X(current,p); target=X(ref,p_ref)
        terms=pose_feedback_terms(obj,obj,target,desired,cfg)
        vi,wi=terms['targets'][0][:3],terms['targets'][0][3:]
        def err(s):
            r=evolved(target,vref,wref,s); o=evolved(obj,vi,wi,s)
            return np.r_[r[:3,3]-o[:3,3],independent_log(r[:3,:3]@o[:3,:3].T)]
        derivative=(err(h)-err(-h))/(2*h)
        e0=err(0); expected=np.r_[-cfg.object_translation_gain_s*e0[:3],-cfg.object_rotation_gain_s*e0[3:]]
        errors.append(np.max(np.abs(derivative-expected)))
    print(json.dumps({'qp2_reference_feedback_fd_max_abs':max(errors),'seed':280203,'cases':100,'h':h}))
    assert max(errors)<2e-8


def test_pi_and_invalid_inputs_rejected():
    with pytest.raises(ValueError,match='SO3_LOG_NEAR_PI_UNSUPPORTED'):
        log_rotation(rot([math.pi,0,0]))
    with pytest.raises(ValueError): FeedbackConfig(relative_translation_gain_s=-1.)
    with pytest.raises(ValueError): FeedbackConfig(object_rotation_gain_s=float('nan'))
    with pytest.raises(ValueError): pose_feedback_terms(np.eye(4),np.eye(4),np.eye(4),[float('nan')]*6,FeedbackConfig())


def test_frozen_seed_controller_affine_and_unchanged_tracking_gate():
    import gzip
    from offline.task28_p5_qp_0.core import VelocityInput
    from offline.task28_p5_qp_2.feedback import FeedbackVelocityQP
    with gzip.open(ROOT/'offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz','rt') as stream:
        saved=json.load(stream)
    obj,q=map(np.asarray,saved['transport'][len(saved['transport'])//2])
    controller=FeedbackVelocityQP(feedback_config=FeedbackConfig(.2,.2,.2,.2))
    desired=np.array([.005,-.003,.002,.01,-.005,.02])
    result=controller.solve(VelocityInput(q,obj,desired,.01))
    assert result.status=='FEASIBLE_TRACKING_OK', result.reasons
    x=np.asarray(result.qdot); diag=result.diagnostics
    A=np.asarray(diag['affine_A']); b=np.asarray(diag['affine_b'])
    assert np.linalg.norm(A@x-b,np.inf)<1e-8
    error=np.asarray(diag['B'])@x-desired
    assert np.linalg.norm(error[:3])<=1e-4 and np.linalg.norm(error[3:])<=1e-3
    assert abs(np.linalg.norm(error[:3])-diag['tracking_translation_error_m_s'])<1e-14
    assert not result.execution_authorized
    assert not diag['collision_checked_in_core']
    wrong=controller.solve(VelocityInput(q,obj,desired,.01,cube_id='Cube_99'))
    assert wrong.status=='INPUT_REJECTED' and wrong.qdot is None
    invalid=controller.solve(VelocityInput(np.r_[q[:13],np.nan],obj,desired,.01))
    assert invalid.status=='INPUT_REJECTED' and invalid.qdot is None
