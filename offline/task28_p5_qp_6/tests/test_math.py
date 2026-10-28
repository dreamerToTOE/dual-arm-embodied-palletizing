"""Independent analytic and finite-difference controls, no physics required."""
import itertools
import numpy as np
import pytest
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation
from offline.task28_p5_qp_6.model import (skew,contact_maps,object_dynamics,coupled_equalities,virtual_centres,centre_jdot,local_contact_jacobian)
from offline.task28_p5_qp_6.solver import qp,hqp
from offline.task28_p5_qp_6.profiles import Profile,time_law,full_joint_reference
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps,evaluate_closure
from offline.task28_p2_1.grasp import GraspModel,metric_matrix


def test_action_reaction_1d_and_gravity():
    # 2 kg robot coordinate pushing 3 kg payload upwards at 1 m/s2.
    # Object: f=3*(1+9.81); robot requires +f as reaction compensation.
    f=32.43; tau=2*(1+9.81)+f
    assert tau==pytest.approx(54.05)
    assert (tau-f-2*9.81)/2==pytest.approx(1.)
    M,h=object_dynamics(np.eye(4),np.zeros(6),3,np.eye(3))
    assert M[2,2]+h[2]==pytest.approx(f)


@pytest.mark.parametrize('angle',[0.,.4,-1.2])
def test_virtual_power_and_grasp_analytic(angle):
    T=np.eye(4); T[:3,:3]=Rotation.from_rotvec([angle,.2,0]).as_matrix()
    v=np.array([.3,-.1,.2,.7,-.4,.6]); f=np.arange(12)*.23
    H,_=contact_maps(T,v); G=GraspModel.frozen(T).matrix()
    # Separately form contact point velocity and rotate local wrench to world.
    total=0.; force=np.zeros(3); moment=np.zeros(3)
    for k,grip in enumerate((nominal_grasps().left,nominal_grasps().right)):
        r=T[:3,:3]@grip[:3,3]; R=T[:3,:3]@grip[:3,:3]
        ff=R@f[k*6:k*6+3]; mm=R@f[k*6+3:k*6+6]
        force+=ff; moment+=np.cross(r,ff)+mm
        total+=(v[:3]+np.cross(v[3:],r))@ff+v[3:]@mm
    assert np.allclose(G@f,np.r_[force,moment],atol=1e-14)
    assert np.allclose(G,H.T,atol=1e-14)
    assert v@(G@f)==pytest.approx(total,abs=1e-14)


def test_contact_acceleration_rotating_axes_fd():
    # Independent d/dt local point velocity, with evolving translation/rotation.
    T=np.eye(4); T[:3,:3]=Rotation.from_rotvec([.3,-.2,.1]).as_matrix()
    V=np.array([.5,-.3,.7,.4,.8,-.2]); a=np.array([.2,.1,-.4,-.3,.2,.1]); dt=1e-6
    H,bias=contact_maps(T,V)
    values=[]
    for sign in [-1,1]:
        R=Rotation.from_rotvec(sign*dt*V[3:]+.5*dt**2*a[3:]).as_matrix()@T[:3,:3]
        vv=V+sign*dt*a; contacts=[]
        for grip in [nominal_grasps().left,nominal_grasps().right]:
            r=R@grip[:3,3]; Ri=R@grip[:3,:3]
            contacts.extend(np.r_[Ri.T@(vv[:3]+np.cross(vv[3:],r)),Ri.T@vv[3:]])
        values.append(np.array(contacts))
    assert np.max(np.abs((values[1]-values[0])/(2*dt)-(H@a+bias)))<1e-9
    assert np.linalg.norm(bias)>0.1  # Dropping Hdot is a falsifiable negative.


def test_reference_point_shift_hand_oracle():
    g=GraspModel.frozen(np.eye(4)); f=np.array([1,0,0,0,0,0.])
    assert np.allclose(g.contact_to_center('left',g.world_to_contact('left',f)),[1,0,0,0,0,.061])
    assert np.allclose(g.center_to_contact('left',g.contact_to_center('left',f)),f)


def test_object_com_offset_and_gyroscopic_terms():
    T=np.eye(4); I=np.diag([.2,.3,.4]); V=np.array([.1,.2,.3,1.,2.,3.])
    M,h=object_dynamics(T,V,2,I,com_body=[.1,0,0]); a=np.array([1.,-.2,.3,.5,.6,.7])
    c=np.array([.1,0,0]); force=2*(a[:3]+np.cross(a[3:],c)+np.cross(V[3:],np.cross(V[3:],c))-[0,0,-9.81])
    torque=I@a[3:]+np.cross(V[3:],I@V[3:])+np.cross(c,force)
    assert np.allclose(M@a+h,np.r_[force,torque])
    assert np.linalg.eigvalsh(M).min()>0


@pytest.mark.parametrize('bad',[0.,-1.,np.nan])
def test_invalid_mass(bad):
    with pytest.raises(ValueError): object_dynamics(np.eye(4),np.zeros(6),bad,np.eye(3))


def test_internal_metric_independent_kkt():
    g=GraspModel.frozen(np.eye(4)); G=g.matrix(); W=metric_matrix(); W[0,0]*=3
    f=np.linspace(-2,3,12); required=G@f
    saddle=np.block([[W,G.T],[G,np.zeros((6,6))]])
    external=np.linalg.solve(saddle,np.r_[np.zeros(12),required])[:12]
    dec=g.decompose(f,W)
    assert np.max(np.abs(external-dec.external))<1e-12
    assert np.linalg.norm(G@dec.internal)<1e-12
    assert abs(external@W@dec.internal)<1e-11
    assert np.linalg.matrix_rank(G)==6
    assert np.linalg.svd(G@np.diag(1/np.sqrt(np.diag(W))),compute_uv=False)[-1]>.01


def test_strict_lexicographic_not_weighted_sum():
    # P1 x+y=1, P2 x-y=0. P3 wants x=y=100; must retain x=y=.5.
    x,r=hqp(2,np.empty((0,2)),[],np.eye(2),[-2,-2],[2,2],[(np.array([[1,1]]),[1]),(np.array([[1,-1]]),[0]),(np.eye(2),[100,100])])
    assert x is not None and np.max(np.abs(x-.5))<1e-7
    assert all(d['stationarity_inf']<2e-7 for d in r)


@pytest.mark.parametrize('P,c,A,l,u',[(np.eye(1),[0],[[1]],[2],[1]),(-np.eye(1),[0],[[1]],[-1],[1]),(np.eye(1),[0],[[np.nan]],[-1],[1])])
def test_invalid_qp_never_returns_command(P,c,A,l,u):
    x,_=qp(P,c,A,l,u); assert x is None


def test_explicit_infeasible_hard_physics():
    # x=0 and x>=1 cannot be softened even if task weight is enormous.
    x,_=hqp(1,[[1]],[0],[[1]],[1],[2],[(np.array([[1e9]]),[1e9])]); assert x is None


@pytest.mark.parametrize('seed',range(5))
def test_qp_independent_active_face_enumeration(seed):
    rng=np.random.default_rng(seed); B=rng.normal(size=(3,3)); P=B.T@B+np.eye(3); c=rng.normal(size=3)*4
    x,_=qp(P,c,np.eye(3),[-1]*3,[1]*3); candidates=[]
    for face in itertools.product([-1,0,1],repeat=3):
        z=np.array(face,dtype=float); free=np.flatnonzero(np.array(face)==0); fixed=np.flatnonzero(np.array(face)!=0)
        if len(free): z[free]=np.linalg.solve(P[np.ix_(free,free)],-c[free]-P[np.ix_(free,fixed)]@z[fixed])
        if np.all(np.abs(z)<=1+1e-9): candidates.append((.5*z@P@z+c@z,z))
    oracle=min(candidates,key=lambda v:v[0])[1]
    assert x is not None and np.max(np.abs(x-oracle))<1e-7


def test_full_coupled_model_feasible_and_infeasible():
    T=np.eye(4); G=GraspModel.frozen(T).matrix(); H,hd=contact_maps(T,np.zeros(6))
    M=np.diag(np.linspace(1,2,18)); h=np.linspace(-1,1,18)
    J=block_diag(np.c_[np.eye(6),np.zeros((6,3))],np.c_[np.eye(6),np.zeros((6,3))])
    Mo,ho=object_dynamics(T,np.zeros(6),.8,np.eye(3)*.00192)
    E,b=coupled_equalities(M,h,J,np.zeros(12),Mo,ho,G,H,hd)
    task=np.zeros((6,54)); task[:,48:]=np.eye(6)
    x,r=hqp(54,E,b,np.eye(54),[-100]*54,[100]*54,[(task,[0,0,.2,0,0,0]),(np.eye(54),np.zeros(54))])
    assert x is not None and np.max(np.abs(E@x-b))<2e-7
    assert np.linalg.norm(G@x[36:48]-(Mo@x[48:]+ho))<2e-7
    assert np.linalg.norm(M@x[:18]+h-x[18:36]+J.T@x[36:48])<2e-7
    assert np.linalg.matrix_rank(E)==36
    # Forbid all f and object acceleration: gravity equilibrium impossible.
    C=np.zeros((18,54)); C[:12,36:48]=np.eye(12); C[12:,48:]=np.eye(6)
    x,_=hqp(54,E,b,C,np.zeros(18),np.zeros(18),[(task,np.zeros(6))]); assert x is None


@pytest.mark.parametrize('name',['S05','S10'])
def test_profile_c2_exact_knots_and_derivatives(name):
    p=Profile(name)
    for k in range(6):
        r=p.sample(k*p.segment_s)
        assert np.allclose(r['pose'],p.poses[k],atol=1e-15)
        assert np.max(np.abs(r['twist']))<1e-14 and np.max(np.abs(r['acceleration']))<1e-12
        assert np.max(np.abs(r['dq14']))<1e-14 and np.max(np.abs(r['qdd14']))<1e-12
    t=.037; eps=1e-6; a=p.sample(t-eps); b=p.sample(t+eps); r=p.sample(t)
    assert np.max(np.abs((b['q14']-a['q14'])/(2*eps)-r['dq14']))<1e-9
    assert np.max(np.abs((b['dq14']-a['dq14'])/(2*eps)-r['qdd14']))<1e-7
    assert np.max(np.abs((b['twist']-a['twist'])/(2*eps)-r['acceleration']))<1e-7
    omega=Rotation.from_matrix(b['pose'][:3,:3]@a['pose'][:3,:3].T).as_rotvec()/(2*eps)
    assert np.max(np.abs(omega-r['twist'][3:]))<1e-9


def test_actual_virtual_jacobian_and_jdot_pose_fd():
    model=RobotModel(); q,dq,_=full_joint_reference(Profile('S05').sample(.037)); eps=1e-4
    poses,B=virtual_centres(model,q); bias=centre_jdot(model,q,dq)
    plus,_=virtual_centres(model,q+eps*dq); minus,_=virtual_centres(model,q-eps*dq)
    for i,d in enumerate([dq[:9],dq[9:]]):
        vel=np.r_[(plus[i][:3,3]-minus[i][:3,3])/(2*eps),Rotation.from_matrix(plus[i][:3,:3]@minus[i][:3,:3].T).as_rotvec()/(2*eps)]
        assert np.max(np.abs(B[i]@d-vel))<1e-9
        acc=(plus[i][:3,3]-2*poses[i][:3,3]+minus[i][:3,3])/eps**2
        assert np.max(np.abs(acc-bias[i][:3]))<1e-7
    J,jd=local_contact_jacobian(model,q,dq,Profile('S05').sample(.037)['pose'])
    assert J.shape==(12,18) and np.isfinite(jd).all()
    assert np.max(np.abs(J[:,[7,8,16,17]]))==0


def test_local_robot_contact_jdot_independent_pose_acceleration():
    model=RobotModel(); q,dq,_=full_joint_reference(Profile('S05').sample(.037)); eps=1e-4
    J,jd=local_contact_jacobian(model,q,dq,np.eye(4))
    for i,(side,part,dpart) in enumerate([('left',q[:7],dq[:7]),('right',q[9:16],dq[9:16])]):
        poses=[model.fk(side,part+k*eps*dpart) for k in [-2,-1,0,1,2]]
        minus,zero,plus=poses[1:4]; R=zero[:3,:3]
        velocity=(plus[:3,3]-minus[:3,3])/(2*eps)
        omega=Rotation.from_matrix(plus[:3,:3]@minus[:3,:3].T).as_rotvec()/(2*eps)
        linear_acc=(plus[:3,3]-2*zero[:3,3]+minus[:3,3])/eps**2
        wp=Rotation.from_matrix(poses[4][:3,:3]@zero[:3,:3].T).as_rotvec()/(2*eps)
        wm=Rotation.from_matrix(zero[:3,:3]@poses[0][:3,:3].T).as_rotvec()/(2*eps)
        alpha=(wp-wm)/(2*eps)
        oracle=np.r_[R.T@(linear_acc-np.cross(omega,velocity)),R.T@alpha]
        assert np.max(np.abs(oracle-jd[i*6:(i+1)*6]))<1e-7


@pytest.mark.parametrize('u',[-.1,1.1,np.nan])
def test_time_domain_rejection(u):
    with pytest.raises(ValueError): time_law(u)
