"""Geometric centre twists [v_W; omega_W], local contact wrenches ON Cube.

a is the ordinary world time derivative of that geometric twist, not the
linear component of a Lie-algebra spatial acceleration about world origin.
All full-articulation matrices retain 18 mixed revolute/prismatic coordinates.
"""
import numpy as np
from scipy.linalg import block_diag
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
from offline.task28_g1_1.task28_validator.se3 import checked_transform, inverse
from offline.task28_p2_1.grasp import GraspModel, metric_matrix


def vector(x, n):
    a = np.asarray(x, float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError('INVALID_FINITE_VECTOR')
    return a


def skew(x):
    x,y,z = vector(x,3)
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def contact_maps(T, V):
    """H maps centre world geometric velocity to local TCP velocity.

Local components rotate with the object. Hdot*V must therefore include
the rotating-axes term; it is not merely a world centripetal acceleration.
"""
    T = checked_transform(T); V = vector(V,6)
    hs, ds = [], []
    for grip in (nominal_grasps().left, nominal_grasps().right):
        r = T[:3,:3] @ grip[:3,3]
        R = T[:3,:3] @ grip[:3,:3]
        axes = block_diag(R.T,R.T)
        hs.append(axes @ np.block([[np.eye(3),-skew(r)],[np.zeros((3,3)),np.eye(3)]]))
        # d(axes)/dt contributes -omega x velocity at contact.
        world_convective = np.r_[np.cross(V[3:],np.cross(V[3:],r)),np.zeros(3)]
        vc = np.r_[V[:3]+np.cross(V[3:],r),V[3:]]
        ds.append(axes @ (world_convective-block_diag(skew(V[3:]),skew(V[3:]))@vc))
    return np.vstack(hs),np.concatenate(ds)


def object_dynamics(T,V,mass,inertia_com_body,gravity=(0.,0.,-9.81),com_body=(0.,0.,0.)):
    """General COM offset, about the declared Cube centre, world components.

Momentum derivative has origin-acceleration term and gyroscopic/centripetal
bias. Gravity is placed on the LHS as compensation h; env excludes gravity.
"""
    T=checked_transform(T); V=vector(V,6); c=T[:3,:3]@vector(com_body,3)
    Ib=np.asarray(inertia_com_body,float)
    if not np.isfinite(mass) or mass<=0 or Ib.shape!=(3,3) or not np.isfinite(Ib).all() or not np.allclose(Ib,Ib.T) or np.linalg.eigvalsh(Ib).min()<=0:
        raise ValueError('INVALID_OBJECT_INERTIA')
    I=T[:3,:3]@Ib@T[:3,:3].T; w=V[3:]; g=vector(gravity,3)
    M=np.block([[mass*np.eye(3),-mass*skew(c)],[mass*skew(c),I-mass*skew(c)@skew(c)]])
    bias_force=mass*(np.cross(w,np.cross(w,c))-g)
    h=np.r_[bias_force,np.cross(c,bias_force)+np.cross(w,I@w)]
    return M,h


def coupled_equalities(M,h,J,jdot,Mobj,hobj,G,H,hdot,environment=None):
    """x=[qdd18,tau18,f_local12,a_obj6], all constraints hard."""
    M=np.asarray(M,float); h=vector(h,18); J=np.asarray(J,float)
    if M.shape!=(18,18) or J.shape!=(12,18) or not np.isfinite(M).all() or not np.isfinite(J).all():
        raise ValueError('FULL_18_DOF_REQUIRED')
    env=np.zeros(6) if environment is None else vector(environment,6)
    E=np.zeros((36,54)); b=np.zeros(36)
    E[:18,:18]=M; E[:18,18:36]=-np.eye(18); E[:18,36:48]=J.T; b[:18]=-h
    E[18:24,36:48]=-G; E[18:24,48:]=Mobj; b[18:24]=env-hobj
    E[24:,:18]=J; E[24:,48:]=-H; b[24:]=vector(hdot,12)-vector(jdot,12)
    if not np.isfinite(E).all() or not np.isfinite(b).all(): raise ValueError('NONFINITE_MODEL')
    return E,b


def virtual_centres(model,q18):
    """No-contact kinematics only. Finger columns zero, dynamics still full 18D."""
    q=vector(q18,18); poses=[]; maps=[]
    for side,part,g in (('left',q[:7],nominal_grasps().left),('right',q[9:16],nominal_grasps().right)):
        X=model.fk(side,part)@inverse(g)
        r=model.fk(side,part)[:3,3]-X[:3,3]
        B=np.zeros((6,9)); B[:,:7]=np.block([[np.eye(3),skew(r)],[np.zeros((3,3)),np.eye(3)]])@model.jacobian(side,part)
        poses.append(X); maps.append(B)
    return poses,maps


def centre_jdot(model,q,dq,eps=1e-5):
    """Central directional derivative, checked against independent pose differences.

This is a declared numerical Jdot source, never labelled analytic/Isaac Jdot.
"""
    q=vector(q,18); dq=vector(dq,18)
    _,plus=virtual_centres(model,q+eps*dq); _,minus=virtual_centres(model,q-eps*dq)
    return [(p-m)@d/(2*eps) for p,m,d in zip(plus,minus,(dq[:9],dq[9:]))]


def local_contact_jacobian(model,q18,dq18,T):
    q=vector(q18,18); dq=vector(dq18,18); rows=[]; biases=[]
    for side,q9,d9 in (('left',q[:9],dq[:9]),('right',q[9:],dq[9:])):
        tcp=model.fk(side,q9[:7]); R=tcp[:3,:3]; axes=block_diag(R.T,R.T)
        Jw=np.zeros((6,9)); Jw[:,:7]=model.jacobian(side,q9[:7])
        eps=1e-5
        Jwd=(model.jacobian(side,q9[:7]+eps*d9[:7])-model.jacobian(side,q9[:7]-eps*d9[:7]))/(2*eps)
        vel=Jw@d9
        bias=axes@(Jwd@d9[:7]-block_diag(skew(vel[3:]),skew(vel[3:]))@vel)
        rows.append(axes@Jw); biases.append(bias)
    return block_diag(*rows),np.concatenate(biases)
