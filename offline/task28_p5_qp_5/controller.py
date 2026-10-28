from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

def explicit_effort(q, dq, q_ref, dq_ref, gravity, kp, kd, effort_bounds, rate_bounds, previous=None):
    q=np.asarray(q,float); dq=np.asarray(dq,float); q_ref=np.asarray(q_ref,float); dq_ref=np.asarray(dq_ref,float); gravity=np.asarray(gravity,float)
    raw=gravity+np.asarray(kp)*(q_ref-q)+np.asarray(kd)*(dq_ref-dq)
    bounded=np.clip(raw,-np.asarray(effort_bounds),np.asarray(effort_bounds)); rate_limited=bounded.copy(); rate_limited_flag=False
    if previous is not None:
        previous=np.asarray(previous,float); rate_limited=np.clip(bounded,previous-np.asarray(rate_bounds),previous+np.asarray(rate_bounds)); rate_limited_flag=bool(np.any(rate_limited != bounded))
    return {"raw":raw,"command":rate_limited,"saturated":bool(np.any(bounded != raw)),"rate_limited":rate_limited_flag}

def _start_q(refs,k):
    if k==0:
        q=np.r_[refs[0].left_q_rad,refs[0].right_q_rad]; qd=np.r_[refs[0].left_qdot_rad_s,refs[0].right_qdot_rad_s]
        return q-0.02*qd
    return np.r_[refs[k-1].left_q_rad,refs[k-1].right_q_rad]

def sample_reference(refs, t, final_hold=True):
    if t < 0: raise ValueError('NEGATIVE_TIME')
    if t >= len(refs)*0.02:
        q=np.r_[refs[-1].left_q_rad,refs[-1].right_q_rad]
        return q, np.zeros(14), len(refs)-1, 'final_hold'
    k=min(int(np.floor(t/0.02)),len(refs)-1); local=float(t-k*0.02)
    q0=_start_q(refs,k); qd=np.r_[refs[k].left_qdot_rad_s,refs[k].right_qdot_rad_s]
    return q0+local*qd, qd, k, 'interval'

def interpolate_pose(refs, initial_pose, t):
    """Piecewise SE(3) interpolation between archived interval endpoints."""
    initial=np.asarray(initial_pose,float)
    if t>=len(refs)*0.02: return np.asarray(refs[-1].object_pose_world,float)
    k=min(int(np.floor(max(0.,t)/0.02)),len(refs)-1); u=float((t-k*0.02)/0.02)
    a=initial if k==0 else np.asarray(refs[k-1].object_pose_world,float); b=np.asarray(refs[k].object_pose_world,float)
    out=np.eye(4); out[:3,3]=(1-u)*a[:3,3]+u*b[:3,3]
    s=Slerp([0.,1.],Rotation.from_matrix(np.stack([a[:3,:3],b[:3,:3]]))); out[:3,:3]=s([u]).as_matrix()[0]
    return out
