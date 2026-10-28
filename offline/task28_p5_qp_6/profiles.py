"""New C2 profiles, exact archived endpoints, quintic stop at every knot."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_p5_qp_4.reference import source_hashes, load_qp3_positive
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps


PROFILES={'S05':.1,'S10':.2}


def knots():
    source_hashes(); a=load_qp3_positive()['raw_qp2']
    return ([np.asarray(a['request']['object_pose'])]+[np.asarray(s['desired_object_pose_next']) for s in a['steps']],
            [np.asarray(a['request']['q14'])]+[np.asarray(s['q_next']) for s in a['steps']])


def time_law(u):
    if not np.isfinite(u) or u<0 or u>1: raise ValueError('TIME_OUT_OF_SEGMENT')
    return 10*u**3-15*u**4+6*u**5,30*u**2-60*u**3+30*u**4,60*u-180*u**2+120*u**3


class Profile:
    def __init__(self,name):
        if name not in PROFILES: raise ValueError('UNKNOWN_PROFILE')
        self.name=name; self.segment_s=PROFILES[name]; self.duration=5*self.segment_s
        self.poses,self.qs=knots(); self.identity='task28_qp6_PROFILE_'+name+'_quintic_object_and_joint_diagnostic_v1'

    def sample(self,t):
        if not np.isfinite(t) or t<0: raise ValueError('INVALID_TIME')
        k=min(int(t/self.segment_s),4); u=min(max((t-k*self.segment_s)/self.segment_s,0.),1.)
        s,sd,sdd=time_law(u); sd/=self.segment_s; sdd/=self.segment_s**2
        A,B=self.poses[k:k+2]; dq=self.qs[k+1]-self.qs[k]
        phi=Rotation.from_matrix(B[:3,:3]@A[:3,:3].T).as_rotvec(); dp=B[:3,3]-A[:3,3]
        T=np.eye(4); T[:3,:3]=Rotation.from_rotvec(s*phi).as_matrix()@A[:3,:3]; T[:3,3]=A[:3,3]+s*dp
        # Preserve the exact archived matrix at segment endpoints.
        if u==0.: T=A.copy()
        if u==1.: T=B.copy()
        V=np.r_[sd*dp,sd*phi]; acc=np.r_[sdd*dp,sdd*phi]
        q=self.qs[k]+s*dq
        if u==0.: q=self.qs[k].copy()
        if u==1.: q=self.qs[k+1].copy()
        grasp=nominal_grasps()
        return {'time_s':float(t),'segment':k,'u':u,'pose':T,'twist':V,'acceleration':acc,
                'q14':q,'dq14':sd*dq,'qdd14':sdd*dq,'left_tcp':T@grasp.left,'right_tcp':T@grasp.right,'identity':self.identity}


def full_joint_reference(row,finger=.04):
    q=row['q14']; dq=row['dq14']; ddq=row['qdd14']
    return (np.r_[q[:7],finger,finger,q[7:],finger,finger],
            np.r_[dq[:7],0.,0.,dq[7:],0.,0.],np.r_[ddq[:7],0.,0.,ddq[7:],0.,0.])
