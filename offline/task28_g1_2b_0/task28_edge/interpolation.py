"""Pure reconstruction; never IK-project, move grasp frames or repair a state."""
from dataclasses import replace
import math
import numpy as np
from scipy.spatial.transform import Rotation
from task28_contact.modes import OperationMode,ObservedAttachment,EvidenceSource
from task28_contact.clearance import ConfirmationEvent
from task28_validator.se3 import pose_matrix
from .types import (Waypoint,Provenance,Source,Motion,JointInterpolation,
                    TranslationInterpolation)


def coefficients(p0,p1,v0,v1,dt,hermite):
    p0,p1=np.asarray(p0,dtype=float),np.asarray(p1,dtype=float)
    if not hermite:return np.zeros_like(p0),np.zeros_like(p0),p1-p0,p0
    v0,v1=np.asarray(v0,dtype=float)*dt,np.asarray(v1,dtype=float)*dt
    return 2*p0-2*p1+v0+v1,-3*p0+3*p1-2*v0-v1,v0,p0


def evaluate(coeff,u):
    a,b,c,d=coeff
    return ((a*u+b)*u+c)*u+d


def derivative_bound(coeff):
    a,b,c,_=coeff
    candidates=[np.abs(c),np.abs(3*a+2*b+c)]
    u=np.divide(-b,3*a,out=np.zeros_like(a),where=np.abs(a)>1e-30)
    interior=np.where((u>0)&(u<1),np.abs(3*a*u*u+2*b*u+c),0)
    return np.maximum.reduce([*candidates,interior])


def rotation_delta(a,b):
    left=Rotation.from_quat(a.object_quaternion_xyzw)
    right=Rotation.from_quat(b.object_quaternion_xyzw)
    return left,(left.inv()*right).as_rotvec()


def span_coefficients(a,b,spec):
    dt=b.time_s-a.time_s
    hermite=spec.joints is JointInterpolation.CUBIC_HERMITE
    return (coefficients(a.left_q,b.left_q,a.left_velocity_rad_s,b.left_velocity_rad_s,dt,hermite),
            coefficients(a.right_q,b.right_q,a.right_velocity_rad_s,b.right_velocity_rad_s,dt,hermite),
            coefficients(a.object_position_m,b.object_position_m,a.object_velocity_m_s,b.object_velocity_m_s,dt,
                         spec.translation is TranslationInterpolation.CUBIC_HERMITE))


def subdivisions(a,b,spec,policy):
    left,right,obj=span_coefficients(a,b,spec)
    _,rotation=rotation_delta(a,b)
    required=max(2,math.ceil((b.time_s-a.time_s)/policy.max_dt_s),
               math.ceil(float(max(derivative_bound(left).max(),derivative_bound(right).max()))/policy.max_joint_step_rad),
               math.ceil(float(np.linalg.norm(derivative_bound(obj)))/policy.max_object_step_m),
               math.ceil(float(np.linalg.norm(rotation))/policy.max_rotation_step_rad))
    return 2*math.ceil(required/2)  # Always include the exact midpoint.


def reconstruct(a,b,u,spec,motion,support,identity):
    if u==0.:return a
    if u==1.:return b
    left,right,obj=span_coefficients(a,b,spec)
    position=tuple(float(x) for x in evaluate(obj,u))
    rotation,delta=rotation_delta(a,b)
    quaternion=tuple(float(x) for x in (rotation*Rotation.from_rotvec(u*delta)).as_quat())
    now=float(a.time_s+u*(b.time_s-a.time_s))
    mode,site=a.mode,a.support_site
    if motion in (Motion.LIFT,Motion.PLACE):
        from task28_contact.modes import SupportSite
        site=SupportSite.SUPPLY if motion is Motion.LIFT else SupportSite.TARGET
        geometric=support.geometry(pose_matrix(position,quaternion),site.value)
        if geometric.valid:mode=OperationMode.BILATERAL_SUPPORTED
        else:mode,site=OperationMode.BILATERAL_TRANSITIONING,None
    evidence=f'{identity}:derived:{now.hex()}'
    observed=ObservedAttachment(evidence,now,a.observed.request_id,a.observed.left,a.observed.right,
                                EvidenceSource.INDEPENDENT_SYNTHETIC)
    return Waypoint(now,tuple(float(x) for x in evaluate(left,u)),tuple(float(x) for x in evaluate(right,u)),
                    position,quaternion,mode,site,a.left_attached,a.right_attached,a.intent,observed,
                    Provenance(Source.MODEL_INTERPOLATION,evidence+':cube',evidence,
                      'Model reconstruction; held explicit feedback flags, synthetic timestamps; not new feedback.'),
                    ConfirmationEvent.OBSERVE)
