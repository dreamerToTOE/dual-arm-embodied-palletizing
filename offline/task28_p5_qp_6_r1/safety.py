"""Shared paired pre-command/post-step gates for new R1 runs."""
import numpy as np
from .common import NAMES
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import residual


def check(states,obj,expected_time,expected_step,c,model,efforts=None,previous=None):
    reasons=[]; metrics={}
    vmax=np.minimum(model.manifest['joint_limits']['velocity_max_rad_s'],model.manifest['joint_limits']['isaac_velocity_max_rad_s'])
    for side in ['left','right']:
        a=states[side]
        if a['names']!=NAMES or a['identity']!='/World/QP6R1_'+side: reasons.append('IDENTITY:'+side)
        if a['time_s']!=expected_time or a['physics_step']!=expected_step: reasons.append('SYNCHRONIZATION:'+side)
        for key,shape in [('q',(9,)),('dq',(9,)),('M',(9,9)),('c',(9,)),('g',(9,)),('tcp',(4,4))]:
            v=np.asarray(a[key])
            if v.shape!=shape or not np.isfinite(v).all(): reasons.append('INVALID_'+key+':'+side)
        if reasons: continue
        q=np.asarray(a['q']); dq=np.asarray(a['dq']); M=np.asarray(a['M'])
        if not model.within_limits(q[:7]): reasons.append('ARM_LIMIT:'+side)
        if np.any(q[7:]<np.asarray(a['finger_lower'])-c['finger_position_numerical_tolerance_m']) or np.any(q[7:]>np.asarray(a['finger_upper'])+c['finger_position_numerical_tolerance_m']): reasons.append('FINGER_LIMIT:'+side)
        if np.any(np.abs(dq[:7])>vmax) or np.any(np.abs(dq[7:])>c['finger_safety_velocity_m_s']): reasons.append('VELOCITY:'+side)
        if np.max(np.abs(M-M.T))>1e-5 or np.linalg.eigvalsh(M).min()<1e-6: reasons.append('MASS_SPD:'+side)
        if a['effort_modes']!=['force']*9 or np.max(np.abs(a['drive_kp']))>0 or np.max(np.abs(a['drive_kd']))>0: reasons.append('EXPLICIT_MODE:'+side)
        p,r=residual(a['tcp'],model.fk(side,q[:7])); metrics[side+'_fk_translation_m']=p; metrics[side+'_fk_rotation_rad']=r
        if p>c['fk_translation_m'] or r>c['fk_rotation_rad']: reasons.append('FK_SOURCE:'+side)
    if not reasons:
        cl=evaluate_closure(obj,states['left']['tcp'],states['right']['tcp'])
        metrics.update(left_translation_m=cl.left_object_translation_error_m,right_translation_m=cl.right_object_translation_error_m,relative_translation_m=cl.translation_error_m,left_rotation_rad=cl.left_object_rotation_error_rad,right_rotation_rad=cl.right_object_rotation_error_rad,relative_rotation_rad=cl.rotation_error_rad,common_translation_m=float(np.linalg.norm((cl.implied_left_object[:3,3]+cl.implied_right_object[:3,3])/2-np.asarray(obj)[:3,3])))
        if not cl.valid: reasons.extend(cl.reasons)
    if efforts is not None:
        u=np.asarray(efforts)
        if u.shape!=(18,) or not np.isfinite(u).all(): reasons.append('EFFORT_SHAPE_OR_FINITE')
        else:
            if np.any(np.abs(u)>np.tile(c['effort_bounds'],2)): reasons.append('EFFORT_BOUND')
            if previous is not None and np.any(np.abs(u-previous)>np.tile(c['effort_rate_bounds_per_s'],2)*c['physics_dt_s']): reasons.append('EFFORT_RATE')
    return {'valid':not reasons,'reasons':reasons,'metrics':metrics}
