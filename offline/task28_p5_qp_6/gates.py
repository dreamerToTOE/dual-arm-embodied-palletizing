"""Latchable two-side pre-command and post-step research gates."""
import numpy as np
from .common import NAMES
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import residual


class Rejection(RuntimeError): pass


class RuntimeGate:
    def __init__(self,config,model): self.c=config; self.model=model; self.rejected=False

    def check(self,states,ref,time_s,step,effort=None,previous=None,dt=None):
        reasons=[]; c=self.c; metrics={}
        if self.rejected: return {'valid':False,'reasons':['LATCHED_REJECTION'],'metrics':{}}
        vel=np.minimum(self.model.manifest['joint_limits']['velocity_max_rad_s'],self.model.manifest['joint_limits']['isaac_velocity_max_rad_s'])
        for side in ['left','right']:
            r=states[side]
            if r['names']!=NAMES: reasons.append('DOF_IDENTITY:'+side)
            if r['time_s']!=time_s or r['step']!=step: reasons.append('STATE_SYNCHRONIZATION:'+side)
            if r.get('source_identity')!='/World/QP6_'+side: reasons.append('ARTICULATION_IDENTITY:'+side)
            for name,shape in [('q',(9,)),('dq',(9,)),('g',(9,)),('M',(9,9)),('c',(9,)),('tcp',(4,4)),('model_tcp',(4,4))]:
                value=np.asarray(r[name]);
                if value.shape!=shape or not np.isfinite(value).all(): reasons.append('INVALID_'+name+':'+side)
            if reasons: continue
            q=r['q']; dq=r['dq']
            if not self.model.within_limits(q[:7]): reasons.append('ARM_POSITION:'+side)
            if np.any(q[7:]<0) or np.any(q[7:]>.0400001): reasons.append('FINGER_POSITION:'+side)
            if np.any(np.abs(dq[:7])>vel) or np.any(np.abs(dq[7:])>.2): reasons.append('VELOCITY:'+side)
            fp,fr=residual(r['tcp'],r['model_tcp']); metrics[side+'_fk_translation_m']=fp; metrics[side+'_fk_rotation_rad']=fr
            if fp>c['fk_translation_m'] or fr>c['fk_rotation_rad']: reasons.append('FK_SOURCE:'+side)
            if r['effort_modes']!=['force']*9 or np.max(np.abs(r['drive_kp']))>0 or np.max(np.abs(r['drive_kd']))>0: reasons.append('EXPLICIT_EFFORT_MODE:'+side)
        if not reasons:
            cl=evaluate_closure(ref['pose'],states['left']['tcp'],states['right']['tcp'])
            metrics.update(left_translation_m=cl.left_object_translation_error_m,right_translation_m=cl.right_object_translation_error_m,
                           relative_translation_m=cl.translation_error_m,relative_rotation_rad=cl.rotation_error_rad,
                           left_rotation_rad=cl.left_object_rotation_error_rad,right_rotation_rad=cl.right_object_rotation_error_rad,
                           common_translation_m=float(np.linalg.norm((cl.implied_left_object[:3,3]+cl.implied_right_object[:3,3])/2-ref['pose'][:3,3])))
            if not cl.valid: reasons.extend(cl.reasons)
        if effort is not None:
            u=np.asarray(effort)
            if u.shape!=(18,) or not np.isfinite(u).all(): reasons.append('EFFORT_NONFINITE_OR_NOT_18D')
            else:
                bound=np.tile(c['effort_bounds_per_robot'],2)
                if np.any(np.abs(u)>bound): reasons.append('EFFORT_BOUND')
                if previous is not None:
                    rate=np.tile(c['effort_rate_bounds_per_s_per_robot'],2)*(c['physics_dt_s'] if dt is None else dt)
                    if np.any(np.abs(u-previous)>rate): reasons.append('EFFORT_RATE_BOUND')
        self.rejected=bool(reasons)
        return {'valid':not reasons,'reasons':reasons,'metrics':metrics}
