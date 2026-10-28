from __future__ import annotations
import numpy as np
from offline.task28_g1_1.task28_validator.se3 import checked_transform, residual
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_p5_qp_5.controller import sample_reference, interpolate_pose

def _a(v,n):
 a=np.asarray(v,float)
 if a.shape!=(n,) or not np.isfinite(a).all(): raise ValueError('NONFINITE_STATE_OR_SHAPE')
 return a

def validate_post(row, *, expected_time, config, model, refs, initial_pose, previous_cmd, phase):
 c=config; reasons=[]
 t=float(row['sim_time_s'])
 if not np.isfinite(t) or abs(t-expected_time)>1e-7: reasons.append('TIMESTAMP_MISMATCH')
 qref,qdref,k,refphase=(sample_reference(refs,t) if phase=='tracking' else (np.r_[row['source_q14'][:7],row['source_q14'][7:]],np.zeros(14),-1,'hold'))
 obj=(interpolate_pose(refs,initial_pose,t) if phase=='tracking' else np.asarray(initial_pose,float))
 if not np.allclose(np.asarray(row['reference']['q_ref_14']),qref,atol=1e-10,rtol=0): reasons.append('REFERENCE_Q_SOURCE_MISMATCH')
 if not np.allclose(np.asarray(row['reference']['qdot_ref_14']),qdref,atol=1e-10,rtol=0): reasons.append('REFERENCE_QDOT_SOURCE_MISMATCH')
 if not np.allclose(np.asarray(row['object_pose_world']),obj,atol=1e-10,rtol=0): reasons.append('REFERENCE_OBJECT_SOURCE_MISMATCH')
 if row.get('reference',{}).get('source_identity')!='task28_qp3_seed28_transport40_5step': reasons.append('REFERENCE_SOURCE_IDENTITY_MISMATCH')
 for side,sl in (('left',slice(0,7)),('right',slice(7,14))):
  arm=row['arms'][side]
  if arm.get('dof_names')!=c['expected_all_dof_names']: reasons.append('DOF_ORDER_MISMATCH:'+side)
  q=_a(arm['q_rad'],9); dq=_a(arm['dq_rad_s'],9); raw=_a(arm['raw_effort'],9); cmd=_a(arm['commanded_effort'],9); app=_a(arm['applied_effort'],9)
  if not model.within_limits(q[:7]): reasons.append('JOINT_LIMIT_VIOLATION:'+side)
  if abs(q[0])>c['q1_abs_limit_rad'] or np.max(np.abs(dq[:7]))>c['arm_speed_limit_rad_s']: reasons.append('SPEED_OR_Q1_LIMIT:'+side)
  if np.any(np.abs(raw)>np.asarray(c['effort_bounds'])+1e-9) or arm.get('saturated'): reasons.append('EFFORT_BOUND_OR_SATURATION:'+side)
  if previous_cmd is not None and previous_cmd.get(side) is not None and np.any(np.abs(cmd-previous_cmd[side])>np.asarray(c['effort_rate_bounds_per_step'])+1e-8): reasons.append('EFFORT_RATE_VIOLATION:'+side)
  if arm.get('rate_limited'): reasons.append('RATE_LIMITED_COMMAND:'+side)
  if np.max(np.abs(cmd-app))>c['command_readback_tolerance']: reasons.append('COMMAND_READBACK_MISMATCH:'+side)
  link=checked_transform(arm['link7_world']); pe,re=residual(link,model._chain(side,q[:7])[0][side+'_fr3_link7'])
  if pe>c['link7_fk_position_tolerance_m'] or re>c['link7_fk_rotation_tolerance_rad']: reasons.append('LIVE_LINK7_FK_MISMATCH:'+side)
 cl=evaluate_closure(obj,checked_transform(row['arms']['left']['tcp_world']),checked_transform(row['arms']['right']['tcp_world']))
 if cl.translation_error_m>c['closure_translation_tolerance_m']: reasons.append('RELATIVE_CLOSURE_TRANSLATION')
 if cl.rotation_error_rad>c['closure_rotation_tolerance_rad']: reasons.append('RELATIVE_CLOSURE_ROTATION')
 for name,v in [('left_absolute_translation',cl.left_object_translation_error_m),('right_absolute_translation',cl.right_object_translation_error_m)]:
  if v>c['closure_translation_tolerance_m']: reasons.append(name.upper())
 for name,v in [('left_absolute_rotation',cl.left_object_rotation_error_rad),('right_absolute_rotation',cl.right_object_rotation_error_rad)]:
  if v>c['closure_rotation_tolerance_rad']: reasons.append(name.upper())
 qa=np.r_[_a(row['arms']['left']['q_rad'],9)[:7],_a(row['arms']['right']['q_rad'],9)[:7]]; dqa=np.r_[_a(row['arms']['left']['dq_rad_s'],9)[:7],_a(row['arms']['right']['dq_rad_s'],9)[:7]]
 if phase=='tracking':
  if np.max(np.abs(qa-qref))>c['tracking_position_tolerance_rad']: reasons.append('POSITION_TRACKING_LIMIT')
  if np.max(np.abs(dqa-qdref))>c['tracking_velocity_tolerance_rad_s']: reasons.append('VELOCITY_TRACKING_LIMIT')
 return {'valid':not reasons,'reasons':sorted(set(reasons)),'closure':{'relative_translation_m':cl.translation_error_m,'relative_rotation_rad':cl.rotation_error_rad,'left_absolute_translation_m':cl.left_object_translation_error_m,'right_absolute_translation_m':cl.right_object_translation_error_m,'left_absolute_rotation_rad':cl.left_object_rotation_error_rad,'right_absolute_rotation_rad':cl.right_object_rotation_error_rad},'position_error_inf':float(np.max(np.abs(qa-qref))),'velocity_error_inf':float(np.max(np.abs(dqa-qdref))),'reference_step_index':k,'reference_phase':refphase}

def stop_after_first_failure(gate_results):
    """Pure control-flow oracle used by tests: reject at first false gate."""
    inspected=[]
    for i,result in enumerate(gate_results):
        inspected.append(i)
        if not result.get('valid',False):
            return {'status':'REJECTED','executed_steps':len(inspected),'first_failure_index':i,'continued_after_failure':False}
    return {'status':'PASS','executed_steps':len(inspected),'first_failure_index':None,'continued_after_failure':False}
