from __future__ import annotations
import json, hashlib
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import checked_transform, transform, residual
from offline.task28_p5_qp_4.reference import load_qp3_positive, build_reference_sequence
from .controller import sample_reference, interpolate_pose
ROOT=Path(__file__).resolve().parents[2]
PREREG=ROOT/'offline/task28_p5_qp_5/preregistration.json'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load_config(): return json.loads(PREREG.read_text())
def _arr(x,n):
 a=np.asarray(x,float)
 if a.shape!=(n,) or not np.isfinite(a).all(): raise ValueError('NONFINITE_OR_SHAPE')
 return a
def evaluate_rows(rows, config=None, moving=False):
 c=config or load_config(); model=RobotModel(); refs=build_reference_sequence(load_qp3_positive())
 out={'status':'PASS','sample_count':len(rows),'reasons':[],'max_closure_translation_m':0.,'max_closure_rotation_rad':0.,'max_link7_fk_translation_m':0.,'max_link7_fk_rotation_rad':0.,'max_position_error_rad':0.,'max_velocity_error_rad_s':0.,'max_command_readback_error':0.,'first_rejected_step':None}
 for i,row in enumerate(rows):
  try:
   t=float(row['sim_time_s']); ql=_arr(row['arms']['left']['q_rad'],9); qr=_arr(row['arms']['right']['q_rad'],9)
   if row['arms']['left']['dof_names'] != c['expected_all_dof_names'] or row['arms']['right']['dof_names'] != c['expected_all_dof_names']: raise ValueError('DOF_ORDER_MISMATCH')
   for side,q in [('left',ql[:7]),('right',qr[:7])]:
    if not model.within_limits(q): raise ValueError('JOINT_LIMIT_VIOLATION:'+side)
    if abs(q[0])>c['q1_abs_limit_rad']: raise ValueError('Q1_LIMIT:'+side)
    link7=checked_transform(row['arms'][side]['link7_world'])
    expected=model._chain(side,q)[0][side+'_fr3_link7']; pe,re=residual(link7,expected)
    out['max_link7_fk_translation_m']=max(out['max_link7_fk_translation_m'],pe); out['max_link7_fk_rotation_rad']=max(out['max_link7_fk_rotation_rad'],re)
    if pe>c['link7_fk_position_tolerance_m'] or re>c['link7_fk_rotation_tolerance_rad']: raise ValueError('LIVE_LINK7_FK_MISMATCH:'+side)
   tcp_l=checked_transform(row['arms']['left']['tcp_world']); tcp_r=checked_transform(row['arms']['right']['tcp_world']); obj=checked_transform(row['object_pose_world'])
   cl=evaluate_closure(obj,tcp_l,tcp_r)
   out['max_closure_translation_m']=max(out['max_closure_translation_m'],cl.translation_error_m); out['max_closure_rotation_rad']=max(out['max_closure_rotation_rad'],cl.rotation_error_rad)
   if not cl.valid: raise ValueError('CLOSURE_GATE:'+','.join(cl.reasons))
   cmd_l=_arr(row['arms']['left']['commanded_effort'],9); app_l=_arr(row['arms']['left']['applied_effort'],9); cmd_r=_arr(row['arms']['right']['commanded_effort'],9); app_r=_arr(row['arms']['right']['applied_effort'],9)
   err=max(float(np.max(np.abs(cmd_l-app_l))),float(np.max(np.abs(cmd_r-app_r)))); out['max_command_readback_error']=max(out['max_command_readback_error'],err)
   if err>c['command_readback_tolerance']: raise ValueError('COMMAND_READBACK_MISMATCH')
   if moving:
    qref,qdref,k,phase=sample_reference(refs,t)
    qa=np.r_[ql[:7],qr[:7]]; dqa=np.r_[ _arr(row['arms']['left']['dq_rad_s'],9)[:7], _arr(row['arms']['right']['dq_rad_s'],9)[:7] ]
    out['max_position_error_rad']=max(out['max_position_error_rad'],float(np.max(np.abs(qa-qref))))
    out['max_velocity_error_rad_s']=max(out['max_velocity_error_rad_s'],float(np.max(np.abs(dqa-qdref))))
  except Exception as e:
   out['status']='REJECTED'; out['reasons'].append(str(e)); out['first_rejected_step']=i if out['first_rejected_step'] is None else out['first_rejected_step']; break
 out['reasons']=sorted(set(out['reasons'])); return out
def evaluate_directory(path, moving=False):
 p=Path(path); status=json.loads((p/'collector_status.json').read_text()); rows=[]
 for name in status.get('completed_files',[]): rows.extend(json.loads((p/name).read_text()).get('observations',[]))
 return evaluate_rows(rows,moving=moving)
