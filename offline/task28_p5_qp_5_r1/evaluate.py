#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import checked_transform
ROOT=Path(__file__).resolve().parents[2]
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arr(x,n):
 a=np.asarray(x,float)
 if a.shape!=(n,) or not np.isfinite(a).all(): raise ValueError('NONFINITE_OR_SHAPE')
 return a
def run(path):
 p=Path(path); c=json.loads((ROOT/'offline/task28_p5_qp_5_r1/preregistration.json').read_text()); st=json.loads((p/'collector_status.json').read_text()); model=RobotModel(); out={'schema':'task28.g2_p5_qp_5_r1.evaluation.v1','status':'PASS','source_status':st.get('status'),'static':{},'tracking':{},'first_failure':None,'reasons':[]}
 static=json.loads((p/'static_hold_observations.json').read_text())['observations']; track=json.loads((p/'tracking_observations.json').read_text())['observations']
 def scan(rows,phase):
  maxs={'relative_translation_m':0.,'relative_rotation_rad':0.,'left_absolute_translation_m':0.,'right_absolute_translation_m':0.,'left_absolute_rotation_rad':0.,'right_absolute_rotation_rad':0.,'position_error_inf':0.,'velocity_error_inf':0.}; reasons=[]
  for i,r in enumerate(rows):
   for side in ('left','right'):
    q=arr(r['arms'][side]['q_rad'],9); dq=arr(r['arms'][side]['dq_rad_s'],9)
    if not model.within_limits(q[:7]): reasons.append((i,'JOINT_LIMIT:'+side))
    if not np.isfinite(dq).all(): reasons.append((i,'NONFINITE_DQ:'+side))
   cl=evaluate_closure(np.asarray(r['object_pose_world']),checked_transform(r['arms']['left']['tcp_world']),checked_transform(r['arms']['right']['tcp_world']))
   vals={'relative_translation_m':cl.translation_error_m,'relative_rotation_rad':cl.rotation_error_rad,'left_absolute_translation_m':cl.left_object_translation_error_m,'right_absolute_translation_m':cl.right_object_translation_error_m,'left_absolute_rotation_rad':cl.left_object_rotation_error_rad,'right_absolute_rotation_rad':cl.right_object_rotation_error_rad}
   for k,v in vals.items(): maxs[k]=max(maxs[k],v)
   qa=np.r_[arr(r['arms']['left']['q_rad'],9)[:7],arr(r['arms']['right']['q_rad'],9)[:7]]; dq=np.r_[arr(r['arms']['left']['dq_rad_s'],9)[:7],arr(r['arms']['right']['dq_rad_s'],9)[:7]]; qr=np.asarray(r['reference']['q_ref_14']); qdr=np.asarray(r['reference']['qdot_ref_14']); maxs['position_error_inf']=max(maxs['position_error_inf'],float(np.max(np.abs(qa-qr)))); maxs['velocity_error_inf']=max(maxs['velocity_error_inf'],float(np.max(np.abs(dq-qdr))))
   if cl.left_object_translation_error_m>c['closure_translation_tolerance_m']: reasons.append((i,'LEFT_ABSOLUTE_TRANSLATION'))
   if cl.right_object_translation_error_m>c['closure_translation_tolerance_m']: reasons.append((i,'RIGHT_ABSOLUTE_TRANSLATION'))
   if cl.translation_error_m>c['closure_translation_tolerance_m']: reasons.append((i,'RELATIVE_TRANSLATION'))
   if cl.rotation_error_rad>c['closure_rotation_tolerance_rad']: reasons.append((i,'RELATIVE_ROTATION'))
  return {'sample_count':len(rows),'maxima':maxs,'reasons':reasons}
 out['static']=scan(static,'static'); out['tracking']=scan(track,'tracking');
 if st.get('status')!='REJECTED': out['reasons'].append('STATUS_NOT_REJECTED')
 if len(track)!=st.get('tracking_steps_executed'): out['reasons'].append('TRACKING_STEP_COUNT_MISMATCH')
 invalid=[i for i,r in enumerate(track) if not r.get('runtime_gate',{}).get('valid',False)]
 if not invalid: out['reasons'].append('NO_RUNTIME_REJECTION')
 else:
  out['first_failure']={'index':invalid[0],'time_s':track[invalid[0]]['sim_time_s'],'runtime_reasons':track[invalid[0]]['runtime_gate']['reasons']}
  if len(track)!=invalid[0]+1: out['reasons'].append('CONTINUED_AFTER_FIRST_FAILURE')
 if out['static']['reasons']: out['reasons'].append('STATIC_GATE_REJECTED')
 out['reasons']=sorted(set(out['reasons'])); out['status']='PASS' if not out['reasons'] else 'REJECTED'; return out
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--input',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args(); r=run(a.input); a.output.write_text(json.dumps(r,indent=2,sort_keys=True,allow_nan=False)+'\n'); print(json.dumps(r,sort_keys=True)); return 0 if r['status']=='PASS' else 1
if __name__=='__main__': raise SystemExit(main())
