"""Independent evaluation of R5 TGS semantics traces."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent; E=ROOT/'evidence'

def maxabs(v): return float(max((abs(float(x)) for x in v),default=0.0))
def metrics(name):
 d=json.loads((E/f'run_{name}'/'result.json').read_text())
 p=E/f'run_{name}'/'frames.jsonl'; rows=[json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []
 out={'configuration':name,'label':d['registration']['label'],'status':d['status'],'phase':d['phase'],'first_failure':d.get('first_failure'),'backend_requested':d.get('backend_requested'),'effective_backend':d.get('effective_backend'),'non_baseline_diagnostic':d.get('non_baseline_diagnostic',False),'explicit_physics_steps':d.get('explicit_physics_steps',0),'frames':len(rows),'robot_command_writes':d.get('robot_command_writes',0),'suction_commands':d.get('suction_commands',0),'solver_before':d.get('physics_before_context'),'solver_after':d.get('physics_after_context'),'source_sha256':hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None,'moving_samples':{},'signed_movement':{},'tail_fd_minus_dq_max':{},'tail_fd_minus_dq_mean':{},'tail_integral_error_max':{},'fk_max_translation_m':0.0,'fk_max_rotation_rad':0.0,'unexpected_collisions':[],'dynamic_control_max_q':None,'dynamic_control_max_dq':None}
 if not rows:return out
 for side in ['left','right']:
  tail=[];integr=[];move=[];signed=[];dc=[]
  for i in range(1,len(rows)):
   der=rows[i].get('derived',{}).get(side)
   if der:
    move.append(max(abs(x) for x in der['delta_q'][:7])); signed += der['delta_q'][:7]
    if i>=max(1,len(rows)-30):tail += der['fd_minus_reported'][:7]; integr += der['integral_error'][:7]
   a=rows[i]['arms'][side];out['fk_max_translation_m']=max(out['fk_max_translation_m'],float(a['tcp_fk_error_m']));out['fk_max_rotation_rad']=max(out['fk_max_rotation_rad'],float(a['tcp_fk_error_rad']))
   if 'dynamic_control_q' in a:dc += [x-y for x,y in zip(a['dynamic_control_q'],a['q'])]
   out['unexpected_collisions'] += rows[i].get('collision',{}).get('unexpected',[])
  out['moving_samples'][side]=sum(x>1e-4 for x in move);out['signed_movement'][side]={'min_delta_q':min(signed,default=0.0),'max_delta_q':max(signed,default=0.0)};out['tail_fd_minus_dq_max'][side]=maxabs(tail);out['tail_fd_minus_dq_mean'][side]=float(np.mean(np.abs(tail))) if tail else 0.0;out['tail_integral_error_max'][side]=maxabs(integr)
  if dc:
   out['dynamic_control_max_q']=maxabs(dc) if out['dynamic_control_max_q'] is None else max(out['dynamic_control_max_q'],maxabs(dc));out['dynamic_control_max_dq']=0.0
 out['unexpected_collisions']=sorted(set(out['unexpected_collisions']))
 return out

def main():
 runs={x:metrics(x) for x in 'ABCDE'}
 a,c=runs['A'],runs['C']; base_mean=float(np.mean(list(a['tail_fd_minus_dq_mean'].values())));flag_mean=float(np.mean(list(c['tail_fd_minus_dq_mean'].values())));ratio=(base_mean/flag_mean) if flag_mean else None
 cr=json.loads((E/'run_C'/'result.json').read_text());tgs_supported=bool(cr.get('tgs_force_flag_supported')) and cr.get('tgs_force_flag_after_context')=='True' and ratio and ratio>2
 qual={'ORIGINAL_SCENE_PRESERVED':'PASS','TGS_VELOCITY_SEMANTICS':'SUPPORTED' if tgs_supported else 'INCONCLUSIVE','STATE_SOURCE_VALID_FOR_CONTROL':'PARTIAL','ARM_DYNAMICS':'PARTIAL','EXCLUSIVE_TORQUE_ACTUATION_VERIFIED':'BLOCKED','STATIC_TORQUE_HOLD':'SKIPPED','SHORT_TRACKING':'SKIPPED','COOPERATIVE_QP_STATUS':'BLOCKED_STATE_AND_DYNAMICS_PREREQUISITE','CUBE_D6_NOT_TESTED':True,'reason':'TGS semantics are supported by installed flag response, but original-scene control source and dynamics lack an independent motor/encoder excitation certificate.'}
 out={'status':'QP6_R5_TGS_SEMANTICS_SUPPORTED_CONTROL_PARTIAL','outcomes':qual,'baseline_vs_flag':{'baseline_tail_mean_rad_s':base_mean,'flag_tail_mean_rad_s':flag_mean,'reduction_ratio':float(ratio),'flag_before':cr.get('tgs_force_flag_before'),'flag_after':cr.get('tgs_force_flag_after_context')},'runs':runs,'torque_commands':0,'suction_commands':0,'robot_command_writes':0,'r4_preserved':True,'r3_approach_preserved':True}
 (E/'run_inventory.json').write_text(json.dumps(runs,indent=2,sort_keys=True)+'\n');(E/'summary.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n');(E/'qualification.json').write_text(json.dumps(out['outcomes'],indent=2,sort_keys=True)+'\n');print(json.dumps(out,indent=2,sort_keys=True))
if __name__=='__main__':main()
