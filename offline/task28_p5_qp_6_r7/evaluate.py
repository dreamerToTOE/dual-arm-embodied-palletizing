"""Independent evaluation of R7 diagnostic/formal traces; never issues physics commands."""
from __future__ import annotations
import json, sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent

def load(name):
 d=HERE/'evidence'/'diagnostic'/name
 return json.loads((d/'result.json').read_text()), [json.loads(x) for x in (d/'frames.jsonl').read_text().splitlines() if x.strip()]

def eval_cell(name):
 r,frames=load(name); out={"status":r.get("status"),"commands":r.get("actual_effort_commands",0),"writes":r.get("robot_command_writes",0),"unexpected_collisions":[],"sides":{}}
 for f in frames: out["unexpected_collisions"] += f.get("collision",{}).get("unexpected",[])
 start_i=next((i for i,f in enumerate(frames) if f.get('arms',{}).get('left',{}).get('position_drives_disabled')),None); control_frames=frames[start_i:] if start_i is not None else []; first=control_frames[0] if control_frames else None; last=control_frames[-1] if control_frames else None
 if first and last:
  for s in ('left','right'):
   a,b=first['arms'][s],last['arms'][s]; q0,q1=np.asarray(a['q'][:7],float),np.asarray(b['q'][:7],float)
   t0,t1=np.asarray(a['tcp_world'],float),np.asarray(b['tcp_world'],float)
   out['sides'][s]={"max_abs_joint_drift_rad":float(np.max(np.abs(q1-q0))),"joint_drift_rad":(q1-q0).tolist(),"tcp_translation_m":float(np.linalg.norm(t1[:3,3]-t0[:3,3])),"tcp_rotation_frobenius":float(np.linalg.norm(t1[:3,:3]-t0[:3,:3])),"max_abs_dq_rad_s":float(max(np.max(np.abs(np.asarray(f['arms'][s]['dq'][:7],float))) for f in control_frames))}
 out['stable_complete_interval']=bool(out['status'] in ('DIAGNOSTIC_STABILIZATION_COMPLETE','FORMAL_STATIC_HOLD_COMPLETE') and not out['unexpected_collisions'] and out['commands']==r.get('registration',{}).get('steps'))
 out['gates']={"source_valid":out['status'] in ('DIAGNOSTIC_STABILIZATION_COMPLETE','FORMAL_STATIC_HOLD_COMPLETE'),"commands_complete":out['commands']==r.get('registration',{}).get('steps'),"tcp_tolerance":all(v['tcp_translation_m']<=0.001 for v in out['sides'].values()) if out['sides'] else False,"joint_tolerance":all(v['max_abs_joint_drift_rad']<=0.02 for v in out['sides'].values()) if out['sides'] else False,"velocity_tolerance":all(v['max_abs_dq_rad_s']<=0.30 for v in out['sides'].values()) if out['sides'] else False}
 out['stable_complete_interval']=out['stable_complete_interval'] and all(out['gates'].values())
 return out

def main():
 cells={};
 for n in ('S1','S2','S3','FORMAL'):
  try: cells[n]=eval_cell(n)
  except FileNotFoundError: cells[n]={"status":"MISSING","stable_complete_interval":False}
 diagnostic={n:cells[n] for n in ('S1','S2','S3')}
 eligible=[n for n in ('S1','S2','S3') if cells[n].get('stable_complete_interval')]
 formal=cells['FORMAL']
 result={"schema":"TASK28_G2_P5_QP_6_R7_EVALUATION_V1","cells":cells,"diagnostic_eligible":eligible,"diagnostic_stabilization_status":"PASS" if len(eligible)>=2 and all(cells[n].get('stable_complete_interval') for n in ('S2','S3')) else "FAIL_OR_INCOMPLETE","formal_static_hold_status":"PASS" if formal.get('stable_complete_interval') else ("FAIL" if formal.get('status')!='MISSING' else "SKIPPED"),"formal_prerequisite":"S2/S3 stable and source-valid","moving_tracking_status":"SKIPPED_FORMAL_STATIC_PREREQUISITE","computed_torque_status":"SKIPPED_MODEL_NOT_QUALIFIED","cooperative_qp_status":"SKIPPED_NO_MOVING_CONTROLLER","cube_suction_status":"NOT_TESTED"}
 (HERE/'evidence'/'evaluation.json').write_text(json.dumps(result,indent=2,sort_keys=True)+"\n"); print(json.dumps(result,indent=2,sort_keys=True)); return 0
if __name__=='__main__': raise SystemExit(main())
