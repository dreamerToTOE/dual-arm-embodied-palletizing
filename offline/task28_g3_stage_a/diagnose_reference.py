import argparse
import json
from pathlib import Path
import numpy as np
from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6_r12.qp_math import se3_log
parser=argparse.ArgumentParser()
parser.add_argument('run',type=Path)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--assert-consistent',action='store_true')
args=parser.parse_args()
run=args.run
model=RobotModel(); rel0=None;first=None;peak=None;maxerr=-1;count=0;over=0
with open_trace(run) as stream:
 for line in stream:
  r=json.loads(line);qp=r.get('qp') or {};a=qp.get('trajectory_adapter') or {}
  if a.get('task_mode')!='DUAL_ATTACHED':continue
  actual={s:np.array(r['tcp_pose_world'][s]) for s in ('left','right')}
  if rel0 is None:rel0=np.linalg.inv(actual['left'])@actual['right']
  ref={s:model.fk(s,r['q_ref_rad'][s]) for s in ('left','right')}
  raw={s:model.fk(s,a['latest_captured_q_ref_rad'][s]) for s in ('left','right')}
  closure=np.linalg.norm(se3_log(np.linalg.inv(rel0)@np.linalg.inv(ref['left'])@ref['right'])[:3])*1000
  rawclosure=np.linalg.norm(se3_log(np.linalg.inv(rel0)@np.linalg.inv(raw['left'])@raw['right'])[:3])*1000
  delta=max(np.linalg.norm(ref[s][:3,3]-raw[s][:3,3])*1000 for s in ref)
  item={'step':r['physics_step'],'time_s':r['time_s'],'desired_closure_mm':closure,'latest_source_closure_mm':rawclosure,'adapter_vs_source_max_tcp_mm':delta,'reference_sources':a['reference_age'],'clock_scale':r['trajectory_clock_scale_source_per_sim'],'qref':r['q_ref_rad'],'source_qref':a['latest_captured_q_ref_rad'],'ref_tcp_xyz_m':{s:ref[s][:3,3].tolist() for s in ref},'raw_tcp_xyz_m':{s:raw[s][:3,3].tolist() for s in ref},'actual_tcp_xyz_m':{s:actual[s][:3,3].tolist() for s in ref}}
  if closure>.15:
   over+=1
   if first is None:first=item
  if closure>maxerr:maxerr=closure;peak=item
  count+=1
out={'loaded_samples':count,'inconsistent_over_0_150mm':over,'first_inconsistent':first,'peak':peak,'source_stamp_limitation':'per-capture stamps not archived; source is latest captured qref, pairing implementation uses equal stamps'}
args.output.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ('loaded_samples','inconsistent_over_0_150mm')},indent=2))
for name in ('first_inconsistent','peak'):
 print(name,{k:v for k,v in (out[name] or {}).items() if k not in ('qref','source_qref')})
if args.assert_consistent:
 assert count > 0 and maxerr<=.150, f'INCOMPATIBLE_DESIRED_GRASP:{maxerr:.6f}mm'
