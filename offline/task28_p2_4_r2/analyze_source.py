#!/usr/bin/env python3
"""Read-only metrics for the completed explicit-effort source run."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np

def sha(p):
 h=hashlib.sha256()
 with open(p,'rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--input',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args()
 status=json.loads((a.input/'collector_status.json').read_text())
 cases={}; all_dts=[]
 for cid in ('zero','plus_q1','minus_q1'):
  d=json.loads((a.input/(cid+'_observations.json')).read_text()); rows=d['samples']
  cmd=np.array([r['commanded_effort'] for r in rows],float); app=np.array([r['applied_effort_readback'] for r in rows],float); proj=np.array([r['measured_projected_effort'] for r in rows],float); q=np.array([r['q_rad_or_m'] for r in rows],float); dq=np.array([r['dq_rad_s_or_m_s'] for r in rows],float); t=np.array([r['sim_time_s'] for r in rows],float)
  dt=np.diff(t); all_dts.extend(dt.tolist())
  cases[cid]={'sample_count':len(rows),'command':cmd[0].tolist(),'max_applied_readback_error':float(np.max(np.abs(app-cmd))), 'mean_projected_minus_command_joint1':float(np.mean(proj[:,0]-cmd[:,0])), 'max_projected_minus_command_joint1':float(np.max(np.abs(proj[:,0]-cmd[:,0]))), 'q1_delta_rad':float(q[-1,0]-q[0,0]), 'mean_last10_dq1_rad_s':float(np.mean(dq[-10:,0])), 'max_arm_speed_rad_s':float(np.max(np.abs(dq[:,:7]))), 'dt_min_s':float(dt.min()),'dt_max_s':float(dt.max()),'observation_sha256':sha(a.input/(cid+'_observations.json')),'oracle_sha256':sha(a.input/(cid+'_oracle.json'))}
 out={'schema':'task28.g3_p2_4_r2.source_metrics.v1','collector_status':status['status'],'preregistration_sha256':status['preregistration_sha256'],'collector_sha256':status['collector_sha256'],'runtime':status['runtime'],'cases':cases,'global_dt_min_s':float(min(all_dts)),'global_dt_max_s':float(max(all_dts)),'interpretation':'applied readback exactly reproduces explicit command buffer; projected effort is retained as solver comparison and is not called a hardware-equivalent torque sensor.'}
 a.output.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
 print(json.dumps({'status':'SOURCE_METRICS_COMPLETE','cases':list(cases)},indent=2))
if __name__=='__main__': main()
