#!/usr/bin/env python3
from __future__ import annotations
import argparse,hashlib,json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent; CONFIG=HERE/'experiment_preregistration.json'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def evaluate(inp:Path):
 cfg=json.loads(CONFIG.read_text()); status=json.loads((inp/'collector_status.json').read_text()); reasons=[]
 if status.get('preregistration_sha256') != sha(CONFIG): reasons.append('PREREGISTRATION_HASH_MISMATCH')
 rt=status.get('runtime',{})
 if rt.get('all_dof_names') != cfg['expected_all_dof_names']: reasons.append('DOF_ORDER_MISMATCH')
 if rt.get('effort_mode') != ['force']*9: reasons.append('FORCE_MODE_MISMATCH')
 if rt.get('runtime_drive_gains_zero') is not True: reasons.append('RUNTIME_DRIVE_GAINS_NOT_ZERO')
 if status.get('status') != 'COLLECTION_COMPLETE_NOT_EVALUATED': reasons.append('COLLECTION_FAILED')
 op=inp/'zero_load_hold_observations.json'; rows=[]
 if not op.is_file(): reasons.append('MISSING_STABILITY_OBSERVATIONS')
 else:
  rows=json.loads(op.read_text()).get('samples',[])
  if len(rows) != cfg['stability_steps']: reasons.append('STABILITY_SAMPLE_COUNT')
 prev_t=None
 for row in rows:
  try:
   t=float(row['sim_time_s']); q=np.asarray(row['q_rad_or_m'],float); qd=np.asarray(row['dq_rad_s_or_m_s'],float); cmd=np.asarray(row['commanded_effort'],float); app=np.asarray(row['applied_effort_readback'],float)
   if any(x.shape!=(9,) for x in (q,qd,cmd,app)) or not np.isfinite(np.r_[q,qd,cmd,app]).all(): reasons.append('NONFINITE_OR_SHAPE')
   if prev_t is not None and (t<=prev_t or abs((t-prev_t)-cfg['physics_dt_s'])>1e-6): reasons.append('TIMING_GATE')
   if np.max(np.abs(app-cmd))>1e-6: reasons.append('COMMAND_READBACK_MISMATCH')
   if abs(q[0])>cfg['q1_abs_limit_rad'] or np.max(np.abs(qd[:7]))>cfg['arm_speed_limit_rad_s']: reasons.append('BOUNDED_MOTION_LIMIT')
   if float(row['source_read_duration_s'])>cfg['read_latency_budget_s']: reasons.append('SOURCE_LATENCY_LIMIT')
   prev_t=t
  except (KeyError,TypeError,ValueError): reasons.append('INVALID_SAMPLE')
 return {'schema':'task28.g3_p2_4_r3.evaluation.v1','status':'STABILITY_GATE_PASS' if not reasons else 'STABILITY_GATE_REJECTED','gate_pass':not reasons,'reasons':sorted(set(reasons)),'preregistration_sha256':sha(CONFIG),'samples':len(rows),'interpretation':'Stable gravity+PD explicit-effort hold only; no known-load or hardware-equivalence claim.'}
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--input',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); a=ap.parse_args(); out=evaluate(a.input); a.output.write_text(json.dumps(out,indent=2)+'\n'); print(json.dumps(out)); return 0 if out['gate_pass'] else 1
if __name__=='__main__': raise SystemExit(main())
