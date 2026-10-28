#!/usr/bin/env python3
"""Known-load causal online wrench acceptance evaluator for R3."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
try:
    from offline.task28_g1_1.task28_validator.model import RobotModel
except ModuleNotFoundError:
    from task28_validator.model import RobotModel

HERE=Path(__file__).resolve().parent; CONFIG=HERE/'experiment_preregistration.json'
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def evaluate(inp:Path,out:Path):
 cfg=json.loads(CONFIG.read_text()); status=json.loads((inp/'collector_status.json').read_text()); reasons=[]
 if status.get('preregistration_sha256')!=sha(CONFIG): reasons.append('PREREGISTRATION_HASH_MISMATCH')
 if status.get('status')!='COLLECTION_COMPLETE_NOT_EVALUATED': reasons.append('COLLECTION_FAILED')
 model=RobotModel(); cases={}; max_joint=0.; max_force=0.; max_moment=0.; rank_min=99; cond_max=0.
 for cid in cfg['online_cases']:
  op=inp/(cid+'_observations.json'); xp=inp/(cid+'_oracle.json'); cr=[]; rows=[]
  if not op.is_file() or not xp.is_file(): cases[cid]={'pass':False,'reasons':['MISSING_FILES'],'sample_count':0}; reasons.append('MISSING_FILES:'+cid); continue
  obs=json.loads(op.read_text()); oracle=json.loads(xp.read_text()); data=obs.get('samples',[]); points=np.asarray(oracle.get('application_points_world_m',[]),float); force=np.asarray(oracle.get('force_world_n',[np.nan]*3),float)
  prev_t=None; prev_dq=None; records=[]
  for k,row in enumerate(data):
   try:
    t=float(row['sim_time_s']); q=np.asarray(row['q_rad_or_m'],float); dq=np.asarray(row['dq_rad_s_or_m_s'],float); cmd=np.asarray(row['commanded_effort'],float); M=np.asarray(row['mass_matrix'],float); c=np.asarray(row['coriolis_compensation'],float); g=np.asarray(row['gravity_compensation'],float)
    if prev_t is not None:
     dt=t-prev_t; qdd=(dq-prev_dq)/dt; ext=M@qdd+c+g-cmd; J=model.jacobian('left',q[:7]); tcp=model.fk('left',q[:7])[:3,3]; app=points[k] if len(points)>k else np.full(3,np.nan); w=np.r_[force,np.cross(app-tcp,force)]; expected=J.T@w; err=ext[:7]-expected; rank=int(np.linalg.matrix_rank(J.T)); s=np.linalg.svd(J.T,compute_uv=False); cond=float(s[0]/s[-1]) if s[-1]>1e-12 else float('inf'); w_hat=np.linalg.lstsq(J.T,ext[:7],rcond=1e-10)[0]; wrench_err=w_hat-w; joint=float(np.max(np.abs(err))); ferr=float(np.linalg.norm(wrench_err[:3])); merr=float(np.linalg.norm(wrench_err[3:])); max_joint=max(max_joint,joint); max_force=max(max_force,ferr); max_moment=max(max_moment,merr); rank_min=min(rank_min,rank); cond_max=max(cond_max,cond); records.append({'step':k,'joint_error_inf_nm':joint,'wrench_estimate':w_hat.tolist(),'oracle_wrench':w.tolist(),'force_error_n':ferr,'moment_error_nm':merr,'rank':rank,'condition':cond,'qdd_arm':qdd[:7].tolist()})
    prev_t=t; prev_dq=dq
   except (KeyError,TypeError,ValueError, np.linalg.LinAlgError) as exc: cr.append('INVALID_SAMPLE:'+type(exc).__name__)
  if len(data)!=cfg['online_steps']: cr.append('SAMPLE_COUNT_MISMATCH')
  if records and (max(r['joint_error_inf_nm'] for r in records)>0.2 or max(r['force_error_n'] for r in records)>0.5 or max(r['moment_error_nm'] for r in records)>0.2): cr.append('KNOWN_WRENCH_GATE')
  cases[cid]={'pass':not cr,'reasons':sorted(set(cr)),'sample_count':len(data),'evaluated_rows':len(records),'max_joint_error_inf_nm':max((r['joint_error_inf_nm'] for r in records),default=None),'max_force_error_n':max((r['force_error_n'] for r in records),default=None),'max_moment_error_nm':max((r['moment_error_nm'] for r in records),default=None),'min_rank':min((r['rank'] for r in records),default=None),'max_condition':max((r['condition'] for r in records),default=None),'observation_sha256':sha(op),'oracle_sha256':sha(xp),'records':records}
 if any(not x['pass'] for x in cases.values()): reasons.append('CASE_GATE_FAILURE')
 result={'schema':'task28.g3_p2_4_r3.online_evaluation.v1','status':'ONLINE_EXPLICIT_CANDIDATE_PASS' if not reasons else 'ONLINE_EXPLICIT_REJECTED','gate_pass':not reasons,'preregistration_sha256':sha(CONFIG),'collector_sha256':status.get('collector_sha256'),'cases':cases,'global_metrics':{'max_joint_error_inf_nm':max_joint,'max_force_error_n':max_force,'max_moment_error_nm':max_moment,'min_rank':rank_min if rank_min<99 else None,'max_condition':cond_max},'reasons':sorted(set(reasons)),'interpretation':'Causal model-based candidate evaluated against independent known-load oracle; not real tau_J or physical contact-force validation.'}
 out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n'); return result
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--input',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); a=ap.parse_args(); r=evaluate(a.input,a.output); print(json.dumps({'status':r['status'],'gate_pass':r['gate_pass'],'reasons':r['reasons']})); return 0 if r['gate_pass'] else 1
if __name__=='__main__': raise SystemExit(main())
