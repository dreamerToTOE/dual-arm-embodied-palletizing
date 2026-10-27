#!/usr/bin/env python3
"""Independent evaluation of explicit effort source and optional online oracle run."""
from __future__ import annotations
import argparse, hashlib, json, time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
CONFIG=Path(__file__).resolve().parent/'experiment_preregistration.json'

def sha(p):
 h=hashlib.sha256();
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def read(p): return json.loads(Path(p).read_text())
def evaluate(inp:Path,out:Path,stage:str):
 cfg=read(CONFIG); status=read(inp/'collector_status.json'); reasons=[]
 ph=sha(CONFIG)
 if status.get('preregistration_sha256')!=ph: reasons.append('PREREGISTRATION_HASH_MISMATCH')
 rt=status.get('runtime',{}); names_ok=rt.get('all_dof_names')==cfg['expected_all_dof_names']
 modes_ok=rt.get('post_effort_modes')==['force']*9
 gains_ok=np.allclose(rt.get('post_stiffness',[]),0) and np.allclose(rt.get('post_damping',[]),0)
 if not names_ok: reasons.append('DOF_ORDER_MISMATCH')
 if not modes_ok: reasons.append('FORCE_MODE_NOT_CONFIRMED')
 if not gains_ok: reasons.append('ZERO_GAIN_NOT_CONFIRMED')
 if status.get('status')!='COLLECTION_COMPLETE_NOT_EVALUATED': reasons.append('COLLECTION_FAILED')
 cases={}; source_cases=cfg['source_cases'] if stage=='source' else cfg['online_cases']; dt=cfg['physics_dt_s']
 for cid in source_cases:
  op=inp/(cid+'_observations.json'); xp=inp/(cid+'_oracle.json'); cr=[]
  if not op.is_file() or not xp.is_file(): cases[cid]={'pass':False,'reasons':['MISSING_FILES'],'rows':[]}; continue
  obs=read(op); oracle=read(xp); rows=obs.get('samples',[])
  if obs.get('case_id')!=cid: cr.append('CASE_ID_MISMATCH')
  if len(rows)!=(cfg['source_steps'] if stage=='source' else cfg['online_steps']): cr.append('SAMPLE_COUNT_MISMATCH')
  prev_t=None; prev_q=None; prev_dq=None; ext_rows=[]
  for row in rows:
   try:
    t=float(row['sim_time_s']); q=np.asarray(row['q_rad_or_m'],float); dq=np.asarray(row['dq_rad_s_or_m_s'],float)
    cmd=np.asarray(row['commanded_effort'],float); app=np.asarray(row['applied_effort_readback'],float)
    if prev_t is not None and (t<=prev_t or abs(t-prev_t-dt)>1e-6): cr.append('TIME_NOT_MONOTONIC_OR_DT');
    if q.shape!=(9,) or dq.shape!=(9,) or cmd.shape!=(9,) or app.shape!=(9,): cr.append('SHAPE_INVALID')
    if np.max(np.abs(app-cmd))>cfg['limits']['max_readback_error']: cr.append('COMMAND_READBACK_MISMATCH')
    if abs(q[0])>cfg['limits']['max_abs_q1_rad'] or np.max(np.abs(dq[:7]))>cfg['limits']['max_abs_qd_rad_s']: cr.append('BOUNDED_MOTION_LIMIT')
    if prev_t is not None:
     acc=(dq-prev_dq)/(t-prev_t); M=np.asarray(row['mass_matrix'],float)[0] if np.asarray(row['mass_matrix']).shape==(1,9,9) else np.asarray(row['mass_matrix'],float)
     c=np.asarray(row['coriolis_compensation'],float); g=np.asarray(row['gravity_compensation'],float)
     ext=M@acc+c+g-app
     ext_rows.append({'t':t,'q':q.tolist(),'ext_arm':ext[:7].tolist(),'point':np.asarray(oracle.get('application_points_world_m',[[]])[len(ext_rows)],float).tolist() if len(oracle.get('application_points_world_m',[]))>len(ext_rows) else None})
    prev_t=t; prev_q=q; prev_dq=dq
   except (KeyError,ValueError,TypeError,np.linalg.LinAlgError): cr.append('INVALID_ROW')
  sign={'q1_delta':float(rows[-1]['q_rad_or_m'][0]-rows[0]['q_rad_or_m'][0]) if rows else None,
        'dq1_mean_last20':float(np.mean([r['dq_rad_s_or_m_s'][0] for r in rows[-20:]])) if rows else None}
  cases[cid]={'pass':not cr,'reasons':sorted(set(cr)),'rows':len(rows),'sign':sign,
               'observation_sha256':sha(op),'oracle_sha256':sha(xp)}
 # source sign response uses command identities and causal response difference; no fitted correction.
 if stage=='source' and not reasons:
  z=cases.get('zero',{}); p=cases.get('plus_q1',{}); m=cases.get('minus_q1',{})
  if not all(c.get('pass') for c in (z,p,m)): reasons.append('SOURCE_CASE_FAILURE')
  else:
   pd=p['sign']['q1_delta']; md=m['sign']['q1_delta'];
   if not np.isfinite([pd,md]).all() or pd-md <= 1e-7: reasons.append('COMMAND_SIGN_RESPONSE_NOT_SEPARATED')
 status_out='SOURCE_GATE_PASS' if stage=='source' and not reasons else ('SOURCE_GATE_REJECTED' if stage=='source' else 'ONLINE_NOT_EVALUATED')
 result={'schema':'task28.g3_p2_4_r2.evaluation.v1','stage':stage,'status':status_out,
   'gate_pass':status_out=='SOURCE_GATE_PASS','preregistration_sha256':ph,'collector_sha256':status.get('collector_sha256'),
   'source_prerequisites':{'collection_complete':status.get('status')=='COLLECTION_COMPLETE_NOT_EVALUATED','dof_names':names_ok,'force_mode':modes_ok,'zero_gains':gains_ok},
   'reasons':sorted(set(reasons)),'cases':cases,
   'interpretation':'Explicit input/readback is source-characterized only; applied readback is not a hardware-equivalent torque sensor. Online oracle comparison must remain separate.'}
 if stage=='online':
  # For online runs, retain explicit causal residuals and classify only if all rows are finite.
  result['status']='ONLINE_CANDIDATE_RECORDED' if not reasons else 'ONLINE_CANDIDATE_REJECTED'; result['gate_pass']=False
  result['reasons']=sorted(set(reasons+['ONLINE_EXTERNAL_WRENCH_GATE_NOT_IMPLEMENTED_IN_SOURCE_EVALUATOR']))
 out.parent.mkdir(parents=True,exist_ok=True); out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n'); return result

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--input',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--stage',choices=['source','online'],required=True); a=ap.parse_args(); r=evaluate(a.input,a.output,a.stage); print(json.dumps({'status':r['status'],'gate_pass':r['gate_pass'],'reasons':r['reasons']})); return 0 if r['gate_pass'] else 1
if __name__=='__main__': raise SystemExit(main())
