"""Independent read-only R3/R4 reprocessing for the R5 literature hypothesis."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[1]
def run():
 out={'source_archives':{},'r3':{},'r4':{},'interpretation':{}}
 for n in ('dev_02','dev_03'):
  p=REPO/'offline/task28_p5_qp_6_r4/evidence/historical'/f'{n}.json';d=json.loads(p.read_text());recs=d['records'];tail=recs[-1]
  out['source_archives'][n]={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'records':len(recs),'archive_aliasing_verdict':d.get('archive_aliasing_verdict'),'tail_left_fd_minus_dq_max':max(abs(x) for x in tail['sides']['left']['difference_reported_vs_fd'][:7]),'tail_right_fd_minus_dq_max':max(abs(x) for x in tail['sides']['right']['difference_reported_vs_fd'][:7]),'tail_left_integral_error_max':max(abs(x) for x in tail['sides']['left']['single_step_integral_error']),'tail_right_integral_error_max':max(abs(x) for x in tail['sides']['right']['single_step_integral_error'])}
 p=REPO/'offline/task28_p5_qp_6_r4/evidence/run_A/frames.jsonl';rows=[json.loads(x) for x in p.read_text().splitlines()];errs=[];tail=[];move=[]
 for i in range(1,len(rows)):
  for s in ('left','right'):
   d=rows[i]['arms'][s];q0=np.asarray(rows[i-1]['arms'][s]['q']);q=np.asarray(d['q']);dt=rows[i]['time_s']-rows[i-1]['time_s'];fd=(q-q0)/dt;errs += (fd[:7]-np.asarray(d['dq'][:7])).tolist();move.append(max(abs(q[:7]-q0[:7])))
   if i>=120:tail += (fd[:7]-np.asarray(d['dq'][:7])).tolist()
 out['r4']={'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'frames':len(rows),'moving_samples':int(sum(x>1e-4 for x in move)),'all_fd_minus_dq_max':float(max(abs(x) for x in errs)),'tail_fd_minus_dq_max':float(max(abs(x) for x in tail)),'tail_end_dq_left':rows[-1]['arms']['left']['dq'][:7],'tail_end_dq_right':rows[-1]['arms']['right']['dq'][:7]}
 out['interpretation']={'r4_contract_result':'BLOCKED under exact fd(q)/dt=dq source contract','r5_solver_hypothesis':'R5 tests end-step solver velocity as a separate quantity; finite difference is retained as an estimator only','historical_aliasing':'R3 pointer/dtype metadata unavailable; R4 live D run found borrowed buffers mutable'}
 return out
if __name__=='__main__':
 p=ROOT/'evidence'/'historical_r3_r4.json';p.write_text(json.dumps(run(),indent=2,sort_keys=True)+'\n');print(p)
