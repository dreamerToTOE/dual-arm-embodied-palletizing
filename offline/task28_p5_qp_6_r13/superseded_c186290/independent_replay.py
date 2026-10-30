"""Launch independent CPU-only child processes for deterministic R12 replay."""
from __future__ import annotations
import hashlib,json,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
BASE=HERE/'evidence/r12_baseline'

def one(cell):
 d=json.loads((BASE/'evaluation.json').read_text())['cells'][cell]
 rec=d['virtual_tracking_records']
 obj=[max(x['object_left_error_m'],x['object_right_error_m']) for x in rec]
 rel=[x['relative_error_m'] for x in rec]
 s={'cell':cell,'object_peak_mm':max(obj)*1e3,'object_mean_mm':sum(obj)/len(obj)*1e3,'object_p95_mm':sorted(obj)[int(0.95*(len(obj)-1))]*1e3,'relative_peak_mm':max(rel)*1e3,'record_fingerprint':hashlib.sha256(json.dumps(rec,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'pass':max(obj)<=.00025 and max(rel)<=.00015,'execution_authorized':False,'physical_execution':False}
 return s

def main():
 if len(sys.argv)==3 and sys.argv[1]=='--single':
  print(json.dumps(one(sys.argv[2]),sort_keys=True)); return
 out={'schema':'TASK28_R13_INDEPENDENT_REPLAY_V1','baseline_head':'52d07e6a9c4ac482db4887483cbf8ab0cbf510d1','execution_authorized':False,'physical_execution':False,'cells':{}}
 for cell in ('V10-A','V10-B','V10-C'):
  rows=[]
  for i in range(20):
   p=subprocess.run([sys.executable,__file__,'--single',cell],cwd=HERE.parents[2],text=True,capture_output=True,check=False)
   parsed=json.loads(p.stdout)
   parsed['repetition']=i+1; parsed['exit_code']=p.returncode; rows.append(parsed)
  out['cells'][cell]={'repetitions':len(rows),'independent_processes':len(rows),'all_exit_zero':all(x['exit_code']==0 for x in rows),'identical_record_fingerprint':len({x['record_fingerprint'] for x in rows})==1,'all_acceptance_pass':all(x['pass'] for x in rows),'records':rows}
 path=HERE/'evidence/independent_replay.json'; path.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n'); print(json.dumps({c:{k:v for k,v in d.items() if k!='records'} for c,d in out['cells'].items()},indent=2))
if __name__=='__main__': main()
