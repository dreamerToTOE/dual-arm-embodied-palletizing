"""R13 complete regression receipt: prior suites plus R13 focused tests."""
from __future__ import annotations
import hashlib, json, os, re, subprocess, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=HERE/'evidence/regressions'

def run(argv,name,expected,env=None):
    env={**os.environ,**(env or {}),'PYTHONDONTWRITEBYTECODE':'1'}
    start=time.monotonic(); p=subprocess.run(argv,cwd=ROOT,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    out=p.stdout; (OUT/f'{name}.log').write_text(out)
    nums=re.findall(r'(?:^|\s)(\d+) passed',out); passed=int(nums[-1]) if nums else 0
    row={'name':name,'argv':argv,'exit_code':p.returncode,'elapsed_s':time.monotonic()-start,'passed':passed,'expected':expected,'pass':p.returncode==0 and passed==expected,'log_sha256':hashlib.sha256(out.encode()).hexdigest()}
    (OUT/f'{name}.json').write_text(json.dumps(row,indent=2)+'\n'); print(f'{name}: {passed}/{expected}',flush=True); return row

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    from offline.task28_p5_qp_6_r8 import run_regressions as frozen
    frozen.OUT=OUT/'pre_r8'; frozen.main(); pre=json.loads((frozen.OUT/'summary.json').read_text())
    prior_env=json.loads((ROOT/'offline/task28_p5_qp_6_r4/evidence/regressions/historical.json').read_text())['PYTHONPATH']
    env={'PYTHONPATH':prior_env}
    r8=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r8/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r8_focused',15,env)
    r9=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r9/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r9_focused',17)
    r12=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r12/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r12_focused',19)
    r13=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r13/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r13_focused',4)
    prior=pre['total_passed']+pre['r7_focused_separate']['passed']+r8['passed']+r9['passed']
    ok=pre['all_requested_historical_pass'] and r8['pass'] and r9['pass'] and r12['pass'] and r13['pass']
    summary={'schema':'TASK28_R13_PRIOR_REGRESSION_SUMMARY_V1','status':'PASS' if ok else 'FAIL','historical_prior':prior,'r12_focused_passed':r12['passed'],'r13_focused_passed':r13['passed'],'new_total_including_r12_r13':prior+r12['passed']+r13['passed'],'expected_prior':1051,'expected_r12_focused':19,'expected_r13_focused':4,'physics_launched':False,'runs':{'pre_r8':pre,'r8_focused':r8,'r9_focused':r9,'r12_focused':r12,'r13_focused':r13}}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n'); print(json.dumps({'status':summary['status'],'prior':prior,'r12':r12['passed'],'r13':r13['passed']})); return 0 if ok else 1
if __name__=='__main__': raise SystemExit(main())
