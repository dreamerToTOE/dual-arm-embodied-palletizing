"""R11 regression receipt: frozen pre-R9 1034 + R9 focused 17 + R11 focused tests."""
from __future__ import annotations
import hashlib, json, os, re, subprocess, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
OUT=HERE/'evidence/regressions'

def run(argv, name, expected, env=None):
    env={**os.environ, **(env or {}), 'PYTHONDONTWRITEBYTECODE':'1'}
    start=time.monotonic(); p=subprocess.run(argv,cwd=ROOT,env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    out=p.stdout; (OUT/f'{name}.log').write_text(out)
    nums=re.findall(r'(?:^|\s)(\d+) passed',out); passed=int(nums[-1]) if nums else 0
    row={'name':name,'argv':argv,'exit_code':p.returncode,'elapsed_s':time.monotonic()-start,'passed':passed,'expected':expected,'pass':p.returncode==0 and passed==expected,'log_sha256':hashlib.sha256(out.encode()).hexdigest()}
    (OUT/f'{name}.json').write_text(json.dumps(row,indent=2)+'\n'); print(f'{name}: {passed}/{expected}',flush=True); return row

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    from offline.task28_p5_qp_6_r8 import run_regressions as frozen
    frozen.OUT=OUT/'pre_r8'; frozen.main(); pre=json.loads((frozen.OUT/'summary.json').read_text())
    hist=json.loads((ROOT/'offline/task28_g2_p5_qp_5'/'evidence/regressions/historical.json').read_text()) if (ROOT/'offline/task28_g2_p5_qp_5'/'evidence/regressions/historical.json').exists() else None
    # Historical R8 focused tests use their archived compatibility environment.
    prior_env=json.loads((ROOT/'offline/task28_p5_qp_6_r4/evidence/regressions/historical.json').read_text())['PYTHONPATH']
    env={'PYTHONPATH':prior_env}
    r8=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r8/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r8_focused',15,env)
    r9=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r9/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r9_focused',17)
    r10=run(['/usr/bin/python3','-m','pytest','offline/task28_p5_qp_6_r11/tests','-q','-p','no:cacheprovider','--import-mode=importlib'],'r11_focused',19)
    total=pre['total_passed']+pre['r7_focused_separate']['passed']+r8['passed']+r9['passed']
    summary={'schema':'TASK28_R11_PRIOR_REGRESSION_SUMMARY_V1','status':'PASS' if pre['all_requested_historical_pass'] and r8['pass'] and r9['pass'] and r10['pass'] else 'FAIL','historical_1034':total,'r11_focused_passed':r10['passed'],'new_total_including_r11':total+r10['passed'],'expected_prior':1051,'expected_r11_focused':19,'physics_launched':False,'runs':{'pre_r8':pre,'r8_focused':r8,'r9_focused':r9,'r11_focused':r10}}
    (OUT/'summary.json').write_text(json.dumps(summary,indent=2)+'\n'); print(json.dumps({'status':summary['status'],'prior':total,'r11':r10['passed']})); return 0 if summary['status']=='PASS' else 1
if __name__=='__main__': raise SystemExit(main())
