"""Fresh isolated Isaac processes running the unmodified R12 runtime.

Only output locations and process environment are R13-owned. R12 source,
registrations, scene, gates, physics step and controller are read unchanged.
"""
from __future__ import annotations
import argparse, datetime, fcntl, hashlib, json, os, shutil, time
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r12 import process_runner, evaluate

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
BASE=HERE.parent/'task28_p5_qp_6_r12'
OUT=HERE/'evidence/isaac_repetitions'
BASELINE='52d07e6a9c4ac482db4887483cbf8ab0cbf510d1'

def write(p,d):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(d,indent=2,sort_keys=True,allow_nan=False)+'\n')

def source_hashes():
    paths=list(BASE.glob('*.py'))+list(BASE.glob('*.json'))
    return {str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}

def metrics(ev):
    r=ev.get('virtual_tracking_records',[])
    e=np.array([max(x['object_left_error_m'],x['object_right_error_m']) for x in r])
    rel=np.array([x['relative_error_m'] for x in r])
    if not len(r): return {'status':'NO_COMPLETE_TRACE'}
    return {'object_peak_m':float(e.max()),'object_mean_m':float(e.mean()),
            'object_p95_m':float(np.percentile(e,95,method='linear')),
            'relative_peak_m':float(rel.max()),
            'object_over_frames':[x['physics_step'] for x,y in zip(r,e) if y>0.00025],
            'relative_over_frames':[x['physics_step'] for x,y in zip(r,rel) if y>0.00015],
            'sample_count':len(r)}

def run(repeats):
    OUT.mkdir(parents=True,exist_ok=True)
    lock=(OUT/'runner.lock').open('a'); fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    manifest=OUT/'plan.json'
    plan={'schema':'TASK28_R13_FRESH_ISAAC_PLAN_V1','baseline_head':BASELINE,
          'runtime_module':'offline.task28_p5_qp_6_r12.runtime_virtual',
          'repeat_count_per_cell':repeats,'order':'interleaved A B C, then next repeat',
          'controller_modified':False,'physical_cube_motion':False,'V10_D_or_FORMAL':False,
          'environment':{k:os.environ.get(k) for k in ['ROS_DISTRO','OPENBLAS_NUM_THREADS','OMP_NUM_THREADS']},
          'source_hashes':source_hashes(),
          'scope':'fresh independent CPU_NUMPY Isaac scenes; simulated arm torque only; ROS graph suppressed by frozen R12 runtime'}
    if manifest.exists():
        assert json.loads(manifest.read_text())==plan,'PLAN_CHANGED'
    else: write(manifest,plan)
    results=[]
    for i in range(1,repeats+1):
        for cell in ('V10-A','V10-B','V10-C'):
            trial=OUT/f'{i:02d}_{cell}'
            receipt=trial/'receipt.json'
            if receipt.exists():
                old=json.loads(receipt.read_text()); assert old['completed'],'PREVIOUS_INCOMPLETE'
                results.append(old); continue
            assert source_hashes()==plan['source_hashes'],'R12_SOURCE_CHANGED'
            trial.mkdir(exist_ok=True)
            for p in BASE.iterdir():
                if p.is_file() and p.suffix in ('.py','.json','.md'): shutil.copy2(p,trial/p.name)
            process_runner.HERE=trial
            started=time.monotonic()
            code=process_runner.run(cell,timeout=300)
            evaluate.HERE=trial
            ev=evaluate.eval_cell(cell)
            life=json.loads((trial/'evidence/process'/cell/'lifecycle.json').read_text())
            result=json.loads((trial/'evidence/diagnostic'/cell/'result.json').read_text())
            hashes={str(p.relative_to(trial)):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (trial/'evidence/diagnostic'/cell).iterdir() if p.is_file()}
            completed=(code==0 and life['isolation_verified'] and life['clean_exit']
                       and result.get('status')=='NO_CONTACT_R12_VIRTUAL_COOPERATION_COMPLETE'
                       and ev.get('commands_per_arm')==210)
            rec={'schema':'TASK28_R13_FRESH_ISAAC_RUN_V1','repeat':i,'cell':cell,
                 'completed':completed,'returncode':code,'child_pid':life.get('child_pid'),
                 'start_utc':life.get('start_utc'),'elapsed_s':time.monotonic()-started,
                 'process_clean_exit':life['clean_exit'],'process_isolated':life['isolation_verified'],
                 'evaluation':ev,'metrics':metrics(ev),'output_hashes':hashes,
                 'source_unchanged':source_hashes()==plan['source_hashes']}
            write(receipt,rec);results.append(rec)
            write(OUT/'progress.json',{'completed':len(results),'planned':3*repeats,
                  'runs':[{'cell':x['cell'],'repeat':x['repeat'],'completed':x['completed'],
                           'status':x['evaluation']['status'],'metrics':x['metrics']} for x in results]})
            print(json.dumps({'repeat':i,'cell':cell,'completed':completed,'status':ev['status'],
                              'metrics':rec['metrics']}),flush=True)
            if not completed: raise RuntimeError('FRESH_RUN_INCOMPLETE:'+str(trial))
    return 0
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--repeats',type=int,default=20)
    a=ap.parse_args();raise SystemExit(run(a.repeats))
