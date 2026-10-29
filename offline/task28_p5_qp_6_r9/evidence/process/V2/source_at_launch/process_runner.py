"""One owned process group at a time; bounded cleanup, no global process kill."""
import argparse, datetime, fcntl, hashlib, json, os, shutil, signal, subprocess, tempfile, time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
ISAAC=Path('/home/ubuntu2004/isaacsim-4.5.0')

def write(path,value):
    path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')

def processes():
    out=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            text=(p/'stat').read_text(); tail=text[text.rindex(')')+2:].split()
            argv=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace').strip()
            out.append({'pid':int(p.name),'ppid':int(tail[1]),'pgid':int(tail[2]),'state':tail[0],'start_ticks':int(tail[19]),'argv':argv})
        except (OSError,ValueError,IndexError): pass
    return out

def competing(rows):
    return [r for r in rows if r['pid']!=os.getpid() and r['state']!='Z' and any(x in r['argv'] for x in ('/isaacsim-4.5.0/kit/python/bin/python3','/kit/kit ','ros2 launch ','ros2 run ','roscore','runtime_virtual --configuration'))]

def run(configuration, timeout=300):
    logdir=HERE/'evidence/process'/configuration
    if logdir.exists(): raise FileExistsError('NO_RERUN:'+str(logdir))
    logdir.mkdir(parents=True)
    journal={'configuration':configuration,'parent_pid':os.getpid(),'start_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'preflight_competing':competing(processes()),'signals_sent':[],'owned_samples':[]}
    def save(): write(logdir/'lifecycle.json',journal)
    save()
    if journal['preflight_competing']:
        journal.update(status='BLOCKED_COMPETING_PROCESS',isolation_verified=False);save();return 2
    # Cross-invocation serialization persists until child cleanup and evidence capture finish.
    lock=(HERE/'evidence/runner.lock').open('a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: raise RuntimeError('R9_RUN_ALREADY_ACTIVE')
    for prior in (HERE/'evidence/process').glob('*/lifecycle.json'):
        old=json.loads(prior.read_text())
        if prior.parent!=logdir and not (old.get('status')=='EXITED' and old.get('residual_owned')==[] and old.get('portable_resources_removed') is True):
            raise RuntimeError('PREVIOUS_PROCESS_EXIT_NOT_VERIFIED:'+str(prior))
    portable=Path(tempfile.mkdtemp(prefix='task28-r9-'+configuration+'-'))
    output=HERE/'evidence/diagnostic'/configuration
    # Same setup as installed python.sh, with exec preserving the actual worker exit/signal.
    script='set -e\nexport CARB_APP_PATH=/home/ubuntu2004/isaacsim-4.5.0/kit\nexport ISAAC_PATH=/home/ubuntu2004/isaacsim-4.5.0\nexport EXP_PATH=/home/ubuntu2004/isaacsim-4.5.0/apps\nsource /home/ubuntu2004/isaacsim-4.5.0/setup_python_env.sh\nexport LD_PRELOAD=/home/ubuntu2004/isaacsim-4.5.0/kit/libcarb.so\nexec /home/ubuntu2004/isaacsim-4.5.0/kit/python/bin/python3 "$@"'
    argv=['/bin/bash','-c',script,'r9-worker','-m','offline.task28_p5_qp_6_r9.runtime_virtual','--configuration',configuration,'--output',str(output),'--portable-root',str(portable),'--/app/tokens/appcache='+str(portable/'appcache')]
    env=os.environ.copy(); env['PYTHONPATH']=str(ROOT)+':'+str(ROOT/'offline/task28_g1_1')+':'+env.get('PYTHONPATH','');env['PYTHONDONTWRITEBYTECODE']='1'
    journal.update(argv=argv,portable_root=str(portable),source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.is_file() and p.suffix in ('.py','.json','.md')})
    snapshots=logdir/'source_at_launch';snapshots.mkdir()
    for p in HERE.iterdir():
        if p.is_file() and p.suffix in ('.py','.json','.md'): shutil.copy2(p,snapshots/p.name)
    start=time.monotonic()
    with (logdir/'stdout_stderr.log').open('w') as log:
        child=subprocess.Popen(argv,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        pgid=child.pid
        journal.update(child_pid=child.pid,pgid=pgid,start_monotonic_s=start);save()
        try:
            while child.poll() is None and time.monotonic()-start < timeout:
                owned=[r for r in processes() if r['pgid']==pgid]
                journal['owned_samples'].append({'elapsed_s':time.monotonic()-start,'members':owned});save()
                time.sleep(2)
        finally:
            for sig,grace in ((signal.SIGTERM,15),(signal.SIGKILL,10)):
                owned=[r for r in processes() if r['pgid']==pgid and r['state']!='Z']
                if not owned: break
                # Session was created by this Popen; its unreaped leader prevents PID reuse.
                if child.poll() is None or any(r['pgid']==pgid for r in owned):
                    os.killpg(pgid,sig)
                    journal['signals_sent'].append({'signal':sig.name,'owned_members':owned});save()
                    end=time.monotonic()+grace
                    while time.monotonic()<end:
                        child.poll()
                        if not any(r['pgid']==pgid and r['state']!='Z' for r in processes()):break
                        time.sleep(.2)
            child.wait(timeout=10)
    residual=[r for r in processes() if r['pgid']==pgid and r['state']!='Z']
    logs=list(portable.rglob('kit_*.log'))
    for i,p in enumerate(logs): shutil.copy2(p,logdir/(str(i)+'_'+p.name))
    logtext=(logdir/'stdout_stderr.log').read_text(errors='replace')+'\n'+'\n'.join(p.read_text(errors='replace') for p in logs)
    result=json.loads((output/'result.json').read_text()) if (output/'result.json').exists() else {}
    tokens=result.get('kit_tokens',{})
    child_events_path=output/'child_lifecycle.jsonl'
    child_events=[json.loads(x) for x in child_events_path.read_text().splitlines()] if child_events_path.exists() else []
    if not tokens:
        tokens=next((x['tokens'] for x in child_events if x['event']=='CACHE_ISOLATION_VERIFIED'),{})
    # Per-run portable root must be effective for shared application cache.
    cache_isolated=bool(all(tokens.get(k,'').startswith(str(portable)+'/') for k in ('appcache','cache','logs','data')))
    journal.update(end_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),elapsed_s=time.monotonic()-start,exit_code=child.returncode,signal=signal.Signals(-child.returncode).name if child.returncode<0 else None,residual_owned=residual,postflight_competing=competing(processes()),kit_tokens=tokens,cache_isolated=cache_isolated,kvdb_lock_warning='another kit process is locking it' in logtext,normal_shutdown_marker='Simulation App Shutting Down' in logtext,controller_status=result.get('status','NO_RESULT'),status='EXITED')
    journal['isolation_verified']=bool(not residual and not journal['postflight_competing'] and cache_isolated and not journal['kvdb_lock_warning'])
    journal['clean_exit']=child.returncode==0 and not journal['signals_sent']
    if not result:
        output.mkdir(parents=True,exist_ok=True)
        write(output/'result.json',{'status':'PROCESS_ABORTED','configuration':configuration,'actual_effort_commands':0,'robot_command_writes':0,'recorded_frames':0,'first_failure':{'phase':'INITIALIZATION','exit_code':child.returncode,'signal':journal['signal']}})
    # Delete only this runner's unique cache after full process exit. Logs are already copied.
    if not residual:
        shutil.rmtree(portable);journal['portable_resources_removed']=not portable.exists()
    else: journal['portable_resources_removed']=False
    save();print(json.dumps({k:journal[k] for k in ('configuration','exit_code','signal','clean_exit','isolation_verified','controller_status','elapsed_s')}),flush=True)
    return 0 if journal['isolation_verified'] else 2

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--configuration',choices=['V1','V1_RECOVERY','V1_RECOVERY2','V2'],required=True);p.add_argument('--timeout',type=float,default=300)
    a=p.parse_args();raise SystemExit(run(a.configuration,a.timeout))
