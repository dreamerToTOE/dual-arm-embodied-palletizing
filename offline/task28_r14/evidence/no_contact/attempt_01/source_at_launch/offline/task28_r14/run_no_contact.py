"""One fresh isolated process for the R14 no-contact preflight."""
from __future__ import annotations
import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
from offline.task28_p5_qp_6_r12.process_runner import processes, competing
from .no_contact import HERE, ROOT, plan, sha, write


def main():
    root_out = HERE / 'evidence/no_contact'
    root_out.mkdir(parents=True, exist_ok=True)
    attempts = sorted(root_out.glob('attempt_*'))
    out = root_out / f'attempt_{len(attempts)+1:02d}'
    out.mkdir(parents=True, exist_ok=False)
    registration = root_out/'preregistration.json'
    write(registration, plan())
    state = {'schema':'TASK28_R14_PROCESS_V1', 'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'preflight_competing': competing(processes()), 'signals_sent': []}
    def save(): write(out/'lifecycle.json', state)
    save()
    if state['preflight_competing']:
        raise RuntimeError('COMPETING_SIMULATION_OR_ROS')
    portable = Path(tempfile.mkdtemp(prefix='task28-r14-stage1-'))
    script = '''set -e
export CARB_APP_PATH=/home/ubuntu2004/isaacsim-4.5.0/kit
export ISAAC_PATH=/home/ubuntu2004/isaacsim-4.5.0
export EXP_PATH=/home/ubuntu2004/isaacsim-4.5.0/apps
source /home/ubuntu2004/isaacsim-4.5.0/setup_python_env.sh
export LD_PRELOAD=/home/ubuntu2004/isaacsim-4.5.0/kit/libcarb.so
exec /home/ubuntu2004/isaacsim-4.5.0/kit/python/bin/python3 "$@"
'''
    argv = ['/bin/bash','-c',script,'r14-stage1','-m','offline.task28_r14.no_contact',
            '--output',str(out/'raw'),'--portable-root',str(portable),'--plan',str(registration)]
    env = {**os.environ,'PYTHONPATH':str(ROOT)+':'+str(ROOT/'offline/task28_g1_1'),
           'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'}
    state.update(argv=argv, portable_root=str(portable), source_hashes=plan()['sources'])
    snap = out/'source_at_launch'; snap.mkdir()
    for rel in list(state['source_hashes']) + [str(Path(__file__).resolve().relative_to(ROOT)), 'offline/task28_p2_4/observer.py']:
        src = ROOT/rel; dest = snap/rel; dest.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(src,dest)
    started = time.monotonic()
    with (out/'stdout_stderr.log').open('w') as log:
        child = subprocess.Popen(argv,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        state.update(child_pid=child.pid, pgid=child.pid); save()
        try:
            child.wait(timeout=240)
        except BaseException as exc:
            state['parent_exception'] = type(exc).__name__+':'+str(exc)
        finally:
            for sig, grace in ((signal.SIGTERM,10),(signal.SIGKILL,5)):
                if not any(p['pgid']==child.pid and p['state']!='Z' for p in processes()): break
                os.killpg(child.pid,sig); state['signals_sent'].append(sig.name); save()
                try: child.wait(timeout=grace)
                except subprocess.TimeoutExpired: pass
            child.wait(timeout=5)
    residual = [p for p in processes() if p['pgid']==child.pid and p['state']!='Z']
    for i,p in enumerate(portable.rglob('kit_*.log')):
        shutil.copy2(p,out/(str(i)+'_'+p.name))
    scope = json.loads((out/'runtime_scope.json').read_text()) if (out/'runtime_scope.json').exists() else {}
    tokens = scope.get('kit_tokens',{})
    state.update(exit_code=child.returncode, elapsed_s=time.monotonic()-started, residual_owned=residual,
                 postflight_competing=competing(processes()), kit_tokens=tokens,
                 cache_isolated=len(tokens)==4 and all(v.startswith(str(portable)+'/') for v in tokens.values()),
                 clean_exit=child.returncode==0 and not state['signals_sent'],
                 ended_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
    state['sources_unchanged'] = all(sha(ROOT/p)==h for p,h in state['source_hashes'].items())
    if not residual: shutil.rmtree(portable)
    state['portable_resources_removed'] = not portable.exists()
    state['isolation_verified'] = state['cache_isolated'] and not residual and not state['postflight_competing']
    save()
    print(json.dumps({k:state[k] for k in ('exit_code','clean_exit','isolation_verified','elapsed_s')}),flush=True)
    return 0 if state['clean_exit'] and state['isolation_verified'] and state['sources_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
