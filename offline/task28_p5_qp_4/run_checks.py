"""CPU-only regression runner; writes new task evidence, never historical logs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--suite', choices=['historical','focused'], required=True)
    ap.add_argument('--output', type=Path, required=True)
    a = ap.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    all_dirs = sorted(ROOT.glob('offline/task28_*/tests'))
    selected = [p for p in all_dirs if (p.parent.name == 'task28_p5_qp_4') == (a.suite == 'focused')]
    paths = ['.', 'offline'] + [str(p.parent.relative_to(ROOT)) for p in all_dirs] + [str(p.relative_to(ROOT)) for p in all_dirs]
    changes = {'PYTHONPATH': ':'.join(paths), 'PYTHONDONTWRITEBYTECODE':'1', 'PYTEST_DISABLE_PLUGIN_AUTOLOAD':'1', 'OPENBLAS_NUM_THREADS':'1', 'OMP_NUM_THREADS':'1'}
    argv = [sys.executable,'-m','pytest',*[str(p.relative_to(ROOT)) for p in selected],'-q','-s','-p','no:cacheprovider','--import-mode=importlib']
    t = time.perf_counter()
    with (a.output/'pytest.log').open('w') as f:
        r = subprocess.run(argv, cwd=ROOT, env={**os.environ,**changes}, stdout=f, stderr=subprocess.STDOUT)
    log = (a.output/'pytest.log').read_bytes()
    record = {'suite':a.suite,'argv':argv,'environment':changes,'python':sys.version,'platform':platform.platform(),'exit_code':r.returncode,'elapsed_s':time.perf_counter()-t,'log_sha256':hashlib.sha256(log).hexdigest(),'result_tail':log.decode().splitlines()[-10:],'execution_authorized':False}
    (a.output/'record.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    print(json.dumps({'suite':a.suite,'exit_code':r.returncode,'result':record['result_tail'][-1]}))
    return r.returncode

if __name__ == '__main__':
    raise SystemExit(main())
