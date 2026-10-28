"""Produce a new CPU audit bundle; archived science is never overwritten."""
import argparse
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import numpy as np
import scipy
from offline.task28_p5_qp_5_r2.analysis import ROOT, HERE, sha, build_diagnostics


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    started=time.perf_counter()
    result=build_diagnostics()
    with (args.output/'diagnostics.json').open('x') as stream:
        json.dump(result,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')
    record=dict(command=[sys.executable,'-m','offline.task28_p5_qp_5_r2.generate_evidence','--output',str(args.output)],
                cwd=str(ROOT),python=sys.version,platform=platform.platform(),numpy=np.__version__,scipy=scipy.__version__,
                head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                elapsed_s=time.perf_counter()-started,exit_code=0,diagnostics_sha256=sha(args.output/'diagnostics.json'),
                source_hashes={str(p.relative_to(ROOT)):sha(p) for p in sorted(HERE.glob('*.py'))+sorted(HERE.glob('*.json'))},
                scope='CPU_ARCHIVE_ANALYSIS_NO_PHYSICS')
    (args.output/'run_record.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    print(json.dumps(dict(status=result['status'],sha256=record['diagnostics_sha256'],elapsed_s=record['elapsed_s'])))


if __name__=='__main__':
    main()
