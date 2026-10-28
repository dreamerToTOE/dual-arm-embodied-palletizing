"""Run a pinned CPU regression suite into a new directory."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--suite', choices=['historical', 'qp4', 'qp5', 'r1', 'r2'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = json.loads((ROOT / 'offline/task28_p5_qp_4/evidence/regression_historical/record.json').read_text())
    argv = list(source['argv'])
    argv[0] = sys.executable
    expected = {'historical': 789, 'qp4': 53, 'qp5': 5, 'r1': 6, 'r2': None}[args.suite]
    if args.suite != 'historical':
        name = {'qp4': '4', 'qp5': '5', 'r1': '5_r1', 'r2': '5_r2'}[args.suite]
        argv = [sys.executable, '-m', 'pytest', f'offline/task28_p5_qp_{name}/tests', '-q', '-p', 'no:cacheprovider', '--import-mode=importlib']
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    with (args.output / 'pytest.log').open('x') as stream:
        process = subprocess.run(argv, cwd=ROOT, env={**os.environ, **source['environment']}, stdout=stream, stderr=subprocess.STDOUT)
    log = (args.output / 'pytest.log').read_bytes()
    matches = re.findall(r'(\d+) passed', log.decode())
    count = int(matches[-1]) if matches else 0
    valid = process.returncode == 0 and (expected is None or count == expected)
    record = dict(suite=args.suite, argv=argv, cwd=str(ROOT), environment=source['environment'], python=sys.version,
                  platform=platform.platform(), exit_code=process.returncode, expected_count=expected, passed_count=count,
                  elapsed_s=time.perf_counter()-started, log_sha256=hashlib.sha256(log).hexdigest(), accepted=valid)
    (args.output / 'record.json').write_text(json.dumps(record, indent=2, sort_keys=True)+'\n')
    print(json.dumps(record, sort_keys=True))
    return 0 if valid else 1


if __name__ == '__main__':
    raise SystemExit(main())
