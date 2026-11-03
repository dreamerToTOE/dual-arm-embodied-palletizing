#!/usr/bin/env python3
"""Run exactly one physics child and verify its exit and evidence."""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.c2_evidence import verify_evidence


def run_child(command, output, timeout_s=240):
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    with tempfile.TemporaryFile(mode='w+b') as log:
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        timed_out = False
        try:
            status = child.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(child.pid, signal.SIGTERM)
            try:
                status = child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                status = child.wait()
        output.mkdir(parents=True, exist_ok=True)
        log.seek(0)
        with (output / 'process.log').open('xb') as stream:
            while chunk := log.read(1024 * 1024):
                stream.write(chunk)
    valid, reason = verify_evidence(output, status)
    valid = valid and not timed_out
    record = {'child_exit_code': status, 'timed_out': timed_out,
              'evidence_check': reason, 'wrapper_exit_code': 0 if valid else 1}
    (output / 'process_exit.json').write_text(json.dumps(record, indent=2) + '\n')
    print(json.dumps(record), flush=True)
    return record['wrapper_exit_code']


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        raise ValueError('invalid run ID')
    out = ROOT / 'offline/task28_stage_b_contact/evidence' / args.run_id
    command = ['/home/ubuntu2004/isaacsim-4.5.0/python.sh',
               str(ROOT / 'isaac/scripts/task28_c2_supported_contact.py'), '--run-id', args.run_id]
    return run_child(command, out)


if __name__ == '__main__':
    raise SystemExit(main())
