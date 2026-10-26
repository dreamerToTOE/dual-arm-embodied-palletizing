#!/usr/bin/env python3
"""R1 deterministic offline regression, sequence, negative and noise evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
START='7c7116a09315e2bebdad7bc2d1a12b2de0ca30af'
sys.path[:0]=[str(REPO/'offline/task28_g1_1'),str(HERE)]
from task28_contact.validator import ContactModeValidator
from r1_experiments import run_representative,sensitivity,support_noise,negative_controls


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def json_numeric(value):
    import numpy as np
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError('UNSUPPORTED_EVIDENCE_TYPE:'+type(value).__name__)


def run_suite(paths):
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1',
         'PYTHONPATH':os.pathsep.join([str(REPO/'offline/task28_g1_1'),str(HERE)])}
    command=[sys.executable,'-m','pytest',*map(str,paths),'-q','-p','no:cacheprovider']
    result=subprocess.run(command,cwd=REPO,env=env,capture_output=True,text=True,timeout=180)
    if result.returncode:
        print(result.stdout);print(result.stderr,file=sys.stderr)
        raise RuntimeError('R1_OFFLINE_TEST_FAILURE')
    match=re.search(r'(\d+) passed',result.stdout)
    if match is None:raise RuntimeError('MISSING_PYTEST_PASS_COUNT')
    return {'passed':int(match.group(1)),'paths':[str(path.relative_to(REPO)) for path in paths]}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,default=HERE/'evidence/r1_validation_results.json')
    args=parser.parse_args()
    baseline=[REPO/'offline/task28_g1_1/tests',*[HERE/'tests'/name for name in (
        'test_contact_support.py','test_contact_metadata.py','test_contact_pipeline.py','test_contact_transitions.py')]]
    r1=[HERE/'tests'/name for name in ('test_clearance_geometry.py','test_clearance_temporal.py','test_clearance_sensitivity.py')]
    baseline_result=run_suite(baseline)
    assert baseline_result['passed']==188
    r1_result=run_suite(r1)
    protected=['fixtures','offline/task28_g1_1','isaac','ros_ws',
               *[str(path.relative_to(REPO)) for path in baseline[1:]]]
    protected_diff=subprocess.check_output(['git','diff','--name-only',START,'--',*protected],cwd=REPO,text=True)
    assert not protected_diff,'PROTECTED_BASELINE_CHANGED:'+protected_diff
    gate=ContactModeValidator()
    sources={}
    for root in ('task28_contact','tests','examples'):
        for path in sorted((HERE/root).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                sources[str(path.relative_to(REPO))]=digest(path)
    for name in ('run_r1_validation.py','r1_experiments.py','README.md','requirements.txt'):
        sources[str((HERE/name).relative_to(REPO))]=digest(HERE/name)
    report=REPO/'docs/tasks/TASK28_G1_2A_R1_CLEARANCE_HYSTERESIS.md'
    if report.exists():sources[str(report.relative_to(REPO))]=digest(report)
    evidence={
        'schema':'task28.g1_2a_r1.clearance_temporal_evidence.v1',
        'status':'G1_2A_R1_PASS','starting_commit':START,
        'scope':'Offline observed-pose geometry and discrete temporal confirmation only. No path/edge/sweep or physical success claim.',
        'fixture_id':gate.model.manifest['fixture_id'],
        'fixture_label':'NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED',
        'fixture_manifest_sha256':digest(gate.model.fixture_dir/'fixture_manifest.json'),
        'protected_baseline_diff':[],
        'source_contract':'Every generated Cube/TCP/attachment observation is synthetic; external source declaration is an API contract, not measured sensor evidence.',
        'parameters':{'d_low_m':.001,'d_high_m':.003,'confirmation_frames':3,'max_observation_gap_s':.1,
                      'phase_threshold_float_comparison_epsilon_m':1e-12,'support_penetration_tolerance_unchanged_m':1e-6,
                      'invalid_observation_action':'RESET_COUNTERS_AND_REVOKE_AUTHORIZATION',
                      'contact_classification':'Strict geometric support candidate or separated; physical_support_observed is always false.'},
        'regression':baseline_result,'r1_tests':r1_result,
        'total_tests_passed':baseline_result['passed']+r1_result['passed'],
        'environment':{'python':platform.python_version(),'numpy':__import__('numpy').__version__,
                       'scipy':__import__('scipy').__version__,
                       'fcl':subprocess.check_output(['pkg-config','--modversion','fcl'],text=True).strip()},
        'representative_sequence':run_representative(gate),
        'negative_controls':negative_controls(gate),
        'threshold_noise_sensitivity':sensitivity(gate),
        'cube_only_support_noise':support_noise(gate),
        'kinematic_controls':'examples/r1_configurations.json',
        'source_sha256':sources,
        'not_performed':['Isaac/ROS/robot/suction execution','force/support measurement','path planning or swept collision validation',
                         'NR/QP/force control/RL','G1-2B']}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(evidence,indent=2,allow_nan=False,default=json_numeric)+'\n')
    print(f"G1_2A_R1_PASS: {evidence['total_tests_passed']} tests ({baseline_result['passed']} baseline + {r1_result['passed']} R1); "
          f"{len(evidence['representative_sequence'])} frames; {len(evidence['threshold_noise_sensitivity'])} sensitivity sequences; {args.output}")


if __name__=='__main__':main()
