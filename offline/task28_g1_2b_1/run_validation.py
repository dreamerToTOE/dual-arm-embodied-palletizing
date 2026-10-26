#!/usr/bin/env python3
"""Reproduce the new evidence only. Never modify accepted historical outputs."""
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import resource
import subprocess
import sys
import time
import numpy as np

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
START='bcc6e7d260eaa45f6f0d055d53334922df31db3a'
sys.path.insert(0,str(HERE))
from continuous_verifier import ContinuousVerifier, VERIFIED, REJECTED, INCONCLUSIVE
from adversarial_controls import adversarial_controls,analytic_control_evidence
from task28_edge.experiments import controls,a_stage,placement_with_support_hold


def numeric(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    raise TypeError(type(value).__name__)


def write(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False,default=numeric)+'\n')
    return {'path':str(path.relative_to(REPO)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
            'bytes':path.stat().st_size}


def suite(paths,name):
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1', 'PYTHONPATH':os.pathsep.join(str(REPO/'offline'/p) for p in
         ('task28_g1_1','task28_g1_2a','task28_g1_2b_0','task28_g1_2b_1'))}
    start=time.perf_counter()
    test=subprocess.run([sys.executable,'-m','pytest',*map(str,paths),'-q','-p','no:cacheprovider'],
                        cwd=REPO,env=env,capture_output=True,text=True,timeout=600)
    output=test.stdout+test.stderr
    (HERE/'evidence'/f'{name}.txt').write_text(output)
    if test.returncode: raise RuntimeError(output)
    return {'passed':int(re.search(r'(\d+) passed',output).group(1)), 'exit_code':test.returncode,
            'duration_s':time.perf_counter()-start,'log':f'offline/task28_g1_2b_1/evidence/{name}.txt'}


def protected_inventory():
    tracked=subprocess.check_output(['git','ls-tree','-r','--name-only',START],cwd=REPO,text=True).splitlines()
    changed=subprocess.check_output(['git','diff','--name-only',START,'--',*tracked],cwd=REPO,text=True).splitlines()
    if changed: raise RuntimeError('ACCEPTED_TRACKED_FILES_CHANGED:'+repr(changed))
    return {'starting_commit':START,'unchanged_tracked_file_count':len(tracked),'changed':[],
            'sha256':{p:hashlib.sha256((REPO/p).read_bytes()).hexdigest() for p in tracked if (REPO/p).is_file()}}


def summarize(result,sampled,receipt):
    leaves=result['intervals']
    unresolved=[{'span':row['span'],'kind':row.get('kind','path_interval'),'u':row['u'],
                 'unresolved_pairs':row['unresolved_pairs'],'closure_status':row['closure_status'],
                 'local_support_rules_unverified':row['local_support_rules_unverified']}
                for row in leaves if row['collision_status']==INCONCLUSIVE or row['closure_status']==INCONCLUSIVE]
    return {'endpoint_validity':result['endpoint_validity'],'sampled_edge_validity':result['sampled_edge_validity'],
            'sampled_reasons':sampled.reasons,'sampled_count':sampled.evaluated_sample_count,
            'continuous_collision':result['collision']['status'],'continuous_closed_chain':result['closed_chain']['status'],
            'continuous_joint_limits':result['joint_limits']['status'],'combined_status':result['combined_status'],
            'interval_count':len(leaves),'unresolved_interval_count':len(unresolved),
            'distinct_unresolved_pairs':sorted({p for row in unresolved for p in row['unresolved_pairs']}),
            'pair_separation_proof_count':sum(len(row['pair_separation_proofs']) for row in leaves),
            'witness_count':len(result['witnesses']),'runtime_s':result['runtime_s'],'evidence':receipt}


def main():
    (HERE/'evidence').mkdir(exist_ok=True)
    started=time.perf_counter()
    preservation=protected_inventory()
    write(HERE/'evidence/protected_baseline.json',preservation)
    baseline=suite([REPO/'offline'/p/'tests' for p in ('task28_g1_1','task28_g1_2a','task28_g1_2b_0')], 'regression_336')
    assert baseline['passed']==336,baseline
    added=suite([HERE/'tests'],'new_tests')
    print('Tests',baseline,added,flush=True)
    verifier=ContinuousVerifier(max_depth=2)
    summary={'schema':'task28.g1_2b_1.summary.v1','status':'G1_2B_1_PARTIAL','starting_commit':START,
             'regression':baseline,'new_tests':added,'a_stage':{},'controls':{},
             'environment':{'python':platform.python_version(),'numpy':np.__version__,'platform':platform.platform()},
             'proof_contract':{'path':'Exact-real polynomial with stored binary64 power coefficients from frozen span_coefficients; explicit endpoints separately enclosed.',
                              'fk':'Exact-real URDF FK with each token parsed to binary64 then treated as exact real; transcendental rotations outward enclosed.',
                              'rotation':'Exact normalized rational constant quaternion only; changing object SO(3) geodesic is unverified.',
                              'geometry':'Full frozen mesh triangles/boxes/cylinders contained by outer AABBs; 414 pairs, frozen 40 ACM skips.',
                              'numerics':'2**80 integer dyadic endpoints, outward each operation; degree63 Taylor plus exact order64 remainder and derivative radius bound.',
                              'contact':'Suction and local support rules unchanged. Overlapping outer bounds and unproved supported-mode policies remain inconclusive.',
                              'hardware':'No physical tracking, sensing, deformation, dynamics, suction or execution claim.'},
             'budget':{'maximum_adaptive_depth':2,'maximum_leaf_intervals_per_span':4,'budget_exhaustion':INCONCLUSIVE}}
    cases=controls(verifier.checker.model)
    cases.update({key:control.trajectory for key,control in adversarial_controls().items()})
    stages=a_stage(verifier.checker)
    stages['placement_with_explicit_support_hold']=placement_with_support_hold(verifier.checker)
    for category,trajectories in (('controls',cases),('a_stage',stages)):
        for name,trajectory in trajectories.items():
            result=verifier.verify(trajectory)
            sampled=verifier.last_sampled_result
            evidence={'trajectory':asdict(trajectory),'continuous_result':result,'frozen_sampled_result':asdict(sampled)}
            if name in adversarial_controls():
                evidence['independent_analytic_control']=analytic_control_evidence(adversarial_controls()[name])
                assert sampled.endpoints_valid and sampled.status.value=='sampled_edge_valid'
                assert result['combined_status']==REJECTED,(name,result['combined_status'])
            receipt=write(HERE/'evidence'/category/f'{name}.json',evidence)
            summary[category][name]=summarize(result,sampled,receipt)
            print(category,name,result['joint_limits']['status'],result['closed_chain']['status'],result['collision']['status'],flush=True)
    summary['total_duration_s']=time.perf_counter()-started
    summary['peak_process_rss_kib']=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    summary['child_peak_rss_kib']=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss
    summary['protected_baseline_unchanged_after_run']=not protected_inventory()['changed']
    summary['source_sha256']={str(p.relative_to(REPO)):hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in sorted(HERE.rglob('*.py')) if '__pycache__' not in p.parts}
    summary['fcl_audit_sha256']=hashlib.sha256((HERE/'evidence/fcl_capabilities.json').read_bytes()).hexdigest()
    summary['evidence_file_bytes']=sum(p.stat().st_size for p in (HERE/'evidence').rglob('*') if p.is_file())
    write(HERE/'evidence/summary.json',summary)
    print('G1_2B_1_PARTIAL',summary['total_duration_s'],summary['peak_process_rss_kib'],flush=True)


if __name__=='__main__':main()
