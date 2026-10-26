#!/usr/bin/env python3
"""Bounded offline tests and complete sampled edge evidence. No execution APIs."""
from dataclasses import asdict
import hashlib,json,os,platform,re,subprocess,sys,time
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
START='bd519c47d226c1fb66bd6e36ed273194dd4ebb8b'
sys.path[:0]=[str(REPO/'offline/task28_g1_1'),str(REPO/'offline/task28_g1_2a'),str(HERE)]
from task28_edge.checker import SampledEdgeValidator
from task28_edge.experiments import controls,a_stage,placement_with_support_hold
from task28_edge.types import EdgeStatus,SamplingPolicy
from task28_validator.se3 import pose_matrix,residual


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def json_numeric(value):
    if isinstance(value,np.ndarray):return value.tolist()
    if isinstance(value,np.generic):return value.item()
    raise TypeError('UNSUPPORTED_EVIDENCE_TYPE:'+type(value).__name__)


def encoded(value):return json.dumps(value,separators=(',',':'),allow_nan=False,default=json_numeric)


def suite(paths):
    env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1',
         'PYTHONPATH':os.pathsep.join([str(REPO/'offline/task28_g1_1'),str(REPO/'offline/task28_g1_2a'),str(HERE)])}
    result=subprocess.run([sys.executable,'-m','pytest',*map(str,paths),'-q','-p','no:cacheprovider'],cwd=REPO,
                          env=env,capture_output=True,text=True,timeout=240)
    if result.returncode:
        print(result.stdout);print(result.stderr,file=sys.stderr);raise RuntimeError('OFFLINE_TEST_FAILURE')
    match=re.search(r'(\d+) passed',result.stdout)
    assert match,result.stdout
    return {'passed':int(match.group(1)),'paths':[str(path.relative_to(REPO)) for path in paths]}


def write_trace(path,trajectory,result):
    path.parent.mkdir(parents=True,exist_ok=True)
    data=asdict(result)
    samples=data.pop('samples');temporal=data.pop('temporal_evidence')
    with path.open('w') as stream:
        stream.write(encoded({'kind':'trajectory','schema':'task28.g1_2b_0.edge_trace.v1','trajectory':asdict(trajectory)})+'\n')
        for i,row in enumerate(samples):stream.write(encoded({'kind':'sample','sample_index':i,**row})+'\n')
        for row in temporal:stream.write(encoded({'kind':'explicit_temporal_observation',**row})+'\n')
        stream.write(encoded({'kind':'edge_result',**data})+'\n')
    pairs=zip(result.samples,result.samples[1:])
    actual={'max_dt_s':0.,'max_joint_step_rad':0.,'max_object_step_m':0.,'max_object_rotation_rad':0.}
    for a,b in pairs:
        aa,bb=a['waypoint'],b['waypoint']
        actual['max_dt_s']=max(actual['max_dt_s'],b['time_s']-a['time_s'])
        actual['max_joint_step_rad']=max(actual['max_joint_step_rad'],*(max(abs(x-y) for x,y in zip(aa[side+'_q'],bb[side+'_q'])) for side in ('left','right')))
        pos,rot=residual(pose_matrix(aa['object_position_m'],aa['object_quaternion_xyzw']),pose_matrix(bb['object_position_m'],bb['object_quaternion_xyzw']))
        actual['max_object_step_m']=max(actual['max_object_step_m'],pos)
        actual['max_object_rotation_rad']=max(actual['max_object_rotation_rad'],rot)
    return {'status':result.status.value,'endpoints_valid':result.endpoints_valid,
            'scope':trajectory.scope,'waypoint_count':len(trajectory.waypoints),
            'planned_sample_count':result.planned_sample_count,'evaluated_sample_count':result.evaluated_sample_count,
            'maxima':result.maxima,'sampling_policy_and_resolution':result.resolution,'actual_sample_increments':actual,
            'reasons':result.reasons,'first_failure':result.rejection_locations[0] if result.rejection_locations else None,
            'rejection_location_count':len(result.rejection_locations),'computation_time_s':result.computation_time_s,
            'continuous_safety':result.continuous_safety,'trace':str(path.relative_to(HERE)),'trace_sha256':digest(path)}


def main():
    baseline=suite([REPO/'offline/task28_g1_1/tests',REPO/'offline/task28_g1_2a/tests'])
    assert baseline['passed']==296
    print('Accepted baseline regression: 296 passed',flush=True)
    added=suite([HERE/'tests'])
    print(f"G1-2B-0 tests: {added['passed']} passed",flush=True)
    protected=['fixtures','offline/task28_g1_1','offline/task28_g1_2a','isaac','ros_ws']
    changed=subprocess.check_output(['git','diff','--name-only',START,'--',*protected],cwd=REPO,text=True)
    assert not changed,'PROTECTED_BASELINE_CHANGED:'+changed
    start=time.perf_counter();checker=SampledEdgeValidator();initialization=time.perf_counter()-start
    summary={'schema':'task28.g1_2b_0.offline_edge_evidence.v1','status':'G1_2B_0_PASS','starting_commit':START,
             'fixture_id':checker.model.manifest['fixture_id'],
             'fixture_manifest_sha256':digest(checker.model.fixture_dir/'fixture_manifest.json'),
             'protected_baseline_diff':[],'baseline_regression':baseline,'edge_tests':added,
             'total_tests_passed':baseline['passed']+added['passed'],
             'checker_initialization_time_s':initialization,
             'environment':{'python':platform.python_version(),'numpy':np.__version__,'scipy':__import__('scipy').__version__,
                            'fcl':subprocess.check_output(['pkg-config','--modversion','fcl'],text=True).strip()},
             'controls':{},'a_stage':{},'source_sha256':{},
             'continuous_safety':'NOT_ESTABLISHED_FINITE_SAMPLING_ONLY',
             'boundary':'No G1-2B-1 swept/CCD or P4-NR; no simulator, ROS, robot, suction, QP, force control or RL execution.',
             'random_seed':'No stochastic path search; explicit preceding-q seeds for bounded single-state IK only.'}
    for name,trajectory in controls(checker.model).items():
        result=checker.validate(trajectory)
        expected=EdgeStatus.SAMPLED_EDGE_VALID if name=='positive_hold' else EdgeStatus.SAMPLED_EDGE_REJECTED
        assert result.status is expected and result.endpoints_valid,(name,result.status,result.reasons)
        summary['controls'][name]=write_trace(HERE/f'evidence/controls/{name}.jsonl',trajectory,result)
        print(name,result.status.value,result.evaluated_sample_count,flush=True)
    trajectories=a_stage(checker)
    trajectories['placement_with_explicit_support_hold']=placement_with_support_hold(checker)
    for name,trajectory in trajectories.items():
        result=checker.validate(trajectory)
        expected=EdgeStatus.EDGE_INCONCLUSIVE if name=='vertical_placement' else EdgeStatus.SAMPLED_EDGE_VALID
        assert result.status is expected and result.endpoints_valid,(name,result.status,result.reasons)
        summary['a_stage'][name]=write_trace(HERE/f'evidence/a_stage/{name}.jsonl',trajectory,result)
        print(name,result.status.value,result.evaluated_sample_count,flush=True)
    for root in (HERE/'task28_edge',HERE/'tests',HERE/'examples',REPO/'offline/task28_g1_1/task28_validator',REPO/'offline/task28_g1_2a/task28_contact'):
        for path in sorted(root.rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts:
                summary['source_sha256'][str(path.relative_to(REPO))]=digest(path)
    for path in (Path(__file__),HERE/'requirements.txt',REPO/'offline/task28_g1_1/tests/reference_fk.py'):
        summary['source_sha256'][str(path.relative_to(REPO))]=digest(path)
    out=HERE/'evidence/summary.json'
    out.write_text(json.dumps(summary,indent=2,allow_nan=False,default=json_numeric)+'\n')
    print(f"G1_2B_0_PASS: {summary['total_tests_passed']} tests; {out}")


if __name__=='__main__':main()
