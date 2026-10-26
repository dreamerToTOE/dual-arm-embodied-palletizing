#!/usr/bin/env python3
"""Run deterministic offline tests and emit quantitative G1-1 evidence.

No ROS/Isaac imports, node launches, robot commands or trajectory queries.
The only executable built is the standalone FCL single-state query helper.
"""
from dataclasses import asdict,replace
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
sys.dont_write_bytecode=True
sys.path.insert(0,str(HERE))
sys.path.insert(0,str(HERE/'tests'))
from task28_validator.closed_chain import GraspPair,desired_tcps,evaluate_closure,nominal_grasps
from task28_validator.collision import CollisionValidator
from task28_validator.model import RobotModel
from task28_validator.se3 import residual,rpy_matrix,transform
from task28_validator.state import AttachmentMode,StatePolicy,validate_state
from task28_validator.validator import ClosedChainValidator
from reference_fk import usd_fk
from test_pipeline import state_from_control
from test_collision import bilateral_tools,pose,separated_links


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def require(condition,message):
    if not condition:raise RuntimeError(message)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,help='Explicit evidence JSON output; otherwise prints JSON')
    args=parser.parse_args()
    environment={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','PYTHONPATH':str(HERE)}
    suite=subprocess.run([sys.executable,'-m','pytest',str(HERE/'tests'),'-q','-p','no:cacheprovider'],
                         cwd=REPO,env=environment,text=True,capture_output=True,check=True,timeout=180)
    match=re.search(r'(\d+) passed',suite.stdout)
    require(match is not None,'No passing test count reported')
    model=RobotModel();collision=CollisionValidator(model.fixture_dir)
    validator=ClosedChainValidator(model,collision)
    source_controls=json.loads((HERE/'tests/control_states.json').read_text())['states']
    positives=[];base_state=None
    independent_position_errors=[];independent_rotation_errors=[]
    for row in source_controls:
        obj=transform(rpy_matrix(0,0,row['object_yaw_rad']),row['object_position_m'])
        targets=desired_tcps(obj)
        solutions={side:model.ik(side,target,max_attempts=8,max_nfev=250) for side,target in zip(('left','right'),targets)}
        require(all(value.success for value in solutions.values()),'Positive IK control failed: '+row['label'])
        live_row={**row,**{side+'_q':list(solutions[side].q) for side in ('left','right')}}
        state=state_from_control(model,live_row)
        verdict=validator.validate(state,10.)
        require(verdict.valid,'Positive pipeline failed: '+row['label']+str(verdict.reasons))
        if base_state is None:base_state=state
        oracle_errors={}
        for side in ('left','right'):
            pos,rot=residual(model.fk(side,solutions[side].q),usd_fk(model.fixture_dir,side,solutions[side].q))
            independent_position_errors.append(pos);independent_rotation_errors.append(rot)
            oracle_errors[side]={'translation_m':pos,'rotation_rad':rot}
        positives.append({'label':row['label'],'object_position_m':row['object_position_m'],'object_yaw_rad':row['object_yaw_rad'],
                          'valid':verdict.valid,'reasons':verdict.reasons,'ik':{side:asdict(value) for side,value in solutions.items()},
                          'left_right_closure_translation_m':verdict.closure.translation_error_m,
                          'left_right_closure_rotation_rad':verdict.closure.rotation_error_rad,
                          'fk_observation_errors':verdict.fk_observation_errors,'independent_usd_fk_errors':oracle_errors,
                          'collision':asdict(verdict.collision)})
    supported_boundaries=[]
    for key in ('initial_pose_world_m','target_pose_world_m'):
        obj=transform(translation=model.manifest['cube'][key])
        solutions={side:model.ik(side,target,max_attempts=8,max_nfev=250) for side,target in zip(('left','right'),desired_tcps(obj))}
        require(all(value.success for value in solutions.values()),'Frozen endpoint IK failed: '+key)
        result=collision.validate(model.forward(solutions['left'].q,solutions['right'].q),obj)
        require(not result.valid and result.reasons==('COLLISION:Cube_01/box|task28_table/box',),'Supported boundary unexpectedly accepted or changed: '+key)
        supported_boundaries.append({'label':key,'object_position_m':model.manifest['cube'][key],
                                    'ik':{side:asdict(value) for side,value in solutions.items()},'collision':asdict(result),
                                    'scope_note':'IK reachable; exact Cube/table support touching is rejected by the carried-state policy. Support/release mode transition is future work.'})
    obj=transform(translation=[.65,0,.4]);left,right=desired_tcps(obj)
    wrong_grasp=nominal_grasps().right.copy();wrong_grasp[0,3]+=.01
    mismatch=evaluate_closure(obj,left,right,GraspPair(nominal_grasps().left,wrong_grasp))
    wrong_orientation=right.copy();wrong_orientation[:3,:3]=right[:3,:3]@np.diag([-1.,-1.,1.])
    orientation=evaluate_closure(obj,left,wrong_orientation)
    require(not mismatch.valid and not orientation.valid,'Closure negative accepted')
    closure_controls=[]
    for label,row in [('grasp_transform_10mm_mismatch',mismatch),('same_positions_wrong_grasp_rotation_pi',orientation)]:
        closure_controls.append({'label':label,'valid':row.valid,'translation_error_m':row.translation_error_m,
                                 'rotation_error_rad':row.rotation_error_rad,'reasons':row.reasons})
    unreachable=model.ik('left',transform(translation=[100.,0,0]))
    require(not unreachable.success and unreachable.reason=='IK_OUTSIDE_REACH_BOUND','Unreachable control accepted')
    q=list(base_state.left_joints.positions_rad);q[0]=float(model.joint_upper[0]+.01)
    negative_states={
        'missing_left_joints':replace(base_state,left_joints=None),
        'nonfinite_joint':replace(base_state,left_joints=replace(base_state.left_joints,positions_rad=(float('nan'),)*7)),
        'invalid_quaternion':replace(base_state,cube=replace(base_state.cube,quaternion_xyzw=(0.,0.,0.,0.))),
        'wrong_frame':replace(base_state,cube=replace(base_state.cube,frame_id='map')),
        'wrong_cube_id':replace(base_state,cube=replace(base_state.cube,object_id='Cube_02')),
        'stale_cube':replace(base_state,cube=replace(base_state.cube,stamp_s=9.5)),
        'asynchronous_right_joints':replace(base_state,right_joints=replace(base_state.right_joints,stamp_s=9.95)),
        'future_observation':replace(base_state,cube=replace(base_state.cube,stamp_s=10.01)),
        'inconsistent_attachment':replace(base_state,right_attached=False),
        'unsupported_single_arm_mode':replace(base_state,attachment_mode='left_only'),
        'joint_limit_violation_10mrad':replace(base_state,left_joints=replace(base_state.left_joints,positions_rad=tuple(q))),
    }
    state_controls=[]
    for label,state in negative_states.items():
        verdict=validator.validate(state,10.)
        require(not verdict.valid,'State negative accepted: '+label)
        state_controls.append({'label':label,'valid':False,'reasons':verdict.reasons})
    collision_cases=[
        ('nominal_gap',bilateral_tools(collision,.001),pose(.65,0,.4),'bilateral',True),
        ('intentional_cap_contact',bilateral_tools(collision),pose(.65,0,.4),'bilateral',True),
        ('detached_cap_contact',bilateral_tools(collision),pose(.65,0,.4),'detached',False),
        ('payload_table_20mm',separated_links(collision),pose(.65,0,.24),'bilateral',False),
        ('payload_fully_enclosed_by_table',separated_links(collision),pose(.65,0,.1),'bilateral',False),
    ]
    intruding=bilateral_tools(collision);intruding['left_fr3_side_suction'][1,3]+=.002
    collision_cases.append(('cup_intrusion_2mm',intruding,pose(.65,0,.4),'bilateral',False))
    collision_controls=[]
    for label,links,object_pose,mode,expected in collision_cases:
        result=collision.validate(links,object_pose,mode)
        require(result.valid is expected,'Collision control expectation failed: '+label)
        collision_controls.append({'label':label,**asdict(result)})
    rng=np.random.default_rng(28)
    for q in [model.home,*[rng.uniform(model.joint_lower,model.joint_upper) for _ in range(20)]]:
        for side in ('left','right'):
            pos,rot=residual(model.fk(side,q),usd_fk(model.fixture_dir,side,q))
            independent_position_errors.append(pos);independent_rotation_errors.append(rot)
    jacobian_errors=[]
    q=np.array([.3,-.7,.5,-2.2,.6,1.4,.4]);step=1e-6
    for side in ('left','right'):
        numeric=np.zeros((6,7))
        for index in range(7):
            plus=q.copy();minus=q.copy();plus[index]+=step;minus[index]-=step
            a,b=model.fk(side,plus),model.fk(side,minus)
            numeric[:3,index]=(a[:3,3]-b[:3,3])/(2*step)
            numeric[3:,index]=Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).as_rotvec()/(2*step)
        jacobian_errors.append(float(np.max(np.abs(model.jacobian(side,q)-numeric))))
    source_hashes={str(path.relative_to(HERE)):digest(path) for path in sorted(HERE.rglob('*'))
                   if path.is_file() and path.suffix in ('.py','.cpp','.json') and '__pycache__' not in path.parts and 'evidence' not in path.parts}
    evidence={'schema':'task28.g1_1.offline_evidence.v1','status':'G1_1_PASS','scope':'offline single-state validation only',
              'starting_commit':'ce407473d6fd380916328c6a5d882a596480ec3d','fixture_id':model.manifest['fixture_id'],
              'fixture_manifest_sha256':digest(model.fixture_dir/'fixture_manifest.json'),'fixture_modified':False,
              'environment':{'python':platform.python_version(),'numpy':np.__version__,'scipy':scipy.__version__,
                             'fcl':subprocess.check_output(['pkg-config','--modversion','fcl'],text=True).strip(),
                             'libfcl_sha256':digest(Path('/usr/lib/x86_64-linux-gnu/libfcl.so.0.7.0'))},
              'test_suite':{'passed':int(match.group(1)),'failed':0},'source_sha256':source_hashes,
              'input_state_policy':asdict(StatePolicy()),'synthetic_now_s':10.,'nominal_observation_stamp_s':9.99,
              'positive_full_pipeline_controls':positives,'negative_closure_controls':closure_controls,
              'frozen_supported_boundary_controls':supported_boundaries,
              'ik_unreachable_control':asdict(unreachable),'negative_state_controls':state_controls,
              'collision_controls':collision_controls,
              'independent_fk':{'reference':'Frozen USD joint frames + separate scipy rotation implementation',
                                'comparisons':len(independent_position_errors),'max_translation_m':max(independent_position_errors),
                                'max_rotation_rad':max(independent_rotation_errors)},
              'geometric_jacobian_central_difference_max_abs_errors':jacobian_errors,
              'not_performed':['Isaac/ROS execution','robot/suction commands','trajectory-edge or swept-path validation','NR planning','QP/force control','RL','physical experiment']}
    rendered=json.dumps(evidence,indent=2,allow_nan=False)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(rendered)
        print(f'G1_1_PASS: {evidence["test_suite"]["passed"]} tests; 3 full-pipeline positive controls; evidence: {args.output}')
    else:print(rendered,end='')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
