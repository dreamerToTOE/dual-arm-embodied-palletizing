"""Small registered CPU-only QP controls; read NR-0 evidence, never run NR."""
import argparse
from collections import Counter
from dataclasses import asdict, is_dataclass
from enum import Enum
import gzip
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import resource
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from offline.task28_p5_qp_0.core import (
    CentralizedVelocityQP, PreviousCommand, QPConfig, VelocityInput)
from offline.task28_p5_qp_0.bootstrap import nominal_grasps
from offline.task28_p5_qp_0.integration import OfflineStepDiagnostics
from offline.task28_p5_qp_0.solver import solve_box_equality_qp
from task28_contact.modes import SupportSite

HERE = Path(__file__).resolve().parent


def default(value):
    if is_dataclass(value): return asdict(value)
    if isinstance(value, Enum): return value.value
    if isinstance(value, np.ndarray): return value.tolist()
    if isinstance(value, np.generic): return value.item()
    raise TypeError(type(value).__name__)


def dump(path, data):
    with path.open('x') as stream:
        json.dump(data,stream,indent=2,default=default,allow_nan=False); stream.write('\n')


def load_gzip(path):
    with gzip.open(path,'rt') as stream:return json.load(stream)


def compact_step(row):
    qp=row['qp']; d=qp['diagnostics']; solver=d.get('solver',{})
    edge=row.get('sampled_edge') or {}
    return dict(identity=row['identity'],status=row['status'],qp_status=qp['status'],
        sampled_tracking_candidate=row['sampled_tracking_candidate'],
        tracking_translation_error_m_s=d.get('tracking_translation_error_m_s'),
        tracking_rotation_error_rad_s=d.get('tracking_rotation_error_rad_s'),
        closure_velocity_inf=float(np.max(np.abs(d['closure_velocity_residual']))) if 'closure_velocity_residual' in d else None,
        minimum_position_margin_rad=d.get('minimum_position_margin_rad'),
        minimum_velocity_margin_rad_s=d.get('minimum_velocity_margin_rad_s'),
        kkt_stationarity_inf=solver.get('kkt_stationarity_inf'),
        primal_dual_gap=solver.get('primal_dual_gap'),
        active_lower=solver.get('active_lower'),active_upper=solver.get('active_upper'),
        solve_wall_time_s=solver.get('solve_wall_time_s'),
        independent_kkt_verified=solver.get('independent_kkt_verified'),
        predicted_closure=(row.get('predicted_state') or {}).get('closure'),
        sampled_edge_status=edge.get('status'),sampled_edge_maxima=edge.get('maxima'),
        numerical_reason=qp['reasons'],execution_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True); args=parser.parse_args()
    config=json.loads((HERE/'evidence/experiment_config.json').read_text())
    for relative,digest in config['implementation_hashes'].items():
        if hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()!=digest:
            raise RuntimeError('REGISTERED_IMPLEMENTATION_CHANGED:'+relative)
    args.output.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter(); controller=CentralizedVelocityQP()
    adapter=OfflineStepDiagnostics(controller); rows=[]; references=[]; primary=[]; controls=[]
    source=ROOT/'offline/task28_p4_nr_0/evidence/experiments_v1'
    selected={}
    for seed in config['reference_seeds']:
        path=source/f'seed_{seed}/result.json.gz'
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != config['NR0_reference_hashes'][str(seed)]:
            raise RuntimeError('NR0_REFERENCE_HASH_CHANGED')
        saved=load_gzip(path)
        references.append(dict(seed=seed,path=str(path.relative_to(ROOT)),sha256=digest,
            inherited_label=saved['label'],inherited_execution_authorized=saved['execution_authorized'],
            inherited_continuous={k:{d:p[d]['status'] for d in ('joint_limits','closed_chain','collision')}
                                   for k,p in saved['validation']['continuous'].items()}))
        for segment in ('lift','transport','place'):
            index=len(saved[segment])//2
            X,q=(np.asarray(a) for a in saved[segment][index])
            selected[(seed,segment)]=(X,q,index)
            for name,twist in config['primary_twists'].items():
                req=VelocityInput(q,X,twist,config['dt_s'])
                row=adapter.evaluate(req,f'seed{seed}:{segment}:midpoint{index}:{name}')
                row.update(reference=dict(seed=seed,segment=segment,node_index=index,
                                          result_sha256=digest),input=asdict(req))
                rows.append(row);primary.append(compact_step(row))
        del saved
    X,q,index=selected[(28,'transport')]
    def point_control(name,req,expected_status,ctrl=controller):
        result=ctrl.solve(req)
        row=dict(identity=name,scope='FROZEN_FR3_SINGLE_QP_NOT_A_TRAJECTORY',
                 input=asdict(req),qp=asdict(result),execution_authorized=False,
                 expected_status=expected_status,passed=result.status==expected_status)
        controls.append(row);return result
    # Feasible objective misses are distinct from constraint infeasibility.
    req=VelocityInput(q,X,[5.,0.,0.,0.,0.,0.],config['dt_s'])
    hard_task=adapter.evaluate(req,'control:unachievable_requested_twist')
    hard_task.update(input=asdict(req),scope='FROZEN_FR3_ONE_STEP_TRACKING_NEGATIVE',
        passed=hard_task['qp']['status']=='FEASIBLE_TRACKING_REJECTED' and not hard_task['sampled_tracking_candidate'])
    controls.append(hard_task)
    point_control('wrong_cube',VelocityInput(q,X,np.zeros(6),.01,cube_id='Cube_02'),'INPUT_REJECTED')
    point_control('wrong_frame',VelocityInput(q,X,np.zeros(6),.01,frame_id='map'),'INPUT_REJECTED')
    point_control('missing_joints',VelocityInput(None,X,np.zeros(6),.01),'INPUT_REJECTED')
    point_control('zero_timestep',VelocityInput(q,X,np.zeros(6),0.),'INPUT_REJECTED')
    point_control('wrong_frozen_grasp',VelocityInput(q,X,np.zeros(6),.01,grasps=nominal_grasps(0.)),'INPUT_REJECTED')
    point_control('finite_arithmetic_overflow',VelocityInput(q,X,[1e308]*6,.01),'NUMERICAL_FAILURE')
    for dt in config['dt_sensitivity_s']:
        req=VelocityInput(q,X,config['primary_twists']['combined'],dt)
        row=adapter.evaluate(req,'control:synthetic_dt:'+str(dt))
        row.update(input=asdict(req),scope='SYNTHETIC_NUMERICAL_DT_NOT_MEASURED_SERVO',passed=row['sampled_tracking_candidate'])
        controls.append(row)
    # A previous zero command is actually numerically accepted before it is used.
    hold=controller.solve(VelocityInput(q,X,np.zeros(6),.01))
    assert hold.status=='FEASIBLE_TRACKING_OK'
    caps=np.full(14,config['illustrative_acceleration_rad_s2'])
    accel_controller=CentralizedVelocityQP(controller.model,QPConfig(
        acceleration_caps_rad_s2=caps,acceleration_label='ILLUSTRATIVE_OFFLINE'))
    prev=PreviousCommand(hold.qdot,.01,'synthetic-numerically-accepted-hold',True)
    accel_req=VelocityInput(q,X,config['primary_twists']['combined'],.01,previous=prev)
    accel=accel_controller.solve(accel_req)
    acceleration_ok=(accel.qdot is not None and np.all(np.abs(np.asarray(accel.qdot)-hold.qdot)<=caps*.01+1e-10))
    controls.append(dict(identity='illustrative_acceleration',scope='ENGINEERING_CAP_NOT_AUTHENTICATED_FR3_ACCELERATION',
        input=asdict(accel_req),qp=asdict(accel),previous_numerical_receipt=asdict(hold),
        passed=bool(acceleration_ok),execution_authorized=False))
    point_control('missing_previous_with_accel',VelocityInput(q,X,np.zeros(6),.01),
                  'INPUT_REJECTED',ctrl=accel_controller)
    high_reg=CentralizedVelocityQP(controller.model,QPConfig(regularization=config['regularization_negative']))
    point_control('regularization_tracking_tradeoff',VelocityInput(q,X,config['primary_twists']['combined'],.01),
                  'FEASIBLE_TRACKING_REJECTED',ctrl=high_reg)
    negatives=load_gzip(source/'negative_controls.json.gz')
    negative=next(c for c in negatives if c['case']=='payload_table_collision')
    badX,badq=negative['node']
    payload=adapter.evaluate(VelocityInput(badq,badX,np.zeros(6),.01),'control:saved_payload_table_penetration')
    payload.update(scope='SAVED_NR0_SINGLE_STATE_PAYLOAD_COLLISION_NOT_A_REACHABLE_PATH',
        source_sha256=hashlib.sha256((source/'negative_controls.json.gz').read_bytes()).hexdigest(),
        passed=payload['status']=='CURRENT_STATE_GEOMETRY_REJECTED' and not payload['sampled_tracking_candidate'])
    controls.append(payload)
    # Abstract numerical matrices are never labelled reachable FR3 states.
    def abstract(name,H,f,C,lo,hi,expected):
        result=solve_box_equality_qp(H,f,C,lo,hi)
        controls.append(dict(identity=name,scope='ABSTRACT_QP_MATRIX_CONTROL_NOT_ROBOT_REACHABILITY',
            H=H,f=f,C=C,lower=lo,upper=hi,result=asdict(result),
            passed=result.status==expected,execution_authorized=False))
    n=14;I=np.eye(n); C=np.zeros((1,n));C[0,0]=1;C[0,7]=-1
    lo=-np.ones(n)*2;hi=np.ones(n)*2;lo[0]=1.;hi[7]=-1.
    abstract('equality_box_infeasible',I,np.zeros(n),C,lo,hi,'QP_INFEASIBLE')
    lo=np.zeros(n);hi=np.ones(n);lo[0]=2.
    abstract('empty_box_infeasible',I,np.zeros(n),np.zeros((0,n)),lo,hi,'QP_INFEASIBLE')
    qnear=.5*(controller.lower+controller.upper);qnear[0]=controller.upper[0]-1e-6
    lo=np.maximum(-controller.velocity,(controller.interior_lower-qnear)/.01)
    hi=np.minimum(controller.velocity,(controller.interior_upper-qnear)/.01)
    abstract('near_frozen_position_bound',I,-np.ones(n),np.zeros((0,n)),lo,hi,'QP_OPTIMAL')
    abstract('rank_zero_homogeneous_constraints',1e-6*I,np.zeros(n),np.zeros((6,n)),
             -np.ones(n),np.ones(n),'QP_OPTIMAL')
    ill=np.zeros((2,n));ill[0,0]=1.;ill[0,7]=-1.;ill[1,1]=1e-14;ill[1,8]=-1e-14
    abstract('ill_conditioned_equality_full_row_recheck',I,np.zeros(n),ill,
             -np.ones(n),np.ones(n),'QP_OPTIMAL')
    # Lossless results include every original state/edge receipt and diagnostics.
    full=dict(config=config,references=references,primary=rows,controls=controls,execution_authorized=False)
    with (args.output/'results.json.gz').open('xb') as raw:
        with gzip.GzipFile(filename='',mode='wb',fileobj=raw,mtime=0) as stream:
            stream.write(json.dumps(full,default=default,allow_nan=False,separators=(',',':')).encode())
    positive_ok=bool(primary) and len(primary)==27 and all(x['sampled_tracking_candidate'] for x in primary)
    negatives_ok=bool(controls) and all(x['passed'] for x in controls)
    summary=dict(status='PASS_G2_P5_QP_0_OFFLINE' if positive_ok and negatives_ok else 'PARTIAL',
        primary_count=len(primary),primary_pass_count=sum(x['sampled_tracking_candidate'] for x in primary),
        primary=primary,controls=[dict(identity=x['identity'],scope=x['scope'],passed=x['passed'],
            qp_status=(x.get('qp') or x.get('result'))['status']) for x in controls],
        controls_pass_count=sum(x['passed'] for x in controls),controls_count=len(controls),
        CPU_wall_s=time.perf_counter()-start,peak_main_RSS_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        python=sys.version,platform=platform.platform(),
        dependencies={name:importlib.metadata.version(name) for name in ('numpy','scipy')},
        execution_authorized=False,continuous_safety='NOT_CERTIFIED_FINITE_SAMPLING_ONLY',
        collision_constraints_in_QP=False,temporal_phase_confirmation_claimed=False,
        stop_after='G2-P5-QP-0')
    dump(args.output/'summary.json',summary)
    print(json.dumps({k:summary[k] for k in ('status','primary_count','primary_pass_count','controls_count','controls_pass_count','CPU_wall_s')},indent=2))
    return 0 if summary['status']=='PASS_G2_P5_QP_0_OFFLINE' else 2


if __name__=='__main__':raise SystemExit(main())
