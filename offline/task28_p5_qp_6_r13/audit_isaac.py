"""Independent all-frame aggregation of fresh R12 runtime repetitions.

A statistical reread never counts as a repetition: only unique successful child
processes with raw scenes/frames and verified lifecycle can contribute here.
"""
from __future__ import annotations
import collections, hashlib, json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel, DEFAULT_FIXTURE
from offline.task28_p5_qp_6_r12.qp_math import lookahead_velocity_bounds
from offline.task28_p5_qp_6_r12 import evaluate
from offline.task28_p5_qp_6_r12 import process_audit
from offline.task28_p5_qp_6_r13.isaac_repetitions import OUT, write, BASELINE, BASE, metrics
from offline.task28_p5_qp_6_r13.acceptance_review import verify_source_binding, frozen_cell_review


def distribution(values):
    x=np.asarray(values,float)
    if x.size==0: return None
    return {'n':int(x.size),'min_m':float(x.min()),'peak_m':float(x.max()),
            'mean_m':float(x.mean()),'p95_m':float(np.percentile(x,95,method='linear')),
            'span_m':float(np.ptp(x))}


def finite_qp_inputs(qp,pre_q):
    arrays=[np.asarray(qp[k]) for k in ('qdot','q_ref','dq_ref')]+[np.asarray(pre_q)]
    goal=np.asarray(qp['virtual_goal'])
    return all(x.shape==(14,) and np.isfinite(x).all() for x in arrays) and goal.shape==(4,4) and np.isfinite(goal).all()


def trajectory_digest(control):
    canonical=[]
    for r in control:
        qp=r['r12_qp']
        canonical.append({'step':r['physics_step'],'q_ref':qp['q_ref'],'dq_ref':qp['dq_ref'],
                          'qdot':qp['qdot'],'virtual_goal':qp['virtual_goal'],
                          'states':{s:{k:r['arms'][s][k] for k in ['q','dq','tcp_world','applied_command_not_sensor']} for s in ['left','right']}})
    return hashlib.sha256(json.dumps(canonical,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def audit_run(p,model,cfg,binding):
    rec=json.loads(p.read_text());cell=rec['cell'];trial=p.parent
    raw=trial/'evidence/diagnostic'/cell
    result=json.loads((raw/'result.json').read_text())
    life=json.loads((trial/'evidence/process'/cell/'lifecycle.json').read_text())
    frames=[json.loads(s) for s in (raw/'frames.jsonl').read_text().splitlines()]
    previous_here=evaluate.HERE
    try:
        evaluate.HERE=trial
        independently_evaluated=evaluate.eval_cell(cell)
    finally:
        evaluate.HERE=previous_here
    independent_metrics=metrics(independently_evaluated)
    raw_hashes_valid=bool(rec.get('output_hashes')) and all(
        (trial/name).is_file() and hashlib.sha256((trial/name).read_bytes()).hexdigest()==digest
        for name,digest in rec.get('output_hashes',{}).items())
    original_audit_here=process_audit.HERE
    try:
        process_audit.HERE=trial
        isolated_process=process_audit.audit(cell)
    finally:
        process_audit.HERE=original_audit_here
    snapshot=trial/'evidence/process'/cell/'source_at_launch'
    frozen_paths=list(BASE.glob('*.py'))+list(BASE.glob('*.json'))
    snapshots_ok=all((trial/p.name).is_file() and (snapshot/p.name).is_file()
                     and p.read_bytes()==(trial/p.name).read_bytes()==(snapshot/p.name).read_bytes()
                     for p in frozen_paths)
    source_ok=binding['verified'] and rec['source_unchanged'] and raw_hashes_valid and snapshots_ok
    registration=json.loads((BASE/'registrations.json').read_text())[cell]
    acceptance=json.loads((BASE/'preregistration.json').read_text())['acceptance']
    frozen=frozen_cell_review(independently_evaluated,result,isolated_process,registration,
                             cfg,acceptance,source_ok,frames)
    control=[r for r in frames if 'r12_qp' in r]
    previous=result['ownership_initial_state']
    active=[];violations=[];statuses=collections.Counter();residual=[];finite_qp=True;solve_times=[]
    for r in control:
        qp=r['r12_qp'];q=np.r_[previous['arms']['left']['q'][:7],previous['arms']['right']['q'][:7]]
        qdot=np.asarray(qp['qdot'])
        valid=finite_qp_inputs(qp,q)
        finite_qp &= valid
        if not valid:
            violations.append(r['physics_step']);previous=r;continue
        lo,hi=lookahead_velocity_bounds(model,q,1/60,.12,.02)
        box=[int(j) for j in np.flatnonzero((np.abs(qdot-.12)<=2e-8)|(np.abs(qdot+.12)<=2e-8))]
        pos=[int(j) for j in np.flatnonzero(((lo>-.12)&(np.abs(qdot-lo)<=2e-8))|((hi<.12)&(np.abs(qdot-hi)<=2e-8)))]
        if box or pos: active.append({'step':r['physics_step'],'velocity_box_joints':box,'lookahead_joints':pos})
        if np.any(qdot<lo-2e-8) or np.any(qdot>hi+2e-8): violations.append(r['physics_step'])
        statuses[qp['diagnostics']['status']]+=1;residual.append(qp['diagnostics']['hard_relative_residual_inf'])
        solve_times.append(float(qp['diagnostics']['calculation_time_s']))
        previous=r
    limits=np.array(cfg['limits']['research_effort_abs_max_Nm']);rates=np.array(cfg['limits']['arm_effort_rate_max_Nm_s'])
    effort_ok=True;rate_ok=True;effort_by_joint={};rate_by_joint={}
    for s in ['left','right']:
        a=np.array([x['effort_arm'][s] for x in result.get('command_sequence',[])])
        effort_ok &= bool(a.shape==(210,7) and np.all(np.abs(a)<=limits+1e-9))
        rate_ok &= bool(len(a)>1 and np.all(np.abs(np.diff(a,axis=0))*60<=rates+1e-9))
        initial=np.asarray(result['ownership_initial_state']['arms'][s]['applied_command_not_sensor'][:7])
        effort_by_joint[s]=np.max(np.abs(a),axis=0).tolist()
        rate_by_joint[s]=np.max(np.abs(np.diff(np.vstack((initial,a)),axis=0))*60,axis=0).tolist()
    timestamps=[float(r['time_s']) for r in control]
    time_ok=bool(len(control)==210 and np.all(np.abs(np.diff(timestamps)-1/60)<=1e-8))
    cube0=np.asarray(result['ownership_initial_state']['cube_world'])
    cube_max=max(float(np.linalg.norm(np.asarray(r['cube_world'])[:3,3]-cube0[:3,3])) for r in control)
    cube_transform_error=max(float(np.max(np.abs(np.asarray(r['cube_world'])-cube0))) for r in control)
    startup_cube_delta=max(float(np.linalg.norm(np.asarray(r['cube_world'])[:3,3]-cube0[:3,3])) for r in frames if 'r12_qp' not in r)
    safety={'fresh_scene_complete':result.get('status')=='NO_CONTACT_R12_VIRTUAL_COOPERATION_COMPLETE',
            'isolated_clean_exit':bool(life['clean_exit'] and life['isolation_verified']),
            'no_contact_accuracy':independently_evaluated.get('status')=='PASS_NO_CONTACT_ACCURACY_GATE_PASS',
            'independent_evaluation_matches_receipt':independently_evaluated==rec['evaluation'] and independent_metrics==rec['metrics'],
            'raw_hashes_valid':raw_hashes_valid,
            'source_hashes_unchanged':source_ok,
            'all_registered_frozen_gates':frozen['pass'],
            'receipt_lifecycle_identity':bool(rec['child_pid']==life.get('child_pid') and rec['start_utc']==life.get('start_utc') and rec['child_pid'] and rec['start_utc']),
            'complete_receipt':rec['completed'] is True and rec['returncode']==0,
            'effort_limits':effort_ok,'effort_rates':rate_ok,'period':time_ok,
            'qp_all_optimal':statuses=={'QP_OPTIMAL':210},'qp_bounds':not violations,
            'qp_finite':bool(finite_qp),
            'cube_static_all_owned_control_frames':cube_transform_error<=1e-8,
            'suction_zero':result.get('suction_commands')==0,
            'D6_zero':result.get('virtual_execution',{}).get('D6_commands')==0,
            'ros_graph_suppressed':result.get('ros_graph_suppressed') is True}
    return {'cell':cell,'repeat':rec['repeat'],'child_pid':rec['child_pid'],'start_utc':rec['start_utc'],
            'metrics':independent_metrics,'pass':all(safety.values()),'gates':safety,'frozen_gates':frozen,
            'qp_status_counts':dict(statuses),'active_bound_frames':active,'bound_violations':violations,
            'hard_residual_peak':max(residual,default=None),'owned_cube_displacement_peak_m':cube_max,
            'startup_cube_translation_delta_to_ownership_peak_m':startup_cube_delta,
            'motion':{k:independently_evaluated[k] for k in ['qdot_peak_rad_s','native_dq_peak_rad_s','joint_tracking_peak_rad']},
            'effort_abs_peak_by_joint_Nm':effort_by_joint,'effort_rate_peak_by_joint_Nm_s':rate_by_joint,
            'calculation_time_s':{'min':min(solve_times),'mean':float(np.mean(solve_times)),
                                  'p95':float(np.percentile(solve_times,95,method='linear')),'max':max(solve_times)},
            'calculation_exceeds_simulated_period_frames':[r['physics_step'] for r,t in zip(control,solve_times) if t>1/60],
            'calculation_time_is_not_a_registered_real_time_gate':True,
            'trajectory_sha256':trajectory_digest(control),
            'receipt':str(p.relative_to(OUT))}


def main():
    cfg=json.loads((Path(__file__).resolve().parent.parent/'task28_p5_qp_6_r12/source_contract.json').read_text())
    binding=verify_source_binding()
    model=RobotModel(DEFAULT_FIXTURE); rows=[audit_run(p,model,cfg,binding) for p in sorted(OUT.glob('*/receipt.json'))]
    cells={}
    for cell in ['V10-A','V10-B','V10-C']:
        runs=[x for x in rows if x['cell']==cell];all_e=[];all_rel=[]
        baseline_path=BASE/'evidence/diagnostic'/cell/'frames.jsonl'
        baseline_control=[r for r in (json.loads(line) for line in baseline_path.read_text().splitlines()) if 'r12_qp' in r]
        baseline_digest=trajectory_digest(baseline_control)
        for row in runs:
            row['numeric_trace_matches_frozen_baseline']=row['trajectory_sha256']==baseline_digest
        for x in runs:
            receipt=json.loads((OUT/x['receipt']).read_text())
            for r in receipt['evaluation'].get('virtual_tracking_records',[]):
                all_e.append(max(r['object_left_error_m'],r['object_right_error_m']));all_rel.append(r['relative_error_m'])
        keys={(x['child_pid'],x['start_utc']) for x in runs}
        cells[cell]={'fresh_runs':len(runs),'unique_processes':len(keys),'completed_20':len(runs)==20 and len(keys)==20 and {x['repeat'] for x in runs}==set(range(1,21)),
                     'all_frozen_gates_pass':bool(runs) and all(x['pass'] for x in runs),
                     'pooled_frame_object':distribution(all_e),'pooled_frame_relative':distribution(all_rel),
                     'run_peak_distribution':distribution([x['metrics']['object_peak_m'] for x in runs]),
                     'failed_run_count':sum(not x['pass'] for x in runs),
                     'object_exceeded_runs':sum(bool(x['metrics']['object_over_frames']) for x in runs),
                     'object_exceeded_frames':sum(len(x['metrics']['object_over_frames']) for x in runs),
                     'relative_exceeded_frames':sum(len(x['metrics']['relative_over_frames']) for x in runs),
                     'unique_trajectory_hashes':len({x['trajectory_sha256'] for x in runs}),
                     'frozen_baseline_trajectory_sha256':baseline_digest,
                     'frozen_baseline_raw_frames_sha256':hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
                     'baseline_matching_runs':sum(x['numeric_trace_matches_frozen_baseline'] for x in runs),
                     'active_bound_frames':sum(len(x['active_bound_frames']) for x in runs),
                     'runs':runs}
    complete=all(c['completed_20'] for c in cells.values())
    d={'schema':'TASK28_R13_FRESH_ISAAC_SUMMARY_V1','baseline_head':BASELINE,'complete':complete,
       'study':'60 fresh isolated R12 scenes, unmodified runtime and gates',
       'joint_pass':complete and all(c['all_frozen_gates_pass'] for c in cells.values()),
       'percentile_method':'numpy.percentile(method=linear), frame distribution and run peak distribution separate',
       'does_not_prove_perturbation_or_hardware_robustness':True,'source_binding':binding,'cells':cells}
    write(OUT/'summary.json',d)
    print(json.dumps({'complete':complete,'joint_pass':d['joint_pass'],'cells':{c:{k:v for k,v in x.items() if k!='runs'} for c,x in cells.items()}},indent=2))
if __name__=='__main__': main()
