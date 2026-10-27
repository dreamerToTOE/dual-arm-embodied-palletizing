"""Run one preregistered QP1 versus SE(3)-feedback batch, CPU/offline only."""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import platform
import resource
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from offline.task28_p5_qp_2.driver import run_comparison
from offline.task28_p5_qp_2.feedback import FeedbackConfig

HERE = Path(__file__).resolve().parent


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def write_json(path, data):
    with path.open('x') as f:
        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')


def write_gzip(path, data):
    raw = (json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode()
    with path.open('xb') as f:
        f.write(gzip.compress(raw, mtime=0))


def max_optional(values):
    values = [x for x in values if x is not None]
    return max(values) if values else None


def min_optional(values):
    values = [x for x in values if x is not None]
    return min(values) if values else None


def drift_metrics(closure, reference):
    """Relative and reference drift from G1's implied object SE(3) receipts.

    Mean rotation is the average of two world tangent logs about the reference,
    not a claim of an exact global SE(3) geodesic mean.
    """
    if closure is None:
        return None
    L = np.asarray(closure['implied_left_object'], dtype=float)
    R = np.asarray(closure['implied_right_object'], dtype=float)
    X = np.asarray(reference, dtype=float)
    def log(M):
        return Rotation.from_matrix(M).as_rotvec()
    relative_p = L[:3, 3] - R[:3, 3]
    relative_r = log(L[:3, :3] @ R[:3, :3].T)
    left_p = L[:3, 3] - X[:3, 3]
    right_p = R[:3, 3] - X[:3, 3]
    left_r = log(L[:3, :3] @ X[:3, :3].T)
    right_r = log(R[:3, :3] @ X[:3, :3].T)
    mean_p, mean_r = .5*(left_p+right_p), .5*(left_r+right_r)
    return dict(relative_translation_m=float(np.linalg.norm(relative_p)),
        relative_rotation_rad=float(np.linalg.norm(relative_r)),
        mean_reference_translation_m=float(np.linalg.norm(mean_p)),
        mean_reference_rotation_tangent_rad=float(np.linalg.norm(mean_r)),
        left_reference_translation_m=float(np.linalg.norm(left_p)),
        left_reference_rotation_rad=float(np.linalg.norm(left_r)),
        right_reference_translation_m=float(np.linalg.norm(right_p)),
        right_reference_rotation_rad=float(np.linalg.norm(right_r)),
        relative_translation_world=relative_p.tolist(), relative_rotation_log_world=relative_r.tolist(),
        mean_translation_world=mean_p.tolist(), mean_rotation_tangent_world=mean_r.tolist(),
        left_translation_world=left_p.tolist(), right_translation_world=right_p.tolist(),
        left_rotation_log_world=left_r.tolist(), right_rotation_log_world=right_r.tolist(),
        metric_scope='G1_SAVED_IMPLIED_OBJECT_SE3_WORLD_NOT_PHYSICAL_OBSERVATION', execution_authorized=False)


def add_drift_trace(record):
    """Attach per-attempt, including rejected, drift calculated from G1 receipts."""
    request = record.get('request') or {}
    initial = record.get('initial_state') or {}
    record['initial_drift'] = drift_metrics(initial.get('closure'), request.get('object_pose')) if initial.get('closure') is not None else None
    for step in record['steps']:
        state = step.get('predicted_state') or {}
        pose = step.get('desired_object_pose_next')
        step['drift_metrics'] = drift_metrics(state.get('closure'), pose) if state.get('closure') is not None and pose is not None else None
    return record


def compact(record, case, method):
    steps = record['steps']
    accepted = [s for s in steps if s['accepted']]
    diagnostics = [s.get('independent_velocity_diagnostics') or {} for s in steps]
    bounds = [s['frozen_bounds'] for s in steps if s.get('frozen_bounds')]
    solvers = [((s.get('qp') or {}).get('diagnostics') or {}).get('solver') or {} for s in steps]
    metrics = [s['drift_metrics'] for s in accepted if s.get('drift_metrics')]
    final = accepted[-1]['drift_metrics'] if accepted else record.get('initial_drift')
    failed = None
    if record.get('failure') and steps:
        step = steps[-1]
        failed = dict(step_index=step['step_index'], candidate_retained=step.get('q_next') is not None,
            qp_status=(step.get('qp') or {}).get('status'), reasons=step['reasons'],
            drift_metrics=step.get('drift_metrics'),
            current_valid=(step.get('current_state') or {}).get('valid'),
            predicted_valid=(step.get('predicted_state') or {}).get('valid'),
            predicted_collision_reasons=((step.get('predicted_state') or {}).get('collision') or {}).get('reasons'),
            sampled_edge_status=(step.get('sampled_edge') or {}).get('status'), execution_authorized=False)
    accepted_counts = dict(Counter((s.get('qp') or {}).get('status','MISSING') for s in steps))
    return dict(identity=case['identity'], node_identity=case.get('node_identity'), seed=case.get('seed'),
        purpose=case['purpose'], method=method, status=record['status'],
        initial_valid=(record.get('initial_state') or {}).get('valid'),
        initial_reasons=(record.get('initial_state') or {}).get('reasons'),
        initial_drift=record.get('initial_drift'),
        planned_steps=case['max_steps'], dt_s=case['dt_s'], planned_duration_s=case.get('planned_duration_s'),
        attempted_steps=record['attempted_step_count'], accepted_steps=record['accepted_step_count'],
        accepted_duration_s=(record.get('retained_state') or {}).get('time_s'),
        initial_state_source='SAVED_NR0_NOT_NEW_IK', final_accepted_drift=final, failure=record.get('failure'), failure_detail=failed,
        max_accepted_relative_translation_m=max_optional([m['relative_translation_m'] for m in metrics]),
        max_accepted_relative_rotation_rad=max_optional([m['relative_rotation_rad'] for m in metrics]),
        max_accepted_mean_reference_translation_m=max_optional([m['mean_reference_translation_m'] for m in metrics]),
        max_accepted_mean_reference_rotation_tangent_rad=max_optional([m['mean_reference_rotation_tangent_rad'] for m in metrics]),
        minimum_attempted_position_margin_rad=min_optional([b['minimum_position_margin_rad'] for b in bounds]),
        minimum_attempted_velocity_margin_rad_s=min_optional([b['minimum_velocity_margin_rad_s'] for b in bounds]),
        max_attempted_original_tracking_translation_m_s=max_optional([d.get('tracking_translation_error_m_s') for d in diagnostics]),
        max_attempted_original_tracking_rotation_rad_s=max_optional([d.get('tracking_rotation_error_rad_s') for d in diagnostics]),
        max_attempted_hard_equality_inf=max_optional([d.get('affine_residual_inf') if method=='feedback' else d.get('original_equality_inf') for d in diagnostics]),
        max_attempted_original_C_inf=max_optional([d.get('original_C_inf') if method=='feedback' else d.get('original_equality_inf') for d in diagnostics]),
        max_attempted_KKT_stationarity_inf=max_optional([s.get('kkt_stationarity_inf') for s in solvers]),
        max_attempted_abs_primal_dual_gap=max_optional([abs(s['primal_dual_gap']) for s in solvers if s.get('primal_dual_gap') is not None]),
        active_lower_total=sum(len(s.get('active_lower', [])) for s in solvers),
        active_upper_total=sum(len(s.get('active_upper', [])) for s in solvers),
        QP_status_counts=accepted_counts, sampled_count=sum((s.get('sampled_edge') or {}).get('evaluated_sample_count',0) for s in steps),
        sampled_rejections=sum((s.get('sampled_edge') or {}).get('status')=='sampled_edge_rejected' for s in steps),
        sampled_inconclusive=sum((s.get('sampled_edge') or {}).get('status')=='edge_inconclusive' for s in steps),
        numerical_phase_observations=record['phase_confirmation_observations_generated'],
        wall_time_s=record['wall_time_s'], execution_authorized=False)


def pair_comparison(case, baseline, feedback):
    """Apply preregistered common-duration and noise-floor acceptance."""
    n = case['max_steps']; b, f = baseline['final_accepted_drift'], feedback['final_accepted_drift']
    completed = (baseline['accepted_steps']==n and feedback['accepted_steps']==n
                 and baseline['status']=='COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED'
                 and feedback['status']=='COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED')
    T = case['planned_duration_s']; bt = baseline.get('accepted_duration_s'); ft = feedback.get('accepted_duration_s')
    valid_times = all(isinstance(t, (int, float)) and not isinstance(t, bool) and np.isfinite(t) for t in (T, bt, ft))
    matched_times = bool(valid_times and abs(case['dt_s']*n-T) <= 1e-12 and abs(bt-T) <= 1e-12 and abs(ft-T) <= 1e-12)
    values = [d.get(k) if isinstance(d, dict) else None for d in (b, f)
              for k in ('relative_translation_m', 'relative_rotation_rad')]
    valid_metrics = all(isinstance(v, (int, float)) and not isinstance(v, bool) and np.isfinite(v) and v >= 0 for v in values)
    comparable = bool(completed and matched_times and valid_metrics and b['relative_translation_m'] > 1e-8 and b['relative_rotation_rad'] > 1e-8)
    trans_ratio = f['relative_translation_m']/b['relative_translation_m'] if comparable else None
    rot_ratio = f['relative_rotation_rad']/b['relative_rotation_rad'] if comparable else None
    return dict(identity=case['identity'], node_identity=case['node_identity'], seed=case['seed'],
        purpose=case['purpose'], dt_s=case['dt_s'], planned_duration_s=case['planned_duration_s'],
        initial_same_saved_node=True, same_reference_twist=case['desired_object_twist'],
        baseline=baseline, feedback=feedback, comparable_at_preregistered_endpoint=comparable,
        not_comparable_reason=None if comparable else ('NOT_BOTH_COMPLETE_TO_SAME_ENDPOINT' if not completed else 'MISMATCHED_OR_NONFINITE_ENDPOINT_TIME' if not matched_times else 'INVALID_DRIFT_METRICS' if not valid_metrics else 'BASELINE_BELOW_PREREGISTERED_ANALYSIS_FLOOR'),
        feedback_over_baseline_relative_translation_ratio=trans_ratio,
        feedback_over_baseline_relative_rotation_ratio=rot_ratio,
        both_relative_metrics_improved_at_least_20_percent=bool(comparable and trans_ratio<=.8 and rot_ratio<=.8),
        execution_authorized=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    registration=json.loads((HERE/'evidence/run_registration.json').read_text())
    for rel,digest in registration['bound_file_hashes'].items():
        if sha(ROOT/rel)!=digest:raise RuntimeError('REGISTERED_FILE_CHANGED:'+rel)
    config=json.loads((HERE/'evidence/preregistration.json').read_text())
    if any(getattr(FeedbackConfig(),key)!=value for key,value in config['gains'].items() if key!='provenance'):
        raise RuntimeError('REGISTERED_FEEDBACK_GAINS_MISMATCH')
    if FeedbackConfig().log_pi_exclusion_rad != config['logarithm_domain']['principal_so3_log_pi_exclusion_rad']:
        raise RuntimeError('REGISTERED_LOG_DOMAIN_MISMATCH')
    protected=json.loads((HERE/'evidence/protected_baseline.json').read_text())
    for item in protected['files']:
        if sha(ROOT/item['path'])!=item['sha256']:raise RuntimeError('PROTECTED_BASELINE_CHANGED:'+item['path'])
    source={}
    for ref in config['references']:
        if sha(ROOT/ref['path'])!=ref['sha256']:raise RuntimeError('SAVED_NR0_HASH_MISMATCH')
        source[ref['seed']]=json.loads(gzip.decompress((ROOT/ref['path']).read_bytes()))
    nodes={n['identity']:n for n in config['nodes']}
    for node in nodes.values():
        saved=source[node['seed']][node['segment']]
        if (node['node_index']!=len(saved)//2 or node['segment_node_count']!=len(saved)
                or saved[node['node_index']] != [node['object_pose'],node['q14']]):
            raise RuntimeError('SAVED_NODE_MISMATCH:'+node['identity'])
    neg=config['penetration_control']; p=ROOT/neg['source_path']
    if sha(p)!=neg['source_sha256']:raise RuntimeError('SAVED_NEGATIVE_HASH_MISMATCH')
    saved_neg=next(x for x in json.loads(gzip.decompress(p.read_bytes())) if x['case']==neg['source_case'])
    if saved_neg['node'] != [neg['object_pose'],neg['q14']]:raise RuntimeError('SAVED_NEGATIVE_NODE_MISMATCH')
    args.output.mkdir(parents=True,exist_ok=False)
    start=time.perf_counter();write_json(args.output/'registered_config.json',config)
    cases=list(config['cases'])+[neg];artifacts=[];summaries=[];pairs=[];all_records={}
    for case in cases:
        node=nodes[case['node_identity']] if 'node_identity' in case else case
        results={}; case_briefs={}
        for method in case['methods']:
            provenance=dict(source='SAVED_NR0_NODE_THEN_SYNTHETIC_INTEGRATION',
                source_sha256=node['source_sha256'],seed=node.get('seed',node.get('source_seed')),
                segment=node.get('segment','archived_negative_control'),node_index=node.get('node_index'),execution_authorized=False)
            record=run_comparison(node['q14'],node['object_pose'],case['desired_object_twist'],case['dt_s'],
                case['max_steps'],case['identity'],method=method,observation_provenance=provenance)
            record=add_drift_trace(record)
            file=f'case_{len(artifacts):03d}.json.gz';write_gzip(args.output/file,record)
            brief=compact(record,case,method);brief['artifact']=file
            artifacts.append(dict(identity=case['identity'],method=method,path=file,
                                  bytes=(args.output/file).stat().st_size,sha256=sha(args.output/file),execution_authorized=False))
            summaries.append(brief);case_briefs[method]=brief;results[method]=record
            print(json.dumps(dict(identity=case['identity'],method=method,status=record['status'],
                accepted=record['accepted_step_count'],attempted=record['attempted_step_count'],execution_authorized=False)),flush=True)
        if 'node_identity' in case:
            # The desired request inputs must match independently of the chosen controller.
            a=results['baseline']['request'];b=results['feedback']['request']
            assert a==b, 'PAIR_REQUESTS_MISMATCH:'+case['identity']
            pairs.append(pair_comparison(case,case_briefs['baseline'],case_briefs['feedback']))
        all_records[case['identity']]=results
    primary=[p for p in pairs if p['purpose']=='primary_combined']
    primary_per_dt={}
    for dt in (.02,.05):
        group=[p for p in primary if p['dt_s']==dt]
        primary_per_dt[str(dt)]=dict(case_count=len(group), comparable_count=sum(p['comparable_at_preregistered_endpoint'] for p in group),
            improved_both_20pct=sum(p['both_relative_metrics_improved_at_least_20_percent'] for p in group),
            required_count=2, execution_authorized=False)
    negative=all_records[neg['identity']]
    collision_pair='COLLISION:Cube_01/box|task28_table/box'
    penetration_rejected=all(not r['initial_state']['valid'] and r['attempted_step_count']==0
        and collision_pair in r['initial_state']['reasons'] and r['execution_authorized'] is False
        for r in negative.values())
    valid_initials=all(b['initial_valid'] is True for b in summaries if b['purpose']!='current_state_FCL_negative_not_reachable_path')
    flags=all(r['execution_authorized'] is False and r['phase_confirmation_observations_generated']==0
              and r['continuous_verifier_run'] is False and r['nr_projection_performed'] is False
              for caseset in all_records.values() for r in caseset.values())
    # Detect any path attempting to retain or continue after a rejected state.
    prefixes=True; all_accepted_geometry=True
    for caseset in all_records.values():
        for record in caseset.values():
            rows=record['steps'];a=record['accepted_step_count']
            prefixes &= all(s['accepted'] for s in rows[:a]) and all(not s['accepted'] for s in rows[a:]) and len(rows)-a<=1
            for step in rows[:a]:
                all_accepted_geometry &= bool(step['frozen_bounds']['valid'] and step['current_state']['valid']
                    and step['predicted_state']['valid'] and step['sampled_edge']['status']=='sampled_edge_valid')
    requirements=dict(all_registered_source_initial_states_valid=bool(valid_initials),
        all_primary_pairs_reach_same_preregistered_time=all(v['comparable_count']==3 for v in primary_per_dt.values()),
        primary_two_of_three_per_dt_improve_both_by_20pct=all(v['improved_both_20pct']>=2 for v in primary_per_dt.values()),
        preserved_archived_payload_table_penetration_rejected_both=bool(penetration_rejected),
        all_accepted_steps_keep_frozen_state_edge_bounds=bool(all_accepted_geometry),
        first_failure_retained_no_continuation=bool(prefixes),
        no_new_phase_observation_or_execution=bool(flags))
    summary=dict(schema='task28.g2_p5_qp_2.experiment_summary.v1',
        status='FORMAL_EXPERIMENT_REQUIREMENTS_MET_PENDING_REGRESSION_AND_MATH' if all(requirements.values()) else 'PARTIAL_EXPERIMENT_REQUIREMENTS_UNMET',
        execution_authorized=False, configuration_sha256=sha(HERE/'evidence/preregistration.json'),
        registration_sha256=sha(HERE/'evidence/run_registration.json'),
        experiment_requirements=requirements,primary_per_dt=primary_per_dt,pairs=pairs,
        cases=summaries,artifacts=artifacts,scenario_count=len(cases),trajectory_count=len(artifacts),
        planned_maximum_step_attempts=config['budget']['max_rollout_steps'],
        actual_step_attempts=sum(r['attempted_steps'] for r in summaries),
        accepted_step_count=sum(r['accepted_steps'] for r in summaries),
        total_B0_samples=sum(r['sampled_count'] for r in summaries),
        CPU_wall_s=time.perf_counter()-start,peak_main_RSS_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        environment=dict(python=sys.version,platform=platform.platform(),numpy=np.__version__),
        physical_timing_measured=False,continuous_safety='NOT_CERTIFIED_FINITE_SAMPLING_ONLY',
        new_NR_IK_or_reprojection_calls=0,new_R1_observations=0,stop_after='G2-P5-QP-2')
    write_json(args.output/'summary.json',summary)
    print(json.dumps(dict(status=summary['status'],experiment_requirements=requirements,
        primary_per_dt=primary_per_dt,CPU_wall_s=summary['CPU_wall_s'],execution_authorized=False)),flush=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
