"""Run the frozen QP-1 CPU matrix once into a new output directory.

No NR, IK, simulator, temporal confirmer or actuator is called. All states,
including rejected candidates and unsafe unbounded references, are archived.
"""
from __future__ import annotations
import argparse
from collections import Counter
from dataclasses import asdict
import datetime
import gzip
import hashlib
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
from offline.task28_p5_qp_1.driver import MultiStepDriver, MultiStepRequest
from offline.task28_p4_nr_0.validation import CandidateValidator

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path, data):
    with path.open('x') as out:
        json.dump(data, out, ensure_ascii=False, indent=2, allow_nan=False)
        out.write('\n')


def write_gzip(path, data):
    plain = (json.dumps(data, ensure_ascii=False, separators=(',', ':'), allow_nan=False)+'\n').encode()
    with path.open('xb') as out:
        # No timestamp or absolute output path enters compressed scientific data.
        out.write(gzip.compress(plain, mtime=0))


def max_or_none(values):
    values = [v for v in values if v is not None]
    return max(values) if values else None


def min_or_none(values):
    values = [v for v in values if v is not None]
    return min(values) if values else None


def closure_extrema(rows):
    tr, rot = [], []
    for row in rows:
        c = row.get('closure') if row else None
        if c:
            tr.append(max(c[k] for k in ('translation_error_m', 'left_object_translation_error_m', 'right_object_translation_error_m')))
            rot.append(max(c[k] for k in ('rotation_error_rad', 'left_object_rotation_error_rad', 'right_object_rotation_error_rad')))
    return dict(max_translation_m=max_or_none(tr), max_rotation_rad=max_or_none(rot))


def compact(record):
    steps = record['steps']
    accepted = [s for s in steps if s['accepted']]
    def metrics(rows):
        diagnostics = [(s.get('qp') or {}).get('diagnostics', {}) for s in rows]
        bounds = [s['frozen_bounds'] for s in rows if s.get('frozen_bounds')]
        states = [s.get('predicted_state') for s in rows if s.get('predicted_state')]
        return dict(
            tracking_translation_m_s=max_or_none([d.get('tracking_translation_error_m_s') for d in diagnostics]),
            tracking_rotation_rad_s=max_or_none([d.get('tracking_rotation_error_rad_s') for d in diagnostics]),
            predicted_closure=closure_extrema(states),
            sampled_closure_translation_m=max_or_none([(s.get('sampled_edge') or {}).get('maxima', {}).get('max_closure_translation_m') for s in rows]),
            sampled_closure_rotation_rad=max_or_none([(s.get('sampled_edge') or {}).get('maxima', {}).get('max_closure_rotation_rad') for s in rows]),
            minimum_position_margin_rad=min_or_none([b['minimum_position_margin_rad'] for b in bounds]),
            minimum_velocity_margin_rad_s=min_or_none([b['minimum_velocity_margin_rad_s'] for b in bounds]),
            max_abs_velocity_change_rad_s=max_or_none([max(abs(x) for x in s['velocity_change_rad_s']) for s in rows if s.get('velocity_change_rad_s') is not None]),
            max_abs_numerical_velocity_change_rate_rad_s2=max_or_none([max(abs(x) for x in s['velocity_change_rate_rad_s2']) for s in rows if s.get('velocity_change_rate_rad_s2') is not None]))
    return dict(identity=record['request']['identity'], method=record['method'], status=record['status'],
        initial_valid=(record.get('initial_state') or {}).get('valid'),
        initial_closure=closure_extrema([record.get('initial_state')]),
        attempted=record['attempted_step_count'], accepted=record['accepted_step_count'],
        qp_status_counts=dict(Counter((s.get('qp') or {}).get('status', 'MISSING') for s in steps)),
        active_source_counts=dict(Counter(label for s in steps for bound in active_sources(s) for label in bound['sources'])),
        failure=record['failure'],
        geometry_rejection_after_tracking_and_hard_pass=[s['step_index'] for s in steps
            if (s.get('qp') or {}).get('status') == 'FEASIBLE_TRACKING_OK'
            and (s.get('frozen_bounds') or {}).get('valid')
            and (s.get('independent_velocity_diagnostics') or {}).get('original_equality_inf', float('inf')) <= 1e-8
            and (s.get('current_state') or {}).get('valid')
            and s.get('q_next') is not None
            and ((s.get('predicted_state') is not None and not s['predicted_state']['valid'])
                 or (s.get('sampled_edge') or {}).get('status') == 'sampled_edge_rejected')
            and record.get('failure') is not None
            and record['failure'].get('retained_failing_candidate') is True
            and record['failure']['step_index'] == s['step_index'] == len(steps)-1],
        all_attempts=metrics(steps), accepted_only=metrics(accepted),
        total_B0_samples=sum((s.get('sampled_edge') or {}).get('evaluated_sample_count', 0) for s in steps),
        wall_time_s=record['wall_time_s'], continuous_safety=record['continuous_safety'],
        execution_authorized=False)


def active_sources(step):
    """Classify original QP-0 active box rows by their frozen bound source."""
    bounds = step.get('frozen_bounds')
    if not bounds:
        return []
    q = np.asarray(step['q_initial']); x = np.asarray(step['qdot14'])
    dt = step['qp']['diagnostics']['dt_s']
    v = np.asarray(bounds['velocity_max_rad_s'])
    qmin = np.nextafter(np.asarray(bounds['position_lower_rad']), np.inf)
    qmax = np.nextafter(np.asarray(bounds['position_upper_rad']), -np.inf)
    rows = []
    for direction, active, vel, pos in (
            ('lower', bounds['active_lower_indices'], -v, (qmin-q)/dt),
            ('upper', bounds['active_upper_indices'], v, (qmax-q)/dt)):
        for j in active:
            box = bounds[direction+'_rad_s'][j]
            labels = []
            if abs(box-vel[j]) <= 1e-12: labels.append('velocity')
            if abs(box-pos[j]) <= 1e-12: labels.append('position_lookahead')
            rows.append(dict(joint_index=j, joint_name=('left' if j < 7 else 'right')+f'_fr3_joint{j%7+1}',
                direction=direction, value_rad_s=float(x[j]), bound_rad_s=box, sources=labels,
                active_reporting_tolerance_rad_s=bounds['active_bound_reporting_tolerance_rad_s'],
                execution_authorized=False))
    return rows


def request(case, node, **mutation):
    fields = dict(q14=node['q14'], object_pose=node['object_pose'],
        desired_object_twist=case.get('desired_object_twist', [.005,-.003,.002,.01,-.005,.02]),
        dt_s=case.get('dt_s', .01), max_steps=case.get('max_steps', 1), identity=case['identity'],
        observation_provenance=dict(source='SAVED_NR0_NODE_THEN_SYNTHETIC_OFFLINE_INTEGRATION',
            seed=node['seed'], segment=node['segment'], node_index=node['node_index'],
            source_sha256=node['source_sha256'], execution_authorized=False))
    fields.update(mutation)
    return MultiStepRequest(**fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    registration = json.loads((HERE/'evidence/run_registration.json').read_text())
    for rel, expected in registration['bound_file_hashes'].items():
        if sha(ROOT/rel) != expected:
            raise RuntimeError('REGISTERED_SOURCE_OR_CONFIG_CHANGED:'+rel)
    config = json.loads((HERE/'evidence/preregistration.json').read_text())
    protected = json.loads((HERE/'evidence/protected_baseline.json').read_text())
    for item in protected['files']:
        if sha(ROOT/item['path']) != item['sha256']:
            raise RuntimeError('PROTECTED_BASELINE_CHANGED:'+item['path'])
    source_data = {}
    for item in config['references']:
        if sha(ROOT/item['path']) != item['sha256']:
            raise RuntimeError('SAVED_NR0_CHANGED')
        source_data[item['seed']] = json.loads(gzip.decompress((ROOT/item['path']).read_bytes()))
    for node in config['nodes']:
        saved_nodes = source_data[node['seed']][node['segment']]
        if (node['node_index'] != len(saved_nodes)//2
                or node['segment_node_count'] != len(saved_nodes)
                or saved_nodes[node['node_index']] != [node['object_pose'], node['q14']]):
            raise RuntimeError('REGISTERED_NODE_DIFFERS_FROM_SAVED_NR0:'+node['identity'])
    del source_data
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()
    write_json(args.output/'registered_config.json', config)
    nodes = {n['identity']: n for n in config['nodes']}
    validator = CandidateValidator()
    initial = []
    for node in config['nodes']:
        row = validator.state(node['object_pose'], node['q14'])
        initial.append(dict(identity=node['identity'], result=row, execution_authorized=False))
    # CandidateValidator state includes NumPy arrays; use driver's lossless encoder.
    from offline.task28_p5_qp_1.driver import _jsonable
    write_gzip(args.output/'initial_states.json.gz', _jsonable(initial))
    driver = MultiStepDriver()
    summaries = []; inventory = []; pairs = []
    def save(identity, result, group):
        # Identity is metadata. Stable ordinal filenames avoid path interpretation.
        rel = f'case_{len(inventory):03d}.json.gz'
        write_gzip(args.output/rel, result)
        brief = compact(result); brief.update(group=group, artifact=rel)
        summaries.append(brief)
        inventory.append(dict(identity=identity, group=group, method=result['method'],
            path=rel, bytes=(args.output/rel).stat().st_size, sha256=sha(args.output/rel), execution_authorized=False))
        print(json.dumps(dict(identity=identity, method=result['method'], status=result['status'],
            accepted=result['accepted_step_count'], attempted=result['attempted_step_count'], execution_authorized=False)), flush=True)
        return brief
    for case in config['point_cases']:
        node = nodes[case['node_identity']]; results = {}
        for method in case['methods']:
            result = driver.run(request(case, node), method=method)
            results[method] = result
            save(case['identity'], result, 'paired_point')
        bounded, free = (results[k] for k in ('bounded', 'unbounded_reference'))
        bs = bounded['steps'][0] if bounded['steps'] else {}
        fs = free['steps'][0] if free['steps'] else {}
        paired = dict(identity=case['identity'], purpose=case['purpose'],
            matched_input=bounded['request']==free['request'],
            current_valid=bool((bounded.get('initial_state') or {}).get('valid') and (free.get('initial_state') or {}).get('valid')),
            bounded_status=(bs.get('qp') or {}).get('status'), free_status=(fs.get('qp') or {}).get('status'),
            bounded_hard_valid=bool((bs.get('frozen_bounds') or {}).get('valid')
                and (bs.get('independent_velocity_diagnostics') or {}).get('original_equality_inf', float('inf')) <= 1e-8
                and ((bs.get('qp') or {}).get('diagnostics', {}).get('solver') or {}).get('independent_kkt_verified')),
            free_hard_valid=bool((fs.get('frozen_bounds') or {}).get('valid')),
            free_numeric_verified=bool(fs.get('qdot14') is not None
                and (fs.get('qp') or {}).get('status') in ('FEASIBLE_TRACKING_OK', 'FEASIBLE_TRACKING_REJECTED')
                and (fs.get('independent_velocity_diagnostics') or {}).get('original_equality_inf', float('inf')) <= 1e-8
                and (fs.get('qp') or {}).get('diagnostics', {}).get('stationarity_inf', float('inf')) <= 1e-7),
            free_frozen_bound_violation_observed=bool(fs.get('frozen_bounds')
                and any(reason in ('FROZEN_VELOCITY_LIMIT_VIOLATION', 'FROZEN_LOOKAHEAD_JOINT_LIMIT_VIOLATION')
                    for reason in fs['frozen_bounds']['reasons'])),
            active_bounds=active_sources(bs), free_active_bounds=active_sources(fs),
            bounded_metrics=compact(bounded), free_metrics=compact(free),
            execution_authorized=False)
        pairs.append(paired)
    invalid_summaries = []
    for case in config['invalid_requests']:
        mutation = dict(case['mutation']); method = mutation.pop('method', 'bounded')
        result = driver.run(request(case, nodes[case['node_identity']], **mutation), method=method)
        invalid_summaries.append(save(case['identity'], result, 'invalid_request'))
    rollouts = []
    for case in config['rollouts']:
        result = driver.run(request(case, nodes[case['node_identity']]), method=case['method'])
        rollouts.append(save(case['identity'], result, 'bounded_rollout'))
    meaningful = [p['identity'] for p in pairs if p['current_valid'] and p['matched_input'] and p['active_bounds'] and p['bounded_hard_valid'] and p['free_numeric_verified'] and p['free_frozen_bound_violation_observed']]
    tracking_misses = [p['identity'] for p in pairs if p['current_valid'] and p['bounded_status']=='FEASIBLE_TRACKING_REJECTED']
    predicted_rejections = [s['identity'] for s in summaries if s['method']=='bounded' and s['initial_valid'] and s['geometry_rejection_after_tracking_and_hard_pass']]
    requirements = dict(all_nine_original_states_valid=len(initial)==9 and all(r['result']['valid'] for r in initial),
        paired_inputs_identical=all(p['matched_input'] for p in pairs),
        all_nine_small_positive_controls_accepted=len([p for p in pairs if p['purpose']=='small_combined'])==9
            and all(p['bounded_metrics']['accepted']==1 and p['free_metrics']['accepted']==1
                    for p in pairs if p['purpose']=='small_combined'),
        meaningful_real_FR3_active_bound_difference=bool(meaningful),
        real_QP_feasible_tracking_miss=bool(tracking_misses),
        nonzero_rollout_at_least_two_accepted=any(r['accepted']>=2 for r in rollouts),
        current_valid_predicted_geometry_rejection=bool(predicted_rejections),
        invalid_requests_rejected=all(r['status']=='INPUT_REJECTED' and r['accepted']==0 for r in invalid_summaries))
    summary = dict(schema='task28.g2_p5_qp_1.results.v1',
        status='OFFLINE_EXPERIMENT_REQUIREMENTS_MET_PENDING_REGRESSION' if all(requirements.values()) else 'PARTIAL',
        experiment_requirements=requirements,
        configuration_sha256=sha(HERE/'evidence/preregistration.json'),
        registration_sha256=sha(HERE/'evidence/run_registration.json'),
        initial_state_count=len(initial), paired_case_count=len(pairs), invalid_request_count=len(invalid_summaries),
        rollout_count=len(rollouts), meaningful_active_pairs=meaningful, feasible_tracking_misses=tracking_misses,
        predicted_geometry_rejections=predicted_rejections,
        active_source_counts=dict(sum((Counter(s['active_source_counts']) for s in summaries if s['method']=='bounded'), Counter())),
        position_bound_status='TRIGGERED' if any(s['active_source_counts'].get('position_lookahead', 0)>0 for s in summaries if s['method']=='bounded') else 'NOT_TRIGGERED',
        acceleration='DISABLED_NOT_AUTHENTICATED_NO_CAP_EXPERIMENT',
        rollouts=rollouts, cases=summaries, paired_comparisons=pairs, artifacts=inventory,
        CPU_wall_s=time.perf_counter()-started, peak_main_RSS_KiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        environment=dict(python=sys.version, platform=platform.platform(), numpy=np.__version__),
        continuous_safety='NOT_CERTIFIED_FINITE_SAMPLING_ONLY', execution_authorized=False,
        physical_timing_measured=False, new_NR_IK_or_reprojection_calls=0,
        new_R1_observations=0, stop_after='G2-P5-QP-1')
    write_json(args.output/'summary.json', summary)
    print(json.dumps(dict(experiment_requirements=requirements, status=summary['status'], CPU_wall_s=summary['CPU_wall_s'], execution_authorized=False)), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
