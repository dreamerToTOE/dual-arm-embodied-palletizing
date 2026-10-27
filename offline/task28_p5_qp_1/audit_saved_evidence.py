"""Audit saved QP-1 receipts without optimization, FK, FCL or experiments.

Only JSON/gzip parsing and independent NumPy algebra are performed. Saved G1,
FK and FCL receipts are checked for completeness and acceptance; their physics
is not rerun. Constant-reference rotations use an independent Rodrigues formula.
"""
from __future__ import annotations

import argparse
from collections import Counter
import datetime
import gzip
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_text())


def exp_rotation(vector):
    v = np.asarray(vector, dtype=float)
    angle = float(np.linalg.norm(v))
    if angle == 0.:
        return np.eye(3)
    x, y, z = v/angle
    K = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*K + (1.-np.cos(angle))*(K@K)


def constant_pose(initial, twist, time_s):
    result = np.asarray(initial, dtype=float).copy()
    result[:3, 3] += time_s*np.asarray(twist[:3])
    result[:3, :3] = exp_rotation(time_s*np.asarray(twist[3:]))@result[:3, :3]
    return result


def quaternion_matrix(quaternion):
    x, y, z, w = np.asarray(quaternion, dtype=float)
    return np.array([[1.-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1.-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1.-2*(x*x+y*y)]])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='Completed experiment directory')
    parser.add_argument('--audit', type=Path, required=True, help='New audit JSON; never overwrite')
    args = parser.parse_args()
    output = args.output.resolve()
    summary_path = output/'summary.json'
    summary = load(summary_path)
    config_path = HERE/'evidence/preregistration.json'
    registration_path = HERE/'evidence/run_registration.json'
    config, registration = load(config_path), load(registration_path)
    checks = 0
    errors = []
    metrics = Counter()
    maxima = Counter()
    accepted_minimum_position = []
    accepted_minimum_velocity = []
    records = {}
    audit_inputs = []

    def check(ok, context, detail):
        nonlocal checks
        checks += 1
        if not bool(ok):
            errors.append(dict(context=context, detail=detail))

    def near(actual, expected, context, detail, atol=1e-12):
        a, b = np.asarray(actual, dtype=float), np.asarray(expected, dtype=float)
        check(a.shape == b.shape and np.isfinite(a).all() and np.isfinite(b).all()
              and np.allclose(a, b, rtol=0., atol=atol), context, detail)

    def authorization(data, context):
        if isinstance(data, dict):
            for key, value in data.items():
                if key == 'execution_authorized':
                    metrics['execution_false_flags_checked'] += 1
                    check(value is False, context, 'execution_authorized must remain false')
                authorization(value, context)
        elif isinstance(data, list):
            for value in data:
                authorization(value, context)

    def state_receipt(state, context, q=None, X=None, require_valid=False):
        check(state is not None, context, 'original state receipt present')
        if state is None:
            return
        metrics['state_receipts_checked'] += 1
        if require_valid:
            check(state['valid'] is True and not state['reasons'], context, 'accepted state is valid without reasons')
        waypoint = state['waypoint']
        if q is not None:
            near(list(waypoint['left_q'])+list(waypoint['right_q']), q, context, 'receipt joint values bind to candidate')
        quat = waypoint['object_quaternion_xyzw']
        check(abs(float(np.linalg.norm(quat))-1.) <= 1e-10, context, 'unit quaternion')
        if X is not None:
            near(waypoint['object_position_m'], X[:3, 3], context, 'receipt Cube center binds to reference')
            near(quaternion_matrix(quat), X[:3, :3], context, 'receipt Cube orientation binds to reference')
        check(waypoint['left_attached'] is True and waypoint['right_attached'] is True,
              context, 'bilateral attachment retained')
        check(waypoint['mode'] == 'bilateral_transitioning' and waypoint['support_site'] is None,
              context, 'generic transport never gains support/contact exception')
        check(waypoint['event'] == 'observe', context, 'no attachment/release phase event')
        if state['valid']:
            closure = state['closure']
            check(closure['valid'] is True, context, 'G1 closure valid')
            for key in ('translation_error_m', 'left_object_translation_error_m', 'right_object_translation_error_m'):
                check(closure[key] <= 1e-4, context, 'G1 translation tolerance: '+key)
                maxima['valid_state_closure_translation_m'] = max(maxima['valid_state_closure_translation_m'], closure[key])
            for key in ('rotation_error_rad', 'left_object_rotation_error_rad', 'right_object_rotation_error_rad'):
                check(closure[key] <= 1e-3, context, 'G1 rotation tolerance: '+key)
                maxima['valid_state_closure_rotation_rad'] = max(maxima['valid_state_closure_rotation_rad'], closure[key])
            check(len(state['independent_fk']) == 20, context, 'all 20 independent FK comparisons present')
            for fk in state['independent_fk']:
                check(fk['translation_error_m'] <= 1e-6 and fk['rotation_error_rad'] <= 1e-6,
                      context, 'original independent FK gate: '+fk['link'])
            collision = state['collision']
            reference = collision['reference']
            check(collision['valid'] is True and reference['valid'] is True, context, 'FCL/contact receipt valid')
            check(reference['checked_shape_pairs'] == 414 and reference['skipped_acm_shape_pairs'] == 40
                  and reference['backend'] == 'FCL-0.7.0', context, 'full frozen pair inventory/backend')
            check(not collision['remaining_collisions'] and not collision['accepted_support_contacts'],
                  context, 'no remaining collision or unsupported table exception')

    check(load(output/'registered_config.json') == config, 'configuration', 'saved config exactly matches preregistration')
    check(summary['configuration_sha256'] == sha(config_path), 'configuration', 'preregistration hash')
    check(summary['registration_sha256'] == sha(registration_path), 'configuration', 'run registration hash')
    for rel, digest in registration['bound_file_hashes'].items():
        check(sha(ROOT/rel) == digest, rel, 'registered source/config hash unchanged')
    protected = load(HERE/'evidence/protected_baseline.json')
    for item in protected['files']:
        check(sha(ROOT/item['path']) == item['sha256'], item['path'], 'protected baseline hash unchanged')
    metrics['protected_baseline_files_checked'] = len(protected['files'])
    source = {}
    for item in config['references']:
        check(sha(ROOT/item['path']) == item['sha256'], item['path'], 'saved NR0 source hash')
        source[item['seed']] = load(ROOT/item['path'])
    nodes = {n['identity']: n for n in config['nodes']}
    for identity, node in nodes.items():
        saved = source[node['seed']][node['segment']]
        check(node['node_index'] == len(saved)//2 and node['segment_node_count'] == len(saved)
              and saved[node['node_index']] == [node['object_pose'], node['q14']], identity,
              'full archived q14/SE3 and deterministic midpoint identity')
    limits = load(ROOT/'fixtures/task28_g1_0_new_single_cube_v0/moveit/task28_single_cube_description/config/effective_joint_limits.json')
    qmin = np.tile(limits['position_lower_rad'], 2)
    qmax = np.tile(limits['position_upper_rad'], 2)
    vmax = np.tile(limits['velocity_max_rad_s'], 2)

    expected_cases = []
    for case in config['point_cases']:
        expected_cases.extend((case, method, 'paired_point') for method in case['methods'])
    for case in config['invalid_requests']:
        expected_cases.append((case, case['mutation'].get('method', 'bounded'), 'invalid_request'))
    expected_cases.extend((case, case['method'], 'bounded_rollout') for case in config['rollouts'])
    check(len(summary['artifacts']) == len(expected_cases) == 66, 'inventory', 'all 66 declared case artifacts')
    check({p.name for p in output.glob('case_*.json.gz')} == {a['path'] for a in summary['artifacts']},
          'inventory', 'no missing or extra case files')

    for ordinal, (artifact, expected) in enumerate(zip(summary['artifacts'], expected_cases)):
        case, method, group = expected
        context = f'{ordinal}:{case["identity"]}:{method}'
        path = output/artifact['path']
        check(path.stat().st_size == artifact['bytes'] and sha(path) == artifact['sha256'], context, 'artifact bytes/SHA256')
        audit_inputs.append(dict(path=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                                 bytes=path.stat().st_size, sha256=sha(path)))
        record = load(path)
        records[(case['identity'], method)] = record
        authorization(record, context)
        metrics['case_count'] += 1
        check(record['method'] == method and artifact['group'] == group, context, 'declared method/group')
        check(record['continuous_verifier_run'] is False and record['nr_projection_performed'] is False
              and record['phase_confirmation_observations_generated'] == 0
              and record['collision_gradient_constraint_implemented'] is False,
              context, 'no continuous verifier, NR, R1 promotion or collision-QP claim')
        check(record['injected_backend_scope'] == 'DEFAULT_UNCHANGED_FROZEN_FR3_QP0_G1_B0',
              context, 'default frozen implementation, no injected backend')
        request = record['request']
        node = nodes[case['node_identity']]
        mutation = case.get('mutation', {})
        check(request['q14'] == mutation.get('q14', node['q14'])
              and request['object_pose'] == mutation.get('object_pose', node['object_pose']),
              context, 'initial request equals saved source plus explicit registered mutation')
        check(request['desired_object_twist'] == mutation.get('desired_object_twist', case.get('desired_object_twist', [.005,-.003,.002,.01,-.005,.02])),
              context, 'fixed registered twist request')
        check(record['attempted_step_count'] == len(record['steps']) <= request['max_steps'], context, 'attempt count and fixed budget')
        if group == 'invalid_request':
            check(record['status'] == 'INPUT_REJECTED' and not record['steps']
                  and record['accepted_step_count'] == 0 and record['retained_state'] is None,
                  context, 'invalid/unsupported request rejected before integration')
            metrics['invalid_request_count'] += 1
            continue

        dt = request['dt_s']
        q = np.asarray(request['q14'], dtype=float)
        X0 = np.asarray(request['object_pose'], dtype=float)
        twist = np.asarray(request['desired_object_twist'], dtype=float)
        X = X0.copy()
        state_receipt(record['initial_state'], context+':initial', q, X, require_valid=True)
        accepted_count = 0
        previous_velocity = None
        for index, step in enumerate(record['steps']):
            ctx = context+f':step{index}'
            metrics['attempted_steps'] += 1
            check(step['step_index'] == index and step['time_s'] == index*dt, ctx, 'monotone synthetic step identity/time')
            near(step['q_initial'], q, ctx, 'current joints equal last accepted state')
            near(step['object_pose_initial'], X, ctx, 'current object equals last accepted desired reference')
            near(step['desired_object_twist'], twist, ctx, 'constant registered twist')
            if step['qdot14'] is None:
                check(not step['accepted'] and step['q_next'] is None, ctx, 'missing QP output cannot advance')
            else:
                x = np.asarray(step['qdot14'], dtype=float)
                qnext = q+dt*x
                Xnext = constant_pose(X0, twist, (index+1)*dt)
                near(step['q_next'], qnext, ctx, 'q_next equals q+dt*qdot')
                near(step['desired_object_pose_next'], Xnext, ctx, 'desired SE3 uses initial pose/fixed world twist, not achieved twist')
                maxima['joint_integration_recompute_error_rad'] = max(maxima['joint_integration_recompute_error_rad'], float(np.max(np.abs(np.asarray(step['q_next'])-qnext))))
                maxima['reference_pose_recompute_element_error'] = max(maxima['reference_pose_recompute_element_error'], float(np.max(np.abs(np.asarray(step['desired_object_pose_next'])-Xnext))))
                diag = step['qp']['diagnostics']
                B, C = np.asarray(diag['B']), np.asarray(diag['C'])
                actual, closure_v = B@x, C@x
                error = actual-twist
                near(step['independent_velocity_diagnostics']['achieved_object_twist'], actual, ctx, 'saved achieved twist recomputes')
                near(step['independent_velocity_diagnostics']['closure_velocity_residual'], closure_v, ctx, 'saved complete C residual recomputes')
                near(diag['tracking_translation_error_m_s'], np.linalg.norm(error[:3]), ctx, 'translation tracking norm recomputes')
                near(diag['tracking_rotation_error_rad_s'], np.linalg.norm(error[3:]), ctx, 'rotation tracking norm recomputes')
                bounds = step['frozen_bounds']
                lower = np.maximum(-vmax, (np.nextafter(qmin, np.inf)-q)/dt)
                upper = np.minimum(vmax, (np.nextafter(qmax, -np.inf)-q)/dt)
                near(bounds['lower_rad_s'], lower, ctx, 'authenticated lower lookahead/velocity intersection')
                near(bounds['upper_rad_s'], upper, ctx, 'authenticated upper lookahead/velocity intersection')
                near(bounds['minimum_position_margin_rad'], np.min(np.minimum(qnext-qmin, qmax-qnext)), ctx, 'position margin recomputes')
                near(bounds['minimum_velocity_margin_rad_s'], np.min(vmax-np.abs(x)), ctx, 'velocity margin recomputes')
                W = np.asarray(diag['weight_matrix'])
                H = B.T@W@B+diag['regularization']*np.eye(14)
                f = -B.T@W@twist
                near(diag['soft_tracking_objective'], .5*error@W@error+.5*diag['regularization']*(x@x), ctx, 'weighted regularized objective recomputes')
                if method == 'bounded':
                    solver = diag['solver']
                    y = np.asarray(solver['dual_equality'])
                    alpha, beta = np.asarray(solver['dual_lower']), np.asarray(solver['dual_upper'])
                    stationarity = H@x+f+C.T@y-alpha+beta
                    near(solver['kkt_stationarity_inf'], np.linalg.norm(stationarity, ord=np.inf), ctx, 'KKT stationarity independently recomputes')
                    check(solver['independent_kkt_verified'] is True
                          and np.max(np.abs(stationarity)) <= 1e-7
                          and np.min(alpha) >= -1e-7 and np.min(beta) >= -1e-7
                          and abs(solver['primal_dual_gap']) <= solver['primal_dual_gap_tolerance'],
                          ctx, 'bounded numerical KKT/dual gates pass even for tracking negatives')
                else:
                    y = np.asarray(diag['equality_multipliers'])
                    check(np.max(np.abs(H@x+f+C.T@y)) <= 1e-7 and np.max(np.abs(closure_v)) <= 1e-8,
                          ctx, 'unbounded optimum retains original equality and stationarity')
                state_receipt(step['current_state'], ctx+':current', q, X, require_valid=True)
                state_receipt(step['predicted_state'], ctx+':predicted', qnext, Xnext, require_valid=step['accepted'])
                edge = step['sampled_edge']
                check(edge is not None and edge['evaluated_sample_count'] == len(edge['samples'])
                      and edge['planned_sample_count'] == edge['evaluated_sample_count'], ctx, 'complete planned sampled-edge coverage')
                check(not edge['temporal_evidence'] and edge['resolution']['temporal_confirmation_requested'] is False
                      and edge['resolution']['temporal_observation_checks'] == 0
                      and edge['resolution']['interpolated_points_used_for_temporal_confirmation'] == 0,
                      ctx, 'samples never create R1 observations')
                metrics['B0_samples_checked'] += len(edge['samples'])
                for sample_index, sample in enumerate(edge['samples']):
                    state_receipt(sample, ctx+f':sample{sample_index}', require_valid=step['accepted'])
                if previous_velocity is not None:
                    near(step['velocity_change_rad_s'], x-previous_velocity, ctx, 'velocity change recomputes')
                    near(step['velocity_change_rate_rad_s2'], (x-previous_velocity)/dt, ctx, 'illustrative velocity change rate recomputes')
                else:
                    check(step['velocity_change_rad_s'] is None and step['velocity_change_rate_rad_s2'] is None,
                          ctx, 'first step has no invented preceding command')
                if step['accepted']:
                    check(step['qp']['status'] == 'FEASIBLE_TRACKING_OK' and not step['reasons']
                          and np.linalg.norm(error[:3]) <= 1e-4 and np.linalg.norm(error[3:]) <= 1e-3
                          and np.max(np.abs(closure_v)) <= 1e-8
                          and np.all(qnext > qmin) and np.all(qnext < qmax)
                          and np.all(np.abs(x) <= vmax+1e-10)
                          and edge['status'] == 'sampled_edge_valid', ctx, 'all accepted numerical/state/edge hard gates')
                    accepted_minimum_position.append(float(np.min(np.minimum(qnext-qmin, qmax-qnext))))
                    accepted_minimum_velocity.append(float(np.min(vmax-np.abs(x))))
                    q, X, previous_velocity = qnext, np.asarray(step['desired_object_pose_next']), x
                    accepted_count += 1
                    metrics['accepted_steps'] += 1
            if not step['accepted']:
                check(index == len(record['steps'])-1 and record['failure']['step_index'] == index,
                      ctx, 'first failure is final attempt; no rejected-state continuation')
                if step['q_next'] is not None:
                    check(record['failure']['retained_failing_candidate'] is True, ctx, 'rejected candidate remains in evidence')
                metrics['failed_steps'] += 1
        check(record['accepted_step_count'] == accepted_count, context, 'accepted prefix count')
        near(record['retained_state']['q14'], q, context, 'retained joints are last accepted, never failing candidate')
        near(record['retained_state']['object_pose'], X, context, 'retained Cube is last accepted reference')
        near(record['retained_state']['time_s'], accepted_count*dt, context, 'retained synthetic time')

    paired_actual_violations = []
    for case in config['point_cases']:
        bounded, free = (records[(case['identity'], m)] for m in ('bounded', 'unbounded_reference'))
        check(bounded['request'] == free['request'], case['identity'], 'paired methods receive identical complete request')
        bs, fs = bounded['steps'][0], free['steps'][0]
        near(bs['qp']['diagnostics']['B'], fs['qp']['diagnostics']['B'], case['identity'], 'same B')
        near(bs['qp']['diagnostics']['C'], fs['qp']['diagnostics']['C'], case['identity'], 'same C')
        near(bs['qp']['diagnostics']['weight_matrix'], fs['qp']['diagnostics']['weight_matrix'], case['identity'], 'same objective weights')
        x = np.asarray(fs['qdot14'])
        qnext = np.asarray(fs['q_next'])
        violation = bool(np.any(np.abs(x)>vmax+1e-10) or np.any(qnext<=qmin) or np.any(qnext>=qmax))
        if violation:
            paired_actual_violations.append(case['identity'])
    check(set(summary['meaningful_active_pairs']).issubset(paired_actual_violations),
          'summary', 'every claimed meaningful pair has actual free velocity/position violation')
    authorization(summary, 'summary')
    for path in (summary_path, output/'registered_config.json', output/'initial_states.json.gz', config_path, registration_path):
        audit_inputs.append(dict(path=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                                 bytes=path.stat().st_size, sha256=sha(path)))
    report = dict(
        schema='task28.g2_p5_qp_1.saved_evidence_audit.v1',
        status='SAVED_EVIDENCE_AUDIT_PASS' if not errors else 'SAVED_EVIDENCE_AUDIT_FAILED',
        audit_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='Independent saved-data parsing and arithmetic; no QP optimization, FK/FCL execution, planning or experiment rerun',
        audit_script=dict(path=str(Path(__file__).resolve().relative_to(ROOT)), sha256=sha(Path(__file__).resolve())),
        checks_performed=checks, error_count=len(errors), errors=errors,
        counts=dict(metrics), maximum_recompute_and_valid_state_residuals=dict(maxima),
        minimum_accepted_position_margin_rad=min(accepted_minimum_position) if accepted_minimum_position else None,
        minimum_accepted_velocity_margin_rad_s=min(accepted_minimum_velocity) if accepted_minimum_velocity else None,
        independently_observed_free_bound_violation_pairs=paired_actual_violations,
        input_artifacts=audit_inputs,
        environment=dict(python=sys.version, numpy=np.__version__),
        limitations=['Original FK and FCL receipts are verified for content/completeness, not recomputed by this audit',
                     'No new physical, real-time or continuous-safety evidence is generated',
                     'Same saved environment; this is not third-party or cross-platform certification'],
        continuous_safety='NOT_CERTIFIED_FINITE_SAMPLING_ONLY', execution_authorized=False)
    with args.audit.open('x') as out:
        json.dump(report, out, ensure_ascii=False, indent=2, allow_nan=False)
        out.write('\n')
    print(json.dumps({key: report[key] for key in ('status', 'checks_performed', 'error_count', 'counts')}, indent=2))
    return 0 if not errors else 1


if __name__ == '__main__':
    raise SystemExit(main())
