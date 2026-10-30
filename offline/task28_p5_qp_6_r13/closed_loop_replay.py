"""CPU closed-loop qualification diagnostic; no Isaac, ROS or actuation imports.

Each episode re-solves the unmodified R12 QP and torque law at every step.
The approximate plant is NOT PhysX TGS: M0 qdd = tau - archived_g - archived_c,
with semi-implicit Euler and exact assumed gravity cancellation. Its validity
is checked by comparing predicted kinematic outputs with the R12 archive
before sensitivity is interpreted. Gravity and Coriolis schedules are inputs
from the same archive, so this is not independent plant validation. No plant
parameters or controller parameters are fitted.
"""
from __future__ import annotations
import argparse
import copy
import gzip
import hashlib
import json
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r8.controller import gains, torque
from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate, Rejected
from offline.task28_p5_qp_6_r12.qp_math import (
    solve_pose_aware_qp, virtual_pose, virtual_twist_quintic,
    velocity_capture_reference,
)
from offline.task28_p5_qp_6_r12.virtual_frames import VirtualCooperativeFrame

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / 'task28_p5_qp_6_r12'
EVIDENCE = HERE / 'evidence'
CASES = ('V10-A', 'V10-B', 'V10-C')
DT = 1 / 60
BASELINE = '52d07e6a9c4ac482db4887483cbf8ab0cbf510d1'
SCENARIOS = ('nominal', 'initial_ulp_minus', 'initial_ulp_plus',
             'measured_dt_minus', 'measured_dt_plus',
             'observed_mass_delta_minus', 'observed_mass_delta_plus')


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def numerical_diagnostics(diagnostics):
    """Retain solver results, excluding measured wall-clock durations.

    A trace hash describes recomputed numerical results. Process timing is an
    episode receipt field and cannot establish numerical nondeterminism.
    """
    return {key: value for key, value in diagnostics.items()
            if key not in ('calculation_time_s', 'solve_wall_time_s')}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def inputs(case):
    d = BASE / 'evidence/diagnostic' / case
    result = json.loads((d / 'result.json').read_text())
    rows = [json.loads(line) for line in (d / 'frames.jsonl').read_text().splitlines()]
    rows = [r for r in rows if 'r12_qp' in r]
    cfg = json.loads((BASE / 'source_contract.json').read_text())
    reg = json.loads((BASE / 'registrations.json').read_text())[case]
    if reg['feedback_gain_scale'] != .75 or reg['velocity_capture']['tau_s'] != .10:
        raise ValueError('FROZEN_R12_PARAMETERS_CHANGED')
    if cfg['acceptance']['virtual_object_tracking_peak_m'] != .00025 or cfg['acceptance']['relative_tcp_translation_peak_m'] != .00015:
        raise ValueError('FROZEN_R12_THRESHOLDS_CHANGED')
    return result, rows, cfg, reg


def derive_manifest():
    evaluation = json.loads((BASE / 'evidence/evaluation.json').read_text())
    sources = [BASE / 'runtime_virtual.py', BASE / 'qp_math.py', BASE / 'virtual_frames.py',
               BASE / 'source_contract.json', BASE / 'registrations.json', BASE / 'evidence/evaluation.json',
               HERE / 'closed_loop_replay.py',
               HERE.parent / 'task28_p5_qp_6_r8/controller.py',
               HERE.parent / 'task28_p5_qp_6_r8/source_gate.py',
               HERE.parent / 'task28_p5_qp_6_r9/qp_math.py',
               HERE.parent / 'task28_p5_qp_0/solver.py',
               HERE.parent / 'task28_g1_1/task28_validator/model.py',
               HERE.parent / 'task28_g1_1/task28_validator/se3.py']
    parameters = {}
    for case in CASES:
        result, rows, cfg, reg = inputs(case)
        initial = result['ownership_initial_state']
        q0 = np.r_[initial['arms']['left']['q'][:7], initial['arms']['right']['q'][:7]]
        plus = np.nextafter(q0.astype(np.float32), np.float32(np.inf)).astype(float) - q0
        minus = np.nextafter(q0.astype(np.float32), np.float32(-np.inf)).astype(float) - q0
        times = np.array([initial['time_s']] + [r['time_s'] for r in rows])
        dt_error = float(np.max(np.abs(np.diff(times) - DT)))
        mass = {}
        for side in ('left', 'right'):
            m0 = np.asarray(initial['arms'][side]['M'])[:7, :7]
            m0 = (m0 + m0.T) / 2
            deltas = [np.asarray(r['arms'][side]['M'])[:7, :7] - m0 for r in rows]
            index = int(np.argmax([np.linalg.norm(d, 2) for d in deltas]))
            delta = (deltas[index] + deltas[index].T) / 2
            mass[side] = {
                'delta_matrix': delta.tolist(), 'source_post_step': rows[index]['physics_step'],
                'relative_spectral_norm': float(np.linalg.norm(delta, 2) / np.linalg.norm(m0, 2)),
                'minus_min_eigenvalue': float(np.linalg.eigvalsh(m0 - delta).min()),
                'plus_min_eigenvalue': float(np.linalg.eigvalsh(m0 + delta).min()),
            }
        parameters[case] = {
            'q_initial_ulp_plus_rad': plus.tolist(), 'q_initial_ulp_minus_rad': minus.tolist(),
            'observed_timestamp_dt_error_s': dt_error,
            'observed_fk_tcp_discrepancy_max_m': max(r['arms'][s]['tcp_fk_error_m'] for r in rows for s in ('left', 'right')),
            'observed_mass_variation': mass,
        }
        sources.extend([BASE / 'evidence/diagnostic' / case / f for f in ('result.json', 'frames.jsonl')])
    return {
        'schema': 'TASK28_R13_CLOSED_LOOP_PREREGISTRATION_V1', 'baseline_head': BASELINE,
        'controller': {'feedback_gain_scale': .75, 'tau_s': .10, 'changed': False},
        'acceptance': {'object_peak_m': .00025, 'relative_translation_peak_m': .00015},
        'nominal_episodes_per_case': 20,
        'sensitivity_episodes_per_case_scenario': 1,
        'scenarios': list(SCENARIOS), 'perturbations': parameters,
        'perturbation_rationale': {
            'initial_ulp': 'One adjacent float32 representable joint position in each signed direction; archive q is float32-derived. Numerical representation sensitivity only, not calibration uncertainty.',
            'measured_dt': 'Plus/minus largest measured R12 post-state timestamp delta from nominal 1/60; existing 1e-8 s dt gate retained; does not cover arbitrary scheduler jitter.',
            'mass_delta': 'Signed M0 plus/minus largest measured 7x7 spectral-norm departure in the archived trajectory, preserving SPD. Posture variation sensitivity, NOT a calibrated bound on model uncertainty.',
            'delay': 'Exactly one 60 Hz sample, tested only against existing gate with declared stale and coherent self-reported-fresh metadata. No delayed command is applied.',
        },
        'plant': {
            'equation': 'dq_next=dq+dt*solve(M0+signed_observed_delta, P+D-archived_c); q_next=q+dt*dq_next',
            'gravity': 'R12 torque law uses archived g at each pre-state; plant subtracts identical g, an exact gravity-cancellation assumption.',
            'tcp': 'RobotModel FK with one constant initial world rigid correction per arm fixed from ownership state; no trajectory error fitting.',
            'omitted': ['PhysX TGS solver projection and end-step velocity semantics', 'finger/arm dynamic coupling', 'joint friction and constraint forces', 'state-dependent mass/coriolis/gravity beyond archived schedule', 'collisions and contacts'],
            'parameter_fitting': False,
        },
        'model_qualification': {
            'necessary_test': 'max same-index predicted-versus-archived TCP translation discrepancy <= archived V10-C remaining object margin; necessary only, never sufficient for hardware qualification',
            'v10c_remaining_margin_m': .00025 - evaluation['cells']['V10-C']['virtual_object_tracking_peak_m'],
            'invalid_model_policy': 'MODEL_NOT_QUALIFIED_FOR_MARGIN; robustness NOT_ESTABLISHED even if model metrics pass or fail thresholds',
        },
        'statistics': 'Per-step e=max(left_implied_object_translation,right_implied_object_translation); numpy.percentile(...,95,method=linear); equal 210 steps per trajectory. Repeatability is distribution of independently recomputed episode peaks. Trace hashes exclude calculation_time_s and solve_wall_time_s; elapsed_s remains in episode receipts.',
        'safety_scope': 'CPU diagnostic duplicates QP, per-joint effort/rate, joint/drift/native-dq checks; it does not implement the complete runtime scene/collision/source/finger/TCP-envelope gates and cannot authorize execution.',
        'feedback_timing_scope': 'One-sample metadata fault injection exercises only the existing pair gate; delayed-feedback closed-loop dynamics and a wall-clock freshness guarantee are not established.',
        'sources_sha256': {str(p.relative_to(ROOT)): digest(p) for p in sources},
        'execution_authorized': False, 'robot_command_writes': 0,
    }


def summarize(records, cfg, failure):
    obj = np.array([r['object_error_m'] for r in records])
    rel = np.array([r['relative_error_m'] for r in records])
    threshold = cfg['acceptance']['virtual_object_tracking_peak_m']
    relative_limit = cfg['acceptance']['relative_tcp_translation_peak_m']
    return {
        'frames': len(records), 'plant_status': 'COMPLETE' if failure is None else 'GATE_REJECTED',
        'first_failure': failure, 'tracking_peak_m': float(obj.max()) if len(obj) else None,
        'tracking_mean_m': float(obj.mean()) if len(obj) else None,
        'tracking_p95_m': float(np.percentile(obj, 95, method='linear')) if len(obj) else None,
        'relative_tcp_max_m': float(rel.max()) if len(rel) else None,
        'object_exceedance_frames': [r['step'] for r in records if r['object_error_m'] > threshold],
        'relative_exceedance_frames': [r['step'] for r in records if r['relative_error_m'] > relative_limit],
        'solver_status_counts': dict(Counter(r['qp']['status'] for r in records)),
        'qp_solves': len(records) + int(failure is not None and failure.get('stage') != 'precommand'),
        'active_box_frames': [r['step'] for r in records if r['qp_box_active_indices']],
        'common_correction_clip_frames': [r['step'] for r in records if r['common_correction_clipped']],
        'relative_correction_clip_frames': [r['step'] for r in records if r['relative_correction_clipped']],
        'hard_relative_residual_peak': max((r['qp']['hard_relative_residual_inf'] for r in records), default=None),
        'qdot_peak_rad_s': max((max(abs(x) for x in r['qdot']) for r in records), default=None),
        'effort_peak_Nm': max((max(abs(x) for x in r['torque_Nm']) for r in records), default=None),
        'tracking_verdict_scope': 'APPROXIMATE_PLANT_ONLY_NOT_RUNTIME_OR_HARDWARE',
    }


def simulate_episode(case, scenario, manifest, max_steps=None):
    """Reset state and re-run every QP and controller call; never scale a trace."""
    result, archived, cfg, reg = inputs(case)
    state0 = result['ownership_initial_state']
    model = RobotModel()
    vf = result['virtual_frame']
    frame = VirtualCooperativeFrame(*(np.asarray(vf[k]) for k in ('T_WV', 'T_VL', 'T_VR')))
    q_hold = np.r_[state0['arms']['left']['q'][:7], state0['arms']['right']['q'][:7]]
    dq0 = np.r_[state0['arms']['left']['dq'][:7], state0['arms']['right']['dq'][:7]]
    q = q_hold.copy(); dq = dq0.copy()
    par = manifest['perturbations'][case]
    if scenario.startswith('initial_ulp_'):
        q += np.asarray(par['q_initial_ulp_' + scenario.rsplit('_', 1)[1] + '_rad'])
    dt = DT
    if scenario.startswith('measured_dt_'):
        dt += (-1 if scenario.endswith('minus') else 1) * par['observed_timestamp_dt_error_s']
    if abs(dt - DT) > cfg['state']['dt_tolerance_s']:
        raise ValueError('PERTURBATION_OUTSIDE_FROZEN_DT_GATE')
    scale = reg['feedback_gain_scale']
    object_gain = np.zeros((6, 6))
    relative_gain = np.zeros((6, 6))
    if reg['feedback'] in ('common_object', 'common_object_and_relative_pose'):
        object_gain = np.diag([scale * cfg['feedback']['common_translation_gain_s_inv']] * 3
                              + [scale * cfg['feedback']['common_rotation_gain_s_inv']] * 3)
    if reg['feedback'] == 'common_object_and_relative_pose':
        relative_gain = np.diag([scale * cfg['feedback']['relative_translation_gain_s_inv']] * 3
                                + [scale * cfg['feedback']['relative_rotation_gain_s_inv']] * 3)
    matrices, plants, transforms = {}, {}, {}
    for side, sl in (('left', slice(0, 7)), ('right', slice(7, 14))):
        matrices[side] = gains(state0['arms'][side]['M'], 'frozen_mass_smooth')
        m = np.asarray(state0['arms'][side]['M'])[:7, :7]; m = (m + m.T) / 2
        if scenario.startswith('observed_mass_delta_'):
            m = m + (-1 if scenario.endswith('minus') else 1) * np.asarray(par['observed_mass_variation'][side]['delta_matrix'])
        if np.linalg.eigvalsh(m).min() <= 0:
            raise ValueError('MASS_PERTURBATION_NOT_SPD')
        plants[side] = np.linalg.inv(m)
        transforms[side] = np.asarray(state0['arms'][side]['tcp_world']) @ inverse(model.fk(side, q_hold[sl]))
    qpath = np.zeros(14); previous_qdot = np.zeros(14)
    previous_effort = {s: np.asarray(state0['arms'][s]['applied_command_not_sensor'])[:7] for s in ('left', 'right')}
    records = []; failure = None
    for k in range(min(len(archived), max_steps or len(archived))):
        pre_archived = state0 if k == 0 else archived[k-1]
        tcp = {side: transforms[side] @ model.fk(side, q[sl]) for side, sl in (('left', slice(0, 7)), ('right', slice(7, 14)))}
        t = k * DT  # R12 reference index clock stays frozen even when plant dt varies.
        goal = virtual_pose(frame, (k+1)*DT, reg['duration_s'], reg['amplitude_m'], reg['axis'][-1])
        vff, _, _ = virtual_twist_quintic(t, reg['duration_s'], reg['amplitude_m'], reg['axis'][-1])
        qref = q_hold + qpath
        qd, diag = solve_pose_aware_qp(model, q, frame, tcp['left'], tcp['right'], goal,
            virtual_feedforward=vff, previous=previous_qdot, dt=DT,
            object_gain=object_gain, relative_gain=relative_gain,
            qdot_limit=cfg['limits']['joint_velocity_rad_s'],
            correction_linear_limit=cfg['feedback']['correction_twist_limit_linear_m_s'],
            correction_angular_limit=cfg['feedback']['correction_twist_limit_angular_rad_s'])
        if qd is None or diag.get('hard_relative_residual_inf', 1.) > 2e-7:
            failure = {'stage': 'QP', 'step': k, 'reason': diag['status']}; break
        dqref = qd + velocity_capture_reference(t, dq0, .10)
        qpath += qd * DT; previous_qdot = qd.copy()
        efforts, accel = [], []
        for side, sl in (('left', slice(0, 7)), ('right', slice(7, 14))):
            source = pre_archived['arms'][side]
            synthetic = {'q': q[sl], 'dq': dq[sl], 'g': source['g']}
            effort, components = torque(synthetic, qref[sl], dqref[sl], *matrices[side])
            if np.any(np.abs(effort) > cfg['limits']['research_effort_abs_max_Nm']) or np.any(np.abs(effort - previous_effort[side]) > np.array(cfg['limits']['arm_effort_rate_max_Nm_s']) * dt):
                failure = {'stage': 'precommand', 'step': k, 'reason': 'EFFORT_OR_RATE', 'side': side}; break
            previous_effort[side] = effort.copy()
            accel.extend(plants[side] @ (effort - np.asarray(source['g'])[:7] - np.asarray(source['c'])[:7]))
            efforts.extend(effort)
        if failure:
            break
        q_pre = q.copy(); dq_pre = dq.copy()
        dq = dq + dt * np.asarray(accel)
        q = q + dt * dq
        if not model.within_limits(q[:7]) or not model.within_limits(q[7:]) or np.max(np.abs(q-q_hold)) > .02 or np.max(np.abs(dq)) > .3:
            failure = {'stage': 'poststep', 'step': k, 'reason': 'POSITION_OR_NATIVE_VELOCITY_GATE'}; break
        tcp_post = {side: transforms[side] @ model.fk(side, q[sl]) for side, sl in (('left', slice(0, 7)), ('right', slice(7, 14)))}
        left_obj = tcp_post['left'] @ inverse(frame.T_VL); right_obj = tcp_post['right'] @ inverse(frame.T_VR)
        el = residual(left_obj, goal); er = residual(right_obj, goal); relative = residual(left_obj, right_obj)
        lower, upper = np.array(diag['lookahead_lower']), np.array(diag['lookahead_upper'])
        clip = np.array([cfg['feedback']['correction_twist_limit_linear_m_s']] * 3
                        + [cfg['feedback']['correction_twist_limit_angular_rad_s']] * 3)
        qp = numerical_diagnostics(diag)
        records.append({'step': k, 'post_time_s': (k+1)*dt, 'reference_time_s': (k+1)*DT,
            'q_pre': q_pre.tolist(), 'dq_pre': dq_pre.tolist(), 'q': q.tolist(), 'dq': dq.tolist(),
            'q_ref': qref.tolist(), 'dq_ref': dqref.tolist(), 'qdot': qd.tolist(), 'torque_Nm': list(efforts),
            'tcp_left': tcp_post['left'].tolist(), 'tcp_right': tcp_post['right'].tolist(), 'virtual_goal': goal.tolist(),
            'object_left_error_m': el[0], 'object_right_error_m': er[0], 'object_error_m': max(el[0], er[0]),
            'relative_error_m': relative[0], 'relative_rotation_rad': relative[1],
            'qp': qp, 'qp_box_active_indices': np.flatnonzero((np.abs(qd-lower) <= 2e-8) | (np.abs(qd-upper) <= 2e-8)).tolist(),
            'common_correction_clipped': bool(np.any(np.abs(object_gain @ np.asarray(diag['object_error_common_world'])) > clip)),
            'relative_correction_clipped': bool(np.any(np.abs(diag['hard_relative_pose_target_raw']) > clip)),
        })
    summary = summarize(records, cfg, failure)
    errors = [max(residual(np.asarray(r['tcp_' + s]), np.asarray(a['arms'][s]['tcp_world']))[0] for s in ('left','right')) for r,a in zip(records, archived)]
    archived_peak = max(max(residual(np.asarray(a['arms'][s]['tcp_world']) @ inverse(frame.T_VL if s=='left' else frame.T_VR), np.asarray(a['r12_qp']['virtual_goal']))[0] for s in ('left','right')) for a in archived)
    summary['archive_comparison'] = {'same_index_tcp_discrepancy_peak_m': max(errors, default=None),
        'same_index_tcp_discrepancy_p95_m': float(np.percentile(errors,95,method='linear')) if errors else None,
        'archived_tracking_peak_m': archived_peak,
        'peak_tracking_metric_difference_m': abs(summary['tracking_peak_m']-archived_peak) if records else None,
        'q_discrepancy_peak_rad': max((max(abs(np.array(r['q']) - np.r_[a['arms']['left']['q'][:7], a['arms']['right']['q'][:7]])) for r,a in zip(records,archived)), default=None),
        'native_dq_discrepancy_peak_rad_s': max((max(abs(np.array(r['dq']) - np.r_[a['arms']['left']['dq'][:7], a['arms']['right']['dq'][:7]])) for r,a in zip(records,archived)), default=None)}
    return summary, records


def gate_faults():
    result, _, _, _ = inputs('V10-C')
    good = result['ownership_initial_state']['arms']
    findings = {}
    for name, declared_stale, dt_fault in (('declared_one_sample_stale', True, False), ('coherent_one_sample_stale_marked_fresh', False, False), ('dt_just_outside_existing_gate', False, True)):
        states = copy.deepcopy(good)
        for s in states.values():
            s['physics_step'] -= 1; s['time_s'] -= DT
            if declared_stale: s['fresh_copy'] = False
            if dt_fault: s['dt_s'] = float(np.nextafter(DT+1e-8, np.inf))
        try:
            R8DiagnosticGate().validate_pair(states)
            findings[name] = {'status': 'ACCEPTED_BY_PAIR_GATE', 'command_applied': False,
                'limitation': 'Pair gate checks agreement and caller fresh_copy assertion; no external current-cursor comparison. Later adapter post-step cursor does not prove pre-command stale rejection.'}
        except Rejected as exc:
            findings[name] = {'status': 'REJECTED_BEFORE_COMMAND', 'reason': str(exc), 'command_applied': False}
    return findings


def run_job(job):
    case, scenario, repetition, manifest = job
    start = time.monotonic()
    summary, records = simulate_episode(case, scenario, manifest)
    data = encoded(records)
    sha = hashlib.sha256(data).hexdigest()
    trace = EVIDENCE / 'closed_loop_traces' / (sha + '.json.gz')
    if not trace.exists():
        trace.parent.mkdir(parents=True, exist_ok=True)
        with trace.open('wb') as raw:
            with gzip.GzipFile(fileobj=raw,mode='wb',mtime=0) as stream:
                stream.write(data)
    return {'case':case, 'scenario':scenario, 'repetition':repetition,
            'elapsed_s':time.monotonic()-start, 'trace_sha256_uncompressed':sha,
            'trace':str(trace.relative_to(ROOT)), 'summary':summary,
            'computed_qp_calls':summary['qp_solves'], 'independent_state_reset':True}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--freeze', action='store_true')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--workers', type=int, default=2)
    args = parser.parse_args()
    manifest_path = EVIDENCE / 'closed_loop_manifest.json'
    if args.freeze:
        if manifest_path.exists(): raise FileExistsError('Manifest already frozen; do not retune after observing results')
        write(manifest_path, derive_manifest())
        print('FROZEN', digest(manifest_path), flush=True)
    if not args.run: return
    manifest = json.loads(manifest_path.read_text())
    for path, expected in manifest['sources_sha256'].items():
        if digest(ROOT / path) != expected: raise ValueError('SOURCE_HASH_CHANGED:' + path)
    receipts = []
    # Finish nominal model falsification before interpreting any perturbation.
    for case in CASES:
        receipt = run_job((case,'nominal',0,manifest)); receipts.append(receipt)
        print(case,'nominal',json.dumps(receipt['summary']['archive_comparison']), flush=True)
    margin = manifest['model_qualification']['v10c_remaining_margin_m']
    validity = {r['case']:{'status':'MODEL_NOT_QUALIFIED_FOR_MARGIN' if r['summary']['frames'] != 210 or r['summary']['archive_comparison']['same_index_tcp_discrepancy_peak_m'] > margin else 'NECESSARY_DISCREPANCY_TEST_ONLY_PASSED',
                 'same_index_tcp_discrepancy_peak_m':r['summary']['archive_comparison']['same_index_tcp_discrepancy_peak_m'],
                 'v10c_remaining_margin_m':margin} for r in receipts}
    write(EVIDENCE/'closed_loop_model_validation.json', {'schema':'TASK28_R13_PLANT_QUALIFICATION_V1', 'manifest_sha256':digest(manifest_path), 'cases':validity, 'robustness_qualification':'NOT_ESTABLISHED', 'gate_faults':gate_faults()})
    jobs = [(c,'nominal',n,manifest) for c in CASES for n in range(1,20)]
    jobs += [(c,s,0,manifest) for c in CASES for s in SCENARIOS if s!='nominal']
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for index, receipt in enumerate(pool.map(run_job,jobs)):
            receipts.append(receipt)
            if (index+1)%10==0: print('episodes',len(receipts),flush=True)
    cells = {}
    for case in CASES:
        reps=[r for r in receipts if r['case']==case and r['scenario']=='nominal']
        peak=np.array([r['summary']['tracking_peak_m'] for r in reps])
        cells[case]={'episodes':len(reps),'independent_qp_calls':sum(r['computed_qp_calls'] for r in reps),
            'unique_state_trace_hashes':len({r['trace_sha256_uncompressed'] for r in reps}),
            'peak_distribution_m':{'max':float(peak.max()),'mean':float(peak.mean()),'p95':float(np.percentile(peak,95,method='linear'))},
            'model_validation':validity[case], 'runtime_repeatability_claim':False,
            'robustness_status':'NOT_ESTABLISHED',
            'sensitivity_scope':'Unqualified approximate plant diagnostic, no calibrated model uncertainty bound; signed coordinate-wide ULP and observed dt/M variation do not span all disturbances.'}
    write(EVIDENCE/'closed_loop_results.json', {'schema':'TASK28_R13_CLOSED_LOOP_RESULTS_V1', 'baseline_head':BASELINE,
        'manifest_sha256':digest(manifest_path),'cells':cells,'episodes':receipts,'episode_count':len(receipts),
        'computed_qp_calls':sum(r['computed_qp_calls'] for r in receipts),
        'robustness_status':'NOT_ESTABLISHED','gate_faults':gate_faults(),
        'physical_execution':False,'runtime_acceptance_claim':False,'controller_changed':False})
    print('COMPLETE',len(receipts),flush=True)


if __name__ == '__main__': main()
