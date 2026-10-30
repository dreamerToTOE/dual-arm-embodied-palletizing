"""Read-only phase review; registered conditions and R13 scope remain separate.

A receipt read twenty times is not twenty diagnostic processes.  Only fresh
Isaac receipts plus their own raw evidence enter the repetition inventory.
"""
from __future__ import annotations

import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np

from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate, Rejected

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE.parent / 'task28_p5_qp_6_r12'
BASELINE = '52d07e6a9c4ac482db4887483cbf8ab0cbf510d1'
CELLS = ('V10-A', 'V10-B', 'V10-C')


def finite_le(value, limit):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value <= limit


def frozen_cell_review(evaluation, result, process, registration, contract, acceptance, source_verified, frames):
    """Check the frozen gates from independent evaluator and raw run evidence."""
    gates = {}
    accuracy = evaluation.get('accuracy', {})
    for name in ('virtual_object_tracking_peak_m', 'relative_tcp_translation_peak_m',
                 'relative_tcp_rotation_peak_rad', 'joint_tracking_peak_rad', 'native_dq_peak_rad_s'):
        metric = accuracy.get(name, {})
        gates[name] = (metric.get('limit') == acceptance[name] and metric.get('pass') is True
                       and finite_le(metric.get('value'), acceptance[name]))
    gates['independent_evaluator_eligible'] = evaluation.get('eligible_for_v10d') is True
    gates['runtime_complete'] = (result.get('status') == 'NO_CONTACT_R12_VIRTUAL_COOPERATION_COMPLETE'
                                 and result.get('first_failure') is None
                                 and result.get('actual_effort_commands') == registration['steps'])
    gates['source_binding'] = source_verified is True
    gates['source_and_frozen_controller'] = (
        result.get('level1_source') == 'PASS'
        and result.get('level2_source') == 'R12_RUNTIME_CANDIDATE_PENDING_INDEPENDENT_EVALUATION'
        and result.get('source_asset_sha256') == contract['fr3_asset_sha256']
        and result.get('registration') == registration
        and result.get('feedback_gain_scale') == .75
        and result.get('velocity_capture', {}).get('tau_s') == .1)
    gates['independent_process'] = (process.get('verified') is True
                                    and process.get('normal_exit') is True
                                    and process.get('exit_code') == 0)
    gates['no_contact_cube_suction_d6_ros'] = (
        evaluation.get('no_contact_status') == 'PASS'
        and evaluation.get('unexpected_collision') is None
        and finite_le(evaluation.get('cube_displacement_m'), acceptance['cube_translation_m'])
        and evaluation.get('suction_commands') == 0 and evaluation.get('D6_commands') == 0
        and result.get('cube_contact_commands') == 0 and result.get('suction_commands') == 0
        and result.get('virtual_execution', {}).get('D6_commands') == 0
        and result.get('ros_graph_suppressed') is True)
    dt = contract['state']['dt_s']
    tolerance = contract['state']['dt_tolerance_s']
    times = result.get('callback_dts_s', [])
    configured = result.get('configured_dt_s')
    gates['timing'] = (isinstance(configured, (int, float)) and math.isfinite(configured)
                      and abs(configured-dt) <= tolerance and bool(times)
                      and all(isinstance(t, (int, float)) and math.isfinite(t) and abs(t-dt) <= tolerance for t in times))
    sequence = result.get('command_sequence', [])
    absolute = contract['limits']['research_effort_abs_max_Nm']
    rates = contract['limits']['arm_effort_rate_max_Nm_s']
    effort_ok = len(sequence) == registration['steps']
    # The frozen adapter rate-checks the first command against the owned
    # pre-command buffer too.  Consecutive-command differences alone omit it.
    initial = result.get('ownership_initial_state', {}).get('arms', {})
    previous = {}
    for side in ('left', 'right'):
        applied = initial.get(side, {}).get('applied_command_not_sensor', [])
        if len(applied) != 9 or not all(finite_le(abs(x), float('inf')) for x in applied):
            effort_ok = False
        else:
            previous[side] = applied[:7]
    for command in sequence:
        for side in ('left', 'right'):
            efforts = command.get('effort_arm', {}).get(side, [])
            if len(efforts) != 7 or not all(finite_le(abs(x), limit) for x, limit in zip(efforts, absolute)):
                effort_ok = False
                continue
            if side in previous:
                effort_ok &= all(finite_le(abs(x-y)/dt, limit) for x, y, limit in zip(efforts, previous[side], rates))
            previous[side] = efforts
    gates['per_joint_effort_and_rate'] = bool(effort_ok)
    virtual = [row for row in frames if 'r12_qp' in row]
    source_gate = R8DiagnosticGate(dt=dt)
    source_states_ok = bool(virtual)
    for row in virtual:
        try:
            source_gate.validate_pair(row.get('arms', {}), require_disabled=True)
            source_states_ok &= all(
                row['arms'][side].get('physics_step') == row.get('physics_step')
                and row['arms'][side].get('time_s') == row.get('time_s')
                for side in ('left', 'right'))
        except (Rejected, TypeError, ValueError, KeyError):
            source_states_ok = False
    gates['all_sample_frozen_R8_state_gate'] = bool(source_states_ok)
    cube0 = np.asarray(result.get('ownership_initial_state', {}).get('cube_world', []), dtype=float)
    gates['all_sample_cube_immobile'] = (
        cube0.shape == (4, 4) and np.isfinite(cube0).all() and bool(virtual)
        and all(np.asarray(row.get('cube_world', [])).shape == (4, 4)
                and np.isfinite(np.asarray(row['cube_world'], dtype=float)).all()
                and np.max(np.abs(np.asarray(row['cube_world'], dtype=float) - cube0)) <= 1e-8
                for row in virtual))
    gates['all_sample_qp_fcl_joint_and_sink'] = (
        len(virtual) == registration['steps']
        and finite_le(evaluation.get('qdot_peak_rad_s'), contract['limits']['qdot_abs_max_rad_s'])
        and all(row.get('collision', {}).get('valid') is True
                and row.get('collision', {}).get('unexpected') == []
                and row['r12_qp'].get('diagnostics', {}).get('status') == 'QP_OPTIMAL'
                and finite_le(row['r12_qp'].get('diagnostics', {}).get('hard_relative_residual_inf'), 2e-7)
                and all(row.get('arms', {}).get(side, {}).get('within_original_joint_limits') is True
                        and row['arms'][side].get('competing_sink') is False
                        and row['arms'][side].get('unexpected_contact') is False for side in ('left', 'right'))
                for row in virtual))
    return {'pass': all(gates.values()), 'gates': gates,
            'failed_or_missing': [key for key, ok in gates.items() if not ok]}


def evaluate_admission(registrations, baseline_cells, source_binding, repetitions=None,
                       continuous_safety=None, robustness=None, task_scope=None):
    """Pure decision helper: missing evidence fails closed; no command is emitted."""
    repetitions = repetitions or {}
    safety = continuous_safety or {}
    scope = task_scope or {}
    source_ok = source_binding.get('verified') is True
    cells = {}
    unique_diagnostics = set()
    for cell in CELLS:
        historical = baseline_cells.get(cell, {})
        baseline_ok = source_ok and historical.get('pass') is True
        if baseline_ok and historical.get('independent_process_id'):
            unique_diagnostics.add(historical['independent_process_id'])
        fresh = repetitions.get(cell, [])
        identities = [(x.get('repeat'), x.get('independent_process_id')) for x in fresh]
        exact_twenty = (len(fresh) == 20 and {x.get('repeat') for x in fresh} == set(range(1, 21))
                        and all(p for _, p in identities) and len({p for _, p in identities}) == 20)
        fresh_ok = exact_twenty and all(x.get('pass') is True and x.get('source_verified') is True for x in fresh)
        cells[cell] = {'frozen_baseline_all_gates': baseline_ok,
                       'baseline_gates': historical.get('gates', {}),
                       'baseline_failed_or_missing': historical.get('failed_or_missing', ['NO_BASELINE_EVIDENCE']),
                       'fresh_process_count': len(fresh), 'fresh_20_unique_processes': exact_twenty,
                       'fresh_20_all_frozen_gates_pass': fresh_ok,
                       'fresh_failed_or_missing': [x.get('repeat') for x in fresh if x.get('pass') is not True or x.get('source_verified') is not True]}
    all_abc = all(cells[c]['frozen_baseline_all_gates'] for c in CELLS)
    two_diagnostics = len(unique_diagnostics) >= 2
    phases = {}
    for name in ('V10-D', 'FORMAL'):
        reg = registrations.get(name, {})
        registered = all_abc and (name != 'FORMAL' or two_diagnostics)
        authorized = scope.get(name) is True and reg.get('execute') is True
        phases[name] = {
            'registered_requirement_text': reg.get('selection_precondition', reg.get('precondition')),
            'registered_all_ABC_gates_demonstrated': all_abc,
            'registered_two_independent_diagnostics': two_diagnostics if name == 'FORMAL' else 'NOT_A_V10_D_REQUIREMENT',
            'registered_conditions_demonstrated': registered,
            'execution_authorized': authorized,
            'continuous_safety_required_by_this_no_contact_registration': False,
            'r13_repetition_or_perturbation_not_added_to_historical_registration': True,
            'may_start': bool(registered and authorized),
            'status': 'NOT_RUN_R13_SCOPE' if not authorized else ('ELIGIBLE_REVIEW_ONLY' if registered else 'BLOCKED_MISSING_REGISTERED_EVIDENCE')}
    return {
        'schema': 'TASK28_R13_PHASE_ADMISSION_REVIEW_V2', 'baseline_head': BASELINE,
        'source_binding': source_binding, 'cells': cells,
        'FORMAL_diagnostic_interpretation': {
            'exact_requirement': registrations.get('FORMAL', {}).get('precondition'),
            'archive_eligible_independent_process_ids': sorted(unique_diagnostics),
            'interpretation': 'The registration says two independent eligible diagnostics; it does not explicitly require identical feedback configuration or V10-D. A/B/C process identities count only when every frozen gate and source check passes.',
            'two_fresh_same_configuration_runs_are_separate_evidence': any(
                len({x.get('independent_process_id') for x in repetitions.get(c, []) if x.get('pass') is True and x.get('source_verified') is True and x.get('independent_process_id')}) >= 2 for c in CELLS)},
        **phases,
        'R13_review_findings_not_redefined_historical_gates': {
            'all_three_fresh_repetition_sets_pass': all(x['fresh_20_all_frozen_gates_pass'] for x in cells.values()),
            'robustness': robustness or {'status': 'NOT_ESTABLISHED_NO_VALIDATED_DYNAMIC_PERTURBATION_EVIDENCE'},
            'recommendation': 'Review fresh repetition and validated sensitivity evidence before requesting a later phase; this task never starts V10-D or FORMAL.'},
        'physical_approach': {'status': 'BLOCKED_R13_SCOPE', 'execution_authorized': False,
                              'continuous_safety': safety.get('status', 'CONTINUOUS_SAFETY_NOT_ESTABLISHED'),
                              'continuous_proof_required_before_approach': True},
        'next_phase_auto_start': False, 'physical_execution': False,
    }


def read_json(path):
    return json.loads(path.read_text())


def verify_source_binding():
    paths = sorted(list(BASE.glob('*.py')) + list(BASE.glob('*.json')))
    paths += [ROOT / 'offline/task28_p5_qp_6_r8' / name for name in ('controller.py', 'source_gate.py', 'effort_adapter.py')]
    rows = []
    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        baseline = subprocess.run(['git', 'show', f'{BASELINE}:{relative}'], cwd=ROOT, capture_output=True)
        current = path.read_bytes() if path.is_file() else b''
        rows.append({'path': relative, 'baseline_sha256': hashlib.sha256(baseline.stdout).hexdigest(),
                     'current_sha256': hashlib.sha256(current).hexdigest(),
                     'match': baseline.returncode == 0 and path.is_file() and baseline.stdout == current})
    return {'verified': bool(rows) and all(x['match'] for x in rows), 'files': rows,
            'scope': 'R12 top-level Python/JSON and frozen R8 controller/gate/adapter compared with baseline Git objects'}


def collect_cell(base, cell, evaluation, process, registration, contract, acceptance, source_ok):
    folder = base / 'evidence/diagnostic' / cell
    result = read_json(folder / 'result.json')
    frames = [json.loads(line) for line in (folder / 'frames.jsonl').read_text().splitlines() if line.strip()]
    audit = frozen_cell_review(evaluation, result, process, registration, contract, acceptance, source_ok, frames)
    lifecycle = read_json(base / 'evidence/process' / cell / 'lifecycle.json')
    audit['independent_process_id'] = (f"{lifecycle['start_utc']}:{lifecycle['child_pid']}"
                                       if lifecycle.get('start_utc') and lifecycle.get('child_pid') else None)
    return audit


def main():
    registrations = read_json(BASE / 'registrations.json')
    contract = read_json(BASE / 'source_contract.json')
    acceptance = read_json(BASE / 'preregistration.json')['acceptance']
    binding = verify_source_binding()
    evaluations = read_json(BASE / 'evidence/evaluation.json')['cells']
    processes = read_json(BASE / 'evidence/process_isolation_audit.json')
    historical = {c: collect_cell(BASE, c, evaluations[c], processes[c], registrations[c], contract,
                                 acceptance, binding['verified']) for c in CELLS}
    fresh = {c: [] for c in CELLS}
    from offline.task28_p5_qp_6_r12 import process_audit
    original_here = process_audit.HERE
    try:
        for path in sorted((HERE / 'evidence/isaac_repetitions').glob('*/receipt.json')):
            receipt = read_json(path)
            cell = receipt.get('cell')
            if cell not in CELLS:
                continue
            trial = path.parent
            process_audit.HERE = trial
            process = process_audit.audit(cell)
            hashes_ok = all((trial / name).is_file() and hashlib.sha256((trial / name).read_bytes()).hexdigest() == digest
                            for name, digest in receipt.get('output_hashes', {}).items())
            snapshot = trial / 'evidence/process' / cell / 'source_at_launch'
            frozen_paths = list(BASE.glob('*.py')) + list(BASE.glob('*.json'))
            snapshots_ok = all((trial / p.name).is_file() and (snapshot / p.name).is_file()
                               and p.read_bytes() == (trial / p.name).read_bytes() == (snapshot / p.name).read_bytes()
                               for p in frozen_paths)
            source_ok = (binding['verified'] and receipt.get('source_unchanged') is True
                         and bool(receipt.get('output_hashes')) and hashes_ok and snapshots_ok)
            audit = collect_cell(trial, cell, receipt.get('evaluation', {}), process, registrations[cell],
                                 contract, acceptance, source_ok)
            audit.update(repeat=receipt.get('repeat'), source_verified=source_ok)
            audit['pass'] &= receipt.get('completed') is True and receipt.get('returncode') == 0
            fresh[cell].append(audit)
    finally:
        process_audit.HERE = original_here
    safety_path = HERE / 'evidence/continuous_safety_audit.json'
    safety = read_json(safety_path) if safety_path.exists() else {}
    result = evaluate_admission(registrations, historical, binding, fresh, safety,
                                task_scope={'V10-D': False, 'FORMAL': False})
    output = HERE / 'evidence/phase_admission_review.json'
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + '\n')
    print(json.dumps({name: result[name] for name in ('V10-D', 'FORMAL')}, indent=2))


if __name__ == '__main__':
    main()
