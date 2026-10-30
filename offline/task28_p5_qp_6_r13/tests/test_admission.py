"""Negative controls for phase evidence and the R13 no-start boundary."""
from copy import deepcopy
import json

import pytest

from offline.task28_p5_qp_6_r13.acceptance_review import (
    BASE, CELLS, evaluate_admission, frozen_cell_review, read_json,
)


def baseline_cells():
    return {c: {'pass': True, 'gates': {'all_frozen': True}, 'failed_or_missing': [],
                'independent_process_id': 'independent-' + c} for c in CELLS}


def registrations():
    return read_json(BASE / 'registrations.json')


def test_registered_conditions_do_not_invent_continuous_or_robustness_gate():
    r = evaluate_admission(registrations(), baseline_cells(), {'verified': True})
    for name in ('V10-D', 'FORMAL'):
        assert r[name]['registered_conditions_demonstrated'] is True
        assert r[name]['continuous_safety_required_by_this_no_contact_registration'] is False
        assert r[name]['may_start'] is False
        assert r[name]['status'] == 'NOT_RUN_R13_SCOPE'
    assert r['physical_approach']['continuous_proof_required_before_approach'] is True
    assert r['physical_approach']['execution_authorized'] is False


def test_source_mismatch_rejects_historical_eligibility_even_when_numbers_pass():
    r = evaluate_admission(registrations(), baseline_cells(), {'verified': False})
    assert all(not x['frozen_baseline_all_gates'] for x in r['cells'].values())
    assert r['V10-D']['registered_conditions_demonstrated'] is False
    assert r['FORMAL']['registered_conditions_demonstrated'] is False


def test_formal_two_diagnostics_requires_distinct_evidence_identity():
    b = baseline_cells()
    for item in b.values():
        item['independent_process_id'] = 'same-process'
    r = evaluate_admission(registrations(), b, {'verified': True})
    assert r['V10-D']['registered_conditions_demonstrated'] is True
    assert r['FORMAL']['registered_two_independent_diagnostics'] is False


def test_registration_flag_does_not_supply_missing_task_authorization():
    reg = registrations()
    reg['V10-D']['execute'] = reg['FORMAL']['execute'] = True
    r = evaluate_admission(reg, baseline_cells(), {'verified': True})
    assert r['V10-D']['may_start'] is False
    assert r['FORMAL']['may_start'] is False


def test_repeated_archive_identity_is_not_twenty_fresh_runs():
    rows = {c: [{'repeat': i, 'independent_process_id': 'same-archive', 'pass': True,
                 'source_verified': True} for i in range(1, 21)] for c in CELLS}
    r = evaluate_admission(registrations(), baseline_cells(), {'verified': True}, rows)
    assert all(not c['fresh_20_unique_processes'] for c in r['cells'].values())


def test_unique_fresh_runs_still_require_every_frozen_gate_and_source():
    rows = {c: [{'repeat': i, 'independent_process_id': f'{c}-{i}', 'pass': True,
                 'source_verified': True} for i in range(1, 21)] for c in CELLS}
    rows['V10-C'][19]['source_verified'] = False
    r = evaluate_admission(registrations(), baseline_cells(), {'verified': True}, rows)
    assert r['cells']['V10-A']['fresh_20_all_frozen_gates_pass'] is True
    assert r['cells']['V10-C']['fresh_20_all_frozen_gates_pass'] is False
    assert r['cells']['V10-C']['fresh_failed_or_missing'] == [20]


@pytest.fixture
def frozen_inputs():
    contract = read_json(BASE / 'source_contract.json')
    acceptance = read_json(BASE / 'preregistration.json')['acceptance']
    registration = {'steps': 2}
    accuracy_names = ('virtual_object_tracking_peak_m', 'relative_tcp_translation_peak_m',
                      'relative_tcp_rotation_peak_rad', 'joint_tracking_peak_rad', 'native_dq_peak_rad_s')
    evaluation = {'accuracy': {k: {'value': 0., 'limit': acceptance[k], 'pass': True} for k in accuracy_names},
                  'eligible_for_v10d': True, 'no_contact_status': 'PASS', 'unexpected_collision': None,
                  'cube_displacement_m': 0., 'suction_commands': 0, 'D6_commands': 0,
                  'qdot_peak_rad_s': 0.}
    result = {'status': 'NO_CONTACT_R12_VIRTUAL_COOPERATION_COMPLETE', 'first_failure': None,
              'actual_effort_commands': 2, 'level1_source': 'PASS',
              'level2_source': 'R12_RUNTIME_CANDIDATE_PENDING_INDEPENDENT_EVALUATION',
              'source_asset_sha256': contract['fr3_asset_sha256'], 'registration': registration,
              'feedback_gain_scale': .75, 'velocity_capture': {'tau_s': .1}, 'cube_contact_commands': 0,
              'suction_commands': 0, 'virtual_execution': {'D6_commands': 0}, 'ros_graph_suppressed': True,
              'callback_dts_s': [1/60, 1/60], 'configured_dt_s': 1/60,
              'command_sequence': [{'effort_arm': {'left': [0.]*7, 'right': [0.]*7}} for _ in range(2)]}
    process = {'verified': True, 'normal_exit': True, 'exit_code': 0}
    raw = BASE / 'evidence/diagnostic/V10-A'
    result['ownership_initial_state'] = read_json(raw / 'result.json')['ownership_initial_state']
    # A recorded valid state supplies the 9-DOF/M/FK/drive identity contract;
    # unit tests alter one gate at a time rather than fabricating that schema.
    frame = next(json.loads(line) for line in (raw / 'frames.jsonl').read_text().splitlines()
                 if 'r12_qp' in json.loads(line))
    frame['r12_qp']['diagnostics'] = {'status': 'QP_OPTIMAL', 'hard_relative_residual_inf': 0.}
    frame['cube_world'] = deepcopy(result['ownership_initial_state']['cube_world'])
    return [evaluation, result, process, registration, contract, acceptance, True, [frame, deepcopy(frame)]]


def test_all_frozen_gate_inputs_have_positive_control(frozen_inputs):
    assert frozen_cell_review(*frozen_inputs)['pass'] is True


@pytest.mark.parametrize('change, gate', [
    ('relative_rotation', 'relative_tcp_rotation_peak_rad'),
    ('period', 'timing'),
    ('joint_effort', 'per_joint_effort_and_rate'),
    ('joint_rate', 'per_joint_effort_and_rate'),
    ('source', 'source_and_frozen_controller'),
    ('process', 'independent_process'),
    ('ros', 'no_contact_cube_suction_d6_ros'),
    ('collision', 'all_sample_qp_fcl_joint_and_sink'),
    ('qp_residual', 'all_sample_qp_fcl_joint_and_sink'),
    ('cube_transient', 'all_sample_cube_immobile'),
    ('state_source', 'all_sample_frozen_R8_state_gate'),
    ('first_command_rate', 'per_joint_effort_and_rate'),
])
def test_two_tracking_numbers_cannot_override_other_frozen_gates(frozen_inputs, change, gate):
    evaluation, result, process, _, _, _, _, frames = frozen_inputs
    if change == 'relative_rotation':
        evaluation['accuracy']['relative_tcp_rotation_peak_rad']['value'] = .002
    elif change == 'period':
        result['callback_dts_s'][1] = 1/60 + 1e-4
    elif change == 'joint_effort':
        result['command_sequence'][1]['effort_arm']['left'][2] = 9.  # < global 35 Nm, > joint-3 8 Nm.
    elif change == 'joint_rate':
        result['command_sequence'][0]['effort_arm']['left'][6] = -4.
        result['command_sequence'][1]['effort_arm']['left'][6] = 4.
    elif change == 'source':
        result['source_asset_sha256'] = 'different-asset'
    elif change == 'process':
        process['normal_exit'] = False
    elif change == 'ros':
        result['ros_graph_suppressed'] = False
    elif change == 'collision':
        frames[1]['collision']['valid'] = False
    elif change == 'qp_residual':
        frames[1]['r12_qp']['diagnostics']['hard_relative_residual_inf'] = 3e-7
    elif change == 'cube_transient':
        frames[0]['cube_world'][0][3] += 2e-8
    elif change == 'state_source':
        frames[1]['arms']['left']['fresh_copy'] = False
    elif change == 'first_command_rate':
        result['ownership_initial_state']['arms']['left']['applied_command_not_sensor'][6] = 4.01
    audited = frozen_cell_review(*frozen_inputs)
    assert audited['pass'] is False
    assert gate in audited['failed_or_missing']
