import json
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]

def test_frozen_controller_and_repetition_contract():
    p=json.loads((HERE/'preregistration.json').read_text())
    assert p['created_from_head']=='52d07e6a9c4ac482db4887483cbf8ab0cbf510d1'
    assert p['controller_frozen']['feedback_gain_scale']==0.75
    assert p['controller_frozen']['velocity_capture_tau_s']==0.1
    assert p['repetition_plan']['deterministic_repetitions_per_cell']==20
    assert p['physical_execution'] is False

def test_deterministic_replays_pass_and_are_identical():
    d=json.loads((HERE/'evidence/robustness_replay.json').read_text())
    for cell in ('V10-A','V10-B','V10-C'):
        x=d['cells'][cell]['deterministic_repetition']
        assert x['repetitions']==20
        assert x['identical_record_fingerprint'] is True
        assert x['peak_span_mm']==0.0
        assert x['all_acceptance_pass'] is True
        assert x['solver_status']['all_qp_optimal'] is True

def test_independent_cpu_process_replays_pass():
    d=json.loads((HERE/'evidence/independent_replay.json').read_text())
    for cell in ('V10-A','V10-B','V10-C'):
        x=d['cells'][cell]
        assert x['independent_processes']==20
        assert x['all_exit_zero'] is True
        assert x['identical_record_fingerprint'] is True
        assert x['all_acceptance_pass'] is True

def test_margin_and_perturbation_are_not_promoted_to_acceptance():
    d=json.loads((HERE/'evidence/robustness_replay.json').read_text())
    assert d['cells']['V10-C']['r12_acceptance_margin_mm'] > 0.0023
    assert d['cells']['V10-C']['combined_extremes']['all_acceptance_pass'] is False
    assert d['cells']['V10-C']['combined_extremes']['worst_case']['object_over_limit_count'] > 0
    assert all(x['solver_status'].startswith('NOT_RE_SOLVED') for x in d['cells']['V10-C']['perturbations'].values())

def test_continuous_safety_and_phase_gate_remain_blocked():
    s=json.loads((HERE/'evidence/continuous_safety_audit.json').read_text())
    a=json.loads((HERE/'evidence/phase_admission_review.json').read_text())
    assert s['status']=='CONTINUOUS_SAFETY_NOT_ESTABLISHED'
    assert s['execution']['cube_commands']==0
    assert a['V10-D']['status'].startswith('BLOCKED_')
    assert a['FORMAL']['status'].startswith('BLOCKED_')
    assert a['physical_execution'] is False
