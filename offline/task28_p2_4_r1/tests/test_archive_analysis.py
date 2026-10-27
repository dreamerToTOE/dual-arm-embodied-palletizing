import json
from pathlib import Path
import pytest

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'task28_p2_4_r1' / 'evidence'


def test_archive_has_causal_960_step_identity_and_timing():
    data = json.loads((EVIDENCE / 'motion_archive_diagnostics.json').read_text())
    assert data['sample_count'] == 960
    assert data['archive_observer_input_excludes_oracle'] is True
    assert data['timing']['monotonic_timestamps'] is True
    assert data['timing']['median_dt_s'] == pytest.approx(0.008333333767950535)
    assert data['timing']['effort_midpoint_to_state_s']['mean'] == pytest.approx(-1/240, abs=1e-12)
    assert data['continuity']['joint_order_verified'] is True


def test_all_timing_sign_variants_are_diagnostics_only():
    data = json.loads((EVIDENCE / 'motion_archive_diagnostics.json').read_text())
    expected = {
        'causal_current_model_current_effort',
        'causal_previous_model_current_effort',
        'centered_future_model_current_effort',
        'forward_future_model_current_effort',
        'current_model_previous_effort',
        'current_model_next_effort',
    }
    assert set(data['variants']) == expected
    assert data['sign_hypothesis']['zero_load_cannot_identify_sign'] is True
    assert data['sign_hypothesis']['no_sign_or_lag_selected_retrospectively'] is True
    assert data['rejection_or_stop_reason'] == 'DYNAMIC_SOURCE_SEMANTICS_UNRESOLVED'


def test_motion_residuals_are_segmented_and_not_called_calibration():
    data = json.loads((EVIDENCE / 'motion_archive_diagnostics.json').read_text())
    assert set(data['segments']) == {'rest_pre', 'accel', 'decel', 'stop'}
    assert data['variants']['causal_current_model_current_effort']['inf_norm']['max_abs'] > 0
    assert data['source_and_model_limits']['dynamic_semantics_validated'] is False


def test_source_evidence_rejects_actuator_torque_claim():
    data = json.loads((EVIDENCE / 'source_semantics.json').read_text())
    assert data['decision']['position_drive_actuator_torque_source_validated'] is False
    assert data['decision']['dynamic_source_gate'] == 'BLOCKED_FOR_PASS'
    assert 'projected' in data['source_findings'][0]['semantic_result']
    assert 'alternate mode' in json.loads((EVIDENCE / 'phase3_decision.json').read_text())['recommended_next_test']


def test_runtime_mode_probe_is_read_only_and_not_a_torque_calibration():
    data = json.loads((EVIDENCE / 'mode_probe.json').read_text())
    assert data['status'] == 'PROBE_COMPLETE'
    assert data['control_mode_changed'] is False
    assert data['effort_modes'] == [['force'] * 9]
    assert data['applied_effort_initial'] == [0.0] * 9
    assert data['read_semantics']['torque_validated'] is False
