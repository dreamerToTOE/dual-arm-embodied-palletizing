"""Executable counterexamples for closed-loop replay and its claim boundary."""
import json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p5_qp_6_r13 import closed_loop_replay as replay


@pytest.fixture(scope='module')
def manifest():
    return json.loads((replay.EVIDENCE / 'closed_loop_manifest.json').read_text())


def test_every_step_resolves_actual_qp_and_updates_plant(monkeypatch, manifest):
    solve = replay.solve_pose_aware_qp
    seen = []
    def observed(*args, **kwargs):
        seen.append(np.asarray(args[1]).copy())
        return solve(*args, **kwargs)
    monkeypatch.setattr(replay, 'solve_pose_aware_qp', observed)
    summary, records = replay.simulate_episode('V10-C', 'nominal', manifest, max_steps=4)
    assert summary['plant_status'] == 'COMPLETE'
    assert len(seen) == len(records) == 4
    for index in range(1, 4):
        np.testing.assert_array_equal(seen[index], records[index-1]['q'])
    assert not np.array_equal(seen[0], seen[-1])
    assert all(row['qp']['status'] == 'QP_OPTIMAL' for row in records)
    assert all(row['qp']['execution_authorized'] is False for row in records)


def test_initial_state_perturbation_changes_qp_and_torque(manifest):
    _, nominal = replay.simulate_episode('V10-C', 'nominal', manifest, max_steps=3)
    _, perturbed = replay.simulate_episode('V10-C', 'initial_ulp_plus', manifest, max_steps=3)
    delta = np.asarray(manifest['perturbations']['V10-C']['q_initial_ulp_plus_rad'])
    np.testing.assert_allclose(np.array(perturbed[0]['q_pre'])-nominal[0]['q_pre'], delta, atol=1e-15, rtol=0)
    assert np.max(np.abs(np.array(perturbed[0]['qdot'])-nominal[0]['qdot'])) > 1e-10
    assert np.max(np.abs(np.array(perturbed[0]['torque_Nm'])-nominal[0]['torque_Nm'])) > 1e-10
    assert not np.array_equal(perturbed[-1]['q'], nominal[-1]['q'])


def test_independent_reset_recomputes_identical_deterministic_trace(manifest):
    _, first = replay.simulate_episode('V10-B', 'nominal', manifest, max_steps=4)
    _, second = replay.simulate_episode('V10-B', 'nominal', manifest, max_steps=4)
    assert replay.encoded(first) == replay.encoded(second)
    assert first is not second
    assert all('solve_wall_time_s' not in row['qp'] and 'calculation_time_s' not in row['qp'] for row in first)


def test_wall_clock_difference_does_not_fake_numerical_variation():
    first = {'status': 'QP_OPTIMAL', 'solve_wall_time_s': .01,
             'calculation_time_s': .03, 'hard_relative_residual_inf': 1e-12}
    second = {**first, 'solve_wall_time_s': .02, 'calculation_time_s': .04}
    assert replay.numerical_diagnostics(first) == replay.numerical_diagnostics(second)
    changed_result = {**second, 'hard_relative_residual_inf': 1e-10}
    assert replay.numerical_diagnostics(first) != replay.numerical_diagnostics(changed_result)


def test_observed_mass_perturbation_is_spd_and_not_arbitrary_percent(manifest):
    for case in replay.CASES:
        for mass in manifest['perturbations'][case]['observed_mass_variation'].values():
            assert mass['minus_min_eigenvalue'] > 0
            assert mass['plus_min_eigenvalue'] > 0
            assert mass['relative_spectral_norm'] > 0
    rationale = manifest['perturbation_rationale']['mass_delta']
    assert 'NOT a calibrated bound' in rationale


def test_period_envelope_is_measured_and_inside_frozen_gate(manifest):
    for case in replay.CASES:
        error = manifest['perturbations'][case]['observed_timestamp_dt_error_s']
        assert 0 < error < 1e-8
        for signed in (-1, 1):
            assert abs(replay.DT + signed*error - replay.DT) <= 1e-8


def test_staleness_fault_contract_is_not_claimed_as_timestamp_proof():
    faults = replay.gate_faults()
    assert faults['declared_one_sample_stale']['status'] == 'REJECTED_BEFORE_COMMAND'
    assert faults['declared_one_sample_stale']['reason'] == 'STALE_OR_ALIASED_STATE'
    assert faults['coherent_one_sample_stale_marked_fresh']['status'] == 'ACCEPTED_BY_PAIR_GATE'
    assert faults['dt_just_outside_existing_gate']['reason'] == 'PHYSICS_DT'
    assert all(value['command_applied'] is False for value in faults.values())


def test_full_trace_discrepancy_invalidates_margin_claim():
    evidence = json.loads((replay.EVIDENCE / 'closed_loop_model_validation.json').read_text())
    assert evidence['robustness_qualification'] == 'NOT_ESTABLISHED'
    for case in evidence['cases'].values():
        assert case['status'] == 'MODEL_NOT_QUALIFIED_FOR_MARGIN'
        assert case['same_index_tcp_discrepancy_peak_m'] > case['v10c_remaining_margin_m']


def test_source_binding_unchanged(manifest):
    for name, expected in manifest['sources_sha256'].items():
        assert replay.digest(replay.ROOT / name) == expected
    assert manifest['controller'] == {'feedback_gain_scale': .75, 'tau_s': .10, 'changed': False}
    assert manifest['acceptance'] == {'object_peak_m': .00025, 'relative_translation_peak_m': .00015}
