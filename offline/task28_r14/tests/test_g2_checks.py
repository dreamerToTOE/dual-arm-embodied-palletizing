"""R14 limited G2 diagnostics must not be promoted into runtime acceptance."""
import copy
import gzip
import hashlib
import json

import pytest

from offline.task28_r14 import g2_checks as checks


def test_threshold_observation_never_authorizes_runtime():
    summary = {'frames': 210, 'plant_status': 'COMPLETE',
               'object_exceedance_frames': [], 'relative_exceedance_frames': []}
    observed = checks.observation(summary)
    assert observed['approximate_model_observation'] == 'NO_TRACKING_THRESHOLD_EXCEEDANCE_OBSERVED'
    assert observed['runtime_acceptance_claim'] is False
    assert observed['execution_authorized'] is False
    assert observed['robustness_status'] == 'NOT_ESTABLISHED'
    summary['object_exceedance_frames'] = [5, 6]
    assert checks.observation(summary)['object_exceedance_count'] == 2
    assert checks.observation(summary)['approximate_model_observation'] == 'TRACKING_THRESHOLD_EXCEEDED'


def test_truncated_episode_cannot_hide_failure():
    summary = {'frames': 10, 'plant_status': 'COMPLETE',
               'object_exceedance_frames': [], 'relative_exceedance_frames': []}
    assert checks.observation(summary)['approximate_model_observation'] == 'INCOMPLETE_OR_GATE_REJECTED'


def test_registered_small_plan_rejects_expansion_or_retuning():
    plan = checks.read(checks.MANIFEST)
    checks.validate_plan(plan)
    changed = copy.deepcopy(plan)
    changed['cases'].append('V10-A')
    with pytest.raises(ValueError, match='SMALL_PLAN_CHANGED'):
        checks.validate_plan(changed)
    changed = copy.deepcopy(plan)
    changed['perturbations']['V10-C']['q_initial_ulp_plus_rad'][0] *= 2
    with pytest.raises(ValueError, match='NOT_FROZEN_R13_VALUES'):
        checks.validate_plan(changed)
    changed = copy.deepcopy(plan)
    changed['acceptance']['object_peak_m'] = .001
    with pytest.raises(ValueError, match='THRESHOLDS_CHANGED'):
        checks.validate_plan(changed)


def test_changed_source_hash_rejected_before_replay():
    plan = checks.read(checks.MANIFEST)
    changed = copy.deepcopy(plan)
    first = next(iter(changed['sources_sha256']))
    changed['sources_sha256'][first] = '0' * 64
    with pytest.raises(ValueError, match='SOURCE_HASH_CHANGED'):
        checks.validate_plan(changed)


def test_archived_results_are_fresh_small_scope_with_verified_traces():
    result = checks.read(checks.RESULTS)
    assert result['preregistration_sha256'] == checks.replay.digest(checks.MANIFEST)
    assert result['episode_count'] == 8 and result['perturbed_episode_count'] == 6
    assert result['computed_qp_calls'] == 8 * 210
    assert {(row['case'], row['scenario']) for row in result['episodes']} == {
        (case, scenario) for case in checks.CASES for scenario in checks.SCENARIOS}
    for row in result['episodes']:
        path = checks.ROOT / row['trace']
        assert checks.replay.digest(path) == row['trace_sha256_gzip']
        data = gzip.decompress(path.read_bytes())
        assert hashlib.sha256(data).hexdigest() == row['trace_sha256_uncompressed']
        records = json.loads(data)
        assert len(records) == 210
        assert row['independent_state_reset'] and row['fresh_qp_and_controller_recomputation']
        assert row['runtime_acceptance_claim'] is False
        assert all(record['qp']['execution_authorized'] is False for record in records)
    for qualification in result['nominal_model_qualification'].values():
        assert qualification['status'] == 'MODEL_NOT_QUALIFIED_FOR_MARGIN'
        assert qualification['discrepancy_to_margin_ratio'] > 1
    assert result['continuous_approach_safety'] == 'CONTINUOUS_SAFETY_NOT_ESTABLISHED'
    assert not result['independent_g3_offline_research_blocked_by_new_perturbations']
