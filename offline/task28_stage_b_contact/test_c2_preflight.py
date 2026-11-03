import numpy as np

from offline.task28_stage_b_contact.c2_preflight import SupportPreflight


def observation(step, support=7.848, **extra):
    row = {'step': step, 'sensor_step': step, 'time_s': step / 60, 'dt_s': 1 / 60,
           'support_force_z': support, 'support_event_z': support, 'force_norm': 0., 'age_ns': 0,
           'valid': True, 'frame': 'world', 'unexpected_pairs': []}
    row.update(extra)
    return row


def test_stationary_balanced_support_passes_only_after_30_steps():
    gate = SupportPreflight(.8, [0, 0, -9.81])
    for step in range(1, 31):
        result = gate.update(observation(step), [0, 0, 0], [0, 0, 7.848 / 60])
        assert result['status'] == ('PASS' if step == 30 else 'PENDING')


def test_old_low_support_pattern_stops_before_motion():
    gate = SupportPreflight(.8, [0, 0, -9.81])
    for step in range(1, 13):
        result = gate.update(observation(step, support=1.177), [0, 0, 0], [0, 0, 1.177 / 60])
    assert result['reason'] == 'PREFLIGHT_SUPPORT_OR_MOMENTUM_MISMATCH'
    assert result['status'] == 'FAILED'


def test_momentum_mismatch_fails_even_if_support_force_matches_weight():
    gate = SupportPreflight(.8, [0, 0, -9.81])
    for step in range(1, 13):
        result = gate.update(observation(step), [0, 0, 0], [0, 0, 0])
    assert result['reason'] == 'PREFLIGHT_SUPPORT_OR_MOMENTUM_MISMATCH'


def test_missing_stale_and_nonfinite_observations_fail_closed():
    assert SupportPreflight(.8, [0, 0, -9.81]).update(observation(1, age_ns=50_000_001), [0]*3, [0]*3)['status'] == 'FAILED'
    assert SupportPreflight(.8, [0, 0, -9.81]).update(observation(1), [np.nan, 0, 0], [0]*3)['status'] == 'FAILED'
    row = observation(1); del row['support_force_z']
    assert SupportPreflight(.8, [0, 0, -9.81]).update(row, [0]*3, [0]*3)['status'] == 'FAILED'
