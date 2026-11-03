import copy
import json
from pathlib import Path

import pytest

from offline.task28_stage_b_contact.d2_analysis import compare


def sample(step, force=7.848, sleeping=False):
    return {'step': step, 'cube_xyz_m': [.5, 0, .26], 'cube_velocity_m_s': [0, 0, 0],
            'tool_xyz_m': [.5, -.218, .34], 'support_force_N': [0, 0, force],
            'late_support_force_N': [0, 0, force], 'support_impulse_Ns': [0, 0, force/60],
            'event_table_impulse_Ns': [0, 0, force/60], 'residual_Ns': [0, 0, 0], 'sleeping': sleeping,
            'pre_monotonic_ns': 1, 'event_monotonic_ns': [2], 'early_monotonic_ns': 3, 'late_monotonic_ns': 4}


def test_comparison_separates_awake_from_sleeping():
    a = [sample(1), sample(2, 0, True)]
    result = compare(a, copy.deepcopy(a))
    assert result['initial_states_matched']
    assert result['A']['awake_force_mean_N'] == pytest.approx(7.848)
    assert result['A']['sleeping_samples'] == 1


def test_unmatched_and_incomplete_are_not_silent():
    a, b = [sample(1), sample(2)], [sample(1), sample(2)]
    b[0]['cube_xyz_m'][0] += .01
    assert not compare(a, b)['initial_states_matched']
    with pytest.raises(ValueError):
        compare(a, b[:1])
    b[1]['step'] = 7
    with pytest.raises(ValueError):
        compare(a, b)


def test_callback_and_event_disagreement_detected():
    a, b = [sample(1)], [sample(1)]
    b[0]['late_support_force_N'][2] = 2
    b[0]['event_table_impulse_Ns'][2] = 0
    result = compare(a, b)['B']
    assert result['max_early_late_force_difference_N'] > 5
    assert result['max_event_matrix_impulse_difference_Ns'] > .1


def test_recorded_ab_demonstrates_view_count_does_not_remove_decay():
    root = Path(__file__).parent / 'evidence/task28_d2_view_compare_v1'
    read = lambda name: [json.loads(line) for line in (root / name / 'timeseries.jsonl').read_text().splitlines()]
    a, b = read('A'), read('B')
    result = compare(a, b)
    assert result['initial_states_matched']
    assert all(abs(x['support_force_N'][2] - y['support_force_N'][2]) < 1e-6 for x, y in zip(a, b))
    assert result['A']['awake_force_first_N'] > 7.8 and result['A']['awake_force_last_N'] < 1.3
    assert result['A']['callback_order_violations'] == result['B']['callback_order_violations'] == 0
