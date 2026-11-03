import copy
import json
from pathlib import Path

import pytest

from offline.task28_stage_b_contact.analyze_force_probe import analyze


@pytest.fixture
def trace():
    root = Path(__file__).parent / 'evidence/task28_force_observability_v4'
    def read(name):
        return [json.loads(line) for line in (root / name).read_text().splitlines()]
    return read('timeseries.jsonl'), read('contact_events.jsonl')


def test_recorded_probe_passes(trace):
    rows, events = trace
    assert analyze(rows, events)['status'] == 'PASS'


def test_contact_loss_during_load_is_not_a_pass(trace):
    rows, events = copy.deepcopy(trace)
    next(row for row in rows if row['phase'] == 'LOAD')['tool_cube_force_world_N'] = [0, 0, 0]
    assert not analyze(rows, events)['checks']['load_positive_y_every_step']


def test_opposite_force_is_not_a_pass(trace):
    rows, events = copy.deepcopy(trace)
    next(row for row in rows if row['phase'] == 'LOAD')['tool_cube_force_world_N'][1] *= -1
    assert analyze(rows, events)['status'] == 'PARTIAL'


def test_unrelated_cube_contact_is_not_silently_accepted(trace):
    rows, events = copy.deepcopy(trace)
    events[0]['colliders'][0] = '/World/Table'
    assert not analyze(rows, events)['checks']['no_other_cube_contact_events']


def test_out_of_order_observation_is_rejected(trace):
    rows, events = copy.deepcopy(trace)
    rows[1]['step'] = rows[0]['step']
    with pytest.raises(ValueError, match='out-of-order'):
        analyze(rows, events)


def test_report_after_its_claimed_step_is_not_a_pass(trace):
    rows, events = copy.deepcopy(trace)
    events[0]['wall_ns'] = rows[events[0]['step_at_callback']]['wall_ns'] + 1
    assert not analyze(rows, events)['checks']['pair_report_and_step_timing']


def test_inconsistent_cube_momentum_is_not_a_pass(trace):
    rows, events = copy.deepcopy(trace)
    last = max(i for i, row in enumerate(rows) if row['contact'])
    rows[last]['cube_velocity_m_s'][1] += 0.01
    assert not analyze(rows, events)['checks']['cube_momentum_matches_pair_impulse']
