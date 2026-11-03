import math
import re
from pathlib import Path

import pytest
import numpy as np
import json
import shutil
import sys

from offline.task28_stage_b_contact.c2_control import Controller, json_record


def test_record_serializes_contact_numpy_boolean_and_rejects_nonfinite():
    assert json.loads(json_record({'tool_pair': np.bool_(True)})) == {'tool_pair': True}
    with pytest.raises(ValueError):
        json_record({'force': np.float64(float('nan'))})


def test_geometry_fixture_matches_original_scene_constants():
    root = Path(__file__).resolve().parents[2]
    source = (root / 'isaac/scripts/task26_truck_box_scene.py').read_text()
    constants = {'CUBE_SIZE': .12, 'CUBE_MASS': .8, 'VERTICAL_DROP_Z': .08,
                 'LATERAL_STANDOFF_Y': .13, 'VERTICAL_SUPPORT_WIDTH': .028,
                 'LATERAL_SUPPORT_WIDTH': .018, 'MANIFOLD_Y': .138,
                 'MANIFOLD_X_SIZE': .07, 'MANIFOLD_Y_SIZE': .014,
                 'MANIFOLD_Z_SIZE': .07, 'ARRAY_X_OFFSET': .024,
                 'ARRAY_Z_OFFSET': .024, 'CUP_RADIUS': .01, 'CUP_LENGTH': .01, 'CUP_CENTER_Y': .15}
    for name, value in constants.items():
        match = re.search(rf'^{name}\s*=\s*([\d.]+)', source, re.MULTILINE)
        assert match and float(match.group(1)) == pytest.approx(value), name


def sample(step, force=0.0, **changes):
    o = {'step': step, 'sensor_step': step, 'time_s': step / 60, 'sensor_time_s': step / 60,
         'dt_s': 1 / 60, 'age_ns': 0, 'frame': 'world', 'source': 'physx_pair', 'valid': True,
         'force_y': force, 'force_norm': abs(force), 'event_force_y': force, 'tool_pair': force > 0,
         'unexpected_pairs': [], 'tool_y': 0.0, 'cube_y': 0.0, 'tool_velocity': 0.0,
         'cube_velocity': 0.0, 'penetration_m': 0.0, 'support_force_z': 7.85, 'support_event_z': 7.85,
         'tool_displacement': 0.0, 'cube_displacement': 0.0, 'tool_speed': 0.0, 'cube_speed': 0.0,
         'expected_weight_N': 7.848}
    o.update(changes)
    return o


def test_sequence_requires_persistent_contact_force_feedback_and_clear_exit():
    c = Controller()
    assert c.update(sample(1))['state'] == 'APPROACH'
    assert c.update(sample(2, 0.03))['state'] == 'CONTACT'
    assert c.update(sample(3, 0.03))['state'] == 'CONTACT'
    assert c.update(sample(4, 0.03))['state'] == 'FORCE_RAMP'
    outputs = [c.update(sample(i, 0.04)) for i in range(5, 114)]
    assert max(x['reference_N'] for x in outputs) == pytest.approx(0.15)
    assert outputs[-1]['state'] == 'EXIT'
    for i in range(114, 140):
        result = c.update(sample(i, 0.0, tool_y=-0.005))
        if result['state'] == 'DONE':
            break
    assert result['state'] == 'DONE'
    assert result['command_N'] == 0.0


@pytest.mark.parametrize('change,reason', [
    ({'valid': False}, 'stale_or_inconsistent_observation'),
    ({'sensor_step': 0}, 'stale_or_inconsistent_observation'),
    ({'sensor_time_s': 0}, 'stale_or_inconsistent_observation'),
    ({'age_ns': 50_000_001}, 'stale_or_inconsistent_observation'),
    ({'frame': 'map'}, 'stale_or_inconsistent_observation'),
    ({'force_y': math.nan}, 'nonfinite_observation'),
    ({'unexpected_pairs': ['tool/table']}, 'unexpected_contact_pair'),
    ({'support_force_z': 0.0}, 'cube_support_missing'),
    ({'support_event_z': 0.0}, 'support_force_disagreement'),
    ({'force_y': 0.3, 'force_norm': 0.3}, 'pair_force_disagreement'),
    ({'force_y': 1.1, 'force_norm': 1.1, 'event_force_y': 1.1}, 'excessive_force'),
    ({'penetration_m': 0.0021}, 'excessive_penetration'),
    ({'tool_displacement': 0.016}, 'motion_limit'),
    ({'tool_speed': 0.041}, 'motion_limit'),
    ({'force_y': -0.1, 'force_norm': 0.1, 'event_force_y': -0.1}, 'contact_identity_or_direction'),
    ({'force_y': 0.1, 'force_norm': 0.1, 'event_force_y': 0.1, 'tool_pair': False}, 'contact_identity_or_direction'),
])
def test_unsafe_observations_fail_closed(change, reason):
    result = Controller().update(sample(1, **change))
    assert result['state'] == 'FAILED'
    assert result['command_N'] == 0
    assert result['reason'] == reason


def test_missing_or_skipped_observations_fail_closed():
    c = Controller()
    o = sample(1)
    del o['force_y']
    assert c.update(o)['reason'] == 'missing_or_invalid_observation'
    c = Controller()
    assert c.update(sample(2))['reason'] == 'stale_or_inconsistent_observation'


def test_contact_loss_and_missing_contact_timeout():
    c = Controller()
    c.update(sample(1, 0.03))
    assert c.update(sample(2))['reason'] == 'contact_lost_during_confirmation'
    c = Controller()
    for i in range(1, 182):
        result = c.update(sample(i))
    assert result['reason'] == 'contact_timeout'


def test_support_reading_inconsistent_with_weight_fails_before_contact():
    c = Controller()
    for i in range(1, 14):
        result = c.update(sample(i, support_force_z=1.2, support_event_z=1.2))
    assert result['reason'] == 'support_force_weight_inconsistent'


def test_ramp_command_uses_measured_force_and_exit_times_out():
    low, high = Controller(), Controller()
    low.state = high.state = 'FORCE_RAMP'
    assert low.update(sample(1, .03))['command_N'] > high.update(sample(1, .10))['command_N']
    c = Controller()
    c.state, c.exit_y = 'EXIT', 0.0
    for i in range(1, 182):
        result = c.update(sample(i, .03))
    assert result['reason'] == 'exit_timeout'


def test_truncated_run_cannot_be_accepted(tmp_path, monkeypatch):
    from offline.task28_stage_b_contact import analyze_c2
    evidence = Path(__file__).parent / 'evidence/task28_c2_supported_v1'
    for name in ('timeseries.jsonl', 'summary.json'):
        shutil.copy(evidence / name, tmp_path / name)
    monkeypatch.setattr(sys, 'argv', ['analyze_c2.py', str(tmp_path)])
    analyze_c2.main()
    result = json.loads((tmp_path / 'analysis.json').read_text())
    assert result['status'] == 'B_C2_PARTIAL'
    assert not result['acceptance_checks']['complete_state_sequence']
    assert not result['acceptance_checks']['contact_events_available']
    assert not result['acceptance_checks']['gravity_and_all_contact_momentum_balance']
