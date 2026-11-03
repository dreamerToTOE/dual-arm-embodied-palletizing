import json
from pathlib import Path

import numpy as np
import pytest

from offline.task28_stage_b_contact.d1_math import balance


def test_resting_cube_balances_gravity_and_contact():
    result = balance(.8, [0, 0, -9.81], 1/60, [0, 0, 0], [0, 0, 0],
                     [0, 0, .8*9.81/60], [0, 0, 0])
    assert np.allclose(result['unexplained_impulse_Ns'], [0, 0, 0], atol=1e-12)


def test_underreported_support_retains_positive_residual():
    result = balance(.8, [0, 0, -9.81], 1/60, [0, 0, 0], [0, 0, 0],
                     [0, 0, 1.2/60], [0, 0, 0])
    assert result['unexplained_impulse_Ns'][2] == pytest.approx((7.848-1.2)/60)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), 0.0, -1.0])
def test_bad_dt_is_rejected(value):
    with pytest.raises(ValueError):
        balance(.8, [0, 0, -9.81], value, [0]*3, [0]*3, [0]*3, [0]*3)


def test_recorded_c2r_discrepancy_is_red_capable():
    evidence = Path(__file__).parent / 'evidence/task28_c2r_support_v1'
    rows = [json.loads(line) for line in (evidence / 'timeseries.jsonl').read_text().splitlines()]
    residuals = []
    for previous, row in zip(rows, rows[1:]):
        residuals.append(balance(.8, [0, 0, -9.81], row['dt_s'], previous['cube_velocity_xyz_m_s'],
                                 row['cube_velocity_xyz_m_s'], row['cube_contact_impulse_world_Ns'],
                                 np.asarray(row['cube_table_friction_force_world_N']) * row['dt_s'])['unexplained_impulse_Ns'])
    assert np.sum(residuals, axis=0)[2] == pytest.approx(1.017519, abs=1e-5)


def test_minimal_run_awake_balance_and_sleeping_contact_semantics():
    evidence = Path(__file__).parent / 'evidence/task28_d1_support_v1'
    rows = [json.loads(line) for line in (evidence / 'timeseries.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (evidence / 'contact_events.jsonl').read_text().splitlines()]
    assert len(rows) == 240 and len(events) == 24
    awake = [row for row in rows if not row['sleeping']]
    sleeping = [row for row in rows if row['sleeping']]
    assert len(awake) == 24 and len(sleeping) == 216
    assert np.mean([row['matrix_force_world_N'][2] for row in awake]) == pytest.approx(7.848, abs=.02)
    assert abs(sum(row['unexplained_impulse_Ns'][2] for row in awake)) < 1e-5
    assert all(row['matrix_force_world_N'][2] == 0 for row in sleeping)
    assert all(set(event['colliders']) == {'/World/Cube_01', '/World/Table'} for event in events)
