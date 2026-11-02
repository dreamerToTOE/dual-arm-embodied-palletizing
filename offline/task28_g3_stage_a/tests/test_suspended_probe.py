import pytest
from offline.task28_g3_stage_a.suspended_probe import motion_progress, MOTION_START_S, MOVE_S, HOLD_S, END_S


def test_motion_profile_reaches_endpoint_and_returns_with_zero_velocity():
    assert motion_progress(0) == (0, 0, 0)
    assert motion_progress(MOTION_START_S) == (0, 0, 0)
    assert motion_progress(MOTION_START_S + MOVE_S) == (1, 0, 0)
    assert motion_progress(MOTION_START_S + MOVE_S + HOLD_S) == (1, 0, 0)
    assert motion_progress(END_S) == (0, 0, 0)
    assert motion_progress(MOTION_START_S + MOVE_S / 2)[0] == pytest.approx(0.5)
    assert motion_progress(MOTION_START_S + MOVE_S + HOLD_S + MOVE_S / 2)[0] == pytest.approx(0.5)
