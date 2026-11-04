import math
import pytest
from research.stage_b_contact_switch_g0.scripts.observer_math import aligned_observation, finite_vector, impulse_to_force, normal_component

def test_impulse_uses_physics_dt():
    assert impulse_to_force([0, 0, 0.981], 1/60)[2] == pytest.approx(58.86)
    assert impulse_to_force([0, 0, 0.981], 1.0)[2] == pytest.approx(.981)

def test_rejects_invalid_values_and_dt():
    with pytest.raises(ValueError): finite_vector([0, math.nan, 1])
    with pytest.raises(ValueError): impulse_to_force([0, 0, 1], 0)

def test_step_alignment_rejects_late_or_skipped_samples():
    assert aligned_observation(2, .033, 1/60, previous_step=1, previous_time=.016)
    assert not aligned_observation(4, .050, 1/60, previous_step=1, previous_time=.016)
    assert not aligned_observation(2, .010, 1/60, previous_step=1, previous_time=.016)

def test_normal_component_checks_direction_frame():
    assert normal_component([0, 2, 0], [0, 1, 0]) == pytest.approx(2)
    with pytest.raises(ValueError): normal_component([0, 1, 0], [0, 0, 2])
