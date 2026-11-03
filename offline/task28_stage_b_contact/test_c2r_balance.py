import numpy as np

from offline.task28_stage_b_contact.c2r_balance import calculate


def test_all_contact_impulses_and_gravity_in_three_axes():
    base = {'step': 1, 'time_s': .1, 'dt_s': .1, 'cube_velocity_xyz_m_s': [0, 0, 0],
            'cube_tool_impulse_world_Ns': [0, 0, 0], 'cube_table_impulse_world_Ns': [0, 0, 0],
            'cube_contact_impulse_world_Ns': [0, 0, 0]}
    current = dict(base, step=2, time_s=.2, cube_velocity_xyz_m_s=[.1, -.2, 0],
                   cube_tool_impulse_world_Ns=[.08, 0, 0], cube_table_impulse_world_Ns=[0, 0, .7848],
                   cube_contact_impulse_world_Ns=[.08, -.16, .7848])
    result = calculate([base, current], .8, [0, 0, -9.81])[0]
    assert np.allclose(result['other_contact_impulse_Ns'], [0, -.16, 0])
    assert np.allclose(result['unexplained_residual_Ns'], [0, 0, 0])
    assert result['other_external_impulse_Ns'] is None


def test_friction_is_reported_separately_and_never_invented():
    base = {'step': 1, 'time_s': .1, 'dt_s': .1, 'cube_velocity_xyz_m_s': [0, 0, 0],
            'cube_tool_impulse_world_Ns': [0, 0, 0], 'cube_table_impulse_world_Ns': [0, 0, 0],
            'cube_contact_impulse_world_Ns': [0, 0, 0]}
    current = dict(base, step=2, time_s=.2, cube_velocity_xyz_m_s=[.1, 0, 0],
                   cube_table_impulse_world_Ns=[0, 0, .7848], cube_contact_impulse_world_Ns=[0, 0, .7848],
                   cube_table_friction_force_world_N=[.8, 0, 0])
    result = calculate([base, current], .8, [0, 0, -9.81])[0]
    assert np.allclose(result['residual_after_table_friction_Ns'], [0, 0, 0])
    del current['cube_table_friction_force_world_N']
    assert calculate([base, current], .8, [0, 0, -9.81])[0]['table_friction_impulse_Ns'] is None
