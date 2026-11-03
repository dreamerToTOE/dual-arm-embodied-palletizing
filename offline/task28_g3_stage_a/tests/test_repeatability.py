import numpy as np

from offline.task28_g3_stage_a.analyze_repeatability import phase_statistics, process_status


def test_phase_rms_and_terminal_are_derived_from_simulation_reference():
    heights = [.26, .30, .54, .54, .50, .28, .26, .26]
    rows = [dict(time_s=float(i), desired_xyz_m=[.77, .06, z], error_mm=float(i + 1),
                 terminal_active=i >= 6) for i, z in enumerate(heights)]
    phases, bounds = phase_statistics(rows)
    assert bounds == {'lift_end': 2, 'transport_end': 4, 'terminal_start': 6}
    assert np.isclose(phases['lift']['rms_mm'], np.sqrt(2.5))
    assert phases['terminal_placement']['peak_mm'] == 8.
    assert phases['terminal_placement']['peak_time_s'] == 7.
    assert phase_statistics([]) == ({}, None)


def test_shutdown_child_status_is_not_the_launch_status():
    log = '[ERROR] [move_group-3]: process has died [pid 12, exit code -11, cmd abc]'
    assert process_status(log, 'move_group') == {'status': 'exited', 'code': -11}
    assert process_status('', 'move_group') == {'status': 'not_observed', 'code': None}
