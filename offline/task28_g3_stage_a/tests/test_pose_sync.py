import pytest
from offline.task28_task27_integration.analyze_pose_sync import join_evidence


def test_acceptance_is_joined_to_exact_message_not_later_sample():
    rows = [dict(header_ns=7, wall_time_ns=1000000, ros_usd_xyz_m=[1.096, .06, .26],
                 rigid_prim_xyz_m=[1.097, .06, .26], frame_id='world'),
            dict(header_ns=8, wall_time_ns=2000000, ros_usd_xyz_m=[1.099, .06, .26],
                 rigid_prim_xyz_m=[1.099, .06, .26], frame_id='world')]
    log = ('TASK28_B0_SEAT_SAMPLE header_ns=7 received_wall_ns=1200000 acceptance_wall_ns=1400000 '
           'revision=2 frame=world xyz=(1.096000000,0.060000000,0.260000000) gap_mm=4.000000\n[INFO] next')
    result = join_evidence(log, rows)
    assert result['executor_gap_mm'] == 4.
    assert result['rigid_geometry_gap_mm_at_publication'] == pytest.approx(3.)
    assert result['receive_to_accept_ms'] == pytest.approx(.2)
    with pytest.raises(ValueError, match='publication matches'):
        join_evidence(log, rows[1:])


def test_acceptance_join_rejects_clock_order_and_frame_mismatch():
    row = dict(header_ns=7, wall_time_ns=1000000, frame_id='world',
               ros_usd_xyz_m=[1.096, .06, .26], rigid_prim_xyz_m=[1.097, .06, .26])
    log = ('TASK28_B0_SEAT_SAMPLE header_ns=7 received_wall_ns=1200000 acceptance_wall_ns=1400000 '
           'revision=2 frame=world xyz=(1.096000000,0.060000000,0.260000000) gap_mm=4.000000')
    with pytest.raises(ValueError, match='timestamps are out of order'):
        join_evidence(log.replace('received_wall_ns=1200000', 'received_wall_ns=900000'), [row])
    with pytest.raises(ValueError, match='timestamps are out of order'):
        join_evidence(log.replace('acceptance_wall_ns=1400000', 'acceptance_wall_ns=1100000'), [row])
    with pytest.raises(ValueError, match='frame does not match'):
        join_evidence(log.replace('frame=world', 'frame=map'), [row])
