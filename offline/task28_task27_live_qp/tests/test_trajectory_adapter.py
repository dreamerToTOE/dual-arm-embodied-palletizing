import numpy as np

from offline.task28_task27_live_qp.trajectory_adapter import Task27JointTrajectoryAdapter


def test_timestamped_stream_is_c1_interpolated_and_holds_final_position():
    adapter = Task27JointTrajectoryAdapter()
    adapter.push(10.0, np.zeros(7))
    adapter.push(11.0, np.ones(7))
    mid = adapter.sample(10.5)
    assert mid is not None
    np.testing.assert_allclose(mid.q_rad, np.full(7, 0.5), atol=1e-12)
    np.testing.assert_allclose(mid.dq_rad_s, np.ones(7), atol=1e-12)
    final = adapter.sample(20.0)
    np.testing.assert_allclose(final.q_rad, np.ones(7), atol=1e-12)
    np.testing.assert_allclose(final.dq_rad_s, np.zeros(7), atol=1e-12)
    assert final.clamped


def test_repeated_stamp_replaces_and_nonmonotonic_is_rejected():
    adapter = Task27JointTrajectoryAdapter()
    adapter.push(1.0, np.zeros(7))
    adapter.push(1.0, np.ones(7))
    assert adapter.point_count == 1
    np.testing.assert_allclose(adapter.sample(1.0).q_rad, np.ones(7))
    adapter.push(2.0, np.full(7, 2.0))
    try:
        adapter.push(1.5, np.full(7, 1.5))
    except ValueError as exc:
        assert str(exc) == "NONMONOTONIC_TRAJECTORY_TIMESTAMP"
    else:
        raise AssertionError("non-monotonic timestamp was accepted")


def test_unilateral_stream_is_independent():
    left = Task27JointTrajectoryAdapter()
    right = Task27JointTrajectoryAdapter()
    left.push(0.0, np.zeros(7))
    assert left.sample(0.1) is not None
    assert right.sample(0.1) is None


def test_explicit_acceleration_is_retained_during_interpolation_and_terminal_hold_is_zero():
    """A richer source may provide ddq; the adapter must expose it to the QP.

    The current Task27 publisher supplies positions only, so this is a
    contract test for the source-adapter extension.  Endpoint acceleration
    values are blended for the interior diagnostic sample, while the terminal
    sample always holds zero velocity and acceleration for settling.
    """
    adapter = Task27JointTrajectoryAdapter()
    adapter.push(
        10.0,
        np.zeros(7),
        dq_rad_s=np.full(7, 0.2),
        ddq_rad_s2=np.full(7, 1.5),
    )
    adapter.push(
        11.0,
        np.ones(7),
        dq_rad_s=np.full(7, 0.4),
        ddq_rad_s2=np.full(7, -0.5),
    )
    mid = adapter.sample(10.5)
    assert mid is not None
    np.testing.assert_allclose(mid.ddq_rad_s2, np.full(7, 0.5), atol=1e-12)
    final = adapter.sample(11.0)
    assert final is not None and final.clamped
    np.testing.assert_allclose(final.dq_rad_s, np.zeros(7), atol=1e-12)
    np.testing.assert_allclose(final.ddq_rad_s2, np.zeros(7), atol=1e-12)
