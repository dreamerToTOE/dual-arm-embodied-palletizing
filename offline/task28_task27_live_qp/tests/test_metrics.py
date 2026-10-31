import numpy as np
import pytest

from offline.task28_task27_live_qp.metrics import summarize_events


def _frame(step, left_x, right_x, *, status="QP_OPTIMAL"):
    left = np.eye(4).tolist()
    right = np.eye(4)
    right[0, 3] = right_x
    left[0][3] = left_x
    return {
        "event": "TORQUE_FRAME",
        "physics_step": step,
        "time_s": step / 60.0,
        "tcp_pose_world": {"left": left, "right": right.tolist()},
        "dq_rad_s": {"left": [0.1] * 7, "right": [0.2] * 7},
        "qp": {
            "status": status,
            "solve_wall_time_s": 0.002,
            "optimizer_iterations": 3,
            "common_tcp_error": [0.001, 0, 0, 0, 0, 0],
            "relative_tcp_error": [0, 0.002, 0, 0, 0, 0],
            "common_acceleration_residual_inf": 0.1,
            "relative_acceleration_residual_inf": 0.2,
            "torque_saturated_indices": [1] if step else [],
            "torque_rate_active_indices": [2] if step else [],
        },
    }


def test_summary_reports_common_relative_motion_and_status():
    events = [
        {"event": "TASK27_TORQUE_BRIDGE_ACTIVE"},
        {"event": "IMMEDIATE_READBACK"},
        _frame(0, 0.0, 0.10),
        _frame(1, 0.01, 0.11),
        {"event": "TASK27_TORQUE_BRIDGE_STOPPED", "torque_writes": 2,
         "position_arm_write_count": 0},
    ]
    summary = summarize_events(events)
    assert summary["sample_count"] == 2
    assert summary["qp_status_counts"] == {"QP_OPTIMAL": 2}
    assert summary["qp_optimal_count"] == 2
    assert summary["torque_write_count"] == 2
    assert summary["position_arm_write_count"] == 0
    assert summary["common_displacement_m"]["max"] == pytest.approx(0.01)
    assert summary["relative_displacement_m"]["max"] < 1e-12
    assert summary["common_error_translation_m"]["max"] == pytest.approx(0.001)
    assert summary["relative_error_translation_m"]["max"] == pytest.approx(0.002)
    assert summary["frames_with_torque_saturation"] == 1
    assert summary["frames_with_torque_rate_active"] == 1


def test_summary_keeps_fail_closed_and_handles_incomplete_log():
    summary = summarize_events([
        {"event": "FAIL_CLOSED", "error": "QP_INFEASIBLE"},
    ])
    assert summary["sample_count"] == 0
    assert summary["pose_sample_count"] == 0
    assert summary["fail_closed_count"] == 1
    assert summary["last_failure"] == "QP_INFEASIBLE"
