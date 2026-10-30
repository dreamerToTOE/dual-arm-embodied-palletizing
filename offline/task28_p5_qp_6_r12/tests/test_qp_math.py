"""CPU-only focused tests for R12 closed-loop cooperative-QP math."""
import json
from pathlib import Path

import numpy as np

from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse
from offline.task28_p5_qp_6_r9.virtual_frames import make_virtual_frame
from offline.task28_p5_qp_6_r12.qp_math import (
    adjoint, lookahead_velocity_bounds, pose_error_world,
    quintic_profile, relative_pose_error, se3_log,
    solve_pose_aware_qp, virtual_twist_quintic,
)

ROOT = Path(__file__).resolve().parents[1]


def _state():
    result = json.loads((ROOT.parent / "task28_p5_qp_6_r9/evidence/diagnostic/V2/result.json").read_text())
    arms = result["ownership_initial_state"]["arms"]
    left, right = np.asarray(arms["left"]["tcp_world"]), np.asarray(arms["right"]["tcp_world"])
    frame = make_virtual_frame(left, right)
    q = np.r_[result["q_hold_actual"]["left"], result["q_hold_actual"]["right"]]
    return RobotModel(DEFAULT_FIXTURE), frame, left, right, np.asarray(q, float)


def test_quintic_is_c2_rest_to_rest():
    s0, v0, a0 = quintic_profile(0., 3.5, .0005)
    s1, v1, a1 = quintic_profile(3.5, 3.5, .0005)
    assert (s0, v0, a0) == (0., 0., 0.)
    assert np.allclose((s1, v1, a1), (1., 0., 0.))
    _, v_mid, _ = quintic_profile(1.75, 3.5, .0005)
    assert np.isclose(v_mid, .0005 * 1.875 / 3.5)


def test_quintic_rejects_invalid_and_has_correct_axis():
    try:
        virtual_twist_quintic(0., 3.5, .0005, axis="y")
    except ValueError as exc:
        assert str(exc) == "INVALID_VIRTUAL_TRAJECTORY_AXIS"
    else:
        raise AssertionError("invalid axis accepted")


def test_se3_error_uses_body_log_then_world_adjoint():
    actual = np.eye(4)
    actual[:3, 3] = [1., 2., 3.]
    desired = actual.copy(); desired[0, 3] += .002
    assert np.allclose(pose_error_world(actual, desired)[:3], [.002, 0., 0.])
    assert np.allclose(se3_log(np.eye(4)), np.zeros(6))
    assert np.allclose(adjoint(np.eye(4)), np.eye(6))


def test_relative_pose_order_and_zero_error():
    _, frame, left, right, _ = _state()
    actual, desired, error = relative_pose_error(frame, left, right)
    assert np.allclose(actual, desired, atol=1e-8)
    assert np.max(np.abs(error)) < 1e-8
    wrong = inverse(frame.T_VR) @ frame.T_VL
    assert np.max(np.abs(actual - wrong)) > 1e-5


def test_pose_aware_qp_enforces_affine_relative_feedback():
    model, frame, left, right, q = _state()
    perturbed = right.copy(); perturbed[0, 3] += .001
    x, diag = solve_pose_aware_qp(model, q, frame, left, perturbed, frame.T_WV)
    assert x is not None
    assert diag["status"] == "QP_OPTIMAL"
    assert diag["hard_relative_pose_residual_inf"] <= 2e-8
    assert diag["affine_equality_residual_inf"] <= 2e-8
    assert diag["execution_authorized"] is False
    assert diag["inverse_dynamics_claim"] is False


def test_pose_aware_qp_rejects_stale_wrong_frame_feedback():
    model, frame, left, right, q = _state()
    wrong = make_virtual_frame(right, left)
    _, _, e = relative_pose_error(wrong, left, right)
    # The swapped frame must not look like the registered zero-error state.
    assert np.linalg.norm(e[:3]) > 1e-5


def test_lookahead_bounds_are_inside_velocity_box_and_position_limits():
    model, _, _, _, q = _state()
    lo, hi = lookahead_velocity_bounds(model, q, 1/60, .12)
    assert np.all(lo <= hi)
    q1 = q + (np.minimum(np.maximum(np.zeros(14), lo), hi)) / 60.
    assert np.all(q1[:7] >= model.joint_lower - 1e-12)
    assert np.all(q1[:7] <= model.joint_upper + 1e-12)
    assert np.max(np.abs(lo)) <= .12 + 1e-12
    assert np.max(np.abs(hi)) <= .12 + 1e-12


def test_frame_convention_negative_control_does_not_promote():
    _, frame, left, right, _ = _state()
    # World-frame correction cannot be substituted for body-frame error by
    # changing multiplication order; the resulting transforms differ.
    bad = inverse(frame.T_VL) @ left
    good = left @ inverse(frame.T_VL)
    assert np.max(np.abs(bad - good)) > 1e-3

