"""R12 focused, CPU-only tests; no Isaac, ROS, suction, Cube or D6 execution."""
import json
from pathlib import Path
import numpy as np

from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse
from offline.task28_p5_qp_6_r9.virtual_frames import make_virtual_frame
from offline.task28_p5_qp_6_r12.qp_math import (
    native_velocity_handoff, pose_error_world, quintic_profile,
    relative_pose_error, solve_pose_aware_qp, virtual_twist_quintic,
    lookahead_velocity_bounds, velocity_capture_reference,
)

ROOT = Path(__file__).resolve().parents[1]
R9 = ROOT.parent / "task28_p5_qp_6_r9"


def state():
    r = json.loads((R9 / "evidence/diagnostic/V2/result.json").read_text())
    a = r["ownership_initial_state"]["arms"]
    left, right = np.asarray(a["left"]["tcp_world"]), np.asarray(a["right"]["tcp_world"])
    q = np.r_[r["q_hold_actual"]["left"], r["q_hold_actual"]["right"]]
    return RobotModel(DEFAULT_FIXTURE), make_virtual_frame(left, right), left, right, q


def test_quintic_virtual_profile_is_c2_rest_to_rest():
    assert quintic_profile(0, 3.5, .0005) == (0., 0., 0.)
    assert np.allclose(quintic_profile(3.5, 3.5, .0005), (1., 0., 0.))
    assert virtual_twist_quintic(0., 3.5, .0005, "x")[0].tolist() == [0., 0., 0., 0., 0., 0.]


def test_native_handoff_is_c2_and_zero_net_area():
    dq = np.linspace(-.01, .01, 14)
    p0, v0, a0 = native_velocity_handoff(0., dq)
    p1, v1, a1 = native_velocity_handoff(.15, dq)
    assert np.allclose(p0, 0.) and np.allclose(p1, 0.)
    assert np.allclose(v0, dq) and np.allclose(v1, 0.)
    assert np.allclose(a0, 0.) and np.allclose(a1, 0.)


def test_velocity_capture_is_only_reference_velocity_and_frozen_tau():
    dq = np.linspace(-.01, .01, 14)
    captured = velocity_capture_reference(0.0, dq, 0.10)
    assert np.allclose(captured, -dq)
    assert np.linalg.norm(velocity_capture_reference(0.10, dq, 0.10)) < np.linalg.norm(captured)
    try:
        velocity_capture_reference(0.0, dq, 0.0)
    except ValueError as exc:
        assert str(exc) == "INVALID_VELOCITY_CAPTURE_INPUT"
    else:
        raise AssertionError("invalid capture tau accepted")


def test_pose_error_is_world_spatial_error():
    actual = np.eye(4); desired = np.eye(4); desired[0, 3] = .002
    assert np.allclose(pose_error_world(actual, desired)[:3], [.002, 0., 0.])


def test_relative_pose_order_is_registered():
    _, frame, left, right, _ = state()
    actual, desired, error = relative_pose_error(frame, left, right)
    assert np.allclose(actual, desired, atol=1e-8)
    assert np.max(np.abs(error)) < 1e-8
    assert np.max(np.abs(actual - inverse(frame.T_VR) @ frame.T_VL)) > 1e-5


def test_pose_aware_qp_is_14d_hard_relative_and_fail_closed():
    model, frame, left, right, q = state()
    right_bad = right.copy(); right_bad[0, 3] += .001
    x, d = solve_pose_aware_qp(model, q, frame, left, right_bad, frame.T_WV)
    assert x is not None and x.shape == (14,)
    assert d["hard_relative_pose_residual_inf"] <= 2e-8
    assert d["affine_equality_residual_inf"] <= 2e-8
    assert d["execution_authorized"] is False and d["inverse_dynamics_claim"] is False


def test_position_lookahead_box_has_margin_and_velocity_limit():
    model, _, _, _, q = state()
    lo, hi = lookahead_velocity_bounds(model, q, 1/60, .12, .02)
    assert np.all(lo <= hi)
    qnext = q + np.minimum(np.maximum(np.zeros(14), lo), hi) / 60
    assert np.all(qnext[:7] >= model.joint_lower + .02 - 1e-12)
    assert np.all(qnext[:7] <= model.joint_upper - .02 + 1e-12)


def test_cpu_candidate_and_reconstruction_evidence():
    cand = json.loads((ROOT / "evidence/cpu_candidates.json").read_text())
    assert all(cand["candidates"][x]["status"] == "PASS" for x in ("A", "B", "C"))
    assert all(not cand["candidates"][x]["execution_authorized"] for x in ("A", "B", "C"))
    rec = json.loads((ROOT / "evidence/r9_reconstruction.json").read_text())
    peak = rec["cells"]["V2"]["peak"]
    assert peak["object_translation_m"] > .001 and peak["relative_translation_m"] > .00015
    row = next(x for x in rec["cells"]["V2"]["records"] if x["physics_step"] == 201)
    assert abs(row["decomposition"]["right"]["vector_sum_error_m"] - row["common_right_translation_m"]) < 1e-12


def test_original_cube_approach_is_blocked_without_continuous_certificate():
    d = json.loads((ROOT / "evidence/continuous_safety_audit.json").read_text())
    assert d["status"] == "CONTINUOUS_SAFETY_NOT_ESTABLISHED"
    assert d["execution"]["status"] == "BLOCKED"
    assert d["not_a_certificate"] is True
