"""Focused R9 tests; all checks are CPU/read-only and never start Isaac."""
import json
from pathlib import Path

import numpy as np
import pytest

from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r9.qp_math import (
    finite_difference_jacobian,
    object_to_tcp_map,
    solve_cooperative_qp,
    virtual_pose,
    virtual_twist,
)
from offline.task28_p5_qp_6_r9.virtual_frames import (
    VirtualCooperativeFrame,
    make_virtual_frame,
    nominal_cube_grasps,
    physical_cube_pose,
    reject_virtual_as_physical,
)

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence"


def _cpu():
    return json.loads((EVIDENCE / "virtual_qp_cpu.json").read_text())


def _frame_and_q():
    r8 = json.loads((ROOT.parent / "task28_p5_qp_6_r8/evidence/diagnostic/MOVING/result.json").read_text())
    arms = r8["ownership_initial_state"]["arms"]
    left, right = np.asarray(arms["left"]["tcp_world"]), np.asarray(arms["right"]["tcp_world"])
    q = np.r_[r8["q_hold_actual"]["left"], r8["q_hold_actual"]["right"]]
    return RobotModel(DEFAULT_FIXTURE), make_virtual_frame(left, right), np.asarray(q, float)


def test_virtual_identity_is_typed_and_distinct_from_physical_cube():
    model, frame, _ = _frame_and_q()
    assert frame.identity == "FREE_HOME_VIRTUAL_COOPERATION"
    assert physical_cube_pose().identity == "ATTACHED_PHYSICAL_CUBE_CANDIDATE"
    assert nominal_cube_grasps().identity == "PRE_CONTACT_ORIGINAL_CUBE"
    assert frame.identity != physical_cube_pose().identity


def test_virtual_frame_reconstructs_both_measured_tcps():
    _, frame, _ = _frame_and_q()
    left, right = frame.tcp_goals()
    assert np.max(np.abs(left - frame.T_WV @ frame.T_VL)) < 1e-12
    assert np.max(np.abs(right - frame.T_WV @ frame.T_VR)) < 1e-12


def test_object_to_tcp_map_has_world_linear_angular_convention():
    T = np.eye(4)
    tcp = np.eye(4)
    tcp[:3, 3] = [0.2, -0.1, 0.3]
    A = object_to_tcp_map(T, tcp)
    twist = np.r_[np.zeros(3), [0.0, 0.0, 1.0]]
    assert np.allclose(A @ twist, np.r_[[0.1, 0.2, 0.0], [0.0, 0.0, 1.0]])


def test_virtual_trajectory_is_zero_velocity_at_boundaries():
    for t in (0.0, 3.5):
        twist, _ = virtual_twist(t, 3.5, 0.0005, "x")
        assert np.max(np.abs(twist)) == 0.0
    _, s0 = virtual_twist(0.0, 3.5, 0.0005, "x")
    _, s1 = virtual_twist(3.5, 3.5, 0.0005, "x")
    assert s0 == 0.0 and s1 == 1.0


def test_qp_is_centralized_14d_and_kkt_verified():
    model, frame, q = _frame_and_q()
    x, diag = solve_cooperative_qp(model, q, frame, 0.2, duration_s=3.5, amplitude_m=0.0005, previous=np.zeros(14))
    assert x is not None and x.shape == (14,)
    assert diag["dimension"] == 14
    assert diag["independent_kkt_verified"] is True
    assert diag["execution_authorized"] is False
    assert diag["inverse_dynamics_claim"] is False


def test_qp_hard_relative_constraint_and_box_are_satisfied():
    model, frame, q = _frame_and_q()
    x, diag = solve_cooperative_qp(model, q, frame, 0.5, duration_s=3.5, amplitude_m=0.0005)
    assert x is not None
    assert diag["hard_relative_residual_inf"] <= 2e-7
    assert np.max(np.abs(x)) <= 0.12 + 1e-10


def test_qp_rejects_invalid_axis_and_empty_box():
    model, frame, q = _frame_and_q()
    with pytest.raises(ValueError, match="INVALID_VIRTUAL_TRAJECTORY"):
        solve_cooperative_qp(model, q, frame, 0.0, axis="y")
    from offline.task28_p5_qp_0.solver import solve_box_equality_qp
    out = solve_box_equality_qp(np.eye(1), np.zeros(1), np.zeros((0, 1)), np.array([1.0]), np.array([0.0]))
    assert out.status == "QP_INFEASIBLE"


def test_fd_jacobians_match_world_twist_sources():
    model, _, q = _frame_and_q()
    for side, part in (("left", q[:7]), ("right", q[7:])):
        analytic, numeric = finite_difference_jacobian(model, side, part)
        assert analytic.shape == (6, 7)
        assert np.max(np.abs(analytic - numeric)) < 2e-6


def test_virtual_to_physical_promotion_is_fail_closed():
    _, frame, _ = _frame_and_q()
    decision = reject_virtual_as_physical(frame, physical_cube_pose(), nominal_cube_grasps())
    assert decision["promotion_allowed"] is False
    assert decision["reason"] == "VIRTUAL_FRAME_NEVER_PROMOTES_TO_PHYSICAL_CUBE"


def test_free_home_nominal_grasp_negative_is_preserved():
    d = _cpu()
    neg = d["negative_controls"]["nominal_grasp_on_free_home"]
    assert neg["pass"] is True
    assert "LEFT_RIGHT_OBJECT_TRANSLATION_MISMATCH" in neg["closure_reasons"]


def test_cpu_plan_has_no_unexpected_fcl_collision():
    d = _cpu()
    plan = d["virtual_plan"]
    assert d["status"] == "PASS"
    assert plan["fcl_samples"] == 211
    assert plan["fcl_unexpected"] == []
    assert plan["object_tracking_peak_m"] < 1e-4


def test_cpu_jacobian_and_negative_controls_are_recorded():
    d = _cpu()
    assert all(v["pass"] for v in d["jacobian_fd"].values())
    assert all(d["negative_controls"][k]["pass"] for k in ("wrong_axis_frame", "wrong_left_right_tcp_frame", "wrong_object_transform_order", "infeasible_qp_box"))


def test_original_approach_is_sampled_only_and_keeps_r3_collision():
    d = json.loads((EVIDENCE / "approach_audit.json").read_text())
    assert d["status"] == "SAMPLED_CANDIDATE_ONLY"
    assert all(v["sampled_valid"] for v in d["synchronized_edges"].values())
    assert d["physical_execution"]["status"] == "SKIPPED"
    assert d["direct_home_to_nominal_negative"]["first_unexpected_collision"]["fraction"] == 0.92


def test_successful_cells_are_no_contact_but_frozen_accuracy_gate_fails():
    d = json.loads((EVIDENCE / "evaluation.json").read_text())
    for name in ("V1_RECOVERY2", "V2"):
        cell = d["cells"][name]
        assert cell["status"] == "PASS_NO_CONTACT_ACCURACY_GATE_FAIL"
        assert cell["no_contact_status"] == "PASS"
        assert cell["accuracy_status"] == "FAIL_FROZEN_PREREG_THRESHOLDS"
        assert cell["cube_unchanged"] is True
        assert cell["suction_commands"] == 0


def test_rejected_cells_have_no_effort_commands():
    d = json.loads((EVIDENCE / "evaluation.json").read_text())
    assert d["cells"]["V1"]["commands"] == 0
    assert d["cells"]["V1_RECOVERY"]["commands"] == 0
    assert d["cells"]["V1"]["status"] == "NOT_ELIGIBLE"
    assert d["cells"]["V1_RECOVERY"]["status"] == "NOT_ELIGIBLE"


def test_process_audit_covers_all_four_registered_cells():
    d = json.loads((EVIDENCE / "process_isolation_audit.json").read_text())
    assert set(d) == {"V1", "V1_RECOVERY", "V1_RECOVERY2", "V2"}
    assert all(v["verified"] for v in d.values())


def test_source_contract_keeps_r8_limits_and_forbids_contact_runtime():
    d = json.loads((ROOT / "source_contract.json").read_text())
    assert d["limits"]["source_effort_abs_max_Nm"] == [87, 87, 87, 87, 12, 12, 12]
    assert d["limits"]["research_effort_abs_max_Nm"] == [35, 12, 8, 35, 4, 4, 4]
    assert d["r8_interface"]["full_effort_buffer_dof"] == 9
    assert "suction" in d["forbidden"] and "internal-wrench control" in d["forbidden"]

