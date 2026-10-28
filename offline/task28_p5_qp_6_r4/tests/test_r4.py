import json
from pathlib import Path
import numpy as np
import pytest

from offline.task28_p5_qp_6_r4.source_gate import LatchedSourceGate, SourceRejected, torque_attempt

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "evidence"


def load_result(name):
    return json.loads((EVIDENCE / f"run_{name}" / "result.json").read_text())


def load_rows(name):
    return [json.loads(x) for x in (EVIDENCE / f"run_{name}" / "frames.jsonl").read_text().splitlines()]


def test_exactly_six_registered_diagnostic_cells_and_no_command_writes():
    regs = json.loads((ROOT / "registrations.json").read_text())
    assert list(regs) == list("ABCDEF")
    for name in "ABCDEF":
        d = load_result(name)
        assert d["robot_command_writes"] == 0
        assert d["suction_commands"] == 0
        assert d["scene_before"]["status"] == "SCENE_IDENTITY_VERIFIED"
        assert d["cube_mass_kg"] == pytest.approx(0.8, abs=1e-6)


def test_each_cell_reproduces_the_latched_stale_state_gate():
    for name in "ABCDEF":
        d = load_result(name)
        assert d["status"] == "REJECTED"
        assert d["first_failure"]["message"] == "FROZEN_Q_NONZERO_DQ"
        assert d["explicit_physics_steps"] == 128
        assert d["recorded_frames"] == 129


def test_physics_cursor_and_dt_are_monotonic_in_every_cell():
    for name in "ABCDEF":
        rows = load_rows(name)
        steps = [r["physics_step"] for r in rows]
        times = [r["time_s"] for r in rows]
        assert all(b == a + 1 for a, b in zip(steps, steps[1:]))
        assert all(b > a for a, b in zip(times, times[1:]))
        assert max(abs((b - a) - 1 / 60) for a, b in zip(times, times[1:])) < 1e-8


def test_callback_snapshot_matches_immediate_fetch_in_c():
    for row in load_rows("C")[1:]:
        assert row["post_callback"] is not None
        for side in ("left", "right"):
            assert np.allclose(row["post_callback"]["snapshot"][side]["q"], row["arms"][side]["q"], atol=0, rtol=0)
            assert np.allclose(row["post_callback"]["snapshot"][side]["dq"], row["arms"][side]["dq"], atol=0, rtol=0)


def test_dynamic_control_agrees_but_is_not_independent_in_b_and_e():
    for name in "BE":
        for row in load_rows(name):
            for side in ("left", "right"):
                assert np.allclose(row["arms"][side]["dynamic_control_q"], row["arms"][side]["q"], atol=0, rtol=0)
                assert np.allclose(row["arms"][side]["dynamic_control_dq"], row["arms"][side]["dq"], atol=0, rtol=0)


def test_borrowed_tensor_reference_is_mutable_and_not_used_as_certificate():
    rows = load_rows("D")
    changes = []
    for row in rows[1:]:
        arm = row["arms"]["left"]
        changes.append(max(abs(a - b) for a, b in zip(arm["previous_borrowed_q_after_new_read"], arm["previous_immediate_q"])))
    assert max(changes) > 0.08
    assert all(row["arms"]["left"]["native_wrapper_q_shares_memory"] is False for row in rows)


def test_link_tcp_fk_and_collision_checks_remain_valid_before_source_failure():
    for name in "ABCDEF":
        for row in load_rows(name):
            assert row["collision"]["valid"] is True
            assert row["collision"]["unexpected"] == []
            for side in ("left", "right"):
                assert row["arms"][side]["tcp_fk_error_m"] < 1e-3
                assert row["arms"][side]["tcp_fk_error_rad"] < 2e-3


def test_historical_r3_reconstruction_preserves_unknown_aliasing_and_detects_tail_error():
    d = json.loads((EVIDENCE / "historical" / "dev_02.json").read_text())
    assert d["archive_aliasing_verdict"].startswith("UNKNOWN")
    tail = d["records"][-1]
    assert max(abs(x) for x in tail["sides"]["left"]["difference_reported_vs_fd"][:7]) > 0.01
    assert max(abs(x) for x in tail["sides"]["right"]["difference_reported_vs_fd"][:7]) > 0.01


def test_negative_controls_all_reject_and_are_write_free():
    d = json.loads((EVIDENCE / "negative_controls.json").read_text())
    assert d["status"] == "ALL_NEGATIVE_CONTROLS_REJECTED"
    assert len(d["cases"]) == 13
    assert d["robot_command_writes"] == 0
    assert all(x["outcome"] == "REJECTED" and x["reason"] == x["expected"] for x in d["cases"])


def test_latched_rejection_prevents_a_later_torque_attempt():
    gate = LatchedSourceGate()
    with pytest.raises(SourceRejected, match="UNEXPECTED_COLLISION"):
        gate.reject("UNEXPECTED_COLLISION")
    with pytest.raises(SourceRejected, match="LATCHED_REJECTION"):
        torque_attempt(gate)


def test_experiment_status_is_fail_closed():
    d = json.loads((EVIDENCE / "experiment_status.json").read_text())
    assert d == {
        "ORIGINAL_SCENE_UNCHANGED": True,
        "JOINT_STATE_SOURCE": "BLOCKED",
        "NATIVE_SETTLE": "UNRESOLVED",
        "ARM_DYNAMICS_SOURCE": "PARTIAL",
        "EXCLUSIVE_TORQUE_COMMAND": "BLOCKED",
        "STATIC_TORQUE_HOLD": "SKIPPED",
        "SHORT_TORQUE_TRACKING": "SKIPPED",
        "VIRTUAL_COOPERATIVE_QP": "BLOCKED_SOURCE_PREREQUISITE",
        "CUBE_SUCTION": "UNTESTED",
        "stop_at_r4": True,
    }


def test_source_contract_keeps_native_settle_limit_and_separates_moving_claim():
    c = json.loads((ROOT / "source_contract.json").read_text())
    assert c["settle_arm_speed_rad_s"] == pytest.approx(0.01)
    assert c["moving_and_settled_qualified_separately"] is True
    assert c["raw_array_policy"].startswith("immutable")
