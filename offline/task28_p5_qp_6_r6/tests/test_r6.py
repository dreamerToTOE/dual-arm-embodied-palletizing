import hashlib
import json
from pathlib import Path
import numpy as np
import pytest

from offline.task28_p5_qp_6_r2.common import DOF_NAMES, FR3_SHA256, LEFT_ROOT, RIGHT_ROOT, TASK_ROOT
from offline.task28_p5_qp_6_r6.effort_adapter import DiagnosticCertificate, R6ExclusiveEffortAdapter, Reject
from offline.task28_p5_qp_6_r6.source_gate import R6DiagnosticGate, Rejected


class Prim:
    def IsValid(self): return True
    def IsActive(self): return False
    def GetCustomDataByKey(self, key):
        return {"task28_g1_0_fixture_id": "task28_g1_0_new_single_cube_v0", "task28_g1_0_fr3_usd_sha256": FR3_SHA256}.get(key)


class Stage:
    def GetPrimAtPath(self, path): return Prim()


class View:
    def __init__(self):
        self.kp = np.array([[10.0] * 7 + [60000.0, 0.0]])
        self.kd = np.array([[2.0] * 7 + [6000.0, 0.0]])
        self.mode = ["force"] * 9
    def get_gains(self): return self.kp.copy(), self.kd.copy()
    def get_effort_modes(self): return [self.mode.copy()]
    def set_gains(self, kps, kds, joint_indices, save_to_usd):
        assert save_to_usd is False
        self.kp[0, joint_indices] = kps
        self.kd[0, joint_indices] = kds


class Arm:
    def __init__(self, path):
        self.prim_path = path
        self.dof_names = DOF_NAMES
        self.handles_initialized = True
        self._articulation_view = View()
        self.applied = np.array([0.0] * 9)
        self.calls = []
    def get_applied_joint_efforts(self): return self.applied.copy()
    def set_joint_efforts(self, efforts, joint_indices):
        self.applied = np.asarray(efforts, dtype=float).copy()
        self.calls.append((self.applied.copy(), np.asarray(joint_indices).copy()))


def state(side="left", disabled=False, step=0, time=0.0):
    return {"identity": side, "physics_step": step, "time_s": time, "dt_s": 1 / 60,
            "q": np.zeros(9), "dq": np.zeros(9), "g": np.zeros(9), "c": np.zeros(9), "M": np.eye(9),
            "tcp_frame": "original_side_suction_tcp", "effort_mode": "force", "fresh_copy": True,
            "solver_semantics_ack": True, "position_drives_disabled": disabled, "finger_policy_ok": True,
            "competing_sink": False, "unexpected_contact": False, "tcp_fk_error_m": 0.0, "tcp_fk_error_rad": 0.0}


def make(tmp_path, disabled=False):
    evidence = {"status": "DIAGNOSTIC_ACTUATION_SOURCE_READY", "level": 1}
    path = tmp_path / "level1.json"; path.write_text(json.dumps(evidence))
    cert = DiagnosticCertificate(evidence["status"], FR3_SHA256, 1 / 60, "task28_g1_0_new_single_cube_v0", hashlib.sha256(path.read_bytes()).hexdigest(), str(path))
    arms = {"left": Arm(LEFT_ROOT), "right": Arm(RIGHT_ROOT)}
    events = []
    gate = R6DiagnosticGate()
    adapter = R6ExclusiveEffortAdapter(arms, Stage(), cert, gate, events.append, np.full(7, 45.0), np.full(7, 2400.0))
    return adapter, arms, events, {"left": state("left", disabled), "right": state("right", disabled)}


def test_level1_gate_accepts_tgs_end_step_velocity_without_fd_equality():
    gate = R6DiagnosticGate()
    left, right = state("left", True), state("right", True)
    left["dq"][3] = 0.5; right["dq"][3] = -0.5
    left["fd_minus_reported"] = [99.0] * 9
    assert gate.validate_pair({"left": left, "right": right})["valid"]


def test_activation_zeroes_arm_gains_and_preserves_fingers(tmp_path):
    adapter, arms, events, states = make(tmp_path)
    states["left"]["position_drives_disabled"] = False; states["right"]["position_drives_disabled"] = False
    adapter.activate(states)
    for arm in arms.values():
        assert np.allclose(arm._articulation_view.kp[0, :7], 0)
        assert np.allclose(arm._articulation_view.kd[0, :7], 0)
        assert np.array_equal(arm._articulation_view.kp[0, 7:], [60000, 0])
        assert np.array_equal(arm._articulation_view.kd[0, 7:], [6000, 0])
    assert [e["event"] for e in events] == ["OWNERSHIP_PREPARED", "OWNERSHIP_ACTIVE"]


def test_full_buffer_and_immediate_readback_preserve_finger_buffer(tmp_path):
    adapter, arms, events, states = make(tmp_path)
    adapter.activate(states)
    states["left"]["position_drives_disabled"] = True; states["right"]["position_drives_disabled"] = True
    full = adapter.apply_pair(states, {"left": np.ones(7), "right": np.ones(7) * 2}, 1 / 60)
    assert np.array_equal(full["left"][7:], [0, 0])
    assert np.array_equal(arms["left"].calls[-1][1], np.arange(9))
    adapter.post_step({"left": state("left", True, 1, 1 / 60), "right": state("right", True, 1, 1 / 60)})
    assert not adapter.pending


def test_pair_prevalidation_rejects_before_either_write(tmp_path):
    adapter, arms, events, states = make(tmp_path)
    adapter.activate(states)
    states["left"]["position_drives_disabled"] = True; states["right"]["position_drives_disabled"] = True
    with pytest.raises(Reject, match="EFFORT_OR_RATE"):
        adapter.apply_pair(states, {"left": np.ones(7) * 100, "right": np.ones(7)}, 1 / 60)
    assert all(not arm.calls for arm in arms.values())


def test_latched_failure_blocks_after_command_attempt():
    gate = R6DiagnosticGate()
    with pytest.raises(Rejected): gate.reject("PRE_COMMAND_REJECTED")
    with pytest.raises(Rejected, match="LATCHED:PRE_COMMAND_REJECTED"): gate.require()


def test_effort_units_sign_and_rate_faults():
    with pytest.raises(Rejected, match="INVALID_EFFORT_UNITS"):
        R6DiagnosticGate.validate_effort_request(np.zeros(7), np.zeros(7), np.ones(7), units="deg")
    with pytest.raises(Rejected, match="INCORRECT_TORQUE_SIGN"):
        R6DiagnosticGate.validate_effort_request(-np.ones(7), np.zeros(7), np.ones(7))
    with pytest.raises(Rejected, match="EFFORT_OR_RATE"):
        R6DiagnosticGate.validate_effort_request(np.ones(7) * 45, np.zeros(7), np.ones(7), rates=1.0)


def test_source_gate_rejects_finger_corruption_and_active_position_drive():
    gate = R6DiagnosticGate()
    left, right = state("left", True), state("right", True)
    left["finger_policy_ok"] = False
    with pytest.raises(Rejected, match="FINGER_POLICY_CHANGED"): gate.validate_pair({"left": left, "right": right})
    gate = R6DiagnosticGate(); left, right = state("left", False), state("right", True)
    with pytest.raises(Rejected, match="POSITION_DRIVE_ACTIVE"): gate.validate_pair({"left": left, "right": right})


def test_source_gate_requires_two_arm_time_sync():
    gate = R6DiagnosticGate(); left, right = state("left", True), state("right", True, step=1, time=1 / 60)
    with pytest.raises(Rejected, match="TIME_SYNC"): gate.validate_pair({"left": left, "right": right})


def test_registered_perturbation_is_small_and_bounded():
    reg = json.loads((Path(__file__).resolve().parents[1] / "registrations.json").read_text())
    assert {x["perturbation"] for x in reg.values()} == {0.0, 0.25, -0.25}
    assert max(abs(x["perturbation"]) for x in reg.values()) < 1.0


def test_no_r5_nonbaseline_flag_in_r6_registration():
    data = json.loads((Path(__file__).resolve().parents[1] / "preregistration.json").read_text())
    assert data["scene_policy"].startswith("fresh anonymous")
    assert "external_forces" not in json.dumps(data).lower()


def test_r6_negative_controls_are_write_free():
    data = json.loads((Path(__file__).resolve().parents[1] / "evidence/negative_controls.json").read_text())
    assert data["status"] == "ALL_R6_NEGATIVE_CONTROLS_REJECTED"
    assert data["robot_command_writes"] == 0
    assert len(data["cases"]) == 15


def test_no_static_or_tracking_claim_is_predeclared():
    contract = json.loads((Path(__file__).resolve().parents[1] / "source_contract.json").read_text())
    assert "mounted-tool 9-coordinate" in contract["model_scope"]

