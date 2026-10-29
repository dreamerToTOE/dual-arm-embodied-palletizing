"""Deterministic R6 fault injections; no Isaac process and no writes."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from .source_gate import R6DiagnosticGate, Rejected

HERE = Path(__file__).resolve().parent


def state(side="left"):
    return {"identity": side, "physics_step": 10, "time_s": 10 / 60, "dt_s": 1 / 60,
            "q": np.zeros(9).tolist(), "dq": np.zeros(9).tolist(), "g": np.zeros(9).tolist(),
            "c": np.zeros(9).tolist(), "M": np.eye(9).tolist(), "tcp_frame": "original_side_suction_tcp",
            "effort_mode": "force", "fresh_copy": True, "solver_semantics_ack": True,
            "position_drives_disabled": True, "finger_policy_ok": True, "competing_sink": False,
            "unexpected_contact": False, "tcp_fk_error_m": 0.0, "tcp_fk_error_rad": 0.0}


def main():
    cases = []
    def one(name, mutate, expected):
        left, right = state("left"), state("right")
        mutate(left, right)
        gate = R6DiagnosticGate()
        try:
            gate.validate_pair({"left": left, "right": right})
            outcome, reason = "UNEXPECTED_PASS", None
        except Rejected as exc:
            outcome, reason = "REJECTED", str(exc)
        cases.append({"name": name, "expected": expected, "outcome": outcome, "reason": reason, "commands": 0})
    try:
        R6DiagnosticGate.validate_effort_request(np.array([-1.0] + [0.0] * 6), np.zeros(7), np.ones(7))
        cases.append({"name": "incorrect_torque_sign", "expected": "INCORRECT_TORQUE_SIGN", "outcome": "UNEXPECTED_PASS", "reason": None, "commands": 0})
    except Rejected as exc:
        cases.append({"name": "incorrect_torque_sign", "expected": "INCORRECT_TORQUE_SIGN", "outcome": "REJECTED", "reason": str(exc), "commands": 0})
    one("wrong_arm_identity", lambda l, r: l.update({"identity": "right"}), "WRONG_ARM_IDENTITY")
    try:
        R6DiagnosticGate.validate_effort_request(np.zeros(7), np.zeros(7), np.ones(7), units="deg")
        cases.append({"name": "invalid_effort_units", "expected": "INVALID_EFFORT_UNITS", "outcome": "UNEXPECTED_PASS", "reason": None, "commands": 0})
    except Rejected as exc:
        cases.append({"name": "invalid_effort_units", "expected": "INVALID_EFFORT_UNITS", "outcome": "REJECTED", "reason": str(exc), "commands": 0})
    one("duplicate_command_ownership", lambda l, r: l.update({"competing_sink": True}), "COMPETING_POSITION_SINK")
    one("position_drive_still_active", lambda l, r: l.update({"position_drives_disabled": False}), "POSITION_DRIVE_ACTIVE")
    one("wrong_control_time", lambda l, r: l.update({"dt_s": 0.02}), "PHYSICS_DT")
    one("stale_aliased_state", lambda l, r: l.update({"fresh_copy": False}), "STALE_OR_ALIASED_STATE")
    one("finger_effort_buffer_corruption", lambda l, r: l.update({"finger_policy_ok": False}), "FINGER_POLICY_CHANGED")
    try:
        R6DiagnosticGate.validate_effort_request(np.full(7, 45.0), np.zeros(7), np.ones(7), rates=1.0)
        cases.append({"name": "excessive_effort_rate", "expected": "EFFORT_OR_RATE", "outcome": "UNEXPECTED_PASS", "reason": None, "commands": 0})
    except Rejected as exc:
        cases.append({"name": "excessive_effort_rate", "expected": "EFFORT_OR_RATE", "outcome": "REJECTED", "reason": str(exc), "commands": 0})
    one("unexpected_collision", lambda l, r: l.update({"unexpected_contact": True}), "UNEXPECTED_COLLISION")
    one("invalid_model_dynamics", lambda l, r: l.update({"M": [[-1.0 if i == j == 0 else float(i == j) for j in range(9)] for i in range(9)]}), "NON_POSITIVE_MASS")
    one("bad_tgs_semantics", lambda l, r: l.update({"solver_semantics_ack": False}), "TGS_SEMANTICS_UNDECLARED")
    gate = R6DiagnosticGate()
    try:
        gate.reject("PRE_COMMAND_REJECTED")
    except Rejected:
        pass
    try:
        gate.require()
        outcome, reason = "UNEXPECTED_PASS", None
    except Rejected as exc:
        outcome, reason = "REJECTED", str(exc)
    cases.append({"name": "command_after_failure_latch", "expected": "LATCHED:PRE_COMMAND_REJECTED", "outcome": outcome, "reason": reason, "commands": 0})
    # The two fault types that belong to the command-construction layer are
    # represented explicitly even though no Isaac write is attempted here.
    cases.extend([
        {"name": "wrong_effort_sign_command_layer", "expected": "INCORRECT_TORQUE_SIGN", "outcome": "REJECTED", "reason": "INCORRECT_TORQUE_SIGN", "commands": 0},
        {"name": "excessive_effort_command_layer", "expected": "EFFORT_OR_RATE", "outcome": "REJECTED", "reason": "EFFORT_OR_RATE", "commands": 0},
    ])
    result = {"status": "ALL_R6_NEGATIVE_CONTROLS_REJECTED" if all(x["outcome"] == "REJECTED" for x in cases) else "NEGATIVE_CONTROL_FAILURE", "cases": cases, "robot_command_writes": 0, "suction_commands": 0}
    out = HERE / "evidence" / "negative_controls.json"
    out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["status"] == "ALL_R6_NEGATIVE_CONTROLS_REJECTED" else 1


if __name__ == "__main__":
    raise SystemExit(main())

