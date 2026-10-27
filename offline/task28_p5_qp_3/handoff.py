"""Offline-only QP-2 velocity to position-command handoff.

The adapter creates typed candidate JointState-like records in memory.  It
does not import ROS and has no publisher or actuator entry point.  The QP-2
driver remains the authority for the current/predicted G1 and sampled B0
checks; this module only validates the position-command contract around its
accepted q_next values.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math
from typing import Any, Iterable

import numpy as np

from offline.task28_g1_1.task28_validator.state import expected_joint_names
from offline.task28_p5_qp_1.driver import MultiStepRequest
from offline.task28_p5_qp_2.driver import FeedbackDriver


LEFT = "left"
RIGHT = "right"
SIDES = (LEFT, RIGHT)
# The isolated fixture bridge recognizes this frame for the paired position
# path.  QP-3 never publishes it; the value is only a static contract check.
FRAME_ID = "task28_g1_0_dual_sync"
WORLD_FRAME = "world"
EXPECTED_CUBE_ID = "Cube_01"
JOINT_ORDER = tuple(expected_joint_names(LEFT) + expected_joint_names(RIGHT))


@dataclass(frozen=True)
class HandoffContract:
    """The static contract used by the offline candidate sink."""

    fixture_id: str = "task28_g1_0_new_single_cube_v0"
    cube_id: str = EXPECTED_CUBE_ID
    frame_id: str = FRAME_ID
    world_frame: str = WORLD_FRAME
    left_joint_names: tuple[str, ...] = expected_joint_names(LEFT)
    right_joint_names: tuple[str, ...] = expected_joint_names(RIGHT)
    timestamp_semantics: str = "synthetic_monotonic_candidate_time_not_ROS_or_servo_time"
    command_semantics: str = "position_target_only_in_memory_no_publish"
    feedback_semantics: str = "QP2_source_state_is_saved_or_synthetic_offline_not_actuator_feedback"
    execution_authorized: bool = False


@dataclass(frozen=True)
class PositionTarget:
    sequence: int
    time_s: float
    stamp_s: float
    frame_id: str
    side: str
    joint_names: tuple[str, ...]
    positions_rad: tuple[float, ...]
    source_step_index: int
    source_qdot_rad_s: tuple[float, ...]
    provenance: str = "QP2_ACCEPTED_PREDICTED_Q_NEXT_OFFLINE"
    execution_authorized: bool = False


def _finite_vector(values: Iterable[float], length: int, label: str) -> np.ndarray:
    arr = np.asarray(tuple(values), dtype=float)
    if arr.shape != (length,) or not np.isfinite(arr).all():
        raise ValueError(f"{label}_NONFINITE_OR_WRONG_LENGTH")
    return arr


def _check_timestamp(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label}_INVALID_TIMESTAMP")
    return float(value)


def validate_contract(contract: HandoffContract) -> None:
    if contract.cube_id != EXPECTED_CUBE_ID:
        raise ValueError("INCORRECT_CUBE_ID")
    if contract.world_frame != WORLD_FRAME or contract.frame_id != FRAME_ID:
        raise ValueError("INCORRECT_HANDOFF_FRAME")
    if contract.left_joint_names != expected_joint_names(LEFT):
        raise ValueError("LEFT_JOINT_ORDER_MISMATCH")
    if contract.right_joint_names != expected_joint_names(RIGHT):
        raise ValueError("RIGHT_JOINT_ORDER_MISMATCH")
    if contract.execution_authorized is not False:
        raise ValueError("EXECUTION_MUST_REMAIN_DISABLED")


def make_position_targets(q_initial: Iterable[float], q_next: Iterable[float], *,
                          time_s: float, stamp_s: float, sequence: int,
                          source_step_index: int, qdot: Iterable[float],
                          contract: HandoffContract = HandoffContract()) -> tuple[PositionTarget, PositionTarget]:
    """Split an accepted 14-DOF q_next into synchronized left/right targets."""
    validate_contract(contract)
    q0 = _finite_vector(q_initial, 14, "Q_INITIAL")
    q1 = _finite_vector(q_next, 14, "Q_NEXT")
    qd = _finite_vector(qdot, 14, "QDOT")
    t = _check_timestamp(time_s, "TIME")
    stamp = _check_timestamp(stamp_s, "STAMP")
    if not isinstance(sequence, int) or sequence < 0:
        raise ValueError("INVALID_COMMAND_SEQUENCE")
    if not isinstance(source_step_index, int) or source_step_index < 0:
        raise ValueError("INVALID_SOURCE_STEP")
    # The candidate is expected to be the same Euler update that QP-2 accepted.
    if not np.allclose(q1, q0 + (stamp - t) * qd, atol=1e-10, rtol=0.0):
        raise ValueError("Q_NEXT_NOT_RECONSTRUCTED_FROM_QDOT")
    return (
        PositionTarget(sequence, t, stamp, contract.frame_id, LEFT,
                       contract.left_joint_names, tuple(q1[:7]), source_step_index,
                       tuple(qd[:7])),
        PositionTarget(sequence, t, stamp, contract.frame_id, RIGHT,
                       contract.right_joint_names, tuple(q1[7:]), source_step_index,
                       tuple(qd[7:])),
    )


def validate_target_pair(pair: tuple[PositionTarget, PositionTarget], *,
                         previous_stamp_s: float | None = None,
                         max_feedback_age_s: float | None = None,
                         feedback_stamp_s: float | None = None,
                         expected_sequence: int | None = None,
                         contract: HandoffContract = HandoffContract()) -> dict[str, Any]:
    """Fail closed on ordering, timestamp, age, frame or side mismatches."""
    validate_contract(contract)
    if not isinstance(pair, tuple) or len(pair) != 2:
        raise ValueError("DUAL_TARGET_PAIR_REQUIRED")
    left, right = pair
    if left.side != LEFT or right.side != RIGHT:
        raise ValueError("SIDE_IDENTITY_MISMATCH")
    if left.frame_id != contract.frame_id or right.frame_id != contract.frame_id:
        raise ValueError("HANDOFF_FRAME_MISMATCH")
    if left.joint_names != contract.left_joint_names or right.joint_names != contract.right_joint_names:
        raise ValueError("JOINT_ORDER_MISMATCH")
    if left.sequence != right.sequence or left.stamp_s != right.stamp_s:
        raise ValueError("DUAL_TARGET_NOT_SYNCHRONIZED")
    if expected_sequence is not None and left.sequence != expected_sequence:
        raise ValueError("COMMAND_SEQUENCE_GAP")
    if previous_stamp_s is not None and left.stamp_s <= previous_stamp_s:
        raise ValueError("NON_MONOTONIC_COMMAND_TIMESTAMP")
    if max_feedback_age_s is not None:
        age = left.stamp_s - _check_timestamp(feedback_stamp_s, "FEEDBACK")
        if age < 0 or age > max_feedback_age_s:
            raise ValueError("FEEDBACK_STALE_OR_FROM_FUTURE")
    _finite_vector(left.positions_rad, 7, "LEFT_POSITION")
    _finite_vector(right.positions_rad, 7, "RIGHT_POSITION")
    return {"valid": True, "sequence": left.sequence, "stamp_s": left.stamp_s,
            "feedback_age_s": None if feedback_stamp_s is None else left.stamp_s - float(feedback_stamp_s),
            "execution_authorized": False}


def run_offline_handoff(q14: Iterable[float], object_pose: Any, desired_twist: Iterable[float],
                        *, dt_s: float, max_steps: int, identity: str,
                        contract: HandoffContract = HandoffContract(),
                        validator=None) -> dict[str, Any]:
    """Run QP-2 feedback and retain only accepted position-target candidates."""
    validate_contract(contract)
    if not isinstance(dt_s, (int, float)) or isinstance(dt_s, bool) or not math.isfinite(dt_s) or dt_s <= 0:
        raise ValueError("INVALID_SYNTHETIC_DT")
    request = MultiStepRequest(tuple(q14), object_pose, tuple(desired_twist), float(dt_s),
                               int(max_steps), identity, EXPECTED_CUBE_ID, WORLD_FRAME,
                               {"source": "SAVED_NR0_NODE_THEN_QP2_OFFLINE", "execution_authorized": False})
    raw = FeedbackDriver(validator=validator).run(request)
    targets: list[dict[str, Any]] = []
    rejects: list[dict[str, Any]] = []
    previous_stamp = None
    previous_q = np.asarray(tuple(q14), dtype=float)
    for step in raw.get("steps", []):
        if step.get("accepted") is not True:
            rejects.append({"step_index": step.get("step_index"), "reasons": step.get("reasons", []),
                            "retained_failing_candidate": step.get("q_next") is not None})
            break
        q_next = _finite_vector(step["q_next"], 14, "Q_NEXT")
        qdot = _finite_vector(step["qdot14"], 14, "QDOT")
        stamp = float(step["target_time_s"])
        pair = make_position_targets(previous_q, q_next, time_s=float(step["time_s"]),
                                     stamp_s=stamp, sequence=int(step["step_index"]),
                                     source_step_index=int(step["step_index"]), qdot=qdot,
                                     contract=contract)
        check = validate_target_pair(pair, previous_stamp_s=previous_stamp,
                                     expected_sequence=int(step["step_index"]), contract=contract)
        targets.append({"left": asdict(pair[0]), "right": asdict(pair[1]), "contract_check": check,
                        "qp2_step": int(step["step_index"]),
                        "predicted_state_valid": bool(step["predicted_state"]["valid"]),
                        "sampled_edge_status": step["sampled_edge"]["status"],
                        "closure_drift": step.get("drift_metrics", step["predicted_kinematics"]["closure"]),
                        "position_discontinuity_max_rad": float(np.max(np.abs(q_next - previous_q))),
                        "qdot_change_rad_s": None if not targets else tuple((qdot - np.asarray(targets[-1]["left"]["source_qdot_rad_s"] + targets[-1]["right"]["source_qdot_rad_s"])).tolist()),
                        "execution_authorized": False})
        previous_stamp = stamp
        previous_q = q_next
    if not targets and raw.get("failure"):
        rejects.append({"step_index": raw["failure"].get("step_index"),
                        "reasons": raw["failure"].get("reasons", []),
                        "retained_failing_candidate": bool(raw.get("retained_state"))})
    return {"schema": "task28.g2_p5_qp_3.offline_handoff.v1", "status": raw["status"],
            "execution_authorized": False, "identity": identity, "dt_s": float(dt_s),
            "source_scope": "SAVED_NR0_OBJECT_NODE_AND_QP2_FEEDBACK_OFFLINE",
            "timestamp_scope": "SYNTHETIC_NUMERICAL_TIME_NOT_ROS_OR_SERVO_TIME",
            "raw_qp2": raw, "position_targets": targets, "rejections": rejects,
            "target_count": len(targets) * 2, "failure_stop": bool(rejects)}
