"""Typed, deterministic offline input gate for the frozen Task28 fixture.

All stamps are synthetic seconds in one caller-provided clock domain. The age,
skew, and future tolerances below are a NEW offline gate, not recovered Task24
settings or measured ROS timing performance. No ROS or robot APIs are imported.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real
from typing import Literal


class AttachmentMode(str, Enum):
    DETACHED = "detached"
    BILATERAL = "bilateral"


@dataclass(frozen=True)
class JointObservation:
    side: Literal["left", "right"]
    frame_id: str
    stamp_s: float
    names: tuple[str, ...]
    positions_rad: tuple[float, ...]


@dataclass(frozen=True)
class PoseObservation:
    frame_id: str
    stamp_s: float
    position_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    object_id: str | None = None


@dataclass(frozen=True)
class InputState:
    left_joints: JointObservation
    right_joints: JointObservation
    left_tcp: PoseObservation
    right_tcp: PoseObservation
    cube: PoseObservation
    attachment_mode: AttachmentMode
    left_attached: bool
    right_attached: bool


@dataclass(frozen=True)
class StatePolicy:
    """NEW offline gate limits; callers may tighten but never relax them.

    Age/skew/norm tolerances must remain positive. A zero future tolerance is
    permitted. Quaternion values are never normalized.
    """

    max_age_s: float = 0.1
    max_skew_s: float = 0.02
    max_future_s: float = 0.001
    quaternion_norm_tolerance: float = 1e-6


@dataclass(frozen=True)
class StateValidationResult:
    valid: bool
    reasons: tuple[str, ...]


def expected_joint_names(side: Literal["left", "right"]) -> tuple[str, ...]:
    return tuple(f"{side}_fr3_joint{i}" for i in range(1, 8))


def _finite_number(value: object) -> bool:
    # bool is a Real in Python but is not a meaningful physical measurement.
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def _numeric_tuple(value: object, length: int) -> bool:
    return (
        isinstance(value, tuple)
        and len(value) == length
        and all(_finite_number(item) for item in value)
    )


def validate_state(
    state: InputState, now_s: float, policy: StatePolicy = StatePolicy()
) -> StateValidationResult:
    """Fail closed on malformed, stale, asynchronous, or incoherent input.

    Every observation uses ``world`` and all seven named joints must be present
    in fixture order. Timestamp skew spans BOTH joint observations, BOTH TCP
    observations, and the object observation. Joint limits, FK and SE(3) closure
    are separate checks performed after this input gate.

    Reason strings are deterministic machine-readable codes, optionally scoped
    with ``:<observation>``. This function returns rejection reasons instead of
    throwing for malformed caller input, including ``None`` observation fields.
    """

    reasons: list[str] = []
    if not isinstance(policy, StatePolicy):
        return StateValidationResult(False, ("INVALID_POLICY",))
    policy_values = (
        getattr(policy, "max_age_s", None),
        getattr(policy, "max_skew_s", None),
        getattr(policy, "max_future_s", None),
        getattr(policy, "quaternion_norm_tolerance", None),
    )
    if not all(_finite_number(value) and value >= 0 for value in policy_values):
        return StateValidationResult(False, ("INVALID_POLICY",))
    if not (0 < policy.max_age_s <= 0.1 and 0 < policy.max_skew_s <= 0.02
            and 0 <= policy.max_future_s <= 0.001
            and 0 < policy.quaternion_norm_tolerance <= 1e-6):
        return StateValidationResult(False, ("INVALID_POLICY",))
    if not _finite_number(now_s):
        return StateValidationResult(False, ("INVALID_NOW",))
    if not isinstance(state, InputState):
        return StateValidationResult(False, ("INVALID_STATE_TYPE",))

    stamps: list[float] = []

    def common_observation(observation: JointObservation | PoseObservation, label: str) -> None:
        frame = getattr(observation, "frame_id", None)
        if not isinstance(frame, str) or frame != "world":
            reasons.append(f"INCORRECT_FRAME:{label}")
        stamp = getattr(observation, "stamp_s", None)
        if not _finite_number(stamp):
            reasons.append(f"INVALID_TIMESTAMP:{label}")
            return
        stamps.append(float(stamp))
        if now_s - stamp > policy.max_age_s:
            reasons.append(f"STALE_OBSERVATION:{label}")
        if stamp - now_s > policy.max_future_s:
            reasons.append(f"FUTURE_OBSERVATION:{label}")

    for label, side in (("left_joints", "left"), ("right_joints", "right")):
        observation = getattr(state, label, None)
        if not isinstance(observation, JointObservation):
            reasons.append(f"MISSING_OR_INVALID_JOINT_OBSERVATION:{label}")
            continue
        common_observation(observation, label)
        arm_side = getattr(observation, "side", None)
        if not isinstance(arm_side, str) or arm_side != side:
            reasons.append(f"INCORRECT_ARM_SIDE:{label}")
        names = getattr(observation, "names", None)
        if (not isinstance(names, tuple) or len(names) != 7
                or not all(isinstance(name, str) for name in names)
                or names != expected_joint_names(side)):
            reasons.append(f"INCORRECT_JOINT_NAMES:{label}")
        if not _numeric_tuple(getattr(observation, "positions_rad", None), 7):
            reasons.append(f"INVALID_JOINT_VALUES:{label}")

    for label in ("left_tcp", "right_tcp", "cube"):
        observation = getattr(state, label, None)
        if not isinstance(observation, PoseObservation):
            reasons.append(f"MISSING_OR_INVALID_POSE_OBSERVATION:{label}")
            continue
        common_observation(observation, label)
        if not _numeric_tuple(getattr(observation, "position_m", None), 3):
            reasons.append(f"INVALID_POSITION:{label}")
        quaternion = getattr(observation, "quaternion_xyzw", None)
        if not _numeric_tuple(quaternion, 4):
            reasons.append(f"INVALID_QUATERNION:{label}")
        elif abs(math.hypot(*quaternion) - 1.0) > policy.quaternion_norm_tolerance:
            reasons.append(f"NONUNIT_QUATERNION:{label}")
        object_id = getattr(observation, "object_id", None)
        if label == "cube" and (not isinstance(object_id, str) or object_id != "Cube_01"):
            reasons.append("INCORRECT_CUBE_ID:cube")

    if stamps and max(stamps) - min(stamps) > policy.max_skew_s:
        reasons.append("ASYNCHRONOUS_OBSERVATIONS")

    attachment_mode = getattr(state, "attachment_mode", None)
    left_attached = getattr(state, "left_attached", None)
    right_attached = getattr(state, "right_attached", None)
    if not isinstance(attachment_mode, AttachmentMode):
        reasons.append("INVALID_ATTACHMENT_MODE")
    if type(left_attached) is not bool or type(right_attached) is not bool:
        reasons.append("INVALID_ATTACHMENT_FLAGS")
    elif isinstance(attachment_mode, AttachmentMode):
        expected_attached = attachment_mode is AttachmentMode.BILATERAL
        if (left_attached, right_attached) != (expected_attached, expected_attached):
            reasons.append("INCONSISTENT_ATTACHMENT_MODE")

    return StateValidationResult(not reasons, tuple(reasons))
