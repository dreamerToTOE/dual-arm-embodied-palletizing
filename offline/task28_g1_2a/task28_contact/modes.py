"""Typed offline contact modes and independent synthetic attachment metadata.

The intent is a latched planned/commanded desire. It is not feedback. Feedback
must be supplied independently by the caller; this module never manufactures it
from the intent. ``INDEPENDENT_SYNTHETIC`` identifies deterministic test evidence,
not a measured D6 attachment or real sensor observation.

All timestamps are synthetic seconds in the same clock domain as G1-1. The
unchanged G1-1 ``StatePolicy`` applies to the five observations and the independent
attachment feedback together. A latched intent may be old, but cannot be future
dated, and its corresponding feedback cannot predate it.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from numbers import Real

from task28_validator.state import (
    AttachmentMode,
    InputState,
    StatePolicy,
    StateValidationResult,
    validate_state,
)


class OperationMode(str, Enum):
    DETACHED_SUPPORTED = "detached_supported"
    BILATERAL_SUPPORTED = "bilateral_supported"
    # A geometrically separated bilateral snapshot, including lift/place
    # observations before temporal confirmation. The R1 confirmer determines
    # the operation phase separately; this enum never asserts physical support.
    BILATERAL_TRANSITIONING = "bilateral_transitioning"
    BILATERAL_AIRBORNE = "bilateral_airborne"
    RELEASED_SUPPORTED = "released_supported"


class SupportSite(str, Enum):
    SUPPLY = "supply"
    TARGET = "target"


class EvidenceSource(str, Enum):
    INDEPENDENT_SYNTHETIC = "independent_synthetic"
    COMMAND_ECHO = "command_echo"


class TransitionEvent(str, Enum):
    ATTACH = "attach"
    LIFT = "lift"
    TRANSPORT_SNAPSHOT = "transport_snapshot"
    PLACE = "place"
    RELEASE = "release"


@dataclass(frozen=True)
class AttachmentIntent:
    request_id: str
    issued_at_s: float
    left: bool
    right: bool


@dataclass(frozen=True)
class ObservedAttachment:
    evidence_id: str
    stamp_s: float
    request_id: str
    left: bool
    right: bool
    source: EvidenceSource = EvidenceSource.INDEPENDENT_SYNTHETIC


@dataclass(frozen=True)
class Snapshot:
    input_state: InputState
    mode: OperationMode
    support_site: SupportSite | None
    intent: AttachmentIntent
    observed: ObservedAttachment


def _finite(value: object) -> bool:
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        return False


def _identifier(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


def validate_snapshot_metadata(
    snapshot: Snapshot, now_s: float, policy: StatePolicy = StatePolicy()
) -> StateValidationResult:
    """Fail closed on mode/intent/feedback inconsistency and stale metadata.

    This gate checks types, timestamps, identity and attachment metadata only.
    Joint limits, FK, full bilateral SE(3) closure, support geometry and collision
    checks belong to the caller. In particular, detached states do not acquire
    a bilateral closed-chain requirement through this function.

    Rejection codes have deterministic order: policy/clock, G1-1 input-state
    reasons, mode/site, intent, feedback and combined observation skew.
    """

    # Use the original public gate to enforce exactly its non-relaxable policy
    # and clock validation, without duplicating those numeric policy bounds.
    probe = validate_state(None, now_s, policy)
    if probe.reasons != ("INVALID_STATE_TYPE",):
        return probe
    if type(snapshot) is not Snapshot:
        return StateValidationResult(False, ("INVALID_SNAPSHOT_TYPE",))

    state = snapshot.input_state
    reasons = list(validate_state(state, now_s, policy).reasons)
    mode = snapshot.mode
    mode_valid = type(mode) is OperationMode
    if not mode_valid:
        reasons.append("INVALID_OPERATION_MODE")
    site = snapshot.support_site
    if site is not None and type(site) is not SupportSite:
        reasons.append("INVALID_SUPPORT_SITE")
    elif mode_valid:
        expected_site = {
            OperationMode.DETACHED_SUPPORTED: SupportSite.SUPPLY,
            OperationMode.RELEASED_SUPPORTED: SupportSite.TARGET,
        }.get(mode)
        if mode in (OperationMode.BILATERAL_TRANSITIONING, OperationMode.BILATERAL_AIRBORNE):
            if site is not None:
                reasons.append("AIRBORNE_SUPPORT_SITE_PRESENT")
        elif mode is OperationMode.BILATERAL_SUPPORTED:
            if site is None:
                reasons.append("MISSING_SUPPORT_SITE")
        elif site is not expected_site:
            reasons.append("INCORRECT_SUPPORT_SITE")

    expected_attached = (
        mode in (OperationMode.BILATERAL_SUPPORTED, OperationMode.BILATERAL_TRANSITIONING,
                 OperationMode.BILATERAL_AIRBORNE)
        if mode_valid else None
    )
    if isinstance(state, InputState) and expected_attached is not None:
        expected_old_mode = AttachmentMode.BILATERAL if expected_attached else AttachmentMode.DETACHED
        if isinstance(state.attachment_mode, AttachmentMode) and state.attachment_mode is not expected_old_mode:
            reasons.append("MODE_ATTACHMENT_MISMATCH")
        if (type(state.left_attached) is bool and type(state.right_attached) is bool
                and (state.left_attached, state.right_attached) != (expected_attached, expected_attached)):
            reasons.append("MODE_ATTACHMENT_FLAGS_MISMATCH")

    intent = snapshot.intent
    intent_valid = type(intent) is AttachmentIntent
    intent_stamp_valid = False
    if not intent_valid:
        reasons.append("MISSING_OR_INVALID_ATTACHMENT_INTENT")
    else:
        if not _identifier(intent.request_id):
            reasons.append("INVALID_INTENT_REQUEST_ID")
        intent_stamp_valid = _finite(intent.issued_at_s)
        if not intent_stamp_valid:
            reasons.append("INVALID_INTENT_TIMESTAMP")
        elif intent.issued_at_s - now_s > policy.max_future_s:
            reasons.append("FUTURE_ATTACHMENT_INTENT")
        if type(intent.left) is not bool or type(intent.right) is not bool:
            reasons.append("INVALID_INTENT_FLAGS")
        elif expected_attached is not None and (intent.left, intent.right) != (expected_attached, expected_attached):
            reasons.append("INTENT_MODE_MISMATCH")

    observed = snapshot.observed
    feedback_stamp_valid = False
    if type(observed) is not ObservedAttachment:
        reasons.append("MISSING_OR_INVALID_ATTACHMENT_FEEDBACK")
    else:
        if not _identifier(observed.evidence_id):
            reasons.append("INVALID_FEEDBACK_EVIDENCE_ID")
        if not _identifier(observed.request_id):
            reasons.append("INVALID_FEEDBACK_REQUEST_ID")
        elif intent_valid and _identifier(intent.request_id) and observed.request_id != intent.request_id:
            reasons.append("FEEDBACK_REQUEST_MISMATCH")
        if type(observed.source) is not EvidenceSource:
            reasons.append("INVALID_FEEDBACK_SOURCE")
        elif observed.source is not EvidenceSource.INDEPENDENT_SYNTHETIC:
            reasons.append("ATTACHMENT_COMMAND_ECHO_NOT_FEEDBACK")
        feedback_stamp_valid = _finite(observed.stamp_s)
        if not feedback_stamp_valid:
            reasons.append("INVALID_FEEDBACK_TIMESTAMP")
        else:
            if now_s - observed.stamp_s > policy.max_age_s:
                reasons.append("STALE_ATTACHMENT_FEEDBACK")
            if observed.stamp_s - now_s > policy.max_future_s:
                reasons.append("FUTURE_ATTACHMENT_FEEDBACK")
            if intent_stamp_valid and observed.stamp_s < intent.issued_at_s:
                reasons.append("FEEDBACK_PREDATES_INTENT")
        if type(observed.left) is not bool or type(observed.right) is not bool:
            reasons.append("INVALID_FEEDBACK_FLAGS")
        elif expected_attached is not None and (observed.left, observed.right) != (expected_attached, expected_attached):
            reasons.append("FEEDBACK_MODE_MISMATCH")
        if (intent_valid and type(intent.left) is bool and type(intent.right) is bool
                and type(observed.left) is bool and type(observed.right) is bool
                and (observed.left, observed.right) != (intent.left, intent.right)):
            reasons.append("INTENT_FEEDBACK_MISMATCH")

    if feedback_stamp_valid and isinstance(state, InputState):
        # Include all five original observations, not only the TCP timestamps.
        stamps = [float(observed.stamp_s)]
        for label in ("left_joints", "right_joints", "left_tcp", "right_tcp", "cube"):
            stamp = getattr(getattr(state, label, None), "stamp_s", None)
            if _finite(stamp):
                stamps.append(float(stamp))
        if max(stamps) - min(stamps) > policy.max_skew_s:
            reasons.append("ASYNCHRONOUS_ATTACHMENT_FEEDBACK")
    return StateValidationResult(not reasons, tuple(reasons))
