"""Read-only Stage-B shadow mode selector.

The selector proposes a transition and records why. It never writes a motion
command, changes controller ownership, or bypasses FCL/suction gates.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping

try:
    from ..estimator.stage_b_contact import ContactEstimate, ContactState
except ImportError:  # tests also import ``shadow`` as a top-level package
    from estimator.stage_b_contact import ContactEstimate, ContactState


class ShadowMode(str, Enum):
    CONTINUE_PUSH = "CONTINUE_PUSH"
    REQUEST_DEEP_WALL_CHECK = "REQUEST_DEEP_WALL_CHECK"
    SIDE_APPROACH_READY = "SIDE_APPROACH_READY"
    HOLD_UNKNOWN = "HOLD_UNKNOWN"


@dataclass(frozen=True)
class ShadowDecision:
    mode: ShadowMode
    confidence: float
    timestamp_s: float
    supporting_state: ContactState
    supporting_edges: Mapping[str, float]
    reason: str
    supervisor_mode: str | None
    differs_from_supervisor: bool
    rejected_for_stale_or_ambiguous_input: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode.value,
            "confidence": self.confidence,
            "timestamp_s": self.timestamp_s,
            "supporting_state": self.supporting_state.value,
            "supporting_edges": dict(self.supporting_edges),
            "reason": self.reason,
            "supervisor_mode": self.supervisor_mode,
            "differs_from_supervisor": self.differs_from_supervisor,
            "rejected_for_stale_or_ambiguous_input": self.rejected_for_stale_or_ambiguous_input,
        }


class ShadowModeSelector:
    """Conservative, read-only mode proposal policy."""

    def select(self, estimate: ContactEstimate, frame: Mapping[str, Any] | None = None) -> ShadowDecision:
        frame = frame or {}
        supervisor = frame.get("supervisor_mode")
        stale = estimate.state is ContactState.UNKNOWN or estimate.uncertainty >= 0.65
        if stale:
            mode, reason = ShadowMode.HOLD_UNKNOWN, "visual_or_runtime_observation_is_stale_or_ambiguous"
        elif estimate.state is ContactState.DEEP_WALL_CONTACT_POSSIBLE:
            mode, reason = ShadowMode.REQUEST_DEEP_WALL_CHECK, "deep_wall_contact_possible_requires_read_only_check"
        elif estimate.state is ContactState.SIDE_CONTACT_POSSIBLE:
            preflight = bool(frame.get("side_preflight_passed", False))
            if preflight:
                mode, reason = ShadowMode.SIDE_APPROACH_READY, "side_contact_possible_with_explicit_preflight_receipt"
            else:
                mode, reason = ShadowMode.HOLD_UNKNOWN, "side_preflight_receipt_missing_or_failed"
        elif estimate.state in (ContactState.PUSH_PROGRESS, ContactState.SINGLE_ARM_PUSHING):
            mode, reason = ShadowMode.CONTINUE_PUSH, "visual_progress_or_tool_proximity_supports_continuation"
        else:
            mode, reason = ShadowMode.HOLD_UNKNOWN, "state_does_not_support_a_safe_transition"
        return ShadowDecision(
            mode=mode,
            confidence=estimate.confidence if mode is not ShadowMode.HOLD_UNKNOWN else min(estimate.confidence, 0.35),
            timestamp_s=estimate.time_s,
            supporting_state=estimate.state,
            supporting_edges=estimate.edge_probabilities,
            reason=reason,
            supervisor_mode=None if supervisor is None else str(supervisor),
            differs_from_supervisor=supervisor is not None and str(supervisor) != mode.value,
            rejected_for_stale_or_ambiguous_input=stale or mode is ShadowMode.HOLD_UNKNOWN,
        )
