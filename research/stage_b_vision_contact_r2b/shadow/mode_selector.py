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

    @staticmethod
    def _skill(frame: Mapping[str, Any]) -> str | None:
        values = [frame.get(key) for key in ("skill_id", "current_skill", "executor_skill", "phase") if frame.get(key) is not None]
        normalized = {str(value).strip().lower() for value in values}
        if len(normalized) > 1:
            return None
        return next(iter(normalized), None)

    @staticmethod
    def _owner_valid(frame: Mapping[str, Any]) -> bool:
        owner = frame.get("controller_owner", frame.get("control_owner"))
        if owner is None:
            reference = frame.get("controller_reference") or {}
            owner = reference.get("control_owner")
        return bool(str(owner).strip()) if owner is not None else False

    @staticmethod
    def _observations_valid(frame: Mapping[str, Any], *, require_command: bool = False) -> bool:
        if frame.get("observation_valid") is False:
            return False
        health = frame.get("sensor_health") or {}
        required = ("camera", "q", "dq", "tcp") + (("command",) if require_command else ())
        for name in required:
            value = health.get(name)
            value = getattr(value, "value", value)
            if value is not None and str(value) != "PRESENT":
                return False
        if frame.get("observation_valid") is None and health and any(name not in health for name in required):
            return False
        return True

    @staticmethod
    def _attachment_valid(frame: Mapping[str, Any]) -> bool:
        if frame.get("attachment_valid") is False:
            return False
        attachment = frame.get("attachment_confidence")
        if attachment is None:
            return False
        values = [attachment.get(side) for side in ("left", "right") if isinstance(attachment, Mapping)]
        return len(values) == 2 and all(value is not None and float(value) >= 0.5 for value in values)

    def select(self, estimate: ContactEstimate, frame: Mapping[str, Any] | None = None) -> ShadowDecision:
        frame = frame or {}
        supervisor = frame.get("supervisor_mode")
        stale = estimate.state is ContactState.UNKNOWN or estimate.uncertainty >= 0.65
        skill = self._skill(frame)
        owner_valid = self._owner_valid(frame)
        contradictory_skill = any(frame.get(key) is not None for key in ("skill_id", "current_skill", "executor_skill", "phase")) and skill is None
        if stale:
            mode, reason = ShadowMode.HOLD_UNKNOWN, "visual_or_runtime_observation_is_stale_or_ambiguous"
        elif contradictory_skill:
            mode, reason = ShadowMode.HOLD_UNKNOWN, "contradictory_executor_skill_receipts"
        elif estimate.state is ContactState.DEEP_WALL_CONTACT_POSSIBLE and skill in ("single_arm_insertion", "deep_wall_arrival") and owner_valid and self._observations_valid(frame, require_command=True):
            mode, reason = ShadowMode.REQUEST_DEEP_WALL_CHECK, "deep_wall_contact_possible_requires_read_only_check"
        elif estimate.state is ContactState.SIDE_CONTACT_POSSIBLE:
            preflight = bool(frame.get("side_preflight_passed", False))
            if preflight and skill == "auxiliary_side_compaction" and owner_valid and self._observations_valid(frame):
                mode, reason = ShadowMode.SIDE_APPROACH_READY, "side_contact_possible_with_explicit_preflight_receipt"
            else:
                mode, reason = ShadowMode.HOLD_UNKNOWN, "side_skill_owner_observation_or_preflight_gate_missing"
        elif estimate.state in (ContactState.PUSH_PROGRESS, ContactState.SINGLE_ARM_PUSHING) and skill == "single_arm_insertion" and owner_valid and self._observations_valid(frame, require_command=True) and self._attachment_valid(frame):
            mode, reason = ShadowMode.CONTINUE_PUSH, "visual_progress_or_tool_proximity_supports_continuation"
        else:
            mode, reason = ShadowMode.HOLD_UNKNOWN, "skill_owner_observation_or_attachment_gate_missing"
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
