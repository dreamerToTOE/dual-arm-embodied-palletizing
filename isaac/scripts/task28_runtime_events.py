"""Small typed event interface for bounded Stage B runtime supervision.

The classifier is deliberately independent of Isaac/ROS so synthetic tests can
exercise the transition policy without a simulator.  It never converts an
unknown observation into contact and it never turns control faults into normal
operation.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path
import time
from typing import Any, Mapping


class EventClass(str, Enum):
    NORMAL_OPERATION = "NORMAL_OPERATION"
    RECOVERABLE_TASK_EVENT = "RECOVERABLE_TASK_EVENT"
    CONTROL_FEASIBILITY_EVENT = "CONTROL_FEASIBILITY_EVENT"
    OBSERVATION_UNAVAILABLE = "OBSERVATION_UNAVAILABLE"
    FATAL_EXECUTION_ERROR = "FATAL_EXECUTION_ERROR"


class SupervisorState(str, Enum):
    FREE_SPACE = "FREE_SPACE"
    TOOL_CONTACT = "TOOL_CONTACT"
    INSERTION = "INSERTION"
    SIDE_COMPACTION = "SIDE_COMPACTION"
    HOLD = "HOLD"
    RECOVERY = "RECOVERY"
    STOPPED = "STOPPED"


@dataclass(frozen=True)
class RuntimeEvent:
    name: str
    classification: EventClass
    state: SupervisorState
    next_state: SupervisorState
    reason: str
    recoverable: bool
    details: Mapping[str, Any] = field(default_factory=dict)
    time_ns: int = 0

    def as_dict(self) -> dict[str, Any]:
        return {
            "event": self.name,
            "classification": self.classification.value,
            "state": self.state.value,
            "next_state": self.next_state.value,
            "reason": self.reason,
            "recoverable": self.recoverable,
            "details": dict(self.details),
            "time_ns": self.time_ns,
        }


class RuntimeSupervisor:
    """Event journal and explicit phase transitions for one bounded trial."""

    def __init__(self, path: str | Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.state = SupervisorState.FREE_SPACE
        self.events: list[RuntimeEvent] = []

    def emit(self, name: str, classification: EventClass, *, reason: str,
             next_state: SupervisorState | None = None,
             recoverable: bool | None = None,
             details: Mapping[str, Any] | None = None) -> RuntimeEvent:
        if next_state is None:
            next_state = self.state
        if recoverable is None:
            recoverable = classification in {
                EventClass.NORMAL_OPERATION,
                EventClass.RECOVERABLE_TASK_EVENT,
            }
        event = RuntimeEvent(
            name=name,
            classification=classification,
            state=self.state,
            next_state=next_state,
            reason=reason,
            recoverable=bool(recoverable),
            details=details or {},
            time_ns=time.time_ns(),
        )
        self.events.append(event)
        self.state = next_state
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(event.as_dict(), ensure_ascii=False,
                                    allow_nan=False, sort_keys=True) + "\n")
        return event

    def controlled_hold(self, name: str, *, reason: str, details=None) -> RuntimeEvent:
        return self.emit(name, EventClass.CONTROL_FEASIBILITY_EVENT,
                         reason=reason, next_state=SupervisorState.HOLD,
                         recoverable=True, details=details)

    def stop(self, name: str, *, reason: str, fatal: bool, details=None) -> RuntimeEvent:
        return self.emit(name,
                         EventClass.FATAL_EXECUTION_ERROR if fatal else EventClass.CONTROL_FEASIBILITY_EVENT,
                         reason=reason, next_state=SupervisorState.STOPPED,
                         recoverable=False, details=details)

    @staticmethod
    def classify_exception(exc: BaseException) -> EventClass:
        text = str(exc).upper()
        if any(token in text for token in ("STALE", "MISSING", "OBSERVATION", "CONTACT_UNCERTAIN", "ATTACHMENT")):
            return EventClass.OBSERVATION_UNAVAILABLE
        if any(token in text for token in ("QP_", "TRACKING", "SATURATION", "HANDOFF", "SWEEP_LIMIT", "TORQUE", "VELOCITY")):
            return EventClass.CONTROL_FEASIBILITY_EVENT
        if any(token in text for token in ("PLACEMENT", "JAM", "BLOCK", "ATTACHMENT")):
            return EventClass.RECOVERABLE_TASK_EVENT
        return EventClass.FATAL_EXECUTION_ERROR
