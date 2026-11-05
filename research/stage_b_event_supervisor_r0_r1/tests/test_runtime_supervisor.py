from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from isaac.scripts.task28_runtime_events import (  # noqa: E402
    EventClass,
    RuntimeSupervisor,
    SupervisorState,
)


def test_event_transitions(tmp_path):
    journal = tmp_path / "events.jsonl"
    sup = RuntimeSupervisor(journal)
    sup.emit("FREE_SPACE", EventClass.NORMAL_OPERATION,
             reason="valid long trajectory")
    sup.emit("SUSPECTED_STALL", EventClass.RECOVERABLE_TASK_EVENT,
             reason="sustained lag", next_state=SupervisorState.HOLD)
    assert sup.state is SupervisorState.HOLD
    sup.stop("FATAL_EXECUTION_ERROR", reason="nonfinite state", fatal=True)
    rows = [json.loads(line) for line in journal.read_text().splitlines()]
    assert rows[1]["classification"] == "RECOVERABLE_TASK_EVENT"
    assert rows[-1]["next_state"] == "STOPPED"


def test_exception_classification():
    assert RuntimeSupervisor.classify_exception(RuntimeError("QP_INFEASIBLE")) is EventClass.CONTROL_FEASIBILITY_EVENT
    assert RuntimeSupervisor.classify_exception(RuntimeError("CONTACT_OBSERVATION_STALE")) is EventClass.OBSERVATION_UNAVAILABLE
    assert RuntimeSupervisor.classify_exception(RuntimeError("PLACEMENT_NOT_SEATED")) is EventClass.RECOVERABLE_TASK_EVENT
    assert RuntimeSupervisor.classify_exception(RuntimeError("nonfinite robot state")) is EventClass.FATAL_EXECUTION_ERROR
