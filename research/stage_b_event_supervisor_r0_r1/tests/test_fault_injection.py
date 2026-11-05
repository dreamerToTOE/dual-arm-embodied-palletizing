from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from isaac.scripts.task28_runtime_events import EventClass, RuntimeSupervisor, SupervisorState  # noqa: E402


def make():
    return RuntimeSupervisor(Path('/tmp') / 'stage_b_event_supervisor_fault_injection.jsonl')


def test_A_legitimate_long_distance_motion():
    sup = make()
    event = sup.emit('LONG_FREE_SPACE', EventClass.NORMAL_OPERATION,
                     reason='trajectory is position-owned and collision checked')
    assert event.classification is EventClass.NORMAL_OPERATION


def test_B_tracking_divergence_stops_control():
    assert RuntimeSupervisor.classify_exception(RuntimeError('TRACKING_ERROR')) is EventClass.CONTROL_FEASIBILITY_EVENT


def test_C_cube_progress_anomaly_holds():
    sup = make()
    event = sup.emit('SUSPECTED_STALL_CONFIRMED', EventClass.RECOVERABLE_TASK_EVENT,
                     reason='two-window lag', next_state=SupervisorState.HOLD)
    assert event.next_state is SupervisorState.HOLD and event.recoverable


def test_D_deep_wall_arrival_is_normal():
    sup = make()
    event = sup.emit('WALL_SEATED', EventClass.NORMAL_OPERATION,
                     reason='deep wall target reached')
    assert event.classification is EventClass.NORMAL_OPERATION


def test_E_unexpected_collision_is_fatal():
    assert RuntimeSupervisor.classify_exception(RuntimeError('UNEXPECTED_COLLISION')) is EventClass.FATAL_EXECUTION_ERROR


def test_F_invalid_observation_is_not_executed():
    assert RuntimeSupervisor.classify_exception(RuntimeError('OBSERVATION_STALE')) is EventClass.OBSERVATION_UNAVAILABLE


def test_G_qp_infeasibility_is_control_failure():
    assert RuntimeSupervisor.classify_exception(RuntimeError('QP_INFEASIBLE')) is EventClass.CONTROL_FEASIBILITY_EVENT


def test_H_final_acceptance_failure_is_recoverable_task_event():
    assert RuntimeSupervisor.classify_exception(RuntimeError('PLACEMENT_NOT_SEATED')) is EventClass.RECOVERABLE_TASK_EVENT


def test_I_attachment_unavailable_blocks_assumed_recovery():
    assert RuntimeSupervisor.classify_exception(RuntimeError('ATTACHMENT_UNAVAILABLE')) is EventClass.OBSERVATION_UNAVAILABLE
