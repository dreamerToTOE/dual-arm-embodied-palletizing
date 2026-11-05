from gzip import open as gzip_open
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "research/stage_b_event_supervisor_r0_r1/evidence"


def test_r3_position_drive_completed():
    summary = json.loads((EVIDENCE / "r3_position_precontact03/summary.json").read_text())
    assert summary["status"] == "COMPLETED"
    assert summary["executor_exit"] == 0
    assert summary["control_architecture"] == "FREE_SPACE_TRAJECTORY_POSITION_DRIVE"
    assert summary["live_qp"] is False


def test_r4_contact_receipts():
    summary = json.loads((EVIDENCE / "r4_physical_push02/physical_trace_summary.json").read_text())
    assert summary["contact_source"] == "PhysX_simulation_contact_report"
    assert summary["tool_cube_contact_events"] > 0
    assert summary["deep_wall_contact_events"] > 0
    assert summary["cube_displacement_m"] > 0.1
    seen_tool = seen_wall = False
    with gzip_open(EVIDENCE / "r4_physical_push02/contact_events.jsonl.gz", "rt", encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            paths = row["colliders"]
            seen_tool |= any("side_suction_tool" in p for p in paths)
            seen_wall |= any("WallDeep" in p for p in paths)
            if seen_tool and seen_wall:
                break
    assert seen_tool and seen_wall


def test_preserved_limits_and_temporal_stall_gate():
    source = (ROOT / "ros_ws/src/fr3_dual_palletize/src/task26_truck_box_push_in.cpp").read_text()
    assert "constexpr double kPushJamTolerance = 0.020;" in source
    assert "suspected_stall_slices >= 2" in source
    assert "PUSH_CUBE_OBSERVATION_STALE" in source
    assert "WALL_SEATED" in source
    assert "WALL_NOT_SEATED" in source
    torque = (ROOT / "isaac/scripts/task28_task27_torque_bridge.py").read_text()
    assert "self._qp_sweep_limit_m = 0.020" in torque


def test_new_r1_event_supervisor_run_completed():
    root = EVIDENCE / "r1_event_supervisor_physical_push01"
    summary = json.loads((root / "summary.json").read_text())
    trace = json.loads((root / "physical_trace/physical_trace_summary.json").read_text())
    assert summary["status"] == "COMPLETED"
    assert summary["executor_exit"] == 0 and summary["moveit_exit"] == 0
    assert summary["control_architecture"] == "FREE_SPACE_TRAJECTORY_POSITION_DRIVE"
    assert trace["tool_cube_contact_events"] > 0
    assert trace["deep_wall_contact_events"] > 0
    assert trace["cube_displacement_m"] > 0.1
    events = [json.loads(line) for line in (root / "runtime_events.jsonl").read_text().splitlines()]
    assert [row["event"] for row in events] == ["RUN_STARTED", "FREE_SPACE_POSITION_DRIVE", "TRIAL_COMPLETED"]
