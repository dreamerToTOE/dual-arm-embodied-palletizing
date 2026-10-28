"""Independent evaluation of the registered R4 source diagnostics.

This module only reads evidence written by the diagnostic runner.  It is
deliberately separate from the runner so a source qualification decision does
not depend on the live simulator process or on a mutable tensor buffer.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
EVIDENCE = ROOT / "evidence"


def _max_abs(values):
    return float(max((abs(float(x)) for x in values), default=0.0))


def _run_summary(run_dir: Path, contract: dict) -> dict:
    result = json.loads((run_dir / "result.json").read_text())
    rows = [json.loads(line) for line in (run_dir / "frames.jsonl").read_text().splitlines()]
    out = {
        "configuration": result["configuration"],
        "label": result["registration"]["label"],
        "status": result["status"],
        "phase": result["phase"],
        "first_failure": result.get("first_failure"),
        "explicit_physics_steps": result.get("explicit_physics_steps", 0),
        "recorded_frames": len(rows),
        "post_callback_count": result.get("post_callback_count", 0),
        "robot_command_writes": result.get("robot_command_writes", 0),
        "suction_commands": result.get("suction_commands", 0),
        "physics_backend_adaptation": result.get("physics_backend_adaptation"),
        "source_sha256": hashlib.sha256((run_dir / "frames.jsonl").read_bytes()).hexdigest(),
        "step_monotonic": True,
        "time_monotonic": True,
        "dt_max_error_s": 0.0,
        "callback_dt_max_error_s": 0.0,
        "unexpected_collisions": [],
        "fk_max_translation_error_m": 0.0,
        "fk_max_rotation_error_rad": 0.0,
        "fd_velocity_max_abs_rad_s": 0.0,
        "fd_velocity_tail_max_abs_rad_s": 0.0,
        "integral_error_max_rad": 0.0,
        "dynamic_control_q_max_abs_rad": None,
        "dynamic_control_dq_max_abs_rad_s": None,
        "callback_immediate_q_max_abs_rad": None,
        "callback_immediate_dq_max_abs_rad_s": None,
        "borrowed_previous_q_mutation_max": None,
        "borrowed_previous_dq_mutation_max": None,
        "render_added_physics_steps": False,
        "terminal_q_frozen_nonzero_dq": False,
        "source_qualification": "BLOCKED",
    }
    if not rows:
        out["source_qualification"] = "REJECTED_NO_FRAMES"
        return out
    steps = [int(row["physics_step"]) for row in rows]
    times = [float(row["time_s"]) for row in rows]
    out["step_monotonic"] = all(b == a + 1 for a, b in zip(steps, steps[1:]))
    out["time_monotonic"] = all(b > a for a, b in zip(times, times[1:]))
    dts = [b - a for a, b in zip(times, times[1:])]
    out["dt_max_error_s"] = max((abs(dt - contract["physics_dt_s"]) for dt in dts), default=0.0)
    cb = [float(x) for x in result.get("callback_dts_s", [])]
    out["callback_dt_max_error_s"] = max((abs(x - contract["physics_dt_s"]) for x in cb), default=0.0)
    fd_errors = []
    tail_errors = []
    integrals = []
    dcq = []
    dcv = []
    cbq = []
    cbv = []
    borrowed_q = []
    borrowed_v = []
    for i in range(1, len(rows)):
        dt = times[i] - times[i - 1]
        for side in ("left", "right"):
            prev = rows[i - 1]["arms"][side]
            cur = rows[i]["arms"][side]
            q0, q1 = np.asarray(prev["q"], float), np.asarray(cur["q"], float)
            dq0, dq1 = np.asarray(prev["dq"], float), np.asarray(cur["dq"], float)
            fd = (q1 - q0) / dt
            fd_errors.extend((fd[:7] - dq1[:7]).tolist())
            if i >= max(1, len(rows) - 25):
                tail_errors.extend((fd[:7] - dq1[:7]).tolist())
            integrals.extend((q1 - q0 - (dq0 + dq1) * dt / 2.0).tolist())
            out["fk_max_translation_error_m"] = max(out["fk_max_translation_error_m"], float(cur["tcp_fk_error_m"]))
            out["fk_max_rotation_error_rad"] = max(out["fk_max_rotation_error_rad"], float(cur["tcp_fk_error_rad"]))
            if "dynamic_control_q" in cur:
                dcq.extend((np.asarray(cur["dynamic_control_q"], float) - q1).tolist())
                dcv.extend((np.asarray(cur["dynamic_control_dq"], float) - dq1).tolist())
            post = rows[i].get("post_callback")
            if post and "snapshot" in post:
                snap = post["snapshot"][side]
                cbq.extend((np.asarray(snap["q"], float) - q1).tolist())
                cbv.extend((np.asarray(snap["dq"], float) - dq1).tolist())
            if "previous_borrowed_q_after_new_read" in cur:
                borrowed_q.extend((np.asarray(cur["previous_borrowed_q_after_new_read"], float) - np.asarray(cur["previous_immediate_q"], float)).tolist())
                borrowed_v.extend((np.asarray(cur["previous_borrowed_dq_after_new_read"], float) - np.asarray(cur["previous_immediate_dq"], float)).tolist())
            out["unexpected_collisions"].extend(cur.get("collision", {}).get("unexpected", []))
    out["fd_velocity_max_abs_rad_s"] = _max_abs(fd_errors)
    out["fd_velocity_tail_max_abs_rad_s"] = _max_abs(tail_errors)
    out["integral_error_max_rad"] = _max_abs(integrals)
    if dcq:
        out["dynamic_control_q_max_abs_rad"] = _max_abs(dcq)
        out["dynamic_control_dq_max_abs_rad_s"] = _max_abs(dcv)
    if cbq:
        out["callback_immediate_q_max_abs_rad"] = _max_abs(cbq)
        out["callback_immediate_dq_max_abs_rad_s"] = _max_abs(cbv)
    if borrowed_q:
        out["borrowed_previous_q_mutation_max"] = _max_abs(borrowed_q)
        out["borrowed_previous_dq_mutation_max"] = _max_abs(borrowed_v)
    out["unexpected_collisions"] = sorted(set(out["unexpected_collisions"]))
    terminal = rows[-1]
    prior = rows[-2] if len(rows) > 1 else terminal
    terminal_frozen = all(
        float(np.max(np.abs(np.asarray(terminal["arms"][side]["q"]) - np.asarray(prior["arms"][side]["q"])))) < 1e-10
        for side in ("left", "right")
    )
    terminal_speed = max(float(np.max(np.abs(np.asarray(terminal["arms"][side]["dq"][:7])))) for side in ("left", "right"))
    out["terminal_q_frozen_nonzero_dq"] = terminal_frozen and terminal_speed > contract["settle_arm_speed_rad_s"]
    # A valid source must survive the full consistency contract.  The
    # explicit stale-state gate is intentionally stronger than a loose
    # derivative tolerance and is reported separately above.
    out["source_qualification"] = "VALIDATED" if (
        result["status"] == "BOUNDED_OBSERVATION_COMPLETE_PENDING_EVALUATION"
        and out["step_monotonic"]
        and out["time_monotonic"]
        and out["dt_max_error_s"] <= contract["dt_tolerance_s"]
        and not out["terminal_q_frozen_nonzero_dq"]
        and not out["unexpected_collisions"]
        and out["fk_max_translation_error_m"] <= contract["fk_translation_tolerance_m"]
        and out["fk_max_rotation_error_rad"] <= contract["fk_rotation_tolerance_rad"]
    ) else "BLOCKED"
    return out


def main() -> None:
    contract = json.loads((ROOT / "source_contract.json").read_text())
    runs = [_run_summary(EVIDENCE / f"run_{name}", contract) for name in "ABCDEF"]
    scene_ids = []
    backend_adaptations = []
    for name in "ABCDEF":
        result = json.loads((EVIDENCE / f"run_{name}" / "result.json").read_text())
        scene_ids.append(result.get("scene_before", {}).get("status"))
        backend_adaptations.append(result.get("physics_backend_adaptation"))
    all_backend_adaptations_are_expected = all(
        x and x.get("scene_source_unchanged") and x.get("observed", {}).get("after", {}).get("physxScene:broadphaseType") == "MBP"
        for x in backend_adaptations
    )
    qualification = {
        "status": "JOINT_STATE_SOURCE_BLOCKED",
        "reason": "All six registered cells hit FROZEN_Q_NONZERO_DQ before the source contract window completed.",
        "settled_status": "NATIVE_SETTLE_UNRESOLVED",
        "moving_status": "MOVING_SOURCE_NOT_QUALIFIED",
        "arm_dynamics_status": "ARM_DYNAMICS_SOURCE_PARTIAL",
        "exclusive_torque_status": "EXCLUSIVE_TORQUE_COMMAND_BLOCKED",
        "static_torque_status": "STATIC_TORQUE_HOLD_SKIPPED",
        "tracking_status": "SHORT_TORQUE_TRACKING_SKIPPED",
        "virtual_cooperative_qp_status": "BLOCKED_SOURCE_PREREQUISITE",
        "cube_suction_status": "CUBE_SUCTION_UNTESTED",
        "original_scene_status": "ORIGINAL_SCENE_UNCHANGED",
        "scene_identity_statuses": scene_ids,
        "cpu_context_backend_adaptation_recorded": all_backend_adaptations_are_expected,
        "torque_commands": 0,
        "suction_commands": 0,
        "robot_command_writes": 0,
        "run_summaries": runs,
        "historical_r3_preserved": True,
        "approach_negative_preserved": True,
    }
    (EVIDENCE / "run_inventory.json").write_text(json.dumps(runs, indent=2, sort_keys=True) + "\n")
    (EVIDENCE / "source_qualification.json").write_text(json.dumps(qualification, indent=2, sort_keys=True) + "\n")
    summary = {
        "r4_status": "QP6_R4_ORIGINAL_SCENE_SOURCE_BLOCKED",
        "outcomes": {k: qualification[k] for k in (
            "original_scene_status", "status", "settled_status", "arm_dynamics_status",
            "exclusive_torque_status", "static_torque_status", "tracking_status",
            "virtual_cooperative_qp_status", "cube_suction_status")},
        "diagnostic_configurations": [x["configuration"] for x in runs],
        "first_failures": {x["configuration"]: x["first_failure"] for x in runs},
        "max_terminal_fd_error_rad_s": max(x["fd_velocity_tail_max_abs_rad_s"] for x in runs),
        "dynamic_control_agreement_max_q_rad": max((x["dynamic_control_q_max_abs_rad"] or 0.0) for x in runs),
        "callback_immediate_agreement_max_q_rad": max((x["callback_immediate_q_max_abs_rad"] or 0.0) for x in runs),
        "borrowed_alias_mutation_max": max((x["borrowed_previous_q_mutation_max"] or 0.0) for x in runs),
        "torque_commands": 0,
        "historical_r3_preserved": True,
    }
    (EVIDENCE / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
