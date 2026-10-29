"""Independent R6 evaluation; never issues simulation commands."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def load(cell):
    root = HERE / "evidence" / ("run_" + cell)
    result = json.loads((root / "result.json").read_text())
    frames = [json.loads(line) for line in (root / "frames.jsonl").read_text().splitlines()]
    return result, frames


def main():
    cells = {}
    for cell in "ABCD":
        try:
            cells[cell] = load(cell)
        except FileNotFoundError:
            cells[cell] = ({"status": "MISSING"}, [])
    complete = {c: r for c, (r, _) in cells.items() if r.get("status") == "DIAGNOSTIC_ACTUATION_COMPLETE"}
    per_cell = {}
    for cell, (result, frames) in cells.items():
        commands = int(result.get("actual_effort_commands", 0))
        readback = True
        collisions = []
        response = {}
        residuals = {}
        for row in frames:
            collisions.extend(row.get("collision", {}).get("unexpected", []))
            for side in ("left", "right"):
                arm = row.get("arms", {}).get(side, {})
                if arm and arm.get("requested_full_effort") is not None and not arm.get("applied_readback_match", False):
                    readback = False
        if frames:
            first = next((f for f in frames if f.get("arms", {}).get("left", {}).get("position_drives_disabled")), None)
            last = frames[-1]
            if first:
                for side in ("left", "right"):
                    q0 = np.asarray(first["arms"][side]["q"][:7], float)
                    q1 = np.asarray(last["arms"][side]["q"][:7], float)
                    response[side] = {"delta_q_arm": (q1 - q0).tolist(), "joint3_delta_rad": float(q1[3] - q0[3]), "tcp_delta_m": float(np.linalg.norm(np.asarray(last["arms"][side]["tcp_world"])[:3, 3] - np.asarray(first["arms"][side]["tcp_world"])[:3, 3]))}
                    residual_rows = []
                    for i in range(1, len(frames)):
                        a0 = frames[i - 1].get("arms", {}).get(side)
                        a1 = frames[i].get("arms", {}).get(side)
                        if not a0 or not a1 or a1.get("requested_full_effort") is None:
                            continue
                        dt = float(frames[i]["time_s"] - frames[i - 1]["time_s"])
                        if dt <= 0:
                            continue
                        # Reported dq is the end-step solver velocity.  This
                        # derivative is a diagnostic estimator only.
                        qdd = (np.asarray(a1["dq"], float) - np.asarray(a0["dq"], float)) / dt
                        tau = np.asarray(a1["requested_full_effort"], float)
                        pred = np.asarray(a1["M"], float) @ qdd + np.asarray(a1["c"], float) + np.asarray(a1["g"], float)
                        residual_rows.append(float(np.linalg.norm((pred - tau)[:7])))
                    residuals[side] = {"count": len(residual_rows), "max_arm_norm_Nm": max(residual_rows) if residual_rows else None, "median_arm_norm_Nm": float(np.median(residual_rows)) if residual_rows else None, "meaning": "solver-dq derivative estimator; not a calibrated torque residual"}
        per_cell[cell] = {"status": result.get("status"), "actual_effort_commands": commands, "robot_command_writes": result.get("robot_command_writes", 0), "readback_all_match": readback, "unexpected_collisions": collisions, "response": response, "model_residual": residuals, "first_failure": result.get("first_failure")}

    signed = {}
    for cell, perturb in (("B", 0.25), ("C", -0.25), ("D", 0.25)):
        vals = per_cell[cell].get("response", {})
        signed[cell] = {side: float(vals.get(side, {}).get("joint3_delta_rad", 0.0)) for side in ("left", "right")}
    sign_pass = all((signed["B"][s] > 0 and signed["C"][s] < 0 and signed["D"][s] > 0) for s in ("left", "right"))
    repeat_error = max(abs(signed["B"][s] - signed["D"][s]) for s in ("left", "right")) if complete.keys() >= {"B", "D"} else None
    level1 = all(per_cell[c]["status"] == "DIAGNOSTIC_ACTUATION_COMPLETE" and not per_cell[c]["unexpected_collisions"] for c in "ABCD")
    actual = sum(per_cell[c]["actual_effort_commands"] for c in "ABCD") > 0 and all(per_cell[c]["readback_all_match"] for c in "ABCD" if per_cell[c]["actual_effort_commands"])
    bounded_response = bool(level1 and actual and sign_pass and (repeat_error is None or repeat_error < 1e-2))
    # A non-small residual is deliberately retained as a limitation; it does
    # not silently promote the mounted-tool model to formal controller scope.
    residual_max = max((x["max_arm_norm_Nm"] for c in per_cell.values() for x in c["model_residual"].values() if x["max_arm_norm_Nm"] is not None), default=None)
    result = {
        "status": "R6_EVALUATED",
        "outcomes": {
            "ORIGINAL_SCENE_PRESERVED": "PASS" if level1 else "PARTIAL",
            "DIAGNOSTIC_ACTUATION_SOURCE_READY": "PASS" if level1 else "BLOCKED",
            "ACTUAL_EFFORT_COMMAND_EXECUTED": "PASS" if actual else "BLOCKED",
            "BOUNDED_TORQUE_RESPONSE_VALIDATED": "PASS" if bounded_response else "BLOCKED",
            "ARM_DYNAMICS_STATUS": "PARTIAL_MEASURED_RESPONSE_MODEL_RESIDUAL_NOT_CERTIFIED" if bounded_response else "BLOCKED",
            "STATIC_TORQUE_HOLD_STATUS": "SKIPPED_MODEL_BASED_PREREQUISITE",
            "SHORT_TRACKING_STATUS": "SKIPPED_STATIC_HOLD_PREREQUISITE",
            "VIRTUAL_COOPERATIVE_QP_STATUS": "SKIPPED_R6_SCOPE",
            "CUBE_D6_NOT_TESTED": True
        },
        "signed_response_joint3_rad": signed,
        "sign_response_pass": sign_pass,
        "repeat_error_max_rad": repeat_error,
        "max_model_residual_estimator_Nm": residual_max,
        "per_cell": per_cell,
        "limitations": ["applied readback is command buffer, not torque sensor", "M/c/g include native finger coordinates and do not identify L-tool inertia", "end-step dq derivative is an estimator under TGS", "no static hold/tracking/Cube contact/hardware"],
    }
    write = HERE / "evidence" / "evaluation.json"
    write.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

