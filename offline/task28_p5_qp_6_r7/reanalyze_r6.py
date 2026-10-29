"""Independent offline reproduction of every R6 commanded interval.

No runtime imports or physics calls.  Post-step solver velocity derivatives are
estimators, not independently measured acceleration or calibrated torque data.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
R6 = HERE.parent / "task28_p5_qp_6_r6" / "evidence"
SIDES = ("left", "right")
HISTORICAL_RESIDUAL = 0.05492453116950269
RESIDUAL_DEFINITION = (
    "norm2((M_post @ ((dq_post-dq_pre)/(time_post-time_pre)) "
    "+ c_post + g_post - tau_requested)[:7]); includes first commanded interval"
)


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def load_run(directory):
    return json.loads((directory / "result.json").read_text()), read_jsonl(directory / "frames.jsonl")


def load(cell):
    return load_run(R6 / f"run_{cell}")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    return {"count": int(a.size), "max_abs": float(np.max(np.abs(a))),
            "mean": float(np.mean(a)), "median": float(np.median(a)),
            "rms": float(np.sqrt(np.mean(a * a)))} if a.size else {"count": 0}


def vector_stats(values):
    a = np.asarray(values, dtype=float)
    return {"samples": len(a), "max_abs_by_joint": np.max(np.abs(a), axis=0).tolist(),
            "rms_by_joint": np.sqrt(np.mean(a * a, axis=0)).tolist(),
            "max_norm": float(np.max(np.linalg.norm(a, axis=1)))} if a.size else {"samples": 0}


def so3_angle(first, second):
    r = np.asarray(first)[:3, :3].T @ np.asarray(second)[:3, :3]
    return float(np.arccos(np.clip((np.trace(r) - 1) / 2, -1., 1.)))


def commanded_intervals(frames, side):
    """First command uses preceding ownership-transfer row even without command."""
    rows = []
    for i in range(1, len(frames)):
        pre, post = frames[i - 1], frames[i]
        a, b = pre.get("arms", {}).get(side), post.get("arms", {}).get(side)
        if not a or not b or b.get("requested_full_effort") is None:
            continue
        dt = float(post["time_s"] - pre["time_s"])
        if dt <= 0 or int(post["physics_step"]) != int(pre["physics_step"]) + 1:
            raise ValueError("Invalid commanded interval timestamp/step")
        for row, arm in ((pre, a), (post, b)):
            if arm["physics_step"] != row["physics_step"] or arm["time_s"] != row["time_s"]:
                raise ValueError("Arm/state timestamps disagree")
        rows.append((pre, post, a, b, dt))
    return rows


def historical_residuals(frames, side):
    rows = []
    for pre, post, a, b, dt in commanded_intervals(frames, side):
        qdd = (np.asarray(b["dq"]) - a["dq"]) / dt
        residual = np.asarray(b["M"]) @ qdd + b["c"] + np.asarray(b["g"]) - b["requested_full_effort"]
        rows.append({"pre_step": pre["physics_step"], "post_step": post["physics_step"],
                     "dt_s": dt, "arm_vector_Nm": residual[:7].tolist(),
                     "arm_norm_Nm": float(np.linalg.norm(residual[:7]))})
    return {"definition": RESIDUAL_DEFINITION, "count": len(rows),
            "max": max((r["arm_norm_Nm"] for r in rows), default=None),
            "median": float(np.median([r["arm_norm_Nm"] for r in rows])) if rows else None,
            "rows": rows, "interpretation": "TGS solver-dq estimator; not a torque sensor residual"}


def target_from_frozen_source():
    path = ROOT / "fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py"
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "FR3_OFFICIAL_START_Q_RAD" for t in node.targets):
            return np.asarray(ast.literal_eval(node.value), float)
    raise ValueError("Frozen native drive target not found")


def analyze_cell(cell):
    directory = R6 / f"run_{cell}"
    result, frames = load_run(directory)
    journal = read_jsonl(directory / "command_journal.jsonl")
    precommands = [x for x in journal if x["event"] == "PRE_COMMAND"]
    immediate = [x for x in journal if x["event"] == "IMMEDIATE_READBACK"]
    transfer = next(x for x in frames if x["arms"]["left"]["position_drives_disabled"])
    native = result["native_settle_final"]
    target = target_from_frozen_source()
    out = {"status": result["status"], "completed_commands": result["actual_effort_commands"],
           "full_nine_coordinate_writes": 2 * len(precommands), "frame_count": len(frames),
           "command_post_steps": [x[1]["physics_step"] for x in commanded_intervals(frames, "left")],
           "immediate_readback_count": len(immediate), "sides": {},
           "unexpected_collisions": [y for x in frames for y in x.get("collision", {}).get("unexpected", [])],
           "source_hashes": {n: sha256(directory / n) for n in ("frames.jsonl", "result.json", "command_journal.jsonl")}}
    # Pair the ordered PRE_COMMAND and IMMEDIATE_READBACK records independently.
    out["immediate_readback_recomputed_match"] = len(precommands) == len(immediate) and all(
        all(np.asarray(p["full_effort"][s]).shape == (9,) and np.allclose(
            p["full_effort"][s], r["readback"][s], atol=1e-6, rtol=0) for s in SIDES)
        for p, r in zip(precommands, immediate))
    for side in SIDES:
        a = transfer["arms"][side]
        intervals = commanded_intervals(frames, side)
        b = intervals[-1][3]
        duration = intervals[-1][1]["time_s"] - transfer["time_s"]
        q0, q1, v0 = np.asarray(a["q"]), np.asarray(b["q"]), np.asarray(a["dq"])
        states = [a] + [x[3] for x in intervals]
        delta = q1[:7] - q0[:7]
        fd = [(np.asarray(y["q"]) - x["q"]) / dt - y["dq"] for _, _, x, y, dt in intervals]
        req = np.asarray([x[3]["requested_full_effort"] for x in intervals])
        expected = np.asarray(result["registered_hold_effort"][side]).copy()
        expected[3] += result["registered_perturbation"]
        rates = [(np.asarray(p["full_effort"][side])[:7] - np.asarray(p["previous_applied"][side])[:7]) / p["dt_s"] for p in precommands]
        position_path = [np.asarray(x["tcp_world"])[:3, 3] for x in states]
        residual = historical_residuals(frames, side)
        side_out = {
            "delta_q_arm_rad": delta.tolist(), "joint4_delta_rad": float(delta[3]),
            "max_abs_delta_q_rad": float(np.max(np.abs(delta))),
            "max_abs_joint_drift_over_interval_rad": np.max(np.abs(np.asarray([x["q"] for x in states])[:, :7] - q0[:7]), axis=0).tolist(),
            "duration_s": duration, "link8_translation_m": float(np.linalg.norm(np.asarray(b["link8_world"])[:3, 3] - np.asarray(a["link8_world"])[:3, 3])),
            "link8_rotation_rad": so3_angle(a["link8_world"], b["link8_world"]),
            "tcp_translation_m": float(np.linalg.norm(position_path[-1] - position_path[0])),
            "tcp_rotation_rad": so3_angle(a["tcp_world"], b["tcp_world"]),
            "max_tcp_translation_m": max(float(np.linalg.norm(p-position_path[0])) for p in position_path),
            "tcp_path_length_m": sum(float(np.linalg.norm(y-x)) for x,y in zip(position_path,position_path[1:])),
            "max_tcp_rotation_rad": max(so3_angle(a["tcp_world"], x["tcp_world"]) for x in states),
            "requested_effort_matches_registered_vector": bool(np.allclose(req[:, :7], expected, atol=1e-12, rtol=0)),
            "finger_effort_all_zero": bool(np.all(req[:, 7:] == 0)),
            "poststep_readback_max_abs_error_Nm": float(max(np.max(np.abs(np.asarray(x[3]["applied_command_not_sensor"])-x[3]["requested_full_effort"])) for x in intervals)),
            "max_effort_Nm_by_joint": np.max(np.abs(req[:, :7]), axis=0).tolist(),
            "command_rate_Nm_s": vector_stats(rates),
            "historical_rate_margin_Nm_s": (2400. - np.max(np.abs(rates), axis=0)).tolist(),
            "fd_minus_end_dq_stats": vector_stats(np.asarray(fd)[:, :7]),
            "model_residual_Nm": residual,
            "ownership_transition": {
                "physics_step": transfer["physics_step"], "time_s": transfer["time_s"],
                "native_target_rad": target.tolist(), "target_provenance": "literal FR3_OFFICIAL_START_Q_RAD in frozen builder; arm target getter was not archived",
                "q_rad": q0[:7].tolist(), "q_minus_native_target_rad": (q0[:7]-target).tolist(),
                "solver_dq_rad_s": v0[:7].tolist(), "gravity_Nm": a["g"][:7],
                "state_change_without_physics_q": (q0 - native["arms"][side]["q"]).tolist(),
                "state_change_without_physics_dq": (v0 - native["arms"][side]["dq"]).tolist(),
                "arm_drive_gains_before": {k: native["arms"][side][k][:7] for k in ("drive_kp", "drive_kd")},
                "arm_drive_gains_after": {k: a[k][:7] for k in ("drive_kp", "drive_kd")},
                "constant_initial_velocity_displacement_rad": (v0[:7]*duration).tolist(),
                "observed_minus_constant_initial_velocity_rad": (delta-v0[:7]*duration).tolist(),
                "initial_velocity_explanation": "Nonzero inherited solver velocity is compatible with part of drift; v0*T is a counterfactual estimate, not causal isolation under TGS."},
            "fingers": {"max_abs_q_m": float(np.max(np.abs(np.asarray([x["q"][7:] for x in states])))),
                        "max_abs_dq_m_s": float(np.max(np.abs(np.asarray([x["dq"][7:] for x in states]))))}}
        out["sides"][side] = side_out
    return out


def main():
    cells = {c: analyze_cell(c) for c in "ABCD"}
    signed = {}
    for side in SIDES:
        a,b,c,d = [np.asarray(cells[k]["sides"][side]["delta_q_arm_rad"]) for k in "ABCD"]
        signed[side] = {"B_minus_A_arm_rad": (b-a).tolist(), "C_minus_A_arm_rad": (c-a).tolist(),
                        "odd_half_B_minus_C_arm_rad": ((b-c)/2).tolist(),
                        "even_half_B_plus_C_minus_A_arm_rad": ((b+c)/2-a).tolist(),
                        "B_minus_D_arm_rad": (b-d).tolist(),
                        "repeatability_max_abs_all_seven_rad": float(np.max(np.abs(b-d))),
                        "B_joint4_rad": float(b[3]), "C_joint4_rad": float(c[3]), "D_joint4_rad": float(d[3]),
                        "cross_joint_odd_norm_excluding_joint4_rad": float(np.linalg.norm(np.delete((b-c)/2,3)))}
    r7 = {}
    for cell in ("S1", "S3"):
        directory = HERE / "evidence" / "diagnostic" / cell
        if (directory / "frames.jsonl").exists():
            result, frames = load_run(directory)
            r7[cell] = {"status": result["status"], "sides": {s: historical_residuals(frames, s) for s in SIDES}}
    residual_max = max(cells[c]["sides"][s]["model_residual_Nm"]["max"] for c in cells for s in SIDES)
    out = {"schema": "TASK28_G2_P5_QP_6_R7_R6_REANALYSIS_V2", "cells": cells,
           "total_full_nine_coordinate_writes": sum(x["full_nine_coordinate_writes"] for x in cells.values()),
           "signed_response": signed, "max_model_residual_estimator_Nm": residual_max,
           "r6_historical_residual_Nm": HISTORICAL_RESIDUAL,
           "historical_residual_absolute_reproduction_error_Nm": abs(residual_max-HISTORICAL_RESIDUAL),
           "r7_same_definition_residuals": r7,
           "r7_vs_r6_interpretation": "Same definition, different controllers and durations: descriptive comparison only; no formal model improvement claim.",
           "conclusions": {"r6_archive_complete": all(cells[c]["completed_commands"] == 18 and all(cells[c]["sides"][s]["model_residual_Nm"]["count"] == 18 for s in SIDES) for c in cells),
                           "signed_response_reproduced": all(signed[s]["B_joint4_rad"]>0 and signed[s]["C_joint4_rad"]<0 for s in SIDES),
                           "repeatability_max_abs_rad": max(signed[s]["repeatability_max_abs_all_seven_rad"] for s in SIDES),
                           "A_is_formal_static_hold": False,
                           "tgs_uncertainty": "End-step solver dq and whole-step displacement velocity differ; neither differencing dq nor finite q acceleration establishes independent acceleration or physical torque sensing."}}
    (HERE / "evidence" / "r6_reanalysis.json").write_text(json.dumps(out, indent=2, sort_keys=True, allow_nan=False)+"\n")
    print(json.dumps({"commands": {c: cells[c]["completed_commands"] for c in cells}, "writes": out["total_full_nine_coordinate_writes"], "max_residual_Nm": residual_max, "historical_error_Nm": out["historical_residual_absolute_reproduction_error_Nm"], "r7_residuals": {c: {s: r7[c]["sides"][s]["max"] for s in SIDES} for c in r7}}, indent=2))


if __name__ == "__main__":
    main()
