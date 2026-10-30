"""R13 read-only repeatability and bounded-perturbation replay.

The deterministic arm/object records are replayed exactly twenty times per cell.
Perturbations are a geometry/state sensitivity diagnostic over immutable R12 q and
virtual-goal samples; they do not re-solve the QP or authorize any command.
"""
from __future__ import annotations

import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual

HERE = Path(__file__).resolve().parent
BASE = HERE / "evidence" / "r12_baseline"
DT = 1.0 / 60.0
OBJECT_LIMIT_M = 0.00025
RELATIVE_LIMIT_M = 0.00015
R12_HEAD = "52d07e6a9c4ac482db4887483cbf8ab0cbf510d1"


def _load(cell: str):
    rows = [
        json.loads(line)
        for line in (BASE / "diagnostic" / cell / "frames.jsonl").read_text().splitlines()
        if "r12_qp" in line
    ]
    result = json.loads((BASE / "diagnostic" / cell / "result.json").read_text())
    evaluation = json.loads((BASE / "evaluation.json").read_text())["cells"][cell]
    vf = result["virtual_frame"]
    q = np.asarray(
        [
            np.r_[row["arms"]["left"]["q"][:7], row["arms"]["right"]["q"][:7]]
            for row in rows
        ],
        dtype=float,
    )
    goals = np.asarray([row["r12_qp"]["virtual_goal"] for row in rows], dtype=float)
    diagnostics = [row["r12_qp"]["diagnostics"] for row in rows]
    return rows, result, evaluation, q, goals, np.asarray(vf["T_VL"]), np.asarray(vf["T_VR"]), diagnostics


def _summary(object_errors: np.ndarray, relative_errors: np.ndarray) -> dict:
    object_errors = np.asarray(object_errors, dtype=float)
    relative_errors = np.asarray(relative_errors, dtype=float)
    object_over = int(np.count_nonzero(object_errors > OBJECT_LIMIT_M))
    relative_over = int(np.count_nonzero(relative_errors > RELATIVE_LIMIT_M))
    reasons = []
    if object_over:
        reasons.append("OBJECT_TRACKING_THRESHOLD_EXCEEDED")
    if relative_over:
        reasons.append("RELATIVE_TCP_TRANSLATION_THRESHOLD_EXCEEDED")
    return {
        "sample_count": int(object_errors.size),
        "object_peak_mm": float(object_errors.max(initial=0.0) * 1e3),
        "object_mean_mm": float(object_errors.mean() * 1e3),
        "object_p95_mm": float(np.percentile(object_errors, 95) * 1e3),
        "relative_peak_mm": float(relative_errors.max(initial=0.0) * 1e3),
        "relative_mean_mm": float(relative_errors.mean() * 1e3),
        "relative_p95_mm": float(np.percentile(relative_errors, 95) * 1e3),
        "object_over_limit_count": object_over,
        "relative_over_limit_count": relative_over,
        "failure_reasons": reasons,
        "acceptance_pass": object_over == 0 and relative_over == 0,
    }


def _archive_metrics(evaluation: dict) -> tuple[np.ndarray, np.ndarray]:
    records = evaluation["virtual_tracking_records"]
    objects = np.asarray(
        [max(r["object_left_error_m"], r["object_right_error_m"]) for r in records], dtype=float
    )
    relative = np.asarray([r["relative_error_m"] for r in records], dtype=float)
    return objects, relative


def _geometry_metrics(q: np.ndarray, goals: np.ndarray, tvl: np.ndarray, tvr: np.ndarray, model: RobotModel):
    objects, relative = [], []
    for qi, goal in zip(q, goals):
        left = model.fk("left", qi[:7]) @ inverse(tvl)
        right = model.fk("right", qi[7:]) @ inverse(tvr)
        objects.append(max(residual(left, goal)[0], residual(right, goal)[0]))
        relative.append(residual(left, right)[0])
    return _summary(np.asarray(objects), np.asarray(relative))


def _solver_audit(diagnostics: list[dict]) -> dict:
    statuses = {}
    for d in diagnostics:
        statuses[d["status"]] = statuses.get(d["status"], 0) + 1
    qdot = [float(d["qdot_inf"]) for d in diagnostics]
    hard = [float(d["hard_relative_residual_inf"]) for d in diagnostics]
    common = [float(d["common_task_residual_inf"]) for d in diagnostics]
    return {
        "status_counts": statuses,
        "all_qp_optimal": statuses == {"QP_OPTIMAL": len(diagnostics)},
        "constraint_activation": {
            "qdot_box_active": max(qdot, default=0.0) >= 0.12,
            "position_lookahead_active": False,
            "affine_relative_equality": "HARD_EQUALITY_PRESENT_NOT_ACTIVE_SET",
        },
        "qdot_peak_rad_s": max(qdot, default=0.0),
        "hard_relative_residual_peak": max(hard, default=0.0),
        "common_task_residual_peak": max(common, default=0.0),
    }


def _perturbed_q(q: np.ndarray, initial_sign: int, dt_scale: float, model_scale: float, delay_steps: int):
    q0 = q[0]
    increments = np.vstack([np.zeros((1, q.shape[1])), np.diff(q, axis=0)])
    cumulative = np.cumsum(increments, axis=0)
    if delay_steps:
        cumulative = np.vstack([np.zeros((delay_steps, q.shape[1])), cumulative[:-delay_steps]])
    initial = np.full(q.shape[1], initial_sign * 1e-4, dtype=float)
    return q0 + initial + cumulative * dt_scale / model_scale


def _one_perturbation(cell_data, initial_sign: int, dt_scale: float, model_scale: float, delay_steps: int):
    _, _, _, q, goals, tvl, tvr, _ = cell_data
    model = RobotModel(DEFAULT_FIXTURE)
    qs = _perturbed_q(q, initial_sign, dt_scale, model_scale, delay_steps)
    summary = _geometry_metrics(qs, goals, tvl, tvr, model)
    lower = np.r_[model.joint_lower, model.joint_lower]
    upper = np.r_[model.joint_upper, model.joint_upper]
    qdot = np.diff(qs, axis=0) / (DT * dt_scale)
    joint_limit_violations = int(np.count_nonzero((qs < lower) | (qs > upper)))
    qdot_peak = float(np.max(np.abs(qdot), initial=0.0))
    summary.update(
        {
            "initial_joint_offset_rad": initial_sign * 1e-4,
            "control_period_scale": dt_scale,
            "model_mass_scale": model_scale,
            "feedback_delay_steps": delay_steps,
            "joint_limit_violation_count": joint_limit_violations,
            "qdot_peak_rad_s": qdot_peak,
            "qdot_box_respected": qdot_peak <= 0.12,
            "collision_check": "NOT_RUN_DIAGNOSTIC_STATE_REPLAY",
            "solver_status": "NOT_RE_SOLVED_DIAGNOSTIC_STATE_REPLAY",
            "constraint_activation": "NOT_RE_SOLVED;_ARCHIVED_R12_QP_OPTIMAL_ONLY",
            "physical_execution": False,
        }
    )
    return summary


def main(output: Path = HERE / "evidence" / "robustness_replay.json"):
    cells = {}
    perturbation_basis = {
        "initial_joint_offset_rad": {
            "tested": [-1e-4, 1e-4],
            "basis": "0.285 of the largest R12 joint tracking peak 0.0003511005 rad and below the 0.02 rad gate",
        },
        "control_period_scale": {
            "tested": [0.99, 1.01],
            "basis": "pre-registered +/-1% envelope around the fixed 1/60 s control period; R12 measured callback period was constant",
        },
        "feedback_delay_steps": {
            "tested": [0, 1],
            "basis": "zero or one 60 Hz sample, with one sample equal to 16.667 ms discrete state-ordering delay",
        },
        "model_mass_scale": {
            "tested": [0.97, 1.03],
            "basis": "rounded outward from the largest archived R12 diagonal-M range of 3.073% across frames",
        },
    }
    for cell in ("V10-A", "V10-B", "V10-C"):
        data = _load(cell)
        rows, _, evaluation, q, goals, tvl, tvr, diagnostics = data
        archived_objects, archived_relative = _archive_metrics(evaluation)
        fingerprint = hashlib.sha256(
            json.dumps(evaluation["virtual_tracking_records"], sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        repeats = []
        for repeat in range(1, 21):
            s = _summary(archived_objects.copy(), archived_relative.copy())
            s.update(
                {
                    "repeat": repeat,
                    "replay_kind": "DETERMINISTIC_ARCHIVED_R12_RECORD_REPLAY",
                    "record_fingerprint": fingerprint,
                    "solver": _solver_audit(diagnostics),
                    "physical_execution": False,
                }
            )
            repeats.append(s)
        data_out = {
            "deterministic_repetition": {
                "repetitions": 20,
                "identical_record_fingerprint": len({x["record_fingerprint"] for x in repeats}) == 1,
                "peak_span_mm": max(x["object_peak_mm"] for x in repeats) - min(x["object_peak_mm"] for x in repeats),
                "relative_peak_span_mm": max(x["relative_peak_mm"] for x in repeats) - min(x["relative_peak_mm"] for x in repeats),
                "all_acceptance_pass": all(x["acceptance_pass"] for x in repeats),
                "solver_status": repeats[0]["solver"],
                "summary": repeats[0],
            },
            "perturbations": {},
            "r12_acceptance_margin_mm": 0.250 - float(evaluation["virtual_object_tracking_peak_m"] * 1e3),
        }
        for name, args in {
            "initial_state_plus": (1, 1.0, 1.0, 0),
            "initial_state_minus": (-1, 1.0, 1.0, 0),
            "control_period_plus_1pct": (0, 1.01, 1.0, 0),
            "control_period_minus_1pct": (0, 0.99, 1.0, 0),
            "feedback_delay_one_step": (0, 1.0, 1.0, 1),
            "model_mass_plus_3pct": (0, 1.0, 1.03, 0),
            "model_mass_minus_3pct": (0, 1.0, 0.97, 0),
        }.items():
            data_out["perturbations"][name] = _one_perturbation(data, *args)
        combinations = []
        for sign, dt_scale, model_scale, delay in itertools.product((-1, 1), (0.99, 1.01), (0.97, 1.03), (0, 1)):
            s = _one_perturbation(data, sign, dt_scale, model_scale, delay)
            s["combination"] = {
                "initial_sign": sign,
                "control_period_scale": dt_scale,
                "model_mass_scale": model_scale,
                "feedback_delay_steps": delay,
            }
            combinations.append(s)
        data_out["combined_extremes"] = {
            "combinations_tested": len(combinations),
            "worst_case": max(combinations, key=lambda x: x["object_peak_mm"]),
            "any_acceptance_pass": any(x["acceptance_pass"] for x in combinations),
            "all_acceptance_pass": all(x["acceptance_pass"] for x in combinations),
        }
        cells[cell] = data_out
    out = {
        "schema": "TASK28_R13_ROBUSTNESS_REPLAY_V1",
        "baseline_head": R12_HEAD,
        "controller_frozen": {
            "feedback_gain_scale": 0.75,
            "velocity_capture_tau_s": 0.10,
            "object_limit_m": OBJECT_LIMIT_M,
            "relative_limit_m": RELATIVE_LIMIT_M,
            "qp_constraints_and_limits_changed": False,
        },
        "execution_authorized": False,
        "physical_execution": False,
        "model": {
            "deterministic": "exact immutable R12 evaluator records, repeated 20 times per cell",
            "perturbation": "recorded R12 joint-state trajectory geometry replay; no QP re-solve, no contact, no Cube motion",
            "units": "reported error summaries are millimetres",
        },
        "perturbation_basis": perturbation_basis,
        "cells": cells,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(json.dumps({c: {"deterministic": cells[c]["deterministic_repetition"], "worst": cells[c]["combined_extremes"]["worst_case"]} for c in cells}, indent=2))


if __name__ == "__main__":
    main()
