#!/usr/bin/env python3
"""Non-actuating Task27 QP replay diagnostic.

This intentionally does not connect to Isaac, ROS, suction, or a torque sink.
It binds the frozen R12 kinematic model to settled Task27 joint feedback,
checks the model/FK translation residual against the recorded TCP position,
and records R12 velocity-QP solves.  The result is diagnostic evidence only:
R12 marks its output execution-unauthorized and inverse-dynamics-unproven.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6_r12.qp_math import solve_pose_aware_qp
from offline.task28_p5_qp_6_r12.virtual_frames import make_virtual_frame


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--settle-index", type=int, default=100)
    args = parser.parse_args()
    rows = []
    with args.input.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("event") == "TORQUE_FRAME":
                rows.append(row)
    if len(rows) <= args.settle_index:
        raise SystemExit("INSUFFICIENT_SETTLED_FRAMES")
    model = RobotModel()
    anchor = rows[args.settle_index]
    q_anchor = np.r_[anchor["q_rad"]["left"], anchor["q_rad"]["right"]]
    T_anchor_l = model.fk("left", q_anchor[:7])
    T_anchor_r = model.fk("right", q_anchor[7:])
    frame = make_virtual_frame(T_anchor_l, T_anchor_r)
    previous = np.zeros(14)
    solves = []
    model_residuals_mm = []
    for row in rows[args.settle_index:]:
        q = np.r_[row["q_rad"]["left"], row["q_rad"]["right"]]
        T_l = model.fk("left", q[:7])
        T_r = model.fk("right", q[7:])
        for side, T in (("left", T_l), ("right", T_r)):
            measured = np.asarray(row["tcp_world_m"][side], dtype=float)
            model_residuals_mm.append(float(np.linalg.norm(T[:3, 3] - measured) * 1000.0))
        qdot, diag = solve_pose_aware_qp(
            model, q, frame, T_l, T_r, frame.T_WV,
            virtual_feedforward=np.zeros(6), previous=previous,
            dt=float(row.get("dt_s", 1.0 / 60.0)), qdot_limit=0.12,
            position_margin=0.02,
        )
        solves.append({
            "physics_step": int(row["physics_step"]),
            "status": diag.get("status"),
            "calculation_time_s": float(diag.get("calculation_time_s", 0.0)),
            "common_task_residual_inf": diag.get("common_task_residual_inf"),
            "hard_relative_residual_inf": diag.get("hard_relative_residual_inf"),
            "execution_authorized": bool(diag.get("execution_authorized", False)),
            "inverse_dynamics_claim": bool(diag.get("inverse_dynamics_claim", False)),
            "qdot_inf": diag.get("qdot_inf"),
        })
        if qdot is not None:
            previous = np.asarray(qdot, dtype=float)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "status": "DIAGNOSTIC_ONLY",
        "source": str(args.input),
        "model_identity": str(model.fixture_dir),
        "settle_index": args.settle_index,
        "frame_count": len(solves),
        "solve_count": len(solves),
        "qp_status_counts": {status: sum(x["status"] == status for x in solves)
                              for status in sorted({x["status"] for x in solves})},
        "calculation_time_s_mean": float(np.mean([x["calculation_time_s"] for x in solves])),
        "calculation_time_s_max": float(np.max([x["calculation_time_s"] for x in solves])),
        "common_task_residual_inf_max": float(max(
            x["common_task_residual_inf"] for x in solves if x["common_task_residual_inf"] is not None
        )),
        "hard_relative_residual_inf_max": float(max(
            x["hard_relative_residual_inf"] for x in solves if x["hard_relative_residual_inf"] is not None
        )),
        "model_fk_tcp_translation_residual_max_mm": max(model_residuals_mm),
        "model_fk_tcp_translation_residual_p95_mm": float(np.percentile(model_residuals_mm, 95)),
        "execution_authorized": False,
        "inverse_dynamics_claim": False,
        "physical_feedback": "joint_positions_and_logged_tcp_translation_only; orientation/J/M/c/g not bound",
        "reason": "R12 is a kinematic velocity QP and remains unauthorized for Task27 torque execution",
        "first_frames": solves[:3],
        "last_frames": solves[-3:],
    }
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
