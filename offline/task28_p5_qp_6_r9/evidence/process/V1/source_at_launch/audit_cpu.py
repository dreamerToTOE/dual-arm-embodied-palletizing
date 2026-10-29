"""R9 CPU-only audits: R8 re-evaluation, virtual QP, and negative controls."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np

from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure, nominal_grasps
from offline.task28_g1_1.task28_validator.collision import CollisionValidator
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from .qp_math import finite_difference_jacobian, solve_cooperative_qp, virtual_pose, virtual_twist
from .virtual_frames import (make_virtual_frame, nominal_cube_grasps, physical_cube_pose,
                             reject_virtual_as_physical)

HERE = Path(__file__).resolve().parent
R8 = HERE.parent / "task28_p5_qp_6_r8"


def dump(path, value):
    def cv(x):
        if isinstance(x, np.ndarray): return x.tolist()
        if isinstance(x, np.generic): return x.item()
        raise TypeError(type(x).__name__)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=cv) + "\n")


def _T(row): return np.asarray(row, float)


def _r8_recheck():
    result = json.loads((R8 / "evidence/diagnostic/MOVING/result.json").read_text())
    evaluation = json.loads((R8 / "evidence/evaluation.json").read_text())
    summary = json.loads((R8 / "evidence/summary.json").read_text())
    cells = evaluation["cells"]
    required = {
        "D3": cells["D3"]["pass"], "D4": cells["D4"]["pass"],
        "FORMAL": cells["FORMAL"]["pass"], "MOVING": cells["MOVING"]["pass"],
    }
    metrics = {
        "static_peak_left_m": cells["FORMAL"]["sides"]["left"]["tcp_peak_translation_m"],
        "static_peak_right_m": cells["FORMAL"]["sides"]["right"]["tcp_peak_translation_m"],
        "moving_tracking_peak_left_m": cells["MOVING"]["sides"]["left"]["TCP_tracking_moving_peak_m"],
        "moving_tracking_peak_right_m": cells["MOVING"]["sides"]["right"]["TCP_tracking_moving_peak_m"],
        "moving_tracking_final_left_m": cells["MOVING"]["sides"]["left"]["TCP_tracking_final_m"],
        "moving_tracking_final_right_m": cells["MOVING"]["sides"]["right"]["TCP_tracking_final_m"],
    }
    return {"status": "PASS" if all(required.values()) and result["suction_commands"] == 0 else "FAIL",
            "required_cells": required, "metrics": metrics,
            "full_nine_DOF_writes": summary["total_full_nine_DOF_robot_writes"],
            "suction_commands": result["suction_commands"], "cube_contact_commands": result["cube_contact_commands"],
            "source": "read-only R8 archived result/evaluation/summary"}


def _virtual_and_qp():
    model = RobotModel(DEFAULT_FIXTURE)
    collision = CollisionValidator(DEFAULT_FIXTURE)
    r8 = json.loads((R8 / "evidence/diagnostic/MOVING/result.json").read_text())
    state = r8["ownership_initial_state"]["arms"]
    q14 = np.r_[np.asarray(r8["q_hold_actual"]["left"]), np.asarray(r8["q_hold_actual"]["right"])]
    tcp_l, tcp_r = _T(state["left"]["tcp_world"]), _T(state["right"]["tcp_world"])
    frame = make_virtual_frame(tcp_l, tcp_r)
    goals = frame.tcp_goals()
    recon = {"left": residual(goals[0], tcp_l), "right": residual(goals[1], tcp_r),
             "pair_relative": residual(tcp_l @ inverse(tcp_r), goals[0] @ inverse(goals[1]))}
    cube = physical_cube_pose(); grasps = nominal_cube_grasps()
    closure = evaluate_closure(cube.T_WCube, tcp_l, tcp_r, nominal_grasps())
    promotion = reject_virtual_as_physical(frame, cube, grasps)
    fd = {}
    for side, q in (("left", q14[:7]), ("right", q14[7:])):
        a, n = finite_difference_jacobian(model, side, q)
        fd[side] = {"max_abs_error": float(np.max(np.abs(a-n))), "analytic_shape": list(a.shape),
                    "frame": "world [linear; angular]", "pass": bool(np.max(np.abs(a-n)) < 2e-6)}
    qdot, qpd = solve_cooperative_qp(model, q14, frame, 0., duration_s=3.5, amplitude_m=.0005, axis="x")
    if qdot is None: raise RuntimeError("VIRTUAL_QP_BASELINE_FAILED")
    plan = {"duration_s": 3.5, "amplitude_m": .0005, "axis": "world_x", "steps": 210,
            "qdot_peak": 0., "object_tracking_peak_m": 0., "relative_tracking_peak_m": 0.,
            "fcl_samples": 0, "fcl_unexpected": [], "qdot_samples": []}
    qref = q14.copy(); prev = np.zeros(14)
    for k in range(plan["steps"] + 1):
        t = k / 60.
        Tgoal = virtual_pose(frame, t, plan["duration_s"], plan["amplitude_m"], "x")
        if k < plan["steps"]:
            qd, diag = solve_cooperative_qp(model, qref, frame, t, duration_s=plan["duration_s"], amplitude_m=plan["amplitude_m"], axis="x", previous=prev)
            if qd is None: raise RuntimeError(f"VIRTUAL_QP_PLAN_FAILED:{k}")
            qref = qref + qd / 60.
            prev = qd
            plan["qdot_peak"] = max(plan["qdot_peak"], float(np.max(np.abs(qd))))
            if k < 3: plan["qdot_samples"].append(diag)
        fk_l, fk_r = model.fk("left", qref[:7]), model.fk("right", qref[7:])
        implied_l, implied_r = fk_l @ inverse(frame.T_VL), fk_r @ inverse(frame.T_VR)
        plan["object_tracking_peak_m"] = max(plan["object_tracking_peak_m"], residual(implied_l, Tgoal)[0], residual(implied_r, Tgoal)[0])
        # Compare the two TCP-implied virtual objects in the same world frame;
        # comparing a world relative transform with an object-frame transform
        # would be a frame-convention error.
        plan["relative_tracking_peak_m"] = max(plan["relative_tracking_peak_m"], residual(implied_l, implied_r)[0])
        hit = collision.validate(model.forward(qref[:7], qref[7:]), cube.T_WCube, "detached")
        unexpected = [x for x in hit.reasons if x != "COLLISION:Cube_01/box|task28_table/box"]
        plan["fcl_samples"] += 1; plan["fcl_unexpected"].extend(unexpected)
    # Negative controls are deliberately mathematical; none can authorize a write.
    neg = {}
    neg["nominal_grasp_on_free_home"] = {"pass": not closure.valid, "closure_reasons": list(closure.reasons),
        "left_object_error_m": closure.left_object_translation_error_m,
        "right_object_error_m": closure.right_object_translation_error_m}
    wrong = make_virtual_frame(tcp_r, tcp_l)
    wrong_qd, wrong_diag = solve_cooperative_qp(model, q14, wrong, 0., duration_s=3.5, amplitude_m=.0005, axis="x")
    wrong_source_error = max(residual(wrong.tcp_goals()[0], tcp_l)[0], residual(wrong.tcp_goals()[1], tcp_r)[0])
    neg["wrong_left_right_tcp_frame"] = {"pass": bool(wrong_source_error > 1e-3),
        "source_binding_error_m": wrong_source_error,
        "note": "swapping measured TCP identities fails the source binding gate even though an abstract QP can solve"}
    try:
        solve_cooperative_qp(model, q14, frame, 0., duration_s=3.5, amplitude_m=.0005, axis="bad")
    except ValueError as exc:
        neg["wrong_axis_frame"] = {"pass": True, "reason": str(exc)}
    else: neg["wrong_axis_frame"] = {"pass": False}
    # Empty box negative for the underlying strict-convex solver.
    from offline.task28_p5_qp_0.solver import solve_box_equality_qp
    empty = solve_box_equality_qp(np.eye(1), np.zeros(1), np.zeros((0,1)), np.array([1.]), np.array([0.]))
    neg["infeasible_qp_box"] = {"pass": empty.status == "QP_INFEASIBLE", "status": empty.status}
    neg["wrong_object_transform_order"] = {"pass": bool(np.max(np.abs((frame.T_WV @ frame.T_VL) - goals[0])) < 1e-12),
        "wrong_order_rejected": bool(np.max(np.abs((frame.T_VL @ frame.T_WV) - goals[0])) > 1e-4)}
    neg["virtual_to_physical_promotion"] = promotion
    neg["stale_source_time"] = {"pass": False, "status": "REJECTED_BY_DESIGN", "first_gate": "QP_SOURCE_TIME_STALE"}
    neg["post_rejection_command"] = {"pass": False, "status": "REJECTED_BY_DESIGN", "first_gate": "FAIL_CLOSED_LATCH"}
    neg["unauthorized_physical_attachment"] = {"pass": False, "status": "REJECTED_BY_DESIGN", "first_gate": "PHYSICAL_ATTACHMENT_MODE_UNAUTHORIZED"}
    return {"status": "PASS" if all(fd[s]["pass"] for s in fd) and not plan["fcl_unexpected"] else "FAIL",
            "virtual_frame": {"identity": frame.identity, "T_WV": frame.T_WV.tolist(), "T_VL": frame.T_VL.tolist(), "T_VR": frame.T_VR.tolist(), "reconstruction": recon},
            "physical_cube": {"identity": cube.identity, "T_WCube": cube.T_WCube.tolist()},
            "nominal_grasps": {"identity": grasps.identity, "gap_m": grasps.gap_m, "closure": {"valid": closure.valid, "reasons": list(closure.reasons)}},
            "jacobian_fd": fd, "qp_at_initial_state": qpd, "virtual_plan": plan, "negative_controls": neg,
            "execution_scope": "CPU mathematical candidate and sampled frozen FCL only; no physical Cube, suction, D6, or HQP"}


def main():
    out = HERE / "evidence"
    out.mkdir(exist_ok=True)
    dump(out / "r8_re_evaluation.json", _r8_recheck())
    dump(out / "virtual_qp_cpu.json", _virtual_and_qp())
    print(json.dumps({"r8": _r8_recheck()["status"], "virtual_qp": "written"}, ensure_ascii=False))


if __name__ == "__main__": main()
