"""CPU-only staged original-Cube approach audit.

The candidate is synchronized and FCL-screened at finite samples.  It is not a
continuous collision proof and is never sent to Isaac or suction hardware.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.collision import CollisionValidator
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps, desired_tcps
from offline.task28_g1_1.task28_validator.se3 import residual
from .virtual_frames import physical_cube_pose, nominal_cube_grasps

HERE = Path(__file__).resolve().parent
R8 = HERE.parent / "task28_p5_qp_6_r8"


def T_stage(cube, grasp, side, stage):
    T = cube @ grasp
    # Original nominal grasp has 1 mm gap.  The outer waypoints move farther
    # along the frozen Cube face normal; orientation and Cube pose are unchanged.
    if stage == "HIGH_OUTER": delta = np.array([0., (-.100 if side == "left" else .100), .280])
    elif stage == "OUTER_CONTACT": delta = np.array([0., (-.100 if side == "left" else .100), 0.])
    elif stage == "PRE_CONTACT_READY": delta = np.zeros(3)
    else: raise ValueError(stage)
    T[:3, 3] += cube[:3, :3] @ delta
    return T


def _edge(model, collision, cube, q0, q1, samples=51):
    first = None; unexpected_count = 0
    for i, u in enumerate(np.linspace(0., 1., samples)):
        ql = (1-u)*q0[0] + u*q1[0]; qr = (1-u)*q0[1] + u*q1[1]
        hit = collision.validate(model.forward(ql, qr), cube, "detached")
        bad = [x for x in hit.reasons if x != "COLLISION:Cube_01/box|task28_table/box"]
        if bad and first is None: first = {"sample": i, "fraction": float(u), "reasons": bad}
        unexpected_count += len(bad)
    return {"samples": samples, "first_unexpected_collision": first,
            "unexpected_collision_count": unexpected_count, "sampled_valid": first is None,
            "continuous_safety": "NOT_ESTABLISHED_FINITE_SAMPLING_ONLY"}


def main():
    model, collision = RobotModel(DEFAULT_FIXTURE), CollisionValidator(DEFAULT_FIXTURE)
    cube = physical_cube_pose().T_WCube; grasps = nominal_cube_grasps()
    r8 = json.loads((R8 / "evidence/diagnostic/MOVING/result.json").read_text())
    qhome = (np.asarray(r8["q_hold_actual"]["left"]), np.asarray(r8["q_hold_actual"]["right"]))
    stages = ("HIGH_OUTER", "OUTER_CONTACT", "PRE_CONTACT_READY")
    solutions = {"FREE_HOME": {"left": qhome[0].tolist(), "right": qhome[1].tolist()}}
    stage_targets = {}
    prev = {"left": qhome[0], "right": qhome[1]}
    endpoint = {}
    for stage in stages:
        goals = {"left": T_stage(cube, grasps.T_Cube_GL, "left", stage),
                 "right": T_stage(cube, grasps.T_Cube_GR, "right", stage)}
        stage_targets[stage] = {s: goals[s].tolist() for s in goals}
        solved = {}
        for side in ("left", "right"):
            result = model.ik(side, goals[side], seeds=[prev[side]], max_attempts=8, max_nfev=400)
            if not result.success:
                endpoint.setdefault(stage, {})[side] = {"success": False, "reason": result.reason,
                    "translation_error_m": result.translation_error_m, "rotation_error_rad": result.rotation_error_rad}
                solved[side] = prev[side]
            else:
                solved[side] = np.asarray(result.q)
                endpoint.setdefault(stage, {})[side] = {"success": True, "q": list(map(float, result.q)),
                    "translation_error_m": result.translation_error_m, "rotation_error_rad": result.rotation_error_rad,
                    "attempts": result.attempts, "function_evaluations": result.function_evaluations}
        prev = solved
        solutions[stage] = {s: solved[s].tolist() for s in solved}
    edges = {}
    for a, b in zip(("FREE_HOME", "HIGH_OUTER", "OUTER_CONTACT"), ("HIGH_OUTER", "OUTER_CONTACT", "PRE_CONTACT_READY")):
        q0 = (np.asarray(solutions[a]["left"]), np.asarray(solutions[a]["right"]))
        q1 = (np.asarray(solutions[b]["left"]), np.asarray(solutions[b]["right"]))
        edges[f"{a}_TO_{b}"] = _edge(model, collision, cube, q0, q1)
    # Reproduce the archived direct-joint negative without changing the fixture.
    direct = json.loads((HERE.parent / "task28_p5_qp_6_r3/evidence/approach_audit.json").read_text())
    direct_negative = {"source": "R3 archived approach_audit.json", "first_unexpected_collision": direct["straight_joint_interpolation"]["first_unexpected_collision"],
                      "samples_checked": direct["straight_joint_interpolation"]["samples_checked"], "continuous_safety": False,
                      "acm_modified": False, "fcl_modified": False, "cube_pose_modified": False, "tool_shape_modified": False}
    all_edges = all(v["sampled_valid"] for v in edges.values()) and all(
        endpoint[stage][side]["success"] for stage in stages for side in ("left", "right"))
    out = {
        "status": "SAMPLED_CANDIDATE_ONLY" if all_edges else "BLOCKED_NO_SAMPLED_CANDIDATE",
        "scope": "CPU-only synchronized bilateral IK/FCL screen; no trajectory execution, suction, D6 or Cube motion",
        "cube": {"identity": "PRE_CONTACT_ORIGINAL_CUBE", "T_WCube": cube.tolist(), "nominal_gap_m": .001,
                 "nominal_grasps": {"left": grasps.T_Cube_GL.tolist(), "right": grasps.T_Cube_GR.tolist()}},
        "stages": list(stages), "stage_targets": stage_targets, "solutions": solutions,
        "endpoint_ik": endpoint, "synchronized_edges": edges,
        "direct_home_to_nominal_negative": direct_negative,
        "task24_sequence_reference": ["HIGH_PRE_CONTACT", "OUTER_DESCENT", "LEFT_RIGHT_CONTACT_MICROTIMING", "COMMON_LIFT"],
        "physical_execution": {"status": "SKIPPED", "reason": "No continuous proof or qualified dynamic/torque margin; R9 forbids forcing long HOME-to-Cube motion into R8 small-motion controller."},
        "joint_interpolation_is_not_continuous_proof": True,
    }
    (HERE / "evidence").mkdir(exist_ok=True)
    (HERE / "evidence/approach_audit.json").write_text(json.dumps(out, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status":out["status"],"edges":{k:v["sampled_valid"] for k,v in edges.items()}},ensure_ascii=False))


if __name__ == "__main__": main()
