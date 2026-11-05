#!/usr/bin/env python3
"""Bounded free-space FR3 articulation smoke test for G1-A-FR3.

The scene asset and tool geometry come from the accepted Task27 scene.  The
Cube/table/box are excluded by loading only the constructor definitions.
Joint commands are Isaac ArticulationAction position targets; no joint state
is teleported.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys

import numpy as np

REPO = Path(__file__).resolve().parents[3]
# Task27 is a thin wrapper around this accepted constructor; use the underlying
# file so the helper definitions can be reused without executing the full scene.
ISAAC_SCENE = REPO / "isaac/scripts/task26_truck_box_scene.py"
OUT = REPO / "research/stage_b_contact_switch_fr3/evidence/f0_smoke"


def _load_isolated_scene(stage):
    """Reuse Task27 FR3/tool constructors without executing its full scene."""
    import omni.kit.app
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    source = ISAAC_SCENE.read_text(encoding="utf-8")
    # The selected scene wraps Task26 and executes the complete scene at the
    # first manager setup.  Everything above that point is constants/helpers.
    cut = source.index("\n_require_stopped_timeline()\nmanager =")
    namespace = {"__name__": "g1a_fr3_f0_scene"}
    exec(compile(source[:cut], str(ISAAC_SCENE), "exec"), namespace, namespace)
    namespace["stage"] = stage
    namespace["_MATERIAL"] = namespace["_make_material"]()
    physics = UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics)
    scene_api.CreateEnableCCDAttr().Set(True)
    scene_api.CreateEnableStabilizationAttr().Set(True)
    scene_api.CreateSolverTypeAttr().Set("TGS")
    UsdGeom.Xform.Define(stage, "/World")
    namespace["_add_fr3"]("/World/left_fr3", namespace["LEFT_BASE"])
    namespace["_apply_official_joint_limits"]()
    namespace["_set_fr3_official_start_target"]("/World/left_fr3")
    namespace["_build_tool"]("/World/left_fr3", -1.0)
    return namespace


def _pose(stage, path):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"TCP_PRIM_MISSING:{path}")
    transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    position = transform.ExtractTranslation()
    quat = transform.ExtractRotationQuat()
    imaginary = quat.GetImaginary()
    return [float(position[i]) for i in range(3)] + [
        float(quat.GetReal()), float(imaginary[0]), float(imaginary[1]), float(imaginary[2])
    ]


def _finite_vector(value, size, label):
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise RuntimeError(f"{label}_NONFINITE_OR_SHAPE:{array.shape}")
    return array


def main():
    from isaacsim import SimulationApp

    OUT.mkdir(parents=True, exist_ok=False)
    app = SimulationApp({"headless": True, "create_new_stage": True, "renderer": "RaytracedLighting", "multi_gpu": False})
    timeline = None
    articulation = None
    records = []
    summary = {
        "schema": "g1a.fr3.f0.smoke.v1",
        "status": "RUNNING",
        "scene_entry": str(ISAAC_SCENE),
        "excluded_prims": ["/World/right_fr3", "/World/Table", "/World/Task27", "/World/left_rail", "/World/right_rail"],
        "control_interface": "isaacsim.core.prims.SingleArticulation.apply_action(ArticulationAction.joint_positions)",
        "control_mode": "position_drive_target",
        "teleport_used": False,
        "contact_forces_used": False,
        "frames": 0,
        "physics_dt_s": 1.0 / 60.0,
    }
    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        _load_isolated_scene(stage)
        app.update()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()
        articulation = SingleArticulation("/World/left_fr3", name="g1a_f0_left_fr3")
        articulation.initialize()
        if not articulation.handles_initialized:
            raise RuntimeError("FR3_ARTICULATION_HANDLE_INVALID")
        if tuple(articulation.dof_names)[:7] != (
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ):
            raise RuntimeError(f"FR3_DOF_ORDER:{tuple(articulation.dof_names)}")

        q0 = _finite_vector(articulation.get_joint_positions(), 9, "Q0")[:7]
        target_start = q0.copy()
        target_move = q0.copy()
        target_move[0] += 0.05
        max_delta = np.zeros(7)
        max_dq = 0.0
        target_prev = target_start.copy()
        for step in range(150):
            if step < 30:
                phase = "settle"
                target = target_start
            elif step < 90:
                phase = "move"
                alpha = float(step - 30 + 1) / 60.0
                target = target_start + alpha * (target_move - target_start)
            else:
                phase = "stationary_hold"
                target = target_move
            target = _finite_vector(target, 7, "TARGET")
            articulation.apply_action(ArticulationAction(
                joint_positions=target,
                joint_indices=np.arange(7, dtype=np.int64),
            ))
            app.update()
            q = _finite_vector(articulation.get_joint_positions(), 9, "Q")[:7]
            dq = _finite_vector(articulation.get_joint_velocities(), 9, "DQ")[:7]
            applied = _finite_vector(articulation.get_applied_joint_efforts(), 9, "APPLIED")[:7]
            try:
                measured = _finite_vector(articulation.get_measured_joint_efforts(), 9, "MEASURED")[:7]
                measured_available = True
            except Exception:
                measured = np.full(7, np.nan)
                measured_available = False
            max_delta = np.maximum(max_delta, np.abs(q - target))
            max_dq = max(max_dq, float(np.max(np.abs(dq))))
            records.append({
                "step": step,
                "time_s": float((step + 1) * summary["physics_dt_s"]),
                "dt_s": summary["physics_dt_s"],
                "phase": phase,
                "target_q_rad": target.tolist(),
                "q_rad": q.tolist(),
                "dq_rad_s": dq.tolist(),
                "tcp_world_xyz_quat_wxyz": _pose(stage, "/World/left_fr3/fr3_link8/side_suction_tool/side_suction_tcp"),
                "applied_effort_Nm": applied.tolist(),
                "measured_effort_Nm": measured.tolist(),
                "measured_effort_available": measured_available,
                "target_step_max_rad": float(np.max(np.abs(target - target_prev))),
            })
            target_prev = target.copy()
        q_final = _finite_vector(articulation.get_joint_positions(), 9, "Q_FINAL")[:7]
        summary.update({
            "status": "PASS",
            "frames": len(records),
            "dof_names": list(articulation.dof_names),
            "q0_rad": q0.tolist(),
            "q_final_rad": q_final.tolist(),
            "max_abs_tracking_error_rad": float(np.max(max_delta)),
            "max_abs_joint_velocity_rad_s": max_dq,
            "max_target_step_rad": float(max(item["target_step_max_rad"] for item in records)),
            "tcp_displacement_m": float(np.linalg.norm(np.asarray(records[-1]["tcp_world_xyz_quat_wxyz"][:3]) - np.asarray(records[0]["tcp_world_xyz_quat_wxyz"][:3]))),
            "measured_effort_any": any(item["measured_effort_available"] for item in records),
            "transition": {"from": "move", "to": "stationary_hold", "step": 90, "objective": "stationary_position_target"},
        })
    except Exception as exc:
        summary["status"] = "BLOCKED"
        summary["exception"] = repr(exc)
    finally:
        (OUT / "timeseries.jsonl").write_text(
            "".join(json.dumps(item, sort_keys=True) + "\n" for item in records), encoding="utf-8"
        )
        (OUT / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if timeline is not None:
            try:
                timeline.stop()
            except Exception:
                pass
        try:
            app.update()
            app.close()
        except Exception as exc:
            summary["app_close_error"] = repr(exc)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
