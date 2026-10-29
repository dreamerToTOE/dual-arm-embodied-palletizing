"""Fresh original-scene bounded R8 closed-loop torque stabilization experiment.

This runner never teleports a body, never enables the R5 non-baseline PhysX
flag, and never writes a ROS/suction/Cube command.  It uses the R3 full-DOF
effort ownership pattern through :mod:`effort_adapter`.
"""
from __future__ import annotations
import argparse
import builtins
import hashlib
import importlib.util
import json
import os
import traceback
import sys
import time
from pathlib import Path

import numpy as np

from offline.task28_p5_qp_6_r2.common import (
    CUBE_PATH, DOF_NAMES, FIXTURE, FR3_ASSET, FR3_SHA256, LEFT_ROOT,
    RIGHT_ROOT, TASK_ROOT, sha256,
)
from offline.task28_p5_qp_6_r2.runtime_probe import _runtime_identity
from offline.task28_p5_qp_6_r8.effort_adapter import DiagnosticCertificate, R8ExclusiveEffortAdapter, Reject
from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate, Rejected

HERE = Path(__file__).resolve().parent


def write(path, value):
    def conv(x):
        if isinstance(x, np.ndarray):
            return x.tolist()
        if isinstance(x, np.generic):
            return x.item()
        raise TypeError(type(x).__name__)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False, default=conv) + "\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--configuration", choices=["D1","D2","D3","D4","FORMAL"], required=True)
    ap.add_argument("--output", type=Path, required=True)
    args, kit_args = ap.parse_known_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    cfg = json.loads((HERE / "source_contract.json").read_text())
    reg = json.loads((HERE / "registrations.json").read_text())[args.configuration]
    if reg["strategy"] == "selected_D2_or_D3":
        selection = json.loads((HERE / "evidence/selection.json").read_text())
        reg["strategy"] = selection["strategy"]
        if args.configuration == "FORMAL":
            prerequisite = json.loads((HERE / "evidence/formal_prerequisite.json").read_text())
            if prerequisite.get("eligible") is not True:
                raise RuntimeError("FORMAL_PREREQUISITE_NOT_PASS")
    from offline.task28_p5_qp_6_r8.controller import gains, reference, torque
    lifecycle_path = args.output / "child_lifecycle.jsonl"
    def lifecycle(event, **extra):
        with lifecycle_path.open("a") as stream:
            stream.write(json.dumps({"event":event,"pid":os.getpid(),"pgid":os.getpgid(0),"monotonic_s":time.monotonic(),**extra})+"\n")
            stream.flush(); os.fsync(stream.fileno())
    lifecycle("PYTHON_STARTED", kit_args=kit_args)
    result = {
        "status": "REJECTED", "phase": "BOOT", "first_failure": None,
        "configuration": args.configuration, "registration": reg,
        "explicit_physics_steps": 0, "actual_effort_commands": 0,
        "robot_command_writes": 0, "suction_commands": 0,
        "cube_contact_commands": 0, "ros_graph_suppressed": False,
        "non_baseline_diagnostic": False, "result_written_before_close": True,
        "level1_source": "PENDING", "level2_source": "PENDING",
    }
    rows = 0
    count = []
    arms = {}
    links = {}
    views = {}
    app = None
    subs = []
    last = None
    journal_path = args.output / "command_journal.jsonl"

    def journal(event):
        with journal_path.open("a") as stream:
            stream.write(json.dumps(event, sort_keys=True, allow_nan=False, default=lambda x: x.tolist() if isinstance(x, np.ndarray) else x.item() if isinstance(x, np.generic) else str(x)) + "\n")
            stream.flush(); os.fsync(stream.fileno())

    def record(row):
        nonlocal rows
        with (args.output / "frames.jsonl").open("a") as stream:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False) + "\n")
            stream.flush(); os.fsync(stream.fileno())
        rows += 1

    def save():
        result["callback_dts_s"] = list(count)
        result["recorded_frames"] = rows
        result["journal_sha256"] = hashlib.sha256(journal_path.read_bytes()).hexdigest() if journal_path.exists() else None
        write(args.output / "result.json", result)

    os.environ["TASK28_FR3_USD"] = str(FR3_ASSET)
    os.environ["TASK28_FR3_USD_SHA256"] = FR3_SHA256
    gate = R8DiagnosticGate()
    adapter = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False, "anti_aliasing": 0, "sync_loads": False, "fast_shutdown": True, "extra_args": ["--/app/settings/persistent=0"]})
        lifecycle("SIMULATION_APP_STARTED")
        import carb
        import carb.tokens
        result["kit_tokens"] = {name:carb.tokens.get_tokens_interface().resolve("${"+name+"}") for name in ("cache","appcache","logs","data")}
        portable = Path(kit_args[kit_args.index("--portable-root")+1]).resolve()
        if not result["kit_tokens"]["appcache"].startswith(str(portable)+"/"):
            raise Reject("CACHE_ISOLATION_NOT_EFFECTIVE")
        lifecycle("CACHE_ISOLATION_VERIFIED",tokens=result["kit_tokens"])
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import UsdGeom, UsdPhysics
        from scipy.spatial.transform import Rotation
        from isaacsim.core.api.simulation_context import SimulationContext
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from offline.task28_g1_1.task28_validator.collision import CollisionValidator
        from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
        from offline.task28_g1_1.task28_validator.se3 import residual

        manager = omni.kit.app.get_app().get_extension_manager()
        result["ros_bridge_enabled_at_app_start"] = manager.is_extension_enabled("isaacsim.ros2.bridge")
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        spec = importlib.util.spec_from_file_location("frozen_r8_scene", FIXTURE / "scene/task28_single_cube_scene.py")
        scene_mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(scene_mod)
        result["phase"] = "BUILD"
        scene_mod.build_scene()
        result["scene_before"] = _runtime_identity(stage, scene_mod)
        if result["scene_before"].get("status") != "SCENE_IDENTITY_VERIFIED" or sha256(FR3_ASSET) != FR3_SHA256:
            raise Reject("SCENE_IDENTITY")
        result["source_asset_sha256"] = sha256(FR3_ASSET)

        physics_prim = stage.GetPrimAtPath(TASK_ROOT + "/physicsScene")
        def physics_config():
            names = ["physxScene:timeStepsPerSecond", "physxScene:solverType", "physxScene:enableCCD", "physxScene:enableStabilization", "physics:gravityMagnitude", "physxScene:broadphaseType", "physxScene:enableGPUDynamics", "physxScene:enableExternalForcesEveryIteration"]
            return {n: str(physics_prim.GetAttribute(n).Get()) if physics_prim.GetAttribute(n).IsValid() else None for n in names}
        result["physics_before_context"] = physics_config()
        if result["physics_before_context"].get("physxScene:solverType") != "TGS" or result["physics_before_context"].get("physxScene:enableExternalForcesEveryIteration") not in ("False", "0"):
            raise Reject("NON_BASELINE_PHYSX_CONFIGURATION")
        if not stage.GetPrimAtPath(CUBE_PATH).IsValid():
            raise Reject("CUBE_IDENTITY")
        graph = stage.GetPrimAtPath(TASK_ROOT + "/ROSGraph")
        if not graph.IsValid() or not graph.IsActive():
            raise Reject("ROS_GRAPH_IDENTITY")
        graph.SetActive(False)
        result["ros_bridge_enabled_before"] = manager.is_extension_enabled("isaacsim.ros2.bridge")
        if result["ros_bridge_enabled_before"]:
            manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", False)
        result["ros_bridge_enabled_after"] = manager.is_extension_enabled("isaacsim.ros2.bridge")
        if result["ros_bridge_enabled_after"]:
            raise Reject("ROS_BRIDGE_STILL_ENABLED")
        result["ros_graph_suppressed"] = not graph.IsActive()
        result["graph_suppression"] = "anonymous stage only; no ROS command sink started"
        if getattr(builtins, "_task28_g1_0_single_cube_bridge", None) is not None:
            raise Reject("POSITION_BRIDGE_PRESENT")

        model = RobotModel()
        collision = CollisionValidator(DEFAULT_FIXTURE)
        result["phase"] = "CONTEXT"
        sim = SimulationContext(physics_prim_path=TASK_ROOT + "/physicsScene", set_defaults=False, backend="numpy", device="cpu")
        result["physics_after_context"] = physics_config()
        result["physics_backend_adaptation"] = {"before": result["physics_before_context"], "after": result["physics_after_context"], "scene_source_unchanged": True, "source": "installed Isaac 4.5 CPU PhysicsContext"}
        result["effective_backend"] = "CPU_NUMPY"
        result["configured_dt_s"] = float(sim.get_physics_dt())
        if abs(result["configured_dt_s"] - cfg["state"]["dt_s"]) > cfg["state"]["dt_tolerance_s"]:
            raise Reject("PHYSICS_DT_CONFIGURATION")
        subs.append(omni.physx.get_physx_interface().subscribe_physics_step_events(lambda dt: count.append(float(dt))))

        result["phase"] = "NATIVE_DRIVES_PLAY"
        sim.play()
        for side, path in (("left", LEFT_ROOT), ("right", RIGHT_ROOT)):
            arm = SingleArticulation(path)
            arm.initialize()
            arms[side] = arm
            views[side] = arm._articulation_view
            if not arm.handles_initialized or list(arm.dof_names) != DOF_NAMES:
                raise Reject("DOF_IDENTITY:" + side)
            links[side] = {}
            for link in ("fr3_link7", "fr3_link8"):
                link_prim = SingleRigidPrim(path + "/" + link)
                link_prim.initialize()
                links[side][link] = link_prim
        cube = SingleRigidPrim(CUBE_PATH)
        cube.initialize()
        local = {}
        for side, sign in (("left", -1), ("right", 1)):
            T = np.eye(4)
            T[:3, :3] = Rotation.from_euler("z", sign * np.pi / 2).as_matrix()
            T[:3, 3] = [0.0, sign * 0.155, 0.08]
            local[side] = T

        def tf(prim):
            p, q = prim.get_world_pose()
            T = np.eye(4)
            T[:3, 3] = np.asarray(p, dtype=float).copy()
            T[:3, :3] = Rotation.from_quat(np.asarray(q)[[1, 2, 3, 0]]).as_matrix()
            return T

        def read():
            obj = tf(cube)
            values = {}
            step = int(sim.current_time_step_index)
            time_s = float(sim.current_time)
            for side in ("left", "right"):
                arm, view = arms[side], views[side]
                q = np.array(arm.get_joint_positions(), dtype=float, copy=True).reshape(-1)
                dq = np.array(arm.get_joint_velocities(), dtype=float, copy=True).reshape(-1)
                link7, link8 = tf(links[side]["fr3_link7"]), tf(links[side]["fr3_link8"])
                tcp = link8 @ local[side]
                kp, kd = (np.asarray(x, dtype=float).reshape(-1) for x in view.get_gains())
                M = np.array(view.get_mass_matrices(), dtype=float, copy=True)[0]
                c = np.array(view.get_coriolis_and_centrifugal_forces(), dtype=float, copy=True)[0]
                g = np.array(view.get_generalized_gravity_forces(), dtype=float, copy=True)[0]
                applied = np.array(arm.get_applied_joint_efforts(), dtype=float, copy=True).reshape(-1)
                measured = np.array(arm.get_measured_joint_efforts(), dtype=float, copy=True).reshape(-1)
                modes = list(view.get_effort_modes()[0])
                fk_m, fk_r = residual(model.fk(side, q[:7]), tcp)
                values[side] = {
                    "identity": side, "names": list(arm.dof_names), "physics_step": step, "time_s": time_s,
                    "dt_s": float(sim.get_physics_dt()), "q": q.tolist(), "dq": dq.tolist(),
                    "M": M.tolist(), "c": c.tolist(), "g": g.tolist(),
                    "link7_world": link7.tolist(), "link8_world": link8.tolist(), "tcp_world": tcp.tolist(),
                    "tcp_frame": "original_side_suction_tcp", "tcp_fk_error_m": float(fk_m), "tcp_fk_error_rad": float(fk_r),
                    "drive_kp": kp.tolist(), "drive_kd": kd.tolist(), "effort_mode": modes[0] if modes else None, "effort_modes": modes,
                    "finger_kp": kp[7:].tolist(), "finger_kd": kd[7:].tolist(), "finger_q": q[7:].tolist(), "finger_dq": dq[7:].tolist(),
                    "finger_targets": np.array(view._physics_view.get_dof_position_targets(), copy=True).reshape(-1)[7:].tolist(),
                    "applied_command_not_sensor": applied.tolist(), "solver_projected_not_sensor": measured.tolist(),
                    "fresh_copy": True, "solver_semantics_ack": True, "position_drives_disabled": bool(np.allclose(kp[:7], 0) and np.allclose(kd[:7], 0)),
                    "finger_policy_ok": True, "within_original_joint_limits": model.within_limits(q[:7]), "competing_sink": bool(manager.is_extension_enabled("isaacsim.ros2.bridge") or getattr(builtins, "_task28_g1_0_single_cube_bridge", None) is not None), "unexpected_contact": False,
                }
            hit = collision.validate(model.forward(np.asarray(values["left"]["q"][:7]), np.asarray(values["right"]["q"][:7])), obj, "detached")
            unexpected = [reason for reason in hit.reasons if reason != "COLLISION:Cube_01/box|task28_table/box"]
            return {"physics_step": step, "time_s": time_s, "timeline_playing": bool(omni.timeline.get_timeline_interface().is_playing()), "cube_world": obj.tolist(), "arms": values, "collision": {"valid": not unexpected, "reasons": list(hit.reasons), "unexpected": unexpected, "checked_pairs": hit.checked_shape_pairs, "fcl_scope": "detached original FCL"}, "state": "FREE_HOME_OR_SETTLING", "suction_commands": 0}

        def step_and_read(previous):
            prior = len(count)
            sim.step(render=False)
            result["explicit_physics_steps"] += 1
            row = read()
            if row["physics_step"] != previous["physics_step"] + 1 or len(count) != prior + 1:
                raise Reject("STEP_CURSOR")
            if abs(row["time_s"] - previous["time_s"] - 1 / 60) > 1e-8 or abs(count[-1] - 1 / 60) > 1e-8:
                raise Reject("PHYSICS_DT")
            for side in ("left", "right"):
                q0 = np.asarray(previous["arms"][side]["q"], float); q1 = np.asarray(row["arms"][side]["q"], float)
                v0 = np.asarray(previous["arms"][side]["dq"], float); v1 = np.asarray(row["arms"][side]["dq"], float)
                dt = row["time_s"] - previous["time_s"]
                row.setdefault("derived", {})[side] = {"delta_q": (q1 - q0).tolist(), "whole_step_fd_dq": ((q1 - q0) / dt).tolist(), "reported_end_dq": v1.tolist(), "fd_minus_reported": ((q1 - q0) / dt - v1).tolist(), "link8_delta_m": float(np.linalg.norm(np.asarray(row["arms"][side]["link8_world"])[:3, 3] - np.asarray(previous["arms"][side]["link8_world"])[:3, 3]))}
            return row

        # Native startup is observed; no state is teleported.  The registered
        # transition is deliberately longer than R5's 150-step window.
        row = read()
        record(row)
        for _ in range(180):
            if not row["collision"]["valid"]:
                raise Reject("UNEXPECTED_COLLISION_DURING_STARTUP")
            next_row = step_and_read(row)
            record(next_row)
            row = next_row
        result["native_settle_final"] = row
        result["free_home_observed"] = all(np.max(np.abs(np.asarray(row["arms"][s]["q"][:7]) - np.asarray(scene_mod.FR3_OFFICIAL_START_Q_RAD))) < 5e-3 for s in ("left", "right"))
        if not result["free_home_observed"]:
            raise Reject("FREE_HOME_NOT_REACHED")
        if not row["collision"]["valid"]:
            raise Reject("CLEARANCE_GATE")

        # Level-1 evidence is a diagnostic certificate; it does not require full ID.
        result["phase"] = "LEVEL1_GATE"
        result["level1_source"] = "PASS"
        level1 = {"status": "DIAGNOSTIC_ACTUATION_SOURCE_READY", "level": 1, "scene": result["scene_before"], "free_home_observed": True, "tgs_semantics": "END_STEP_SOLVER_VELOCITY", "actual_dt_s": result["configured_dt_s"], "finger_policy": "PRESERVED_NATIVE_DRIVES", "no_competing_sink": True}
        level1_path = args.output / "level1_evidence.json"
        write(level1_path, level1)
        cert = DiagnosticCertificate("DIAGNOSTIC_ACTUATION_SOURCE_READY", FR3_SHA256, 1 / 60, "task28_g1_0_new_single_cube_v0", hashlib.sha256(level1_path.read_bytes()).hexdigest(), str(level1_path))
        adapter = R8ExclusiveEffortAdapter(arms, stage, cert, gate, journal, np.asarray(cfg["limits"]["research_effort_abs_max_Nm"], dtype=float), np.asarray(cfg["limits"]["arm_effort_rate_max_Nm_s"], dtype=float))
        adapter.activate(row["arms"])
        row = read()
        record(row)
        if any(row["arms"][s]["position_drives_disabled"] is not True for s in ("left", "right")):
            raise Reject("POSITION_DRIVE_ACTIVE_AFTER_ACTIVATE")
        result["ownership"] = {s: {"arm_kp": row["arms"][s]["drive_kp"][:7], "arm_kd": row["arms"][s]["drive_kd"][:7], "finger_kp": row["arms"][s]["finger_kp"], "finger_kd": row["arms"][s]["finger_kd"], "modes": row["arms"][s]["effort_modes"]} for s in ("left", "right")}

        # Registered controller reference is captured from this actual, source-validated posture.
        q_hold = {s: np.asarray(row["arms"][s]["q"][:7], dtype=float).copy() for s in ("left", "right")}
        result["q_hold_actual"] = {s: q_hold[s].tolist() for s in ("left", "right")}
        dq0 = {s: np.asarray(row["arms"][s]["dq"][:7],float).copy() for s in ("left","right")}
        matrices = {s:gains(row["arms"][s]["M"],reg["strategy"]) for s in ("left","right")}
        result["controller_formula"] = "g(q)+Kp@(q_ref-q)+Kd@(dq_ref-native_end_step_dq)"
        result["controller"] = reg["strategy"]
        result["gains"] = {s:{"Kp":matrices[s][0].tolist(),"Kd":matrices[s][1].tolist(),"M0":row["arms"][s]["M"],"dq0":dq0[s].tolist()} for s in ("left","right")}
        result["ownership_initial_state"] = row
        def safety(current):
            if not current["collision"]["valid"]:
                raise Reject("UNEXPECTED_COLLISION")
            for side in ("left","right"):
                state=current["arms"][side]
                if not state["within_original_joint_limits"]: raise Reject("ORIGINAL_JOINT_LIMIT")
                if np.max(np.abs(np.asarray(state["q"][:7])-q_hold[side])) > .02: raise Reject("SAFETY_JOINT_DRIFT")
                if np.max(np.abs(np.asarray(state["dq"][:7]))) > .3: raise Reject("SAFETY_NATIVE_DQ")
                trans,rot=residual(np.asarray(row["arms"][side]["tcp_world"]), np.asarray(state["tcp_world"]))
                if trans > .01 or rot > .05: raise Reject("SAFETY_TCP_ENVELOPE")
                for key in ("finger_kp","finger_kd","finger_targets"):
                    if state[key] != row["arms"][side][key]: raise Reject("FINGER_POLICY_CHANGED:"+key)
        safety(row)
        result["phase"] = "CLOSED_LOOP"
        result["command_sequence"] = []
        previous = row
        steps = int(reg["steps"])
        for k in range(steps):
            safety(previous)
            effort, components = {}, {}
            for side in ("left","right"):
                qref,dqref=reference(q_hold[side],dq0[side],k/60,reg["strategy"])
                effort[side],components[side]=torque(previous["arms"][side],qref,dqref,*matrices[side])
            journal({"event":"CONTROLLER_COMPONENTS","index":k,"physics_step":previous["physics_step"],"components":components})
            full = adapter.apply_pair(previous["arms"], effort, 1 / 60)
            result["actual_effort_commands"] += 1; result["robot_command_writes"] += 2
            next_row = step_and_read(previous)
            next_row["control_components_pre_step"] = components
            record(next_row)
            adapter.post_step(next_row["arms"])
            safety(next_row)
            for side in ("left", "right"):
                requested=np.asarray(full[side],float); applied=np.asarray(next_row["arms"][side]["applied_command_not_sensor"],float)
                if not np.allclose(requested, applied, atol=1e-6, rtol=0): raise Reject("APPLIED_READBACK_MISMATCH:"+side)
                next_row["arms"][side]["requested_full_effort"]=requested.tolist(); next_row["arms"][side]["applied_readback_match"]=True
                if next_row["arms"][side]["finger_kp"] != row["arms"][side]["finger_kp"] or next_row["arms"][side]["finger_kd"] != row["arms"][side]["finger_kd"]: raise Reject("FINGER_POLICY_CHANGED")
            result["command_sequence"].append({"index":k,"physics_step":next_row["physics_step"],"effort_arm":{s:np.asarray(full[s])[:7].tolist() for s in ("left","right")},"components":components})
            previous=next_row
        result["response"] = {s: {"delta_q_arm": (np.asarray(previous["arms"][s]["q"][:7]) - np.asarray(row["arms"][s]["q"][:7])).tolist(), "delta_link8_m": float(np.linalg.norm(np.asarray(previous["arms"][s]["link8_world"])[:3, 3] - np.asarray(row["arms"][s]["link8_world"])[:3, 3])), "delta_tcp_m": float(np.linalg.norm(np.asarray(previous["arms"][s]["tcp_world"])[:3, 3] - np.asarray(row["arms"][s]["tcp_world"])[:3, 3]))} for s in ("left", "right")}
        result["status"] = "FORMAL_STATIC_HOLD_COMPLETE" if args.configuration == "FORMAL" else "DIAGNOSTIC_STABILIZATION_COMPLETE"
        result["level2_source"] = "CANDIDATE_PENDING_EVALUATION"
        result["phase"] = "COMPLETE"
    except BaseException as exc:
        result["status"] = "REJECTED"
        result["first_failure"] = {"phase": result["phase"], "type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc(), "last_frame_index": rows - 1}
        if adapter is not None:
            result["level2_source"] = "BLOCKED_AFTER_FAILURE"
    finally:
        save()
        lifecycle("RESULT_SAVED", status=result["status"], recorded_frames=rows)
        if app is not None:
            try:
                omni.timeline.get_timeline_interface().stop()
                lifecycle("TIMELINE_STOPPED")
            except Exception as exc:
                lifecycle("TIMELINE_STOP_ERROR", error=str(exc))
        for sub in subs:
            try:
                sub.unsubscribe()
            except Exception:
                pass
        if app is not None:
            lifecycle("APP_CLOSE_BEGIN")
            app.close()
            lifecycle("APP_CLOSE_RETURNED")
    return 0


if __name__ == "__main__":
    code=main()
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)  # parent still verifies worker and owned-group exit; no destructor linger
