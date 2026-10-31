#!/usr/bin/env python3
"""GUI runner for the Task27 torque integration attempt.

``--mode smoke`` exercises the original Task27 feed bridge plus exclusive arm
effort ownership without ROS planning.  ``--mode integrated`` starts the
existing MoveIt and Task27 five-Cube executor in the same fresh GUI scene.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
ISAAC_ROOT = Path("/home/ubuntu2004/isaacsim-4.5.0")
SCENE = REPO / "isaac/scripts/task27_five_cube_center_insert_scene.py"
TORQUE_BRIDGE = REPO / "isaac/scripts/task28_task27_torque_bridge.py"
OUT_ROOT = REPO / "offline/task28_task27_integration/evidence"
# Isaac Kit mutates PYTHONPATH/LD_LIBRARY_PATH after SimulationApp starts.
# ROS subprocesses must inherit the clean pre-Kit environment instead.
BASE_ROS_ENV = os.environ.copy()

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("TASK28_T0_GUI", "1")
os.environ.setdefault("ROS_DISTRO", "humble")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
_isaac_ros_lib = ISAAC_ROOT / "exts/isaacsim.ros2.bridge/humble/lib"
_ld_parts = [part for part in os.environ.get("LD_LIBRARY_PATH", "").split(":") if part]
for _required_lib in (Path("/opt/ros/humble/lib/x86_64-linux-gnu"),
                      Path("/opt/ros/humble/lib"), _isaac_ros_lib):
    if _required_lib.is_dir() and str(_required_lib) not in _ld_parts:
        _ld_parts.append(str(_required_lib))
if _ld_parts:
    os.environ["LD_LIBRARY_PATH"] = ":".join(_ld_parts)
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_scene():
    namespace = {"__name__": "__task28_task27_selected_scene__"}
    source = SCENE.read_text(encoding="utf-8")
    exec(compile(source, str(SCENE), "exec"), namespace, namespace)
    if "build_scene" in namespace:
        namespace["build_scene"]()
    return namespace


def enable_lighting():
    import omni.kit.commands
    result = omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default", usd_context_name="")
    if not result or result[0] is not True:
        raise RuntimeError(f"VIEWPORT_LIGHTING_DEFAULT_REJECTED:{result!r}")
    return result


def capture(app, viewport, path):
    from omni.kit.viewport.utility import capture_viewport_to_file
    path.parent.mkdir(parents=True, exist_ok=True)
    capture_viewport_to_file(viewport, file_path=str(path), is_hdr=False)
    for _ in range(15):
        app.update()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"VIEWPORT_CAPTURE_MISSING:{path}")


def start_process(command: str, log_path: Path, cwd: Path):
    log_path.parent.mkdir(parents=True, exist_ok=True)
    env = BASE_ROS_ENV.copy()
    env["ROS_LOCALHOST_ONLY"] = "0"
    env.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    # MoveIt and the C++ executor use system ROS libraries.  The Isaac ROS
    # bridge library is already loaded in Kit and must not contaminate these
    # subprocesses with Kit's bundled Python/spdlog ABI.
    env["PYTHONPATH"] = "/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
    env.pop("PYTHONHOME", None)
    env["LD_LIBRARY_PATH"] = "/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib"
    env["PATH"] = "/opt/ros/humble/bin:/usr/bin:/bin"
    stream = log_path.open("w", encoding="utf-8")
    overlay = Path("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/ros_ws/install/setup.bash")
    overlay_cmd = f"source {overlay} && " if overlay.is_file() else ""
    proc = subprocess.Popen(
        ["bash", "-lc", f"source /opt/ros/humble/setup.bash && {overlay_cmd}source install/setup.bash && {command}"],
        cwd=str(cwd), env=env, stdout=stream, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    proc._task28_log_stream = stream
    return proc


def stop_process(proc):
    if proc is None:
        return None
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=8)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass
    stream = getattr(proc, "_task28_log_stream", None)
    if stream is not None:
        stream.close()
    return proc.returncode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("smoke", "integrated"), default="smoke")
    parser.add_argument("--frames", type=int, default=240)
    parser.add_argument("--timeout-s", type=float, default=600.0)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--planning-only", action="store_true",
                        help="Run the Task27 executor's strict zero-command planning gate.")
    parser.add_argument("--max-batches", type=int, default=5)
    parser.add_argument("--small-motion", action="store_true",
                        help="Run a bounded 5 mrad arm target and return-to-home in the torque bridge without ROS/Cube commands.")
    parser.add_argument("--live-qp", action="store_true",
                        help="Enable the live bilateral acceleration/torque QP for no-contact smoke experiments.")
    parser.add_argument("--experiment", choices=("hold", "common", "relative", "combined"), default="hold")
    args = parser.parse_args()
    run_id = args.run_id or time.strftime("issue1_%Y%m%d_%H%M%S")
    out = OUT_ROOT / run_id
    out.mkdir(parents=True, exist_ok=False)
    torque_log = out / "task27_torque_bridge.jsonl"
    summary = {
        "schema": "task28.task27.one_pass.v1",
        "run_id": run_id,
        "mode": args.mode,
        "planning_only": bool(args.planning_only),
        "max_batches": int(args.max_batches),
        "small_motion": bool(args.small_motion),
        "live_qp": bool(args.live_qp),
        "experiment": args.experiment,
        "headless": False,
        "scene_entry": str(SCENE),
        "scene_entry_sha256": sha256(SCENE),
        "original_task26_scene_source": "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_scene.py",
        "original_task26_scene_source_sha256": sha256(Path("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_scene.py")),
        "original_task26_bridge_source": "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_bridge.py",
        "original_task26_bridge_source_sha256": sha256(Path("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/task26_truck_box_bridge.py")),
        "scene_root": "/World/Task27",
        "arm_roots": ["/World/left_fr3", "/World/right_fr3"],
        "torque_log": str(torque_log),
        "moveit_exit": None,
        "executor_exit": None,
        "status": "RUNNING",
    }
    app = None
    torque = None
    moveit = executor = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": False, "create_new_stage": True, "renderer": "RaytracedLighting", "multi_gpu": False, "anti_aliasing": 2, "sync_loads": True})
        import omni.timeline
        import omni.usd
        from pxr import UsdGeom
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        ctx = omni.usd.get_context()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        load_scene()
        summary["lighting"] = repr(enable_lighting())
        set_camera_view(eye=[2.8, -3.2, 2.3], target=[0.92, 0.0, 0.38], camera_prim_path="/OmniverseKit_Persp")
        viewport = get_active_viewport()
        if viewport is not None:
            try:
                viewport.render_mode = "RaytracedLighting"
            except Exception:
                pass
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(120):
            app.update()
        capture(app, viewport, out / "scene_loaded.png")
        spec = importlib.util.spec_from_file_location("task28_task27_torque_bridge", TORQUE_BRIDGE)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        torque = module.start_task27_torque_bridge(
            output=torque_log, live_qp=bool(args.live_qp), experiment=args.experiment
        )
        for _ in range(60):
            app.update()
        if args.mode == "integrated":
            ros_ws = REPO / "ros_ws"
            moveit = start_process("ros2 launch fr3_dual_side_suction_description moveit_dual_side_suction.launch.py use_rviz:=false", out / "moveit.log", ros_ws)
            summary["moveit_started"] = True
            deadline = time.monotonic() + 25.0
            while time.monotonic() < deadline and moveit.poll() is None:
                app.update()
                time.sleep(0.02)
            if moveit.poll() is not None:
                raise RuntimeError(f"MOVEIT_EXIT_EARLY:{moveit.returncode}")
            planning_flag = " -p planning_only:=true" if args.planning_only else ""
            if args.max_batches < 1 or args.max_batches > 5:
                raise ValueError("max-batches must be in 1..5")
            executor = start_process("ros2 run fr3_dual_palletize task27_five_cube_center_insert --ros-args -p first_batch:=1 -p max_batches:=" + str(args.max_batches) + " -p execution_time_scale:=3.0" + planning_flag, out / "task27_executor.log", ros_ws)
            summary["executor_started"] = True
            start = time.monotonic()
            while executor.poll() is None and time.monotonic() - start < args.timeout_s:
                app.update()
                if getattr(torque, "_latched_error", None):
                    break
                time.sleep(0.01)
            summary["timeout_reached"] = executor.poll() is None
        else:
            if args.small_motion:
                # This is a no-contact controller diagnostic.  It injects two
                # timestamped target captures through the same patched sink
                # used by ROS, while leaving the original feed/gripper/rail
                # owners untouched and never publishing a ROS command.
                names = tuple(module.ARM_DOF_NAMES)
                stamp0 = (0, 0)
                for side, sign in (("left", 1.0), ("right", -1.0)):
                    target = torque._q0[side].copy()
                    target[0] += sign * 0.005
                    torque.feed._apply_joint_command(side, (stamp0, names, target))
            for index in range(max(1, args.frames)):
                if args.small_motion and index == max(1, args.frames) // 2:
                    names = tuple(module.ARM_DOF_NAMES)
                    for side in ("left", "right"):
                        torque.feed._apply_joint_command(
                            side, ((1, 0), names, torque._q0[side].copy())
                        )
                app.update()
        capture(app, viewport, out / "final.png")
        if getattr(torque, "_latched_error", None):
            summary["status"] = "TORQUE_BRIDGE_FAIL_CLOSED"
        elif args.live_qp and int(getattr(torque, "_qp_solve_count", 0) or 0) <= 0:
            summary["status"] = "LIVE_QP_NO_SOLVES"
        else:
            summary["status"] = "COMPLETED"
    except Exception as exc:
        summary["status"] = "BLOCKED_RUNTIME_EXCEPTION"
        summary["exception"] = repr(exc)
    finally:
        summary["torque_latched_error"] = getattr(torque, "_latched_error", None)
        summary["torque_steps"] = getattr(torque, "_step", None)
        summary["torque_writes"] = getattr(torque, "_writes", None)
        summary["position_arm_write_count"] = getattr(torque, "_position_arm_write_count", None)
        summary["qp_solve_count"] = getattr(torque, "_qp_solve_count", None)
        summary["qp_status_counts"] = getattr(torque, "_qp_status_counts", None)
        summary["qp_stop_reason"] = getattr(torque, "_qp_stop_reason", None)
        summary["moveit_exit"] = stop_process(moveit)
        summary["executor_exit"] = stop_process(executor)
        # Persist the endpoint status before closing Kit; Isaac shutdown can
        # terminate extension callbacks without returning control to Python.
        (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if torque is not None:
            try:
                torque.close()
            except Exception as exc:
                summary["torque_close_error"] = repr(exc)
        if app is not None:
            try:
                omni.timeline.get_timeline_interface().stop()
                app.update()
                app.close()
            except Exception as exc:
                summary["app_close_error"] = repr(exc)
        (out / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["status"] == "COMPLETED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
