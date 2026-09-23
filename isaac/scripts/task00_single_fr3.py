#!/usr/bin/env python3
"""Build and run Task 0's minimal arm-only FR3 Isaac Sim scene.

Run this from an Isaac-only clean terminal.  Isaac Sim 4.5 is built with
Python 3.10 and its ROS 2 Humble bridge loads the matching system ROS client
libraries.  This project script intentionally does not import ``rclpy``;
all ROS traffic is handled inside Isaac's ROS 2 Bridge Action Graph.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from datetime import datetime, timezone
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = PROJECT_ROOT / "isaac" / "scenes" / "task00_single_fr3.usd"
DEFAULT_REPORT = PROJECT_ROOT / "artifacts" / "task00" / "isaac_runtime_report.json"
URDF_PATH = PROJECT_ROOT / "isaac" / "assets" / "fr3_no_hand.urdf"
FRANKA_SHARE_PARENT = PROJECT_ROOT / "ros_ws" / "install" / "franka_description" / "share"
FR3_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
READY = np.asarray([0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim GUI.")
    parser.add_argument(
        "--run-seconds",
        type=float,
        default=600.0,
        help="Maximum wall-clock time to keep simulation playback running.",
    )
    parser.add_argument("--scene-path", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--report-path", type=Path, default=DEFAULT_REPORT)
    return parser.parse_args()


def _write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _add_table(stage, usd_geom, usd_physics, gf, sdf) -> None:
    usd_geom.Xform.Define(stage, sdf.Path("/World"))
    table = usd_geom.Cube.Define(stage, sdf.Path("/World/Worktable"))
    table.CreateSizeAttr(1.0)
    table.AddTranslateOp().Set(gf.Vec3d(0.75, 0.0, 0.38))
    table.AddScaleOp().Set(gf.Vec3f(1.20, 0.80, 0.76))
    usd_physics.CollisionAPI.Apply(table.GetPrim())


def _configure_joint_drives(stage, usd_physics) -> int:
    """Use stable position drives for named FR3 revolute joints."""
    configured = 0
    for prim in stage.Traverse():
        if prim.GetName() not in FR3_JOINTS or not prim.IsA(usd_physics.RevoluteJoint):
            continue
        drive = usd_physics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = usd_physics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr().Set(4000.0)
        drive.CreateDampingAttr().Set(400.0)
        configured += 1
    if configured != len(FR3_JOINTS):
        raise RuntimeError("Expected seven FR3 revolute drives, configured %d" % configured)
    return configured


def _build_action_graph(og, usdrt, articulation_path: str) -> None:
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishJointState.inputs:topicName", "/joint_states"),
                ("SubscribeJointState.inputs:topicName", "/joint_command"),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(articulation_path)]),
                ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(articulation_path)]),
                ("PublishTF.inputs:topicName", "/tf"),
                ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path(articulation_path)]),
            ],
        },
    )


def main() -> int:
    args = parse_args()
    report = {
        "task": "Task 0 minimal single FR3 Isaac scene",
        "utc": datetime.now(timezone.utc).isoformat(),
        "gui_requested": not args.headless,
        "project_imports_rclpy": False,
        "ros2_bridge_runtime": "Isaac Sim 4.5 ROS 2 Bridge with ROS 2 Humble ABI",
        "success": False,
    }
    app = None
    try:
        if not URDF_PATH.is_file():
            raise FileNotFoundError("Generated arm-only URDF is missing: %s" % URDF_PATH)
        if not FRANKA_SHARE_PARENT.is_dir():
            raise FileNotFoundError("Project FR3 package must be built first: %s" % FRANKA_SHARE_PARENT)

        # URDF package:// meshes are asset paths, not Python ROS imports.
        os.environ["ROS_PACKAGE_PATH"] = str(FRANKA_SHARE_PARENT)

        from isaacsim import SimulationApp

        # This scene has no cameras or synthetic-data workload.  Disabling
        # DLSS and multi-GPU avoids an unnecessary first-launch RTX shader
        # warm-up on the single 8 GB RTX 3070 while retaining RTX Real-Time
        # rendering for the GUI validation.
        app = SimulationApp(
            {
                "headless": args.headless,
                "renderer": "RaytracedLighting",
                "anti_aliasing": 0,
                "multi_gpu": False,
                "sync_loads": False,
                "fast_shutdown": True,
            }
        )

        import carb
        import omni.graph.core as og
        import omni.kit.commands
        import omni.timeline
        import omni.usd
        import usdrt.Sdf
        from isaacsim.core.prims import Articulation
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

        enable_extension("isaacsim.asset.importer.urdf")
        enable_extension("isaacsim.ros2.bridge")
        carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/publish_without_verification", True)
        for _ in range(12):
            app.update()

        omni.usd.get_context().new_stage()
        status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("URDFCreateImportConfig failed")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = True
        import_config.fix_base = True
        import_config.self_collision = False
        import_config.distance_scale = 1.0
        import_config.collision_from_visuals = False
        import_config.create_physics_scene = False

        status, articulation_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=str(URDF_PATH),
            import_config=import_config,
            get_articulation_root=True,
        )
        if not status or not articulation_path:
            raise RuntimeError("URDFParseAndImportFile failed: %s" % articulation_path)
        articulation_path = str(articulation_path)
        report["articulation_path"] = articulation_path

        stage = omni.usd.get_context().get_stage()
        scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
        physx_scene.CreateEnableCCDAttr(True)
        physx_scene.CreateEnableStabilizationAttr(True)
        physx_scene.CreateEnableGPUDynamicsAttr(False)
        physx_scene.CreateBroadphaseTypeAttr("MBP")
        physx_scene.CreateSolverTypeAttr("TGS")

        PhysicsSchemaTools.addGroundPlane(
            stage,
            "/World/Ground",
            "Z",
            1500.0,
            Gf.Vec3f(0.0, 0.0, -0.01),
            Gf.Vec3f(0.35),
        )
        _add_table(stage, UsdGeom, UsdPhysics, Gf, Sdf)
        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/KeyLight"))
        light.CreateIntensityAttr(1200.0)
        _configure_joint_drives(stage, UsdPhysics)
        _build_action_graph(og, usdrt, articulation_path)
        for _ in range(8):
            app.update()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(24):
            app.update()
        articulation = Articulation(articulation_path)
        articulation.initialize()
        if not articulation.is_physics_handle_valid():
            raise RuntimeError("Imported FR3 is not a valid Articulation: %s" % articulation_path)
        dof_names = list(articulation.dof_names)
        missing = sorted(set(FR3_JOINTS) - set(dof_names))
        if missing:
            raise RuntimeError("FR3 articulation is missing expected DOFs: %s" % missing)
        positions = np.asarray(articulation.get_joint_positions(), dtype=np.float64)
        for name, value in zip(FR3_JOINTS, READY):
            positions[dof_names.index(name)] = value
        articulation.set_joint_positions(positions)
        articulation.set_joint_position_targets(positions)
        report["dof_names"] = dof_names
        report["initial_positions_rad"] = {name: float(value) for name, value in zip(FR3_JOINTS, READY)}

        args.scene_path.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(args.scene_path))
        if not args.scene_path.is_file() or args.scene_path.stat().st_size == 0:
            raise RuntimeError("Failed to save scene: %s" % args.scene_path)
        report["scene_path"] = str(args.scene_path)
        report["scene_size_bytes"] = args.scene_path.stat().st_size
        report["graph_topics"] = {"clock": "/clock", "joint_states": "/joint_states", "joint_command": "/joint_command", "tf": "/tf"}
        report["status"] = "playback_running"
        _write_report(args.report_path, report)
        print("[TASK00] Isaac Sim initialized; playback running.", flush=True)
        print("[TASK00] articulation=%s, dofs=%s" % (articulation_path, dof_names), flush=True)
        print("[TASK00] ROS traffic is owned by Isaac Sim 4.5 ROS 2 Bridge.", flush=True)

        deadline = time.monotonic() + max(0.0, args.run_seconds)
        while app.is_running() and time.monotonic() < deadline:
            app.update()

        report["status"] = "completed_runtime_window"
        report["success"] = True
        _write_report(args.report_path, report)
        return 0
    except KeyboardInterrupt:
        report["status"] = "interrupted"
        _write_report(args.report_path, report)
        return 130
    except Exception as exc:
        report["error"] = str(exc)
        report["traceback"] = traceback.format_exc()
        _write_report(args.report_path, report)
        print("[TASK00][ERROR] %s" % exc, file=sys.stderr, flush=True)
        return 1
    finally:
        if app is not None:
            try:
                import omni.timeline

                omni.timeline.get_timeline_interface().stop()
                app.close()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
