#!/usr/bin/env python3
"""Fresh Isaac process probe for the existing Task28 single-Cube fixture.

The probe calls the frozen fixture builder; it does not recreate geometry and
does not import or start the legacy ROS bridge.  The two bounded runs are
static explicit effort and short no-contact torque control. Suction remains
telemetry-only.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import sys
import traceback
from pathlib import Path

import numpy as np

from .common import (
    ARM_DOF_NAMES,
    BRIDGE_CONTRACT_PATH,
    CUBE_PATH,
    DOF_NAMES,
    FIXTURE,
    FIXTURE_ID,
    FR3_ASSET,
    FR3_SHA256,
    LEFT_ROOT,
    LINK8_PATHS,
    PHYSICS_DT,
    RIGHT_ROOT,
    TASK_ROOT,
    TCP_PATHS,
    TORQUE_SYNC_FRAME,
    dump_json,
    load_json,
    serialise_manifest_hashes,
    sha256,
)
from .suction_interface import SuctionStateInterface
from .torque_adapter import (
    ActuatorOwnership,
    coordinated_effort,
    synchronized_effort_messages,
    validate_effort_payload,
    validate_snapshot,
)


def _import_fixture():
    spec = importlib.util.spec_from_file_location(
        "task28_g1_0_frozen_scene_for_r2", FIXTURE / "scene/task28_single_cube_scene.py"
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _op_values(xform):
    values = {}
    for op in xform.GetOrderedXformOps():
        values[str(op.GetOpName())] = op.Get()
    return values


def _vec(value):
    return [float(value[0]), float(value[1]), float(value[2])]


def _runtime_identity(stage, fixture_module):
    from pxr import PhysxSchema, UsdPhysics, UsdGeom, Usd

    failures = []
    root = stage.GetPrimAtPath(TASK_ROOT)
    if not root.IsValid() or root.GetCustomDataByKey("task28_g1_0_fixture_id") != FIXTURE_ID:
        failures.append("ROOT_OR_FIXTURE_ID")
    expected_paths = [TASK_ROOT, LEFT_ROOT, RIGHT_ROOT, CUBE_PATH]
    for path in expected_paths:
        if not stage.GetPrimAtPath(path).IsValid():
            failures.append("MISSING:" + path)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    def world_matrix(path):
        mat = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
        return [[float(mat[i][j]) for j in range(4)] for i in range(4)]
    observations = {"prim_paths": expected_paths, "tools": {}, "fingers": {},
                    "cube_world_transform_usd_row_vector": world_matrix(CUBE_PATH)}
    for side, sign in (("left", -1.0), ("right", 1.0)):
        tool = LINK8_PATHS[side] + "/side_suction_tool"
        tcp = stage.GetPrimAtPath(TCP_PATHS[side])
        values = _op_values(UsdGeom.Xformable(tcp)) if tcp.IsValid() else {}
        translation = values.get("xformOp:translate")
        rz = values.get("xformOp:rotateZ")
        expected = [0.0, sign * 0.155, 0.080]
        if translation is None or max(abs(a - b) for a, b in zip(_vec(translation), expected)) > 1e-9:
            failures.append("TCP_TRANSLATION:" + side)
        if rz is None or abs(float(rz) - sign * 90.0) > 1e-9:
            failures.append("TCP_LOCAL_RZ:" + side)
        cup_paths = [f"{tool}/cup_r{row}_c{col}" for row in (1, 2) for col in (1, 2)]
        if not all(stage.GetPrimAtPath(path).IsValid() for path in cup_paths):
            failures.append("CUP_COUNT_OR_PATH:" + side)
        if not stage.GetPrimAtPath(tool + "/vertical_support").IsValid():
            failures.append("VERTICAL_SUPPORT:" + side)
        if not stage.GetPrimAtPath(tool + "/lateral_support").IsValid():
            failures.append("LATERAL_SUPPORT:" + side)
        if not stage.GetPrimAtPath(tool + "/vacuum_manifold").IsValid():
            failures.append("MANIFOLD:" + side)
        observations["tools"][side] = {
            "tool": tool,
            "tcp": TCP_PATHS[side],
            "local_translation_m": _vec(translation) if translation is not None else None,
            "local_rz_deg": float(rz) if rz is not None else None,
            "cups": cup_paths,
            "link8_world_transform_usd_row_vector": world_matrix(LINK8_PATHS[side]),
            "tcp_world_transform_usd_row_vector": world_matrix(TCP_PATHS[side]),
            "vertical_scale_m": _vec(_op_values(UsdGeom.Xformable(stage.GetPrimAtPath(tool + "/vertical_support")))["xformOp:scale"]),
            "lateral_scale_m": _vec(_op_values(UsdGeom.Xformable(stage.GetPrimAtPath(tool + "/lateral_support")))["xformOp:scale"]),
            "manifold_scale_m": _vec(_op_values(UsdGeom.Xformable(stage.GetPrimAtPath(tool + "/vacuum_manifold")))["xformOp:scale:task28_g1_0_size"]),
        }
        hand = LINK8_PATHS[side].rsplit("/", 1)[0] + "/fr3_hand"
        finger2 = stage.GetPrimAtPath(hand + "/fr3_finger_joint2")
        try:
            mimic = PhysxSchema.PhysxMimicJointAPI(finger2, "rotX")
            targets = [str(x) for x in mimic.GetReferenceJointRel().GetTargets()]
            gearing = float(mimic.GetGearingAttr().Get())
            offset = float(mimic.GetOffsetAttr().Get())
        except Exception as error:
            failures.append("MIMIC_SCHEMA:" + side)
            targets, gearing, offset = [], None, None
        limits = []
        for index in (1, 2):
            prim = stage.GetPrimAtPath(hand + f"/fr3_finger_joint{index}")
            try:
                joint = UsdPhysics.PrismaticJoint(prim)
                limits.append({"lower_m": float(joint.GetLowerLimitAttr().Get()), "upper_m": float(joint.GetUpperLimitAttr().Get())})
            except Exception:
                limits.append({"lower_m": None, "upper_m": None})
                failures.append("FINGER_LIMITS:" + side)
        if gearing != -1.0 or offset != 0.0 or targets != [hand + "/fr3_finger_joint1"]:
            failures.append("MIMIC_IDENTITY:" + side)
        observations["fingers"][side] = {
            "joint_names": [hand + "/fr3_finger_joint1", hand + "/fr3_finger_joint2"],
            "mimic_gearing": gearing,
            "mimic_offset_m": offset,
            "mimic_reference": targets,
            "limits": limits,
            "visual_collision_policy": "hidden_by_frozen_fixture",
            "native_state_retained": True,
        }
    physics = stage.GetPrimAtPath(TASK_ROOT + "/physicsScene")
    try:
        observations["physics"] = {
            "time_steps_per_second": float(physics.GetAttribute("physxScene:timeStepsPerSecond").Get()),
            "gravity_magnitude": float(physics.GetAttribute("physics:gravityMagnitude").Get()),
            "solver": str(physics.GetAttribute("physxScene:solverType").Get()),
        }
    except Exception as error:
        failures.append("PHYSICS_RUNTIME_READ")
        observations["physics"] = {"error": str(error)}
    observations["asset"] = {
        "path": str(FR3_ASSET),
        "sha256": sha256(FR3_ASSET),
        "expected_sha256": FR3_SHA256,
        "match": sha256(FR3_ASSET) == FR3_SHA256,
    }
    if not observations["asset"]["match"]:
        failures.append("FR3_ASSET_HASH")
    observations["failures"] = failures
    observations["status"] = "SCENE_IDENTITY_VERIFIED" if not failures else "SCENE_IDENTITY_REJECTED"
    return observations


def run_probe(output: Path) -> dict:
    """Build and inspect the original scene with timeline stopped; no commands."""
    os.environ["TASK28_FR3_USD"] = str(FR3_ASSET)
    os.environ["TASK28_FR3_USD_SHA256"] = FR3_SHA256
    result = {
        "task": "Task28 G2-P5-QP-6-R2",
        "source": "frozen Task28 scene builder",
        "status": "RUNTIME_SCENE_BUILD_REJECTED",
        "fresh_process": True, "timeline_played": False,
        "physics_steps": 0, "robot_commands": 0,
        "legacy_bridge_started": False, "suction_activation_attempted": False,
        "ros_graph_executed": False,
        "source_hashes_before": serialise_manifest_hashes(),
    }
    app = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False,
                             "anti_aliasing": 0, "sync_loads": False, "fast_shutdown": True})
        import omni.usd
        import omni.timeline
        from pxr import UsdGeom
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        # Fresh anonymous stage only: explicitly set required construction metadata,
        # before invoking the original builder; no user's stage is touched.
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        result["fresh_stage_metadata"] = {"up_axis": str(UsdGeom.GetStageUpAxis(stage)),
                                           "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage))}
        if omni.timeline.get_timeline_interface().is_playing():
            raise RuntimeError("TIMELINE_UNEXPECTEDLY_PLAYING")
        scene_module = _import_fixture()
        scene_module.build_scene()
        result["scene"] = _runtime_identity(stage, scene_module)
        result["status"] = result["scene"]["status"]
        result["source_hashes_after"] = serialise_manifest_hashes()
        result["source_hashes_match"] = result["source_hashes_after"] == result["source_hashes_before"]
    except BaseException as error:
        result["exception"] = {"type": type(error).__name__, "message": str(error),
                                "traceback": traceback.format_exc()}
    finally:
        dump_json(output, result)
        if app is not None:
            app.close()
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = run_probe(args.output)
    return 0 if result["status"] == "SCENE_IDENTITY_VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
