"""Static, source-bound identity checks for the original Task28 fixture.

This module intentionally does not import Isaac, ROS, MoveIt, or any bridge.
It verifies the frozen source contracts and records a provenance map before a
fresh runtime stage is attempted.
"""

from __future__ import annotations

import math
import re
from pathlib import Path

from .common import (
    BRIDGE_CONTRACT_PATH,
    FIXTURE,
    FIXTURE_ID,
    FR3_ASSET,
    FR3_SHA256,
    MANIFEST_PATH,
    REPO,
    SCENE_CONTRACT_PATH,
    load_json,
    serialise_manifest_hashes,
    sha256,
)


def _close(a, b, tol=1e-12):
    return len(a) == len(b) and all(abs(float(x) - float(y)) <= tol for x, y in zip(a, b))


def inspect_static() -> dict:
    manifest = load_json(MANIFEST_PATH)
    scene = load_json(SCENE_CONTRACT_PATH)
    bridge = load_json(BRIDGE_CONTRACT_PATH)
    failures: list[str] = []

    if manifest.get("fixture_id") != FIXTURE_ID:
        failures.append("FIXTURE_ID")
    if manifest.get("physical_validation") is not False:
        failures.append("FIXTURE_PHYSICAL_VALIDATION_FLAG")
    if not FR3_ASSET.is_file() or sha256(FR3_ASSET) != FR3_SHA256:
        failures.append("FR3_ASSET_HASH")
    if scene.get("scene_namespace") != "/World/Task28G10SingleCube":
        failures.append("SCENE_NAMESPACE")
    if scene.get("cube_paths") != ["/World/Task28G10SingleCube/Supply/Cube_01"]:
        failures.append("CUBE_IDENTITY_ORDER")
    if scene.get("active_cube_count") != 1 or scene.get("cube_ids") != ["Cube_01"]:
        failures.append("CUBE_COUNT")
    if scene.get("physics", {}).get("steps_per_second") != 60:
        failures.append("PHYSICS_RATE")
    if scene.get("stage_requirements", {}).get("meters_per_unit") != 1.0:
        failures.append("METERS_PER_UNIT")
    if scene.get("stage_requirements", {}).get("up_axis") != "Z":
        failures.append("UP_AXIS")
    if bridge.get("suction_contract") != manifest.get("suction"):
        failures.append("SUCTION_CONTRACT_DRIFT")
    if bridge.get("tcp_contract") != manifest.get("tcp"):
        failures.append("TCP_CONTRACT_DRIFT")

    tcp = manifest["tcp"]
    if not _close(tcp["left_link8_translation_m"], [0.0, -0.155, 0.080]):
        failures.append("LEFT_LINK8_TCP")
    if not _close(tcp["right_link8_translation_m"], [0.0, 0.155, 0.080]):
        failures.append("RIGHT_LINK8_TCP")
    if not _close(tcp["left_link8_rpy_rad"], [0.0, 0.0, -math.pi / 2]):
        failures.append("LEFT_TCP_RZ")
    if not _close(tcp["right_link8_rpy_rad"], [0.0, 0.0, math.pi / 2]):
        failures.append("RIGHT_TCP_RZ")

    scene_source = (FIXTURE / "scene/task28_single_cube_scene.py").read_text()
    bridge_source = (FIXTURE / "bridge/task28_single_cube_bridge.py").read_text()
    xacro = (REPO / "ros_ws/src/fr3_dual_side_suction_description/urdf/dual_side_suction.xacro").read_text()
    required_scene_tokens = [
        'TASK_ROOT = "/World/Task28G10SingleCube"',
        'PHYSICS_STEPS_PER_SECOND = 60',
        'tool}/side_suction_tcp',
        'Gf.Vec3d(0.0, branch_sign * TCP_Y, VERTICAL_DROP_Z)',
        'tcp_xform.AddRotateZOp().Set(branch_sign * 90.0)',
    ]
    for token in required_scene_tokens:
        if token not in scene_source:
            failures.append("SCENE_SOURCE:" + token)
    required_bridge_tokens = [
        "start_bridge(enable_robot_commands=False)",
        "Surface_Gripper_Properties",
        "props.gripThreshold = 0.003",
        "props.forceLimit = RIGID_GRASP_FORCE_LIMIT",
        "props.bendAngle = 0.0",
        "props.stiffness = GRASP_STIFFNESS[side]",
        "props.damping = GRASP_DAMPING[side]",
    ]
    for token in required_bridge_tokens:
        if token not in bridge_source:
            failures.append("BRIDGE_SOURCE:" + token)
    for token in [
        'name="${prefix}side_suction_tcp"',
        'origin xyz="0 ${side_sign * 0.155} 0.080"',
        'rpy="0 0 ${side_sign * 1.57079632679}"',
    ]:
        if token not in xacro:
            failures.append("XACRO:" + token)

    # The fixture itself records the boundary: hand visuals/colliders are hidden,
    # while USD bodies/inertia remain. This is a declared dynamics boundary.
    hidden_boundary = manifest.get("dynamics_boundary", {}).get("hidden_hand_and_fingers")
    if not hidden_boundary:
        hidden_boundary = manifest.get("constraints", {}).get("hidden_hand_and_fingers")
    if not hidden_boundary and "hidden_hand_and_fingers" not in json_text(manifest):
        failures.append("HIDDEN_FINGER_BOUNDARY_MISSING")

    provenance = {
        "task": "Task28 G2-P5-QP-6-R2",
        "fixture_id": FIXTURE_ID,
        "scene_identity": {
            "root": scene["scene_namespace"],
            "left_articulation": scene["robot_paths"]["left"],
            "right_articulation": scene["robot_paths"]["right"],
            "cube": scene["cube_paths"][0],
            "physics_dt_s": 1.0 / scene["physics"]["steps_per_second"],
            "asset_sha256": sha256(FR3_ASSET) if FR3_ASSET.is_file() else None,
        },
        "tool_geometry": {
            "asset": "existing mirrored L-shaped side-suction tool authored by the Task28 fixture",
            "vertical_support_m": 0.080,
            "lateral_support_m": 0.130,
            "manifold_m": [0.070, 0.070],
            "cups_per_arm": 4,
            "link8_to_tcp_left_m": tcp["left_link8_translation_m"],
            "link8_to_tcp_right_m": tcp["right_link8_translation_m"],
            "local_rz_left_rad": tcp["left_link8_rpy_rad"][2],
            "local_rz_right_rad": tcp["right_link8_rpy_rad"][2],
            "nominal_grasp_world_quaternions_are_distinct": True,
        },
        "dynamics_boundary": {
            "arm_control_dofs": 14,
            "native_finger_dofs_per_arm": 2,
            "native_fingers_active_suction_device": False,
            "hidden_hand_visual_collision": True,
            "native_usd_bodies_and_inertia_retained": True,
            "tool_mass_inertia_in_articulation": "UNKNOWN_UNTIL_RUNTIME_SOURCE_READ",
            "arbitrary_7x7_mass_slice_allowed": False,
        },
        "suction": bridge["suction_contract"],
        "legacy_sources": {
            "task24": str(REPO / "isaac/scripts/task24_side_suction_tight_scene.py"),
            "task26": str(REPO / "isaac/scripts/task26_truck_box_scene.py"),
            "task27": str(REPO / "docs/tasks/TASK27_FIVE_CUBE_CENTER_INSERT.md"),
            "current_fixture": str(FIXTURE),
            "historical_results_transfer": False,
        },
        "hashes": serialise_manifest_hashes(),
        "failures": failures,
        "status": "SCENE_IDENTITY_VERIFIED" if not failures else "SCENE_IDENTITY_REJECTED",
    }
    return provenance


def json_text(value) -> str:
    import json
    return json.dumps(value, ensure_ascii=False)


def main() -> int:
    import argparse
    from .common import dump_json

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = inspect_static()
    dump_json(args.output, result)
    print(result["status"])
    return 0 if result["status"] == "SCENE_IDENTITY_VERIFIED" else 1


if __name__ == "__main__":
    raise SystemExit(main())
