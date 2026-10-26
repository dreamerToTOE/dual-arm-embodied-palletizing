#!/usr/bin/env python3
"""Bounded, offline source/configuration checks. Never imports Isaac or ROS."""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True

FIXTURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FIXTURE_DIR.parents[1]
MOVEIT = FIXTURE_DIR / "moveit/task28_single_cube_description"


class FixtureError(RuntimeError):
    pass


def equal(label, actual, expected):
    if actual != expected:
        raise FixtureError(f"{label}: expected {expected!r}, got {actual!r}")


def require_file(path):
    if not path.is_file():
        raise FixtureError(f"missing configuration/asset dependency: {path}")
    return path


def sha256(path):
    return hashlib.sha256(require_file(path).read_bytes()).hexdigest()


def load_json(path):
    try:
        result = json.loads(require_file(path).read_text(encoding="utf-8"))
        if not isinstance(result, dict):
            raise FixtureError(f"JSON root is not an object: {path}")
        return result
    except (OSError, json.JSONDecodeError) as error:
        raise FixtureError(str(error)) from error


def inside(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise FixtureError(f"dependency escapes fixture/repository: {relative}")
    return path


def constants(path):
    """Read top-level literals/constant expressions without importing the source."""
    tree = ast.parse(require_file(path).read_text(), filename=str(path))
    values = {}

    def value(node):
        if isinstance(node, ast.Name):
            return values[node.id]
        if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
            return value(node.left) + value(node.right)
        if isinstance(node, (ast.List, ast.Tuple)):
            result = [value(item) for item in node.elts]
            return tuple(result) if isinstance(node, ast.Tuple) else result
        if isinstance(node, ast.Dict):
            return {value(key): value(val) for key, val in zip(node.keys, node.values)}
        return ast.literal_eval(node)

    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            try:
                values[node.targets[0].id] = value(node.value)
            except (KeyError, ValueError, TypeError):
                pass
    return values


def validate_bundle(bundle):
    try:
        _validate_bundle(bundle)
    except (KeyError, TypeError, IndexError, ValueError, OSError) as error:
        raise FixtureError(f"incomplete/invalid fixture: {error}") from error


def _validate_bundle(b):
    m, s, br, c, mv = (b[key] for key in ("manifest", "scene", "bridge", "controller", "moveit"))
    equal("status", m["status"], "NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED")
    equal("physical validation", m["physical_validation"], False)
    equal("historical claim isolation", m["historical_task24_h"]["claim_not_transferred"], True)
    equal("approved new baseline", m["approved_new_baseline"], True)
    for name, contract in (("scene", s), ("bridge", br), ("controller", c), ("moveit", mv)):
        equal(name + " fixture id", contract["fixture_id"], m["fixture_id"])
        equal(name + " status", contract["status"], m["status"])
        equal(name + " frame", contract["world_frame"], "world")
    cube = m["cube"]
    equal("cube identity", (cube["id"], cube["index"], cube["count"]), ("Cube_01", 0, 1))
    equal("initial position", cube["initial_pose_world_m"], [0.65, 0.0, 0.26])
    equal("target position", cube["target_pose_world_m"], [0.82, -0.075, 0.26])
    equal("cube size", cube["size_m"], [0.12] * 3)
    equal("cube mass", cube["mass_kg"], 0.8)
    equal("cube orientation", cube["orientation_xyzw"], [0.0, 0.0, 0.0, 1.0])
    equal("dynamic cube", cube["dynamic"], True)
    for key, val in s["cube"].items():
        equal("scene Cube " + key, val, cube[key])
    for name, contract in (("scene", s), ("bridge", br)):
        equal(name + " count", contract["active_cube_count"], 1)
        equal(name + " ids", contract["cube_ids"], ["Cube_01"])
        equal(name + " order", contract["pose_array_order"], ["Cube_01"])
    equal("controller count", c["cube_count"], 1)
    equal("controller order", c["pose_array_order"], ["Cube_01"])
    equal("controller map", c["task_index_map"], [{"task_index": 0, "cube_id": "Cube_01", "cube_index": 0,
          "target_pose_world_m": cube["target_pose_world_m"]}])
    equal("controller source", c["source_pose_world_m"], cube["initial_pose_world_m"])
    equal("scene table", s["table"], m["table"])
    equal("scene bases", s["bases_world_m"], m["bases_world_m"])
    for contract in (s, c):
        equal("HOME", contract["home_q_rad"], m["home_q_rad"])
    for name, contract in (("bridge", br), ("controller", c), ("moveit", mv)):
        equal(name + " TCP", contract["tcp_contract"], m["tcp"])
    equal("suction", br["suction_contract"], m["suction"])
    equal("suction threshold", m["suction"]["grip_threshold_m"], 0.003)
    equal("force break", m["suction"]["force_break_N"], 1e6)
    equal("torque break", m["suction"]["torque_break_Nm"], 1e6)
    equal("bend", m["suction"]["bend_angle_rad"], 0.0)
    equal("stiffness", m["suction"]["stiffness"], 1e4)
    equal("damping", m["suction"]["damping"], 1e3)
    equal("gravity disabled", m["suction"]["disable_gravity"], False)
    equal("retry", m["suction"]["retry_close"], True)
    for policy in (m["collision_policy"], mv["collision_policy"]):
        equal("ACM change", policy["acm_changes"], False)
        equal("additional ACM pairs", policy["additional_allowed_collision_pairs"], [])
    equal("payload required", mv["collision_policy"]["payload_in_validator_required"], True)
    equal("collision object ids", [obj["id"] for obj in mv["collision_objects"]], ["task28_table", "Cube_01"])
    table, payload = mv["collision_objects"]
    equal("table geometry", (table["center_world_m"], table["size_m"]), (m["table"]["center_world_m"], m["table"]["size_m"]))
    equal("payload geometry", (payload["cube_index"], payload["pose_world_m"], payload["size_m"]),
          (0, cube["initial_pose_world_m"], cube["size_m"]))
    equal("trajectory execution", mv["execution"]["trajectory_execution_enabled"], False)
    for name in ("scene", "bridge", "controller", "moveit", "launch_protocol", "static_checker"):
        require_file(inside(FIXTURE_DIR, m["contracts"][name]))
    for path, expected in m["fixture_file_hashes"].items():
        equal("fixture hash " + path, sha256(inside(FIXTURE_DIR, path)), expected)
    for path, expected in m["legacy_execution_path_hashes"].items():
        equal("legacy hash " + path, sha256(inside(REPO_ROOT, path)), expected)
    for path, expected in m["repo_source_hashes"].items():
        equal("source hash " + path, sha256(inside(REPO_ROOT, path)), expected["sha256"])
    asset = m["external_dependencies"]["isaac_sim"]
    if not asset["asset_identity"] or not isinstance(asset["asset_sha256"], str) or len(asset["asset_sha256"]) != 64:
        raise FixtureError("missing identified/hashed required FR3 USD")
    sc, bc, cc = b["source_constants"]["scene"], b["source_constants"]["bridge"], b["source_constants"]["controller"]
    for name, source in (("scene", sc), ("bridge", bc)):
        equal(name + " source Cube mapping", (source["CUBE_ID"], source["CUBE_INDEX"], source["ACTIVE_CUBE_COUNT"]), ("Cube_01", 0, 1))
        equal(name + " source root", source["TASK_ROOT"], s["scene_namespace"])
        equal(name + " asset hash", source["EXPECTED_FR3_USD_SHA256"], asset["asset_sha256"])
        equal(name + " ROS namespace", source["ROS_NAMESPACE"], "/task28_g1_0")
    equal("scene source supply", sc["SUPPLY_POSES"], (tuple(cube["initial_pose_world_m"]),))
    equal("scene source targets", sc["TARGETS"], (tuple(cube["target_pose_world_m"]),))
    equal("scene source mass", sc["CUBE_MASS"], cube["mass_kg"])
    equal("scene source size", sc["CUBE_SIZE"], cube["size_m"][0])
    equal("scene source HOME", sc["FR3_OFFICIAL_START_Q_RAD"], tuple(m["home_q_rad"]))
    equal("scene source table", (sc["TABLE_CENTER"], sc["TABLE_SIZE"], sc["TABLE_TOP_Z"]),
          (tuple(m["table"]["center_world_m"]), tuple(m["table"]["size_m"]), m["table"]["top_z_m"]))
    for side in ("left", "right"):
        equal(side + " base source", sc[side.upper() + "_BASE"], tuple(m["bases_world_m"][side]))
        equal(side + " bridge TCP translation", bc["TCP_LOCAL_TRANSLATIONS"][side], tuple(m["tcp"][side + "_link8_translation_m"]))
        if abs(math.radians(bc["TCP_LOCAL_RZ_DEGREES"][side]) - m["tcp"][side + "_link8_rpy_rad"][2]) > 1e-10:
            raise FixtureError("TCP local rotation mismatch")
        equal(side + " world grasp", bc["DESIRED_WORLD_GRASP_XYZW"][side], tuple(m["tcp"][side + "_world_quaternion_xyzw"]))
    equal("controller source mapping", (cc["CUBE_ID"], cc["CUBE_INDEX"], cc["TASK_INDEX"], cc["CUBE_COUNT"]), ("Cube_01", 0, 0, 1))
    equal("controller source target", cc["TARGET_WORLD_M"], tuple(cube["target_pose_world_m"]))
    for key, source_key in (("gravity_direction", "GRAVITY_DIRECTION"), ("gravity_magnitude_m_s2", "GRAVITY_MAGNITUDE_M_S2"), ("steps_per_second", "PHYSICS_STEPS_PER_SECOND")):
        actual = sc[source_key]
        equal("physics " + key, list(actual) if isinstance(actual, tuple) else actual, s["physics"][key])
    geom = b["moveit_geometry"]
    equal("MoveIt geometry identity", (geom["cube_id"], geom["cube_index"], geom["cube_count"], geom["frame_id"]), ("Cube_01", 0, 1, "world"))
    for key, cube_key in (("cube_initial_pose_world_m", "initial_pose_world_m"), ("cube_orientation_xyzw", "orientation_xyzw"), ("cube_size_m", "size_m")):
        equal("MoveIt geometry " + key, geom[key], cube[cube_key])
    equal("MoveIt geometry table", (geom["table_id"], geom["table_center_world_m"], geom["table_size_m"]),
          (m["table"]["id"], m["table"]["center_world_m"], m["table"]["size_m"]))
    limits = m["joint_limits"]
    equal("raw joint limits snapshot", sha256(inside(FIXTURE_DIR, limits["snapshot"])), limits["source_sha256"])
    effective = b["effective_limits"]
    equal("effective joint order", effective["joint_order"], limits["joint_order"])
    for index, home in enumerate(m["home_q_rad"]):
        lower = max(limits["position_lower_rad"][index], limits["isaac_position_lower_rad"][index])
        upper = min(limits["position_upper_rad"][index], limits["isaac_position_upper_rad"][index])
        equal("effective lower", effective["position_lower_rad"][index], lower)
        equal("effective upper", effective["position_upper_rad"][index], upper)
        if not lower <= home <= upper:
            raise FixtureError("HOME violates effective joint limits")
        for field, source_field, usd_field in (("velocity_max_rad_s", "velocity_max_rad_s", "isaac_velocity_max_rad_s"),
                                                ("effort_max_Nm", "effort_max_Nm", "isaac_effort_max_Nm")):
            equal("effective " + field, effective[field][index], min(limits[source_field][index], limits[usd_field][index]))


def source_checks():
    # These parsers do not expand xacro, import ROS/Isaac, or execute launch code.
    for path in FIXTURE_DIR.rglob("*.py"):
        ast.parse(path.read_text(), filename=str(path))
    for pattern in ("*.xml", "*.xacro", "*.urdf"):
        for path in MOVEIT.rglob(pattern):
            ET.parse(path)
    legacy = REPO_ROOT / "ros_ws/src/fr3_dual_side_suction_description"
    for local in ("urdf/dual_side_suction.xacro", "srdf/dual_fr3_side_suction.srdf.xacro", "config/kinematics.yaml"):
        equal("unchanged geometry/ACM " + local, sha256(MOVEIT / local), sha256(legacy / local))
    urdf = ET.parse(MOVEIT / "urdf/dual_fr3_side_suction.urdf.xacro").getroot()
    for side, y in (("left", -0.6), ("right", 0.6)):
        origin = urdf.find(f"joint[@name='world_to_{side}_fr3']/origin")
        equal("URDF base", [float(v) for v in origin.attrib["xyz"].split()], [0.65, y, 0.0])
    tool = ET.parse(MOVEIT / "urdf/dual_side_suction.xacro").getroot()
    tcp = next(joint for joint in tool.iter("joint") if joint.attrib["name"] == "${prefix}side_suction_tcp_joint")
    equal("TCP xyz expression", tcp.find("origin").attrib["xyz"], "0 ${side_sign * 0.155} 0.080")
    equal("TCP local rpy expression", tcp.find("origin").attrib["rpy"], "0 0 ${side_sign * 1.57079632679}")
    bridge_tree = ast.parse((FIXTURE_DIR / "bridge/task28_single_cube_bridge.py").read_text())
    assignments = {}
    for node in ast.walk(bridge_tree):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name) and target.value.id == "props":
                    assignments[target.attr] = ast.unparse(node.value)
    for name, expected in {"gripThreshold": "0.003", "bendAngle": "0.0", "retryClose": "True", "disableGravity": "False",
                           "forceLimit": "RIGID_GRASP_FORCE_LIMIT", "torqueLimit": "RIGID_GRASP_TORQUE_LIMIT",
                           "stiffness": "GRASP_STIFFNESS[side]", "damping": "GRASP_DAMPING[side]"}.items():
        equal("actual gripper " + name, assignments[name], expected)
    start = next(node for node in bridge_tree.body if isinstance(node, ast.FunctionDef) and node.name == "start_bridge")
    equal("bridge default commands", ast.literal_eval(start.args.defaults[0]), False)
    launch_text = (MOVEIT / "launch/task28_single_cube_moveit.launch.py").read_text()
    if '"allow_trajectory_execution": False' not in launch_text or 'NAMESPACE = "/task28_g1_0"' not in launch_text:
        raise FixtureError("MoveIt execution/namespace policy changed")
    if 'kinematics = {"robot_description_kinematics": _load_yaml(' not in launch_text:
        raise FixtureError("MoveIt kinematics parameters are not under robot_description_kinematics")
    environment = ast.parse((MOVEIT / "scripts/task28_environment_publisher.py").read_text())
    forbidden = {"REMOVE", "allowed_collision_matrix", "AttachedCollisionObject"}
    if any((isinstance(node, ast.Attribute) and node.attr in forbidden) or
           (isinstance(node, ast.Name) and node.id in forbidden) for node in ast.walk(environment)):
        raise FixtureError("unexpected payload removal/ACM/attachment implementation")


def inert_module(name, path):
    spec = importlib.util.spec_from_file_location(name, require_file(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def model_checks(bundle, xacro=None):
    renderer = inert_module("task28_static_renderer", MOVEIT / "scripts/render_robot_description.py")
    path = MOVEIT / "urdf/task28_single_cube_frozen.urdf"
    frozen = require_file(path).read_text().strip()
    root = ET.fromstring(frozen)
    effective = bundle["effective_limits"]
    for side in ("left", "right"):
        for index in range(7):
            limit = root.find(f"joint[@name='{side}_fr3_joint{index + 1}']/limit")
            for key, field in (("lower", "position_lower_rad"), ("upper", "position_upper_rad"),
                               ("velocity", "velocity_max_rad_s"), ("effort", "effort_max_Nm")):
                equal("frozen joint " + key, float(limit.attrib[key]), effective[field][index])
        for index in range(8):
            mesh = root.find(f"link[@name='{side}_fr3_link{index}']/collision/geometry/mesh")
            expected = f"package://task28_single_cube_description/meshes/fr3_link{index}_conservative_union.stl"
            equal("frozen collision mesh", mesh.attrib["filename"], expected)
            require_file(MOVEIT / "meshes" / expected.rsplit("/", 1)[1])
        tcp = root.find(f"joint[@name='{side}_fr3_side_suction_tcp_joint']/origin")
        equal("frozen TCP translation", [float(v) for v in tcp.attrib["xyz"].split()], bundle["manifest"]["tcp"][side + "_link8_translation_m"])
        for actual, expected in zip(map(float, tcp.attrib["rpy"].split()), bundle["manifest"]["tcp"][side + "_link8_rpy_rad"]):
            if abs(actual - expected) > 1e-10:
                raise FixtureError("frozen TCP local rotation mismatch")
    # Applying the same effective intervals twice must preserve the frozen model.
    equal("frozen model limit idempotence", renderer.narrow_joint_limits(frozen, effective), frozen)
    bad_cases = [
        lambda e: e["position_lower_rad"].__setitem__(0, -3.5),
        lambda e: e["velocity_max_rad_s"].__setitem__(0, 3.0),
        lambda e: e["effort_max_Nm"].__setitem__(0, 100.0),
        lambda e: e["position_upper_rad"].__setitem__(0, float("nan")),
    ]
    for mutate in bad_cases:
        bad = copy.deepcopy(effective)
        mutate(bad)
        try:
            renderer.narrow_joint_limits(frozen, bad)
        except renderer.LimitError:
            continue
        raise FixtureError("renderer accepted expanded/invalid limit")
    raw_geometry = frozen
    for index in range(8):
        raw_geometry = raw_geometry.replace(f"package://task28_single_cube_description/meshes/fr3_link{index}_conservative_union.stl",
                                            f"package://franka_description/meshes/robots/fr3/collision/link{index}.stl")
    equal("geometry mapping", renderer.conservative_collision_meshes(raw_geometry), frozen)
    try:
        renderer.conservative_collision_meshes(raw_geometry.replace("collision/link0.stl", "collision/wrong.stl", 1))
    except renderer.GeometryError:
        pass
    else:
        raise FixtureError("renderer accepted incorrect source collision asset")
    if xacro:
        # Offline XML generation only: never executes a ROS node or launch file.
        result = subprocess.run([str(xacro), str(MOVEIT / "urdf/dual_fr3_side_suction.urdf.xacro")],
                                check=True, capture_output=True, text=True)
        actual = renderer.conservative_collision_meshes(renderer.narrow_joint_limits(result.stdout, effective))
        equal("xacro regeneration", actual, frozen)
    return 5


def expect_reject(label, bundle, mutate):
    candidate = copy.deepcopy(bundle)
    mutate(candidate)
    try:
        validate_bundle(candidate)
    except FixtureError:
        return
    raise FixtureError("negative case was accepted: " + label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--external-model-root", type=Path, help="Verify frozen external franka_description files at this relocated root")
    parser.add_argument("--asset-file", type=Path, help="Also hash an already acquired FR3 primary USD; never downloads")
    parser.add_argument("--xacro", type=Path, help="Also regenerate the frozen URDF with offline xacro; requires the frozen external model in the package search path")
    args = parser.parse_args()
    m = load_json(FIXTURE_DIR / "fixture_manifest.json")
    bundle = {"manifest": m}
    for key in ("scene", "bridge", "controller", "moveit"):
        bundle[key] = load_json(inside(FIXTURE_DIR, m["contracts"][key]))
    bundle["source_constants"] = {key: constants(FIXTURE_DIR / path) for key, path in {
        "scene": "scene/task28_single_cube_scene.py", "bridge": "bridge/task28_single_cube_bridge.py", "controller": "controller/task28_single_cube_task.py"}.items()}
    bundle["moveit_geometry"] = load_json(MOVEIT / "config/fixture_geometry.json")
    bundle["effective_limits"] = load_json(MOVEIT / "config/effective_joint_limits.json")
    validate_bundle(bundle)
    source_checks()
    renderer_negatives = model_checks(bundle, args.xacro)
    envelopes = inert_module("task28_static_envelopes", FIXTURE_DIR / "tests/check_collision_envelopes.py")
    try:
        envelope_count = envelopes.check_envelopes(MOVEIT / "meshes")
    except (ValueError, OSError) as error:
        raise FixtureError(str(error)) from error
    negative_cases = [
        ("incorrect Cube ID", lambda b: b["bridge"]["cube_ids"].__setitem__(0, "Cube_02")),
        ("incorrect task index", lambda b: b["controller"]["task_index_map"][0].__setitem__("task_index", 1)),
        ("incorrect Cube index", lambda b: b["controller"]["task_index_map"][0].__setitem__("cube_index", 1)),
        ("mismatched Cube count", lambda b: b["scene"].__setitem__("active_cube_count", 2)),
        ("inconsistent TCP", lambda b: b["moveit"]["tcp_contract"]["left_link8_translation_m"].__setitem__(1, -0.154)),
        ("wrong local TCP rotation", lambda b: b["source_constants"]["bridge"]["TCP_LOCAL_RZ_DEGREES"].__setitem__("left", 90.0)),
        ("missing configuration", lambda b: b["manifest"]["contracts"].__setitem__("scene", "missing/config.json")),
        ("missing asset identity", lambda b: b["manifest"]["external_dependencies"]["isaac_sim"].__setitem__("asset_identity", "")),
        ("missing asset hash", lambda b: b["manifest"]["external_dependencies"]["isaac_sim"].__setitem__("asset_sha256", None)),
        ("wrong frame", lambda b: b["bridge"].__setitem__("world_frame", "map")),
        ("wrong mass", lambda b: b["scene"]["cube"].__setitem__("mass_kg", 0.9)),
        ("expanded ACM", lambda b: b["manifest"]["collision_policy"].__setitem__("acm_changes", True)),
        ("new allowed pair", lambda b: b["moveit"]["collision_policy"]["additional_allowed_collision_pairs"].append(["Cube_01", "left_fr3_side_suction"])),
        ("payload omitted", lambda b: b["moveit"]["collision_objects"].pop()),
        ("weakened limits", lambda b: b["effective_limits"]["position_lower_rad"].__setitem__(0, -3.0)),
        ("expanded velocity", lambda b: b["effective_limits"]["velocity_max_rad_s"].__setitem__(0, 3.0)),
        ("weakened suction", lambda b: b["manifest"]["suction"].__setitem__("grip_threshold_m", 0.004)),
    ]
    for label, mutate in negative_cases:
        expect_reject(label, bundle, mutate)
    task_module = inert_module("task28_static_task", FIXTURE_DIR / "controller/task28_single_cube_task.py")
    equal("controller immutable task", task_module.load_task().cube_id, "Cube_01")
    for kwargs in ({"cube_ids": ["Cube_02"]}, {"pose_count": 2}, {"task_index": 1}, {"cube_index": 1}):
        try:
            task_module.load_task(**kwargs)
        except ValueError:
            continue
        raise FixtureError("controller task input accepted incorrect mapping")
    model = m["external_dependencies"]["franka_description"]
    model_root = args.external_model_root or Path(model["source_root_at_audit"])
    for relative, entry in model["files"].items():
        equal("external model hash " + relative, sha256(inside(model_root, relative)), entry["sha256"])
    if args.asset_file:
        equal("primary USD hash", sha256(args.asset_file), m["external_dependencies"]["isaac_sim"]["asset_sha256"])
    print(f"G1-0B static PASS: {len(negative_cases)} bundle, 4 task-input, {renderer_negatives} renderer and {envelope_count} envelope negative cases; {len(model['files'])} external model hashes")
    print("No Isaac/ROS/MoveIt runtime, launch, motion planning, or robot commands executed.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except FixtureError as error:
        print("G1-0B static FAIL: " + str(error), file=sys.stderr)
        raise SystemExit(1)
