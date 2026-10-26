#!/usr/bin/env python3
"""Render xacro with frozen conservative joint bounds and collision meshes.

This source imports no ROS/Isaac module and sends no commands. Its pure XML
transforms narrow the 14 arm joint limits and replace the 16 arm collision mesh
references with verified conservative hulls. Raw xacro/tool/SRDF sources, all
collision origins, visual geometry and ACM entries remain unchanged.
"""

import argparse
import json
import math
import subprocess
import sys
import xml.etree.ElementTree as ET


EXPECTED_ORDER = [f"joint{index}" for index in range(1, 8)]


class LimitError(ValueError):
    """The requested frozen joint limits cannot be safely applied."""


class GeometryError(ValueError):
    """The expected arm collision mesh cannot be replaced conservatively."""


def _finite_number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise LimitError(f"{label} must be a finite number")
    return float(value)


def _serialize(root):
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def narrow_joint_limits(xml_text, limits):
    """Return URDF with strictly validated, never-expanded arm joint bounds.

    ``limits`` contains joint_order [joint1..joint7], position_lower_rad and
    position_upper_rad, velocity_max_rad_s and effort_max_Nm. Both arms receive
    those same seven frozen bounds. The input contains the audited USD/YAML
    intersection; bounds larger than the original rendered URDF are rejected,
    rather than silently accepted or clipped. There are no fallback bounds.
    """
    if not isinstance(limits, dict) or limits.get("joint_order") != EXPECTED_ORDER:
        raise LimitError("joint_order must be exactly joint1 through joint7")
    lower = limits.get("position_lower_rad")
    upper = limits.get("position_upper_rad")
    velocity = limits.get("velocity_max_rad_s")
    effort = limits.get("effort_max_Nm")
    if not all(isinstance(values, list) and len(values) == 7 for values in (lower, upper, velocity, effort)):
        raise LimitError("position, velocity and effort bounds must each contain seven values")
    bounds = []
    for index, (low, high, speed, force) in enumerate(zip(lower, upper, velocity, effort), start=1):
        low = _finite_number(low, f"joint{index} lower")
        high = _finite_number(high, f"joint{index} upper")
        speed = _finite_number(speed, f"joint{index} velocity")
        force = _finite_number(force, f"joint{index} effort")
        if low >= high:
            raise LimitError(f"joint{index} lower must be less than upper")
        if speed <= 0 or force <= 0:
            raise LimitError(f"joint{index} velocity and effort must be positive")
        bounds.append((low, high, speed, force))

    try:
        root = ET.fromstring(xml_text)
    except (ET.ParseError, TypeError) as error:
        raise LimitError(f"invalid rendered URDF: {error}") from error
    if root.tag != "robot":
        raise LimitError("rendered URDF root must be robot")

    by_name = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        if not name or name in by_name:
            raise LimitError(f"missing or duplicate URDF joint name: {name!r}")
        by_name[name] = joint

    # Validate every target before changing the XML, so an error cannot return
    # a partially transformed model.
    changes = []
    for side in ("left", "right"):
        for index, (low, high, speed, force) in enumerate(bounds, start=1):
            name = f"{side}_fr3_joint{index}"
            joint = by_name.get(name)
            if joint is None:
                raise LimitError(f"required arm joint is missing: {name}")
            if joint.get("type") != "revolute":
                raise LimitError(f"required arm joint is not revolute: {name}")
            elements = joint.findall("limit")
            if len(elements) != 1:
                raise LimitError(f"joint must have exactly one limit element: {name}")
            limit = elements[0]
            try:
                original_low = float(limit.attrib["lower"])
                original_high = float(limit.attrib["upper"])
                original_speed = float(limit.attrib["velocity"])
                original_force = float(limit.attrib["effort"])
            except (KeyError, TypeError, ValueError) as error:
                raise LimitError(f"joint has missing/invalid original bounds: {name}") from error
            if not math.isfinite(original_low) or not math.isfinite(original_high) or original_low >= original_high:
                raise LimitError(f"joint has nonfinite or unordered original bounds: {name}")
            if not math.isfinite(original_speed) or not math.isfinite(original_force) or original_speed <= 0 or original_force <= 0:
                raise LimitError(f"joint has invalid original velocity/effort bounds: {name}")
            if low < original_low or high > original_high or speed > original_speed or force > original_force:
                raise LimitError(f"frozen bounds expand original URDF limit: {name}")
            changes.append((limit, low, high, speed, force))

    for limit, low, high, speed, force in changes:
        limit.set("lower", repr(low))
        limit.set("upper", repr(high))
        limit.set("velocity", repr(speed))
        limit.set("effort", repr(force))
    return _serialize(root)


def _identity_vector(text, expected, label):
    try:
        values = [float(value) for value in text.split()]
    except (AttributeError, TypeError, ValueError) as error:
        raise GeometryError(f"invalid {label}") from error
    if values != expected or not all(math.isfinite(value) for value in values):
        raise GeometryError(f"nonidentity {label}")


def conservative_collision_meshes(xml_text, package="task28_single_cube_description"):
    """Replace only the expected arm collision filenames; preserve origins.

    Exactly one collision mesh per left/right_fr3_link0..7 must point to the
    audited original Franka STL at identity origin/scale. Missing origins and
    scales mean the URDF identity defaults and remain omitted. The destination
    hulls must be independently hash/containment-verified by the fixture checker.
    This function does not claim a containment proof from filenames alone.
    """
    if package != "task28_single_cube_description":
        raise GeometryError("only the frozen Task28 collision package is allowed")
    try:
        root = ET.fromstring(xml_text)
    except (ET.ParseError, TypeError) as error:
        raise GeometryError(f"invalid rendered URDF: {error}") from error
    if root.tag != "robot":
        raise GeometryError("rendered URDF root must be robot")
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise GeometryError(f"missing or duplicate link name: {name!r}")
        links[name] = link
    changes = []
    for side in ("left", "right"):
        for index in range(8):
            name = f"{side}_fr3_link{index}"
            link = links.get(name)
            if link is None:
                raise GeometryError(f"required arm link missing: {name}")
            collisions = link.findall("collision")
            if len(collisions) != 1:
                raise GeometryError(f"expected exactly one collision element: {name}")
            collision = collisions[0]
            origins = collision.findall("origin")
            if len(origins) > 1:
                raise GeometryError(f"duplicate collision origins: {name}")
            if origins:
                _identity_vector(origins[0].get("xyz", "0 0 0"), [0.0, 0.0, 0.0], name + " collision translation")
                _identity_vector(origins[0].get("rpy", "0 0 0"), [0.0, 0.0, 0.0], name + " collision rotation")
            geometries = collision.findall("geometry")
            if len(geometries) != 1 or len(geometries[0]) != 1 or geometries[0][0].tag != "mesh":
                raise GeometryError(f"expected exactly one collision mesh geometry: {name}")
            mesh = geometries[0][0]
            expected = f"package://franka_description/meshes/robots/fr3/collision/link{index}.stl"
            if mesh.get("filename") != expected:
                raise GeometryError(f"unexpected source collision mesh: {name}")
            _identity_vector(mesh.get("scale", "1 1 1"), [1.0, 1.0, 1.0], name + " collision scale")
            changes.append((mesh, f"package://{package}/meshes/fr3_link{index}_conservative_union.stl"))
    for mesh, filename in changes:
        mesh.set("filename", filename)
    return _serialize(root)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", required=True, help="Exact xacro executable selected by launch")
    parser.add_argument("--limits", required=True, help="Frozen effective_joint_limits.json")
    parser.add_argument("robot_xacro", help="Isolated Task28 robot xacro source")
    arguments = parser.parse_args(argv)
    try:
        with open(arguments.limits, "r", encoding="utf-8") as stream:
            limits = json.load(stream)
        result = subprocess.run(
            [arguments.xacro, arguments.robot_xacro], check=True,
            capture_output=True, text=True,
        )
        rendered = conservative_collision_meshes(narrow_joint_limits(result.stdout, limits))
    except (OSError, json.JSONDecodeError, subprocess.CalledProcessError, LimitError, GeometryError) as error:
        print(f"Task28 robot description refused: {error}", file=sys.stderr)
        return 1
    print(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
