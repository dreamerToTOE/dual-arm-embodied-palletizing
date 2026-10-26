"""Single-state solid FCL collision gate for the unchanged Task28 fixture.

Every transform is T_world_link: column-vector active transforms, metres.
This module never launches ROS, Isaac, robot drivers, or suction commands.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import itertools
import json
import math
from pathlib import Path
import shlex
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
from .model import verify_fixture


@dataclass(frozen=True)
class CollisionResult:
    valid: bool
    collisions: tuple[dict, ...]
    accepted_attachment_contacts: tuple[dict, ...]
    reasons: tuple[str, ...]
    checked_shape_pairs: int
    skipped_acm_shape_pairs: int
    backend: str = "FCL-0.7.0"


@dataclass(frozen=True)
class _Shape:
    owner: str
    name: str
    kind: str
    params: tuple
    origin: np.ndarray
    cup: bool = False


def _transform(value, name):
    matrix = np.asarray(value)
    if (matrix.shape != (4, 4) or not np.issubdtype(matrix.dtype, np.number)
            or np.issubdtype(matrix.dtype, np.bool_) or np.issubdtype(matrix.dtype, np.complexfloating)
            or not np.all(np.isfinite(matrix))):
        raise ValueError(f"INVALID_SE3:{name}")
    matrix = matrix.astype(float)
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0):
        raise ValueError(f"INVALID_SE3:{name}")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or abs(np.linalg.det(rotation) - 1) > 1e-8):
        raise ValueError(f"INVALID_SE3:{name}")
    return matrix


def _origin(node):
    result = np.eye(4)
    if node is None:
        return result
    roll, pitch, yaw = map(float, node.get("rpy", "0 0 0").split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    result[:3, :3] = [
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ]
    result[:3, 3] = list(map(float, node.get("xyz", "0 0 0").split()))
    return result


def build_backend(cache_dir=None):
    """Compile only the standalone collision helper, never any robot software."""
    source = Path(__file__).parent / "backend" / "fcl_state.cpp"
    version = subprocess.check_output(["pkg-config", "--modversion", "fcl"], text=True).strip()
    if version != "0.7.0":
        raise RuntimeError(f"FCL_VERSION_MISMATCH:{version}; expected frozen 0.7.0")
    flags = shlex.split(subprocess.check_output(["pkg-config", "--cflags", "--libs", "fcl"], text=True))
    signature = hashlib.sha256(source.read_bytes() + version.encode() + repr(flags).encode()).hexdigest()[:20]
    directory = Path(cache_dir) if cache_dir else Path(tempfile.gettempdir()) / "task28_g1_1_fcl"
    directory.mkdir(parents=True, exist_ok=True)
    binary = directory / f"fcl_state_{signature}"
    if not binary.exists():
        temporary = binary.with_suffix(".building")
        command = ["g++", "-O2", "-std=c++14", str(source), "-o", str(temporary), *flags]
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=120)
        temporary.replace(binary)
    return binary


class CollisionValidator:
    """Fail-closed state query over all frozen solid collision subgeometries.

    SRDF exceptions are copied exactly. The separate attachment policy may accept
    only zero-depth cup front-cap contacts on the Cube's designated object-Y
    faces. It never modifies the ACM or skips an entire tool/Cube pair.
    """
    CONTACT_PLANE_TOLERANCE_M = 1e-6
    CONTACT_DEPTH_TOLERANCE_M = 1e-6
    CAP_AXIS_ANGLE_RAD = 1e-6
    # GJK normals at zero penetration are numerically ill-conditioned. The cap
    # axis, cap-plane location and depth have separate, much tighter gates.
    CONTACT_NORMAL_ANGLE_RAD = 1e-2

    def __init__(self, fixture_dir, backend_path=None):
        self.fixture_dir = Path(fixture_dir).resolve()
        verify_fixture(self.fixture_dir)
        self.description = self.fixture_dir / "moveit/task28_single_cube_description"
        self.backend_path = Path(backend_path) if backend_path else build_backend()
        if not self.backend_path.is_file():
            raise FileNotFoundError("COLLISION_BACKEND_MISSING")
        self.shapes = self._load_shapes()
        self.allowed_pairs = self._load_acm()
        geometry = json.loads((self.description / "config/fixture_geometry.json").read_text())
        self.cube_half_size = np.asarray(geometry["cube_size_m"]) / 2
        table_pose = np.eye(4)
        table_pose[:3, 3] = geometry["table_center_world_m"]
        self.environment = (
            _Shape("Cube_01", "Cube_01/box", "B", tuple(geometry["cube_size_m"]), np.eye(4)),
            _Shape("task28_table", "task28_table/box", "B", tuple(geometry["table_size_m"]), table_pose),
        )
        self.required_links = frozenset(shape.owner for shape in self.shapes)

    def _load_shapes(self):
        root = ET.parse(self.description / "urdf/task28_single_cube_frozen.urdf").getroot()
        shapes = []
        for link in root.findall("link"):
            owner = link.get("name")
            for index, collision in enumerate(link.findall("collision")):
                geometry = collision.find("geometry")
                primitive = list(geometry)[0]
                if primitive.tag == "mesh":
                    uri = primitive.get("filename")
                    prefix = "package://task28_single_cube_description/"
                    if not uri.startswith(prefix) or primitive.get("scale", "1 1 1") != "1 1 1":
                        raise ValueError("UNSUPPORTED_OR_UNFROZEN_MESH")
                    path = (self.description / uri[len(prefix):]).resolve()
                    if self.description not in path.parents or not path.is_file():
                        raise FileNotFoundError(f"COLLISION_MESH_MISSING:{path}")
                    kind, params = "M", (str(path),)
                elif primitive.tag == "box":
                    kind, params = "B", tuple(map(float, primitive.get("size").split()))
                elif primitive.tag == "cylinder":
                    kind, params = "C", (float(primitive.get("radius")), float(primitive.get("length")))
                else:
                    raise ValueError(f"UNSUPPORTED_COLLISION_GEOMETRY:{primitive.tag}")
                shapes.append(_Shape(owner, f"{owner}/{index}", kind, params,
                                     _origin(collision.find("origin")),
                                     kind == "C" and owner.endswith("side_suction")))
        if len(shapes) != 30:
            raise ValueError("FROZEN_COLLISION_SHAPE_COUNT_MISMATCH")
        return tuple(shapes)

    def _load_acm(self):
        root = ET.parse(self.description / "srdf/dual_fr3_side_suction.srdf.xacro").getroot()
        xacro = "{http://www.ros.org/wiki/xacro}"
        macros = {node.get("name"): node for node in root.findall(xacro + "macro")}
        pairs = set()
        for node in root:
            if node.tag == "disable_collisions":
                pairs.add(frozenset((node.get("link1"), node.get("link2"))))
            elif node.tag.startswith(xacro) and node.tag != xacro + "macro":
                name = node.tag[len(xacro):]
                if name not in macros or macros[name].get("params") != "p":
                    raise ValueError("UNKNOWN_ACM_MACRO")
                for disabled in macros[name].findall("disable_collisions"):
                    names = tuple(disabled.get(key).replace("${p}", node.get("p"))
                                  for key in ("link1", "link2"))
                    if any("${" in value for value in names):
                        raise ValueError("UNRESOLVED_ACM")
                    pairs.add(frozenset(names))
        if len(pairs) != 48:
            raise ValueError("FROZEN_ACM_COUNT_MISMATCH")
        if any("Cube_01" in pair or "task28_table" in pair for pair in pairs):
            raise ValueError("UNEXPECTED_PAYLOAD_ACM_ALLOWANCE")
        return frozenset(pairs)

    def _cup_on_designated_face(self, shape, pose, object_pose):
        if not shape.cup:
            return False
        # Expected physical contact is the four cup caps on each local object Y
        # face; a rotated Cube carries those faces and normals with it.
        side = -1 if shape.owner.startswith("left_") else 1
        inward_object = np.array([0., -side, 0.])
        inward_world = object_pose[:3, :3] @ inward_object
        cap_inward_world = -pose[:3, 2]
        if np.dot(cap_inward_world, inward_world) < math.cos(self.CAP_AXIS_ANGLE_RAD):
            return False
        radius, length = shape.params
        cap_world = pose[:3, 3] - pose[:3, 2] * length / 2
        cap_object = object_pose[:3, :3].T @ (cap_world - object_pose[:3, 3])
        tolerance = self.CONTACT_PLANE_TOLERANCE_M
        # Frozen front-cap centres are +/-24 mm in object X/Z. Derive which
        # centre belongs to this particular frozen cylinder, never an arbitrary
        # point elsewhere on the Cube side.
        expected_cap_object = np.array([shape.origin[0, 3],
                                        side*self.cube_half_size[1],
                                        .080-shape.origin[2, 3]])
        if np.max(np.abs(cap_object-expected_cap_object)) > tolerance:
            return False
        if any(abs(cap_object[axis]) + radius > self.cube_half_size[axis] + tolerance for axis in (0, 2)):
            return False
        return True

    @staticmethod
    def _checked_backend_pairs(output, shape_count, requested_pairs):
        """Validate the complete response schema before trusting an empty list."""
        if not isinstance(output, dict) or set(output) != {
                "backend", "pairs", "shape_count", "checked_pair_count"}:
            raise ValueError("INVALID_COLLISION_RECEIPT")
        if (output["backend"] != "FCL-0.7.0"
                or type(output["shape_count"]) is not int or output["shape_count"] != shape_count
                or type(output["checked_pair_count"]) is not int
                or output["checked_pair_count"] != len(requested_pairs)):
            raise ValueError("INCOMPLETE_COLLISION_RECEIPT")
        if not isinstance(output["pairs"], list):
            raise ValueError("INVALID_COLLISION_PAIR_LIST")
        requested, seen = set(requested_pairs), set()
        for pair in output["pairs"]:
            if not isinstance(pair, dict) or set(pair) != {"a", "b", "contacts"}:
                raise ValueError("INVALID_COLLISION_PAIR")
            a, b = pair["a"], pair["b"]
            if type(a) is not int or type(b) is not int or (a, b) not in requested or (a, b) in seen:
                raise ValueError("UNREQUESTED_OR_DUPLICATE_COLLISION_PAIR")
            seen.add((a, b))
            contacts = pair["contacts"]
            if not isinstance(contacts, list) or not 1 <= len(contacts) <= 128:
                raise ValueError("INVALID_COLLISION_CONTACT_LIST")
            for contact in contacts:
                if (not isinstance(contact, dict) or set(contact) != {
                        "depth_m", "normal_world", "position_world_m"}):
                    raise ValueError("INVALID_COLLISION_CONTACT")
                if any(not isinstance(contact[key], list) or len(contact[key]) != 3
                       for key in ("normal_world", "position_world_m")):
                    raise ValueError("INVALID_COLLISION_CONTACT_VECTOR")
                values = [contact["depth_m"], *contact["normal_world"], *contact["position_world_m"]]
                if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
                    raise ValueError("NONFINITE_COLLISION_CONTACT")
                normal_norm = np.linalg.norm(contact["normal_world"])
                if abs(normal_norm - 1.) > 1e-5:
                    raise ValueError("INVALID_COLLISION_CONTACT_NORMAL")
        return output["pairs"]

    def _accept_cup_contact(self, shape, pose, object_pose, contacts, attachment_mode):
        if (attachment_mode != "bilateral" or not contacts or len(contacts) >= 128
                or not self._cup_on_designated_face(shape, pose, object_pose)):
            return False
        side = -1 if shape.owner.startswith("left_") else 1
        inward_world = object_pose[:3, :3] @ np.array([0., -side, 0.])
        radius, length = shape.params
        tolerance = self.CONTACT_PLANE_TOLERANCE_M
        for contact in contacts:
            depth = contact["depth_m"]
            point = np.asarray(contact["position_world_m"])
            normal = np.asarray(contact["normal_world"])
            if not np.all(np.isfinite([depth, *point, *normal])) or abs(depth) > self.CONTACT_DEPTH_TOLERANCE_M:
                return False
            point_object = object_pose[:3, :3].T @ (point - object_pose[:3, 3])
            point_cup = pose[:3, :3].T @ (point - pose[:3, 3])
            if (abs(point_object[1] - side*self.cube_half_size[1]) > tolerance
                    or abs(point_cup[2] + length/2) > tolerance
                    or np.linalg.norm(point_cup[:2]) > radius + tolerance
                    or abs(np.dot(normal, inward_world)) < math.cos(self.CONTACT_NORMAL_ANGLE_RAD)):
                return False
        return True

    def validate(self, link_transforms, object_pose, attachment_mode="bilateral"):
        if attachment_mode not in ("bilateral", "detached"):
            return CollisionResult(False, (), (), ("INVALID_ATTACHMENT_MODE",), 0, 0)
        try:
            object_pose = _transform(object_pose, "Cube_01")
            missing = sorted(self.required_links - set(link_transforms))
            if missing:
                raise ValueError("MISSING_COLLISION_LINKS:" + ",".join(missing))
            links = {name: _transform(link_transforms[name], name) for name in self.required_links}
        except (ValueError, TypeError) as error:
            return CollisionResult(False, (), (), (str(error),), 0, 0)
        shapes = (*self.shapes, *self.environment)
        poses = [links[shape.owner] @ shape.origin if shape.owner in links else
                 object_pose if shape.owner == "Cube_01" else shape.origin for shape in shapes]
        # An enum alone cannot authorize contact: all eight cup caps must form
        # the bilateral designated contact geometry in this very state.
        contact_context = attachment_mode if all(
            self._cup_on_designated_face(shape, pose, object_pose)
            for shape, pose in zip(shapes, poses) if shape.cup) else "detached"
        pairs, skipped = [], 0
        for a, b in itertools.combinations(range(len(shapes)), 2):
            if shapes[a].owner == shapes[b].owner:
                continue  # Overlapping primitives constitute a single compound tool.
            if frozenset((shapes[a].owner, shapes[b].owner)) in self.allowed_pairs:
                skipped += 1
            else:
                pairs.append((a, b))
        rows = [f"{len(shapes)} {len(pairs)}"]
        for shape, pose in zip(shapes, poses):
            rows.append(" ".join([shape.kind, *map(str, shape.params),
                                  *map(lambda v: format(v, ".17g"), pose[:3].reshape(-1))]))
        rows.extend(f"{a} {b}" for a, b in pairs)
        try:
            process = subprocess.run([str(self.backend_path)], input="\n".join(rows) + "\n",
                                     text=True, capture_output=True, check=True, timeout=30)
            output = json.loads(process.stdout)
            backend_pairs = self._checked_backend_pairs(output, len(shapes), pairs)
            collisions, accepted = [], []
            for pair in backend_pairs:
                a, b = pair["a"], pair["b"]
                record = {"shape_a": shapes[a].name, "shape_b": shapes[b].name,
                          "owner_a": shapes[a].owner, "owner_b": shapes[b].owner,
                          "contacts": pair["contacts"]}
                tool_index = a if shapes[b].owner == "Cube_01" else b if shapes[a].owner == "Cube_01" else None
                if tool_index is not None and self._accept_cup_contact(
                        shapes[tool_index], poses[tool_index], object_pose, pair["contacts"], contact_context):
                    accepted.append(record)
                else:
                    collisions.append(record)
            reasons = tuple(f"COLLISION:{c['shape_a']}|{c['shape_b']}" for c in collisions)
            return CollisionResult(not collisions, tuple(collisions), tuple(accepted), reasons, len(pairs), skipped)
        except (subprocess.SubprocessError, OSError, ValueError, KeyError, TypeError) as error:
            return CollisionResult(False, (), (), (f"COLLISION_BACKEND_FAILURE:{error}",), len(pairs), skipped)
