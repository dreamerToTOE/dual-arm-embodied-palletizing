"""Frozen-URDF FK, limits and bounded deterministic inverse kinematics.

IK is scipy's bounded trust-region reflective least squares, not a closed-chain
projection or NR planner. A failed local solve means budget exhausted; only the
explicit reach-sphere bound can establish the reported unreachable control.
"""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .se3 import axis_rotation, checked_transform, residual, rpy_matrix, transform

DEFAULT_FIXTURE = Path(__file__).resolve().parents[3] / "fixtures/task28_g1_0_new_single_cube_v0"


def verify_fixture(fixture_dir):
    fixture = Path(fixture_dir).resolve()
    binding = json.loads(Path(__file__).with_name("fixture_binding.json").read_text())
    manifest_bytes = (fixture / "fixture_manifest.json").read_bytes()
    if hashlib.sha256(manifest_bytes).hexdigest() != binding["manifest_sha256"]:
        raise ValueError("FIXTURE_MANIFEST_HASH_MISMATCH")
    manifest = json.loads(manifest_bytes)
    for relative, digest in manifest["fixture_file_hashes"].items():
        path = (fixture / relative).resolve()
        if not path.is_relative_to(fixture) or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError("FIXTURE_FILE_HASH_MISMATCH:" + relative)
    return manifest


@dataclass(frozen=True)
class IKResult:
    success: bool
    q: tuple
    translation_error_m: float | None
    rotation_error_rad: float | None
    reason: str
    attempts: int
    function_evaluations: int
    reach_bound_excess_m: float | None = None


class RobotModel:
    def __init__(self, fixture_dir=DEFAULT_FIXTURE):
        self.fixture_dir = Path(fixture_dir).resolve()
        self.manifest = verify_fixture(self.fixture_dir)
        pkg = self.fixture_dir / "moveit/task28_single_cube_description"
        self.root = ET.parse(pkg / "urdf/task28_single_cube_frozen.urdf").getroot()
        self.links = tuple(link.attrib["name"] for link in self.root.findall("link"))
        self.joints = []
        self.by_child = {}
        for element in self.root.findall("joint"):
            kind = element.attrib["type"]
            if kind not in ("fixed", "revolute"):
                raise ValueError("UNSUPPORTED_JOINT_TYPE")
            origin = element.find("origin")
            xyz = [float(x) for x in origin.get("xyz", "0 0 0").split()] if origin is not None else [0., 0., 0.]
            rpy = [float(x) for x in origin.get("rpy", "0 0 0").split()] if origin is not None else [0., 0., 0.]
            axis = element.find("axis")
            row = {"name": element.attrib["name"], "kind": kind,
                   "parent": element.find("parent").attrib["link"], "child": element.find("child").attrib["link"],
                   "origin": transform(rpy_matrix(*rpy), xyz),
                   "axis": np.asarray([float(x) for x in axis.attrib["xyz"].split()]) if axis is not None else np.array([0.,0.,1.])}
            self.joints.append(row)
            if row["child"] in self.by_child:
                raise ValueError("DUPLICATE_CHILD_LINK")
            self.by_child[row["child"]] = row
        self.home = np.asarray(self.manifest["home_q_rad"])
        effective = json.loads((pkg / "config/effective_joint_limits.json").read_text())
        self.joint_lower = np.asarray(effective["position_lower_rad"])
        self.joint_upper = np.asarray(effective["position_upper_rad"])
        self.chains = {}
        self.reach_bounds = {}
        for side in ("left", "right"):
            chain, link = [], f"{side}_fr3_side_suction_tcp"
            while link != "world":
                row = self.by_child[link]
                chain.insert(0, row)
                link = row["parent"]
                if len(chain) > len(self.joints):
                    raise ValueError("JOINT_GRAPH_CYCLE")
            self.chains[side] = chain
            # Sum of fixed inter-joint translations and TCP standoff: a necessary
            # (loose) geometric reach bound from that arm's link0 origin.
            self.reach_bounds[side] = sum(np.linalg.norm(row["origin"][:3, 3]) for row in chain[1:])
            for i in range(7):
                element = self.root.find(f"joint[@name='{side}_fr3_joint{i+1}']/limit")
                if float(element.attrib["lower"]) != self.joint_lower[i] or float(element.attrib["upper"]) != self.joint_upper[i]:
                    raise ValueError("FROZEN_LIMIT_SOURCE_MISMATCH")

    def within_limits(self, q):
        q = np.asarray(q)
        return bool(q.shape == (7,) and np.issubdtype(q.dtype, np.number) and not np.issubdtype(q.dtype, np.complexfloating) and not np.issubdtype(q.dtype, np.bool_) and
                    np.isfinite(q).all() and np.all(q >= self.joint_lower) and np.all(q <= self.joint_upper))

    @staticmethod
    def _q(q):
        q = np.asarray(q)
        if q.shape != (7,) or not np.issubdtype(q.dtype, np.number) or np.issubdtype(q.dtype, np.complexfloating) or np.issubdtype(q.dtype, np.bool_) or not np.isfinite(q).all():
            raise ValueError("INVALID_JOINT_VECTOR")
        return q.astype(float)

    def _chain(self, side, q):
        if side not in self.chains:
            raise ValueError("INVALID_ARM_SIDE")
        q = self._q(q)
        values = {f"{side}_fr3_joint{i+1}": q[i] for i in range(7)}
        frames, pre_joints, current = {"world": np.eye(4)}, [], np.eye(4)
        for row in self.chains[side]:
            current = current @ row["origin"]
            if row["kind"] == "revolute":
                pre_joints.append((current.copy(), row["axis"]))
                current = current @ transform(axis_rotation(row["axis"], values[row["name"]]))
            frames[row["child"]] = current.copy()
        return frames, pre_joints

    def fk(self, side, q):
        # FK itself intentionally supports out-of-limit algebra controls. Acceptance
        # always calls within_limits separately; FK is not an acceptance decision.
        return self._chain(side, q)[0][f"{side}_fr3_side_suction_tcp"]

    def forward(self, q_left, q_right):
        ql, qr = self._q(q_left), self._q(q_right)
        values = {f"{side}_fr3_joint{i+1}": q[i] for side,q in (("left",ql),("right",qr)) for i in range(7)}
        frames, pending = {"world": np.eye(4)}, list(self.joints)
        while pending:
            progressed = False
            for row in pending[:]:
                if row["parent"] not in frames:
                    continue
                matrix = frames[row["parent"]] @ row["origin"]
                if row["kind"] == "revolute":
                    matrix = matrix @ transform(axis_rotation(row["axis"], values[row["name"]]))
                frames[row["child"]] = matrix
                pending.remove(row)
                progressed = True
            if not progressed:
                raise ValueError("DISCONNECTED_OR_CYCLIC_MODEL")
        return frames

    def jacobian(self, side, q):
        frames, joints = self._chain(side, q)
        point = frames[f"{side}_fr3_side_suction_tcp"][:3, 3]
        columns = []
        for frame, axis in joints:
            world_axis = frame[:3, :3] @ axis
            columns.append(np.r_[np.cross(world_axis, point-frame[:3, 3]), world_axis])
        return np.column_stack(columns)

    def ik(self, side, target, seeds=None, max_attempts=8, max_nfev=300):
        target = checked_transform(target)
        if side not in self.chains or type(max_attempts) is not int or not 1 <= max_attempts <= 32 or type(max_nfev) is not int or not 1 <= max_nfev <= 2000:
            raise ValueError("INVALID_IK_BUDGET_OR_SIDE")
        base = np.asarray(self.manifest["bases_world_m"][side])
        if np.linalg.norm(target[:3, 3]-base) > self.reach_bounds[side] + 1e-12:
            return IKResult(False, (), None, None, "IK_OUTSIDE_REACH_BOUND", 0, 0,
                            float(np.linalg.norm(target[:3, 3]-base)-self.reach_bounds[side]))
        rng = np.random.default_rng(28)
        candidates = [self._q(q) for q in seeds] if seeds is not None else []
        candidates += [self.home, (self.joint_lower+self.joint_upper)/2]
        candidates += [rng.uniform(self.joint_lower,self.joint_upper) for _ in range(max_attempts)]
        total, best = 0, None
        def error(q):
            actual = self.fk(side, q)
            rotation = Rotation.from_matrix(target[:3,:3].T @ actual[:3,:3]).as_rotvec()
            return np.r_[actual[:3,3]-target[:3,3], .2*rotation]
        for attempt, initial in enumerate(candidates[:max_attempts], 1):
            if not self.within_limits(initial):
                raise ValueError("IK_SEED_OUT_OF_LIMITS")
            result = least_squares(error, initial, bounds=(self.joint_lower,self.joint_upper), method="trf",
                                   max_nfev=max_nfev, ftol=1e-11, xtol=1e-11, gtol=1e-11)
            total += int(result.nfev)
            pos, rot = residual(self.fk(side,result.x),target)
            row = IKResult(pos <= 1e-6 and rot <= 1e-5 and self.within_limits(result.x),tuple(float(x) for x in result.x),pos,rot,
                           "IK_VERIFIED" if pos <= 1e-6 and rot <= 1e-5 else "IK_NO_SOLUTION_WITHIN_BUDGET",attempt,total)
            if best is None or pos+.2*rot < best.translation_error_m+.2*best.rotation_error_rad:
                best = row
            if row.success:
                return row
        return IKResult(False,best.q,best.translation_error_m,best.rotation_error_rad,"IK_NO_SOLUTION_WITHIN_BUDGET",max_attempts,total)
