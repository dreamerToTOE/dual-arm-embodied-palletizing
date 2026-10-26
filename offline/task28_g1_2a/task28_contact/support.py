"""Strict local Cube/table support geometry, never an ACM exemption.

Column vectors and T_world_object are inherited from G1-1; metres/radians.
Geometry may establish a numerical near-zero support plane even when exact-touch
FCL returns no contact. This is an offline geometric classification, not measured
support, force, friction, attachment or stability. An actual reported contact is
accepted only if its location, normal and depth independently meet this policy.
Ill-conditioned FCL normals are rejected; the angular tolerance is not widened.
"""
from __future__ import annotations

from dataclasses import dataclass
import itertools
import json
import math
from pathlib import Path
from types import MappingProxyType

import numpy as np

from task28_validator.model import verify_fixture
from task28_validator.se3 import checked_transform, real_array


_FROZEN_TOLERANCES = (
    ("PLANE_TOLERANCE_M", 1e-6),
    ("DEPTH_TOLERANCE_M", 1e-6),
    ("FOOTPRINT_TOLERANCE_M", 1e-6),
    ("BOTTOM_NORMAL_ANGLE_RAD", 1e-6),
    ("CONTACT_NORMAL_ANGLE_RAD", 1e-6),
    ("CONTACT_NORMAL_NORM_TOLERANCE", 1e-6),
    ("MAX_CONTACT_COUNT", 128),
)


@dataclass(frozen=True)
class SupportGeometryResult:
    valid: bool
    reasons: tuple[str, ...]
    site: str
    bottom_gap_m: float | None
    bottom_gap_range_m: tuple[float, float] | None
    bottom_normal_angle_rad: float | None
    max_penetration_m: float | None
    max_site_overhang_m: float | None
    max_table_overhang_m: float | None
    bottom_corners_world_m: tuple[tuple[float, float, float], ...]
    numerical_near_zero_support: bool = False
    physical_support_observed: bool = False

    @property
    def metrics(self):
        return {
            "bottom_gap_m": self.bottom_gap_m,
            "bottom_gap_range_m": self.bottom_gap_range_m,
            "bottom_normal_angle_rad": self.bottom_normal_angle_rad,
            "max_penetration_m": self.max_penetration_m,
            "max_site_overhang_m": self.max_site_overhang_m,
            "max_table_overhang_m": self.max_table_overhang_m,
            "numerical_near_zero_support": self.numerical_near_zero_support,
            "physical_support_observed": self.physical_support_observed,
        }


@dataclass(frozen=True)
class SupportContactResult:
    valid: bool
    reasons: tuple[str, ...]
    geometry: SupportGeometryResult
    normalized_contacts: tuple[dict, ...]
    # Normals below are always Cube -> table, regardless of FCL owner order.
    normal_convention: str = "Cube_01_to_task28_table"


class SupportPolicy:
    """Only the frozen supply/target rectangular patches can support this Cube.

    Each patch is the frozen Cube's 0.12 x 0.12 m footprint. Consequently a
    translated or yawed Cube can overhang a patch even while fully on the broad
    table. That conservative rejection must not be fixed by resizing geometry.
    """

    PLANE_TOLERANCE_M = 1e-6
    DEPTH_TOLERANCE_M = 1e-6
    FOOTPRINT_TOLERANCE_M = 1e-6
    BOTTOM_NORMAL_ANGLE_RAD = 1e-6
    CONTACT_NORMAL_ANGLE_RAD = 1e-6
    CONTACT_NORMAL_NORM_TOLERANCE = 1e-6
    MAX_CONTACT_COUNT = 128

    def __setattr__(self, name, value):
        if getattr(self, "_sealed", False):
            raise AttributeError("SUPPORT_POLICY_IS_FROZEN")
        object.__setattr__(self, name, value)

    @staticmethod
    def _readonly(values):
        array = np.asarray(values, dtype=float).copy()
        array.setflags(write=False)
        return array

    def __init__(self, fixture_dir):
        self.fixture_dir = Path(fixture_dir).resolve()
        verify_fixture(self.fixture_dir)
        scene = json.loads((self.fixture_dir / "scene/task28_g1_0_scene_contract.json").read_text())
        geometry = json.loads((self.fixture_dir / "moveit/task28_single_cube_description/config/fixture_geometry.json").read_text())
        if (scene["cube"]["id"] != "Cube_01" or scene["cube"]["index"] != 0
                or scene["active_cube_count"] != 1 or geometry["cube_id"] != "Cube_01"
                or geometry["table_id"] != "task28_table"
                or scene["cube"]["size_m"] != geometry["cube_size_m"]
                or scene["table"]["center_world_m"] != geometry["table_center_world_m"]
                or scene["table"]["size_m"] != geometry["table_size_m"]):
            raise ValueError("SUPPORT_FROZEN_GEOMETRY_MISMATCH")
        self.half_size = self._readonly(np.asarray(scene["cube"]["size_m"], dtype=float) / 2)
        table_center = np.asarray(scene["table"]["center_world_m"], dtype=float)
        table_half = np.asarray(scene["table"]["size_m"], dtype=float) / 2
        self.table_top_z_m = float(table_center[2] + table_half[2])
        if abs(self.table_top_z_m - scene["table"]["top_z_m"]) > 1e-12:
            raise ValueError("SUPPORT_TABLE_TOP_MISMATCH")
        self.table_bounds_xy = (self._readonly(table_center[:2] - table_half[:2]),
                                self._readonly(table_center[:2] + table_half[:2]))
        self.site_centers = MappingProxyType({
            "supply": self._readonly(scene["cube"]["initial_pose_world_m"][:2]),
            "target": self._readonly(scene["cube"]["target_pose_world_m"][:2]),
        })
        self._expected_geometry = self._geometry_signature()
        self._sealed = True

    def _geometry_signature(self):
        return (tuple(self.half_size), self.table_top_z_m,
                tuple(tuple(bound) for bound in self.table_bounds_xy),
                tuple((name, tuple(self.site_centers[name])) for name in ("supply", "target")))

    def _configuration_intact(self):
        """Catch accidental instance/class tolerance or geometry relaxation.

        Normal assignment and direct array edits are blocked. The check also
        fails closed after object-level replacement, without treating Python's
        unrestricted reflection as a security boundary.
        """
        try:
            if any(type(getattr(self, name)) is not type(expected)
                   or getattr(self, name) != expected
                   for name, expected in _FROZEN_TOLERANCES):
                return False
            arrays = (self.half_size, *self.table_bounds_xy, *self.site_centers.values())
            return (isinstance(self.site_centers, MappingProxyType)
                    and tuple(self.site_centers) == ("supply", "target")
                    and all(type(array) is np.ndarray and not array.flags.writeable
                            and np.isfinite(array).all() for array in arrays)
                    and self._geometry_signature() == self._expected_geometry)
        except (TypeError, ValueError, AttributeError, KeyError):
            return False

    @staticmethod
    def _angle(first, second):
        return float(math.atan2(np.linalg.norm(np.cross(first, second)), np.dot(first, second)))

    def geometry(self, object_pose, site: str) -> SupportGeometryResult:
        """Check all eight vertices, the four bottom vertices and bottom normal.

        Nonpenetration is checked against the table top plane for all vertices;
        all bottom points must be within 1 um of that plane and the intended
        patch and physical table bounds. No contact enum authorizes geometry.
        """
        if not self._configuration_intact():
            return SupportGeometryResult(False, ("SUPPORT_POLICY_CONFIGURATION_CHANGED",), str(site),
                                         None, None, None, None, None, None, ())
        if not isinstance(site, str) or site not in self.site_centers:
            return SupportGeometryResult(False, ("INVALID_SUPPORT_SITE",), str(site),
                                         None, None, None, None, None, None, ())
        try:
            pose = checked_transform(object_pose)
        except (ValueError, TypeError, OverflowError) as error:
            return SupportGeometryResult(False, (f"INVALID_SUPPORT_POSE:{error}",), site,
                                         None, None, None, None, None, None, ())
        local = np.asarray(list(itertools.product(*[(-half, half) for half in self.half_size])))
        world = (pose[:3, :3] @ local.T).T + pose[:3, 3]
        bottom = world[local[:, 2] == -self.half_size[2]]
        gaps = bottom[:, 2] - self.table_top_z_m
        bottom_normal = -pose[:3, 2]
        normal_angle = self._angle(bottom_normal, np.array([0., 0., -1.]))
        penetration = max(0., float(self.table_top_z_m - world[:, 2].min()))
        center = self.site_centers[site]
        lower, upper = center - self.half_size[:2], center + self.half_size[:2]
        site_overhang = max(0., float(np.max(lower - bottom[:, :2])),
                            float(np.max(bottom[:, :2] - upper)))
        table_lower, table_upper = self.table_bounds_xy
        table_overhang = max(0., float(np.max(table_lower - bottom[:, :2])),
                             float(np.max(bottom[:, :2] - table_upper)))
        reasons = []
        if penetration > self.DEPTH_TOLERANCE_M:
            reasons.append("SUPPORT_TABLE_PENETRATION")
        if float(np.max(np.abs(gaps))) > self.PLANE_TOLERANCE_M:
            reasons.append("SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE")
        if normal_angle > self.BOTTOM_NORMAL_ANGLE_RAD:
            reasons.append("SUPPORT_BOTTOM_NORMAL_MISMATCH")
        if site_overhang > self.FOOTPRINT_TOLERANCE_M:
            reasons.append("SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT")
        if table_overhang > self.FOOTPRINT_TOLERANCE_M:
            reasons.append("SUPPORT_OUTSIDE_TABLE_FOOTPRINT")
        return SupportGeometryResult(
            not reasons, tuple(reasons), site, float(np.mean(gaps)),
            (float(gaps.min()), float(gaps.max())), normal_angle, penetration,
            site_overhang, table_overhang,
            tuple(tuple(float(v) for v in point) for point in bottom), not reasons, False)

    def accept_contact(self, record, object_pose, site: str) -> SupportContactResult:
        """Check only one actual G1-1 Cube/table record, with fail-closed schema.

        FCL normal is a -> b. Owner order is normalized explicitly; signed normal
        direction is required, so an upside-down/sideways zero-depth normal is
        rejected. Count 128 can indicate bounded-report truncation and is rejected.
        No record is fabricated when the geometric near-zero test has no FCL hit.
        """
        geometry = self.geometry(object_pose, site)
        reasons = list(geometry.reasons)
        normalized = []
        expected_keys = {"shape_a", "shape_b", "owner_a", "owner_b", "contacts"}
        if not isinstance(record, dict) or set(record) != expected_keys:
            return SupportContactResult(False, tuple(reasons + ["INVALID_SUPPORT_CONTACT_RECORD"]), geometry, ())
        if any(type(record[key]) is not str for key in ("shape_a", "shape_b", "owner_a", "owner_b")):
            return SupportContactResult(False, tuple(reasons + ["INVALID_SUPPORT_CONTACT_ID_TYPE"]), geometry, ())
        order = (record["owner_a"], record["owner_b"])
        if order == ("Cube_01", "task28_table"):
            sign, expected_shapes = 1., ("Cube_01/box", "task28_table/box")
        elif order == ("task28_table", "Cube_01"):
            sign, expected_shapes = -1., ("task28_table/box", "Cube_01/box")
        else:
            return SupportContactResult(False, tuple(reasons + ["NOT_CUBE_TABLE_CONTACT"]), geometry, ())
        if (record["shape_a"], record["shape_b"]) != expected_shapes:
            reasons.append("INVALID_SUPPORT_CONTACT_SHAPE")
        contacts = record["contacts"]
        if not isinstance(contacts, list) or not 1 <= len(contacts) < self.MAX_CONTACT_COUNT:
            return SupportContactResult(False, tuple(reasons + ["SUPPORT_CONTACT_COUNT_OR_TRUNCATION"]), geometry, ())
        if not geometry.valid:
            return SupportContactResult(False, tuple(reasons), geometry, ())
        pose = checked_transform(object_pose)
        center = self.site_centers[site]
        lower, upper = center - self.half_size[:2], center + self.half_size[:2]
        table_lower, table_upper = self.table_bounds_xy
        for index, contact in enumerate(contacts):
            prefix = f"SUPPORT_CONTACT_{index}:"
            try:
                if not isinstance(contact, dict) or set(contact) != {"depth_m", "normal_world", "position_world_m"}:
                    raise ValueError("INVALID_SCHEMA")
                if type(contact["depth_m"]) not in (int, float):
                    raise ValueError("INVALID_DEPTH_TYPE")
                depth = float(contact["depth_m"])
                normal = real_array(contact["normal_world"])
                point = real_array(contact["position_world_m"])
                if normal.shape != (3,) or point.shape != (3,) or not np.isfinite([depth, *normal, *point]).all():
                    raise ValueError("INVALID_VECTOR_OR_NONFINITE")
                length = float(np.linalg.norm(normal))
                if abs(length - 1.) > self.CONTACT_NORMAL_NORM_TOLERANCE:
                    raise ValueError("NONUNIT_NORMAL")
                cube_to_table = sign * normal
                angle = self._angle(cube_to_table, np.array([0., 0., -1.]))
                point_object = pose[:3, :3].T @ (point - pose[:3, 3])
                if abs(depth) > self.DEPTH_TOLERANCE_M:
                    reasons.append(prefix + "DEPTH_EXCEEDED")
                if angle > self.CONTACT_NORMAL_ANGLE_RAD:
                    reasons.append(prefix + "UNEXPECTED_NORMAL")
                if (abs(point[2] - self.table_top_z_m) > self.PLANE_TOLERANCE_M
                        or abs(point_object[2] + self.half_size[2]) > self.PLANE_TOLERANCE_M):
                    reasons.append(prefix + "NOT_BOTTOM_TOP_PLANE")
                if (np.any(point[:2] < lower - self.FOOTPRINT_TOLERANCE_M)
                        or np.any(point[:2] > upper + self.FOOTPRINT_TOLERANCE_M)
                        or np.any(point[:2] < table_lower - self.FOOTPRINT_TOLERANCE_M)
                        or np.any(point[:2] > table_upper + self.FOOTPRINT_TOLERANCE_M)
                        or np.any(np.abs(point_object[:2]) > self.half_size[:2] + self.FOOTPRINT_TOLERANCE_M)):
                    reasons.append(prefix + "OUTSIDE_SUPPORT_FOOTPRINT")
                normalized.append({
                    "depth_m": depth,
                    "normal_cube_to_table_world": tuple(float(v) for v in cube_to_table),
                    "position_world_m": tuple(float(v) for v in point),
                    "normal_angle_rad": angle,
                })
            except (ValueError, TypeError, OverflowError) as error:
                reasons.append(prefix + str(error))
        return SupportContactResult(not reasons, tuple(reasons), geometry, tuple(normalized))
