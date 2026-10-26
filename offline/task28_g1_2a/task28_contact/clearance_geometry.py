"""Observed OBB-to-tabletop vertical clearance, including finite footprint.

T_world_object maps Cube coordinates into world coordinates (column vectors).
Only a validated observed Cube transform is accepted; this module has no TCP,
command, force, or temporal input. The frozen rectangular table is horizontal.
For a partial projection, the diagnostic clearance is the minimum z of the
Cube's intersection with the vertical prism above the finite tabletop. Such a
partial projection is deliberately ineligible for phase confirmation. This is
single-pose geometry, and makes no statement about motion between poses.
"""
from __future__ import annotations

from dataclasses import dataclass
import itertools

import numpy as np

from task28_validator.se3 import checked_transform
from .support import SupportPolicy


# Floating-point feasibility only; never a contact/penetration allowance or a
# research hysteresis threshold. The existing SupportPolicy tolerances govern
# contact and penetration independently.
_CLIP_NUMERICAL_EPS_M = 1e-12


@dataclass(frozen=True)
class ClearanceGeometryResult:
    valid: bool
    reasons: tuple[str, ...]
    clearance_m: float | None
    full_vertical_clearance_m: float | None
    footprint_overlap: bool
    fully_inside_table_footprint: bool
    vertices_world_m: tuple[tuple[float, float, float], ...]
    clipped_vertices_world_m: tuple[tuple[float, float, float], ...]


def _points(values):
    return tuple(tuple(float(number) for number in point) for point in values)


def _empty(reason):
    return ClearanceGeometryResult(False, (reason,), None, None, False, False, (), ())


def _clipped_vertices(rotation, center, half_size, table_lower, table_upper):
    """Vertices of OBB intersected with x/y table slabs by plane enumeration.

The OBB contributes six halfspaces and the rectangular prism contributes four.
A linear z objective achieves its minimum at a vertex of this bounded convex
polyhedron. Enumerating the 120 plane triples also handles edge/face contact
with the prism and does not extrapolate to an infinite supporting plane.
"""
    normals, bounds = [], []
    for axis in range(3):
        normal = rotation[:, axis]
        for sign in (-1., 1.):
            normals.append(sign * normal)
            bounds.append(float(half_size[axis] + sign * np.dot(normal, center)))
    for axis in range(2):
        normal = np.zeros(3)
        normal[axis] = 1.
        normals.extend((normal, -normal))
        bounds.extend((float(table_upper[axis]), float(-table_lower[axis])))
    normals, bounds = np.asarray(normals), np.asarray(bounds)
    vertices = []
    for chosen in itertools.combinations(range(10), 3):
        matrix = normals[list(chosen)]
        if abs(float(np.linalg.det(matrix))) <= 1e-12:
            continue
        point = np.linalg.solve(matrix, bounds[list(chosen)])
        if np.all(normals @ point <= bounds + _CLIP_NUMERICAL_EPS_M):
            # Duplicate vertices do not affect the minimum, but bounded output
            # and deterministic source-order improve machine-readable evidence.
            if not any(np.max(np.abs(point - previous)) <= _CLIP_NUMERICAL_EPS_M
                       for previous in vertices):
                vertices.append(point)
    return vertices


@dataclass(frozen=True)
class ClearanceGeometry:
    policy: SupportPolicy

    def __post_init__(self):
        if type(self.policy) is not SupportPolicy:
            raise ValueError("CLEARANCE_REQUIRES_FROZEN_SUPPORT_POLICY")
        if not self.policy._configuration_intact():
            raise ValueError("SUPPORT_POLICY_CONFIGURATION_CHANGED")

    def evaluate(self, object_pose) -> ClearanceGeometryResult:
        """Compute signed clearance from all eight transformed Cube corners.

        `valid` additionally requires full table-footprint coverage and no
        penetration exceeding the unchanged 1 um frozen tolerance. A positive
        number means geometrically separated; it never implies support merely
        because it is below a millimetre-scale phase-confirmation threshold.
        """
        if type(self.policy) is not SupportPolicy or not self.policy._configuration_intact():
            return _empty("SUPPORT_POLICY_CONFIGURATION_CHANGED")
        try:
            pose = checked_transform(object_pose)
        except (ValueError, TypeError, OverflowError) as error:
            return _empty(f"INVALID_CLEARANCE_POSE:{error}")
        local = np.asarray(list(itertools.product(*[(-half, half) for half in self.policy.half_size])))
        rotation, center = pose[:3, :3], pose[:3, 3]
        world = (rotation @ local.T).T + center
        lower, upper = self.policy.table_bounds_xy
        inside = bool(np.all(world[:, :2] >= lower - _CLIP_NUMERICAL_EPS_M)
                      and np.all(world[:, :2] <= upper + _CLIP_NUMERICAL_EPS_M))
        full_clearance = float(world[:, 2].min() - self.policy.table_top_z_m)
        clipped = list(world) if inside else _clipped_vertices(
            rotation, center, self.policy.half_size, lower, upper)
        overlap = bool(clipped)
        clearance = (float(min(point[2] for point in clipped) - self.policy.table_top_z_m)
                     if overlap else None)
        reasons = []
        if not overlap:
            reasons.append("CLEARANCE_NO_TABLE_FOOTPRINT_OVERLAP")
        elif not inside:
            reasons.append("CLEARANCE_OUTSIDE_TABLE_FOOTPRINT")
        if clearance is not None and clearance < -self.policy.DEPTH_TOLERANCE_M:
            reasons.append("CLEARANCE_TABLE_PENETRATION")
        return ClearanceGeometryResult(not reasons, tuple(reasons), clearance, full_clearance,
                                       overlap, inside, _points(world), _points(clipped))
