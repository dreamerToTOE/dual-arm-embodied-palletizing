#!/usr/bin/env python3
"""Replay the frozen Task28 conservative collision envelopes without Isaac/ROS.

Requires Python 3, numpy 1.24.4, scipy 1.8.0. The frozen point snapshot is the
only geometry input. --check writes only to a temporary directory and compares
regenerated STL bytes with the expected directory. --output-dir must name a new,
nonexistent directory; existing outputs are never overwritten.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import tempfile

import numpy as np
import scipy
from scipy.spatial import ConvexHull

EXPECTED_NUMPY = "1.24.4"
EXPECTED_SCIPY = "1.8.0"
PAD_M = 1e-6
REQUIRED_INTERIOR_MARGIN_M = 5e-7
STL_HEADER = b"Task28 conservative union hull; units=m; not physical validation".ljust(80, b" ")
LINKS = tuple(f"link{i}" for i in range(8))


def fail(message):
    raise ValueError(message)


def load_points(path):
    source = json.loads(path.read_text(encoding="utf-8"))
    if source.get("schema") != "task28.collision_source_point_clouds.v1":
        fail("Unexpected source point snapshot schema")
    if source.get("units") != "m" or set(source.get("links", {})) != set(LINKS):
        fail("Expected metres and exactly link0 through link7")
    result = {}
    for name in LINKS:
        groups = {}
        for key in ("usd_points", "stl_points"):
            values = np.asarray(source["links"][name].get(key), dtype=np.float64)
            if values.ndim != 2 or values.shape[1] != 3 or values.shape[0] < 4:
                fail(f"Invalid point shape: {name}/{key}")
            if not np.isfinite(values).all():
                fail(f"Nonfinite point: {name}/{key}")
            groups[key] = np.unique(values, axis=0)
        result[name] = groups
    return result


def generate_link(name, groups):
    points = np.unique(np.concatenate([groups["usd_points"], groups["stl_points"]]), axis=0)
    hull = ConvexHull(points)
    center = points[hull.vertices].mean(axis=0)
    facet_distances = -(hull.equations[:, :3] @ center + hull.equations[:, 3])
    if float(min(facet_distances)) <= 0:
        fail(f"Hull center is not strictly interior: {name}")
    scale = 1 + PAD_M / min(facet_distances)
    expanded = center + scale * (points - center)
    triangles = []
    for simplex, equation in zip(hull.simplices, hull.equations):
        triangle = expanded[simplex].astype(np.float32).astype(np.float64)
        if np.dot(np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0]), equation[:3]) < 0:
            triangle = triangle[[0, 2, 1]]
        start = min(range(3), key=lambda j: tuple(triangle[j]))
        triangle = np.concatenate([triangle[start:], triangle[:start]])
        triangles.append(triangle)
    triangles.sort(key=lambda triangle: tuple(triangle.reshape(-1)))
    data = bytearray(STL_HEADER)
    data.extend(struct.pack("<I", len(triangles)))
    normals = []
    offsets = []
    edges = collections.Counter()
    volume = 0.0
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        length = np.linalg.norm(normal)
        if not math.isfinite(length) or length <= 0:
            fail(f"Degenerate exported triangle: {name}")
        normal /= length
        offset = -np.dot(normal, triangle[0])
        if np.dot(normal, center) + offset >= 0:
            fail(f"Exported triangle points inward: {name}")
        normals.append(normal)
        offsets.append(offset)
        data.extend(struct.pack("<12fH", *normal, *triangle.reshape(-1), 0))
        volume += np.dot(triangle[0], np.cross(triangle[1], triangle[2])) / 6
        for a, b in ((0, 1), (1, 2), (2, 0)):
            edges[tuple(sorted((tuple(triangle[a]), tuple(triangle[b]))))] += 1
    normals = np.asarray(normals)
    offsets = np.asarray(offsets)
    if set(edges.values()) != {2} or volume <= 0:
        fail(f"Exported mesh is not a closed, positive-volume manifold: {name}")
    maxima = {}
    for key, source_points in groups.items():
        maximum = float(np.max(source_points @ normals.T + offsets))
        if maximum > -REQUIRED_INTERIOR_MARGIN_M:
            fail(f"Exported mesh does not conservatively contain {name}/{key}: {maximum}")
        maxima[key] = maximum
    outlier = points[0] + np.asarray([1.0, 0.0, 0.0])
    if float(np.max(normals @ outlier + offsets)) <= 0:
        fail(f"Negative outlier was not rejected: {name}")
    return bytes(data), {
        "mesh_file": f"fr3_{name}_conservative_union.stl",
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
        "triangles": len(triangles),
        "radial_scale_factor": float(scale),
        "max_source_signed_face_distances_m": maxima,
        "source_points_contained": True,
        "negative_outlier_rejected": True,
    }


def generate_all(source, destination):
    destination.mkdir(exist_ok=False, parents=True)
    report = {}
    for name, groups in load_points(source).items():
        data, row = generate_link(name, groups)
        (destination / row["mesh_file"]).write_bytes(data)
        report[name] = row
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path, help="Frozen source_collision_points.json")
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument("--check", action="store_true", help="Regenerate in temporary storage and compare bytes")
    actions.add_argument("--output-dir", type=Path, help="Explicit new, nonexistent directory for regenerated STLs")
    parser.add_argument("--expected-dir", type=Path, help="For --check; defaults to source snapshot's directory")
    args = parser.parse_args()
    if np.__version__ != EXPECTED_NUMPY or scipy.__version__ != EXPECTED_SCIPY:
        fail(f"Pinned generator requires numpy {EXPECTED_NUMPY}, scipy {EXPECTED_SCIPY}; found {np.__version__}, {scipy.__version__}")
    source = args.source.resolve(strict=True)
    if args.check:
        expected = (args.expected_dir or source.parent).resolve(strict=True)
        with tempfile.TemporaryDirectory(prefix="task28-envelope-check-") as tmp:
            report = generate_all(source, Path(tmp) / "meshes")
            for name, row in report.items():
                actual = expected / row["mesh_file"]
                if not actual.is_file() or hashlib.sha256(actual.read_bytes()).hexdigest() != row["sha256"]:
                    fail(f"Regenerated mesh differs: {actual}")
                print(f"{name}: byte-identical; containment PASS; outlier REJECTED")
    else:
        if args.expected_dir is not None:
            fail("--expected-dir is valid only with --check")
        target = args.output_dir.resolve()
        if target.exists():
            fail(f"Refusing to overwrite existing output directory: {target}")
        report = generate_all(source, target)
        print(json.dumps({"source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "numpy": np.__version__, "scipy": scipy.__version__, "links": report}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (ValueError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
