#!/usr/bin/env python3
"""Independently verify frozen planning envelopes using only the stdlib."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x - y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def check_envelopes(root):
    """Require a closed convex envelope containing both source point sets.

    The signed triangle planes are recomputed from the delivered float32 STL,
    rather than trusting the generation report's normals or containment flag.
    Every original surface triangle is contained if all of its vertices are.
    """
    report = json.loads((root / "conservative_collision_provenance.json").read_text())
    source_path = root / report["source_point_cloud_snapshot"]["file"]
    if hashlib.sha256(source_path.read_bytes()).hexdigest() != report["source_point_cloud_snapshot"]["sha256"]:
        raise ValueError("source point snapshot hash mismatch")
    clouds = json.loads(source_path.read_text())
    if set(report["links"]) != {f"link{i}" for i in range(8)}:
        raise ValueError("must freeze exactly eight arm-link envelopes")
    for name, row in report["links"].items():
        data = (root / row["mesh_file"]).read_bytes()
        if hashlib.sha256(data).hexdigest() != row["sha256"]:
            raise ValueError("planning envelope hash mismatch: " + name)
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) != 84 + 50 * count:
            raise ValueError("invalid binary STL: " + name)
        planes, edges, volume = [], Counter(), 0.0
        for index in range(count):
            values = struct.unpack_from("<12fH", data, 84 + 50 * index)
            vertices = (values[3:6], values[6:9], values[9:12])
            normal = cross(subtract(vertices[1], vertices[0]), subtract(vertices[2], vertices[0]))
            length = math.sqrt(dot(normal, normal))
            if not math.isfinite(length) or length <= 0:
                raise ValueError("degenerate envelope triangle")
            normal = tuple(value / length for value in normal)
            offset = -dot(normal, vertices[0])
            if dot(normal, row["interior_center_m"]) + offset >= 0:
                raise ValueError("inward triangle winding")
            planes.append((normal, offset))
            volume += dot(vertices[0], cross(vertices[1], vertices[2])) / 6
            for first, second in ((0, 1), (1, 2), (2, 0)):
                edges[tuple(sorted((vertices[first], vertices[second])))] += 1
        if set(edges.values()) != {2} or volume <= 0:
            raise ValueError("envelope is not closed with positive volume")
        for source in ("usd_points", "stl_points"):
            points = clouds["links"][name][source]
            max_distance = max(dot(normal, point) + offset for point in points for normal, offset in planes)
            if max_distance > -5e-7:
                raise ValueError(f"envelope excludes original {source}: {name}: {max_distance}")
        if max(dot(normal, row["negative_outlier_m"]) + offset for normal, offset in planes) <= 0:
            raise ValueError("negative outlier was accepted: " + name)
    return len(report["links"])


if __name__ == "__main__":
    fixture = Path(__file__).resolve().parents[1]
    count = check_envelopes(fixture / "moveit/task28_single_cube_description/meshes")
    print(f"Collision envelopes PASS: {count} source containments, closed meshes and outlier rejections")
