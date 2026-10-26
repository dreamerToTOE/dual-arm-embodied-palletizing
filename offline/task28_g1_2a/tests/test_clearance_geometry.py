"""Independent analytic and LP oracles for actual complete-Cube geometry."""
from pathlib import Path
import shutil

import numpy as np
import pytest
from scipy.optimize import linprog
from scipy.spatial.transform import Rotation

from task28_contact.clearance_geometry import ClearanceGeometry
from task28_contact.support import SupportPolicy


FIXTURE = Path(__file__).resolve().parents[3] / "fixtures/task28_g1_0_new_single_cube_v0"


@pytest.fixture(scope="module")
def geometry():
    return ClearanceGeometry(SupportPolicy(FIXTURE))


def pose(x=.65, y=0., z=.26, angles=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", angles).as_matrix()
    result[:3, 3] = (x, y, z)
    return result


@pytest.mark.parametrize("gap", [0., .0005, .001, .002, .003, .0031])
def test_axis_aligned_cube_observed_clearance(geometry, gap):
    result = geometry.evaluate(pose(z=.26 + gap))
    assert result.valid, result.reasons
    assert result.clearance_m == pytest.approx(gap, abs=1e-15)
    assert result.full_vertical_clearance_m == pytest.approx(gap, abs=1e-15)
    assert result.footprint_overlap and result.fully_inside_table_footprint
    assert len(result.vertices_world_m) == 8
    # The two planes are independently defined from the frozen Cube dimensions.
    assert sorted({round(point[2], 12) for point in result.vertices_world_m}) == [round(.2 + gap, 12), round(.32 + gap, 12)]


@pytest.mark.parametrize("angles", [(.2, 0., 0.), (0., -.3, 0.), (.2, -.3, .4),
                                    (np.pi/2, 0., 0.), (np.pi, .1, -.3)])
def test_rotated_obb_matches_support_function_not_center_height(geometry, angles):
    matrix = pose(z=.45, angles=angles)
    result = geometry.evaluate(matrix)
    # Analytic support function: h_z = sum_i h_i |R_zi|. This does not
    # enumerate corners or use the implementation's clipping routine.
    expected = .45 - .06 * sum(abs(value) for value in matrix[2, :3]) - .2
    assert result.valid
    assert result.clearance_m == pytest.approx(expected, abs=2e-15)
    assert result.clearance_m != pytest.approx(.45 - .06 - .2, abs=1e-6) or angles[0] == np.pi/2


def test_rotated_cube_center_clearance_cannot_hide_a_low_corner(geometry):
    matrix = pose(z=.27, angles=(np.pi/4, 0., 0.))
    result = geometry.evaluate(matrix)
    assert .27 - .06 - .2 > .003  # An incorrect axis-aligned computation passes.
    assert result.clearance_m == pytest.approx(.27 - .06*np.sqrt(2) - .2, abs=1e-15)
    assert not result.valid
    assert "CLEARANCE_TABLE_PENETRATION" in result.reasons


def test_partial_overhang_clips_out_the_lowest_corner(geometry):
    # Pitch -45 degrees: the lowest corner lies at the Cube center's world x,
    # while the leftmost corner lies .06*sqrt(2) left of it at center height.
    # Put only a 10 mm x slice of the Cube over the table's right edge.
    theta = -np.pi/4
    matrix = pose(x=1.15+.06*np.sqrt(2)-.01, z=.4, angles=(0., theta, 0.))
    result = geometry.evaluate(matrix)
    assert result.footprint_overlap and not result.fully_inside_table_footprint
    assert result.reasons == ("CLEARANCE_OUTSIDE_TABLE_FOOTPRINT",)
    # Left-facing rotated edge has z=.4 when x=center_x-.06*sqrt(2).
    # Moving inward across the 10 mm overlap lowers this edge by 10 mm.
    assert result.clearance_m == pytest.approx(.4 - .01 - .2, abs=2e-15)
    assert result.full_vertical_clearance_m == pytest.approx(.4 - .06*np.sqrt(2) - .2, abs=2e-15)
    assert result.clearance_m > result.full_vertical_clearance_m + .05
    assert all(point[0] <= 1.15 + 1e-12 for point in result.clipped_vertices_world_m)


@pytest.mark.parametrize("x,y", [(1.3, 0.), (-.2, 0.), (.65, .6), (.65, -.6)])
def test_disjoint_projection_has_no_invented_table_clearance(geometry, x, y):
    result = geometry.evaluate(pose(x=x, y=y, z=.4))
    assert not result.valid and not result.footprint_overlap
    assert result.clearance_m is None
    assert result.full_vertical_clearance_m == pytest.approx(.14)
    assert result.reasons == ("CLEARANCE_NO_TABLE_FOOTPRINT_OVERLAP",)


@pytest.mark.parametrize("angle", [.1, .4, .8])
def test_footprint_disjoint_even_when_projected_axis_aligned_bounds_overlap(geometry, angle):
    # Exact polygon clipping, not a projected AABB overlap test. A yawed corner
    # is near the table's top-right corner but the diamond misses the rectangle.
    extent = .06 * (np.cos(angle) + np.sin(angle))
    matrix = pose(x=1.15 + .9*extent, y=.4 + .9*extent, z=.4, angles=(0., 0., angle))
    result = geometry.evaluate(matrix)
    assert min(point[0] for point in result.vertices_world_m) < 1.15
    assert min(point[1] for point in result.vertices_world_m) < .4
    assert not result.footprint_overlap
    assert result.clearance_m is None


@pytest.mark.parametrize("angles,x,y", [((.2, -.3, .4), 1.12, .0),
                                         ((-.7, .4, -.2), 1.16, .38),
                                         ((1.1, -.9, .3), -.04, -.39),
                                         ((.01, .02, .03), .65, .0),
                                         ((np.pi/2, 0., 0.), 1.15, .0)])
def test_clipped_minimum_matches_independent_linear_program(geometry, angles, x, y):
    matrix = pose(x=x, y=y, z=.5, angles=angles)
    result = geometry.evaluate(matrix)
    # Optimize directly in local coordinates u. Table constraints are on
    # R_xy u + t_xy; the implementation optimizes world polyhedron vertices.
    rotation = matrix[:3, :3]
    coefficients = np.vstack((rotation[:2], -rotation[:2]))
    bounds = np.r_[np.array([1.15, .4]) - matrix[:2, 3],
                   matrix[:2, 3] - np.array([-.05, -.4])]
    oracle = linprog(rotation[2], A_ub=coefficients, b_ub=bounds,
                     bounds=[(-.06, .06)]*3, method="highs")
    assert oracle.success
    assert result.clearance_m == pytest.approx(oracle.fun + .5 - .2, abs=2e-10)


@pytest.mark.parametrize("penetration,valid", [(5e-7, True), (2e-6, False), (.02, False)])
def test_penetration_uses_unchanged_micrometre_tolerance(geometry, penetration, valid):
    result = geometry.evaluate(pose(z=.26 - penetration))
    assert result.valid is valid
    assert result.clearance_m == pytest.approx(-penetration, abs=1e-15)
    assert geometry.policy.DEPTH_TOLERANCE_M == 1e-6


@pytest.mark.parametrize("bad", [np.eye(3), np.eye(4, dtype=complex),
                                 np.eye(4, dtype=bool), np.full((4, 4), np.nan)])
def test_invalid_observed_pose_fails_closed(geometry, bad):
    result = geometry.evaluate(bad)
    assert not result.valid
    assert result.clearance_m is None
    assert result.reasons[0].startswith("INVALID_CLEARANCE_POSE:")


def test_policy_mutation_is_not_a_clearance_geometry_override():
    policy = SupportPolicy(FIXTURE)
    geometry = ClearanceGeometry(policy)
    object.__setattr__(policy, "DEPTH_TOLERANCE_M", .1)
    result = geometry.evaluate(pose())
    assert result.reasons == ("SUPPORT_POLICY_CONFIGURATION_CHANGED",)
    assert result.clearance_m is None


def test_nonfixture_policy_is_not_accepted():
    with pytest.raises(ValueError, match="CLEARANCE_REQUIRES_FROZEN_SUPPORT_POLICY"):
        ClearanceGeometry(object())


@pytest.mark.parametrize("damage", ["modify", "missing"])
def test_frozen_dependency_integrity_is_checked_before_geometry(tmp_path, damage):
    isolated = tmp_path / "fixture"
    shutil.copytree(FIXTURE, isolated)
    dependency = isolated / "scene/task28_g1_0_scene_contract.json"
    if damage == "modify":
        dependency.write_text(dependency.read_text() + "\n")
    else:
        dependency.unlink()
    with pytest.raises(ValueError, match="FIXTURE_FILE_HASH_MISMATCH"):
        ClearanceGeometry(SupportPolicy(isolated))
