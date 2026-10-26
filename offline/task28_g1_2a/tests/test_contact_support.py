"""Independent rectangular-box arithmetic and real-FCL support controls."""
import copy
from pathlib import Path
from types import MappingProxyType

import numpy as np
import pytest

from task28_contact.support import SupportPolicy
from task28_validator.collision import CollisionValidator


FIXTURE = Path(__file__).resolve().parents[3] / "fixtures/task28_g1_0_new_single_cube_v0"


@pytest.fixture(scope="module")
def policy():
    return SupportPolicy(FIXTURE)


def pose(x=.65, y=0., z=.26):
    matrix = np.eye(4)
    matrix[:3, 3] = [x, y, z]
    return matrix


def record(point=(.65, 0., .2), normal=(0., 0., -1.), depth=0., reverse=False):
    result = {"shape_a": "Cube_01/box", "shape_b": "task28_table/box",
              "owner_a": "Cube_01", "owner_b": "task28_table",
              "contacts": [{"position_world_m": list(point), "normal_world": list(normal), "depth_m": depth}]}
    if reverse:
        result["shape_a"], result["shape_b"] = result["shape_b"], result["shape_a"]
        result["owner_a"], result["owner_b"] = result["owner_b"], result["owner_a"]
        result["contacts"][0]["normal_world"] = [-v for v in normal]
    return result


@pytest.mark.parametrize("site,xyz", [("supply", [.65, 0., .26]), ("target", [.82, -.075, .26])])
def test_actual_frozen_support_geometry_with_independent_corner_arithmetic(policy, site, xyz):
    result = policy.geometry(pose(*xyz), site)
    assert result.valid, result.reasons
    # Eight corners have z in {.20, .32}; bottom points have the exact frozen
    # site's x/y +/- .06. This independent arithmetic does not call SE(3) code.
    expected = sorted((xyz[0]+dx, xyz[1]+dy, .20) for dx in (-.06, .06) for dy in (-.06, .06))
    assert np.array(sorted(result.bottom_corners_world_m)) == pytest.approx(np.asarray(expected), abs=1e-15)
    assert result.bottom_gap_m == pytest.approx(0., abs=1e-15)
    assert result.bottom_normal_angle_rad == 0.
    assert result.max_penetration_m < 1e-15
    assert result.numerical_near_zero_support
    assert not result.physical_support_observed


@pytest.mark.parametrize("penetration", [.02, 2e-6])
def test_real_penetration_is_never_support(policy, penetration):
    result = policy.geometry(pose(z=.26-penetration), "supply")
    assert not result.valid
    assert "SUPPORT_TABLE_PENETRATION" in result.reasons
    assert result.max_penetration_m == pytest.approx(penetration, abs=1e-15)


def test_numerical_tiny_positive_gap_is_classified_without_fabricating_contact(policy):
    result = policy.geometry(pose(z=.2600005), "supply")
    assert result.valid
    assert result.bottom_gap_m == pytest.approx(5e-7, abs=1e-15)
    assert result.numerical_near_zero_support
    assert not result.physical_support_observed
    assert not policy.geometry(pose(z=.260002), "supply").valid


def test_cube_on_table_but_off_designated_patch_is_rejected(policy):
    result = policy.geometry(pose(x=.651), "supply")
    assert not result.valid
    assert result.max_table_overhang_m == 0.
    assert result.max_site_overhang_m == pytest.approx(.001)
    assert "SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT" in result.reasons


def test_actual_table_edge_overhang_rejected(policy):
    result = policy.geometry(pose(x=1.11), "supply")
    assert not result.valid
    # Table right edge = .55 + 1.2/2 = 1.15; Cube right = 1.17.
    assert result.max_table_overhang_m == pytest.approx(.02)
    assert "SUPPORT_OUTSIDE_TABLE_FOOTPRINT" in result.reasons


def test_wrong_site_rejected_even_though_frozen_pose_is_supported(policy):
    assert not policy.geometry(pose(), "target").valid
    assert not policy.geometry(pose(.82, -.075, .26), "supply").valid


@pytest.mark.parametrize("kind", ["tilt", "upside_down", "yaw_patch_overhang"])
def test_rotation_requires_bottom_face_and_intended_footprint(policy, kind):
    matrix = pose()
    if kind == "tilt":
        angle = 2e-6
        matrix[:3, :3] = [[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]]
    elif kind == "upside_down":
        matrix[:3, :3] = np.diag([1., -1., -1.])
    else:
        angle = np.pi/4
        matrix[:3, :3] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    result = policy.geometry(matrix, "supply")
    assert not result.valid
    if kind != "yaw_patch_overhang":
        assert "SUPPORT_BOTTOM_NORMAL_MISMATCH" in result.reasons
    else:
        assert "SUPPORT_OUTSIDE_DESIGNATED_FOOTPRINT" in result.reasons


@pytest.mark.parametrize("bad_pose", [np.eye(3), np.eye(4, dtype=complex), np.eye(4, dtype=bool),
                                      [[1, 0], [0]], np.full((4, 4), np.nan)])
def test_bad_support_transforms_fail_closed(policy, bad_pose):
    result = policy.geometry(bad_pose, "supply")
    assert not result.valid
    assert result.reasons[0].startswith("INVALID_SUPPORT_POSE:")


@pytest.mark.parametrize("site", [None, "SUPPLY", "elsewhere", 1, []])
def test_unknown_support_site_fails_closed(policy, site):
    assert policy.geometry(pose(), site).reasons == ("INVALID_SUPPORT_SITE",)


@pytest.mark.parametrize("reverse", [False, True])
def test_zero_depth_contact_uses_owner_order_and_signed_normal(policy, reverse):
    result = policy.accept_contact(record(reverse=reverse), pose(), "supply")
    assert result.valid, result.reasons
    assert result.normalized_contacts[0]["normal_cube_to_table_world"] == (0., 0., -1.)


@pytest.mark.parametrize("normal", [(0., 0., 1.), (1., 0., 0.), (0., 1e-3, -np.sqrt(1-1e-6))])
def test_unexpected_or_numerically_unstable_normal_is_not_accepted(policy, normal):
    result = policy.accept_contact(record(normal=normal), pose(), "supply")
    assert not result.valid
    assert any("UNEXPECTED_NORMAL" in reason for reason in result.reasons)
    assert policy.CONTACT_NORMAL_ANGLE_RAD == 1e-6


@pytest.mark.parametrize("point", [(.65, 0., .200002), (.65, 0., .32), (.711, 0., .2), (.65, .061, .2)])
def test_wrong_contact_plane_or_footprint_is_rejected(policy, point):
    assert not policy.accept_contact(record(point=point), pose(), "supply").valid


@pytest.mark.parametrize("depth", [2e-6, -2e-6, .02])
def test_contact_depth_absolute_value_is_bounded(policy, depth):
    result = policy.accept_contact(record(depth=depth), pose(), "supply")
    assert not result.valid
    assert "SUPPORT_CONTACT_0:DEPTH_EXCEEDED" in result.reasons


@pytest.mark.parametrize("mutator", [
    lambda value: value.pop("owner_a"),
    lambda value: value.__setitem__("owner_a", "left_fr3_link0"),
    lambda value: value.__setitem__("owner_a", np.asarray(["Cube_01"])),
    lambda value: value.__setitem__("shape_a", "Cube_01/other"),
    lambda value: value.__setitem__("contacts", []),
    lambda value: value.__setitem__("contacts", value["contacts"] * 128),
    lambda value: value["contacts"][0].__setitem__("normal_world", [0., 0., 0.]),
    lambda value: value["contacts"][0].__setitem__("normal_world", [0., 0., -2.]),
    lambda value: value["contacts"][0].__setitem__("normal_world", [0., 0., float("nan")]),
    lambda value: value["contacts"][0].__setitem__("position_world_m", [True, False, True]),
    lambda value: value["contacts"][0].__setitem__("depth_m", True),
    lambda value: value["contacts"][0].__setitem__("depth_m", float("inf")),
    lambda value: value["contacts"][0].__setitem__("ignored_field", 1),
])
def test_malformed_or_truncated_contact_record_fails_closed(policy, mutator):
    value = record()
    mutator(value)
    result = policy.accept_contact(value, pose(), "supply")
    assert not result.valid
    assert result.reasons


def test_contact_record_cannot_override_bad_geometry(policy):
    result = policy.accept_contact(record(), pose(z=.24), "supply")
    assert not result.valid
    assert "SUPPORT_TABLE_PENETRATION" in result.reasons


def test_support_policy_blocks_normal_reassignment_and_array_edits():
    frozen = SupportPolicy(FIXTURE)
    with pytest.raises(AttributeError, match="SUPPORT_POLICY_IS_FROZEN"):
        frozen.PLANE_TOLERANCE_M = float("nan")
    with pytest.raises(AttributeError, match="SUPPORT_POLICY_IS_FROZEN"):
        frozen.site_centers = {"supply": np.array([.651, 0.])}
    with pytest.raises(TypeError):
        frozen.site_centers["supply"] = np.array([.651, 0.])
    for array in (frozen.half_size, *frozen.table_bounds_xy, *frozen.site_centers.values()):
        with pytest.raises(ValueError, match="read-only"):
            array[0] += .01
    assert frozen.geometry(pose(), "supply").valid


@pytest.mark.parametrize("name,value", [
    ("PLANE_TOLERANCE_M", float("nan")),
    ("PLANE_TOLERANCE_M", .03),
    ("DEPTH_TOLERANCE_M", .03),
    ("FOOTPRINT_TOLERANCE_M", .03),
    ("BOTTOM_NORMAL_ANGLE_RAD", .1),
    ("CONTACT_NORMAL_ANGLE_RAD", .1),
    ("CONTACT_NORMAL_NORM_TOLERANCE", .1),
    ("MAX_CONTACT_COUNT", 129),
])
def test_object_level_tolerance_replacement_is_rejected_before_support_query(name, value):
    frozen = SupportPolicy(FIXTURE)
    object.__setattr__(frozen, name, value)
    assert frozen.geometry(pose(z=.24), "supply").reasons == ("SUPPORT_POLICY_CONFIGURATION_CHANGED",)
    assert not frozen.accept_contact(record(depth=.02), pose(z=.24), "supply").valid


def test_class_level_tolerance_relaxation_cannot_open_gate(monkeypatch):
    frozen = SupportPolicy(FIXTURE)
    monkeypatch.setattr(SupportPolicy, "PLANE_TOLERANCE_M", float("nan"))
    assert frozen.geometry(pose(), "supply").reasons == ("SUPPORT_POLICY_CONFIGURATION_CHANGED",)


def test_object_level_site_geometry_replacement_cannot_accept_shifted_cube():
    frozen = SupportPolicy(FIXTURE)
    shifted = np.array([.651, 0.])
    shifted.setflags(write=False)
    object.__setattr__(frozen, "site_centers", MappingProxyType({
        "supply": shifted, "target": frozen.site_centers["target"]}))
    assert frozen.geometry(pose(x=.651), "supply").reasons == ("SUPPORT_POLICY_CONFIGURATION_CHANGED",)


def test_reenabled_write_flag_and_mutated_site_fail_closed():
    frozen = SupportPolicy(FIXTURE)
    frozen.site_centers["supply"].setflags(write=True)
    frozen.site_centers["supply"][0] = .651
    frozen.site_centers["supply"].setflags(write=False)
    assert frozen.geometry(pose(x=.651), "supply").reasons == ("SUPPORT_POLICY_CONFIGURATION_CHANGED",)


@pytest.fixture(scope="module")
def gate():
    return CollisionValidator(FIXTURE)


@pytest.mark.parametrize("site,xyz", [("supply", [.65, 0., .26]), ("target", [.82, -.075, .26])])
def test_real_fcl_initial_target_table_contacts_are_independently_checked(policy, gate, site, xyz):
    links = {name: pose(10+2*index, 0, 2) for index, name in enumerate(sorted(gate.required_links))}
    before = copy.deepcopy(gate.allowed_pairs)
    result = gate.validate(links, pose(*xyz), "detached")
    table = [entry for entry in result.collisions if {entry["owner_a"], entry["owner_b"]} == {"Cube_01", "task28_table"}]
    assert table, "Frozen exact-touch FCL receipt changed; record and assess numerical behavior."
    assert all(policy.accept_contact(entry, pose(*xyz), site).valid for entry in table)
    assert gate.allowed_pairs == before


def test_real_fcl_tiny_positive_gap_does_not_require_invented_contact(policy, gate):
    links = {name: pose(10+2*index, 0, 2) for index, name in enumerate(sorted(gate.required_links))}
    obj = pose(z=.2600005)
    result = gate.validate(links, obj, "detached")
    table = [entry for entry in result.collisions if {entry["owner_a"], entry["owner_b"]} == {"Cube_01", "task28_table"}]
    assert not table
    assert policy.geometry(obj, "supply").valid
