"""Independent geometric controls: real solid FCL, no simulator or ROS."""
import copy
from pathlib import Path

import numpy as np
import pytest

from task28_validator.collision import CollisionValidator


FIXTURE = Path(__file__).resolve().parents[3] / "fixtures/task28_g1_0_new_single_cube_v0"


@pytest.fixture(scope="module")
def gate():
    return CollisionValidator(FIXTURE)


def pose(x=0., y=0., z=0.):
    result = np.eye(4)
    result[:3, 3] = [x, y, z]
    return result


def separated_links(gate):
    # Independent arbitrary poses isolate collision-backend behavior from FK/IK.
    return {name: pose(10 + 2*i, 0, 2) for i, name in enumerate(sorted(gate.required_links))}


def bilateral_tools(gate, gap=0.):
    links = separated_links(gate)
    for side, sign in (("left", -1), ("right", 1)):
        tool = pose(.65, sign*(.215+gap), .48)
        tool[:3, :3] = np.diag([1., -1., -1.])
        links[f"{side}_fr3_side_suction"] = tool
    return links


def payload_pairs(result):
    return [record for record in result.collisions
            if "Cube_01" in (record["owner_a"], record["owner_b"])]


def test_all_frozen_geometry_and_unexpanded_acm_present(gate):
    assert len(gate.required_links) == 18  # link0..7 and side tool per arm.
    assert len(gate.shapes) == 30  # 16 solid hulls + 2*(3 boxes + 4 cylinders).
    assert len(gate.allowed_pairs) == 48
    assert all(not any("Cube" in name or "table" in name for name in pair)
               for pair in gate.allowed_pairs)
    assert frozenset(("left_fr3_link7", "right_fr3_link7")) not in gate.allowed_pairs


def test_nominal_one_mm_gap_is_valid_without_contact_waiver(gate):
    result = gate.validate(bilateral_tools(gate, .001), pose(.65, 0, .4))
    assert result.valid, result.reasons
    assert not result.accepted_attachment_contacts
    assert result.checked_shape_pairs == 414
    assert result.skipped_acm_shape_pairs == 40


def test_zero_depth_front_caps_use_only_local_contact_policy(gate):
    links, obj = bilateral_tools(gate), pose(.65, 0, .4)
    original_acm = copy.deepcopy(gate.allowed_pairs)
    result = gate.validate(links, obj, "bilateral")
    assert result.valid, result.reasons
    # FCL reports 4 left contacts here; the opposite exact touch can round to a
    # positive separation. Every reported contact must pass the bounded policy.
    assert len(result.accepted_attachment_contacts) >= 1
    assert all("side_suction/" in record["shape_a"] and record["shape_b"] == "Cube_01/box"
               for record in result.accepted_attachment_contacts)
    assert max(abs(contact["depth_m"]) for record in result.accepted_attachment_contacts
               for contact in record["contacts"]) < 1e-6
    assert gate.allowed_pairs == original_acm
    detached = gate.validate(links, obj, "detached")
    assert not detached.valid and payload_pairs(detached)
    assert not detached.accepted_attachment_contacts


def test_single_tool_contact_does_not_authorize_bilateral_patch(gate):
    links = bilateral_tools(gate)
    links["right_fr3_side_suction"] = pose(70, 0, 2)
    result = gate.validate(links, pose(.65, 0, .4), "bilateral")
    assert not result.valid and payload_pairs(result)
    assert not result.accepted_attachment_contacts


def test_laterally_shifted_cup_patch_is_rejected(gate):
    links = bilateral_tools(gate)
    links["left_fr3_side_suction"][0, 3] += .005
    result = gate.validate(links, pose(.65, 0, .4), "bilateral")
    assert not result.valid and payload_pairs(result)
    assert not result.accepted_attachment_contacts


def test_deep_cup_intrusion_is_rejected(gate):
    links = bilateral_tools(gate)
    links["left_fr3_side_suction"][1, 3] += .002
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid and payload_pairs(result)
    assert not result.accepted_attachment_contacts
    assert max(contact["depth_m"] for record in payload_pairs(result)
               for contact in record["contacts"]) > .001


def test_tool_manifold_collision_cannot_be_hidden_by_attachment(gate):
    links = bilateral_tools(gate)
    # Translate the broad manifold centre to the Cube centre: not a cup patch.
    links["left_fr3_side_suction"][1, 3] = -.138
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid
    assert any(record["shape_a"] == "left_fr3_side_suction/2"
               for record in payload_pairs(result))


def test_wrong_face_cup_contact_rejected(gate):
    # Rz(90deg) rotates the entire contact arrangement to world-X faces while the
    # Cube stays identity. These geometrically touching caps are not designated.
    links = bilateral_tools(gate)
    center = np.array([.65, 0., .4])
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    for side in ("left", "right"):
        matrix = links[f"{side}_fr3_side_suction"]
        matrix[:3, 3] = center + rotation @ (matrix[:3, 3] - center)
        matrix[:3, :3] = rotation @ matrix[:3, :3]
    result = gate.validate(links, pose(*center))
    assert not result.valid and payload_pairs(result)
    assert not result.accepted_attachment_contacts


def test_rotated_shared_cube_rotates_contact_faces(gate):
    links = bilateral_tools(gate)
    center = np.array([.65, 0., .4])
    angle = .37
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0.],
                         [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    for side in ("left", "right"):
        matrix = links[f"{side}_fr3_side_suction"]
        matrix[:3, 3] = center + rotation @ (matrix[:3, 3] - center)
        matrix[:3, :3] = rotation @ matrix[:3, :3]
    obj = pose(*center)
    obj[:3, :3] = rotation
    result = gate.validate(links, obj)
    assert result.valid, result.reasons


def test_payload_table_penetration_rejected(gate):
    result = gate.validate(separated_links(gate), pose(.65, 0, .24))
    assert not result.valid
    table = [record for record in payload_pairs(result)
             if "task28_table" in (record["owner_a"], record["owner_b"])]
    assert table
    assert max(contact["depth_m"] for record in table for contact in record["contacts"]) == pytest.approx(.02)


def test_payload_completely_inside_table_not_surface_only(gate):
    result = gate.validate(separated_links(gate), pose(.65, 0, .1))
    assert not result.valid and payload_pairs(result)
    assert max(contact["depth_m"] for record in payload_pairs(result)
               for contact in record["contacts"]) > .1


def test_payload_arm_collision_is_included(gate):
    links = separated_links(gate)
    # Place solid link0 hull through the independently specified Cube centre.
    links["left_fr3_link0"] = pose(.65, 0, .35)
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid
    assert any(record["owner_a"] == "left_fr3_link0" for record in payload_pairs(result))


def test_arm_arm_collision_is_included(gate):
    links = separated_links(gate)
    links["left_fr3_link7"] = links["right_fr3_link7"] = pose(5, 0, 2)
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid
    assert any({record["owner_a"], record["owner_b"]} == {"left_fr3_link7", "right_fr3_link7"}
               for record in result.collisions)


def test_nonallowed_self_collision_is_included(gate):
    links = separated_links(gate)
    links["left_fr3_link0"] = links["left_fr3_link7"] = pose(5, 0, 2)
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid
    assert any({record["owner_a"], record["owner_b"]} == {"left_fr3_link0", "left_fr3_link7"}
               for record in result.collisions)


def test_robot_table_collision_is_included(gate):
    links = separated_links(gate)
    links["right_fr3_link0"] = pose(.65, 0, .05)
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid
    assert any({record["owner_a"], record["owner_b"]} == {"right_fr3_link0", "task28_table"}
               for record in result.collisions)


def test_missing_robot_geometry_transform_rejected(gate):
    links = separated_links(gate)
    del links["left_fr3_link0"]
    result = gate.validate(links, pose(.65, 0, .4))
    assert not result.valid and result.reasons[0].startswith("MISSING_COLLISION_LINKS:")


def test_invalid_transform_and_mode_rejected(gate):
    obj = pose(.65, 0, .4)
    obj[0, 0] = 2
    assert not gate.validate(separated_links(gate), obj).valid
    mode = gate.validate(separated_links(gate), pose(.65, 0, .4), "left_only")
    assert not mode.valid and mode.reasons == ("INVALID_ATTACHMENT_MODE",)


def test_backend_failure_rejects_without_fallback(gate, tmp_path):
    original = gate.backend_path
    try:
        gate.backend_path = tmp_path / "missing_backend"
        result = gate.validate(separated_links(gate), pose(.65, 0, .4))
        assert not result.valid and result.reasons[0].startswith("COLLISION_BACKEND_FAILURE:")
    finally:
        gate.backend_path = original


def test_complex_and_boolean_transforms_rejected(gate):
    links = separated_links(gate)
    for matrix in (np.eye(4,dtype=complex),np.eye(4,dtype=bool)):
        assert not gate.validate(links,matrix).valid
    links['left_fr3_link0'] = links['left_fr3_link0'].astype(complex)
    assert not gate.validate(links,pose(.65,0,.4)).valid


@pytest.mark.parametrize('mutate',[
    lambda output: output.pop('checked_pair_count'),
    lambda output: output.__setitem__('checked_pair_count',413),
    lambda output: output.__setitem__('shape_count',31),
    lambda output: output.__setitem__('pairs',{}),
    lambda output: output.__setitem__('pairs',[{'a':0,'b':1,'contacts':[]}]),
    lambda output: output.__setitem__('pairs',[{'a':0,'b':1,'contacts':[{'depth_m':float('nan'),'normal_world':[0,1,0],'position_world_m':[0,0,0]}]}]),
])
def test_incomplete_or_malformed_collision_receipt_fails_closed(gate,monkeypatch,mutate):
    import json
    import types
    output={'backend':'FCL-0.7.0','shape_count':32,'checked_pair_count':414,'pairs':[]}
    mutate(output)
    monkeypatch.setattr('task28_validator.collision.subprocess.run',lambda *args,**kwargs:types.SimpleNamespace(stdout=json.dumps(output)))
    result=gate.validate(separated_links(gate),pose(.65,0,.4))
    assert not result.valid
    assert result.reasons[0].startswith('COLLISION_BACKEND_FAILURE:')
