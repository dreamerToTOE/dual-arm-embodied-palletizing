"""Regress the detached approach semantics and conservative proof boundary."""
import copy
from dataclasses import replace
from fractions import Fraction
import gzip
import hashlib
import json

import pytest

from offline.task28_p5_qp_6_r13.continuous_safety_audit import (
    HERE, REPO, load_trajectory, SampledEdgeValidator, M, SupportSite,
    AttachmentIntent, summarize_span, VERIFIED, UNKNOWN, REJECTED,
)
from offline.task28_p5_qp_6_r13.approach_audit import (
    archived_bytes, bound_bytes, SOURCE_SHA256,
)


@pytest.fixture(scope="module")
def audit():
    return json.loads((HERE / "evidence/continuous_safety_audit.json").read_text())


@pytest.fixture(scope="module")
def checker():
    return SampledEdgeValidator()


def test_detached_supported_metadata_has_one_latched_request(checker):
    _, _, trajectory = load_trajectory()
    assert all(w.mode is M.DETACHED_SUPPORTED and w.support_site is SupportSite.SUPPLY
               and not w.left_attached and not w.right_attached for w in trajectory.waypoints)
    assert len({w.intent for w in trajectory.waypoints}) == 1
    assert len({w.observed.evidence_id for w in trajectory.waypoints}) == 4
    assert checker.mode_schedule(trajectory) == []
    w = trajectory.waypoints[1]
    changed = replace(w, intent=AttachmentIntent("unannounced-new-request", -1., False, False))
    bad = replace(trajectory, waypoints=(trajectory.waypoints[0], changed, *trajectory.waypoints[2:]))
    assert any(reason == "ATTACHMENT_CHANGED_WITHOUT_EXPLICIT_EVENT"
               for _, reason in checker.mode_schedule(bad))


def test_reuses_exact_archived_path_and_invalidates_false_closure(audit):
    assert audit["r13_archived_approach_byte_identical_to_r10"]
    assert audit["semantics"]["no_ik_resolve"]
    for path, expected in audit["source_hashes"].items():
        assert hashlib.sha256((REPO / path).read_bytes()).hexdigest() == expected
    old = json.loads((REPO / audit["invalidation"]["artifact"]).read_text())
    assert all(row["closed_chain"] == "CONTINUOUSLY_REJECTED" for row in old["depth_runs"])
    for row in audit["depth_runs"]:
        assert row["closed_chain"] == "NOT_APPLICABLE_DETACHED_PRECONTACT"
        assert row["sampled_edge_status"] == "sampled_edge_valid"
        assert row["sampled_reasons"] == []
        assert row["witness_count"] == 0


def test_archive_binding_rejects_changed_path_bytes(tmp_path):
    raw = archived_bytes()
    assert hashlib.sha256(raw).hexdigest() == SOURCE_SHA256
    altered = tmp_path / "approach.json"
    altered.write_bytes(raw + b" ")
    with pytest.raises(ValueError, match="ARCHIVED_APPROACH_SOURCE_HASH_MISMATCH"):
        bound_bytes(altered, SOURCE_SHA256)


def test_raw_interval_enclosures_cover_every_span_and_pair(audit):
    for artifact in audit["raw_interval_evidence"]:
        raw = (REPO / artifact["path"]).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == artifact["sha256"]
        result = json.loads(gzip.decompress(raw))
        assert len(result["pair_catalog"]) == 414
        for span in range(3):
            rows = [r for r in result["intervals"]
                    if r["span"] == span and r.get("kind") != "explicit_endpoint"]
            cursor = Fraction(0)
            for row in rows:
                lo, hi = map(Fraction, row["u"])
                assert lo == cursor and hi > lo
                cursor = hi
                proofs = {p["pair"] for p in row["pair_separation_proofs"]}
                unresolved = set(row["unresolved_pairs"])
                assert not proofs & unresolved
                assert proofs | unresolved == set(range(414))
                assert all(p["gap_lower_scaled"] > 0 for p in row["pair_separation_proofs"])
                diagnostics = row["prospective_nominal_grasp_closure_diagnostic_only"]
                assert {r["pair"] for r in diagnostics} == {"left_right", "left_object", "right_object"}
            assert cursor == 1


def test_unknown_intervals_not_promoted_to_safe_or_collision(audit):
    assert audit["status"] == "CONTINUOUS_SAFETY_NOT_ESTABLISHED"
    assert audit["proved_safe_segments"] == []
    assert audit["confirmed_unsafe_staged_segments"] == []
    assert len(audit["unable_to_prove_safe_segments"]) == 3
    for span in audit["segments"]:
        assert span["classification"] == "UNABLE_TO_PROVE_SAFE"
        assert span["unresolved_pairs_count"] > 1
        assert span["proved_pairs_count"] + span["unresolved_pairs_count"] == 414
        assert span["certified_violation_witnesses"] == []


@pytest.mark.parametrize("collision,endpoints_valid,joint_rejected,expected", [
    (UNKNOWN, True, False, "UNABLE_TO_PROVE_SAFE"),
    (REJECTED, True, False, "CONFIRMED_UNSAFE"),
    (VERIFIED, True, False, "PROVED_SAFE"),
    (VERIFIED, False, False, "UNABLE_TO_PROVE_SAFE"),
    (VERIFIED, True, True, "CONFIRMED_UNSAFE"),
])
def test_span_classification_is_derived_from_proof_outcomes(
        collision, endpoints_valid, joint_rejected, expected):
    # Synthetic proof outcomes exercise classification only; they are never
    # written as evidence about the archived approach.
    raw = {"intervals": [{"span": 0, "collision_status": collision,
                           "pair_separation_proofs": ([{"pair": 0}] if collision == VERIFIED else []),
                           "unresolved_pairs": ([] if collision == VERIFIED else [0])},
                          *[{"span": 0, "kind": "explicit_endpoint", "waypoint_index": i,
                             "collision_status": collision} for i in (0, 1)]],
           "joint_limits": {"bounds": [{"span": 0, "status": VERIFIED}]},
           "pair_catalog": [(0, 1)], "endpoint_validity": endpoints_valid,
           "witnesses": ([{"span": 0, "certified_reasons": ["CERTIFIED_JOINT_LIMIT_VIOLATION"]}]
                         if joint_rejected else [])}
    assert summarize_span(raw, 0)["classification"] == expected


def test_supported_cube_is_local_policy_not_broad_collision_exemption(audit, checker):
    _, cube, _ = load_trajectory()
    support = checker.contact.collision.support
    assert support.geometry(cube, "supply").valid
    moved = cube.copy()
    moved[2, 3] += 0.001
    assert not support.geometry(moved, "supply").valid
    assert "SUPPORT_BOTTOM_NOT_ON_TABLE_PLANE" in support.geometry(moved, "supply").reasons
    first = audit["static_cube_table_support"]["first_point_phase_collision_evidence"]
    assert first["valid"] and first["remaining_collisions"] == []
    assert first["support"]["physical_support_observed"] is False
    for accepted in first["accepted_support_contacts"]:
        record = accepted["record"]
        assert support.accept_contact(record, cube, "supply").valid
        wrong = copy.deepcopy(record)
        wrong["contacts"][0]["normal_world"] = [-x for x in wrong["contacts"][0]["normal_world"]]
        assert not support.accept_contact(wrong, cube, "supply").valid


def test_closure_diagnostics_remain_available_without_implying_attachment(audit):
    assert audit["closed_chain"]["requirement"] == "NOT_APPLICABLE_DETACHED_PRECONTACT"
    assert audit["closed_chain"]["sample_count"] == 159
    final = audit["closed_chain"]["final_precontact_sample"]
    assert final["required"] is False
    assert final["diagnostic"]["valid"]
    assert audit["closed_chain"]["attached_lift_or_transport_continuous_closure"] == "NOT_ESTABLISHED_OUTSIDE_THIS_PATH"
    artifact = audit["raw_sample_evidence"]
    raw = (REPO / artifact["path"]).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == artifact["sha256"]
    data = json.loads(gzip.decompress(raw))
    assert len(data["prospective_closure_diagnostics"]) == 159
    assert all(not row["required"] for row in data["prospective_closure_diagnostics"])


def test_archived_direct_path_witness_does_not_relabel_staged_path(audit):
    direct = audit["known_unsafe_separate_direct_path"]
    assert direct["classification"] == "CONFIRMED_UNSAFE_ARCHIVED_POINT_WITNESS"
    assert direct["witness"]["fraction"] == 0.92
    assert direct["source"].startswith("offline/task28_p5_qp_6_r3/")
    assert not direct["new_execution_or_reconstruction"]
    assert all(audit["execution"][k] == 0 for k in ("suction", "cube_commands", "ros", "physics_runs", "real_robot", "D6"))
