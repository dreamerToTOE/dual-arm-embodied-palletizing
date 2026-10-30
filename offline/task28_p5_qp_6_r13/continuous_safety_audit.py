"""Audit the immutable R10 detached approach with outward FK envelopes.

This does not generate IK, execute physics, move Cube, or change frozen gates.
Nominal bilateral closure is recorded as a prospective diagnostic; it is not an
attachment requirement on a detached precontact approach.  Overlapping AABBs
remain unknown, and finite FCL samples never become a continuous certificate.
"""
from __future__ import annotations

from dataclasses import asdict
from fractions import Fraction
import gzip
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from .approach_audit import archived_bytes, bound_bytes, DIRECT_SOURCE, DIRECT_SHA256

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
for name in ("task28_g1_1", "task28_g1_2a", "task28_g1_2b_0", "task28_g1_2b_1"):
    p = str(REPO / "offline" / name)
    if p not in sys.path:
        sys.path.insert(0, p)
from continuous_verifier import (ContinuousVerifier, bernstein, closure_bounds,
                                 matmul, polynomial_range, quaternion_matrix)
from task28_edge.types import Waypoint, Trajectory, Motion, Provenance, Source
from task28_edge.interpolation import span_coefficients
from task28_edge.checker import SampledEdgeValidator
from task28_contact.modes import (OperationMode as M, SupportSite, AttachmentIntent,
                                 ObservedAttachment, EvidenceSource)
from task28_contact.clearance import ConfirmationEvent as E
from task28_validator.closed_chain import evaluate_closure

SOURCE = REPO / "offline/task28_p5_qp_6_r10/evidence/approach_audit.json"
STAGES = ("FREE_HOME", "HIGH_OUTER", "OUTER_CONTACT", "PRE_CONTACT_READY")
VERIFIED = "CONTINUOUSLY_VERIFIED"
UNKNOWN = "CONTINUOUS_VERIFICATION_INCONCLUSIVE"
REJECTED = "CONTINUOUSLY_REJECTED"


def serial(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): serial(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [serial(v) for v in value]
    return value


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_waypoint(q, t, ident, cube):
    """One latched detached request; independent evidence IDs per observation."""
    if not np.array_equal(np.asarray(cube)[:3, :3], np.eye(3)):
        raise ValueError("ARCHIVED_CUBE_ORIENTATION_NOT_IDENTITY")
    request = "R13_OFFLINE_DETACHED_APPROACH"
    evidence = "R13_SYNTHETIC_DETACHED:" + ident
    return Waypoint(
        float(t), tuple(q["left"]), tuple(q["right"]),
        tuple(float(x) for x in np.asarray(cube)[:3, 3]), (0., 0., 0., 1.),
        M.DETACHED_SUPPORTED, SupportSite.SUPPLY, False, False,
        AttachmentIntent(request, -1., False, False),
        ObservedAttachment(evidence, float(t), request, False, False,
                           EvidenceSource.INDEPENDENT_SYNTHETIC),
        Provenance(Source.EXPLICIT_SYNTHETIC, "R13_STATIC_CUBE:" + ident, evidence,
                   "Archived offline detached configuration; no observed attachment or execution."),
        E.OBSERVE, observation_stamp_s=float(t))


def load_trajectory():
    archived = json.loads(archived_bytes())
    cube = np.asarray(archived["cube"]["T_WCube"], dtype=float)
    trajectory = Trajectory(
        "r13_archived_r10_detached_original_cube_approach",
        tuple(make_waypoint(archived["solutions"][n], i, n, cube)
              for i, n in enumerate(STAGES)), Motion.GENERIC,
        scope="Archived R10 joint-linear detached approach; static original Cube; offline only. "
              "Synthetic span seconds specify a path parameter, not actuator timing.")
    return archived, cube, trajectory


def interval_closure_diagnostic(verifier, trajectory, row):
    """Full SE(3) nominal-grasp bounds over each interval, never a detached gate."""
    a, b = trajectory.waypoints[row["span"]:row["span"] + 2]
    if row.get("kind") == "explicit_endpoint":
        a = b = trajectory.waypoints[row["waypoint_index"]]
    power = span_coefficients(a, b, trajectory.interpolation)
    polys = [[bernstein(values) for values in zip(*group)] for group in power]
    lo, hi = map(Fraction, row["u"])
    ql, qr, position = [[polynomial_range(p, lo, hi) for p in group] for group in polys]
    frames = verifier.model.forward(ql, qr)
    obj = quaternion_matrix(a.object_quaternion_xyzw)
    for k in range(3):
        obj[k][3] = position[k]
    return closure_bounds(
        matmul(frames["left_fr3_side_suction_tcp"], verifier.model.grasps[0]),
        matmul(frames["right_fr3_side_suction_tcp"], verifier.model.grasps[1]), obj)


def summarize_span(result, span):
    rows = [r for r in result["intervals"] if r["span"] == span
            and r.get("kind") != "explicit_endpoint"]
    endpoints = [r for r in result["intervals"]
                 if r.get("kind") == "explicit_endpoint"
                 and r["waypoint_index"] in (span, span + 1)]
    limits = [r for r in result["joint_limits"]["bounds"] if r["span"] == span]
    witnesses = [x for x in result["witnesses"] if x["span"] == span]
    joint_rejected = any("CERTIFIED_JOINT_LIMIT_VIOLATION" in w["certified_reasons"]
                         for w in witnesses)
    joint_status = (REJECTED if joint_rejected else VERIFIED
                    if limits and all(x["status"] == VERIFIED for x in limits) else UNKNOWN)
    collision_statuses = [r["collision_status"] for r in rows + endpoints]
    collision_status = (REJECTED if REJECTED in collision_statuses else VERIFIED
                        if collision_statuses and all(s == VERIFIED for s in collision_statuses)
                        else UNKNOWN)
    endpoints_valid = bool(result["endpoint_validity"])
    classification = ("CONFIRMED_UNSAFE" if REJECTED in (joint_status, collision_status) else
                      "PROVED_SAFE" if joint_status == collision_status == VERIFIED
                      and endpoints_valid else "UNABLE_TO_PROVE_SAFE")
    all_proved = set(range(len(result["pair_catalog"])))
    unresolved = set()
    for row in rows:
        all_proved &= {p["pair"] for p in row["pair_separation_proofs"]}
        unresolved |= set(row["unresolved_pairs"])
    return {
        "span": span, "name": STAGES[span] + "_TO_" + STAGES[span + 1],
        "classification": classification,
        "joint_limits": joint_status,
        "frozen_endpoints_valid": endpoints_valid,
        "closed_chain_requirement": "NOT_APPLICABLE_DETACHED_PRECONTACT",
        "continuous_collision": collision_status,
        "leaf_interval_count": len(rows),
        "pair_count": len(result["pair_catalog"]),
        "pair_separation_proved_over_whole_span": sorted(all_proved),
        "unresolved_pair_union": sorted(unresolved),
        "proved_pairs_count": len(all_proved), "unresolved_pairs_count": len(unresolved),
        "certified_violation_witnesses": witnesses,
        "reason": "Conservative articulated FK AABBs overlap for non-ACM pairs; overlap is not a collision. "
                  "Frozen verifier also leaves support contact rules unproved. No full-span certificate.",
    }


def write_gzip(path, data):
    payload = json.dumps(serial(data), sort_keys=True, separators=(",", ":")).encode() + b"\n"
    path.write_bytes(gzip.compress(payload, mtime=0))
    return {"path": str(path.relative_to(REPO)), "sha256": digest(path),
            "uncompressed_bytes": len(payload)}


def main():
    archived, cube, trajectory = load_trajectory()
    checker = SampledEdgeValidator()
    sampled = checker.validate(trajectory)
    sampled_data = asdict(sampled)
    # Full left/object, right/object and left/right SE(3) diagnostics at every sample.
    closure_rows = []
    for row in sampled.samples:
        tcps = row["world_tcp_poses"]
        full = asdict(evaluate_closure(cube, tcps["left"], tcps["right"]))
        closure_rows.append({"time_s": row["time_s"], "span": row["span_index"],
                             "u": row["u"], "required": False, "diagnostic": full})
    sample_artifact = write_gzip(HERE / "evidence/continuous_sampled_raw.json.gz",
                                {"sampled": sampled_data, "prospective_closure_diagnostics": closure_rows})
    depth_rows, artifacts, deepest = [], [], None
    for depth in (0, 1, 2, 4):
        verifier = ContinuousVerifier(checker, max_depth=depth)
        result = verifier.verify(trajectory)
        for row in result["intervals"]:
            row["prospective_nominal_grasp_closure_diagnostic_only"] = interval_closure_diagnostic(verifier, trajectory, row)
        artifacts.append(write_gzip(HERE / f"evidence/continuous_intervals_depth_{depth}.json.gz", result))
        depth_rows.append({
            "max_depth": depth, "combined_status": result["combined_status"],
            "joint_limits": result["joint_limits"]["status"],
            "closed_chain": "NOT_APPLICABLE_DETACHED_PRECONTACT",
            "frozen_verifier_closed_chain_status": result["closed_chain"]["status"],
            "collision": result["collision"]["status"],
            "sampled_edge_status": sampled.status.value,
            "sampled_count": sampled.evaluated_sample_count,
            "sampled_reasons": list(sampled.reasons),
            "unresolved_interval_count": sum(r["collision_status"] == UNKNOWN for r in result["intervals"]),
            "witness_count": len(result["witnesses"]), "max_depth_observed": result["maximum_subdivision_depth"],
        })
        deepest = result
    fixed_pose = all(w.object_position_m == trajectory.waypoints[0].object_position_m
                     and w.object_quaternion_xyzw == trajectory.waypoints[0].object_quaternion_xyzw
                     for w in trajectory.waypoints)
    support_rows = [row["collision"] for row in sampled.samples]
    support_valid = fixed_pose and all(row["support"] is not None and row["support"]["valid"]
                                      and not row["remaining_collisions"] for row in support_rows)
    # At a constant Cube/table transform, pair contact geometry is invariant to arm q.
    # This is a local support component only, never an arm collision certificate.
    support = {
        "cube_transform_constant_by_interpolation": fixed_pose,
        "geometry_policy_valid_all_samples": all(r["support"] is not None and r["support"]["valid"] for r in support_rows),
        "static_support_geometry_and_record_policy": "VERIFIED_OFFLINE_CONSTANT_PAIR" if support_valid else "UNRESOLVED",
        "scope": "Cube/table geometry and reported contacts only; no physical support, forces or stability claim; "
                 "no ACM removal and no improvement of the frozen verifier's full-collision status.",
        "first_point_phase_collision_evidence": support_rows[0],
        "accepted_support_record_count_per_sample": [len(r["accepted_support_contacts"]) for r in support_rows],
    }
    closure_keys = ("translation_error_m", "rotation_error_rad", "left_object_translation_error_m",
                    "left_object_rotation_error_rad", "right_object_translation_error_m",
                    "right_object_rotation_error_rad")
    closure_summary = {
        "requirement": "NOT_APPLICABLE_DETACHED_PRECONTACT", "sample_count": len(closure_rows),
        "prospective_frozen_g1_tolerances": {"translation_m": 0.0001, "rotation_rad": 0.001},
        "sampled_maxima": {k: max(r["diagnostic"][k] for r in closure_rows) for k in closure_keys},
        "final_precontact_sample": closure_rows[-1],
        "continuous_diagnostic_bounds": "Raw intervals include full nominal-grasp SE(3) bounds; they are prospective "
                                        "diagnostics, not false closure violations or grasp evidence.",
        "attached_lift_or_transport_continuous_closure": "NOT_ESTABLISHED_OUTSIDE_THIS_PATH",
    }
    direct_source = DIRECT_SOURCE
    direct = json.loads(bound_bytes(direct_source, DIRECT_SHA256))
    spans = [summarize_span(deepest, i) for i in range(3)]
    out = {
        "schema": "TASK28_R13_ORIGINAL_CUBE_CONTINUOUS_AUDIT_V2",
        "status": "CONTINUOUS_SAFETY_NOT_ESTABLISHED",
        "reason": "All three archived staged spans lack complete continuous collision certificates; "
                  "sampled clearance and partial pair envelopes do not establish whole-path safety.",
        "trajectory": trajectory.identity, "original_cube_identity": "PRE_CONTACT_ORIGINAL_CUBE",
        "semantics": {"mode": M.DETACHED_SUPPORTED.value, "site": SupportSite.SUPPLY.value,
                      "left_attached": False, "right_attached": False,
                      "one_latched_intent": True, "synthesized_feedback_not_measurement": True,
                      "cube_pose_source": "R10 archived approach JSON, unchanged", "no_ik_resolve": True},
        "invalidation": {
            "artifact": "offline/task28_p5_qp_6_r13/evidence/invalidation/continuous_safety_audit_v1.json",
            "reason": "V1 falsely assigned BILATERAL_AIRBORNE to a detached supported approach and changed intent IDs "
                      "without attachment events. Its closure, airborne-clearance and attachment rejection reasons "
                      "and three closure witnesses are invalid for this path, and are withdrawn.",
            "original_r10_v1_same_semantic_defect": True, "historical_artifacts_modified": False},
        "source_hashes": {str(p.relative_to(REPO)): digest(p) for p in (
            SOURCE, HERE / "evidence/approach_audit.json", direct_source,
            HERE / "approach_audit.py", HERE / "evidence/approach_archive_binding.json",
            HERE / "continuous_safety_audit.py",
            HERE / "evidence/invalidation/continuous_safety_audit_v1.json",
            HERE / "evidence/invalidation/continuous_safety_audit_v1.py.txt",
            checker.model.fixture_dir / "fixture_manifest.json",
            REPO / "offline/task28_g1_1/task28_validator/fixture_binding.json",
            REPO / "offline/task28_g1_2b_1/continuous_verifier.py",
            REPO / "offline/task28_g1_2b_1/interval_math.py",
            REPO / "offline/task28_g1_2a/task28_contact/support.py",
            REPO / "offline/task28_g1_2a/task28_contact/collision.py",
            REPO / "offline/task28_g1_2b_0/task28_edge/checker.py")},
        "r13_archived_approach_byte_identical_to_r10": digest(SOURCE) == digest(HERE / "evidence/approach_audit.json"),
        "finite_sample_source": str(SOURCE.relative_to(REPO)), "depth_runs": depth_rows,
        "segments": spans,
        "proved_safe_segments": [s["name"] for s in spans if s["classification"] == "PROVED_SAFE"],
        "unable_to_prove_safe_segments": [s["name"] for s in spans if s["classification"] == "UNABLE_TO_PROVE_SAFE"],
        "confirmed_unsafe_staged_segments": [s["name"] for s in spans if s["classification"] == "CONFIRMED_UNSAFE"],
        "known_unsafe_separate_direct_path": {
            "classification": "CONFIRMED_UNSAFE_ARCHIVED_POINT_WITNESS",
            "source": str(direct_source.relative_to(REPO)),
            "witness": direct["straight_joint_interpolation"]["first_unexpected_collision"],
            "scope": "R3 direct interpolation has its own saved pose/IK. This is not a witness on the different R10 staged path.",
            "new_execution_or_reconstruction": False},
        "static_cube_table_support": support, "closed_chain": closure_summary,
        "raw_sample_evidence": sample_artifact, "raw_interval_evidence": artifacts,
        "proof_scope": "Frozen URDF/STL mathematical joint-linear FK path with outward dyadic 80-bit bounds, "
                       "414 checked non-ACM pairs and 40 frozen ACM exclusions; no dynamics/tracking envelope.",
        "execution": {"status": "BLOCKED", "suction": 0, "cube_commands": 0, "ros": 0,
                      "physics_runs": 0, "real_robot": 0, "D6": 0},
        "not_a_certificate": True,
    }
    (HERE / "evidence/continuous_safety_audit.json").write_text(json.dumps(serial(out), indent=2, sort_keys=True) + "\n")
    print(json.dumps({"status": out["status"], "sampled_status": sampled.status.value,
                      "segments": [{k: s[k] for k in ("name", "classification", "proved_pairs_count", "unresolved_pairs_count")} for s in spans]}))


if __name__ == "__main__":
    main()
