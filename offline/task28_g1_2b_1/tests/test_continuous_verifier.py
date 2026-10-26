"""Actual frozen-model integration, independent of the new interval formulas."""
from dataclasses import asdict, replace

import pytest
from scipy.spatial.transform import Rotation

from adversarial_controls import (WITNESS_U, adversarial_controls,
                                  independent_object_position)
from continuous_verifier import (ContinuousVerifier, INCONCLUSIVE, REJECTED,
                                 VERIFIED)
from task28_edge.checker import SampledEdgeValidator, contract
from task28_edge.experiments import controls
from task28_edge.interpolation import reconstruct
from task28_edge.types import (EdgeStatus, Interpolation, SamplingPolicy,
                               TranslationInterpolation)


@pytest.fixture(scope="module")
def checker():
    return SampledEdgeValidator()


@pytest.fixture(scope="module")
def verifier(checker):
    return ContinuousVerifier(checker, max_depth=2)


@pytest.fixture(scope="module")
def cases():
    return adversarial_controls()


@pytest.fixture(scope="module")
def adversarial_results(checker, verifier, cases):
    output = {}
    for name, control in cases.items():
        ordinary = checker.validate(control.trajectory)
        stronger = verifier.verify(control.trajectory, asdict(ordinary))
        output[name] = ordinary, stronger
    return output


@pytest.mark.parametrize("name,component,reason", (
    ("offgrid_closure_excursion", "closed_chain", "CERTIFIED_CLOSURE_THRESHOLD_VIOLATION"),
    ("offgrid_support_penetration", "collision", "CERTIFIED_CUBE_TABLE_PENETRATION"),
))
def test_real_offgrid_failures_evade_old_grid_and_are_certified_rejected(
        checker, cases, adversarial_results, name, component, reason):
    control = cases[name]
    trajectory = control.trajectory
    ordinary, stronger = adversarial_results[name]
    assert ordinary.endpoints_valid
    assert ordinary.status is EdgeStatus.SAMPLED_EDGE_VALID
    assert ordinary.planned_sample_count == ordinary.evaluated_sample_count == 3
    assert tuple(row["u"] for row in ordinary.samples) == (0.0, 0.5, 1.0)
    assert all(row["valid"] for row in ordinary.samples)
    assert stronger["sampled_evidence_used_as_proof"] is False
    assert stronger["joint_limits"]["status"] == VERIFIED
    assert stronger[component]["status"] == REJECTED
    assert stronger["combined_status"] == REJECTED
    matching = [w for w in stronger["witnesses"] if reason in w["certified_reasons"]]
    assert matching
    witness = min(matching, key=lambda w: abs(w["u"] - WITNESS_U))
    assert witness["u"] == pytest.approx(WITNESS_U, abs=1e-14)
    assert all(abs(witness["u"] - row["u"]) > 1e-6 for row in ordinary.samples)
    first, last = trajectory.waypoints[witness["span"]:witness["span"]+2]
    point = reconstruct(first, last, witness["u"], trajectory.interpolation,
                        trajectory.motion, checker.contact.collision.support,
                        trajectory.identity)
    # The recorded candidate must be an actual point on the typed old path,
    # and its position must match the separate analytic polynomial oracle.
    assert point.object_position_m == pytest.approx(
        independent_object_position(control, witness["u"]), abs=2e-16)
    assert point.object_position_m == pytest.approx(witness["object_position_m"], abs=2e-16)
    replay = checker.evaluate(point)
    assert not replay["valid"] and not witness["frozen_point_gate"]["valid"]
    assert set(control.expected_witness_reasons).issubset(replay["reasons"])
    assert set(control.expected_witness_reasons).issubset(witness["frozen_point_gate"]["reasons"])
    assert replay["collision"]["reference"]["checked_shape_pairs"] == 414
    assert len(replay["independent_fk"]) == 20


def test_positive_frozen_hold_proves_joint_ranges_and_full_closure(checker, verifier):
    trajectory = controls(checker.model)["positive_hold"]
    ordinary = checker.validate(trajectory)
    assert ordinary.endpoints_valid and ordinary.status is EdgeStatus.SAMPLED_EDGE_VALID
    stronger = verifier.verify(trajectory, asdict(ordinary))
    assert stronger["joint_limits"]["status"] == VERIFIED
    assert stronger["closed_chain"]["status"] == VERIFIED
    assert stronger["collision"]["checked_pairs"] == 414
    assert stronger["collision"]["skipped_acm_pairs"] == 40
    assert stronger["witnesses"] == []
    assert stronger["sampled_evidence_used_as_proof"] is False
    assert stronger["intervals"]
    for interval in stronger["intervals"]:
        assert interval["closure_status"] == VERIFIED
        assert len(interval["closure_bounds"]) == 3


def test_changing_object_rotation_fails_closed_without_false_closure_certificate(checker, verifier):
    trajectory = controls(checker.model)["positive_hold"]
    first, last = trajectory.waypoints
    last = replace(last, object_quaternion_xyzw=tuple(map(
        float, Rotation.from_rotvec((0.0, 0.0, 1e-4)).as_quat())))
    trajectory = replace(trajectory, identity="small_changing_object_rotation",
                         waypoints=(first, last))
    ordinary = checker.validate(trajectory)
    assert ordinary.endpoints_valid and ordinary.status is EdgeStatus.SAMPLED_EDGE_VALID
    stronger = verifier.verify(trajectory, asdict(ordinary))
    assert stronger["closed_chain"]["status"] == INCONCLUSIVE
    assert stronger["combined_status"] == INCONCLUSIVE
    path_intervals = [row for row in stronger["intervals"]
                      if row.get("kind") != "explicit_endpoint"]
    assert path_intervals
    assert all(not row["object_rotation_supported"] for row in path_intervals)


def test_equivalent_quaternion_sign_preserves_constant_rotation_scope(checker, verifier):
    trajectory = controls(checker.model)["positive_hold"]
    first, last = trajectory.waypoints
    last = replace(last, object_quaternion_xyzw=tuple(-x for x in last.object_quaternion_xyzw))
    trajectory = replace(trajectory, identity="same_rotation_opposite_quaternion_sign",
                         waypoints=(first, last))
    stronger = verifier.verify(trajectory)
    assert stronger["closed_chain"]["status"] == VERIFIED
    assert all(row["object_rotation_supported"] for row in stronger["intervals"])


@pytest.mark.parametrize("invalid", (None, object(), {"identity": "untyped_dictionary"}))
def test_untyped_inputs_return_inconclusive_not_a_certificate(verifier, invalid):
    result = verifier.verify(invalid, {"status": "sampled_edge_valid", "endpoints_valid": True})
    assert result["combined_status"] == INCONCLUSIVE
    assert result["reasons"] == ["INVALID_TRAJECTORY_TYPE"]
    assert result["witnesses"] == []
    assert all(result[key]["status"] == INCONCLUSIVE
               for key in ("joint_limits", "closed_chain", "collision"))


def test_invalid_numeric_and_unsupported_interpolation_fail_closed(checker, verifier):
    original = controls(checker.model)["positive_hold"]
    first, last = original.waypoints
    invalid_numeric = replace(original, waypoints=(replace(first, left_q=(float("nan"), *first.left_q[1:])), last))
    unsupported = replace(original, interpolation=Interpolation(rotation="unsupported_rotation"))
    for trajectory in (invalid_numeric, unsupported):
        result = verifier.verify(trajectory)
        assert result["combined_status"] == INCONCLUSIVE
        assert all(result[key]["status"] == INCONCLUSIVE
                   for key in ("joint_limits", "closed_chain", "collision"))
        assert result["reasons"] and not result["witnesses"]


def test_finite_but_overflowing_hermite_tangents_return_inconclusive(checker, verifier):
    original = controls(checker.model)["positive_hold"]
    first, last = original.waypoints
    first = replace(first, object_velocity_m_s=(1e308, 0.0, 0.0))
    last = replace(last, time_s=1.0,
                   observed=replace(last.observed, stamp_s=1.0),
                   object_velocity_m_s=(1e308, 0.0, 0.0))
    trajectory = replace(original, identity="finite_tangents_overflow_coefficients",
                         waypoints=(first, last), interpolation=Interpolation(
                             translation=TranslationInterpolation.CUBIC_HERMITE))
    # Every supplied scalar is finite and the old typed contract is satisfied;
    # coefficient overflow still cannot yield a certificate or escape as error.
    assert contract(trajectory, SamplingPolicy()) == ()
    result = verifier.verify(trajectory)
    assert result["combined_status"] == INCONCLUSIVE
    assert all(result[key]["status"] == INCONCLUSIVE
               for key in ("joint_limits", "closed_chain", "collision"))
    assert any("PROOF_INPUT_OR_ARITHMETIC_UNSUPPORTED" in reason
               for reason in result["reasons"])
    assert result["witnesses"] == []


def test_external_pass_evidence_cannot_replace_authoritative_mode_gate(checker, verifier):
    trajectory = controls(checker.model)["D_mode"]
    ordinary = checker.validate(trajectory)
    assert ordinary.endpoints_valid and all(row["valid"] for row in ordinary.samples)
    assert ordinary.status is EdgeStatus.SAMPLED_EDGE_REJECTED
    assert "ILLEGAL_INTERIOR_CONTACT_MODE_TRANSITION" in ordinary.reasons
    spoofed = {"status": "sampled_edge_valid", "endpoints_valid": True}
    result = verifier.verify(trajectory, spoofed)
    assert result["endpoint_validity"] is True
    assert result["sampled_edge_validity"] == "sampled_edge_rejected"
    assert result["combined_status"] != VERIFIED
    assert result["sampled_evidence_used_as_proof"] is False


@pytest.mark.parametrize("max_depth", (-1, 9, True, 1.5))
def test_invalid_interval_budget_is_rejected(checker, max_depth):
    with pytest.raises(ValueError, match="INVALID_INTERVAL_BUDGET"):
        ContinuousVerifier(checker, max_depth=max_depth)
