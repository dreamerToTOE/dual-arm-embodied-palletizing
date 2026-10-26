"""Independent polynomial oracle + unchanged frozen FK/FCL point validation."""
import math

import numpy as np
import pytest

from adversarial_controls import (SHAPE_MAXIMUM, WITNESS_U, adversarial_controls,
                                  analytic_control_evidence,
                                  independent_object_position)
from task28_edge.checker import SampledEdgeValidator, contract
from task28_edge.interpolation import reconstruct
from task28_edge.types import EdgeStatus, SamplingPolicy, Trajectory


@pytest.fixture(scope="module")
def checker():
    return SampledEdgeValidator()


@pytest.fixture(scope="module")
def cases():
    return adversarial_controls()


@pytest.fixture(scope="module")
def actual_results(checker, cases):
    result = {}
    for name, control in cases.items():
        trajectory = control.trajectory
        first, last = trajectory.waypoints
        witness = reconstruct(first, last, control.witness_u,
                              trajectory.interpolation, trajectory.motion,
                              checker.contact.collision.support, trajectory.identity)
        result[name] = (checker.validate(trajectory), checker.evaluate(witness))
    return result


def test_oracle_root_and_peak_are_analytic_not_an_adaptive_grid():
    # An independently written polynomial derivative has exactly one root in
    # [0,1]; its other root lies outside.  Its second derivative is negative.
    assert 0.0 < WITNESS_U < 0.5
    assert abs(1.0 - 3.0 * WITNESS_U + 1.5 * WITNESS_U**2) < 1e-15
    assert -3.0 + 3.0 * WITNESS_U < 0.0
    assert 1.0 + 1.0 / math.sqrt(3.0) > 1.0
    assert 0.5 * WITNESS_U**3 - 1.5 * WITNESS_U**2 + WITNESS_U == pytest.approx(SHAPE_MAXIMUM)
    assert 0.5 * 0.5**3 - 1.5 * 0.5**2 + 0.5 < SHAPE_MAXIMUM


@pytest.mark.parametrize("name", ("offgrid_closure_excursion", "offgrid_support_penetration"))
def test_real_typed_hermite_reconstruction_matches_separate_polynomial(checker, cases, name):
    control = cases[name]
    trajectory = control.trajectory
    assert type(trajectory) is Trajectory
    assert contract(trajectory, SamplingPolicy()) == ()
    first, last = trajectory.waypoints
    assert first.left_q == last.left_q and first.right_q == last.right_q
    assert first.object_position_m == last.object_position_m
    for u in (0.0, WITNESS_U, 0.5, 1.0):
        actual = reconstruct(first, last, u, trajectory.interpolation,
                             trajectory.motion, checker.contact.collision.support,
                             trajectory.identity)
        expected = independent_object_position(control, u)
        assert np.asarray(actual.object_position_m) == pytest.approx(expected, abs=2e-16)
        assert actual.left_q == first.left_q and actual.right_q == first.right_q


@pytest.mark.parametrize("name", ("offgrid_closure_excursion", "offgrid_support_penetration"))
def test_old_default_finite_grid_accepts_but_actual_offgrid_point_rejects(cases, actual_results, name):
    control = cases[name]
    ordinary, witness = actual_results[name]
    assert ordinary.status is EdgeStatus.SAMPLED_EDGE_VALID
    assert ordinary.endpoints_valid and all(row["valid"] for row in ordinary.endpoint_results)
    assert ordinary.resolution["span_subdivisions"] == [2]
    assert tuple(row["u"] for row in ordinary.samples) == (0.0, 0.5, 1.0)
    assert ordinary.planned_sample_count == ordinary.evaluated_sample_count == 3
    assert all(row["valid"] for row in ordinary.samples)
    assert WITNESS_U not in tuple(row["u"] for row in ordinary.samples)
    assert not witness["valid"]
    assert set(control.expected_witness_reasons).issubset(witness["reasons"])
    for row in (*ordinary.samples, witness):
        assert row["minimum_joint_limit_margin_rad"] > 0.3
        assert len(row["independent_fk"]) == 20
        assert row["collision"]["reference"]["checked_shape_pairs"] == 414
        assert row["collision"]["reference"]["skipped_acm_shape_pairs"] == 40


def test_closure_control_crosses_frozen_grasp_tolerance_without_collision(actual_results):
    ordinary, witness = actual_results["offgrid_closure_excursion"]
    assert ordinary.maxima["max_closure_translation_m"] < 1e-4
    assert witness["closure"]["left_object_translation_error_m"] > 1e-4
    assert witness["closure"]["right_object_translation_error_m"] > 1e-4
    assert witness["collision"]["valid"] and witness["geometry"]["valid"]


def test_penetration_control_keeps_closure_valid_but_crosses_frozen_depth(actual_results):
    ordinary, witness = actual_results["offgrid_support_penetration"]
    assert all(row["geometry"]["clearance_m"] >= -1e-6 for row in ordinary.samples)
    assert witness["geometry"]["clearance_m"] < -1e-6
    assert witness["closure"]["valid"]
    assert not witness["collision"]["valid"]
    assert any({record["owner_a"], record["owner_b"]} == {"Cube_01", "task28_table"}
               for record in witness["collision"]["remaining_collisions"])


def test_saved_oracle_facts_identify_unsampled_extremum(cases):
    for control in cases.values():
        facts = analytic_control_evidence(control)
        assert facts["oracle"] == "closed_form_cubic_derivative_root_not_grid_search"
        assert facts["witness_u"] not in facts["ordinary_sample_us"]
        assert abs(facts["signed_peak_offset_m"]) > facts["frozen_threshold_m"]
        assert max(map(abs, facts["ordinary_sample_offsets_m"])) < facts["frozen_threshold_m"]
        assert facts["witness_position_m"] == independent_object_position(control, WITNESS_U)
