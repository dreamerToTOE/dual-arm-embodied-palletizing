"""Actual frozen-model gate controls; stored states are tests-only references.

The adapter implementation does not read these references, old A-stage paths,
or IK solution files.  Tests use independently frozen G1-1/R1 controls only.
"""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from offline.task28_p4_nr_0.validation import CandidateValidator
from task28_contact.modes import (OperationMode, SupportSite, AttachmentIntent)
from task28_contact.clearance import ClearancePhase, ConfirmationEvent
from task28_edge.types import Trajectory, Motion, EdgeStatus, Source, SamplingPolicy


@pytest.fixture(scope="module")
def adapter():
    return CandidateValidator()


@pytest.fixture(scope="module")
def airborne():
    data = json.loads((REPO/"offline/task28_g1_1/tests/control_states.json").read_text())["states"][0]
    X = np.eye(4)
    X[:3, 3] = data["object_position_m"]
    return X, np.array(data["left_q"]+data["right_q"])


@pytest.fixture(scope="module")
def supported():
    data = json.loads((REPO/"offline/task28_g1_2a/examples/r1_configurations.json").read_text())
    data = data["configurations"]["supply_gap_0000um"]
    X = np.eye(4)
    X[:3, 3] = data["object_position_m"]
    return X, np.array(data["left_q"]+data["right_q"])


def test_state_reuses_full_independent_fk_closure_and_original_414_pairs(adapter, airborne):
    result = adapter.state(*airborne)
    assert result["valid"], result["reasons"]
    assert result["closure"]["valid"]
    assert len(result["independent_fk"]) == 20
    assert result["collision"]["reference"]["checked_shape_pairs"] == 414
    assert result["collision"]["reference"]["skipped_acm_shape_pairs"] == 40
    assert result["tcp_reconstruction"] == "hash_bound_USD_frame_oracle_not_measured_observation"


def test_moving_object_without_joints_is_rejected_not_projected(adapter, airborne):
    X, q = airborne
    changed = X.copy()
    changed[0, 3] += .002
    result = adapter.state(changed, q)
    assert not result["valid"]
    assert "LEFT_GRASP_TRANSLATION_MISMATCH" in result["reasons"]
    assert "RIGHT_GRASP_TRANSLATION_MISMATCH" in result["reasons"]
    assert result["collision"]["reference"]["checked_shape_pairs"] == 414
    np.testing.assert_array_equal(X, airborne[0])


def test_joint_limits_are_not_clipped_or_skipped(adapter, airborne):
    X, q = airborne
    changed = q.copy()
    changed[0] = adapter.model.joint_upper[0] + .01
    result = adapter.state(X, changed)
    assert not result["valid"]
    assert "LEFT_JOINT_LIMIT_VIOLATION" in result["reasons"]
    assert result["minimum_joint_limit_margin_rad"] == pytest.approx(-.01)
    assert result["collision"]["reference"]["checked_shape_pairs"] == 414


def test_hold_edge_reuses_original_sampling_including_exact_midpoint(adapter, airborne):
    result = adapter.edge([airborne, airborne], identity="new-hold-control")
    assert result.status is EdgeStatus.SAMPLED_EDGE_VALID
    assert result.evaluated_sample_count == result.planned_sample_count == 3
    assert result.samples[1]["u"] == .5
    assert result.samples[1]["waypoint"]["provenance"]["source"] is Source.MODEL_INTERPOLATION
    assert result.continuous_safety == "NOT_ESTABLISHED_FINITE_SAMPLING_ONLY"
    assert all(row["collision"]["reference"]["checked_shape_pairs"] == 414 for row in result.samples)


def test_state_site_does_not_bypass_original_support_geometry(adapter, supported):
    assert adapter.state(*supported, SupportSite.SUPPLY)["valid"]
    result = adapter.state(*supported, SupportSite.TARGET)
    assert not result["valid"]
    assert any("FOOTPRINT" in reason for reason in result["reasons"])


@pytest.mark.parametrize("kind", ["nonfinite", "bad_dimension", "bad_se3", "boolean_joints"])
def test_input_contract_rejects_before_reconstruction(adapter, airborne, kind):
    X, q = airborne
    X, q = X.copy(), q.copy()
    if kind == "nonfinite": q[0] = np.nan
    elif kind == "bad_dimension": q = q[:-1]
    elif kind == "bad_se3": X[0, 0] = 2.
    else: q = np.zeros(14, dtype=bool)
    with pytest.raises(ValueError):
        adapter.state(X, q)


def test_explicit_temporal_inputs_are_synthetic_fresh_and_not_sampling_counts(adapter, supported):
    intent = AttachmentIntent("test-new-detached", -.1, False, False)
    rows = tuple(adapter.waypoint(*supported, i*.02, f"new-support-{i}",
        site=SupportSite.SUPPLY, mode=OperationMode.DETACHED_SUPPORTED,
        attached=False, intent=intent) for i in range(3))
    trajectory = Trajectory("new-synthetic-support", rows, Motion.HOLD,
        enforce_temporal_confirmation=True,
        required_terminal_phase=ClearancePhase.DETACHED_SUPPORTED_CONFIRMED)
    result = adapter.checker.validate(trajectory, SamplingPolicy(max_dt_s=.001))
    assert result.status is EdgeStatus.SAMPLED_EDGE_VALID
    assert result.evaluated_sample_count > 3
    assert len(result.temporal_evidence) == 3
    assert result.resolution["interpolated_points_used_for_temporal_confirmation"] == 0
    assert result.resolution["temporal_end_confirmed"]
    for event in result.temporal_evidence:
        assert event["frame"]["cube_pose_source"].value == "synthetic_test_input"
        assert not event["result"]["physical_support_observed"]
    two = adapter.checker.validate(replace(trajectory, waypoints=rows[:2]),
                                   SamplingPolicy(max_dt_s=.001))
    assert two.status is EdgeStatus.EDGE_INCONCLUSIVE
    assert two.reasons == ("TERMINAL_PHASE_NOT_CONFIRMED",)


def test_attach_intent_has_causal_issue_time_and_three_frames(adapter, supported):
    detached = AttachmentIntent("new-detached", -.1, False, False)
    held = AttachmentIntent("new-attached", .05, True, True)
    rows = [adapter.waypoint(*supported, i*.02, f"new-detached-{i}",
        site=SupportSite.SUPPLY, mode=OperationMode.DETACHED_SUPPORTED,
        attached=False, intent=detached) for i in range(3)]
    rows += [adapter.waypoint(*supported, (i+3)*.02, f"new-attached-{i}",
        site=SupportSite.SUPPLY, mode=OperationMode.BILATERAL_SUPPORTED,
        event=ConfirmationEvent.ATTACH if i==0 else ConfirmationEvent.OBSERVE,
        intent=held) for i in range(3)]
    from task28_contact.clearance import TemporalClearanceConfirmer, ClearanceFrame, CubePoseSource
    confirmer = TemporalClearanceConfirmer(adapter.checker.contact)
    results = [confirmer.observe(ClearanceFrame(adapter.checker.snapshot(w),
        w.provenance.cube_evidence_id, CubePoseSource.SYNTHETIC_TEST_INPUT),
        w.time_s, w.event) for w in rows]
    assert all(r.valid for r in results)
    assert results[-1].phase is ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED
    assert results[-1].phase_confirmed
    assert not results[-2].phase_confirmed


def test_disconnected_plan_cannot_skip_an_unplanned_boundary(adapter, airborne):
    X, q = airborne
    shifted = X.copy()
    shifted[0, 3] += .001
    result = adapter.confirm_plan([airborne]*2, [(shifted, q)]*2, [(shifted, q)]*2)
    assert not result["valid"]
    assert result["reasons"] == ("DISCONNECTED_LIFT_TRANSPORT_BOUNDARY",)
    assert result["sampled"] == {} and not result["execution_authorized"]


def test_b1_adapter_retains_separate_original_statuses_and_zero_depth(adapter, supported):
    trajectory = adapter.trajectory([supported]*2, identity="new-bounded-proof-control",
                                    site=SupportSite.SUPPLY)
    result = adapter.continuous(trajectory)
    assert result["maximum_subdivision_depth"] == 0
    assert result["joint_limits"]["status"] == "CONTINUOUSLY_VERIFIED"
    assert result["closed_chain"]["status"] == "CONTINUOUSLY_VERIFIED"
    assert result["collision"]["checked_pairs"] == 414
    assert result["collision"]["status"] == "CONTINUOUS_VERIFICATION_INCONCLUSIVE"
    assert result["combined_status"] == "CONTINUOUS_VERIFICATION_INCONCLUSIVE"
    assert not result["sampled_evidence_used_as_proof"]


def _r1_test_node(name):
    """Tests-only independent endpoint controls, never a planner reference path."""
    data = json.loads((REPO/"offline/task28_g1_2a/examples/r1_configurations.json").read_text())
    data = data["configurations"][name]
    X = np.eye(4)
    X[:3, 3] = data["object_position_m"]
    return X, np.array(data["left_q"]+data["right_q"])


def test_r1_confirmation_frame_count_cannot_be_increased_by_dense_planner_nodes(monkeypatch):
    """Mock only B0 to isolate counting; actual R1/FK/FCL remain authoritative.

    No collision/closure feasibility of the long endpoint interpolation is
    claimed by this counting control.  Actual sampled edges are covered above.
    """
    validator = CandidateValidator()
    monkeypatch.setattr(validator.checker, "validate", lambda t:
        SimpleNamespace(status=EdgeStatus.SAMPLED_EDGE_VALID, reasons=()))
    a = _r1_test_node("supply_gap_0000um")
    b = _r1_test_node("supply_gap_4000um")
    c = _r1_test_node("target_gap_4000um")
    d = _r1_test_node("target_gap_0000um")
    sparse = validator.confirm_plan([a,b], [b,c], [c,d], identity="count-sparse")
    dense = validator.confirm_plan([a]*13+[b]*17, [b]*19+[c]*23,
                                   [c]*29+[d]*31, identity="count-dense")
    assert sparse["temporal_session"]["valid"] and dense["temporal_session"]["valid"]
    assert len(sparse["temporal"]) == len(dense["temporal"]) == 19
    for result in (sparse, dense):
        assert result["temporal_session"]["dense_path_nodes_used_as_observations"] == 0
        assert result["temporal_session"]["interpolated_points_used_as_observations"] == 0
        assert result["temporal"][-1]["result"]["phase"] is ClearancePhase.RELEASED_SUPPORTED_CONFIRMED
        assert len({r["provenance"]["cube_evidence_id"] for r in result["temporal"]}) == 19
        for trajectory in result["trajectories"].values():
            assert not trajectory.enforce_temporal_confirmation
            assert trajectory.temporal_prelude == ()
            assert trajectory.required_terminal_phase is None
        assert not result["execution_authorized"]
    assert len(sparse["trajectories"]["vertical_lift"].waypoints) < len(dense["trajectories"]["vertical_lift"].waypoints)
    assert [r["time_s"] for r in sparse["temporal"]] == [r["time_s"] for r in dense["temporal"]]


def test_dense_low_clearance_nodes_cannot_manufacture_airborne_confirmation(monkeypatch):
    validator = CandidateValidator()
    monkeypatch.setattr(validator.checker, "validate", lambda t:
        SimpleNamespace(status=EdgeStatus.SAMPLED_EDGE_VALID, reasons=()))
    a = _r1_test_node("supply_gap_0000um")
    b = _r1_test_node("supply_gap_2000um")
    c = _r1_test_node("target_gap_4000um")
    d = _r1_test_node("target_gap_0000um")
    result = validator.confirm_plan([a]+[b]*50, [b,c], [c,d], identity="low-not-airborne")
    assert not result["valid"] and not result["temporal_session"]["valid"]
    assert len(result["temporal"]) == 19
    assert "PLACE_REQUIRES_CONFIRMED_AIRBORNE" in result["temporal_session"]["reasons"]
    assert not result["execution_authorized"]
