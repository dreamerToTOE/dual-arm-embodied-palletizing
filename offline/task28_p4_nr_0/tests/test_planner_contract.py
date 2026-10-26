"""Bounded-search contracts; mocked tree controls are NOT physical results.

The mock tests deliberately bypass FK/FCL to isolate search bookkeeping.  They
cannot contribute candidate evidence.  Separate real-NR, real reach-bound and
original-B0 hold controls below exercise the actual adapter seams, CPU only.
"""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from offline.task28_p4_nr_0.planner import (Planner, PlannerConfig, PlanningBudget,
                                          interpolate, distances)
from offline.task28_p4_nr_0.projection import NRProjector, ProjectionResult
from offline.task28_p4_nr_0.validation import CandidateValidator
from task28_validator.model import RobotModel
from task28_validator.se3 import transform, rpy_matrix
from task28_edge.types import EdgeStatus, Motion


@pytest.fixture(scope="module")
def model():
    return RobotModel()


class MockSearchValidator:
    """Search-control backend only: makes no geometric feasibility assertion."""
    scope = "MOCK_SEARCH_BOOKKEEPING_ONLY_NO_PHYSICAL_CANDIDATE"

    def __init__(self, model, reject_transport=False, reject_candidate=False):
        self.model = model
        self.reject_transport = reject_transport
        self.reject_candidate = reject_candidate
        self.edges = []
        self.states = []
        self.candidates = []

    def state(self, X, q14, site=None):
        self.states.append((X.copy(), q14.copy(), site))
        return {"valid": True, "reasons": ()}

    def edge(self, nodes, motion, identity):
        self.edges.append((nodes, motion, identity))
        rejected = self.reject_transport and motion is Motion.TRANSPORT
        return SimpleNamespace(
            status=EdgeStatus.SAMPLED_EDGE_REJECTED if rejected else EdgeStatus.SAMPLED_EDGE_VALID,
            reasons=("DELIBERATE_MOCK_PAYLOAD_COLLISION",) if rejected else ())

    def confirm_plan(self, lift, transport, place, **kwargs):
        self.candidates.append((lift, transport, place, kwargs))
        maxima = {"max_closure_translation_m": 0., "max_closure_rotation_rad": 0.,
                  "minimum_joint_limit_margin_rad": .1}
        return {"valid": not self.reject_candidate,
                "reasons": ("DELIBERATE_MOCK_CLOSURE_REJECTION",) if self.reject_candidate else (),
                "sampled": {"mock_only": SimpleNamespace(maxima=maxima)}, "continuous": {},
                "scope": self.scope, "execution_authorized": False}


def mocked_search(model, seed=28, config=None, **validator_options):
    config = config or PlannerConfig(max_lift_trials=1, max_expansions=8,
        anchor_restarts=1, goal_bias=1., leaf_translation_m=1.,
        leaf_rotation_rad=1., leaf_joint_step_rad=1.)
    validator = MockSearchValidator(model, **validator_options)
    events = []
    planner = Planner(seed, config, validator=validator, emit=events.append)
    calls = []

    def project(X, q):
        q = np.asarray(q).copy()
        calls.append((X.copy(), q.copy()))
        margin = float(min(np.min(q-np.tile(model.joint_lower, 2)),
                           np.min(np.tile(model.joint_upper, 2)-q)))
        return ProjectionResult(True, tuple(q), "MOCK_ONLY", 0, (), 0., 0., margin)

    planner.projector.project = project
    return planner, validator, events, calls


def query_poses():
    return transform(translation=(.65, 0., .23)), transform(translation=(.77, 0., .23))


def test_mock_result_backtraces_parent_edges_and_excludes_valid_dead_branch(model, monkeypatch):
    planner, validator, events, calls = mocked_search(model)
    sampling_calls = []

    def sample(target):
        value = target.copy()
        if not sampling_calls:
            value[:3, 3] = (.65, .04, target[2, 3])
        sampling_calls.append(value.copy())
        return value

    monkeypatch.setattr(planner, "sample", sample)
    # No saved A-stage reference, waypoint path or IK solution is read in plan.
    with patch.object(Path, "read_text", side_effect=AssertionError("No precomputed answer reads")):
        result = planner.plan(*query_poses(), continuous=False)
    assert result.found  # MOCK SEARCH CONTRACT ONLY, not a physical success.
    assert result.label == "OFFLINE_CANDIDATE_UNCERTIFIED"
    assert not result.execution_authorized
    insertions = [row for row in events if row["kind"] == "tree_insert"]
    assert insertions[0]["parent"] == insertions[1]["parent"] == 0
    assert insertions[0]["node"][0][1, 3] > .039
    assert all(abs(node[0][1, 3]) < 1e-15 for node in result.transport)
    indexed = {row["node_index"]: row for row in insertions}
    chain = []
    current = insertions[-1]["node_index"]
    while current:
        chain.append(indexed[current])
        current = indexed[current]["parent"]
    expected = list(reversed(chain))
    assert len(result.transport) == len(expected)+1
    for node, insertion in zip(result.transport[1:], expected):
        np.testing.assert_array_equal(node[0], insertion["node"][0])
        np.testing.assert_array_equal(node[1], insertion["node"][1])
    assert len(validator.candidates) == 1
    # Each inserted edge really went through the adapter before insertion.
    edge_events = [row for row in events if row["kind"] == "leaf_edge"]
    assert len(edge_events) >= len(insertions)
    for insertion in insertions:
        insertion_index = next(index for index, row in enumerate(events) if row is insertion)
        assert any(row["kind"] == "leaf_edge"
                   and len(row["nodes"]) == len(insertion["edge"])
                   and all(np.array_equal(a[0], b[0]) and np.array_equal(a[1], b[1])
                           for a, b in zip(row["nodes"], insertion["edge"]))
                   for row in events[:insertion_index])


def test_rejected_transport_leaf_never_enters_tree_or_candidate(model):
    planner, validator, events, calls = mocked_search(model, reject_transport=True)
    result = planner.plan(*query_poses(), continuous=False)
    assert not result.found
    assert result.reason == "BOUNDED_SEARCH_EXHAUSTED_WITHOUT_ACCEPTED_CANDIDATE"
    assert result.statistics["accepted_tree_nodes"] == 1  # lift root only
    assert not any(row["kind"] == "tree_insert" for row in events)
    assert validator.candidates == []
    assert result.statistics["collision_rejections"] == planner.config.max_expansions
    assert sum(motion is Motion.TRANSPORT for _, motion, _ in validator.edges) == planner.config.max_expansions


def test_full_candidate_rejection_is_not_reported_as_planner_success(model):
    planner, validator, events, calls = mocked_search(model, reject_candidate=True)
    result = planner.plan(*query_poses(), continuous=False)
    assert not result.found
    assert result.label == "NO_ACCEPTED_CANDIDATE"
    assert result.statistics["full_candidate_attempts"] > 0
    assert result.statistics["closure_rejections"] > 0
    assert result.lift == result.transport == result.place == []
    assert not result.execution_authorized


def test_global_projection_budget_is_checked_before_extra_call(model):
    config = PlannerConfig(max_projection_calls=3, max_lift_trials=1,
        anchor_restarts=1, goal_bias=1., leaf_translation_m=1., leaf_joint_step_rad=1.)
    planner, validator, events, calls = mocked_search(model, config=config)
    result = planner.plan(*query_poses(), continuous=False)
    assert not result.found
    assert result.reason == "NR_CALL_BUDGET_EXHAUSTED"
    assert result.statistics["ik_attempts"] == len(calls) == 3
    assert validator.candidates == []
    with pytest.raises(PlanningBudget, match="NR_CALL_BUDGET_EXHAUSTED"):
        planner.project(*calls[0], purpose="would_exceed")
    assert len(calls) == 3


def test_accepted_node_cap_is_global_across_lift_trials(model):
    config = PlannerConfig(max_accepted_nodes=2, max_lift_trials=3,
        max_expansions=5, anchor_restarts=1, goal_bias=1.,
        leaf_translation_m=1., leaf_joint_step_rad=1.)
    planner, validator, events, calls = mocked_search(model, config=config)
    result = planner.plan(*query_poses(), continuous=False)
    assert not result.found
    assert result.reason == "TREE_NODE_BUDGET_EXHAUSTED"
    assert result.statistics["accepted_tree_nodes"] == 2
    assert len([row for row in events if row["kind"] == "lift_sample"]) == 1
    assert len([row for row in events if row["kind"] == "tree_insert"]) == 1
    assert validator.candidates == []


def test_continuously_rejected_candidate_is_rejected_even_when_sampled_passes(model):
    planner, validator, _, _ = mocked_search(model)
    original_confirm = validator.confirm_plan

    def confirm(*args, **kwargs):
        value = original_confirm(*args, **kwargs)
        value["continuous"] = {"transport": {"combined_status": "CONTINUOUSLY_REJECTED"}}
        return value

    validator.confirm_plan = confirm
    result = planner.plan(*query_poses())
    assert not result.found
    assert result.statistics["full_candidate_attempts"] > 0
    assert result.statistics["rejection_reasons"]["CONTINUOUS_CERTIFIED_REJECTION:transport"] > 0
    assert not result.execution_authorized


def test_inconclusive_continuous_dimensions_are_retained_and_never_authorize_execution(model):
    planner, validator, _, _ = mocked_search(model)
    original_confirm = validator.confirm_plan
    dimensions = {"combined_status": "CONTINUOUS_VERIFICATION_INCONCLUSIVE",
                  "joint_limits": {"status": "CONTINUOUSLY_VERIFIED"},
                  "closed_chain": {"status": "CONTINUOUS_VERIFICATION_INCONCLUSIVE"},
                  "collision": {"status": "CONTINUOUS_VERIFICATION_INCONCLUSIVE"}}

    def confirm(*args, **kwargs):
        value = original_confirm(*args, **kwargs)
        value["continuous"] = {"transport": dimensions}
        return value

    validator.confirm_plan = confirm
    result = planner.plan(*query_poses())
    assert result.found  # Mock search acceptance only, no physical evidence.
    assert result.validation["continuous"]["transport"] == dimensions
    assert result.label == "OFFLINE_CANDIDATE_UNCERTIFIED"
    assert not result.execution_authorized


def test_connection_depth_budget_fails_before_unchecked_edge_or_projection(model):
    config = PlannerConfig(max_connection_depth=0, leaf_translation_m=.001)
    planner, validator, events, calls = mocked_search(model, config=config)
    a, b = query_poses()
    assert planner.connect((a, np.tile(model.home, 2)), (b, np.tile(model.home, 2)), Motion.TRANSPORT) is None
    assert calls == validator.edges == []
    assert planner.stats["candidate_connection_attempts"] == 1
    assert planner.stats["rejection_reasons"] == {"PROJECTED_RBS_DEPTH_BUDGET": 1}


def test_local_connection_projects_geodesic_midpoint_from_mean_joint_seed(model):
    config = PlannerConfig(max_connection_depth=1, leaf_translation_m=.01,
                           leaf_rotation_rad=.03, leaf_joint_step_rad=.025)
    planner, validator, events, calls = mocked_search(model, config=config)
    a = transform(translation=(.65, 0., .4))
    b = transform(rpy_matrix(0., 0., .04), (.666, 0., .4))
    qa, qb = np.tile(model.home, 2), np.tile(model.home, 2)+.01
    connected = planner.connect((a, qa), (b, qb), Motion.TRANSPORT)
    assert connected is not None and len(connected) == 3
    assert len(calls) == 1 and len(validator.edges) == 2
    np.testing.assert_array_equal(calls[0][1], (qa+qb)/2)
    np.testing.assert_allclose(calls[0][0], interpolate(a, b, .5), atol=0., rtol=0)
    assert any(row.get("purpose") == "projected_RBS_midpoint" for row in events)


def test_only_sampled_edge_valid_enum_can_accept_a_leaf(model):
    for status in (EdgeStatus.ENDPOINT_VALID, EdgeStatus.EDGE_INCONCLUSIVE,
                   EdgeStatus.SAMPLED_EDGE_REJECTED, "sampled_edge_valid"):
        planner, validator, _, _ = mocked_search(model)
        validator.edge = lambda *args, **kwargs: SimpleNamespace(status=status, reasons=("MOCK_NOT_ACCEPTED",))
        a = (transform(translation=(.65, 0., .4)), np.tile(model.home, 2))
        assert planner.connect(a, a, Motion.HOLD) is None
        assert planner.stats["rejection_reasons"]["MOCK_NOT_ACCEPTED"] == 1


def test_fixed_seed_sampling_is_deterministic_and_contains_nonzero_so3(model):
    config = PlannerConfig(goal_bias=0.)
    first = Planner(281, config, validator=SimpleNamespace(model=model))
    second = Planner(281, config, validator=SimpleNamespace(model=model))
    goal = transform(translation=(.77, 0., .35))
    samples = [first.sample(goal) for _ in range(12)]
    repeated = [second.sample(goal) for _ in range(12)]
    for a, b in zip(samples, repeated):
        np.testing.assert_array_equal(a, b)
        assert np.all(a[:3, 3] >= config.sample_translation_lower_m)
        assert np.all(a[:3, 3] <= config.sample_translation_upper_m)
        angle = Rotation.from_matrix(a[:3, :3]).as_rotvec()
        assert np.all(np.abs(angle) <= config.sample_rotation_vector_abs_rad)
        assert np.linalg.norm(angle) > 0.
    assert first.stats["sampled_nodes"] == 12
    other = Planner(282, config, validator=SimpleNamespace(model=model)).sample(goal)
    assert not np.array_equal(samples[0], other)


def test_fixed_seed_mock_search_repeats_path_and_nontiming_counts(model):
    first, _, _, _ = mocked_search(model, seed=7)
    second, _, _, _ = mocked_search(model, seed=7)
    a, b = first.plan(*query_poses(), continuous=False), second.plan(*query_poses(), continuous=False)
    assert a.found == b.found and a.reason == b.reason
    for left, right in zip(a.lift+a.transport+a.place, b.lift+b.transport+b.place):
        np.testing.assert_array_equal(left[0], right[0])
        np.testing.assert_array_equal(left[1], right[1])
    assert {k: v for k, v in a.statistics.items() if k != "planning_time_s"} == {
        k: v for k, v in b.statistics.items() if k != "planning_time_s"}


def test_planner_project_uses_actual_nr_without_old_ik_or_joint_clipping(model):
    planner = Planner(28, validator=SimpleNamespace(model=model))
    assert isinstance(planner.projector, NRProjector)
    q = np.tile(model.home, 2)
    X = transform(translation=(.65, 0., .4))
    with patch.object(model, "ik", side_effect=AssertionError("No old IK")), \
            patch.object(planner.projector, "project", wraps=planner.projector.project) as call:
        node = planner.project(X, q, "actual_NR_contract_control")
    assert node is not None
    call.assert_called_once()
    assert model.within_limits(node[1][:7]) and model.within_limits(node[1][7:])
    # An outside-limit seed is rejected with its exact value intact.
    q[0] = model.joint_upper[0]+.01
    before = q.copy()
    assert planner.project(X, q, "negative_out_of_limit_seed") is None
    np.testing.assert_array_equal(q, before)
    assert planner.stats["joint_limit_rejections"] == 1


def test_actual_model_unreachable_goal_has_independent_necessary_certificate(model):
    events = []
    planner = Planner(28, validator=SimpleNamespace(model=model), emit=events.append)
    start, _ = query_poses()
    goal = transform(translation=(100., 0., .23))
    with patch.object(planner.projector, "project", side_effect=AssertionError("No solve beyond necessary reach")):
        result = planner.plan(start, goal)
    assert not result.found
    assert result.reason == "GOAL_OUTSIDE_NECESSARY_REACH_BOUND"
    assert result.statistics["ik_attempts"] == result.statistics["sampled_nodes"] == 0
    assert not result.execution_authorized
    # Independently, 100m is beyond the sum of all link translations from base.
    for side in ("left", "right"):
        target = goal @ getattr(planner.projector._grasps, side)
        base = np.asarray(model.manifest["bases_world_m"][side])
        segment_sum = sum(np.linalg.norm(row["origin"][:3, 3]) for row in model.chains[side][1:])
        assert np.linalg.norm(target[:3, 3]-base) > segment_sum


def test_planner_leaf_invokes_original_b0_sampling_with_payload_not_endpoint_only():
    adapter = CandidateValidator()
    planner = Planner(28, validator=adapter)
    row = json.loads((REPO/"offline/task28_g1_1/tests/control_states.json").read_text())["states"][0]
    node = (transform(translation=row["object_position_m"]), np.array(row["left_q"]+row["right_q"]))
    with patch.object(adapter.checker, "validate", wraps=adapter.checker.validate) as original_b0:
        connected = planner.connect(node, node, Motion.HOLD)
    assert connected is not None
    original_b0.assert_called_once()
    trajectory = original_b0.call_args.args[0]
    assert len(trajectory.waypoints) == 2
    # Retain the actual B0 result through an independent direct recheck.
    result = adapter.checker.validate(trajectory)
    assert result.status is EdgeStatus.SAMPLED_EDGE_VALID
    assert result.evaluated_sample_count == 3
    assert all(sample["collision"]["reference"]["checked_shape_pairs"] == 414 for sample in result.samples)
    assert result.continuous_safety == "NOT_ESTABLISHED_FINITE_SAMPLING_ONLY"


@pytest.mark.parametrize("seed", [-1, True, 1., None, "28"])
def test_invalid_seed_rejected_before_work(model, seed):
    with pytest.raises(ValueError, match="NONNEGATIVE_INTEGER_SEED_REQUIRED"):
        Planner(seed, validator=SimpleNamespace(model=model))


@pytest.mark.parametrize("kwargs", [
    {"max_expansions": 0}, {"max_projection_calls": True}, {"max_lift_trials": 11},
    {"max_accepted_nodes": 1}, {"anchor_restarts": 0}, {"backtracks": -1},
    {"max_connection_depth": 13}, {"max_nr_iterations_per_call": 2001},
    {"leaf_translation_m": 0.}, {"max_step_rad": True}, {"svd_cutoff": np.nan},
    {"goal_bias": np.inf}, {"goal_bias": -.1},
    {"sample_translation_lower_m": (0., 0.)},
    {"sample_translation_upper_m": (.1, -.2, .1)},
    {"sample_rotation_vector_abs_rad": (.1, -.1, .1)},
    {"lift_height_range_m": (.2, .3)}, {"lift_height_range_m": (.4, .3)},
])
def test_invalid_planner_budgets_and_workspace_fail_before_search(kwargs):
    with pytest.raises(ValueError):
        PlannerConfig(**kwargs)


@pytest.mark.parametrize("bad", [np.eye(3), np.zeros((4, 4)), np.full((4, 4), np.nan)])
def test_invalid_query_transform_rejected_before_projection(model, bad):
    planner, validator, events, calls = mocked_search(model)
    with pytest.raises(ValueError):
        planner.plan(bad, query_poses()[1])
    assert calls == validator.states == validator.edges == events == []


def test_geodesic_interpolation_rotates_and_preserves_se3():
    first = transform(rpy_matrix(.1, -.2, .3), (.1, .2, .3))
    second = transform(rpy_matrix(-.2, .1, -.1), (.3, .4, .5))
    middle = interpolate(first, second, .5)
    np.testing.assert_allclose(middle[:3, 3], (.2, .3, .4), atol=1e-16, rtol=0)
    np.testing.assert_allclose(middle[:3, :3].T @ middle[:3, :3], np.eye(3), atol=1e-14, rtol=0)
    assert distances(first, middle)[1] == pytest.approx(distances(middle, second)[1], abs=1e-14)
