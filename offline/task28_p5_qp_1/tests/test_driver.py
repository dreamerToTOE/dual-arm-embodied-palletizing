"""Bounded offline integration and fail-closed receipt controls.

Real FR3 tests start at an unchanged saved NR-0 configuration.  Injected
rejected/inconclusive receipt tests exercise driver bookkeeping only; they are
explicit abstract controls and do not establish robot-path feasibility.
"""
from dataclasses import replace
import gzip
import json
from pathlib import Path
import unittest

import numpy as np

from offline.task28_p5_qp_0 import CentralizedVelocityQP, QPConfig
from offline.task28_p4_nr_0.validation import CandidateValidator
from offline.task28_p5_qp_1.driver import MultiStepDriver, MultiStepRequest
from task28_edge.types import EdgeStatus
from task28_contact.modes import SupportSite

ROOT = Path(__file__).resolve().parents[3]


def saved_node(segment="transport", index=None):
    path = ROOT / "offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz"
    with gzip.open(path, "rt") as stream:
        saved = json.load(stream)
    rows = saved[segment]
    X, q = rows[len(rows) // 2 if index is None else index]
    return np.asarray(X), np.asarray(q)


class AbstractEdgeReceiptInjection:
    """Wrap real numerical geometry and substitute only one edge receipt.

    This is an abstract return-contract fault, never evidence for an actual
    FR3 obstacle, closure failure or reachable colliding trajectory.
    """
    scope = "ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE"

    def __init__(self, status, fail_at=2):
        self.real = CandidateValidator()
        self.model = self.real.model
        self.checker = self
        self.status = status
        self.fail_at = fail_at
        self.edge_calls = 0

    def trajectory(self, *args, **kwargs):
        return self.real.trajectory(*args, **kwargs)

    def evaluate(self, *args, **kwargs):
        return self.real.checker.evaluate(*args, **kwargs)

    def validate(self, *args, **kwargs):
        self.edge_calls += 1
        receipt = self.real.checker.validate(*args, **kwargs)
        if self.edge_calls == self.fail_at:
            return replace(receipt, status=self.status,
                           reasons=("ABSTRACT_INJECTED_EDGE_RECEIPT_" + self.status.value,))
        return receipt


class AbstractFeasibleStatusInjection:
    """Forge a numerical receipt to test independent driver hard gates only."""
    def __init__(self, fault):
        self.real = CentralizedVelocityQP()
        self.fault = fault
        self.solve_calls = 0

    def __getattr__(self, name):
        return getattr(self.real, name)

    def solve(self, request):
        self.solve_calls += 1
        honest = self.real.solve(request)
        if honest.status != "FEASIBLE_TRACKING_OK":
            raise AssertionError("Abstract injection must start at an honest valid QP")
        candidate = np.asarray(honest.qdot).copy()
        if self.fault == "tracking":
            candidate[:] = 0.
        elif self.fault == "closure":
            candidate[0] += 1e-5
        else:
            raise AssertionError("Unknown abstract injection")
        return replace(honest, status="FEASIBLE_TRACKING_OK",
                       qdot=tuple(candidate), reasons=("ABSTRACT_FORGED_FEASIBLE_RECEIPT",))


class MultiStepTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.X, cls.q = saved_node()
        cls.driver = MultiStepDriver()

    def request(self, *, steps=3, twist=(.003, -.001, .001, .005, -.003, .01)):
        return MultiStepRequest(self.q.copy(), self.X.copy(), tuple(twist), .01,
                                steps, "unit-seed28-transport-midpoint")

    def test_genuine_saved_fr3_repeated_nonzero_reference_and_payload(self):
        request = self.request()
        result = self.driver.run(request)
        self.assertEqual(result["status"], "COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED")
        self.assertEqual(result["attempted_step_count"], 3)
        self.assertEqual(result["accepted_step_count"], 3)
        self.assertFalse(result["execution_authorized"])
        self.assertFalse(result["continuous_verifier_run"])
        self.assertEqual(result["continuous_safety"], "NOT_CERTIFIED_FINITE_SAMPLING_ONLY")
        self.assertEqual(result["phase_confirmation_observations_generated"], 0)
        self.assertFalse(result["nr_projection_performed"])
        self.assertEqual(result["observation_source"], "SYNTHETIC_OFFLINE_COMPUTATION_NOT_MEASURED")
        expected_q, expected_X = self.q.copy(), self.X.copy()
        previous_velocity = None
        for index, row in enumerate(result["steps"]):
            self.assertTrue(row["accepted"])
            np.testing.assert_array_equal(row["q_initial"], expected_q)
            np.testing.assert_array_equal(row["object_pose_initial"], expected_X)
            velocity = np.asarray(row["qdot14"])
            expected_q = expected_q + request.dt_s * velocity
            np.testing.assert_array_equal(row["q_next"], expected_q)
            expected_X = np.asarray(row["desired_object_pose_next"])
            self.assertGreater(float(np.linalg.norm(velocity)), .001)
            d = row["qp"]["diagnostics"]
            self.assertGreater(d["minimum_position_margin_rad"], 0.)
            self.assertGreaterEqual(d["minimum_velocity_margin_rad_s"], -1e-8)
            bounds = row["frozen_bounds"]
            self.assertTrue(bounds["valid"])
            self.assertGreater(bounds["minimum_position_margin_rad"], 0.)
            self.assertGreaterEqual(bounds["minimum_velocity_margin_rad_s"], -1e-10)
            if previous_velocity is None:
                self.assertIsNone(row["velocity_change_rad_s"])
                self.assertIsNone(row["velocity_change_rate_rad_s2"])
            else:
                difference = velocity - previous_velocity
                np.testing.assert_array_equal(row["velocity_change_rad_s"], difference)
                np.testing.assert_array_equal(row["velocity_change_rate_rad_s2"], difference / request.dt_s)
            self.assertEqual(row["velocity_change_semantics"],
                             "NUMERICAL_DIFFERENCE_ONLY_NO_AUTHENTICATED_ACCELERATION_LIMIT")
            for name in ("current_state", "predicted_state"):
                self.assertTrue(row[name]["valid"])
                reference = row[name]["collision"]["reference"]
                self.assertEqual(reference["checked_shape_pairs"], 414)
                self.assertEqual(reference["skipped_acm_shape_pairs"], 40)
            # The exact 14-DOF Euler recurrence above is intentional: no
            # inverse-kinematics/NR state replacement may hide accumulated drift.
            self.assertGreaterEqual(row["sampled_edge"]["evaluated_sample_count"], 3)
            previous_velocity = velocity
        np.testing.assert_array_equal(result["retained_state"]["q14"], expected_q)
        np.testing.assert_array_equal(result["retained_state"]["object_pose"], expected_X)

    def test_first_rejected_or_inconclusive_edge_halts_and_retains_failed_prediction(self):
        for status in (EdgeStatus.SAMPLED_EDGE_REJECTED, EdgeStatus.EDGE_INCONCLUSIVE):
            with self.subTest(status=status):
                validator = AbstractEdgeReceiptInjection(status)
                result = MultiStepDriver(validator=validator).run(self.request(steps=5))
                self.assertEqual(result["status"], "STOPPED_AT_FIRST_REJECTION")
                self.assertEqual(result["attempted_step_count"], 2)
                self.assertEqual(result["accepted_step_count"], 1)
                self.assertEqual(validator.edge_calls, 2)
                accepted, failed = result["steps"]
                self.assertTrue(accepted["accepted"])
                self.assertFalse(failed["accepted"])
                self.assertEqual(failed["sampled_edge"]["status"], status.value)
                np.testing.assert_array_equal(failed["q_initial"], accepted["q_next"])
                np.testing.assert_array_equal(result["retained_state"]["q14"], accepted["q_next"])
                self.assertGreater(float(np.linalg.norm(np.asarray(failed["q_next"])
                                                       - np.asarray(failed["q_initial"]))), 0.)
                self.assertIn("failure", result)
                self.assertFalse(result["execution_authorized"])
                self.assertEqual(result["injected_backend_scope"], validator.scope)

    def test_bad_tracking_halts_at_first_step_but_keeps_numerical_candidate(self):
        result = self.driver.run(self.request(steps=4, twist=(5., 0., 0., 0., 0., 0.)))
        self.assertEqual(result["status"], "STOPPED_AT_FIRST_REJECTION")
        self.assertEqual(result["attempted_step_count"], 1)
        self.assertEqual(result["accepted_step_count"], 0)
        step = result["steps"][0]
        self.assertEqual(step["qp"]["status"], "FEASIBLE_TRACKING_REJECTED")
        self.assertIsNotNone(step["qdot14"])
        self.assertIsNotNone(step["q_next"])
        self.assertIsNotNone(step["predicted_state"])
        np.testing.assert_array_equal(result["retained_state"]["q14"], self.q)

    def test_abstract_forged_feasible_receipts_cannot_bypass_hard_velocity_checks(self):
        for fault, reason in (("tracking", "INDEPENDENT_TASK_TRACKING_REJECTED"),
                              ("closure", "INDEPENDENT_VELOCITY_CLOSURE_REJECTED")):
            with self.subTest(fault=fault):
                controller = AbstractFeasibleStatusInjection(fault)
                result = MultiStepDriver(controller=controller).run(self.request(steps=5))
                self.assertEqual(result["status"], "STOPPED_AT_FIRST_REJECTION")
                self.assertEqual(result["attempted_step_count"], 1)
                self.assertEqual(result["accepted_step_count"], 0)
                self.assertEqual(controller.solve_calls, 1)
                failed = result["steps"][0]
                self.assertEqual(failed["qp"]["status"], "FEASIBLE_TRACKING_OK")
                self.assertIn(reason, failed["reasons"])
                self.assertIsNotNone(failed["q_next"])
                self.assertFalse(failed["accepted"])
                np.testing.assert_array_equal(result["retained_state"]["q14"], self.q)
                self.assertEqual(result["injected_backend_scope"],
                                 "ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE")
                self.assertFalse(result["execution_authorized"])

    def test_unbounded_high_demand_comparator_stops_on_frozen_joint_bounds(self):
        result = self.driver.run(self.request(steps=4, twist=(5., 0., 0., 0., 0., 0.)),
                                 method="unbounded_reference")
        self.assertEqual(result["status"], "STOPPED_AT_FIRST_REJECTION")
        self.assertEqual(result["attempted_step_count"], 1)
        self.assertEqual(result["accepted_step_count"], 0)
        self.assertIsNotNone(result["steps"][0]["qdot14"])
        self.assertFalse(result["steps"][0]["frozen_bounds"]["valid"])
        self.assertIn("FROZEN_VELOCITY_LIMIT_VIOLATION", result["steps"][0]["frozen_bounds"]["reasons"])
        self.assertFalse(result["execution_authorized"])
        np.testing.assert_array_equal(result["retained_state"]["q14"], self.q)

    def test_invalid_requests_do_not_run_iterations(self):
        good = self.request()
        for request in (replace(good, max_steps=0), replace(good, max_steps=True),
                        replace(good, dt_s=0.), replace(good, dt_s=np.nan),
                        replace(good, cube_id="Cube_02"), replace(good, frame_id="map"),
                        replace(good, q14=None), replace(good, identity="")):
            with self.subTest(request=request):
                result = self.driver.run(request)
                self.assertEqual(result["status"], "INPUT_REJECTED")
                self.assertEqual(result["attempted_step_count"], 0)
                self.assertEqual(result["accepted_step_count"], 0)
                self.assertFalse(result["execution_authorized"])

    def test_unsupported_method_is_rejected(self):
        result = self.driver.run(self.request(), method="online_robot_velocity")
        self.assertEqual(result["status"], "INPUT_REJECTED")
        self.assertEqual(result["attempted_step_count"], 0)
        self.assertFalse(result["execution_authorized"])

    def test_original_payload_collision_is_rejected_before_any_integration(self):
        path = ROOT / "offline/task28_p4_nr_0/evidence/experiments_v1/negative_controls.json.gz"
        with gzip.open(path, "rt") as stream:
            saved = json.load(stream)
        X, q = next(row["node"] for row in saved if row["case"] == "payload_table_collision")
        request = replace(self.request(), q14=q, object_pose=X,
                          desired_object_twist=np.zeros(6), identity="saved-point-payload-negative")
        result = self.driver.run(request)
        self.assertEqual(result["status"], "STOPPED_AT_FIRST_REJECTION")
        self.assertEqual(result["attempted_step_count"], 0)
        self.assertFalse(result["initial_state"]["valid"])
        self.assertTrue(any("Cube_01" in reason or "PENETRATION" in reason
                            for reason in result["initial_state"]["reasons"]))
        self.assertEqual(result["accepted_step_count"], 0)
        # This is an archived single-state negative, not a new reachable path.
        self.assertFalse(result["execution_authorized"])

    def test_illustrative_acceleration_or_support_mode_switch_is_unsupported(self):
        configured = CentralizedVelocityQP(config=QPConfig(
            acceleration_caps_rad_s2=np.full(14, .1),
            acceleration_label="ILLUSTRATIVE_OFFLINE"))
        result = MultiStepDriver(controller=configured).run(self.request())
        self.assertEqual(result["status"], "INPUT_REJECTED")
        self.assertEqual(result["attempted_step_count"], 0)
        self.assertFalse(result["execution_authorized"])
        supported = self.driver.run(self.request(), site=SupportSite.SUPPLY)
        self.assertEqual(supported["status"], "INPUT_REJECTED")
        self.assertEqual(supported["attempted_step_count"], 0)


if __name__ == "__main__":
    unittest.main()
