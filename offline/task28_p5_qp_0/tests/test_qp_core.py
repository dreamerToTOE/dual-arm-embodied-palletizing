"""Independent mathematical QP controls and frozen-model contract controls.

Abstract matrices below test optimization only; they are not robot-feasibility
or collision controls. Actual robot states use archived NR-0 boundary samples.
"""
import gzip
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from scipy.optimize import minimize

from offline.task28_p5_qp_0 import (CentralizedVelocityQP, PreviousCommand, QPConfig,
                                   VelocityInput, solve_box_equality_qp)
from offline.task28_p5_qp_0.bootstrap import RobotModel, nominal_grasps, transform

REPO = Path(__file__).resolve().parents[3]


class AbstractQPControls(unittest.TestCase):
    def test_unconstrained_analytic_minimizer(self):
        H = np.array([[3., .5], [.5, 2.]])
        f = np.array([-.3, .4])
        got = solve_box_equality_qp(H, f, np.empty((0, 2)), [-2., -2.], [2., 2.])
        self.assertEqual(got.status, "QP_OPTIMAL")
        np.testing.assert_allclose(got.x, -np.linalg.solve(H, f), atol=1e-7)
        self.assertLess(got.diagnostics["kkt_stationarity_inf"], 1e-7)

    def test_central_equality_not_independent_arm_solves(self):
        # One equality couples both variables. Independent box solves give (1,0)
        # and violate equality; the centralized mathematical optimum is (.5,.5).
        got = solve_box_equality_qp(np.eye(2), [-1., 0.], [[1., -1.]], [-2., -2.], [2., 2.])
        self.assertEqual(got.status, "QP_OPTIMAL")
        np.testing.assert_allclose(got.x, [.5, .5], atol=1e-8)
        self.assertGreater(np.linalg.norm(np.array([[1., -1.]]) @ [1., 0.]), .9)

    def test_active_velocity_bound_kkt(self):
        got = solve_box_equality_qp(np.eye(2), [-3., 2.], np.empty((0, 2)), [-1., -1.], [1., 1.])
        self.assertEqual(got.status, "QP_OPTIMAL")
        np.testing.assert_allclose(got.x, [1., -1.], atol=1e-9)
        self.assertGreater(got.diagnostics["dual_upper"][0], 1.9)
        self.assertGreater(got.diagnostics["dual_lower"][1], .9)

    def test_equality_infeasible_box_not_tracking_miss(self):
        got = solve_box_equality_qp(np.eye(2), [0., 0.], [[1., -1.]], [1., -2.], [2., -1.])
        self.assertEqual(got.status, "QP_INFEASIBLE")
        self.assertIsNone(got.x)
        self.assertEqual(got.diagnostics["feasibility_exit_status"], 2)

    def test_empty_box(self):
        got = solve_box_equality_qp(np.eye(2), [0., 0.], [[1., -1.]], [1., -1.], [0., 1.])
        self.assertEqual(got.status, "QP_INFEASIBLE")
        self.assertEqual(got.diagnostics["reason"], "EMPTY_BOX_INTERSECTION")

    def test_full_original_redundant_equality_checked(self):
        C = np.array([[1., -1.], [2., -2.], [0., 0.]])
        got = solve_box_equality_qp(np.eye(2), [-1., 0.], C, [-2., -2.], [2., 2.])
        self.assertEqual(got.status, "QP_OPTIMAL")
        self.assertEqual(got.diagnostics["equality_rank"], 1)
        np.testing.assert_allclose(C @ got.x, np.zeros(3), atol=1e-9)
        self.assertEqual(len(got.diagnostics["equality_residual"]), 3)

    def test_rank_zero_feasible_zero_is_not_infeasible(self):
        got = solve_box_equality_qp(1e-6 * np.eye(14), np.zeros(14), np.zeros((6, 14)), -np.ones(14), np.ones(14))
        self.assertEqual(got.status, "QP_OPTIMAL")
        np.testing.assert_allclose(got.x, np.zeros(14), atol=1e-8)
        self.assertEqual(got.diagnostics["equality_rank"], 0)

    def test_small_spd_regularization_not_zeroed(self):
        H = np.diag([1e-6, 1.])
        got = solve_box_equality_qp(H, [-1e-7, -.2], np.empty((0, 2)), [-1., -1.], [1., 1.])
        self.assertEqual(got.status, "QP_OPTIMAL")
        np.testing.assert_allclose(got.x, [.1, .2], atol=1e-4)

    def test_near_limit_lookahead_bound_dominates_velocity_bound(self):
        # Abstract14d control: joint0 is10microradians from its upper limit.
        # The position-derived upper rate is1e-3rad/s atdt=.01, stricter than
        # the otherwise1rad/s box. This is not a reachable robot state claim.
        q = np.zeros(14); q[0] = 1. - 1e-5
        lower = np.maximum(-np.ones(14), (-np.ones(14)-q)/.01)
        upper = np.minimum(np.ones(14), (np.ones(14)-q)/.01)
        got = solve_box_equality_qp(np.eye(14), -np.ones(14), np.empty((0,14)), lower, upper)
        self.assertEqual(got.status, 'QP_OPTIMAL')
        self.assertAlmostEqual(got.x[0], .001, places=10)
        self.assertLessEqual(q[0]+.01*got.x[0], 1.+1e-12)
        self.assertIn(0, got.diagnostics['active_upper'])

    def test_acceleration_intersection_can_be_genuinely_infeasible(self):
        # Abstract2d compatibility needs equal rates. Disjoint previous-rate
        # acceleration intervals exclude it, although each individual box exists.
        got = solve_box_equality_qp(np.eye(2), [0.,0.], [[1.,-1.]], [.9,-1.1], [1.1,-.9])
        self.assertEqual(got.status, 'QP_INFEASIBLE')
        self.assertIsNone(got.x)

    def test_numerical_backend_exceptions_return_no_candidate(self):
        # These are injected software/backend failures, not physical failures.
        for target, error in (("np.linalg.svd", np.linalg.LinAlgError("injected SVD failure")),
                              ("linprog", RuntimeError("injected HiGHS failure")),
                              ("minimize", ValueError("injected SLSQP backend failure"))):
            with self.subTest(backend=target):
                with patch('offline.task28_p5_qp_0.solver.' + target, side_effect=error):
                    got = solve_box_equality_qp(np.eye(2), [0.,0.], [[1.,-1.]], [-1.,-1.], [1.,1.])
                self.assertEqual(got.status, 'NUMERICAL_FAILURE')
                self.assertIsNone(got.x)
                self.assertEqual(got.diagnostics['reason'], 'NUMERICAL_BACKEND_EXCEPTION')
                self.assertFalse(got.diagnostics['execution_authorized'])

    def test_nonfinite_optimizer_candidate_rejected(self):
        from scipy.optimize import OptimizeResult
        fake = OptimizeResult(x=np.array([np.nan,0.]), success=True, status=0,
                              message='injected nonfinite backend result', nit=1, nfev=1)
        with patch('offline.task28_p5_qp_0.solver.minimize', return_value=fake):
            got = solve_box_equality_qp(np.eye(2), [0.,0.], [[1.,-1.]], [-1.,-1.], [1.,1.])
        self.assertEqual(got.status, 'NUMERICAL_FAILURE')
        self.assertIsNone(got.x)
        self.assertEqual(got.diagnostics['reason'], 'NONFINITE_OPTIMIZER_RESULT')

    def test_independent_kkt_basis_remains_explicit_on_optimizer_flag(self):
        from scipy.optimize import OptimizeResult
        # Abstract exact zero optimum; non-success status alone is not a
        # mathematical disproof of independently checked convex-QP KKT.
        fake = OptimizeResult(x=np.zeros(2), success=False, status=9,
                              message='injected iteration limit at exact optimum', nit=1, nfev=1)
        with patch('offline.task28_p5_qp_0.solver.minimize', return_value=fake):
            got = solve_box_equality_qp(np.eye(2), [0.,0.], [[1.,-1.]], [-1.,-1.], [1.,1.])
        self.assertEqual(got.status, 'QP_OPTIMAL')
        self.assertFalse(got.diagnostics['optimizer_success'])
        self.assertTrue(got.diagnostics['accepted_despite_optimizer_non_success'])
        self.assertEqual(got.diagnostics['acceptance_basis'], 'INDEPENDENT_PRIMAL_DUAL_KKT_NOT_OPTIMIZER_STATUS')

    def test_bad_dual_sign_rejected_even_when_primal_and_stationarity_pass(self):
        from scipy.optimize import OptimizeResult
        from types import SimpleNamespace
        # Abstract injected backend fault: a fixed variable has both active
        # bounds, so equal negative multipliers cancel in stationarity and the
        # objective gap. Explicit dual-sign verification must still reject it.
        optimizer = OptimizeResult(x=np.zeros(2), success=True, status=0,
                                   message='injected exact primal', nit=1, nfev=1)
        bad_dual = SimpleNamespace(x=np.array([-1., -1.]), success=True)
        with patch('offline.task28_p5_qp_0.solver.minimize', return_value=optimizer), patch('offline.task28_p5_qp_0.solver.lsq_linear', return_value=bad_dual):
            got = solve_box_equality_qp(np.eye(2), [0.,0.], np.empty((0,2)), [0.,-1.], [0.,1.])
        self.assertEqual(got.status, 'NUMERICAL_FAILURE')
        self.assertIsNone(got.x)
        self.assertEqual(got.diagnostics['max_dual_sign_violation'], 1.)
        self.assertEqual(got.diagnostics['kkt_stationarity_inf'], 0.)
        self.assertEqual(got.diagnostics['max_equality_residual'], 0.)
        self.assertTrue(got.diagnostics['dual_all_finite'])
        self.assertFalse(got.diagnostics['independent_kkt_verified'])

    def test_bad_matrix_contract(self):
        for H in (np.zeros((2, 2)), [[1., 1.], [0., 1.]], [[1., 0.], [0., np.nan]]):
            with self.assertRaises(ValueError):
                solve_box_equality_qp(H, [0., 0.], [[1., -1.]], [-1., -1.], [1., 1.])


class FrozenRobotQPControls(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()
        cls.controller = CentralizedVelocityQP(cls.model)
        # Directly preserved, valid supported grasp q from NR-0 evidence; no new
        # planning or IK is executed by these numerical unit controls.
        data = json.loads((REPO / 'offline/task28_p4_nr_0/evidence/path_exports/path_receipts.json').read_text())
        cls.q = np.asarray(next(row for row in data if row['seed'] == 28)['start_q14'])
        cls.obj = transform(translation=(.65, 0., .26))

    def request(self, **kwargs):
        values = dict(q14=self.q, object_pose=self.obj, desired_object_twist=np.zeros(6), dt_s=.01)
        values.update(kwargs)
        return VelocityInput(**values)

    def test_zero_twist_frozen_model(self):
        got = self.controller.solve(self.request())
        self.assertEqual(got.status, 'FEASIBLE_TRACKING_OK')
        np.testing.assert_allclose(got.qdot, np.zeros(14), atol=1e-7)
        self.assertFalse(got.execution_authorized)
        self.assertTrue(got.diagnostics['solver']['independent_kkt_verified'])

    def test_small_translation_full_centralized_qp(self):
        got = self.controller.solve(self.request(desired_object_twist=[.002, -.001, .001, 0., 0., 0.]))
        self.assertEqual(got.status, 'FEASIBLE_TRACKING_OK')
        self.assertEqual(len(got.qdot), 14)
        self.assertLess(np.linalg.norm(got.diagnostics['closure_velocity_residual']), 1e-8)
        self.assertFalse(got.diagnostics['collision_checked_in_core'])

    def test_huge_task_is_soft_rejected_not_infeasible(self):
        got = self.controller.solve(self.request(desired_object_twist=[100., 0., 0., 0., 0., 0.]))
        self.assertEqual(got.status, 'FEASIBLE_TRACKING_REJECTED')
        self.assertIsNotNone(got.qdot)
        self.assertGreater(got.diagnostics['tracking_translation_error_m_s'], 90.)

    def test_wrong_grasp_cannot_change_mapping(self):
        grasp = nominal_grasps(0.)
        got = self.controller.solve(self.request(grasps=grasp))
        self.assertEqual(got.status, 'INPUT_REJECTED')
        self.assertIn('FROZEN_GRASP_MISMATCH', got.reasons)

    def test_wrong_pose_rejected_by_existing_closure(self):
        obj = self.obj.copy(); obj[0, 3] += .01
        got = self.controller.solve(self.request(object_pose=obj))
        self.assertEqual(got.status, 'INPUT_REJECTED')
        self.assertIn('INITIAL_SE3_CLOSURE_REJECTED', got.reasons)

    def test_input_contract_negative_values(self):
        cases = [dict(q14=None), dict(q14=[0.] * 13), dict(q14=[np.nan] * 14),
                 dict(desired_object_twist=[np.inf] * 6), dict(dt_s=0.), dict(dt_s=True),
                 dict(cube_id='Cube_02'), dict(frame_id='map'), dict(object_pose=np.zeros((4,4)))]
        for values in cases:
            with self.subTest(values=values):
                got = self.controller.solve(self.request(**values))
                self.assertEqual(got.status, 'INPUT_REJECTED')
                self.assertIsNone(got.qdot)

    def test_position_violation_before_optimization(self):
        q = self.q.copy(); q[0] = self.controller.upper[0] + .01
        got = self.controller.solve(self.request(q14=q))
        self.assertEqual(got.status, 'INPUT_REJECTED')
        self.assertIn('INITIAL_JOINT_LIMIT_VIOLATION', got.reasons)

    def test_frozen_velocity_limits_hash_and_urdf_match(self):
        self.assertTrue(self.controller.binding['urdf_velocity_equality_verified_both_arms'])
        np.testing.assert_array_equal(self.controller.velocity[:7], self.controller.velocity[7:])
        self.assertEqual(len(self.controller.binding['effective_joint_limits_sha256']), 64)
        self.assertEqual(self.controller.velocity[0], 2.619999770924542)

    def test_acceleration_disabled_default(self):
        got = self.controller.solve(self.request())
        self.assertFalse(got.diagnostics['acceleration']['enabled'])
        self.assertFalse(got.diagnostics['acceleration']['physical_limits_authenticated'])
        got = self.controller.solve(self.request(previous=PreviousCommand(np.zeros(14), .01, 'control', True)))
        self.assertEqual(got.status, 'INPUT_REJECTED')

    def test_acceleration_missing_previous_and_bad_label(self):
        with self.assertRaises(ValueError):
            QPConfig(acceleration_caps_rad_s2=np.ones(14))
        controller = CentralizedVelocityQP(self.model, QPConfig(acceleration_caps_rad_s2=np.ones(14), acceleration_label='ILLUSTRATIVE_OFFLINE'))
        got = controller.solve(self.request())
        self.assertEqual(got.status, 'INPUT_REJECTED')
        self.assertIn('MISSING_OR_INVALID_PREVIOUS_COMMAND', got.reasons)

    def test_illustrative_acceleration_checked_previous_contract(self):
        ctrl = CentralizedVelocityQP(self.model, QPConfig(acceleration_caps_rad_s2=np.ones(14), acceleration_label='ILLUSTRATIVE_OFFLINE'))
        for prev in (PreviousCommand(np.zeros(14), .02, 'id', True), PreviousCommand(np.zeros(14), .01, '', True),
                     PreviousCommand(np.zeros(14), .01, 'id', False), PreviousCommand(np.ones(14)*100., .01, 'id', True),
                     PreviousCommand(np.zeros(14), 0., 'id', True)):
            self.assertEqual(ctrl.solve(self.request(previous=prev)).status, 'INPUT_REJECTED')
        got = ctrl.solve(self.request(previous=PreviousCommand(np.zeros(14), .01, 'synthetic-numeric-0', True)))
        self.assertEqual(got.status, 'FEASIBLE_TRACKING_OK')
        self.assertFalse(got.diagnostics['acceleration']['previous_geometric_validation_claimed'])

    def test_finite_extreme_task_reports_numerical_failure(self):
        got = self.controller.solve(self.request(desired_object_twist=[1e308]*6))
        self.assertEqual(got.status, 'NUMERICAL_FAILURE')
        self.assertIsNone(got.qdot)
        self.assertFalse(got.execution_authorized)

    def test_solver_exception_contained_by_typed_core(self):
        with patch('offline.task28_p5_qp_0.core.solve_box_equality_qp', side_effect=RuntimeError('injected software failure')):
            got = self.controller.solve(self.request())
        self.assertEqual(got.status, 'NUMERICAL_FAILURE')
        self.assertIsNone(got.qdot)
        self.assertEqual(got.diagnostics['exception_type'], 'RuntimeError')
        self.assertFalse(got.execution_authorized)

    def test_nonfinite_or_raising_jacobian_is_numerical_failure(self):
        # Synthetic backend controls; no robot instability/measurement claimed.
        for failure in (np.full((6,7), np.nan), RuntimeError('injected Jacobian backend failure')):
            with self.subTest(failure=type(failure).__name__):
                kwargs = {'side_effect':failure} if isinstance(failure,Exception) else {'return_value':failure}
                with patch.object(self.model, 'jacobian', **kwargs):
                    got = self.controller.solve(self.request())
                self.assertEqual(got.status, 'NUMERICAL_FAILURE')
                self.assertIsNone(got.qdot)
                self.assertFalse(got.execution_authorized)

    def test_mutated_acceleration_configuration_is_revalidated(self):
        caps = np.ones(14)
        ctrl = CentralizedVelocityQP(self.model, QPConfig(acceleration_caps_rad_s2=caps, acceleration_label='ILLUSTRATIVE_OFFLINE'))
        caps[0] = -1.
        got = ctrl.solve(self.request(previous=PreviousCommand(np.zeros(14), .01, 'numeric-control', True)))
        self.assertEqual(got.status, 'INPUT_REJECTED')
        self.assertIsNone(got.qdot)
        self.assertIn('UNAUTHENTICATED_ACCELERATION_CONFIGURATION', got.reasons)

    def test_rank_zero_jacobian_tracking_rejects(self):
        original = self.model.jacobian
        try:
            self.model.jacobian = lambda side, q: np.zeros((6, 7))
            got = self.controller.solve(self.request(desired_object_twist=[.01, 0., 0., 0., 0., 0.]))
        finally:
            self.model.jacobian = original
        self.assertEqual(got.status, 'FEASIBLE_TRACKING_REJECTED')
        self.assertEqual(got.diagnostics['solver']['equality_rank'], 0)
        np.testing.assert_allclose(got.qdot, np.zeros(14), atol=1e-8)


if __name__ == '__main__':
    unittest.main()
