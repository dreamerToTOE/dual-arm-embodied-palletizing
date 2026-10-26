"""Independent numerical controls for the new NR projector, CPU only."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "offline"))
sys.path.insert(0, str(ROOT / "offline/task28_g1_1"))
from task28_p4_nr_0.projection import NRProjector
from task28_validator.closed_chain import nominal_grasps, evaluate_closure
from task28_validator.model import RobotModel
from task28_validator.se3 import transform, rpy_matrix, residual


_oracle_spec = importlib.util.spec_from_file_location(
    "nr_projection_independent_usd_fk", ROOT / "offline/task28_g1_1/tests/reference_fk.py")
_oracle = importlib.util.module_from_spec(_oracle_spec)
_oracle_spec.loader.exec_module(_oracle)


def independent_residual(model, obj, q):
    """USD elementary FK + explicit Rodrigues log, not projector FK/log/J."""
    values = []
    grasps = nominal_grasps()
    for i, side in enumerate(("left", "right")):
        actual = _oracle.usd_fk(model.fixture_dir, side, q[7*i:7*(i+1)])
        target = obj @ getattr(grasps, side)
        rotation = actual[:3, :3] @ target[:3, :3].T
        theta = math.acos(float(np.clip((np.trace(rotation)-1)/2., -1., 1.)))
        axial = np.array([rotation[2, 1]-rotation[1, 2], rotation[0, 2]-rotation[2, 0],
                          rotation[1, 0]-rotation[0, 1]])/2.
        phi = axial if theta < 1e-7 else theta/math.sin(theta)*axial
        values.extend(actual[:3, 3]-target[:3, 3])
        values.extend(phi)
    return np.asarray(values)


class ProjectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()
        cls.home14 = np.tile(cls.model.home, 2)
        cls.controls = json.loads((ROOT / "offline/task28_g1_1/tests/control_states.json").read_text())["states"]

    def test_analytic_log_jacobian_against_independent_usd_central_differences(self):
        projector = NRProjector(self.model)
        q = np.r_[[.3, -.7, .5, -2.2, .6, 1.4, .4],
                  [-.2, -.8, -.4, -2.1, -.5, 1.5, -.4]]
        obj = transform(rpy_matrix(.2, -.1, .15), (.65, .01, .43))
        values, analytic = projector.residual_jacobian(obj, q)
        # The preserved USD oracle has float32-derived translation constants;
        # the existing independent-FK contract permits 1e-6 m/rad.  This tighter
        # 3e-8 control accommodates that known serialization floor, while the
        # production acceptance remains exactly 1e-6 m / 1e-5 rad.
        np.testing.assert_allclose(values, independent_residual(self.model, obj, q), atol=3e-8, rtol=0)
        h = 1e-6
        numerical = np.empty((12, 14))
        for k in range(14):
            delta = np.zeros(14)
            delta[k] = h
            numerical[:, k] = (independent_residual(self.model, obj, q+delta)
                               - independent_residual(self.model, obj, q-delta))/(2*h)
        self.assertLess(float(np.max(np.abs(analytic-numerical))), 3e-8)
        np.testing.assert_array_equal(analytic[:6, 7:], np.zeros((6, 7)))
        np.testing.assert_array_equal(analytic[6:, :7], np.zeros((6, 7)))

    def test_svd_update_matches_independent_full_row_rank_normal_equations(self):
        row = self.controls[0]
        obj = transform(translation=row["object_position_m"])
        seed = np.r_[row["left_q"], row["right_q"]] + .003*np.linspace(-1., 1., 14)
        projector = NRProjector(self.model, max_iterations=1, max_step_rad=math.pi)
        values, jacobian = projector.residual_jacobian(obj, seed)
        # A different mathematical computation of the minimum-norm Newton step.
        reference = -jacobian.T @ np.linalg.solve(jacobian @ jacobian.T, values)
        result = projector.project(obj, seed)
        first = result.trace[0]
        np.testing.assert_allclose(first["uncapped_newton_direction14_rad"], reference, atol=3e-14, rtol=0)
        self.assertEqual(first["numerical_rank"], 12)
        self.assertEqual(first["global_l2_step_cap_scale"], 1.)
        self.assertEqual(first["trials"][0]["reason"], "ACCEPTED")
        np.testing.assert_allclose(first["q14_after"], seed+reference, atol=3e-14, rtol=0)
        self.assertGreater(np.linalg.norm(reference[:7]), 1e-5)
        self.assertGreater(np.linalg.norm(reference[7:]), 1e-5)

    def test_cold_home_projection_passes_independent_fk_and_shared_object_closure(self):
        projector = NRProjector(self.model)
        cases = [transform(translation=(.65, 0., .4)),
                 transform(translation=(.7, -.025, .45)),
                 transform(rpy_matrix(0., 0., .15), (.65, 0., .45))]
        grasps = nominal_grasps()
        with patch.object(self.model, "ik", side_effect=AssertionError("NR must not call old IK")):
            for obj in cases:
                with self.subTest(object=obj.tolist()):
                    result = projector.project(obj, self.home14)
                    self.assertTrue(result.success, result.reason)
                    self.assertEqual(result.reason, "NR_CONVERGED")
                    self.assertLessEqual(result.iterations, 60)
                    self.assertGreaterEqual(result.minimum_joint_limit_margin_rad, 0.)
                    tcps = [_oracle.usd_fk(self.model.fixture_dir, side, result.q14[7*i:7*(i+1)])
                            for i, side in enumerate(("left", "right"))]
                    for side, tcp in zip(("left", "right"), tcps):
                        pos, rot = residual(tcp, obj @ getattr(grasps, side))
                        self.assertLessEqual(pos, 1e-6)
                        self.assertLessEqual(rot, 1e-5)
                    self.assertTrue(evaluate_closure(obj, *tcps).valid)

    def test_same_seed_is_deterministic_without_random_sampling(self):
        projector = NRProjector(self.model)
        obj = transform(translation=(.65, 0., .4))
        first = projector.project(obj, self.home14)
        second = projector.project(obj, self.home14)
        self.assertEqual(first, second)

    def test_already_converged_control_needs_zero_updates(self):
        row = self.controls[0]
        result = NRProjector(self.model).project(
            transform(translation=row["object_position_m"]), np.r_[row["left_q"], row["right_q"]])
        self.assertTrue(result.success)
        self.assertEqual(result.iterations, 0)
        self.assertEqual(result.trace, ())

    def test_iteration_budget_is_hard_and_preserves_last_candidate(self):
        result = NRProjector(self.model, max_iterations=1).project(
            transform(translation=(.65, 0., .4)), self.home14)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_ITERATION_LIMIT")
        self.assertEqual(result.iterations, 1)
        self.assertEqual(len(result.trace), 1)
        self.assertEqual(result.q14, tuple(result.trace[-1]["q14_after"]))

    def test_global_step_cap_and_all_accepted_proposals_preserve_frozen_limits(self):
        cap = .03
        result = NRProjector(self.model, max_iterations=3, max_step_rad=cap).project(
            transform(translation=(.65, 0., .4)), self.home14)
        self.assertFalse(result.success)
        for step in result.trace:
            self.assertLessEqual(np.linalg.norm(step["accepted_delta14_rad"]), cap+1e-14)
            q = step["q14_after"]
            self.assertTrue(self.model.within_limits(q[:7]))
            self.assertTrue(self.model.within_limits(q[7:]))
            accepted = step["trials"][-1]
            self.assertEqual(accepted["reason"], "ACCEPTED")
            self.assertLess(accepted["residual_norm_after"], step["residual_norm_before"])

    def test_limit_violation_seed_rejected_without_repair(self):
        seed = self.home14.copy()
        seed[0] = self.model.joint_upper[0]+.001
        result = NRProjector(self.model).project(transform(translation=(.65, 0., .4)), seed)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_SEED_JOINT_LIMIT_VIOLATION")
        self.assertEqual(result.q14, tuple(seed))
        self.assertEqual(result.trace, ())

    def test_outward_boundary_newton_proposal_rejected_without_clipping(self):
        seed = np.tile(self.model.joint_lower, 2)
        result = NRProjector(self.model, backtracks=0).project(
            transform(translation=(.65, 0., .4)), seed)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_UPDATE_JOINT_LIMIT_VIOLATION")
        self.assertEqual(result.q14, tuple(seed))
        self.assertEqual(result.iterations, 0)
        self.assertEqual(len(result.trace[0]["trials"]), 1)
        self.assertLess(result.trace[0]["trials"][0]["minimum_joint_limit_margin_rad"], 0.)

    def test_unreachable_goal_rejected_by_necessary_reach_bound(self):
        result = NRProjector(self.model).project(transform(translation=(100., 0., 0.)), self.home14)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_TARGET_OUTSIDE_REACH_BOUND")
        self.assertEqual(result.iterations, 0)
        self.assertEqual(result.trace, ())
        self.assertTrue(math.isfinite(result.translation_error_m))

    def test_so3_principal_log_branch_cut_is_explicitly_rejected(self):
        actual = self.model.fk("left", self.model.home)
        desired_tcp_rotation = actual[:3, :3] @ rpy_matrix(0., 0., math.pi)
        obj_rotation = desired_tcp_rotation @ nominal_grasps().left[:3, :3].T
        result = NRProjector(self.model).project(transform(obj_rotation, (.65, 0., .4)), self.home14)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_SO3_PRINCIPAL_CHART_NEAR_PI")
        self.assertEqual(result.iterations, 0)

    def test_bad_q_and_object_inputs_rejected(self):
        projector = NRProjector(self.model)
        obj = transform(translation=(.65, 0., .4))
        for bad in ([0.]*13, [True]*14, [complex(0, 1)]*14, [np.nan]*14, [np.inf]*14, ["0"]*14):
            with self.subTest(q=repr(bad)), self.assertRaises(ValueError):
                projector.project(obj, bad)
        for bad in (np.eye(3), np.zeros((4, 4)), np.full((4, 4), np.nan),
                    transform(translation=(0., 0., 0.))*2):
            with self.subTest(object=repr(bad)), self.assertRaises(ValueError):
                projector.project(bad, self.home14)

    def test_invalid_budgets_rejected_before_work(self):
        for kwargs in ({"max_iterations": 0}, {"max_iterations": True}, {"max_iterations": 2001},
                       {"backtracks": -1}, {"backtracks": True}, {"backtracks": 21},
                       {"max_step_rad": 0}, {"max_step_rad": True}, {"max_step_rad": np.nan},
                       {"relative_svd_cutoff": 0}, {"relative_svd_cutoff": 1},
                       {"relative_svd_cutoff": True}, {"relative_svd_cutoff": np.inf}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                NRProjector(self.model, **kwargs)

    def test_nonfinite_model_jacobian_and_svd_fail_closed(self):
        projector = NRProjector(self.model)
        obj = transform(translation=(.65, 0., .4))
        with patch.object(self.model, "jacobian", return_value=np.full((6, 7), np.nan)):
            result = projector.project(obj, self.home14)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_INVALID_MODEL_JACOBIAN")
        with patch("task28_p4_nr_0.projection.np.linalg.svd", side_effect=np.linalg.LinAlgError("mock")):
            result = projector.project(obj, self.home14)
        self.assertFalse(result.success)
        self.assertEqual(result.reason, "NR_SVD_FAILURE")


if __name__ == "__main__":
    unittest.main()
