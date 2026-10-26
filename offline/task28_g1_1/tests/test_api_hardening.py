"""Acceptance bypass controls for public numeric and policy entry points."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from task28_validator.closed_chain import ClosurePolicy, desired_tcps, evaluate_closure
from task28_validator.model import RobotModel
from task28_validator.se3 import checked_transform, pose_matrix, transform
from task28_validator.state import StatePolicy
from task28_validator.validator import ClosedChainValidator


class APIHardeningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()

    def test_nan_policy_cannot_disable_closure_comparisons(self):
        # Every real residual compares false against NaN. Reject the policy
        # before evaluating either an exact grasp or this grossly wrong grasp.
        obj = np.eye(4)
        left = np.eye(4)
        left[:3, 3] = [1.0, 2.0, 3.0]
        right = np.eye(4)
        right[:3, 3] = [-1.0, 0.0, 0.0]
        fake = SimpleNamespace(translation_tolerance_m=float("nan"),
                               rotation_tolerance_rad=float("nan"))
        with self.assertRaises(ValueError):
            evaluate_closure(obj, left, right, policy=fake)

    def test_tampered_policy_is_rechecked_at_closure_boundary(self):
        # frozen dataclasses protect ordinary assignment, but acceptance gates
        # must still verify values if a foreign caller reconstructs an object.
        obj = np.eye(4)
        left, right = desired_tcps(obj)
        for value in (float("nan"), float("inf"), -1.0):
            policy = ClosurePolicy()
            object.__setattr__(policy, "translation_tolerance_m", value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                evaluate_closure(obj, left, right, policy=policy)

    def test_high_level_constructor_rejects_injected_bad_policies(self):
        fake_closure = SimpleNamespace(translation_tolerance_m=float("nan"),
                                       rotation_tolerance_rad=float("nan"))
        fake_state = SimpleNamespace(max_age_s=100.0, max_skew_s=100.0,
                                     max_future_s=100.0, quaternion_norm_tolerance=1.0)
        with self.assertRaises(ValueError):
            ClosedChainValidator(self.model, object(), closure_policy=fake_closure)
        with self.assertRaises(ValueError):
            ClosedChainValidator(self.model, object(), state_policy=fake_state)
        with self.assertRaises(ValueError):
            ClosedChainValidator(self.model, object(), state_policy=StatePolicy(max_age_s=1.0))
        tampered = ClosurePolicy()
        object.__setattr__(tampered, "rotation_tolerance_rad", float("nan"))
        with self.assertRaises(ValueError):
            ClosedChainValidator(self.model, object(), closure_policy=tampered)

    def test_complex_joint_values_are_not_silently_projected_to_real(self):
        for q in (self.model.home.astype(complex), self.model.home.astype(complex) + 1j):
            with self.subTest(q=q):
                self.assertFalse(self.model.within_limits(q))
                with self.assertRaises(ValueError):
                    self.model.fk("left", q)
                with self.assertRaises(ValueError):
                    self.model.forward(q, self.model.home)

    def test_complex_transforms_and_poses_are_rejected(self):
        matrix = np.eye(4, dtype=complex)
        matrix[0, 3] = 1j
        with self.assertRaises(ValueError):
            checked_transform(matrix)
        for position, quaternion in (([1j, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
                                     ([0.0, 0.0, 0.0], [0j, 0.0, 0.0, 1.0])):
            with self.subTest(position=position, quaternion=quaternion), self.assertRaises(ValueError):
                pose_matrix(position, quaternion)

    def test_transform_dimensions_cannot_broadcast(self):
        for translation in (1.0, [1.0], [[1.0, 2.0, 3.0]]):
            with self.subTest(translation=translation), self.assertRaises(ValueError):
                transform(translation=translation)
        for rotation in (1.0, [1.0, 0.0, 0.0], np.eye(4)):
            with self.subTest(rotation=rotation), self.assertRaises(ValueError):
                transform(rotation=rotation)


if __name__ == "__main__":
    unittest.main()
