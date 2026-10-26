"""URDF implementation versus independent frozen-USD and algebraic controls."""

import json
from pathlib import Path
import sys
import unittest

import numpy as np

MODULE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = MODULE_ROOT.parents[1]
sys.path.insert(0, str(MODULE_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from reference_fk import (  # noqa: E402
    analytic_zero_tcp, frozen_bounds, frozen_data, residuals, usd_fk,
    usd_link_frames,
)
from task28_validator.model import RobotModel  # noqa: E402

FIXTURE_DIR = REPO_ROOT / "fixtures/task28_g1_0_new_single_cube_v0"


class IndependentFKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel(FIXTURE_DIR)

    def test_zero_configuration_analytic_reference(self):
        # This is a kinematic algebra test; q=0 violates joint4/joint6 limits.
        for side in ("left", "right"):
            expected = analytic_zero_tcp(side)
            actual = self.model.fk(side, np.zeros(7))
            oracle = usd_fk(FIXTURE_DIR, side, np.zeros(7))
            for candidate in (actual, oracle):
                translation, rotation = residuals(candidate, expected)
                self.assertLess(translation, 1e-6)
                self.assertLess(rotation, 1e-6)

    def test_frozen_usd_vs_urdf_twenty_states_and_home(self):
        lower, upper = frozen_bounds(FIXTURE_DIR)
        _, scene, _ = frozen_data(FIXTURE_DIR)
        generator = np.random.default_rng(28)
        # Leave a fixed 5% margin from all bounds; reproducible on every run.
        states = [np.asarray(scene["home_q_rad"])]
        states.extend(lower + generator.uniform(0.05, 0.95, (20, 7)) *
                      (upper - lower))
        max_translation = 0.0
        max_rotation = 0.0
        for state_index, q in enumerate(states):
            frames = self.model.forward(q, q)
            for side in ("left", "right"):
                with self.subTest(state=state_index, side=side):
                    translation, rotation = residuals(
                        self.model.fk(side, q), usd_fk(FIXTURE_DIR, side, q)
                    )
                    max_translation = max(max_translation, translation)
                    max_rotation = max(max_rotation, rotation)
                    self.assertLess(translation, 1e-6)
                    self.assertLess(rotation, 1e-6)
                    for link, expected in usd_link_frames(FIXTURE_DIR, side, q).items():
                        translation, rotation = residuals(frames[link], expected)
                        self.assertLess(translation, 1e-6)
                        self.assertLess(rotation, 1e-6)
        print(json.dumps({
            "independent_fk": {
                "random_seed": 28,
                "joint_states": len(states),
                "tcp_comparisons": 2 * len(states),
                "max_translation_m": max_translation,
                "max_rotation_rad": max_rotation,
                "oracle": "frozen USD local joint frames; no USD runtime",
            }
        }, sort_keys=True))

    def test_rotation_control_cannot_pass_by_translation_only(self):
        identity = np.eye(4)
        half_turn = np.diag([1.0, -1.0, -1.0, 1.0])
        translation, rotation = residuals(half_turn, identity)
        self.assertEqual(translation, 0.0)
        self.assertAlmostEqual(rotation, np.pi, places=14)


if __name__ == "__main__":
    unittest.main()
