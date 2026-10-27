"""Independent stationary-equation checks for the equality-only reference.

These use saved FR3 states without planning or reprojection.  The expected
numerical reference is obtained from the full saddle KKT matrix, independently
of the production nullspace/Cholesky method.  No unconstrained command is
interpreted as executable or continuously collision safe.
"""
from dataclasses import replace
import gzip
import json
from pathlib import Path
import unittest

import numpy as np

from offline.task28_p5_qp_0 import CentralizedVelocityQP, VelocityInput
from offline.task28_p5_qp_1.driver import EqualityUnboundedReference

ROOT = Path(__file__).resolve().parents[3]


def saved_transport_node(seed=28):
    path = ROOT / f"offline/task28_p4_nr_0/evidence/experiments_v1/seed_{seed}/result.json.gz"
    with gzip.open(path, "rt") as stream:
        saved = json.load(stream)
    X, q = saved["transport"][len(saved["transport"]) // 2]
    return np.asarray(X), np.asarray(q)


def full_saddle_reference(B, C, W, regularization, desired):
    """Direct full KKT solve; no production solver, SVD or nullspace helper."""
    H = B.T @ W @ B + regularization * np.eye(B.shape[1])
    f = -B.T @ W @ desired
    K = np.block([[H, C.T], [C, np.zeros((C.shape[0], C.shape[0]))]])
    rhs = np.r_[-f, np.zeros(C.shape[0])]
    sol = np.linalg.lstsq(K, rhs, rcond=1e-14)[0]
    if np.linalg.norm(K @ sol - rhs, np.inf) > 1e-9:
        raise AssertionError("Independent full saddle system did not converge")
    return H, f, sol[:B.shape[1]], sol[B.shape[1]:]


class EqualityReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.controller = CentralizedVelocityQP()
        cls.reference = EqualityUnboundedReference(cls.controller)
        cls.X, cls.q = saved_transport_node()

    def request(self, twist):
        return VelocityInput(self.q.copy(), self.X.copy(), np.asarray(twist), .01)

    def test_frozen_fr3_reference_matches_independent_full_saddle_kkt(self):
        cases = ([.003, -.001, .001, .005, -.003, .01], [5., 0., 0., 0., 0., 0.])
        rows = []
        for desired in cases:
            with self.subTest(desired=desired):
                result = self.reference.solve(self.request(desired))
                self.assertIsNotNone(result.qdot)
                d = result.diagnostics
                B, C, W = (np.asarray(d[key]) for key in ("B", "C", "weight_matrix"))
                H, f, expected, dual = full_saddle_reference(
                    B, C, W, d["regularization"], np.asarray(desired))
                got = np.asarray(result.qdot)
                error = float(np.linalg.norm(got - expected, np.inf))
                self.assertLess(error, 2e-7)
                self.assertLess(float(np.linalg.norm(C @ got, np.inf)), 1e-8)
                self.assertLess(float(np.linalg.norm(H @ got + f + C.T @ dual, np.inf)), 1e-8)
                # Independently generate feasible nullspace perturbations by
                # solving C*z=C*r through the original full-row normal system.
                rng = np.random.default_rng(281001)
                perturbation_objective_increments = []
                for _ in range(5):
                    r = rng.normal(size=14)
                    z = r - C.T @ np.linalg.solve(C @ C.T, C @ r)
                    self.assertLess(float(np.linalg.norm(C @ z, np.inf)), 1e-9)
                    perturbed = got + .01 * z
                    delta = float(.5 * perturbed @ H @ perturbed + f @ perturbed
                                  - .5 * got @ H @ got - f @ got)
                    self.assertGreater(delta, 0.)
                    perturbation_objective_increments.append(delta)
                self.assertFalse(result.execution_authorized)
                rows.append({"desired": desired, "candidate": got.tolist(),
                             "full_saddle_reference": expected.tolist(),
                             "max_difference": error,
                             "full_C_equality_inf": float(np.linalg.norm(C @ got, np.inf)),
                             "feasible_perturbation_objective_increments": perturbation_objective_increments})
        print("\n" + json.dumps({"qp1_independent_full_saddle_reference": {
            "scope": "SAVED_FR3_POINT_NUMERICAL_REFERENCE_NOT_EXECUTABLE",
            "seed": 281001, "source_seed": 28, "source_segment": "transport",
            "rows": rows, "execution_authorized": False}}, sort_keys=True))

    def test_high_demand_reference_is_not_silently_bounded_or_clipped(self):
        request = self.request([5., 0., 0., 0., 0., 0.])
        bounded = self.controller.solve(request)
        reference = self.reference.solve(request)
        self.assertEqual(bounded.status, "FEASIBLE_TRACKING_REJECTED")
        self.assertIsNotNone(reference.qdot)
        x = np.asarray(reference.qdot)
        self.assertGreater(float(np.max(np.abs(x) - self.controller.velocity)), 0.)
        self.assertGreater(float(np.linalg.norm(x - bounded.qdot)), 1.)
        self.assertFalse(reference.execution_authorized)

    def test_invalid_or_unsupported_input_emits_no_unbounded_candidate(self):
        good = self.request(np.zeros(6))
        for request in (replace(good, cube_id="Cube_02"), replace(good, frame_id="map"),
                        replace(good, q14=None), replace(good, dt_s=0.)):
            with self.subTest(request=request):
                result = self.reference.solve(request)
                self.assertEqual(result.status, "INPUT_REJECTED")
                self.assertIsNone(result.qdot)
                self.assertFalse(result.execution_authorized)


if __name__ == "__main__":
    unittest.main()
