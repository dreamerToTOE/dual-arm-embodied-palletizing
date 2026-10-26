"""Reproducible abstract QP review independent of the SLSQP implementation.

These controls have no robot, collision, timing, or execution interpretation.
The fixed seed/matrix generation and exhaustive 3**4 face search are the same
algorithm used in the supplemental read-only mathematical review.  Full input
matrices, reference solutions, residuals, and errors are printed as JSON so a
captured test log preserves the numerical inputs as well as the random seed.
"""
import itertools
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from offline.task28_p5_qp_0.solver import solve_box_equality_qp  # noqa: E402


def exhaustive_face_reference(H, f, C, lower, upper):
    """Find the feasible minimum over all 3**n box faces using linear algebra.

    Each coordinate is independently fixed at lower/upper, or left free.  For
    each face, solve its equality-constrained stationary equations directly.
    No SLSQP, HiGHS, bounded dual reconstruction, or production solver helper is
    used.  Enumerating every face includes the face containing the global SPD
    optimum; choosing the lowest feasible objective provides this small-case
    independent reference.  This is intentionally bounded to dimension four.
    """
    n = len(f)
    if n != 4 or C.shape != (1, n):
        raise ValueError("REFERENCE_SCOPE_FOUR_VARIABLES_ONE_EQUALITY")
    candidates = []
    for pattern in itertools.product((-1, 0, 1), repeat=n):
        free = np.array([i for i, value in enumerate(pattern) if value == 0], dtype=int)
        active = np.array([i for i, value in enumerate(pattern) if value != 0], dtype=int)
        x = np.zeros(n)
        for index in active:
            x[index] = lower[index] if pattern[index] == -1 else upper[index]
        if free.size:
            K = np.block([[H[np.ix_(free, free)], C[:, free].T],
                          [C[:, free], np.zeros((1, 1))]])
            rhs = np.r_[-f[free] - H[np.ix_(free, active)] @ x[active],
                        -C[:, active] @ x[active]]
            solved = np.linalg.lstsq(K, rhs, rcond=1e-13)[0]
            if np.linalg.norm(K @ solved - rhs, np.inf) > 1e-9:
                continue
            x[free] = solved[:free.size]
        if (np.linalg.norm(C @ x, np.inf) > 1e-9
                or np.any(x < lower - 1e-9) or np.any(x > upper + 1e-9)):
            continue
        objective = float(.5 * x @ H @ x + f @ x)
        candidates.append((objective, x, tuple(pattern)))
    if not candidates:
        raise AssertionError("Independent exhaustive reference found no feasible face")
    return min(candidates, key=lambda row: row[0]), len(candidates)


class IndependentOptimumTests(unittest.TestCase):
    def test_eight_fixed_seed_spd_qps_against_exhaustive_face_kkt_reference(self):
        rng = np.random.default_rng(2800)
        records = []
        for case in range(8):
            with self.subTest(case=case):
                n = 4
                A = rng.normal(size=(n, n))
                H = A.T @ A + .05 * np.eye(n)
                f = rng.normal(size=n) * 3
                C = rng.normal(size=(1, n))
                lower = -rng.uniform(.1, .5, n)
                upper = rng.uniform(.1, .5, n)
                result = solve_box_equality_qp(H, f, C, lower, upper)
                self.assertEqual(result.status, "QP_OPTIMAL")
                (objective, reference, pattern), feasible_faces = exhaustive_face_reference(
                    H, f, C, lower, upper)
                x = np.asarray(result.x)
                difference = float(np.max(np.abs(x - reference)))
                self.assertLess(difference, 1e-6)
                diag = result.diagnostics
                y = np.asarray(diag["dual_equality"])
                alpha = np.asarray(diag["dual_lower"])
                beta = np.asarray(diag["dual_upper"])
                # A separate Cholesky calculation derives the Lagrangian lower
                # bound; production uses a general linear solve instead.
                k = f + C.T @ y - alpha + beta
                L = np.linalg.cholesky(H)
                z = np.linalg.solve(L, k)
                dual_objective = -.5 * float(z @ z) + float(alpha @ lower - beta @ upper)
                objective_error = float(abs(diag["objective"] - objective))
                dual_error = float(abs(dual_objective - diag["dual_objective"]))
                self.assertLess(objective_error, 1e-8)
                self.assertLess(dual_error, 1e-10)
                self.assertGreaterEqual(float(min(alpha.min(), beta.min())), 0.)
                self.assertLess(float(np.linalg.norm(C @ x, np.inf)), 1e-8)
                self.assertTrue(np.all(x >= lower - 1e-8))
                self.assertTrue(np.all(x <= upper + 1e-8))
                records.append({
                    "case": case, "H": H.tolist(), "f": f.tolist(), "C": C.tolist(),
                    "lower": lower.tolist(), "upper": upper.tolist(),
                    "reference_face_pattern": pattern,
                    "reference_feasible_faces": feasible_faces,
                    "reference_x": reference.tolist(), "reference_objective": objective,
                    "candidate_x": x.tolist(), "candidate_objective": diag["objective"],
                    "dual_equality": y.tolist(), "dual_lower": alpha.tolist(),
                    "dual_upper": beta.tolist(), "independent_dual_objective": dual_objective,
                    "candidate_max_abs_error": difference,
                    "objective_abs_error": objective_error,
                    "independent_dual_objective_abs_error": dual_error,
                    "primal_dual_gap": diag["primal_dual_gap"],
                    "minimum_dual_multiplier": float(min(alpha.min(), beta.min())),
                    "original_equality_inf": float(np.linalg.norm(C @ x, np.inf)),
                })
        self.assertEqual(len(records), 8)
        print("\n" + json.dumps({"qp_independent_exhaustive_review": {
            "scope": "ABSTRACT_LINEAR_ALGEBRA_NOT_ROBOT_OR_PLANNING",
            "seed": 2800, "bit_generator": "numpy.default_rng:PCG64",
            "reference": "exhaustive 3**4 faces; equality KKT via numpy.linalg.lstsq",
            "cases": records,
            "max_candidate_abs_error": max(row["candidate_max_abs_error"] for row in records),
            "max_objective_abs_error": max(row["objective_abs_error"] for row in records),
            "max_dual_obj_abs_error": max(row["independent_dual_objective_abs_error"] for row in records),
            "execution_authorized": False,
        }}, sort_keys=True))

    def test_svd_rank_truncation_cannot_bypass_original_equality_gate(self):
        H = np.eye(2)
        f = np.array([0., -1.])
        C = np.diag([1e8, 1e-5])
        lower, upper = -2. * np.ones(2), 2. * np.ones(2)
        result = solve_box_equality_qp(H, f, C, lower, upper)
        self.assertEqual(result.status, "NUMERICAL_FAILURE")
        self.assertIsNone(result.x)
        self.assertEqual(result.diagnostics["equality_rank"], 1)
        self.assertGreater(result.diagnostics["max_equality_residual"],
                           result.diagnostics["primal_tolerance"])
        print("\n" + json.dumps({"qp_original_C_after_rank_truncation": {
            "scope": "ABSTRACT_LINEAR_ALGEBRA_NOT_ROBOT_OR_PLANNING",
            "H": H.tolist(), "f": f.tolist(), "C": C.tolist(),
            "lower": lower.tolist(), "upper": upper.tolist(),
            "status": result.status, "accepted_candidate": result.x,
            "reduced_equality_rank": result.diagnostics["equality_rank"],
            "original_C_residual_inf": result.diagnostics["max_equality_residual"],
            "primal_tolerance": result.diagnostics["primal_tolerance"],
            "reason": result.diagnostics["reason"], "execution_authorized": False,
        }}, sort_keys=True))


if __name__ == "__main__":
    unittest.main()
