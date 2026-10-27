"""Independent active-face reference and failure controls for affine QPs."""
import itertools
import json
from types import SimpleNamespace

import numpy as np
import pytest

from offline.task28_p5_qp_2 import affine_solver


def enumerate_faces(H, f, A, b, lower, upper):
    """Independent exhaustive face enumeration, used only for small test QPs.

    For each of 3^n faces, solve the equality-constrained stationary equations
    on its free coordinates and independently inspect the signs of its bound
    derivatives. This reference does not call the production QP solver.
    """
    n = len(f)
    best = None
    for pattern in itertools.product((-1, 0, 1), repeat=n):
        active = np.array([j for j, mode in enumerate(pattern) if mode], dtype=int)
        free = np.array([j for j, mode in enumerate(pattern) if not mode], dtype=int)
        if not free.size:
            continue
        x = np.zeros(n)
        for j in active:
            x[j] = lower[j] if pattern[j] == -1 else upper[j]
        K = np.block([[H[np.ix_(free, free)], A[:, free].T],
                      [A[:, free], np.zeros((len(b), len(b)))]])
        rhs = np.r_[-f[free] - H[np.ix_(free, active)] @ x[active],
                    b - A[:, active] @ x[active]]
        try:
            solution = np.linalg.solve(K, rhs)
        except np.linalg.LinAlgError:
            continue
        x[free] = solution[:free.size]
        y = solution[free.size:]
        gradient = H @ x + f + A.T @ y
        if (np.any(x < lower - 1e-9) or np.any(x > upper + 1e-9)
                or np.max(np.abs(A @ x - b)) > 1e-9):
            continue
        if any((pattern[j] == -1 and gradient[j] < -1e-8)
               or (pattern[j] == 1 and gradient[j] > 1e-8) for j in active):
            continue
        objective = .5 * x @ H @ x + f @ x
        if best is None or objective < best[0]:
            best = (float(objective), x, y, pattern)
    assert best is not None
    return best


def test_seeded_nonzero_rhs_against_independent_face_enumeration():
    seed = 280200
    rng = np.random.default_rng(seed)
    records = []
    for case in range(12):
        n = 4
        D = rng.normal(size=(n, n))
        H = D.T @ D + .3 * np.eye(n)
        f = rng.normal(size=n)
        A = rng.normal(size=(1, n))
        b = A @ rng.uniform(-.5, .5, n)
        lower, upper = -np.ones(n), np.ones(n)
        reference = enumerate_faces(H, f, A, b, lower, upper)
        result = affine_solver.solve_affine_box_qp(H, f, A, b, lower, upper)
        assert result.status == "QP_OPTIMAL", result.diagnostics
        x = np.asarray(result.x)
        assert np.max(np.abs(A @ x - b)) <= 1e-8
        assert np.min(x - lower) >= -1e-8 and np.min(upper - x) >= -1e-8
        assert result.diagnostics["kkt_stationarity_inf"] <= 1e-7
        assert np.max(np.abs(x - reference[1])) < 1e-6
        records.append({"case": case, "H": H.tolist(), "f": f.tolist(),
                        "A": A.tolist(), "b": b.tolist(),
                        "lower": lower.tolist(), "upper": upper.tolist(),
                        "reference_x": reference[1].tolist(), "reference_y": reference[2].tolist(),
                        "reference_face": reference[3], "reference_objective": reference[0],
                        "candidate_x": x.tolist(), "candidate_objective": result.diagnostics["objective"],
                        "candidate_primal_dual_gap": result.diagnostics["primal_dual_gap"],
                        "candidate_kkt_stationarity_inf": result.diagnostics["kkt_stationarity_inf"],
                        "candidate_active_lower": result.diagnostics["active_lower"],
                        "candidate_active_upper": result.diagnostics["active_upper"]})
    print("AFFINE_INDEPENDENT_ENUMERATION=" + json.dumps({
        "seed": seed, "case_count": len(records),
        "max_solution_abs_error": max(max(abs(a-b) for a, b in zip(item["candidate_x"], item["reference_x"])) for item in records),
        "max_objective_abs_error": max(abs(item["candidate_objective"]-item["reference_objective"]) for item in records),
        "max_abs_primal_dual_gap": max(abs(item["candidate_primal_dual_gap"]) for item in records),
        "records": records}, sort_keys=True))


def test_nonzero_rhs_dual_objective_has_correct_sign():
    # Analytically x=(.5,.5), y=-.5, objective=.25. The dual requires
    # -b*y; omitting or reversing it gives -.25 or -.75, respectively.
    A = np.array([[1., 1.]])
    b = np.array([1.])
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.zeros(2), A, b,
                                               np.full(2, -2.), np.full(2, 2.))
    assert result.status == "QP_OPTIMAL"
    assert result.x == pytest.approx((.5, .5), abs=1e-12)
    assert result.diagnostics["dual_equality"] == pytest.approx([-.5], abs=1e-12)
    assert result.diagnostics["dual_objective"] == pytest.approx(.25, abs=1e-12)
    assert abs(result.diagnostics["primal_dual_gap"]) < 1e-12


def test_redundant_rows_nonzero_rhs():
    A = np.array([[1., 1.], [2., 2.]])
    b = np.array([1., 2.])
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.zeros(2), A, b,
                                               np.full(2, -2.), np.full(2, 2.))
    assert result.status == "QP_OPTIMAL"
    assert result.diagnostics["equality_rank"] == 1
    assert result.x == pytest.approx((.5, .5), abs=1e-12)
    assert np.max(np.abs(A @ result.x - b)) < 1e-12


@pytest.mark.parametrize("A,b,lower,upper,reason", [
    ([[1., 1.]], [3.], [-1., -1.], [1., 1.], "HIGHS_FULL_LINEAR_CONSTRAINTS_INFEASIBLE"),
    ([[1., 1.], [2., 2.]], [1., 3.], [-2., -2.], [2., 2.], "HIGHS_FULL_LINEAR_CONSTRAINTS_INFEASIBLE"),
    ([[0., 0.]], [1.], [-2., -2.], [2., 2.], "ZERO_EQUALITY_ROW_WITH_NONZERO_RHS"),
    ([[1., 0.]], [0.], [1., 0.], [0., 1.], "EMPTY_BOX_INTERSECTION"),
])
def test_actual_infeasibility_controls(A, b, lower, upper, reason):
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.zeros(2), np.array(A),
                                               np.array(b), np.array(lower), np.array(upper))
    assert result.status == "QP_INFEASIBLE"
    assert result.x is None and result.diagnostics["reason"] == reason


def test_small_singular_value_not_misreported_as_infeasible():
    # Full problem has x=(0,1e8), comfortably within these bounds. The 1e-13
    # singular direction is deliberately under the reduction cutoff; silently
    # ignoring it would produce an incorrect result or infeasibility claim.
    A = np.diag([1., 1e-13])
    b = np.array([0., 1e-5])
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.zeros(2), A, b,
                                               np.array([-1., -1e9]), np.array([1., 1e9]))
    assert result.status == "NUMERICAL_FAILURE" and result.x is None
    assert result.diagnostics["reason"] == "TRUNCATED_EQUALITY_RHS_NOT_REPRESENTABLE"
    assert result.diagnostics["discarded_rhs_projection_inf"] == pytest.approx(1e-5)


def test_recheck_original_row_after_rank_truncation():
    # The RHS projects into the retained subspace but the discarded row is
    # significant at the stationary point. Full original equality must reject.
    A = np.diag([1., 1e-13])
    b = np.array([0., 0.])
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.array([0., -1e6]), A, b,
                                               np.array([-1., -2e6]), np.array([1., 2e6]))
    assert result.status == "NUMERICAL_FAILURE" and result.x is None
    assert result.diagnostics["max_equality_residual"] == pytest.approx(1e-7)
    assert result.diagnostics["reason"] == "INDEPENDENT_PRIMAL_OR_KKT_CHECK_FAILED"


def test_nonfinite_optimizer_candidate_is_never_exposed(monkeypatch):
    def bad_minimize(*args, **kwargs):
        return SimpleNamespace(x=np.array([np.nan, 0.]), success=True, status=0,
                               message="fake success", nit=1, nfev=1)
    monkeypatch.setattr(affine_solver, "minimize", bad_minimize)
    result = affine_solver.solve_affine_box_qp(np.eye(2), np.zeros(2), np.array([[1., 1.]]),
                                               np.array([1.]), np.full(2, -2.), np.full(2, 2.))
    assert result.status == "NUMERICAL_FAILURE" and result.x is None
    assert result.diagnostics["reason"] == "NONFINITE_OPTIMIZER_RESULT"
