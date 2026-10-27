"""Offline strictly convex box QP with affine equalities.

min 1/2 x^T H x + f^T x, A x = b, lower <= x <= upper.
HiGHS, SLSQP and all acceptance checks are floating-point numerical tools,
not exact certificates or evidence of continuous safety. No actuation interface.
"""
from dataclasses import dataclass
import time

import numpy as np
from scipy.optimize import linprog, lsq_linear, minimize


@dataclass(frozen=True)
class AffineBoxQPResult:
    status: str
    x: tuple | None
    diagnostics: dict


def _array(value, shape, name):
    arr = np.asarray(value)
    if (arr.shape != shape or not np.issubdtype(arr.dtype, np.number)
            or np.issubdtype(arr.dtype, np.complexfloating)
            or np.issubdtype(arr.dtype, np.bool_) or not np.isfinite(arr).all()):
        raise ValueError("INVALID_" + name)
    return arr.astype(float)


def _inf_norm(x):
    return float(np.max(np.abs(x))) if x.size else 0.0


def _solve(H, f, A, b, lower, upper, *, max_iterations, primal_tolerance,
           kkt_tolerance):
    start = time.perf_counter()
    n0 = np.asarray(f)
    if n0.ndim != 1 or n0.size < 1:
        raise ValueError("INVALID_f")
    n = n0.size
    f = _array(f, (n,), "f")
    H = _array(H, (n, n), "H")
    A0 = np.asarray(A)
    if A0.ndim != 2 or A0.shape[1] != n:
        raise ValueError("INVALID_A")
    A = _array(A, A0.shape, "A")
    b = _array(b, (A.shape[0],), "b")
    lower = _array(lower, (n,), "lower")
    upper = _array(upper, (n,), "upper")
    if (type(max_iterations) is not int or not 1 <= max_iterations <= 10000
            or any(isinstance(v, bool) or not np.isfinite(v) or v <= 0
                   for v in (primal_tolerance, kkt_tolerance))):
        raise ValueError("INVALID_SOLVER_BUDGET_OR_TOLERANCE")
    if not np.allclose(H, H.T, rtol=0, atol=1e-12):
        raise ValueError("H_NOT_SYMMETRIC")
    H = .5 * (H + H.T)
    eig = np.linalg.eigvalsh(H)
    if eig[0] <= 0:
        raise ValueError("H_NOT_STRICTLY_POSITIVE_DEFINITE")
    diag = {"dimension": n, "equality_rows": A.shape[0],
            "hessian_eigenvalues": eig.tolist(),
            "primal_tolerance": primal_tolerance, "kkt_tolerance": kkt_tolerance,
            "max_iterations": max_iterations, "optimizer": "scipy.optimize.minimize:SLSQP",
            "feasibility_optimizer": "scipy.optimize.linprog:highs",
            "mathematical_scope": "FINITE_BOX_AFFINE_EQUALITY_STRICTLY_CONVEX_QP",
            "execution_authorized": False}

    def finish(status, x=None):
        diag["solve_wall_time_s"] = time.perf_counter() - start
        return AffineBoxQPResult(status, None if x is None else tuple(float(v) for v in x), diag)

    if np.any(lower > upper):
        diag["reason"] = "EMPTY_BOX_INTERSECTION"
        return finish("QP_INFEASIBLE")
    # Normalize each equality row before HiGHS: otherwise its small-matrix
    # threshold can discard an entire physically significant row. Very small
    # coefficients within a row can still be discarded by HiGHS. If it reports
    # infeasible in that case, conservatively return NUMERICAL_FAILURE rather
    # than infer infeasibility from a potentially altered linear problem.
    row_scale = np.max(np.abs(A), axis=1) if A.shape[0] else np.empty(0)
    zero_rows = row_scale == 0
    if np.any(zero_rows & (b != 0)):
        diag["reason"] = "ZERO_EQUALITY_ROW_WITH_NONZERO_RHS"
        return finish("QP_INFEASIBLE")
    nonzero = ~zero_rows
    scaled_A = A[nonzero] / row_scale[nonzero, None]
    scaled_b = b[nonzero] / row_scale[nonzero]
    small_coefficients = bool(np.any((np.abs(scaled_A) > 0) & (np.abs(scaled_A) <= 1e-9)))
    diag.update(feasibility_row_normalization="MAX_ABS_COEFFICIENT",
                feasibility_row_scales=row_scale.tolist(),
                feasibility_has_tiny_scaled_coefficients=small_coefficients)
    feasible = linprog(np.zeros(n), A_eq=scaled_A if scaled_A.shape[0] else None,
                       b_eq=scaled_b if scaled_A.shape[0] else None, bounds=list(zip(lower, upper)),
                       method="highs", options={"time_limit": 10.})
    diag.update(feasibility_exit_status=int(feasible.status),
                feasibility_message=str(feasible.message),
                feasibility_success=bool(feasible.success))
    if feasible.status == 2:
        if small_coefficients:
            diag["reason"] = "HIGHS_INFEASIBLE_WITH_SMALL_COEFFICIENTS_UNCERTAIN"
            return finish("NUMERICAL_FAILURE")
        diag["reason"] = "HIGHS_FULL_LINEAR_CONSTRAINTS_INFEASIBLE"
        return finish("QP_INFEASIBLE")
    if not feasible.success or feasible.x is None or not np.isfinite(feasible.x).all():
        diag["reason"] = "FULL_FEASIBILITY_SOLVER_FAILED"
        return finish("NUMERICAL_FAILURE")
    feasible_x = np.asarray(feasible.x)
    full_feas_residual = _inf_norm(A @ feasible_x - b)
    full_feas_box = float(max(0., np.max(lower - feasible_x), np.max(feasible_x - upper)))
    diag.update(feasibility_full_equality_residual=full_feas_residual,
                feasibility_box_violation=full_feas_box)
    if full_feas_residual > primal_tolerance or full_feas_box > primal_tolerance:
        diag["reason"] = "FULL_FEASIBILITY_SOLUTION_NOT_VERIFIED"
        return finish("NUMERICAL_FAILURE")

    # A=U S V^T. The retained independent equations are V_r^T x=S_r^-1 U_r^T b.
    # The omitted RHS projection is recorded; full original residual is always
    # rechecked. When row truncation fails, it is a numerical failure rather than
    # an assertion that the original mathematical problem is infeasible.
    u, s, vh = np.linalg.svd(A, full_matrices=True)
    cutoff = float(s[0] * 1e-12) if s.size else 0.
    rank = int(np.count_nonzero(s > cutoff))
    reduced = vh[:rank]
    rhs = (u[:, :rank].T @ b) / s[:rank] if rank else np.empty(0)
    lost_rhs = b - (u[:, :rank] @ (u[:, :rank].T @ b) if rank else np.zeros_like(b))
    diag.update(equality_rank=rank, equality_singular_values=s.tolist(),
                equality_relative_svd_cutoff=1e-12,
                discarded_rhs_projection_inf=_inf_norm(lost_rhs),
                reduced_rhs=rhs.tolist())
    if _inf_norm(lost_rhs) > primal_tolerance:
        diag["reason"] = "TRUNCATED_EQUALITY_RHS_NOT_REPRESENTABLE"
        return finish("NUMERICAL_FAILURE")
    constraints = ([] if rank == 0 else
                   [{"type": "eq", "fun": lambda x: reduced @ x - rhs,
                     "jac": lambda x: reduced}])
    null_basis = vh[rank:].T
    particular = reduced.T @ rhs if rank else np.zeros(n)
    stationary = (particular + null_basis @ np.linalg.solve(
        null_basis.T @ H @ null_basis, -null_basis.T @ (H @ particular + f))
        if null_basis.shape[1] else particular)
    inside = bool(np.all(stationary >= lower) and np.all(stationary <= upper))
    initial = stationary if inside else feasible_x
    diag["initial_seed"] = ("AFFINE_NULLSPACE_STATIONARY_POINT_IN_BOX" if inside
                            else "HIGHS_FULL_FEASIBLE_VERTEX")
    result = minimize(lambda x: .5 * float(x @ H @ x) + float(f @ x), initial,
                      jac=lambda x: H @ x + f, method="SLSQP",
                      bounds=list(zip(lower, upper)), constraints=constraints,
                      options={"maxiter": max_iterations, "ftol": 1e-12, "disp": False})
    diag.update(optimizer_success=bool(result.success), optimizer_exit_status=int(result.status),
                optimizer_message=str(result.message), optimizer_iterations=int(result.nit),
                optimizer_function_evaluations=int(result.nfev))
    x = np.asarray(result.x)
    if x.shape != (n,) or not np.isfinite(x).all():
        diag["reason"] = "NONFINITE_OPTIMIZER_RESULT"
        return finish("NUMERICAL_FAILURE")
    # Deterministic active-face quadratic polish. It solves the same KKT
    # equations on SLSQP's inferred active face; no bound is clipped and the
    # candidate must independently satisfy the full equations and exact box.
    # This avoids rejecting an otherwise adequate SLSQP point for small residual
    # stationarity due solely to its absolute objective stopping criterion.
    diag["optimizer_candidate_x"] = x.tolist()
    polish_lower = np.flatnonzero(x - lower <= 1e-7)
    polish_upper = np.flatnonzero(upper - x <= 1e-7)
    active = np.union1d(polish_lower, polish_upper)
    free = np.setdiff1d(np.arange(n), active)
    if active.size and free.size:
        point = np.zeros(n)
        # Fixed bounds are identical; where both are active either is correct.
        for j in active:
            point[j] = lower[j] if j in polish_lower else upper[j]
        hff = H[np.ix_(free, free)]
        af = reduced[:, free]
        kkt_matrix = np.block([[hff, af.T], [af, np.zeros((rank, rank))]])
        rhs_face = np.r_[-f[free] - H[np.ix_(free, active)] @ point[active],
                         rhs - reduced[:, active] @ point[active]]
        point[free] = np.linalg.lstsq(kkt_matrix, rhs_face, rcond=None)[0][:free.size]
        current_obj = .5 * float(x @ H @ x) + float(f @ x)
        point_obj = .5 * float(point @ H @ point) + float(f @ point)
        polished = (np.isfinite(point).all() and np.all(point >= lower) and np.all(point <= upper)
                    and _inf_norm(A @ point - b) <= primal_tolerance
                    and point_obj <= current_obj + 1e-12)
        diag["active_face_polish_attempted"] = True
        diag["active_face_polish_accepted"] = bool(polished)
        diag["active_face_polish_candidate_x"] = point.tolist()
        if polished:
            x = point
    else:
        diag["active_face_polish_attempted"] = False
        diag["active_face_polish_accepted"] = False
    equality = A @ x - b
    margins = np.minimum(x - lower, upper - x)
    box_violation = float(max(0., -np.min(margins)))
    active_tol = 1e-7
    active_lower = np.flatnonzero(x - lower <= active_tol)
    active_upper = np.flatnonzero(upper - x <= active_tol)
    eye = np.eye(n)
    dual_matrix = np.column_stack([A.T, -eye[:, active_lower], eye[:, active_upper]])
    dual_low = np.r_[np.full(A.shape[0], -np.inf),
                     np.zeros(len(active_lower) + len(active_upper))]
    if dual_matrix.shape[1]:
        dual = lsq_linear(dual_matrix, -(H @ x + f),
                          bounds=(dual_low, np.full(dual_matrix.shape[1], np.inf)),
                          tol=1e-13, lsmr_tol=1e-13, max_iter=1000)
        y = dual.x[:A.shape[0]]
        alpha = np.zeros(n)
        beta = np.zeros(n)
        alpha[active_lower] = dual.x[A.shape[0]:A.shape[0] + len(active_lower)]
        beta[active_upper] = dual.x[A.shape[0] + len(active_lower):]
        dual_success = bool(dual.success)
    else:
        y, alpha, beta = np.zeros(A.shape[0]), np.zeros(n), np.zeros(n)
        dual_success = True
    stationarity = H @ x + f + A.T @ y - alpha + beta
    complementarity = max(_inf_norm(alpha * (x - lower)), _inf_norm(beta * (upper - x)))
    gradient_dual = f + A.T @ y - alpha + beta
    objective = .5 * float(x @ H @ x) + float(f @ x)
    dual_objective = (-.5 * float(gradient_dual @ np.linalg.solve(H, gradient_dual))
                      - float(b @ y) + float(alpha @ lower - beta @ upper))
    finite_dual = bool(np.isfinite(y).all() and np.isfinite(alpha).all() and np.isfinite(beta).all())
    sign_violation = float(max(0., -alpha.min(), -beta.min()))
    gap_tolerance = kkt_tolerance * max(1., abs(objective))
    diag.update(objective=objective, dual_objective=dual_objective,
                primal_dual_gap=objective - dual_objective,
                primal_dual_gap_tolerance=gap_tolerance,
                equality_residual=equality.tolist(), max_equality_residual=_inf_norm(equality),
                box_margins=margins.tolist(), max_box_violation=box_violation,
                active_lower=active_lower.tolist(), active_upper=active_upper.tolist(),
                active_bound_tolerance=active_tol, dual_equality=y.tolist(),
                dual_lower=alpha.tolist(), dual_upper=beta.tolist(),
                dual_reconstruction_success=dual_success,
                stationarity_residual=stationarity.tolist(), kkt_stationarity_inf=_inf_norm(stationarity),
                complementarity_inf=complementarity, dual_all_finite=finite_dual,
                max_dual_sign_violation=sign_violation)
    verified = (finite_dual and dual_success and sign_violation <= kkt_tolerance
                and np.isfinite(dual_objective) and abs(objective - dual_objective) <= gap_tolerance
                and _inf_norm(equality) <= primal_tolerance and box_violation <= primal_tolerance
                and _inf_norm(stationarity) <= kkt_tolerance and complementarity <= kkt_tolerance)
    diag.update(independent_kkt_verified=bool(verified),
                acceptance_basis="INDEPENDENT_PRIMAL_DUAL_KKT_NOT_OPTIMIZER_STATUS",
                accepted_despite_optimizer_non_success=bool(verified and not result.success),
                reason="INDEPENDENT_PRIMAL_KKT_VERIFIED" if verified else
                       "INDEPENDENT_PRIMAL_OR_KKT_CHECK_FAILED")
    return finish("QP_OPTIMAL" if verified else "NUMERICAL_FAILURE", x if verified else None)


def solve_affine_box_qp(H, f, A, b, lower, upper, *, max_iterations=500,
                        primal_tolerance=1e-8, kkt_tolerance=1e-7):
    """Solve offline; invalid inputs raise ValueError, numerical failures return no x.

    QP_INFEASIBLE is issued only for an empty box or a full-row HiGHS
    infeasibility status, subject to that backend's floating-point tolerances.
    An optimizer's success flag alone never authorizes a result.
    """
    start = time.perf_counter()
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            return _solve(H, f, A, b, lower, upper, max_iterations=max_iterations,
                          primal_tolerance=primal_tolerance, kkt_tolerance=kkt_tolerance)
    except ValueError as exc:
        if str(exc).startswith("INVALID_") or str(exc) in (
                "H_NOT_SYMMETRIC", "H_NOT_STRICTLY_POSITIVE_DEFINITE"):
            raise
        failure = exc
    except Exception as exc:
        failure = exc
    return AffineBoxQPResult("NUMERICAL_FAILURE", None,
                             {"reason": "NUMERICAL_BACKEND_EXCEPTION",
                              "exception_type": type(failure).__name__,
                              "exception_message": str(failure),
                              "solve_wall_time_s": time.perf_counter() - start,
                              "execution_authorized": False})
