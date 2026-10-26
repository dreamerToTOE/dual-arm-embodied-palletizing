"""Bounded strictly convex box/equality QP; no collision or robot abstraction.

Minimize .5*x.T*H*x + f.T*x, subject to C*x=0 and lower<=x<=upper.
SLSQP is the numerical optimizer. Independent HiGHS linear feasibility and a
separate bounded least-squares dual reconstruction check its result. These are
floating-point checks, not exact-arithmetic certificates or continuous safety.
"""
from dataclasses import dataclass
import time
import numpy as np
from scipy.optimize import linprog, lsq_linear, minimize


@dataclass(frozen=True)
class BoxEqualityResult:
    status: str
    x: tuple | None
    diagnostics: dict


def _array(value, shape, name):
    value = np.asarray(value)
    if (value.shape != shape or not np.issubdtype(value.dtype, np.number)
            or np.issubdtype(value.dtype, np.complexfloating)
            or np.issubdtype(value.dtype, np.bool_) or not np.isfinite(value).all()):
        raise ValueError("INVALID_" + name)
    return value.astype(float)


def _solve_box_equality_qp(H, f, C, lower, upper, *, max_iterations=500,
                          primal_tolerance=1e-8, kkt_tolerance=1e-7):
    """Low-level abstract mathematical control interface, any finite dimension.

    Equality rank reduction is numerical (relative SVD cutoff 1e-12). Acceptance
    always rechecks the original, unreduced C. Strict SPD H is required.
    """
    start = time.perf_counter()
    raw_f = np.asarray(f)
    if raw_f.ndim != 1 or raw_f.size < 1:
        raise ValueError("INVALID_f")
    n = raw_f.size
    f = _array(f, (n,), "f")
    H = _array(H, (n, n), "H")
    C0 = np.asarray(C)
    if C0.ndim != 2 or C0.shape[1] != n:
        raise ValueError("INVALID_C")
    C = _array(C, C0.shape, "C")
    lower, upper = (_array(v, (n,), key) for v, key in ((lower, "lower"), (upper, "upper")))
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
    _, svals, vh = np.linalg.svd(C, full_matrices=True)
    cutoff = float(svals[0] * 1e-12) if svals.size else 0.
    rank = int(np.count_nonzero(svals > cutoff))
    # Unit right singular rows preserve the homogeneous equality null space.
    reduced = vh[:rank]
    diag = {"dimension": n, "equality_rows": C.shape[0], "equality_rank": rank,
            "equality_singular_values": svals.tolist(), "equality_relative_svd_cutoff": 1e-12,
            "hessian_eigenvalues": eig.tolist(), "primal_tolerance": primal_tolerance,
            "kkt_tolerance": kkt_tolerance, "max_iterations": max_iterations,
            "optimizer": "scipy.optimize.minimize:SLSQP",
            "feasibility_optimizer": "scipy.optimize.linprog:highs",
            "mathematical_scope": "FINITE_BOX_HOMOGENEOUS_EQUALITY_STRICTLY_CONVEX_QP",
            "execution_authorized": False}
    def finish(status, x=None):
        diag["solve_wall_time_s"] = time.perf_counter() - start
        return BoxEqualityResult(status, None if x is None else tuple(float(v) for v in x), diag)
    if np.any(lower > upper):
        diag.update(reason="EMPTY_BOX_INTERSECTION", feasibility_exit_status=None)
        return finish("QP_INFEASIBLE")
    feasible = linprog(np.zeros(n), A_eq=reduced if rank else None,
                       b_eq=np.zeros(rank) if rank else None,
                       bounds=list(zip(lower, upper)), method="highs",
                       options={"time_limit": 10.})
    diag.update(feasibility_exit_status=int(feasible.status),
                feasibility_message=str(feasible.message), feasibility_success=bool(feasible.success))
    if feasible.status == 2:
        diag["reason"] = "HIGHS_LINEAR_CONSTRAINTS_INFEASIBLE"
        return finish("QP_INFEASIBLE")
    if not feasible.success or feasible.x is None:
        diag["reason"] = "FEASIBILITY_SOLVER_FAILED"
        return finish("NUMERICAL_FAILURE")
    constraints = [] if not rank else [{"type": "eq", "fun": lambda x: reduced @ x,
                                       "jac": lambda x: reduced}]
    # Independent null-space stationary point is a deterministic warm start.
    # It is used only if already inside every box bound; it is never clipped.
    # This also avoids SLSQP's absolute objective stopping tolerance losing a
    # small regularization eigen-direction near a tiny desired velocity.
    null_basis = vh[rank:].T
    unconstrained = (null_basis @ np.linalg.solve(null_basis.T @ H @ null_basis,
                                                 -null_basis.T @ f)
                     if null_basis.shape[1] else np.zeros(n))
    inside = np.all(unconstrained >= lower) and np.all(unconstrained <= upper)
    initial = unconstrained if inside else feasible.x
    diag["initial_seed"] = "NULLSPACE_STATIONARY_POINT_IN_BOX" if inside else "HIGHS_FEASIBLE_VERTEX"
    result = minimize(lambda x: .5 * float(x @ H @ x) + float(f @ x), initial,
                      jac=lambda x: H @ x + f, method="SLSQP",
                      bounds=list(zip(lower, upper)), constraints=constraints,
                      options={"maxiter": max_iterations, "ftol": 1e-12, "disp": False})
    x = np.asarray(result.x)
    diag.update(optimizer_success=bool(result.success), optimizer_exit_status=int(result.status),
                optimizer_message=str(result.message), optimizer_iterations=int(result.nit),
                optimizer_function_evaluations=int(result.nfev))
    if x.shape != (n,) or not np.isfinite(x).all():
        diag["reason"] = "NONFINITE_OPTIMIZER_RESULT"
        return finish("NUMERICAL_FAILURE")
    equality = C @ x
    margins = np.minimum(x - lower, upper - x)
    max_equality = float(np.max(np.abs(equality))) if equality.size else 0.
    box_violation = float(max(0., -np.min(margins)))
    active_tol = 1e-7
    active_lower = np.flatnonzero(x - lower <= active_tol)
    active_upper = np.flatnonzero(upper - x <= active_tol)
    # Reconstruct dual variables separately, using original full C (including
    # redundant rows), lower coefficient -I and upper coefficient +I.
    eye = np.eye(n)
    A = np.column_stack([C.T, -eye[:, active_lower], eye[:, active_upper]])
    dual_low = np.r_[np.full(C.shape[0], -np.inf), np.zeros(len(active_lower) + len(active_upper))]
    if A.shape[1]:
        dual = lsq_linear(A, -(H @ x + f), bounds=(dual_low, np.full(A.shape[1], np.inf)),
                          tol=1e-13, lsmr_tol=1e-13, max_iter=1000)
        y = dual.x[:C.shape[0]]
        alpha = np.zeros(n); beta = np.zeros(n)
        alpha[active_lower] = dual.x[C.shape[0]:C.shape[0] + len(active_lower)]
        beta[active_upper] = dual.x[C.shape[0] + len(active_lower):]
        dual_success = bool(dual.success)
    else:
        y = np.zeros(C.shape[0]); alpha = np.zeros(n); beta = np.zeros(n)
        dual_success = True
    stationarity = H @ x + f + C.T @ y - alpha + beta
    complementarity = max(float(np.max(np.abs(alpha * (x - lower)))),
                          float(np.max(np.abs(beta * (upper - x)))))
    stationarity_norm = float(np.max(np.abs(stationarity)))
    objective = .5 * float(x @ H @ x) + float(f @ x)
    dual_objective = -.5 * float((f + C.T @ y - alpha + beta) @ np.linalg.solve(H, f + C.T @ y - alpha + beta)) + float(alpha @ lower - beta @ upper)
    dual_all_finite = bool(np.isfinite(y).all() and np.isfinite(alpha).all() and np.isfinite(beta).all())
    dual_sign_violation = float(max(0., -alpha.min(), -beta.min()))
    diag.update(objective=objective, dual_objective=dual_objective,
                primal_dual_gap=objective - dual_objective,
                equality_residual=equality.tolist(), max_equality_residual=max_equality,
                box_margins=margins.tolist(), max_box_violation=box_violation,
                active_lower=active_lower.tolist(), active_upper=active_upper.tolist(),
                active_bound_tolerance=active_tol, dual_equality=y.tolist(),
                dual_lower=alpha.tolist(), dual_upper=beta.tolist(),
                dual_reconstruction_success=dual_success,
                stationarity_residual=stationarity.tolist(), kkt_stationarity_inf=stationarity_norm,
                complementarity_inf=complementarity,
                dual_all_finite=dual_all_finite, max_dual_sign_violation=dual_sign_violation)
    gap_tolerance = kkt_tolerance * max(1., abs(objective))
    diag["primal_dual_gap_tolerance"] = gap_tolerance
    verified = (dual_all_finite and dual_sign_violation <= kkt_tolerance
                and np.isfinite(dual_objective) and abs(objective-dual_objective) <= gap_tolerance
                and max_equality <= primal_tolerance and box_violation <= primal_tolerance
                and stationarity_norm <= kkt_tolerance and complementarity <= kkt_tolerance
                and dual_success)
    diag["independent_kkt_verified"] = bool(verified)
    # Strict convexity makes primal feasibility + dual feasibility + KKT
    # stationarity/complementarity sufficient for the unique optimum within the
    # stated numerical tolerances. The raw SLSQP flag is retained, not treated
    # as a substitute for these independent checks (including primal-dual gap).
    diag["acceptance_basis"] = "INDEPENDENT_PRIMAL_DUAL_KKT_NOT_OPTIMIZER_STATUS"
    diag["accepted_despite_optimizer_non_success"] = bool(verified and not result.success)
    diag["reason"] = "INDEPENDENT_PRIMAL_KKT_VERIFIED" if verified else "INDEPENDENT_PRIMAL_OR_KKT_CHECK_FAILED"
    return finish("QP_OPTIMAL" if verified else "NUMERICAL_FAILURE", x if verified else None)


def solve_box_equality_qp(H, f, C, lower, upper, *, max_iterations=500,
                          primal_tolerance=1e-8, kkt_tolerance=1e-7):
    """Validate mathematical inputs; contain numerical/backend failures.

    Invalid mathematical input raises ValueError. Numerical backend exceptions,
    overflow and nonfinite arithmetic return NUMERICAL_FAILURE with x=None.
    QP_OPTIMAL is based on the independent primal/dual/KKT checks, not the raw
    SLSQP status; strict convexity supplies the mathematical sufficiency reason.
    Raw optimizer status and that acceptance basis remain explicit diagnostics.
    """
    start = time.perf_counter()
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            return _solve_box_equality_qp(H, f, C, lower, upper,
                                         max_iterations=max_iterations,
                                         primal_tolerance=primal_tolerance,
                                         kkt_tolerance=kkt_tolerance)
    except ValueError as exc:
        # Keep explicit public contract violations distinct from exceptions
        # raised by SVD/LP/SLSQP/dual numerical backends after valid inputs.
        text = str(exc)
        if text.startswith("INVALID_") or text in ("H_NOT_SYMMETRIC", "H_NOT_STRICTLY_POSITIVE_DEFINITE"):
            raise
        failure = exc
    except Exception as exc:
        failure = exc
    return BoxEqualityResult("NUMERICAL_FAILURE", None,
                             {"reason": "NUMERICAL_BACKEND_EXCEPTION",
                              "exception_type": type(failure).__name__,
                              "exception_message": str(failure),
                              "solve_wall_time_s": time.perf_counter()-start,
                              "execution_authorized": False})
