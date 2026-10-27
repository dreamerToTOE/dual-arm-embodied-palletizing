"""Offline B-channel algebra only; no sensor, physics, or robot interface.

The input torque must already be an external contribution. A raw actuator or
solver joint effort is not such a contribution. The 6-vector is [Fx,Fy,Fz,
Mx,My,Mz], acting ON the robot at the specified TCP point, expressed in the
specified frame, in N and N m. q is in radians and tau is in N m.
"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Contract:
    joint_names: tuple[str, ...]
    frame: str
    tcp_point: str
    max_age_s: float
    max_skew_s: float
    rank_rtol: float = 1e-10


@dataclass(frozen=True)
class Observation:
    q: object
    tau_external: object
    jacobian: object
    jacobian_q: object
    joint_names: tuple[str, ...]
    jacobian_joint_names: tuple[str, ...]
    frame: str
    tcp_point: str
    q_timestamp_s: float
    torque_timestamp_s: float
    jacobian_timestamp_s: float
    evaluation_timestamp_s: float
    torque_source: str
    torque_provenance: str
    jacobian_provenance: str
    q_units: str = "rad"
    torque_units: str = "N*m"
    wrench_units: str = "N,N*m"
    sign_convention: str = "wrench_on_robot"
    jacobian_order: str = "linear_then_angular"


@dataclass(frozen=True)
class Estimate:
    status: str
    wrench_minimum_norm: tuple[float, ...]
    singular_values: tuple[float, ...]
    rank: int
    condition_number: float | None
    reconstruction_residual_Nm: float
    reconstructed_tau_Nm: tuple[float, ...]
    observable_projector: tuple[tuple[float, ...], ...]
    unobservable_basis: tuple[tuple[float, ...], ...]
    diagnostics: str


def _vec(value, shape, label):
    try:
        arr = np.asarray(value)
        if (arr.shape != shape or not np.issubdtype(arr.dtype, np.number)
                or np.issubdtype(arr.dtype, np.complexfloating)
                or np.issubdtype(arr.dtype, np.bool_)):
            raise ValueError(label)
        arr = arr.astype(float)
        if not np.isfinite(arr).all():
            raise ValueError(label)
        return arr
    except (TypeError, OverflowError):
        raise ValueError(label) from None


def _names(value):
    return isinstance(value, tuple) and len(value) == 7 and all(isinstance(x, str) and x for x in value) and len(set(value)) == 7


def estimate(obs: Observation, contract: Contract) -> Estimate:
    """Solve J.T w=tau via SVD; rank loss is reported, never hidden.

    Provenance and identities are claims from a caller that must be established
    by independent source/measurement checks. No known applied load is used.
    Singular values/condition reflect the supplied SI coordinates and thus
    depend on the length and force/torque scaling conventions.
    """
    if not isinstance(obs, Observation) or not isinstance(contract, Contract):
        raise ValueError("INVALID_CONTRACT_OR_OBSERVATION")
    if not _names(contract.joint_names) or not _names(obs.joint_names) or not _names(obs.jacobian_joint_names):
        raise ValueError("INVALID_JOINT_NAMES")
    if obs.joint_names != contract.joint_names or obs.jacobian_joint_names != contract.joint_names:
        raise ValueError("JOINT_ORDER_MISMATCH")
    if not isinstance(contract.frame, str) or not contract.frame or not isinstance(contract.tcp_point, str) or not contract.tcp_point:
        raise ValueError("INVALID_FRAME_CONTRACT")
    if obs.frame != contract.frame or obs.tcp_point != contract.tcp_point:
        raise ValueError("FRAME_OR_POINT_MISMATCH")
    if (obs.q_units, obs.torque_units, obs.wrench_units, obs.sign_convention, obs.jacobian_order) != (
            "rad", "N*m", "N,N*m", "wrench_on_robot", "linear_then_angular"):
        raise ValueError("UNSUPPORTED_UNITS_SIGN_OR_ORDER")
    if obs.torque_source not in ("paired_static_load_difference", "validated_model_external_torque"):
        raise ValueError("RAW_OR_UNSUPPORTED_TORQUE_SOURCE")
    if not isinstance(obs.torque_provenance, str) or not obs.torque_provenance.strip() or not isinstance(obs.jacobian_provenance, str) or not obs.jacobian_provenance.strip():
        raise ValueError("MISSING_PROVENANCE")
    cfg = _vec([contract.max_age_s, contract.max_skew_s, contract.rank_rtol], (3,), "INVALID_LIMITS")
    if cfg[0] < 0 or cfg[1] < 0 or not 0 < cfg[2] < 1:
        raise ValueError("INVALID_LIMITS")
    times = _vec([obs.q_timestamp_s, obs.torque_timestamp_s, obs.jacobian_timestamp_s, obs.evaluation_timestamp_s], (4,), "INVALID_TIMESTAMP")
    sample, now = times[:3], times[3]
    if np.any(sample < 0) or now < 0 or np.any(sample > now):
        raise ValueError("FUTURE_OR_NEGATIVE_TIMESTAMP")
    if np.any(now - sample > cfg[0]):
        raise ValueError("STALE_OBSERVATION")
    if np.ptp(sample) > cfg[1]:
        raise ValueError("ASYNCHRONOUS_OBSERVATION")
    q = _vec(obs.q, (7,), "INVALID_Q")
    jac_q = _vec(obs.jacobian_q, (7,), "INVALID_JACOBIAN_Q")
    if not np.array_equal(q, jac_q):
        raise ValueError("JACOBIAN_CONFIGURATION_MISMATCH")
    tau = _vec(obs.tau_external, (7,), "INVALID_EXTERNAL_TORQUE")
    J = _vec(obs.jacobian, (6, 7), "INVALID_JACOBIAN")
    # J.T = U S V.T. Wrench-space columns of V define observable modes.
    try:
        with np.errstate(over="ignore", invalid="ignore", divide="ignore", under="ignore"):
            U, s, Vt = np.linalg.svd(J.T, full_matrices=False)
            if not all(np.isfinite(x).all() for x in (U, s, Vt)):
                raise ValueError("NUMERICAL_FAILURE_NONFINITE_SVD")
            threshold = cfg[2] * s[0] if s[0] else 0.0
            rank = int(np.count_nonzero(s > threshold))
            Vr = Vt[:rank].T
            w = Vr @ ((U[:, :rank].T @ tau) / s[:rank]) if rank else np.zeros(6)
            reconstructed = J.T @ w
            proj = Vr @ Vr.T
            null = Vt[rank:].T
            cond = float(s[0] / s[-1]) if rank == 6 else None
            residual = float(np.linalg.norm(reconstructed - tau))
    except np.linalg.LinAlgError as exc:
        raise ValueError("NUMERICAL_FAILURE_SVD") from exc
    if (not all(np.isfinite(x).all() for x in (w, reconstructed, proj, null))
            or not math.isfinite(residual) or (cond is not None and not math.isfinite(cond))):
        raise ValueError("NUMERICAL_FAILURE_NONFINITE_RESULT")
    return Estimate(
        "FULL_RANK" if rank == 6 else "RANK_DEFICIENT_NONUNIQUE",
        tuple(map(float, w)), tuple(map(float, s)), rank, cond,
        residual, tuple(map(float, reconstructed)),
        tuple(tuple(map(float, row)) for row in proj),
        tuple(tuple(map(float, row)) for row in null.T),
        "SVD in SI coordinates; residual is not a sensor calibration or uncertainty estimate.")
