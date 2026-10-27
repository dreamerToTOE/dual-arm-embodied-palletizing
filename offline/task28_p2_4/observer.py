"""Causal model observer on explicitly identified generalized coordinates.

Candidate equation uses generalized efforts ON the robot, in SI:
    M(q) qdd + c(q, qd) + g(q) = tau_actuator + tau_external.
Revolute coordinates use rad, rad/s, rad/s² and N m; prismatic coordinates
use m, m/s, m/s² and N. Matrix element M[i,j] has units of generalized
effort i divided by acceleration j. All dynamic terms below are LHS terms. No applied-load oracle, unloaded
counterpart, contact, or calibration fitting is accepted by this API.

A validation claim is metadata, not proof: callers must independently establish
it for the exact API, source version, timing, sign and physics configuration.
The estimator cannot detect unmodelled inertial/friction errors or determine
whether a wrench is generated at the declared TCP. In particular an arbitrary
contact or multiple disturbances can be projected into an apparent TCP wrench.
"""
from dataclasses import dataclass
import math
import re
import numpy as np

from offline.task28_p2_2.estimator import Contract as WrenchContract
from offline.task28_p2_2.estimator import Observation, estimate


@dataclass(frozen=True)
class ObserverContract:
    joint_names: tuple[str, ...]              # full articulation DOF order
    arm_joint_names: tuple[str, ...]          # ordered 7-element arm subset
    frame: str
    tcp_point: str
    max_age_s: float
    max_skew_s: float
    min_dt_s: float
    max_dt_s: float
    rank_rtol: float
    max_condition: float
    source_status: str                      # SYNTHETIC_ONLY, SOURCE_UNVALIDATED_CANDIDATE, or CALLER_CLAIMS_VALIDATED
    source_evidence_sha256: str
    dof_types: tuple[str, ...]              # revolute or prismatic, full order
    max_tcp_residual_Nm: float              # arm generalized torque residual


@dataclass(frozen=True)
class Sample:
    joint_names: tuple[str, ...]
    arm_joint_names: tuple[str, ...]
    position_SI: object
    velocity_SI: object
    actuator_generalized_effort_SI: object
    mass_matrix: object
    coriolis_lhs_SI: object
    gravity_lhs_SI: object
    jacobian: object                        # 6 x 7, linear then angular
    model_position_SI: object
    model_velocity_SI: object
    jacobian_position_SI: object                  # ordered arm q
    q_time_s: float
    velocity_time_s: float
    effort_time_s: float
    model_time_s: float
    jacobian_time_s: float
    evaluation_time_s: float
    evidence_id: str
    provenance: dict                         # q,dq,effort,mass,coriolis,gravity,jacobian
    frame: str
    tcp_point: str
    dof_types: tuple[str, ...]
    dof_units: tuple[str, ...]              # position, velocity, acceleration, effort
    mass_matrix_units: str                  # effort_i/acceleration_j
    effort_semantics: str = "actuator_on_robot"
    model_semantics: str = "lhs_Mqdd_plus_c_plus_g"
    units: str = "generalized_SI"
    jacobian_order: str = "linear_then_angular"


@dataclass(frozen=True)
class VelocityHistory:
    names: tuple[str, ...]
    velocity_SI: tuple[float, ...]
    timestamp_s: float
    evidence_id: str
    provenance: str
    signal_times_s: tuple[float, ...]


@dataclass(frozen=True)
class StepResult:
    status: str
    history: VelocityHistory
    source_status: str
    acceleration_SI: tuple[float, ...] | None
    inertial_term_SI: tuple[float, ...] | None
    external_generalized_force_on_robot_SI: tuple[float, ...] | None
    arm_external_torque_on_robot_Nm: tuple[float, ...] | None
    wrench: object | None
    sample_age_s: float
    max_sample_skew_s: float
    derivative_dt_s: float | None
    diagnostics: str


def _array(value, shape, label):
    try:
        raw = np.asarray(value)
        if (raw.shape != shape or not np.issubdtype(raw.dtype, np.number)
            or np.issubdtype(raw.dtype, np.complexfloating)
            or np.issubdtype(raw.dtype, np.bool_)):
            raise ValueError(label)
        a = raw.astype(float)
        if not np.all(np.isfinite(a)):
            raise ValueError(label)
        return a
    except (TypeError, OverflowError):
        raise ValueError(label) from None


def _valid_names(names):
    return (isinstance(names, tuple) and len(names) > 0
            and all(isinstance(x, str) and x.strip() for x in names)
            and len(set(names)) == len(names))


def _contract(c):
    if not isinstance(c, ObserverContract) or not _valid_names(c.joint_names) or not _valid_names(c.arm_joint_names) or len(c.arm_joint_names) != 7:
        raise ValueError("INVALID_JOINT_CONTRACT")
    if not set(c.arm_joint_names).issubset(c.joint_names):
        raise ValueError("ARM_NOT_IN_ARTICULATION")
    if (not isinstance(c.dof_types, tuple) or len(c.dof_types) != len(c.joint_names)
        or any(x not in ("revolute", "prismatic") for x in c.dof_types)):
        raise ValueError("INVALID_DOF_TYPES")
    if any(c.dof_types[c.joint_names.index(x)] != "revolute" for x in c.arm_joint_names):
        raise ValueError("NONREVOLUTE_ARM_JOINT")
    limits = _array([c.max_age_s, c.max_skew_s, c.min_dt_s, c.max_dt_s,
                     c.rank_rtol, c.max_condition, c.max_tcp_residual_Nm], (7,), "INVALID_LIMITS")
    if (limits[0] < 0 or limits[1] < 0 or limits[2] <= 0 or limits[3] < limits[2]
        or not 0 < limits[4] < 1 or limits[5] < 1 or limits[6] < 0):
        raise ValueError("INVALID_LIMITS")
    if not isinstance(c.frame, str) or not c.frame.strip() or not isinstance(c.tcp_point, str) or not c.tcp_point.strip():
        raise ValueError("INVALID_FRAME")
    if c.source_status not in ("SYNTHETIC_ONLY", "SOURCE_UNVALIDATED_CANDIDATE", "CALLER_CLAIMS_VALIDATED"):
        raise ValueError("UNSUPPORTED_OR_UNVALIDATED_SOURCE")
    if not isinstance(c.source_evidence_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", c.source_evidence_sha256):
        raise ValueError("MISSING_SOURCE_EVIDENCE")


def step(sample: Sample, contract: ObserverContract, previous: VelocityHistory | None = None) -> StepResult:
    """One causal step; derivative uses only the prior measured velocity.

    First sample is WARMUP with no estimated torque/wrench. An invalid step
    raises ValueError and returns no history (caller must not advance history).
    Does not estimate accuracy, dynamic validity, continuous time latency or
    physical contact. Wrench result is only a TCP-equivalent mathematical fit.
    """
    _contract(contract)
    if not isinstance(sample, Sample):
        raise ValueError("INVALID_SAMPLE")
    if sample.joint_names != contract.joint_names or sample.arm_joint_names != contract.arm_joint_names:
        raise ValueError("JOINT_ORDER_MISMATCH")
    if sample.dof_types != contract.dof_types:
        raise ValueError("DOF_TYPE_MISMATCH")
    expected_units = tuple("rad,rad/s,rad/s2,N*m" if t == "revolute" else "m,m/s,m/s2,N"
                           for t in contract.dof_types)
    if sample.dof_units != expected_units or sample.mass_matrix_units != "effort_i/acceleration_j":
        raise ValueError("DOF_UNITS_MISMATCH")
    if sample.frame != contract.frame or sample.tcp_point != contract.tcp_point:
        raise ValueError("FRAME_OR_POINT_MISMATCH")
    if (sample.effort_semantics, sample.model_semantics, sample.units, sample.jacobian_order) != (
            "actuator_on_robot", "lhs_Mqdd_plus_c_plus_g", "generalized_SI", "linear_then_angular"):
        raise ValueError("UNSUPPORTED_SIGN_UNITS_OR_ORDER")
    keys = ("q", "dq", "effort", "mass", "coriolis", "gravity", "jacobian")
    if (not isinstance(sample.provenance, dict) or set(sample.provenance) != set(keys)
        or any(not isinstance(sample.provenance[k], str) or not sample.provenance[k].strip() for k in keys)
        or not isinstance(sample.evidence_id, str) or not sample.evidence_id.strip()):
        raise ValueError("MISSING_PROVENANCE")
    n = len(contract.joint_names)
    q = _array(sample.position_SI, (n,), "INVALID_Q")
    dq = _array(sample.velocity_SI, (n,), "INVALID_VELOCITY")
    tau = _array(sample.actuator_generalized_effort_SI, (n,), "INVALID_EFFORT")
    mass = _array(sample.mass_matrix, (n, n), "INVALID_MASS_MATRIX")
    cor = _array(sample.coriolis_lhs_SI, (n,), "INVALID_CORIOLIS")
    grav = _array(sample.gravity_lhs_SI, (n,), "INVALID_GRAVITY")
    J = _array(sample.jacobian, (6, 7), "INVALID_JACOBIAN")
    mq = _array(sample.model_position_SI, (n,), "INVALID_MODEL_Q")
    mdq = _array(sample.model_velocity_SI, (n,), "INVALID_MODEL_VELOCITY")
    jq = _array(sample.jacobian_position_SI, (7,), "INVALID_JACOBIAN_Q")
    ix = [contract.joint_names.index(name) for name in contract.arm_joint_names]
    if not np.array_equal(q, mq) or not np.array_equal(dq, mdq):
        raise ValueError("MODEL_STATE_MISMATCH")
    if not np.array_equal(q[ix], jq):
        raise ValueError("JACOBIAN_STATE_MISMATCH")
    if not np.allclose(mass, mass.T, atol=1e-10, rtol=1e-10):
        raise ValueError("NONSYMMETRIC_MASS_MATRIX")
    try:
        eigenvalues = np.linalg.eigvalsh(mass)
        if not np.isfinite(eigenvalues).all():
            raise ValueError("INVALID_MASS_MATRIX")
        if eigenvalues[0] <= 0:
            raise ValueError("NONPOSITIVE_MASS_MATRIX")
    except np.linalg.LinAlgError:
        raise ValueError("INVALID_MASS_MATRIX") from None
    t = _array([sample.q_time_s, sample.velocity_time_s, sample.effort_time_s,
                sample.model_time_s, sample.jacobian_time_s, sample.evaluation_time_s], (6,), "INVALID_TIMESTAMP")
    if np.any(t < 0) or np.any(t[:5] > t[5]):
        raise ValueError("FUTURE_OR_NEGATIVE_TIMESTAMP")
    age = float(np.max(t[5] - t[:5]))
    skew = float(np.ptp(t[:5]))
    if age > contract.max_age_s:
        raise ValueError("STALE_SAMPLE")
    if skew > contract.max_skew_s:
        raise ValueError("ASYNCHRONOUS_SAMPLE")
    history = VelocityHistory(contract.joint_names, tuple(float(v) for v in dq), float(t[1]),
                              sample.evidence_id, sample.provenance["dq"], tuple(map(float, t[:5])))
    if previous is None:
        return StepResult("WARMUP", history, contract.source_status, None, None, None, None,
                          None, age, skew, None, "No acceleration until a second valid sample.")
    if not isinstance(previous, VelocityHistory) or previous.names != contract.joint_names:
        raise ValueError("INVALID_VELOCITY_HISTORY")
    if (not isinstance(previous.evidence_id, str) or not previous.evidence_id.strip()
        or not isinstance(previous.provenance, str) or previous.provenance != sample.provenance["dq"]):
        raise ValueError("INCONSISTENT_VELOCITY_PROVENANCE")
    prev_dq = _array(previous.velocity_SI, (n,), "INVALID_VELOCITY_HISTORY")
    prev_t = _array([previous.timestamp_s], (1,), "INVALID_VELOCITY_HISTORY")[0]
    if previous.evidence_id == sample.evidence_id:
        raise ValueError("DUPLICATE_EVIDENCE")
    previous_times = _array(previous.signal_times_s, (5,), "INVALID_VELOCITY_HISTORY")
    if np.any(t[:5] <= previous_times):
        raise ValueError("NONMONOTONIC_TIMESTAMP")
    dt = float(t[1] - prev_t)
    if prev_t < 0 or dt <= 0:
        raise ValueError("NONMONOTONIC_TIMESTAMP")
    if dt < contract.min_dt_s or dt > contract.max_dt_s:
        raise ValueError("DERIVATIVE_INTERVAL_OUT_OF_BOUNDS")
    with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
        acceleration = (dq - prev_dq) / dt
        inertia = mass @ acceleration
        ext = inertia + cor + grav - tau
    if not all(np.all(np.isfinite(x)) for x in (acceleration, inertia, ext)):
        raise ValueError("NUMERICAL_FAILURE")
    arm_q = q[ix]
    wc = WrenchContract(contract.arm_joint_names, contract.frame, contract.tcp_point,
                        contract.max_age_s, contract.max_skew_s, contract.rank_rtol)
    # The existing SVD API accepts only the historical token below; the token
    # is an algebra-interface tag, not certification of this candidate input.
    # This function labels its result as a candidate regardless of the claim.
    observation = Observation(arm_q, ext[ix], J, arm_q, contract.arm_joint_names,
            contract.arm_joint_names, contract.frame, contract.tcp_point,
            float(t[0]), float(t[2]), float(t[4]), float(t[5]),
            "validated_model_external_torque",
            "candidate model source claim " + contract.source_status + " evidence " + contract.source_evidence_sha256,
            sample.provenance["jacobian"])
    wrench = estimate(observation, wc)
    if wrench.rank < 6:
        status = "RANK_DEFICIENT_NONUNIQUE"
    elif wrench.condition_number is None or wrench.condition_number > contract.max_condition:
        status = "ILL_CONDITIONED"
    elif wrench.reconstruction_residual_Nm > contract.max_tcp_residual_Nm:
        status = "TORQUE_OUTSIDE_TCP_MODEL"
    else:
        status = "CANDIDATE_ESTIMATE"
    return StepResult(status, history, contract.source_status, tuple(acceleration), tuple(inertia),
                      tuple(ext), tuple(ext[ix]), wrench, age, skew, dt,
                      "TCP-equivalent fit only; model/source validity and contact location are external assumptions.")
