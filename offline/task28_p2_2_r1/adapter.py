"""Paired static Isaac solver actuator efforts to on-robot generalized torque.

Static equilibrium is tau_actuator + tau_gravity + J.T @ wrench_on_robot = 0.
For comparable *settled* loaded and unloaded configurations, if gravity and
other terms cancel, the on-robot external torque is -delta(tau_actuator).
The conversion is only justified when an independent, source-specific
equilibrium control has established that the reported effort is actuator-like.
It is not dynamic compensation, hardware tau_J calibration, or force control.
Frames affect J and the wrench; the 7 generalized joint torques are invariant
under a spatial frame change, but depend on the exact arm joint order.
"""
from dataclasses import dataclass
import re
import numpy as np

from offline.task28_p2_2.estimator import Contract, Observation, estimate


SOURCE = "isaac_sim_4_5_single_articulation_measured_joint_efforts"
SEMANTIC = "actuator_effort_loaded_minus_unloaded_opposes_wrench_on_robot"


@dataclass(frozen=True)
class SignCalibration:
    """Explicit provenance claim, not a self-verifying certificate.

    Evidence must come from an independent equilibrium control, not a fit to
    the six-axis cases. This class checks the claim is present, not that the
    cited file or claim is true. The caller must verify its hash and conclusion.
    """
    source: str
    semantic: str
    independent_equilibrium_confirmed: bool
    evidence_sha256: str


@dataclass(frozen=True)
class StaticWindow:
    q_rad: object
    dq_rad_s: object
    effort_Nm: object
    timestamps_s: object
    evaluation_timestamp_s: float
    joint_names: tuple[str, ...]
    source: str
    q_units: str = "rad"
    dq_units: str = "rad/s"
    effort_units: str = "N*m"


@dataclass(frozen=True)
class StaticLimits:
    max_age_s: float
    max_abs_velocity_rad_s: float
    max_window_q_range_rad: float
    max_window_effort_std_Nm: float
    max_pair_mean_q_difference_rad: float


def _finite_array(value, shape, reason):
    try:
        arr = np.asarray(value)
        if (arr.shape != shape or not np.issubdtype(arr.dtype, np.number)
                or np.issubdtype(arr.dtype, np.complexfloating)
                or np.issubdtype(arr.dtype, np.bool_)):
            raise ValueError(reason)
        arr = arr.astype(float)
        if not np.isfinite(arr).all():
            raise ValueError(reason)
        return arr
    except (TypeError, OverflowError):
        raise ValueError(reason) from None


def _window(window, names, limits):
    if not isinstance(window, StaticWindow):
        raise ValueError("INVALID_WINDOW")
    if window.joint_names != names:
        raise ValueError("JOINT_ORDER_MISMATCH")
    if window.source != SOURCE:
        raise ValueError("UNSUPPORTED_EFFORT_SOURCE")
    if (window.q_units, window.dq_units, window.effort_units) != ("rad", "rad/s", "N*m"):
        raise ValueError("UNSUPPORTED_UNITS")
    t = np.asarray(window.timestamps_s)
    if t.ndim != 1 or t.size < 2:
        raise ValueError("INVALID_WINDOW_TIMESTAMPS")
    t = _finite_array(t, (t.size,), "INVALID_WINDOW_TIMESTAMPS")
    q = _finite_array(window.q_rad, (t.size, 7), "INVALID_Q")
    dq = _finite_array(window.dq_rad_s, (t.size, 7), "INVALID_DQ")
    tau = _finite_array(window.effort_Nm, (t.size, 7), "INVALID_EFFORT")
    now = _finite_array([window.evaluation_timestamp_s], (1,), "INVALID_TIMESTAMP")[0]
    if np.any(t < 0) or now < 0 or np.any(np.diff(t) <= 0) or np.any(t > now):
        raise ValueError("INVALID_OR_OUT_OF_ORDER_TIMESTAMP")
    if np.any(now - t > limits.max_age_s):
        raise ValueError("STALE_OBSERVATION")
    if np.max(np.abs(dq)) > limits.max_abs_velocity_rad_s:
        raise ValueError("NOT_QUASISTATIC")
    if np.max(np.ptp(q, axis=0)) > limits.max_window_q_range_rad:
        raise ValueError("UNSTABLE_WINDOW_Q")
    if np.max(np.std(tau, axis=0)) > limits.max_window_effort_std_Nm:
        raise ValueError("UNSTABLE_WINDOW_EFFORT")
    return q.mean(axis=0), tau.mean(axis=0), float(t.mean())


def static_actuator_pair_to_wrench(unloaded, loaded, jacobian, *, contract,
                                   limits, calibration, jacobian_q,
                                   jacobian_joint_names, frame, tcp_point,
                                   jacobian_provenance):
    """Convert one offline paired trial and call the existing SVD estimator.

    Returns raw delta, converted external torque, and full observability /
    reconstruction diagnostics. Neither the applied load nor any oracle is an
    input. q/J are at the loaded window mean in rad; Jacobian maps qdot to
    [linear, angular] TCP velocity in the caller-specified frame. Wrench is
    [Fx,Fy,Fz,Mx,My,Mz] on the robot, in that frame, about that TCP, N and N m.
    Separate reset-trial clocks are checked within each window, not compared.
    """
    if not isinstance(contract, Contract) or not isinstance(limits, StaticLimits):
        raise ValueError("INVALID_CONTRACT_OR_LIMITS")
    if not isinstance(calibration, SignCalibration) or calibration.source != SOURCE or calibration.semantic != SEMANTIC:
        raise ValueError("UNSUPPORTED_OR_UNCALIBRATED_SIGN")
    if calibration.independent_equilibrium_confirmed is not True or not isinstance(calibration.evidence_sha256, str) or not re.fullmatch(r"[0-9a-f]{64}", calibration.evidence_sha256):
        raise ValueError("MISSING_INDEPENDENT_SIGN_CALIBRATION")
    names = contract.joint_names
    if not isinstance(names, tuple) or len(names) != 7 or len(set(names)) != 7 or any(not isinstance(n, str) or not n for n in names):
        raise ValueError("INVALID_JOINT_NAMES")
    if jacobian_joint_names != names:
        raise ValueError("JOINT_ORDER_MISMATCH")
    if frame != contract.frame or tcp_point != contract.tcp_point:
        raise ValueError("FRAME_OR_POINT_MISMATCH")
    if not isinstance(jacobian_provenance, str) or not jacobian_provenance.strip():
        raise ValueError("MISSING_JACOBIAN_PROVENANCE")
    lim = _finite_array([limits.max_age_s, limits.max_abs_velocity_rad_s,
                         limits.max_window_q_range_rad, limits.max_window_effort_std_Nm,
                         limits.max_pair_mean_q_difference_rad], (5,), "INVALID_LIMITS")
    if np.any(lim < 0):
        raise ValueError("INVALID_LIMITS")
    q0, tau0, _ = _window(unloaded, names, limits)
    q1, tau1, t1 = _window(loaded, names, limits)
    if np.max(np.abs(q1 - q0)) > limits.max_pair_mean_q_difference_rad:
        raise ValueError("PAIR_Q_MISMATCH")
    jq = _finite_array(jacobian_q, (7,), "INVALID_JACOBIAN_Q")
    if not np.array_equal(jq, q1):
        raise ValueError("JACOBIAN_CONFIGURATION_MISMATCH")
    delta_actuator = tau1 - tau0
    tau_external_on_robot = -delta_actuator
    obs = Observation(q1, tau_external_on_robot, jacobian, jq, names, names,
                      frame, tcp_point, t1, t1, t1, t1,
                      "paired_static_load_difference",
                      "negative Isaac actuator-like loaded-minus-unloaded mean; independent sign evidence sha256=" + calibration.evidence_sha256,
                      jacobian_provenance)
    result = estimate(obs, contract)
    return {"unloaded_mean_q_rad": q0.tolist(), "loaded_mean_q_rad": q1.tolist(),
            "unloaded_mean_effort_Nm": tau0.tolist(), "loaded_mean_effort_Nm": tau1.tolist(),
            "delta_actuator_effort_Nm": delta_actuator.tolist(),
            "external_generalized_torque_on_robot_Nm": tau_external_on_robot.tolist(),
            "calibration_evidence_sha256": calibration.evidence_sha256,
            "observation_source": SOURCE, "semantic": SEMANTIC,
            "joint_names": names, "frame": frame, "tcp_point": tcp_point,
            "estimate": result}
