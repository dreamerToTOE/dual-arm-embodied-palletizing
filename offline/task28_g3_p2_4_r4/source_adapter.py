"""Strict source/semantics boundary for the G3 external-torque observer.

The existing P2-4 observer computes a *candidate* residual from
``M*qdd + c + g - effort``.  This module prevents callers from silently
substituting one effort source for another.  It is deliberately independent
of Isaac Sim and libfranka so it can be unit-tested with static synthetic
signals.

No source accepted here is a declaration of physical contact-force validity.
Simulation signals are marked ``SIMULATION_CANDIDATE``; only a descriptor that
contains the exact libfranka ``RobotState.tau_J`` identity and an explicit
``REAL_API_VALIDATED`` status can be labelled as real ``tau_J``.  A model
inferred torque or an observer external-torque estimate can never be accepted
as an actuator effort input.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Mapping

import numpy as np


_HEX64 = re.compile(r"^[0-9a-f]{64}$")


class SourceKind(str, Enum):
    """Physically distinct signal origins/quantities."""

    SIMULATION_DRIVE_EFFORT = "simulation_drive_effort"
    SIMULATION_APPLIED_EFFORT = "simulation_applied_effort"
    SIMULATION_PROJECTED_EFFORT = "simulation_projected_effort"
    SIMULATION_JOINT_REACTION_TORQUE = "simulation_joint_reaction_torque"
    MODEL_INFERRED_TORQUE = "model_inferred_torque"
    REAL_FR3_TAU_J = "real_fr3_tau_J"
    ESTIMATED_EXTERNAL_TORQUE = "estimated_external_torque"


class SourceValidation(str, Enum):
    SYNTHETIC_ONLY = "SYNTHETIC_ONLY"
    SIMULATION_CANDIDATE = "SIMULATION_CANDIDATE"
    REAL_API_VALIDATED = "REAL_API_VALIDATED"


class AdapterRole(str, Enum):
    """The only consumers permitted by this boundary."""

    DYNAMICS_EFFORT_INPUT = "dynamics_effort_input"
    MODEL_TERM_INPUT = "model_term_input"
    EXTERNAL_TORQUE_ESTIMATE = "external_torque_estimate"


@dataclass(frozen=True)
class SourceDescriptor:
    """Immutable declaration of a signal's physical semantics.

    ``frame`` is the generalized-coordinate frame (``joint``), not a TCP or
    world wrench frame.  A six-dimensional Cartesian wrench is outside this
    adapter and must carry its own frame contract before projection.
    """

    kind: SourceKind | str
    physical_definition: str
    sign_convention: str
    units: str
    joint_order: tuple[str, ...]
    frame: str
    timestamp_field: str
    sample_period_s: float
    source_identity: str
    validation: SourceValidation | str
    evidence_sha256: str
    origin: str
    declared_freshness: str = "per_sample"


@dataclass(frozen=True)
class SourceSample:
    """One signal sample from a declared source.

    ``timestamp_s`` is the source clock timestamp.  ``arrival_time_s`` is an
    optional same-clock evaluation/arrival timestamp used for age checking.
    ``sequence`` must be strictly increasing for a given adapter instance.
    """

    values: object
    timestamp_s: float
    sequence: int
    dt_s: float | None = None
    arrival_time_s: float | None = None
    valid: bool = True
    quality_flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class AdapterContract:
    """Hard acceptance limits; all failures are fail-closed."""

    joint_order: tuple[str, ...]
    expected_frame: str = "joint"
    expected_units: str = "N*m"
    expected_sign_convention: str = "positive_on_robot"
    nominal_dt_s: float = 1.0 / 60.0
    dt_tolerance_s: float = 1.0e-6
    max_age_s: float = 2.0e-3
    max_abs_effort_Nm: float = 100.0
    max_step_Nm: float = 20.0
    require_evidence_hash: bool = True


@dataclass(frozen=True)
class AdaptedSignal:
    """Accepted signal with source semantics retained end-to-end."""

    values_Nm: tuple[float, ...]
    source_kind: str
    source_identity: str
    role: str
    source_validation: str
    physical_definition: str
    sign_convention: str
    units: str
    joint_order: tuple[str, ...]
    frame: str
    timestamp_s: float
    dt_s: float | None
    sequence: int
    age_s: float | None
    finite: bool
    valid: bool
    is_real_fr3_tau_J: bool
    is_simulation_source: bool
    is_candidate_estimate: bool
    diagnostics: tuple[str, ...]


class SourceAdapterError(ValueError):
    """Raised for any incompatible, stale, ambiguous, or unsafe source."""


# Canonical semantics.  These strings are intentionally exact: changing a
# sign, unit, frame, or definition requires a new adapter review and tests.
_CANONICAL: dict[SourceKind, dict[str, str]] = {
    SourceKind.SIMULATION_DRIVE_EFFORT: {
        "physical_definition": "Isaac articulation drive effort command applied over the following interval; not a sensor torque",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "isaacsim.SingleArticulation.set_joint_efforts",
        "origin": "isaac_simulation",
    },
    SourceKind.SIMULATION_APPLIED_EFFORT: {
        "physical_definition": "Isaac articulation effort readback actually applied by the physics actuator",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "isaacsim.get_applied_joint_efforts",
        "origin": "isaac_simulation",
    },
    SourceKind.SIMULATION_PROJECTED_EFFORT: {
        "physical_definition": "Isaac projected/measured generalized joint effort from the solver; not a motor sensor",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "isaacsim.get_measured_joint_efforts",
        "origin": "isaac_simulation",
    },
    SourceKind.SIMULATION_JOINT_REACTION_TORQUE: {
        "physical_definition": "Isaac joint reaction torque reported by the physics joint/body interface",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "isaacsim.joint_reaction_torque",
        "origin": "isaac_simulation",
    },
    SourceKind.MODEL_INFERRED_TORQUE: {
        "physical_definition": "model-inferred LHS torque M(q)qdd+c(q,qd)+g(q), not actuator effort",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "p2_4_model_inference",
        "origin": "offline_model",
    },
    SourceKind.REAL_FR3_TAU_J: {
        "physical_definition": "libfranka RobotState.tau_J measured joint torque on the robot",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "libfranka::RobotState.tau_J",
        "origin": "real_fr3",
    },
    SourceKind.ESTIMATED_EXTERNAL_TORQUE: {
        "physical_definition": "observer residual estimate of external generalized torque on the robot",
        "sign_convention": "positive_on_robot",
        "units": "N*m",
        "frame": "joint",
        "source_identity": "task28_p2_4_observer.external_torque_estimate",
        "origin": "offline_observer",
    },
}


def _canonical_kind(kind: SourceKind | str) -> SourceKind:
    try:
        return kind if isinstance(kind, SourceKind) else SourceKind(kind)
    except (TypeError, ValueError):
        raise SourceAdapterError("UNKNOWN_SOURCE_KIND") from None


def _finite_scalar(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        raise SourceAdapterError(label) from None
    if not math.isfinite(result):
        raise SourceAdapterError(label)
    return result


def _array(values: object, n: int, label: str) -> np.ndarray:
    try:
        raw = np.asarray(values)
    except (TypeError, ValueError):
        raise SourceAdapterError(label) from None
    if raw.shape != (n,) or not np.issubdtype(raw.dtype, np.number):
        raise SourceAdapterError(label)
    if np.issubdtype(raw.dtype, np.complexfloating) or np.issubdtype(raw.dtype, np.bool_):
        raise SourceAdapterError(label)
    try:
        result = raw.astype(float)
    except (TypeError, ValueError, OverflowError):
        raise SourceAdapterError(label) from None
    if not np.all(np.isfinite(result)):
        raise SourceAdapterError("NONFINITE_VALUES")
    return result


def _check_contract(contract: AdapterContract) -> None:
    if not isinstance(contract, AdapterContract):
        raise SourceAdapterError("INVALID_CONTRACT")
    names = contract.joint_order
    if (not isinstance(names, tuple) or not names or
            any(not isinstance(x, str) or not x.strip() for x in names) or
            len(set(names)) != len(names)):
        raise SourceAdapterError("INVALID_JOINT_ORDER")
    if contract.expected_frame != "joint" or contract.expected_units != "N*m" or contract.expected_sign_convention != "positive_on_robot":
        raise SourceAdapterError("UNSUPPORTED_CONTRACT_SEMANTICS")
    for value, label in ((contract.nominal_dt_s, "INVALID_NOMINAL_DT"),
                         (contract.dt_tolerance_s, "INVALID_DT_TOLERANCE"),
                         (contract.max_age_s, "INVALID_MAX_AGE"),
                         (contract.max_abs_effort_Nm, "INVALID_MAX_EFFORT"),
                         (contract.max_step_Nm, "INVALID_MAX_STEP")):
        value = _finite_scalar(value, label)
        if value <= 0:
            raise SourceAdapterError(label)
    if contract.dt_tolerance_s >= contract.nominal_dt_s:
        raise SourceAdapterError("INVALID_DT_TOLERANCE")


def canonical_descriptor(
    kind: SourceKind | str,
    joint_order: tuple[str, ...],
    *,
    evidence_sha256: str = "0" * 64,
    validation: SourceValidation | str | None = None,
    source_identity: str | None = None,
    timestamp_field: str = "sim_time_s",
    sample_period_s: float = 1.0 / 60.0,
) -> SourceDescriptor:
    """Build a descriptor from immutable canonical semantics.

    The helper refuses identity/semantic overrides.  This makes an Isaac
    signal unable to masquerade as ``tau_J`` by changing a label at a call
    site.  Real ``tau_J`` requires the explicit API validation status.
    """

    ck = _canonical_kind(kind)
    spec = _CANONICAL[ck]
    if source_identity is not None and source_identity != spec["source_identity"]:
        raise SourceAdapterError("SOURCE_IDENTITY_MISMATCH")
    if not isinstance(joint_order, tuple) or not joint_order or len(set(joint_order)) != len(joint_order):
        raise SourceAdapterError("INVALID_JOINT_ORDER")
    if not _HEX64.fullmatch(str(evidence_sha256)):
        raise SourceAdapterError("MISSING_SOURCE_EVIDENCE")
    if validation is None:
        validation = (SourceValidation.REAL_API_VALIDATED if ck is SourceKind.REAL_FR3_TAU_J
                      else SourceValidation.SIMULATION_CANDIDATE if spec["origin"] == "isaac_simulation"
                      else SourceValidation.SYNTHETIC_ONLY)
    try:
        validation = SourceValidation(validation)
    except (TypeError, ValueError):
        raise SourceAdapterError("UNKNOWN_SOURCE_VALIDATION") from None
    if ck is SourceKind.REAL_FR3_TAU_J and validation is not SourceValidation.REAL_API_VALIDATED:
        raise SourceAdapterError("REAL_TAU_J_REQUIRES_API_VALIDATION")
    if ck is SourceKind.REAL_FR3_TAU_J and timestamp_field != "franka_control_time_s":
        raise SourceAdapterError("REAL_TAU_J_CLOCK_REQUIRED")
    if ck is not SourceKind.REAL_FR3_TAU_J and validation is SourceValidation.REAL_API_VALIDATED:
        raise SourceAdapterError("NONREAL_SOURCE_CANNOT_BE_REAL_VALIDATED")
    period = _finite_scalar(sample_period_s, "INVALID_SAMPLE_PERIOD")
    if period <= 0:
        raise SourceAdapterError("INVALID_SAMPLE_PERIOD")
    return SourceDescriptor(
        kind=ck,
        physical_definition=spec["physical_definition"],
        sign_convention=spec["sign_convention"],
        units=spec["units"],
        joint_order=joint_order,
        frame=spec["frame"],
        timestamp_field=timestamp_field,
        sample_period_s=period,
        source_identity=spec["source_identity"],
        validation=validation,
        evidence_sha256=str(evidence_sha256),
        origin=spec["origin"],
    )


def describe_source(descriptor: SourceDescriptor) -> dict[str, object]:
    """Return a serializable source definition for reports/evidence."""

    kind = _canonical_kind(descriptor.kind)
    return {
        "kind": kind.value,
        "physical_definition": descriptor.physical_definition,
        "sign_convention": descriptor.sign_convention,
        "units": descriptor.units,
        "joint_order": list(descriptor.joint_order),
        "frame": descriptor.frame,
        "timestamp_field": descriptor.timestamp_field,
        "sample_period_s": descriptor.sample_period_s,
        "source_identity": descriptor.source_identity,
        "validation": SourceValidation(descriptor.validation).value,
        "evidence_sha256": descriptor.evidence_sha256,
        "origin": descriptor.origin,
        "declared_freshness": descriptor.declared_freshness,
    }


class SourceAdapter:
    """Stateful, fail-closed source adapter.

    Rejected samples never update the sequence/timestamp/value cursor.  This
    is required so a later sample cannot hide a stale or malformed interval.
    """

    def __init__(self, contract: AdapterContract):
        _check_contract(contract)
        self.contract = contract
        self._last_sequence: int | None = None
        self._last_timestamp_s: float | None = None
        self._last_values: np.ndarray | None = None

    @property
    def last_sequence(self) -> int | None:
        return self._last_sequence

    def reset(self) -> None:
        self._last_sequence = None
        self._last_timestamp_s = None
        self._last_values = None

    def _validate_descriptor(self, descriptor: SourceDescriptor, role: AdapterRole) -> SourceKind:
        if not isinstance(descriptor, SourceDescriptor):
            raise SourceAdapterError("INVALID_SOURCE_DESCRIPTOR")
        kind = _canonical_kind(descriptor.kind)
        spec = _CANONICAL[kind]
        try:
            validation = SourceValidation(descriptor.validation)
        except (TypeError, ValueError):
            raise SourceAdapterError("UNKNOWN_SOURCE_VALIDATION") from None
        if (descriptor.physical_definition != spec["physical_definition"] or
                descriptor.sign_convention != spec["sign_convention"] or
                descriptor.units != spec["units"] or
                descriptor.frame != spec["frame"] or
                descriptor.source_identity != spec["source_identity"] or
                descriptor.origin != spec["origin"]):
            raise SourceAdapterError("SOURCE_SEMANTICS_MISMATCH")
        if descriptor.joint_order != self.contract.joint_order:
            raise SourceAdapterError("JOINT_ORDER_MISMATCH")
        if descriptor.timestamp_field not in ("sim_time_s", "monotonic_time_s", "franka_control_time_s"):
            raise SourceAdapterError("UNSUPPORTED_TIMESTAMP_FIELD")
        period = _finite_scalar(descriptor.sample_period_s, "INVALID_SAMPLE_PERIOD")
        if abs(period - self.contract.nominal_dt_s) > self.contract.dt_tolerance_s:
            raise SourceAdapterError("SOURCE_PERIOD_MISMATCH")
        if self.contract.require_evidence_hash and not _HEX64.fullmatch(descriptor.evidence_sha256):
            raise SourceAdapterError("MISSING_SOURCE_EVIDENCE")
        if descriptor.declared_freshness != "per_sample":
            raise SourceAdapterError("UNSUPPORTED_FRESHNESS_DECLARATION")
        if kind is SourceKind.REAL_FR3_TAU_J:
            if validation is not SourceValidation.REAL_API_VALIDATED:
                raise SourceAdapterError("REAL_TAU_J_REQUIRES_API_VALIDATION")
            if descriptor.timestamp_field != "franka_control_time_s":
                raise SourceAdapterError("REAL_TAU_J_CLOCK_REQUIRED")
        elif validation is SourceValidation.REAL_API_VALIDATED:
            raise SourceAdapterError("NONREAL_SOURCE_CANNOT_BE_REAL_VALIDATED")
        if role is AdapterRole.DYNAMICS_EFFORT_INPUT:
            allowed = {
                SourceKind.SIMULATION_DRIVE_EFFORT,
                SourceKind.SIMULATION_APPLIED_EFFORT,
                SourceKind.SIMULATION_PROJECTED_EFFORT,
                SourceKind.SIMULATION_JOINT_REACTION_TORQUE,
                SourceKind.REAL_FR3_TAU_J,
            }
            if kind not in allowed:
                raise SourceAdapterError("INCOMPATIBLE_SOURCE_ROLE")
        elif role is AdapterRole.MODEL_TERM_INPUT:
            if kind is not SourceKind.MODEL_INFERRED_TORQUE:
                raise SourceAdapterError("INCOMPATIBLE_SOURCE_ROLE")
        elif role is AdapterRole.EXTERNAL_TORQUE_ESTIMATE:
            if kind is not SourceKind.ESTIMATED_EXTERNAL_TORQUE:
                raise SourceAdapterError("INCOMPATIBLE_SOURCE_ROLE")
        else:
            raise SourceAdapterError("UNKNOWN_ADAPTER_ROLE")
        return kind

    def accept(
        self,
        descriptor: SourceDescriptor,
        sample: SourceSample,
        *,
        role: AdapterRole = AdapterRole.DYNAMICS_EFFORT_INPUT,
        now_s: float | None = None,
    ) -> AdaptedSignal:
        """Validate and return a traceable signal; reject without state update."""

        _check_contract(self.contract)
        kind = self._validate_descriptor(descriptor, role)
        if not isinstance(sample, SourceSample):
            raise SourceAdapterError("INVALID_SOURCE_SAMPLE")
        if not isinstance(sample.sequence, (int, np.integer)) or int(sample.sequence) < 0:
            raise SourceAdapterError("INVALID_SEQUENCE")
        sequence = int(sample.sequence)
        if self._last_sequence is not None and sequence <= self._last_sequence:
            raise SourceAdapterError("NONMONOTONIC_SEQUENCE")
        t = _finite_scalar(sample.timestamp_s, "INVALID_TIMESTAMP")
        if t < 0:
            raise SourceAdapterError("INVALID_TIMESTAMP")
        if now_s is None:
            now = t
        else:
            now = _finite_scalar(now_s, "INVALID_ARRIVAL_TIME")
            if now < t:
                raise SourceAdapterError("FUTURE_SAMPLE")
        age = now - t
        if age > self.contract.max_age_s:
            raise SourceAdapterError("STALE_SAMPLE")
        if sample.arrival_time_s is not None:
            arrival = _finite_scalar(sample.arrival_time_s, "INVALID_ARRIVAL_TIME")
            if arrival < t or abs(arrival - now) > self.contract.max_age_s:
                raise SourceAdapterError("ARRIVAL_TIME_MISMATCH")
        if sample.valid is not True:
            raise SourceAdapterError("SOURCE_MARKED_INVALID")
        if any(not isinstance(flag, str) or not flag.strip() for flag in sample.quality_flags):
            raise SourceAdapterError("INVALID_QUALITY_FLAGS")
        if sample.quality_flags:
            raise SourceAdapterError("SOURCE_QUALITY_FLAGGED")
        values = _array(sample.values, len(self.contract.joint_order), "INVALID_SOURCE_VALUES")
        if np.any(np.abs(values) > self.contract.max_abs_effort_Nm):
            raise SourceAdapterError("EFFORT_ABS_LIMIT")
        dt: float | None = None
        if self._last_timestamp_s is not None:
            dt = t - self._last_timestamp_s
            if dt <= 0:
                raise SourceAdapterError("NONMONOTONIC_TIMESTAMP")
            if abs(dt - self.contract.nominal_dt_s) > self.contract.dt_tolerance_s:
                raise SourceAdapterError("DT_OUT_OF_BOUNDS")
            if sample.dt_s is not None:
                declared_dt = _finite_scalar(sample.dt_s, "INVALID_DECLARED_DT")
                if abs(declared_dt - dt) > self.contract.dt_tolerance_s:
                    raise SourceAdapterError("DECLARED_DT_MISMATCH")
            if self._last_values is not None and np.any(np.abs(values - self._last_values) > self.contract.max_step_Nm):
                raise SourceAdapterError("EFFORT_STEP_LIMIT")
        elif sample.dt_s is not None:
            declared_dt = _finite_scalar(sample.dt_s, "INVALID_DECLARED_DT")
            if abs(declared_dt - self.contract.nominal_dt_s) > self.contract.dt_tolerance_s:
                raise SourceAdapterError("DECLARED_DT_MISMATCH")
        self._last_sequence = sequence
        self._last_timestamp_s = t
        self._last_values = values.copy()
        validation = SourceValidation(descriptor.validation)
        is_sim = descriptor.origin == "isaac_simulation"
        is_real = kind is SourceKind.REAL_FR3_TAU_J
        candidate = is_sim or validation is not SourceValidation.REAL_API_VALIDATED
        diagnostics = ["SOURCE_SEMANTICS_BOUND", "FINITE", "FRESH", "MONOTONIC"]
        if self._last_sequence == sequence and dt is None:
            diagnostics.append("WARMUP")
        return AdaptedSignal(
            values_Nm=tuple(float(x) for x in values),
            source_kind=kind.value,
            source_identity=descriptor.source_identity,
            role=role.value,
            source_validation=validation.value,
            physical_definition=descriptor.physical_definition,
            sign_convention=descriptor.sign_convention,
            units=descriptor.units,
            joint_order=descriptor.joint_order,
            frame=descriptor.frame,
            timestamp_s=t,
            dt_s=dt,
            sequence=sequence,
            age_s=age,
            finite=True,
            valid=True,
            is_real_fr3_tau_J=is_real,
            is_simulation_source=is_sim,
            is_candidate_estimate=candidate,
            diagnostics=tuple(diagnostics),
        )


def source_registry() -> dict[str, dict[str, str]]:
    """Return canonical registry suitable for a machine-readable evidence file."""

    return {kind.value: dict(spec) for kind, spec in _CANONICAL.items()}
