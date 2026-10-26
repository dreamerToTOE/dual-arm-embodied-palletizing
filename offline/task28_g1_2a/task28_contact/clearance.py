"""R1: observed-Cube clearance, hysteresis and temporal phase confirmation.

Geometric contact classification and controller confirmation are orthogonal.
All support outputs are geometric candidates, never measured force/support.
Only validated observation poses enter the geometry; there is no command-pose
input. No time interpolation, path or swept collision checking is performed.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from task28_validator.se3 import pose_matrix
from .clearance_geometry import ClearanceGeometry, ClearanceGeometryResult
from .modes import (OperationMode, Snapshot, SupportSite, TransitionEvent,
                    validate_snapshot_metadata)
from .validator import ContactModeValidator, ModeValidationResult


class ClearancePhase(str, Enum):
    INITIALIZING = 'initializing'
    DETACHED_SUPPORTED_CONFIRMED = 'detached_supported_confirmed'
    ATTACHING = 'attaching'
    BILATERAL_SUPPORTED_CONFIRMED = 'bilateral_supported_confirmed'
    LIFTING = 'lifting'
    AIRBORNE_CONFIRMED = 'airborne_confirmed'
    PLACING = 'placing'
    NEAR_TABLE_CONFIRMED = 'near_table_confirmed'
    RELEASING = 'releasing'
    RELEASED_SUPPORTED_CONFIRMED = 'released_supported_confirmed'


class ConfirmationEvent(str, Enum):
    OBSERVE = 'observe'
    ATTACH = 'attach'
    BEGIN_LIFT = 'begin_lift'
    BEGIN_PLACE = 'begin_place'
    RELEASE = 'release'


class CubePoseSource(str, Enum):
    SYNTHETIC_TEST_INPUT = 'synthetic_test_input'
    EXTERNAL_OBSERVATION = 'external_observation'


class GeometricContact(str, Enum):
    SUPPORT_CANDIDATE = 'strict_geometric_support_candidate'
    SEPARATED = 'geometrically_separated'
    INVALID = 'invalid'


@dataclass(frozen=True)
class ClearanceFrame:
    snapshot: Snapshot
    cube_evidence_id: str
    cube_pose_source: CubePoseSource


@dataclass(frozen=True)
class ClearanceParameters:
    """Provisional research values; independent of the frozen 1 um contact gate."""
    d_low_m: float = .001
    d_high_m: float = .003
    confirmation_frames: int = 3
    max_observation_gap_s: float = .1

    def __post_init__(self):
        values = (self.d_low_m, self.d_high_m, self.max_observation_gap_s)
        if any(type(value) not in (float, int) or not math.isfinite(value) for value in values):
            raise ValueError('INVALID_CLEARANCE_PARAMETERS')
        if not (1e-6 < self.d_low_m < self.d_high_m and 0 < self.max_observation_gap_s <= .1):
            raise ValueError('INVALID_CLEARANCE_PARAMETER_BOUNDS')
        if type(self.confirmation_frames) is not int or self.confirmation_frames != 3:
            raise ValueError('THREE_CONFIRMATION_FRAMES_REQUIRED')


@dataclass(frozen=True)
class ClearanceObservationResult:
    valid: bool
    reasons: tuple[str, ...]
    phase: ClearancePhase
    phase_confirmed: bool
    contact: GeometricContact
    physical_support_observed: bool
    clearance_m: float | None
    high_count: int
    low_count: int
    support_count: int
    attachment_count: int
    release_count: int
    can_release: bool
    sequence_reset_reason: str | None
    geometry: ClearanceGeometryResult | None = None
    validation: ModeValidationResult | None = None


class TemporalClearanceConfirmer:
    """One offline A-stage session, with explicit attach/lift/place/release events.

    Invalid observations reset all consecutive counters and revoke authorization.
    Historical phase labels latch, but ``phase_confirmed``/``can_release`` become
    false until three fresh qualifying frames reestablish confirmation. Every
    attempted well-formed ID is consumed; all six component stamps must advance.
    Valid metadata advances the temporal watermark even if later geometry, event
    or collision checks fail. Excessive inter-frame gaps also reset counters.
    """
    def __init__(self, validator=None, parameters=None):
        if validator is not None and type(validator) is not ContactModeValidator:
            raise ValueError('FIXED_CONTACT_VALIDATOR_REQUIRED')
        self.validator = validator if validator is not None else ContactModeValidator()
        self.parameters = parameters if parameters is not None else ClearanceParameters()
        if type(self.parameters) is not ClearanceParameters:
            raise ValueError('TYPED_CLEARANCE_PARAMETERS_REQUIRED')
        self.parameters.__post_init__()
        self._parameter_binding = tuple(vars(self.parameters).items())
        self.geometry = ClearanceGeometry(self.validator.collision.support)
        self.phase = ClearancePhase.INITIALIZING
        self._confirmed = False
        self._counts = dict(high=0, low=0, support=0, attachment=0, release=0)
        self._cube_ids = set()
        self._feedback_ids = set()
        self._last_stamps = None
        self._last_now = None
        self._last_snapshot = None
        self._last_snapshot_now = None
        self._placement_started = False
        self._cube_pose_source = None

    def _reset(self):
        self._counts = dict.fromkeys(self._counts, 0)
        self._confirmed = False

    def _count(self, channel, qualifies):
        self._counts[channel] = min(3, self._counts[channel] + 1) if qualifies else 0
        return self._counts[channel] == 3

    @property
    def can_release(self):
        return (self._confirmed and self.phase is ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED
                and self._placement_started and self._last_snapshot is not None
                and self._last_snapshot.support_site is SupportSite.TARGET)

    def _result(self, valid, reasons=(), contact=GeometricContact.INVALID,
                geometry=None, validation=None, reset_reason=None):
        if not valid:
            self._reset()
        return ClearanceObservationResult(
            valid, tuple(reasons), self.phase, self._confirmed if valid else False,
            contact, False, None if geometry is None else geometry.clearance_m,
            self._counts['high'], self._counts['low'], self._counts['support'],
            self._counts['attachment'], self._counts['release'],
            self.can_release if valid else False, reset_reason, geometry, validation)

    def observe(self, frame, now_s, event=ConfirmationEvent.OBSERVE):
        try:
            if type(self.parameters) is not ClearanceParameters:
                raise ValueError('INVALID_CLEARANCE_PARAMETERS')
            self.parameters.__post_init__()
            if tuple(vars(self.parameters).items()) != self._parameter_binding:
                raise ValueError('CLEARANCE_PARAMETERS_CHANGED_MID_SESSION')
        except (ValueError, TypeError):
            return self._result(False, ('INVALID_CLEARANCE_PARAMETERS',))
        if type(frame) is not ClearanceFrame or type(frame.snapshot) is not Snapshot:
            return self._result(False, ('INVALID_CLEARANCE_FRAME',))
        if type(frame.cube_evidence_id) is not str or not frame.cube_evidence_id.strip():
            return self._result(False, ('INVALID_CUBE_EVIDENCE_PROVENANCE',))
        snapshot = frame.snapshot
        feedback_id = getattr(snapshot.observed, 'evidence_id', None)
        duplicate = frame.cube_evidence_id in self._cube_ids or (isinstance(feedback_id, str) and feedback_id in self._feedback_ids)
        self._cube_ids.add(frame.cube_evidence_id)
        if isinstance(feedback_id, str):
            self._feedback_ids.add(feedback_id)
        if duplicate:
            return self._result(False, ('REUSED_OBSERVATION_EVIDENCE',))
        if type(event) is not ConfirmationEvent:
            return self._result(False, ('INVALID_CONFIRMATION_EVENT',))
        if type(frame.cube_pose_source) is not CubePoseSource:
            return self._result(False, ('INVALID_CUBE_EVIDENCE_PROVENANCE',))
        if self._cube_pose_source is not None and frame.cube_pose_source is not self._cube_pose_source:
            return self._result(False, ('CUBE_PROVENANCE_CHANGED_MID_SESSION',))
        metadata = validate_snapshot_metadata(snapshot, now_s, self.validator.state_policy)
        if not metadata.valid:
            return self._result(False, metadata.reasons)
        keys = ('left_joints', 'right_joints', 'left_tcp', 'right_tcp', 'cube')
        stamps = tuple(float(getattr(snapshot.input_state, key).stamp_s) for key in keys) + (float(snapshot.observed.stamp_s),)
        if self._last_now is not None and (now_s <= self._last_now or
                any(new <= old for new, old in zip(stamps, self._last_stamps))):
            return self._result(False, ('NONINCREASING_OBSERVATION_TIME',))
        reset_reason = None
        if self._last_stamps is not None and max(new-old for new, old in zip(stamps, self._last_stamps)) > self.parameters.max_observation_gap_s:
            self._reset()
            reset_reason = 'OBSERVATION_GAP_RESET'
        self._last_now, self._last_stamps = float(now_s), stamps

        pose = pose_matrix(snapshot.input_state.cube.position_m, snapshot.input_state.cube.quaternion_xyzw)
        geometry = self.geometry.evaluate(pose)
        if not geometry.valid:
            return self._result(False, geometry.reasons, geometry=geometry, reset_reason=reset_reason)
        validation = self.validator.validate(snapshot, now_s)
        if not validation.valid:
            return self._result(False, validation.reasons, geometry=geometry, validation=validation, reset_reason=reset_reason)
        supported = snapshot.mode in (OperationMode.DETACHED_SUPPORTED, OperationMode.BILATERAL_SUPPORTED,
                                      OperationMode.RELEASED_SUPPORTED)
        contact = GeometricContact.SUPPORT_CANDIDATE if supported else GeometricContact.SEPARATED
        # Supported mode passed strict original support geometry. Transitioning
        # and airborne modes passed full FCL and >1 um geometric separation.
        gap = geometry.clearance_m
        prior = self._last_snapshot
        phase = self.phase
        target_phase = phase
        reasons = []
        if event is ConfirmationEvent.ATTACH:
            if not (phase is ClearancePhase.DETACHED_SUPPORTED_CONFIRMED and self._confirmed):
                reasons.append('ATTACH_REQUIRES_CONFIRMED_DETACHED_SUPPORT')
            elif snapshot.mode is not OperationMode.BILATERAL_SUPPORTED:
                reasons.append('ATTACH_REQUIRES_BILATERAL_SUPPORT')
            else:
                transition = self.validator.validate_transition(prior, snapshot, TransitionEvent.ATTACH,
                                                              self._last_snapshot_now, now_s)
                reasons.extend(transition.reasons)
                target_phase = ClearancePhase.ATTACHING
        elif event is ConfirmationEvent.BEGIN_LIFT:
            if not (phase is ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED and self._confirmed
                    and prior.support_site is SupportSite.SUPPLY and not self._placement_started):
                reasons.append('LIFT_REQUIRES_CONFIRMED_BILATERAL_SUPPLY')
            else:
                target_phase = ClearancePhase.LIFTING
        elif event is ConfirmationEvent.BEGIN_PLACE:
            if not (phase is ClearancePhase.AIRBORNE_CONFIRMED and self._confirmed):
                reasons.append('PLACE_REQUIRES_CONFIRMED_AIRBORNE')
            else:
                target_phase = ClearancePhase.PLACING
        elif event is ConfirmationEvent.RELEASE:
            if not self.can_release:
                reasons.append('RELEASE_REQUIRES_PRIOR_CONFIRMED_TARGET_SUPPORT')
            elif snapshot.mode is not OperationMode.RELEASED_SUPPORTED:
                reasons.append('RELEASE_REQUIRES_RELEASED_SUPPORTED_INPUT')
            else:
                transition = self.validator.validate_transition(prior, snapshot, TransitionEvent.RELEASE,
                                                              self._last_snapshot_now, now_s)
                reasons.extend(transition.reasons)
                target_phase = ClearancePhase.RELEASING

        # Every stage has a fixed mode/site/request contract. Events cannot
        # replace a latched request except explicit attach/release transitions.
        stage_modes = {
            ClearancePhase.INITIALIZING: (OperationMode.DETACHED_SUPPORTED,),
            ClearancePhase.DETACHED_SUPPORTED_CONFIRMED: (OperationMode.DETACHED_SUPPORTED,),
            ClearancePhase.ATTACHING: (OperationMode.BILATERAL_SUPPORTED,),
            ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED: (OperationMode.BILATERAL_SUPPORTED,),
            ClearancePhase.LIFTING: (OperationMode.BILATERAL_SUPPORTED, OperationMode.BILATERAL_TRANSITIONING, OperationMode.BILATERAL_AIRBORNE),
            ClearancePhase.AIRBORNE_CONFIRMED: (OperationMode.BILATERAL_TRANSITIONING, OperationMode.BILATERAL_AIRBORNE),
            ClearancePhase.PLACING: (OperationMode.BILATERAL_SUPPORTED, OperationMode.BILATERAL_TRANSITIONING, OperationMode.BILATERAL_AIRBORNE),
            ClearancePhase.NEAR_TABLE_CONFIRMED: (OperationMode.BILATERAL_SUPPORTED, OperationMode.BILATERAL_TRANSITIONING, OperationMode.BILATERAL_AIRBORNE),
            ClearancePhase.RELEASING: (OperationMode.RELEASED_SUPPORTED,),
            ClearancePhase.RELEASED_SUPPORTED_CONFIRMED: (OperationMode.RELEASED_SUPPORTED,),
        }
        if snapshot.mode not in stage_modes[target_phase]:
            reasons.append('ILLEGAL_PHASE_MODE_JUMP')
        if supported:
            expected_site = SupportSite.TARGET if self._placement_started or event is ConfirmationEvent.BEGIN_PLACE else SupportSite.SUPPLY
            if snapshot.support_site is not expected_site:
                reasons.append('INCORRECT_PHASE_SUPPORT_SITE')
        if prior is not None and event not in (ConfirmationEvent.ATTACH, ConfirmationEvent.RELEASE) and snapshot.intent != prior.intent:
            reasons.append('UNANNOUNCED_ATTACHMENT_REQUEST_CHANGE')
        if reasons:
            return self._result(False, reasons, contact, geometry, validation, reset_reason)
        if event is not ConfirmationEvent.OBSERVE:
            self._reset()
        self.phase = target_phase
        if event is ConfirmationEvent.BEGIN_PLACE:
            self._placement_started = True

        if self.phase in (ClearancePhase.INITIALIZING, ClearancePhase.DETACHED_SUPPORTED_CONFIRMED):
            if self._count('support', supported):
                self.phase = ClearancePhase.DETACHED_SUPPORTED_CONFIRMED
                self._confirmed = True
        elif self.phase is ClearancePhase.ATTACHING:
            if self._count('attachment', supported):
                self.phase = ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED
                self._counts['support'] = 3
                self._confirmed = True
        elif self.phase is ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED:
            self._confirmed = self._count('support', supported)
        elif self.phase is ClearancePhase.LIFTING:
            if self._count('high', gap >= self.parameters.d_high_m - 1e-12):
                self.phase = ClearancePhase.AIRBORNE_CONFIRMED
                self._confirmed = True
        elif self.phase is ClearancePhase.AIRBORNE_CONFIRMED:
            # Hold the airborne latch throughout the dead band. Three low
            # observations require an explicit placement event; no one-frame
            # direction derivative can switch the operation phase.
            low = self._count('low', gap <= self.parameters.d_low_m + 1e-12)
            if low:
                return self._result(False, ('DESCENT_REQUIRES_BEGIN_PLACE_EVENT',), contact, geometry, validation)
            if not self._confirmed:
                self._confirmed = self._count('high', gap >= self.parameters.d_high_m - 1e-12)
        elif self.phase in (ClearancePhase.PLACING, ClearancePhase.NEAR_TABLE_CONFIRMED):
            low = self._count('low', gap <= self.parameters.d_low_m + 1e-12)
            high = self._count('high', gap >= self.parameters.d_high_m - 1e-12)
            strict_support = self._count('support', supported and snapshot.support_site is SupportSite.TARGET)
            if strict_support:
                self.phase = ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED
                self._confirmed = True
            elif low:
                self.phase = ClearancePhase.NEAR_TABLE_CONFIRMED
                self._confirmed = True
            elif high:
                self.phase = ClearancePhase.PLACING
                self._confirmed = False
            elif self.phase is ClearancePhase.PLACING:
                self._confirmed = False
        elif self.phase in (ClearancePhase.RELEASING, ClearancePhase.RELEASED_SUPPORTED_CONFIRMED):
            if self._count('release', supported):
                self.phase = ClearancePhase.RELEASED_SUPPORTED_CONFIRMED
                self._confirmed = True
        self._last_snapshot = snapshot
        self._last_snapshot_now = float(now_s)
        self._cube_pose_source = frame.cube_pose_source
        return self._result(True, (), contact, geometry, validation, reset_reason)
