"""Adapters to the accepted gates; no alternative acceptance implementation.

Planner poses use column-vector ``T_WO`` matrices (metres, radians).  Every
waypoint here is a newly generated *synthetic computational input*.  Its TCPs
are reconstructed by the original hash-bound USD oracle, never presented as
measured observations.  Dense edge samples cannot confirm a temporal phase.
"""
from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import math
import sys

import numpy as np
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[2]
for _directory in ("task28_g1_1", "task28_g1_2a", "task28_g1_2b_0", "task28_g1_2b_1"):
    _path = str(REPO / "offline" / _directory)
    if _path not in sys.path:
        sys.path.insert(0, _path)

from task28_validator.se3 import checked_transform, real_array
from task28_contact.modes import (OperationMode, SupportSite, AttachmentIntent,
                                  ObservedAttachment, EvidenceSource)
from task28_contact.clearance import (ClearanceFrame, CubePoseSource,
    ConfirmationEvent, ClearancePhase, TemporalClearanceConfirmer)
from task28_edge.checker import SampledEdgeValidator
from task28_edge.types import (Waypoint, Provenance, Source, Motion, Trajectory,
                               SamplingPolicy, TemporalObservation, EdgeStatus)
from continuous_verifier import ContinuousVerifier


SYNTHETIC_SCOPE = (
    "New planner-generated synthetic computational inputs; hash-bound USD FK "
    "is reconstructed, not measured; no execution or physical feedback claim."
)


class CandidateValidator:
    """Reuse unchanged G1-1/R1/B0/B1 gates for new NR-generated configurations.

    ``state(X,q14)`` and ``edge(nodes,...)`` expose original checker outputs.
    ``confirm_plan`` supplies a separate fixed-count R1 boundary/hold sequence.
    NR/RBS nodes remain geometric inputs and never become confirmation frames.
    These are offline contract checks only.
    """

    def __init__(self, fixture_dir=None):
        self.checker = SampledEdgeValidator(fixture_dir)
        self.model = self.checker.model
        self._serial = 0
        self._continuous = None

    @staticmethod
    def _node(node):
        if not isinstance(node, (tuple, list)) or len(node) != 2:
            raise ValueError("EXPECTED_OBJECT_POSE_AND_Q14")
        X = checked_transform(node[0])
        q = real_array(node[1])
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError("INVALID_Q14_SHAPE_OR_NONFINITE")
        return X, q

    @staticmethod
    def _site(site):
        if site is None or type(site) is SupportSite:
            return site
        raise ValueError("TYPED_SUPPORT_SITE_REQUIRED")

    def waypoint(self, object_pose, q14, time_s, identity, *, site=None,
                 mode=None, event=ConfirmationEvent.OBSERVE, attached=True,
                 request="synthetic-bilateral", intent=None):
        X, q = self._node((object_pose, q14))
        site = self._site(site)
        if type(time_s) not in (int, float) or not math.isfinite(time_s):
            raise ValueError("INVALID_SYNTHETIC_TIME")
        if type(identity) is not str or not identity.strip():
            raise ValueError("MISSING_SYNTHETIC_IDENTITY")
        if type(attached) is not bool:
            raise ValueError("BOOLEAN_ATTACHMENT_REQUIRED")
        if mode is None:
            mode = (OperationMode.BILATERAL_SUPPORTED if site is not None
                    else OperationMode.BILATERAL_TRANSITIONING)
        if type(mode) is not OperationMode or type(event) is not ConfirmationEvent:
            raise ValueError("TYPED_MODE_AND_EVENT_REQUIRED")
        if intent is None:
            intent = AttachmentIntent(request, -1., attached, attached)
        if type(intent) is not AttachmentIntent:
            raise ValueError("TYPED_INTENT_REQUIRED")
        stamp = float(time_s)
        feedback_id = identity + ":attachment"
        return Waypoint(stamp, tuple(float(x) for x in q[:7]), tuple(float(x) for x in q[7:]),
            tuple(float(x) for x in X[:3, 3]),
            tuple(float(x) for x in Rotation.from_matrix(X[:3, :3]).as_quat()),
            mode, site, attached, attached, intent,
            ObservedAttachment(feedback_id, stamp, intent.request_id, attached, attached,
                               EvidenceSource.INDEPENDENT_SYNTHETIC),
            Provenance(Source.EXPLICIT_SYNTHETIC, identity + ":cube", feedback_id,
                       SYNTHETIC_SCOPE), event)

    def state(self, object_pose, q14, site=None):
        self._serial += 1
        w = self.waypoint(object_pose, q14, 0., f"nr-state-{self._serial}", site=site)
        return self.checker.evaluate(w)

    def _geometric_mode(self, X, site):
        if site is not None and self.checker.contact.collision.support.geometry(X, site.value).valid:
            return OperationMode.BILATERAL_SUPPORTED, site
        return OperationMode.BILATERAL_TRANSITIONING, None

    def trajectory(self, nodes, motion=Motion.GENERIC, identity="nr-edge", *,
                   site=None, start_time_s=0., time_step_s=.02,
                   temporal_prelude=(), enforce_temporal_confirmation=False,
                   required_terminal_phase=None):
        nodes = tuple(self._node(node) for node in nodes)
        if len(nodes) < 2:
            raise ValueError("AT_LEAST_TWO_NODES_REQUIRED")
        if type(motion) is not Motion:
            raise ValueError("TYPED_MOTION_REQUIRED")
        if (type(time_step_s) not in (int, float) or not math.isfinite(time_step_s)
                or time_step_s <= 0):
            raise ValueError("INVALID_SYNTHETIC_TIME_STEP")
        site = self._site(site)
        rows = []
        for index, (X, q) in enumerate(nodes):
            local_site = (SupportSite.SUPPLY if motion is Motion.LIFT else
                          SupportSite.TARGET if motion is Motion.PLACE else site)
            mode, local_site = self._geometric_mode(X, local_site)
            rows.append(self.waypoint(X, q, float(start_time_s + index*time_step_s),
                f"{identity}:{index}", mode=mode, site=local_site))
        return Trajectory(identity, tuple(rows), motion, scope=SYNTHETIC_SCOPE,
            temporal_prelude=tuple(temporal_prelude),
            enforce_temporal_confirmation=enforce_temporal_confirmation,
            required_terminal_phase=required_terminal_phase)

    def edge(self, nodes, motion=Motion.GENERIC, identity="nr-edge", *,
             sampling_policy=SamplingPolicy(), **kwargs):
        return self.checker.validate(self.trajectory(nodes, motion, identity, **kwargs),
                                     sampling_policy)

    def continuous(self, trajectory):
        """One bounded B1 attempt, retaining each original proof status verbatim."""
        if self._continuous is None:
            self._continuous = ContinuousVerifier(checker=self.checker, max_depth=0)
        return self._continuous.verify(trajectory)

    @staticmethod
    def _same_node(left, right):
        # Planner segment boundaries must be the very same numerical state;
        # acceptance tolerances are not used to bridge unplanned gaps.
        return np.array_equal(left[0], right[0]) and np.array_equal(left[1], right[1])

    def confirm_plan(self, lift_nodes, transport_nodes, place_nodes, *,
                     identity="nr-candidate", continuous=False):
        """Validate fresh complete candidate plus explicit R1 synthetic holds.

        B0/B1 retain every dense NR/RBS node but do not use their timestamps for
        R1 confirmation.  A separate 19-frame R1 sequence uses only selected
        phase boundaries and explicitly declared stationary holds.  Its 20 ms
        clock is an ordering device, not a dynamics or real sensor-rate claim.
        Candidate acceptance is all B0 gates AND that separate R1 session.
        """
        groups = [tuple(self._node(node) for node in group)
                  for group in (lift_nodes, transport_nodes, place_nodes)]
        reasons = []
        if any(len(group) < 2 for group in groups):
            reasons.append("AT_LEAST_TWO_NODES_PER_MOTION_REQUIRED")
        if not reasons:
            if not self._same_node(groups[0][-1], groups[1][0]):
                reasons.append("DISCONNECTED_LIFT_TRANSPORT_BOUNDARY")
            if not self._same_node(groups[1][-1], groups[2][0]):
                reasons.append("DISCONNECTED_TRANSPORT_PLACE_BOUNDARY")
        if reasons:
            return dict(valid=False, reasons=tuple(reasons), trajectories={}, sampled={},
                        temporal=(), continuous={}, execution_authorized=False,
                        temporal_session=dict(valid=False, reasons=tuple(reasons),
                            explicit_observation_count=0,
                            dense_path_nodes_used_as_observations=0,
                            interpolated_points_used_as_observations=0),
                        observation_scope=SYNTHETIC_SCOPE)

        lift, transport, place = groups
        prior = []
        trajectories = {}
        tick = 0
        detached = AttachmentIntent(identity+":detached", -1., False, False)
        held = None

        def row(node, *, mode, site=None, attached=True, event=ConfirmationEvent.OBSERVE,
                intent=None):
            nonlocal tick
            tick += 1
            return self.waypoint(*node, tick*.02, f"{identity}:explicit:{tick}",
                site=site, mode=mode, event=event, attached=attached, intent=intent)

        def segment(name, new_rows, motion, phase):
            rows = tuple(([prior[-1]] if prior else []) + new_rows)
            trajectories[name] = Trajectory(identity+":"+name, rows, motion,
                scope=SYNTHETIC_SCOPE+" Dense planner nodes are geometric inputs, "
                "not R1 observations; temporal confirmation is a separate session.")
            prior.extend(new_rows)

        segment("grasp_boundary", [row(lift[0], mode=OperationMode.DETACHED_SUPPORTED,
            site=SupportSite.SUPPLY, attached=False, intent=detached) for _ in range(3)],
            Motion.HOLD, ClearancePhase.DETACHED_SUPPORTED_CONFIRMED)
        held = AttachmentIntent(identity+":bilateral", prior[-1].time_s+.01, True, True)
        segment("attachment_confirmation", [row(lift[0], mode=OperationMode.BILATERAL_SUPPORTED,
            site=SupportSite.SUPPLY, event=ConfirmationEvent.ATTACH if i==0 else ConfirmationEvent.OBSERVE,
            intent=held) for i in range(3)], Motion.ATTACH, ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED)

        lift_rows = []
        for i, node in enumerate(lift):
            mode, site = self._geometric_mode(node[0], SupportSite.SUPPLY)
            lift_rows.append(row(node, mode=mode, site=site, intent=held,
                event=ConfirmationEvent.BEGIN_LIFT if i==0 else ConfirmationEvent.OBSERVE))
        for _ in range(3):
            mode, site = self._geometric_mode(lift[-1][0], SupportSite.SUPPLY)
            lift_rows.append(row(lift[-1], mode=mode, site=site, intent=held))
        segment("vertical_lift", lift_rows, Motion.LIFT, ClearancePhase.AIRBORNE_CONFIRMED)
        segment("bilateral_transport", [row(node, mode=OperationMode.BILATERAL_TRANSITIONING,
            intent=held) for node in transport], Motion.TRANSPORT, ClearancePhase.AIRBORNE_CONFIRMED)

        place_rows = []
        for i, node in enumerate(place):
            mode, site = self._geometric_mode(node[0], SupportSite.TARGET)
            place_rows.append(row(node, mode=mode, site=site, intent=held,
                event=ConfirmationEvent.BEGIN_PLACE if i==0 else ConfirmationEvent.OBSERVE))
        for _ in range(3):
            place_rows.append(row(place[-1], mode=OperationMode.BILATERAL_SUPPORTED,
                                  site=SupportSite.TARGET, intent=held))
        segment("vertical_placement_and_support_hold", place_rows, Motion.PLACE,
                ClearancePhase.BILATERAL_SUPPORTED_CONFIRMED)
        released = AttachmentIntent(identity+":released", prior[-1].time_s+.01, False, False)
        segment("release_boundary", [row(place[-1], mode=OperationMode.RELEASED_SUPPORTED,
            site=SupportSite.TARGET, attached=False, intent=released,
            event=ConfirmationEvent.RELEASE if i==0 else ConfirmationEvent.OBSERVE)
            for i in range(3)], Motion.RELEASE, ClearancePhase.RELEASED_SUPPORTED_CONFIRMED)

        sampled = {name:self.checker.validate(t) for name,t in trajectories.items()}
        for name, result in sampled.items():
            reasons.extend(name+":"+reason for reason in result.reasons)
            if result.status is not EdgeStatus.SAMPLED_EDGE_VALID and not result.reasons:
                reasons.append(name+":"+result.status.value)

        # Explicit boundary/hold controls are independent of NR/RBS subdivision.
        # Their own synthetic clock and provenance cannot be confused with the
        # dense geometric path or the original B1 proof's prerequisite checks.
        observations = []
        r1_tick = 0
        def r1_row(node, *, mode, site=None, attached=True,
                   event=ConfirmationEvent.OBSERVE, intent=None):
            nonlocal r1_tick
            r1_tick += 1
            w = self.waypoint(*node, r1_tick*.02,
                f"{identity}:r1-boundary-hold:{r1_tick}", site=site, mode=mode,
                event=event, attached=attached, intent=intent)
            observations.append(w)
        r1_detached = AttachmentIntent(identity+":r1-detached", -1., False, False)
        for _ in range(3):
            r1_row(lift[0], mode=OperationMode.DETACHED_SUPPORTED,
                   site=SupportSite.SUPPLY, attached=False, intent=r1_detached)
        r1_held = AttachmentIntent(identity+":r1-bilateral", observations[-1].time_s+.01,
                                   True, True)
        for i in range(3):
            r1_row(lift[0], mode=OperationMode.BILATERAL_SUPPORTED,
                   site=SupportSite.SUPPLY, intent=r1_held,
                   event=ConfirmationEvent.ATTACH if i==0 else ConfirmationEvent.OBSERVE)
        mode, site = self._geometric_mode(lift[0][0], SupportSite.SUPPLY)
        r1_row(lift[0], mode=mode, site=site, intent=r1_held,
               event=ConfirmationEvent.BEGIN_LIFT)
        for _ in range(3):
            mode, site = self._geometric_mode(lift[-1][0], SupportSite.SUPPLY)
            r1_row(lift[-1], mode=mode, site=site, intent=r1_held)
        for node in (transport[0], transport[-1]):
            r1_row(node, mode=OperationMode.BILATERAL_TRANSITIONING, intent=r1_held)
        mode, site = self._geometric_mode(place[0][0], SupportSite.TARGET)
        r1_row(place[0], mode=mode, site=site, intent=r1_held,
               event=ConfirmationEvent.BEGIN_PLACE)
        for _ in range(3):
            r1_row(place[-1], mode=OperationMode.BILATERAL_SUPPORTED,
                   site=SupportSite.TARGET, intent=r1_held)
        r1_released = AttachmentIntent(identity+":r1-released", observations[-1].time_s+.01,
                                       False, False)
        for i in range(3):
            r1_row(place[-1], mode=OperationMode.RELEASED_SUPPORTED,
                   site=SupportSite.TARGET, attached=False, intent=r1_released,
                   event=ConfirmationEvent.RELEASE if i==0 else ConfirmationEvent.OBSERVE)

        confirmer = TemporalClearanceConfirmer(self.checker.contact)
        temporal = []
        temporal_reasons = []
        for w in observations:
            frame = ClearanceFrame(self.checker.snapshot(w), w.provenance.cube_evidence_id,
                                   CubePoseSource.SYNTHETIC_TEST_INPUT)
            result = confirmer.observe(frame, w.time_s, w.event)
            temporal.append(dict(time_s=w.time_s, event=w.event.value,
                                 provenance=asdict(w.provenance), frame=asdict(frame),
                                 result=asdict(result)))
            temporal_reasons.extend(result.reasons)
        if (not temporal or temporal[-1]["result"]["phase"] is not ClearancePhase.RELEASED_SUPPORTED_CONFIRMED
                or not temporal[-1]["result"]["phase_confirmed"]):
            temporal_reasons.append("RELEASE_NOT_CONFIRMED")
        reasons.extend("SESSION:"+reason for reason in temporal_reasons)
        proofs = ({name:self.continuous(t) for name,t in trajectories.items()}
                  if continuous else {})
        return dict(valid=not reasons, reasons=tuple(dict.fromkeys(reasons)),
                    trajectories=trajectories, sampled=sampled, temporal=tuple(temporal),
                    temporal_session=dict(valid=not temporal_reasons,
                        reasons=tuple(dict.fromkeys(temporal_reasons)),
                        explicit_observation_count=len(observations),
                        dense_path_nodes_used_as_observations=0,
                        interpolated_points_used_as_observations=0,
                        clock="independent_fixed_20ms_synthetic_not_execution_time",
                        scope="phase_boundaries_and_explicit_three_frame_holds_only"),
                    continuous=proofs, execution_authorized=False,
                    observation_scope=SYNTHETIC_SCOPE,
                    continuous_safety="ORIGINAL_B1_STATUSES_RETAINED_NO_AUTOMATIC_UPGRADE")
