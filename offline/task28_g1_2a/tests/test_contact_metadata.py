"""Synthetic contact metadata controls; no ROS timing or physical feedback."""

from dataclasses import FrozenInstanceError, replace
import math
import unittest

from task28_validator.state import (
    AttachmentMode, InputState, JointObservation, PoseObservation, StatePolicy,
    expected_joint_names,
)
from task28_contact.modes import (
    AttachmentIntent, EvidenceSource, ObservedAttachment, OperationMode,
    Snapshot, SupportSite, validate_snapshot_metadata,
)


def metadata_snapshot(mode=OperationMode.BILATERAL_SUPPORTED, site=SupportSite.SUPPLY):
    """Well-formed independent metadata; zero joints are not FK evidence."""
    attached = mode in (OperationMode.BILATERAL_SUPPORTED, OperationMode.BILATERAL_AIRBORNE)
    state = InputState(
        JointObservation("left", "world", 10.0, expected_joint_names("left"), (0.0,) * 7),
        JointObservation("right", "world", 10.0, expected_joint_names("right"), (0.0,) * 7),
        PoseObservation("world", 10.0, (0.65, -0.06, 0.26), (0.0, 0.0, 0.0, 1.0)),
        PoseObservation("world", 10.0, (0.65, 0.06, 0.26), (0.0, 0.0, 0.0, 1.0)),
        PoseObservation("world", 10.0, (0.65, 0.0, 0.26), (0.0, 0.0, 0.0, 1.0), "Cube_01"),
        AttachmentMode.BILATERAL if attached else AttachmentMode.DETACHED,
        attached, attached,
    )
    # These two inputs are deliberately independently specified. There is no
    # production helper which copies attachment intent into observed feedback.
    intent = AttachmentIntent("request-001", 9.0, attached, attached)
    observed = ObservedAttachment("synthetic-evidence-001", 10.0, "request-001", attached, attached)
    return Snapshot(state, mode, site, intent, observed)


class ContactMetadataTests(unittest.TestCase):
    def reject(self, snapshot, reason, *, now=10.05, policy=StatePolicy()):
        result = validate_snapshot_metadata(snapshot, now, policy)
        self.assertFalse(result.valid)
        self.assertIn(reason, result.reasons)
        return result

    def test_all_four_valid_metadata_modes(self):
        for mode, site in ((OperationMode.DETACHED_SUPPORTED, SupportSite.SUPPLY),
                           (OperationMode.BILATERAL_SUPPORTED, SupportSite.SUPPLY),
                           (OperationMode.BILATERAL_SUPPORTED, SupportSite.TARGET),
                           (OperationMode.BILATERAL_AIRBORNE, None),
                           (OperationMode.RELEASED_SUPPORTED, SupportSite.TARGET)):
            with self.subTest(mode=mode, site=site):
                self.assertTrue(validate_snapshot_metadata(metadata_snapshot(mode, site), 10.05).valid)

    def test_contract_is_frozen(self):
        snapshot = metadata_snapshot()
        for value, field, replacement in ((snapshot, "mode", OperationMode.RELEASED_SUPPORTED),
                                          (snapshot.intent, "left", False),
                                          (snapshot.observed, "left", False)):
            with self.subTest(field=field):
                with self.assertRaises(FrozenInstanceError):
                    setattr(value, field, replacement)

    def test_raw_strings_and_missing_snapshot_rejected(self):
        for value in (None, {}, "snapshot"):
            self.reject(value, "INVALID_SNAPSHOT_TYPE")
        self.reject(replace(metadata_snapshot(), mode="bilateral_supported"), "INVALID_OPERATION_MODE")
        self.reject(replace(metadata_snapshot(), support_site="supply"), "INVALID_SUPPORT_SITE")

    def test_sites_are_mode_specific(self):
        self.reject(metadata_snapshot(OperationMode.DETACHED_SUPPORTED, SupportSite.TARGET), "INCORRECT_SUPPORT_SITE")
        self.reject(metadata_snapshot(OperationMode.RELEASED_SUPPORTED, SupportSite.SUPPLY), "INCORRECT_SUPPORT_SITE")
        self.reject(metadata_snapshot(OperationMode.BILATERAL_SUPPORTED, None), "MISSING_SUPPORT_SITE")
        self.reject(metadata_snapshot(OperationMode.BILATERAL_AIRBORNE, SupportSite.SUPPLY), "AIRBORNE_SUPPORT_SITE_PRESENT")

    def test_modes_require_matching_old_attachment_state(self):
        base = metadata_snapshot()
        detached_state = replace(base.input_state, attachment_mode=AttachmentMode.DETACHED,
                                 left_attached=False, right_attached=False)
        result = self.reject(replace(base, input_state=detached_state), "MODE_ATTACHMENT_MISMATCH")
        self.assertIn("MODE_ATTACHMENT_FLAGS_MISMATCH", result.reasons)
        for flags in ((True, False), (False, True), (False, False)):
            state = replace(base.input_state, left_attached=flags[0], right_attached=flags[1])
            self.reject(replace(base, input_state=state), "MODE_ATTACHMENT_FLAGS_MISMATCH")

    def test_latched_old_intent_is_allowed_but_future_intent_is_not(self):
        base = metadata_snapshot()
        self.assertTrue(validate_snapshot_metadata(replace(base, intent=replace(base.intent, issued_at_s=-100.0)), 10.05).valid)
        self.reject(replace(base, intent=replace(base.intent, issued_at_s=10.1)), "FUTURE_ATTACHMENT_INTENT")

    def test_missing_intent_or_feedback_is_rejected(self):
        self.reject(replace(metadata_snapshot(), intent=None), "MISSING_OR_INVALID_ATTACHMENT_INTENT")
        self.reject(replace(metadata_snapshot(), observed=None), "MISSING_OR_INVALID_ATTACHMENT_FEEDBACK")

    def test_command_echo_is_explicitly_not_feedback(self):
        base = metadata_snapshot()
        self.reject(replace(base, observed=replace(base.observed, source=EvidenceSource.COMMAND_ECHO)),
                    "ATTACHMENT_COMMAND_ECHO_NOT_FEEDBACK")
        self.reject(replace(base, observed=replace(base.observed, source="independent_synthetic")),
                    "INVALID_FEEDBACK_SOURCE")

    def test_false_bilateral_feedback_and_intent_flags_rejected(self):
        base = metadata_snapshot()
        for flags in ((False, False), (False, True), (True, False)):
            with self.subTest(flags=flags):
                self.reject(replace(base, observed=replace(base.observed, left=flags[0], right=flags[1])),
                            "FEEDBACK_MODE_MISMATCH")
                self.reject(replace(base, intent=replace(base.intent, left=flags[0], right=flags[1])),
                            "INTENT_MODE_MISMATCH")
        self.reject(replace(base, observed=replace(base.observed, left=False)), "INTENT_FEEDBACK_MISMATCH")

    def test_detached_and_released_cannot_contain_true_attachment_feedback(self):
        for mode, site in ((OperationMode.DETACHED_SUPPORTED, SupportSite.SUPPLY),
                           (OperationMode.RELEASED_SUPPORTED, SupportSite.TARGET)):
            base = metadata_snapshot(mode, site)
            self.reject(replace(base, observed=replace(base.observed, left=True, right=True)), "FEEDBACK_MODE_MISMATCH")

    def test_identifiers_and_request_correspondence(self):
        base = metadata_snapshot()
        for identifier in (None, "", " ", 1):
            with self.subTest(identifier=identifier):
                self.reject(replace(base, intent=replace(base.intent, request_id=identifier)), "INVALID_INTENT_REQUEST_ID")
                self.reject(replace(base, observed=replace(base.observed, request_id=identifier)), "INVALID_FEEDBACK_REQUEST_ID")
                self.reject(replace(base, observed=replace(base.observed, evidence_id=identifier)), "INVALID_FEEDBACK_EVIDENCE_ID")
        self.reject(replace(base, observed=replace(base.observed, request_id="wrong-request")), "FEEDBACK_REQUEST_MISMATCH")

    def test_nonfinite_or_nonnumeric_metadata_timestamps_and_flags(self):
        base = metadata_snapshot()
        for stamp in (None, math.nan, math.inf, "10.0", True):
            with self.subTest(stamp=stamp):
                self.reject(replace(base, intent=replace(base.intent, issued_at_s=stamp)), "INVALID_INTENT_TIMESTAMP")
                self.reject(replace(base, observed=replace(base.observed, stamp_s=stamp)), "INVALID_FEEDBACK_TIMESTAMP")
        for flag in (None, 1, "true"):
            self.reject(replace(base, intent=replace(base.intent, left=flag)), "INVALID_INTENT_FLAGS")
            self.reject(replace(base, observed=replace(base.observed, left=flag)), "INVALID_FEEDBACK_FLAGS")

    def test_feedback_must_follow_corresponding_intent(self):
        base = metadata_snapshot()
        self.reject(replace(base, intent=replace(base.intent, issued_at_s=10.01)), "FEEDBACK_PREDATES_INTENT")

    def test_stale_future_and_globally_asynchronous_feedback(self):
        base = metadata_snapshot()
        self.reject(replace(base, observed=replace(base.observed, stamp_s=9.8)), "STALE_ATTACHMENT_FEEDBACK")
        self.reject(replace(base, observed=replace(base.observed, stamp_s=10.1)), "FUTURE_ATTACHMENT_FEEDBACK")
        self.reject(replace(base, observed=replace(base.observed, stamp_s=9.97)), "ASYNCHRONOUS_ATTACHMENT_FEEDBACK")

    def test_feedback_skew_compares_every_original_observation(self):
        base = metadata_snapshot()
        # Five old observations span 15 ms and pass the G1-1 gate; feedback
        # at +10 ms creates 25 ms global skew and must still be rejected.
        for field in ("left_joints", "right_joints", "left_tcp", "right_tcp", "cube"):
            state = replace(base.input_state, **{field: replace(getattr(base.input_state, field), stamp_s=9.985)})
            snapshot = replace(base, input_state=state, observed=replace(base.observed, stamp_s=10.01))
            with self.subTest(field=field):
                result = self.reject(snapshot, "ASYNCHRONOUS_ATTACHMENT_FEEDBACK")
                self.assertNotIn("ASYNCHRONOUS_OBSERVATIONS", result.reasons)

    def test_original_state_gate_remains_active(self):
        base = metadata_snapshot()
        self.reject(replace(base, input_state=replace(base.input_state, left_joints=None)),
                    "MISSING_OR_INVALID_JOINT_OBSERVATION:left_joints")
        self.reject(replace(base, input_state=replace(base.input_state, cube=replace(base.input_state.cube, object_id="Cube_02"))),
                    "INCORRECT_CUBE_ID:cube")
        self.reject(replace(base, input_state=replace(base.input_state, cube=replace(base.input_state.cube, stamp_s=9.8))),
                    "STALE_OBSERVATION:cube")
        self.reject(replace(base, input_state=replace(base.input_state, cube=replace(base.input_state.cube, frame_id="other"))),
                    "INCORRECT_FRAME:cube")
        self.reject(replace(base, input_state=replace(base.input_state, cube=replace(base.input_state.cube, quaternion_xyzw=(0.0,) * 4))),
                    "NONUNIT_QUATERNION:cube")

    def test_clock_and_policy_cannot_be_relaxed(self):
        base = metadata_snapshot()
        for policy in (None, StatePolicy(max_age_s=1.0), StatePolicy(max_skew_s=1.0),
                       StatePolicy(max_future_s=0.1), StatePolicy(quaternion_norm_tolerance=0.01)):
            self.reject(base, "INVALID_POLICY", policy=policy)
        for now in (None, True, math.nan, math.inf):
            self.reject(base, "INVALID_NOW", now=now)
        tight = StatePolicy(max_age_s=0.01, max_skew_s=0.001, max_future_s=0.0,
                            quaternion_norm_tolerance=1e-8)
        self.assertTrue(validate_snapshot_metadata(base, 10.0, tight).valid)
        self.reject(base, "STALE_ATTACHMENT_FEEDBACK", policy=tight)

    def test_reason_order_is_deterministic(self):
        base = metadata_snapshot()
        broken = replace(base, mode="bad", support_site="bad", intent=None, observed=None)
        expected = ("INVALID_OPERATION_MODE", "INVALID_SUPPORT_SITE", "MISSING_OR_INVALID_ATTACHMENT_INTENT",
                    "MISSING_OR_INVALID_ATTACHMENT_FEEDBACK")
        self.assertEqual(validate_snapshot_metadata(broken, 10.05).reasons, expected)


if __name__ == "__main__":
    unittest.main()
