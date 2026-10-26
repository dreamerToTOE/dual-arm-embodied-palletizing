"""Synthetic offline state controls; these do not measure ROS timing."""

from dataclasses import FrozenInstanceError, replace
import math
import unittest

from task28_validator.state import (
    AttachmentMode,
    InputState,
    JointObservation,
    PoseObservation,
    StatePolicy,
    expected_joint_names,
    validate_state,
)


def valid_state() -> InputState:
    """An independently specified well-formed input, not FK test evidence."""
    return InputState(
        left_joints=JointObservation("left", "world", 10.0, expected_joint_names("left"), (0.0,) * 7),
        right_joints=JointObservation("right", "world", 10.0, expected_joint_names("right"), (0.0,) * 7),
        left_tcp=PoseObservation("world", 10.0, (0.65, -0.06, 0.26), (0.0, 0.0, 0.0, 1.0)),
        right_tcp=PoseObservation("world", 10.0, (0.65, 0.06, 0.26), (0.0, 0.0, 0.0, 1.0)),
        cube=PoseObservation("world", 10.0, (0.65, 0.0, 0.26), (0.0, 0.0, 0.0, 1.0), "Cube_01"),
        attachment_mode=AttachmentMode.BILATERAL,
        left_attached=True,
        right_attached=True,
    )


class StateValidationTests(unittest.TestCase):
    def assert_rejected(self, state, reason, *, now=10.05, policy=StatePolicy()):
        result = validate_state(state, now, policy)
        self.assertFalse(result.valid)
        self.assertIn(reason, result.reasons)
        return result

    def test_valid_bilateral_and_detached(self):
        self.assertEqual(validate_state(valid_state(), 10.05).reasons, ())
        detached = replace(valid_state(), attachment_mode=AttachmentMode.DETACHED,
                           left_attached=False, right_attached=False)
        self.assertTrue(validate_state(detached, 10.05).valid)

    def test_contract_is_frozen(self):
        with self.assertRaises(FrozenInstanceError):
            valid_state().cube.frame_id = "other"

    def test_missing_joint_observation(self):
        self.assert_rejected(replace(valid_state(), left_joints=None),
                             "MISSING_OR_INVALID_JOINT_OBSERVATION:left_joints")

    def test_missing_and_reordered_joint_names(self):
        base = valid_state()
        for names in (base.left_joints.names[:-1], tuple(reversed(base.left_joints.names)), None):
            with self.subTest(names=names):
                self.assert_rejected(replace(base, left_joints=replace(base.left_joints, names=names)),
                                     "INCORRECT_JOINT_NAMES:left_joints")

    def test_wrong_arm_side(self):
        base = valid_state()
        self.assert_rejected(replace(base, left_joints=replace(base.left_joints, side="right")),
                             "INCORRECT_ARM_SIDE:left_joints")

    def test_joint_shapes_and_nonfinite_values(self):
        base = valid_state()
        for values in ((0.0,) * 6, (0.0,) * 6 + (math.nan,), (math.inf,) * 7,
                       (True,) * 7, [0.0] * 7, None):
            with self.subTest(values=values):
                self.assert_rejected(replace(base, left_joints=replace(base.left_joints, positions_rad=values)),
                                     "INVALID_JOINT_VALUES:left_joints")

    def test_wrong_frames_for_each_observation(self):
        base = valid_state()
        for field in ("left_joints", "right_joints", "left_tcp", "right_tcp", "cube"):
            with self.subTest(field=field):
                state = replace(base, **{field: replace(getattr(base, field), frame_id="base_link")})
                self.assert_rejected(state, f"INCORRECT_FRAME:{field}")

    def test_invalid_cube_id(self):
        base = valid_state()
        for cube_id in ("Cube_02", "cube_01", None):
            with self.subTest(cube_id=cube_id):
                self.assert_rejected(replace(base, cube=replace(base.cube, object_id=cube_id)),
                                     "INCORRECT_CUBE_ID:cube")

    def test_missing_pose(self):
        self.assert_rejected(replace(valid_state(), cube=None), "MISSING_OR_INVALID_POSE_OBSERVATION:cube")

    def test_position_finiteness_and_shape(self):
        base = valid_state()
        for value in ((0.0, 0.0), (math.nan, 0.0, 0.0), (math.inf, 0.0, 0.0), None):
            with self.subTest(value=value):
                self.assert_rejected(replace(base, cube=replace(base.cube, position_m=value)),
                                     "INVALID_POSITION:cube")

    def test_quaternion_shape_finiteness_and_unit_norm(self):
        base = valid_state()
        for quaternion in ((0.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0), None):
            with self.subTest(quaternion=quaternion):
                self.assert_rejected(replace(base, cube=replace(base.cube, quaternion_xyzw=quaternion)),
                                     "INVALID_QUATERNION:cube")
        for quaternion in ((0.0,) * 4, (0.0, 0.0, 0.0, 2.0), (1e308,) * 4):
            with self.subTest(quaternion=quaternion):
                pose = replace(base.cube, quaternion_xyzw=quaternion)
                self.assert_rejected(replace(base, cube=pose), "NONUNIT_QUATERNION:cube")
                self.assertEqual(pose.quaternion_xyzw, quaternion)  # no silent normalization
        self.assertTrue(validate_state(replace(base, cube=replace(base.cube, quaternion_xyzw=(0.0, 0.0, 0.0, -1.0))), 10.05).valid)

    def test_stale_and_future_observations(self):
        base = valid_state()
        for field in ("left_joints", "right_joints", "left_tcp", "right_tcp", "cube"):
            with self.subTest(field=field):
                stale = replace(base, **{field: replace(getattr(base, field), stamp_s=9.8)})
                self.assert_rejected(stale, f"STALE_OBSERVATION:{field}")
                future = replace(base, **{field: replace(getattr(base, field), stamp_s=10.06)})
                self.assert_rejected(future, f"FUTURE_OBSERVATION:{field}")

    def test_skew_includes_joints_tcps_and_cube(self):
        base = valid_state()
        for field in ("left_joints", "right_joints", "left_tcp", "right_tcp", "cube"):
            with self.subTest(field=field):
                asynchronous = replace(base, **{field: replace(getattr(base, field), stamp_s=9.97)})
                self.assert_rejected(asynchronous, "ASYNCHRONOUS_OBSERVATIONS")

    def test_invalid_timestamp_now_and_policy(self):
        base = valid_state()
        for stamp in (math.nan, math.inf, "10.0", True, None):
            with self.subTest(stamp=stamp):
                self.assert_rejected(replace(base, cube=replace(base.cube, stamp_s=stamp)), "INVALID_TIMESTAMP:cube")
        for now in (math.nan, math.inf, None, True):
            self.assert_rejected(base, "INVALID_NOW", now=now)
        for policy in (None, StatePolicy(max_age_s=-1.0), StatePolicy(max_skew_s=math.nan),
                       StatePolicy(max_future_s=math.inf), StatePolicy(quaternion_norm_tolerance=-1.0)):
            self.assert_rejected(base, "INVALID_POLICY", policy=policy)

    def test_policies_may_tighten_but_cannot_weaken_offline_gate(self):
        base = valid_state()
        for policy in (StatePolicy(max_age_s=0.10001), StatePolicy(max_skew_s=0.02001),
                       StatePolicy(max_future_s=0.001001), StatePolicy(quaternion_norm_tolerance=1.0001e-6),
                       StatePolicy(max_age_s=0.0), StatePolicy(max_skew_s=0.0),
                       StatePolicy(quaternion_norm_tolerance=0.0)):
            with self.subTest(policy=policy):
                self.assert_rejected(base, "INVALID_POLICY", policy=policy)
        tight = StatePolicy(max_age_s=0.01, max_skew_s=0.001, max_future_s=0.0,
                            quaternion_norm_tolerance=1e-8)
        self.assertTrue(validate_state(base, 10.0, tight).valid)
        self.assert_rejected(base, "STALE_OBSERVATION:left_joints", policy=tight)

    def test_attachment_modes_fail_closed(self):
        base = valid_state()
        for mode in ("bilateral", "left_only", None):
            with self.subTest(mode=mode):
                self.assert_rejected(replace(base, attachment_mode=mode), "INVALID_ATTACHMENT_MODE")
        for mode in AttachmentMode:
            for flags in ((True, False), (False, True)):
                with self.subTest(mode=mode, flags=flags):
                    self.assert_rejected(replace(base, attachment_mode=mode,
                                                 left_attached=flags[0], right_attached=flags[1]),
                                         "INCONSISTENT_ATTACHMENT_MODE")
        self.assert_rejected(replace(base, attachment_mode=AttachmentMode.DETACHED), "INCONSISTENT_ATTACHMENT_MODE")
        self.assert_rejected(replace(base, left_attached=1), "INVALID_ATTACHMENT_FLAGS")

    def test_malformed_top_level_is_rejected_without_exception(self):
        for state in (None, {}, "state"):
            with self.subTest(state=state):
                self.assert_rejected(state, "INVALID_STATE_TYPE")

    def test_rejection_order_is_deterministic(self):
        base = valid_state()
        broken = replace(base, left_joints=None, cube=replace(base.cube, frame_id="bad", object_id="bad"))
        expected = ("MISSING_OR_INVALID_JOINT_OBSERVATION:left_joints", "INCORRECT_FRAME:cube", "INCORRECT_CUBE_ID:cube")
        self.assertEqual(validate_state(broken, 10.05).reasons, expected)


if __name__ == "__main__":
    unittest.main()
