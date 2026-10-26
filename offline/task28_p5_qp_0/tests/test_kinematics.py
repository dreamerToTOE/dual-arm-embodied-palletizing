"""Independent frozen-USD and rigid-body controls for QP twist kinematics.

These are CPU algebra tests, not planning, collision, servo-timing, or physical
experiments.  The USD oracle composes authored X-axis joint frames independently
of the production URDF parser and its geometric Z-axis Jacobian.  Saved G1-1
joint controls provide provenance; this file never calls IK or an NR planner.
"""
import importlib.util
import json
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "offline"))
from task28_p5_qp_0.bootstrap import RobotModel, nominal_grasps  # noqa: E402
from task28_p5_qp_0.core import (  # noqa: E402
    reexpress_twist, skew, tcp_twist_transport, velocity_maps,
)

_oracle_spec = importlib.util.spec_from_file_location(
    "qp_independent_frozen_usd_fk", ROOT / "offline/task28_g1_1/tests/reference_fk.py")
_oracle = importlib.util.module_from_spec(_oracle_spec)
_oracle_spec.loader.exec_module(_oracle)


def _pose(rotation=None, position=(0., 0., 0.)):
    """Independently construct an active column-vector T_W_O transform."""
    result = np.eye(4)
    result[:3, :3] = np.eye(3) if rotation is None else rotation
    result[:3, 3] = position
    return result


def _inverse(pose):
    result = np.eye(4)
    result[:3, :3] = pose[:3, :3].T
    result[:3, 3] = -pose[:3, :3].T @ pose[:3, 3]
    return result


def _vee(matrix):
    """Extract from an explicitly antisymmetrized angular-rate matrix."""
    antisymmetric = .5 * (matrix - matrix.T)
    return np.array([antisymmetric[2, 1], antisymmetric[0, 2], antisymmetric[1, 0]])


def _fd_twist(minus, center, plus, step):
    """World-expressed geometric [point velocity, angular velocity]."""
    linear = (plus[:3, 3] - minus[:3, 3]) / (2. * step)
    angular = _vee((plus[:3, :3] - minus[:3, :3]) / (2. * step)
                   @ center[:3, :3].T)
    return np.r_[linear, angular]


def independent_usd_jacobian(model, side, q, step=1e-6):
    center = _oracle.usd_fk(model.fixture_dir, side, q)
    columns = []
    for k in range(7):
        delta = np.zeros(7)
        delta[k] = step
        minus = _oracle.usd_fk(model.fixture_dir, side, q - delta)
        plus = _oracle.usd_fk(model.fixture_dir, side, q + delta)
        columns.append(_fd_twist(minus, center, plus, step))
    return np.column_stack(columns)


def _world_object_motion(pose, twist, time):
    # v is the velocity of the actual Cube center.  This is not exp(se3*t)
    # about the world origin: orientation evolves about that moving center.
    return _pose(Rotation.from_rotvec(time * twist[3:]).as_matrix() @ pose[:3, :3],
                 pose[:3, 3] + time * twist[:3])


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = RobotModel()
        cls.grasps = nominal_grasps()
        cls.controls = json.loads((ROOT / "offline/task28_g1_1/tests/control_states.json")
                                 .read_text())["states"]

    def test_skew_matches_independent_cross_product_and_is_antisymmetric(self):
        a = np.array([.13, -.41, .27])
        b = np.array([-.8, .5, .11])
        np.testing.assert_allclose(skew(a) @ b, np.cross(a, b), atol=2e-17, rtol=0)
        np.testing.assert_array_equal(skew(a).T, -skew(a))

    def test_analytical_jacobians_against_frozen_usd_finite_differences(self):
        # Asymmetric finite joint poses exercise both arms' authored bases and
        # tool rotations.  q=0 and only symmetric/HOME cases are insufficient.
        cases = {
            "left": [np.array([.3, -.7, .5, -2.2, .6, 1.4, .4]),
                     np.asarray(self.controls[2]["left_q"])],
            "right": [np.array([-.2, -.8, -.4, -2.1, -.5, 1.5, -.4]),
                      np.asarray(self.controls[1]["right_q"])],
        }
        errors = []
        for side, configurations in cases.items():
            for q in configurations:
                with self.subTest(side=side, q=q.tolist()):
                    analytic = self.model.jacobian(side, q)
                    reference = independent_usd_jacobian(self.model, side, q)
                    error = float(np.max(np.abs(analytic - reference)))
                    errors.append(error)
                    # USD authored constants are float32-derived; this is much
                    # tighter than the old frozen 1e-6 FK acceptance tolerance.
                    self.assertLess(error, 3e-8)
        print(json.dumps({"qp_independent_jacobian": {
            "oracle": "preserved USD joint-frame FK, independent central differences",
            "comparisons": len(errors), "step_rad": 1e-6,
            "maximum_absolute_error": max(errors)}}))

    def test_world_object_and_tcp_reexpression_keeps_the_physical_origin(self):
        object_rotation = Rotation.from_euler("xyz", [.3, -.2, .7]).as_matrix()
        twist_object = np.array([.03, -.02, .01, .1, -.13, .2])
        expected_world = np.r_[object_rotation @ twist_object[:3],
                               object_rotation @ twist_object[3:]]
        world = reexpress_twist(object_rotation, twist_object)
        np.testing.assert_allclose(world, expected_world, atol=1e-16, rtol=0)
        np.testing.assert_allclose(reexpress_twist(object_rotation.T, world),
                                   twist_object, atol=1e-16, rtol=0)
        for grasp in (self.grasps.left, self.grasps.right):
            tcp_rotation = object_rotation @ grasp[:3, :3]
            tcp_components = np.r_[tcp_rotation.T @ world[:3],
                                    tcp_rotation.T @ world[3:]]
            np.testing.assert_allclose(reexpress_twist(tcp_rotation.T, world),
                                       tcp_components, atol=1e-16, rtol=0)
            np.testing.assert_allclose(reexpress_twist(tcp_rotation, tcp_components),
                                       world, atol=1e-16, rtol=0)

    def test_rotated_offcenter_rigid_motion_matches_independent_pose_derivative(self):
        obj = _pose(Rotation.from_euler("xyz", [.31, -.27, .44]).as_matrix(),
                    [.72, -.035, .41])
        twist = np.array([.024, -.012, .017, .17, -.11, .23])
        dt = 1e-6
        errors = []
        for side in ("left", "right"):
            grasp = getattr(self.grasps, side)
            center = obj @ grasp
            minus = _world_object_motion(obj, twist, -dt) @ grasp
            plus = _world_object_motion(obj, twist, dt) @ grasp
            independent = _fd_twist(minus, center, plus, dt)
            mapped = tcp_twist_transport(obj, grasp) @ twist
            expected = np.r_[twist[:3] + np.cross(twist[3:],
                                                  obj[:3, :3] @ grasp[:3, 3]),
                             twist[3:]]
            np.testing.assert_allclose(mapped, expected, atol=1e-16, rtol=0)
            error = float(np.max(np.abs(mapped - independent)))
            errors.append(error)
            self.assertLess(error, 1e-9)
        print(json.dumps({"qp_rigid_motion_fd": {
            "maximum_absolute_error": max(errors), "step_s": dt,
            "origin": "Cube center", "components": "world"}}))

    def test_rotating_object_requires_different_left_and_right_tcp_linear_velocities(self):
        obj = _pose(Rotation.from_euler("xyz", [.2, -.1, .35]).as_matrix(),
                    [.65, .01, .4])
        omega = np.array([.2, -.1, .4])
        twist = np.r_[np.zeros(3), omega]
        left = tcp_twist_transport(obj, self.grasps.left) @ twist
        right = tcp_twist_transport(obj, self.grasps.right) @ twist
        expected_difference = np.cross(omega,
            obj[:3, :3] @ (self.grasps.right[:3, 3] - self.grasps.left[:3, 3]))
        np.testing.assert_allclose(right[:3] - left[:3], expected_difference,
                                   atol=2e-17, rtol=0)
        self.assertGreater(np.linalg.norm(right[:3] - left[:3]), .04)
        np.testing.assert_array_equal(left[3:], omega)
        np.testing.assert_array_equal(right[3:], omega)

    def test_transport_uses_offset_from_cube_center_not_world_tcp_position(self):
        rotation = Rotation.from_euler("xyz", [.12, -.3, .2]).as_matrix()
        first = _pose(rotation, [.65, 0., .4])
        translated = _pose(rotation, [13.2, -8.1, 4.5])
        for grasp in (self.grasps.left, self.grasps.right):
            np.testing.assert_array_equal(tcp_twist_transport(first, grasp),
                                          tcp_twist_transport(translated, grasp))

    def test_opposite_skew_sign_fails_independent_rigid_motion_control(self):
        obj = _pose(Rotation.from_euler("xyz", [.3, .2, -.1]).as_matrix(),
                    [.65, 0., .4])
        twist = np.array([.01, -.02, .03, .4, -.2, .3])
        grasp = self.grasps.left
        dt = 1e-6
        independent = _fd_twist(_world_object_motion(obj, twist, -dt) @ grasp,
                                obj @ grasp,
                                _world_object_motion(obj, twist, dt) @ grasp, dt)
        r = obj[:3, :3] @ grasp[:3, 3]
        wrong = np.eye(6)
        # Independent element assembly makes this a real sign-negative control.
        x, y, z = r
        wrong[:3, 3:] = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
        self.assertGreater(np.linalg.norm((wrong @ twist - independent)[:3]), .05)
        self.assertLess(np.linalg.norm(tcp_twist_transport(obj, grasp) @ twist - independent),
                        1e-9)

    def test_velocity_maps_match_independent_point_velocity_reconstruction(self):
        rates = np.array([.04, -.03, .02, -.01, .015, -.025, .035,
                          -.02, .03, -.04, .025, -.015, .01, -.035])
        errors = []
        for row in self.controls:
            q14 = np.r_[row["left_q"], row["right_q"]]
            obj = _pose(Rotation.from_euler("z", row["object_yaw_rad"]).as_matrix(),
                        row["object_position_m"])
            maps = velocity_maps(self.model, q14, obj)
            independent = []
            for index, side in enumerate(("left", "right")):
                q = q14[index*7:(index+1)*7]
                tcp = independent_usd_jacobian(self.model, side, q) @ rates[index*7:(index+1)*7]
                grasp = getattr(self.grasps, side)
                r = obj[:3, :3] @ grasp[:3, 3]
                implied = np.r_[tcp[:3] - np.cross(tcp[3:], r), tcp[3:]]
                independent.append(implied)
                errors.append(float(np.max(np.abs(maps[f"B_{side}"]
                                                  @ rates[index*7:(index+1)*7] - implied))))
            np.testing.assert_allclose(maps["B"] @ rates,
                                       .5 * (independent[0] + independent[1]),
                                       atol=3e-9, rtol=0)
            np.testing.assert_allclose(maps["C"] @ rates,
                                       independent[0] - independent[1],
                                       atol=3e-9, rtol=0)
        print(json.dumps({"qp_velocity_maps_independent_fd": {
            "states": len(self.controls), "maximum_absolute_error": max(errors)}}))

    def test_compatible_twist_tangent_has_zero_bilateral_and_correct_object_rate(self):
        # Independent least-squares tangent construction is only a mathematical
        # positive control, not a QP or IK result and not collision acceptance.
        desired = np.array([.015, -.01, .005, .02, -.015, .04])
        maximum_fd = 0.
        for row in self.controls:
            obj = _pose(Rotation.from_euler("z", row["object_yaw_rad"]).as_matrix(),
                        row["object_position_m"])
            q14 = np.r_[row["left_q"], row["right_q"]]
            maps = velocity_maps(self.model, q14, obj)
            rates = []
            for index, side in enumerate(("left", "right")):
                grasp = getattr(self.grasps, side)
                q = q14[index*7:(index+1)*7]
                r = obj[:3, :3] @ grasp[:3, 3]
                target = np.r_[desired[:3] + np.cross(desired[3:], r), desired[3:]]
                # This finite-difference USD J differs from production J.
                derivative = independent_usd_jacobian(self.model, side, q)
                dq, _, rank, _ = np.linalg.lstsq(derivative, target, rcond=1e-12)
                self.assertEqual(rank, 6)
                rates.extend(dq)
                dt = 1e-6
                center = _oracle.usd_fk(self.model.fixture_dir, side, q) @ _inverse(grasp)
                minus = _oracle.usd_fk(self.model.fixture_dir, side, q-dt*dq) @ _inverse(grasp)
                plus = _oracle.usd_fk(self.model.fixture_dir, side, q+dt*dq) @ _inverse(grasp)
                independent_rate = _fd_twist(minus, center, plus, dt)
                maximum_fd = max(maximum_fd, float(np.max(np.abs(independent_rate - desired))))
                np.testing.assert_allclose(independent_rate, desired, atol=3e-9, rtol=0)
            rates = np.asarray(rates)
            np.testing.assert_allclose(maps["C"] @ rates, np.zeros(6), atol=3e-9, rtol=0)
            np.testing.assert_allclose(maps["B"] @ rates, desired, atol=3e-9, rtol=0)
        print(json.dumps({"qp_compatible_tangent_independent_fd": {
            "maximum_absolute_error": maximum_fd, "states": len(self.controls),
            "source": "committed G1-1 control_states.json; no new IK/NR"}}))


if __name__ == "__main__":
    unittest.main()
