"""Pure evidence-bookkeeping controls; no solver or robot feasibility claim.

All compact/active-row examples below are abstract synthetic receipts. They
verify aggregation and request pairing without inventing reachable FR3 states.
"""
from dataclasses import asdict
import unittest

import numpy as np

from offline.task28_p5_qp_1.run_experiments import active_sources, compact, request


class RunnerEvidenceTests(unittest.TestCase):
    def test_compact_uses_evaluated_b0_samples_and_separates_failed_attempts(self):
        def step(index, accepted, count, tracking, margin):
            return dict(step_index=index, accepted=accepted,
                q_initial=[0.] * 14, qdot14=[0.] * 14, q_next=[0.] * 14,
                qp=dict(status="FEASIBLE_TRACKING_OK",
                diagnostics=dict(tracking_translation_error_m_s=tracking,
                                 tracking_rotation_error_rad_s=tracking / 2,
                                 dt_s=.1)),
                frozen_bounds=dict(valid=accepted,
                    minimum_position_margin_rad=margin,
                    minimum_velocity_margin_rad_s=margin / 2,
                    velocity_max_rad_s=[2.] * 14, position_lower_rad=[-1.] * 14,
                    position_upper_rad=[1.] * 14, lower_rad_s=[-2.] * 14,
                    upper_rad_s=[2.] * 14, active_lower_indices=[], active_upper_indices=[],
                    active_bound_reporting_tolerance_rad_s=1e-7),
                independent_velocity_diagnostics=dict(original_equality_inf=0.),
                current_state=dict(valid=True), predicted_state=None, velocity_change_rad_s=None,
                velocity_change_rate_rad_s2=None,
                sampled_edge=dict(evaluated_sample_count=count,
                                  planned_sample_count=100,
                                  status="sampled_edge_valid" if accepted else "sampled_edge_rejected",
                                  maxima=dict(max_closure_translation_m=tracking / 10,
                                              max_closure_rotation_rad=tracking / 5)))
        record = dict(request=dict(identity="ABSTRACT_RECEIPTS_ONLY"), method="bounded",
                      status="STOPPED_AT_FIRST_REJECTION", steps=[step(0, True, 3, .001, .2),
                                                                  step(1, False, 5, .9, -.3)],
                      initial_state=dict(valid=True), attempted_step_count=2,
                      accepted_step_count=1, failure=dict(step_index=1, reasons=["ABSTRACT"]),
                      wall_time_s=.1, continuous_safety="NOT_CERTIFIED_FINITE_SAMPLING_ONLY")
        row = compact(record)
        self.assertEqual(row["total_B0_samples"], 8)
        self.assertEqual(row["all_attempts"]["tracking_translation_m_s"], .9)
        self.assertEqual(row["accepted_only"]["tracking_translation_m_s"], .001)
        self.assertEqual(row["all_attempts"]["minimum_position_margin_rad"], -.3)
        self.assertEqual(row["accepted_only"]["minimum_position_margin_rad"], .2)
        self.assertEqual(row["active_source_counts"], {})
        self.assertEqual(row["geometry_rejection_after_tracking_and_hard_pass"], [])
        self.assertFalse(row["execution_authorized"])

    def test_active_sources_classifies_velocity_and_position_without_scene_changes(self):
        def receipt(q0, velocity0):
            q = np.zeros(14); q[0] = q0
            vmax = np.full(14, 2.); vmax[0] = velocity0
            lower, upper = -np.ones(14), np.ones(14)
            dt = .1
            lo = np.maximum(-vmax, (np.nextafter(lower, np.inf) - q) / dt)
            hi = np.minimum(vmax, (np.nextafter(upper, -np.inf) - q) / dt)
            x = np.zeros(14); x[0] = hi[0]
            return dict(q_initial=q.tolist(), qdot14=x.tolist(),
                qp=dict(diagnostics=dict(dt_s=dt)), frozen_bounds=dict(
                    velocity_max_rad_s=vmax.tolist(), position_lower_rad=lower.tolist(),
                    position_upper_rad=upper.tolist(), lower_rad_s=lo.tolist(), upper_rad_s=hi.tolist(),
                    active_lower_indices=[], active_upper_indices=[0],
                    active_bound_reporting_tolerance_rad_s=1e-7))
        for q0, vmax, expected in ((0., 2., ["velocity"]),
                                   (.99, 2., ["position_lookahead"]),
                                   (.99, .1, ["velocity", "position_lookahead"])):
            with self.subTest(scope="ABSTRACT_ROW_CLASSIFICATION", q0=q0, vmax=vmax):
                rows = active_sources(receipt(q0, vmax))
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["sources"], expected)
                self.assertEqual(rows[0]["joint_name"], "left_fr3_joint1")
                self.assertFalse(rows[0]["execution_authorized"])

    def test_paired_request_values_identical_and_invalid_mutations_preserved(self):
        node = dict(q14=[0.] * 14, object_pose=np.eye(4).tolist(), seed=28,
                    segment="transport", node_index=40, source_sha256="a" * 64)
        case = dict(identity="ABSTRACT_REQUEST_PAIRING", desired_object_twist=[.001, 0., 0., 0., 0., 0.],
                    dt_s=.02, max_steps=3)
        first, second = request(case, node), request(case, node)
        self.assertEqual(asdict(first), asdict(second))
        self.assertEqual(first.dt_s, .02)
        self.assertEqual(first.max_steps, 3)
        self.assertEqual(first.observation_provenance["source_sha256"], node["source_sha256"])
        self.assertFalse(first.observation_provenance["execution_authorized"])
        invalid = request(case, node, cube_id="Cube_02", dt_s=0., q14=None)
        self.assertEqual(invalid.cube_id, "Cube_02")
        self.assertEqual(invalid.dt_s, 0.)
        self.assertIsNone(invalid.q14)
        # Pairing tests never feed these abstract zero joint values to a solver.
        self.assertEqual(invalid.object_pose, first.object_pose)


if __name__ == "__main__":
    unittest.main()
