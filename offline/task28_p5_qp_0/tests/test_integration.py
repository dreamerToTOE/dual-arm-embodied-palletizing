"""Actual frozen FCL/closure diagnostics; no new planning or robot commands."""
import gzip
import json
from pathlib import Path
import unittest
import numpy as np
from offline.task28_p5_qp_0 import CentralizedVelocityQP, VelocityInput
from offline.task28_p5_qp_0.bootstrap import transform
from offline.task28_p5_qp_0.integration import OfflineStepDiagnostics, integrate_object_world_center
from task28_contact.modes import SupportSite

ROOT = Path(__file__).resolve().parents[3]


class OfflineIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.diag = OfflineStepDiagnostics()
        with gzip.open(ROOT/'offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz','rt') as stream:
            saved = json.load(stream)
        cls.X, cls.q = (np.asarray(x) for x in saved['transport'][len(saved['transport'])//2])
        cls.support_X, cls.support_q = (np.asarray(x) for x in saved['lift'][0])
        with gzip.open(ROOT/'offline/task28_p4_nr_0/evidence/experiments_v1/negative_controls.json.gz','rt') as stream:
            negatives = json.load(stream)
        cls.payload_collision = next(c['node'] for c in negatives if c['case']=='payload_table_collision')

    def request(self, twist=(.003, -.001, .001, .005, -.003, .01), dt=.01):
        return VelocityInput(self.q, self.X, twist, dt)

    def test_actual_fcl_nonzero_step_and_original_sampled_gates(self):
        got = self.diag.evaluate(self.request(), 'qp-test-nonzero')
        self.assertEqual(got['status'], 'OFFLINE_SAMPLED_STEP_VALID_UNCERTIFIED')
        self.assertTrue(got['sampled_tracking_candidate'])
        self.assertGreater(np.linalg.norm(got['qp']['qdot']), .001)
        for key in ('current_state', 'predicted_state'):
            self.assertTrue(got[key]['valid'])
            self.assertEqual(got[key]['collision']['reference']['checked_shape_pairs'],414)
            self.assertEqual(got[key]['collision']['reference']['skipped_acm_shape_pairs'],40)
        self.assertFalse(got['execution_authorized'])
        self.assertEqual(got['temporal_confirmation'],'NOT_RUN_NO_PHASE_CHANGE_OR_RELEASE_AUTHORIZATION')
        self.assertFalse(got['continuous_verifier_run'])

    def test_feasible_bad_tracking_does_not_count_as_sampled_task_success(self):
        got = self.diag.evaluate(self.request((5., 0., 0., 0., 0., 0.)), 'qp-test-unachievable')
        self.assertEqual(got['qp']['status'],'FEASIBLE_TRACKING_REJECTED')
        self.assertEqual(got['status'],'FEASIBLE_BUT_TASK_TRACKING_REJECTED')
        self.assertFalse(got['sampled_tracking_candidate'])
        self.assertIsNotNone(got['sampled_edge'])

    def test_saved_payload_collision_still_rejected_with_zero_velocity(self):
        X,q = self.payload_collision
        got = self.diag.evaluate(VelocityInput(q,X,np.zeros(6),.01),'qp-test-payload')
        self.assertEqual(got['qp']['status'],'FEASIBLE_TRACKING_OK')
        self.assertEqual(got['status'],'CURRENT_STATE_GEOMETRY_REJECTED')
        self.assertFalse(got['current_state']['valid'])
        self.assertTrue(any('Cube_01' in r or 'PENETRATION' in r for r in got['current_state']['reasons']))
        self.assertFalse(got['sampled_tracking_candidate'])

    def test_supported_hold_keeps_existing_local_contact_contract(self):
        got = self.diag.evaluate(VelocityInput(self.support_q,self.support_X,np.zeros(6),.01),
                                 'qp-test-supported',site=SupportSite.SUPPLY)
        self.assertEqual(got['status'],'OFFLINE_SAMPLED_STEP_VALID_UNCERTIFIED')
        collision = got['current_state']['collision']
        self.assertTrue(collision['support']['valid'])
        self.assertFalse(collision['support']['physical_support_observed'])
        self.assertEqual(len(collision['accepted_support_contacts']),1)
        self.assertEqual(got['phase_confirmation_observations_generated'],0)

    def test_input_rejection_has_no_predicted_step(self):
        got=self.diag.evaluate(VelocityInput(None,self.X,np.zeros(6),.01),'qp-test-invalid')
        self.assertEqual(got['status'],'INPUT_REJECTED')
        self.assertIsNone(got['sampled_edge'])

    def test_object_increment_uses_center_not_world_origin(self):
        X=transform(translation=(.7,-.03,.4))
        got=integrate_object_world_center(X,[0.,0.,0.,0.,0.,.2],.01)
        np.testing.assert_array_equal(got[:3,3],X[:3,3])
        self.assertGreater(np.linalg.norm(got[:3,:3]-X[:3,:3]),.001)
        for dt in (0.,-1.,np.nan,True):
            with self.assertRaises(ValueError):integrate_object_world_center(X,np.zeros(6),dt)


if __name__=='__main__':unittest.main()
