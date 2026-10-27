"""Software controls and saved frozen FR3 states; no physical execution."""
from dataclasses import replace
import gzip
import json
from pathlib import Path

import numpy as np

from offline.task28_p4_nr_0.validation import CandidateValidator
from offline.task28_p5_qp_0.core import VelocityResult
from offline.task28_p5_qp_1.driver import MultiStepDriver, MultiStepRequest
from offline.task28_p5_qp_2.driver import FeedbackDriver, run_comparison
from offline.task28_p5_qp_2.feedback import FeedbackVelocityQP
from task28_edge.types import EdgeStatus

ROOT = Path(__file__).resolve().parents[3]


def node():
    path = ROOT / 'offline/task28_p4_nr_0/evidence/experiments_v1/seed_28/result.json.gz'
    with gzip.open(path, 'rt') as f:
        rows = json.load(f)['transport']
    return rows[len(rows)//2]


def request(steps=2):
    X, q = node()
    return MultiStepRequest(q, X, [.005, -.003, .002, .01, -.005, .02], .01, steps, 'driver-unit')


class ForgedReceipt:
    def __init__(self):
        self.real = FeedbackVelocityQP()

    def __getattr__(self, name):
        return getattr(self.real, name)

    def solve(self, req):
        honest = self.real.solve(req)
        assert honest.qdot is not None
        return replace(honest, status='FEASIBLE_TRACKING_OK', qdot=(0.,)*14, reasons=())


class InfeasibleReceipt:
    def __init__(self):
        self.real = FeedbackVelocityQP()

    def __getattr__(self, name):
        return getattr(self.real, name)

    def solve(self, req):
        return VelocityResult('QP_INFEASIBLE', None, ('ABSTRACT_INFEASIBLE_CONTROL',))


class InjectedEdge:
    def __init__(self, status):
        self.real = CandidateValidator()
        self.checker = self
        self.status = status
        self.calls = 0

    def trajectory(self, *args, **kwargs):
        return self.real.trajectory(*args, **kwargs)

    def evaluate(self, *args, **kwargs):
        return self.real.checker.evaluate(*args, **kwargs)

    def validate(self, *args, **kwargs):
        self.calls += 1
        receipt = self.real.checker.validate(*args, **kwargs)
        return replace(receipt, status=self.status, reasons=('ABSTRACT_TEST_EDGE_FAILURE',))


def test_original_qp1_baseline_path_exact_state_and_geometry():
    req = request()
    result = run_comparison(req.q14, req.object_pose, req.desired_object_twist,
                            req.dt_s, req.max_steps, req.identity)
    assert result['status'] == 'COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED'
    assert result['accepted_step_count'] == 2
    assert result['phase_confirmation_observations_generated'] == 0
    assert not result['execution_authorized']
    direct = MultiStepDriver().run(req)
    np.testing.assert_array_equal(result['retained_state']['q14'], direct['retained_state']['q14'])
    np.testing.assert_array_equal(result['retained_state']['object_pose'], direct['retained_state']['object_pose'])


def test_feedback_valid_frozen_fr3_uses_exact_euler_and_original_payload_gates():
    req = request()
    result = FeedbackDriver().run(req)
    assert result['status'] == 'COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED', result.get('failure')
    assert result['accepted_step_count'] == 2
    assert result['phase_confirmation_observations_generated'] == 0
    assert not result['execution_authorized'] and not result['continuous_verifier_run']
    assert not result['nr_projection_performed']
    q = np.asarray(req.q14)
    for i, step in enumerate(result['steps']):
        assert step['time_s'] == i*req.dt_s and step['target_time_s'] == (i+1)*req.dt_s
        np.testing.assert_array_equal(step['q_initial'], q)
        q = q + req.dt_s*np.asarray(step['qdot14'])
        np.testing.assert_array_equal(step['q_next'], q)
        assert step['independent_velocity_diagnostics']['affine_residual_inf'] <= 1e-8
        for key in ('current_state', 'predicted_state'):
            assert step[key]['valid']
            collision = step[key]['collision']['reference']
            assert collision['checked_shape_pairs'] == 414
            assert collision['skipped_acm_shape_pairs'] == 40
        assert step['sampled_edge']['status'] == EdgeStatus.SAMPLED_EDGE_VALID.value
    np.testing.assert_array_equal(result['retained_state']['q14'], q)


def test_feasible_receipt_cannot_bypass_original_tracking_or_affine_gate():
    req = request()
    result = FeedbackDriver(controller=ForgedReceipt()).run(req)
    assert result['status'] == 'STOPPED_AT_FIRST_REJECTION'
    assert result['attempted_step_count'] == 1 and result['accepted_step_count'] == 0
    failed = result['steps'][0]
    assert failed['qp']['status'] == 'FEASIBLE_TRACKING_OK'
    assert 'INDEPENDENT_TASK_TRACKING_REJECTED' in failed['reasons']
    assert 'INDEPENDENT_AFFINE_EQUALITY_REJECTED' in failed['reasons']
    assert failed['q_next'] is not None and failed['predicted_state'] is not None
    assert result['injected_backend_scope'] == 'ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE'
    np.testing.assert_array_equal(result['retained_state']['q14'], req.q14)


def test_edge_rejection_and_inconclusive_stop_with_full_candidate():
    for status in (EdgeStatus.SAMPLED_EDGE_REJECTED, EdgeStatus.EDGE_INCONCLUSIVE):
        validator = InjectedEdge(status)
        result = FeedbackDriver(validator=validator).run(request())
        assert result['status'] == 'STOPPED_AT_FIRST_REJECTION'
        assert validator.calls == 1 and result['accepted_step_count'] == 0
        assert result['steps'][0]['sampled_edge']['status'] == status.value
        assert result['steps'][0]['q_next'] is not None
        assert result['injected_backend_scope'] == 'ABSTRACT_INJECTION_NOT_ROBOT_FEASIBILITY_EVIDENCE'


def test_archived_payload_collision_rejected_before_feedback_solve():
    path = ROOT/'offline/task28_p4_nr_0/evidence/experiments_v1/negative_controls.json.gz'
    with gzip.open(path, 'rt') as f:
        controls = json.load(f)
    X, q = next(row['node'] for row in controls if row['case'] == 'payload_table_collision')
    req = replace(request(), q14=q, object_pose=X, desired_object_twist=np.zeros(6))
    result = FeedbackDriver().run(req)
    assert result['status'] == 'STOPPED_AT_FIRST_REJECTION'
    assert result['attempted_step_count'] == 0 and result['accepted_step_count'] == 0
    assert not result['initial_state']['valid']
    assert any('Cube_01' in x or 'PENETRATION' in x for x in result['initial_state']['reasons'])
    assert result['phase_confirmation_observations_generated'] == 0


def test_invalid_input_is_rejected_without_stepping():
    req = request()
    for invalid in (replace(req, q14=None), replace(req, cube_id='Cube_99'),
                    replace(req, frame_id='map'), replace(req, dt_s=0),
                    replace(req, max_steps=True)):
        result = FeedbackDriver().run(invalid)
        assert result['status'] == 'INPUT_REJECTED'
        assert result['attempted_step_count'] == 0


def test_infeasible_feedback_cannot_fall_back_to_unconstrained_or_baseline_qp():
    req = request()
    result = FeedbackDriver(controller=InfeasibleReceipt()).run(req)
    assert result['status'] == 'STOPPED_AT_FIRST_REJECTION'
    assert result['accepted_step_count'] == 0 and result['attempted_step_count'] == 1
    assert result['steps'][0]['qp']['status'] == 'QP_INFEASIBLE'
    assert result['steps'][0]['q_next'] is None
    assert 'QP_QP_INFEASIBLE' in result['steps'][0]['reasons']
    np.testing.assert_array_equal(result['retained_state']['q14'], req.q14)
