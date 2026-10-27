"""Pure-data evidence analysis controls; no QP, robot or experiment runs."""
import json
import math
from pathlib import Path
from dataclasses import asdict

import numpy as np
import pytest

from offline.task28_p5_qp_2.feedback import FeedbackConfig
from offline.task28_p5_qp_2.run_experiments import drift_metrics, pair_comparison

ROOT = Path(__file__).resolve().parents[3]


def test_independent_noncommuting_full_pose_drift_components():
    # X has Rx(90°); L=Rz(90°)Rx(90°), R=Ry(90°)Rx(90°).
    # L R^T=Rz(90°)Ry(-90°) has principal angle 120° and
    # world axis (1,-1,1)/sqrt(3). This independently verifies order.
    rx = np.array([[1.,0.,0.], [0.,0.,-1.], [0.,1.,0.]])
    ry = np.array([[0.,0.,1.], [0.,1.,0.], [-1.,0.,0.]])
    rz = np.array([[0.,-1.,0.], [1.,0.,0.], [0.,0.,1.]])
    X, L, R = np.eye(4), np.eye(4), np.eye(4)
    X[:3,:3], L[:3,:3], R[:3,:3] = rx, rz@rx, ry@rx
    X[:3,3] = [.1,.2,.3]
    L[:3,3] = [.4,-.2,.6]
    R[:3,3] = [-.2,.1,.1]
    m = drift_metrics({'implied_left_object':L, 'implied_right_object':R}, X)
    np.testing.assert_allclose(m['relative_translation_world'], [.6,-.3,.5], atol=1e-15)
    np.testing.assert_allclose(m['relative_rotation_log_world'],
        (2*math.pi/(3*math.sqrt(3))) * np.array([1.,-1.,1.]), atol=1e-15)
    np.testing.assert_allclose(m['left_translation_world'], [.3,-.4,.3], atol=1e-15)
    np.testing.assert_allclose(m['right_translation_world'], [-.3,-.1,-.2], atol=1e-15)
    np.testing.assert_allclose(m['mean_translation_world'], [0.,-.25,.05], atol=1e-15)
    np.testing.assert_allclose(m['left_rotation_log_world'], [0.,0.,math.pi/2], atol=1e-15)
    np.testing.assert_allclose(m['right_rotation_log_world'], [0.,math.pi/2,0.], atol=1e-15)
    np.testing.assert_allclose(m['mean_rotation_tangent_world'], [0.,math.pi/4,math.pi/4], atol=1e-15)
    assert m['relative_translation_m'] == pytest.approx(math.sqrt(.6**2+.3**2+.5**2))
    assert m['relative_rotation_rad'] == pytest.approx(2*math.pi/3)
    assert m['left_reference_translation_m'] == pytest.approx(math.sqrt(.34))
    assert m['right_reference_translation_m'] == pytest.approx(math.sqrt(.14))
    assert m['mean_reference_translation_m'] == pytest.approx(math.sqrt(.065))
    assert m['mean_reference_rotation_tangent_rad'] == pytest.approx(math.pi/(2*math.sqrt(2)))
    assert m['execution_authorized'] is False


def case():
    return {'identity':'abstract-case', 'node_identity':'abstract-node', 'seed':28,
            'purpose':'primary_combined', 'dt_s':.05, 'max_steps':40,
            'planned_duration_s':2., 'desired_object_twist':[.005,-.003,.002,.01,-.005,.02]}


def summary(t, r, *, status='COMPLETED_OFFLINE_SAMPLED_UNCERTIFIED', steps=40, duration=2.):
    return {'final_accepted_drift':{'relative_translation_m':t,'relative_rotation_rad':r},
            'accepted_steps':steps, 'status':status, 'accepted_duration_s':duration}


def test_pair_two_dimensional_twenty_percent_boundary_and_same_endpoint():
    baseline = summary(1., 2.)
    exact = pair_comparison(case(), baseline, summary(.8, 1.6))
    assert exact['comparable_at_preregistered_endpoint']
    assert exact['both_relative_metrics_improved_at_least_20_percent']
    assert exact['feedback_over_baseline_relative_translation_ratio'] == pytest.approx(.8)
    assert exact['feedback_over_baseline_relative_rotation_ratio'] == pytest.approx(.8)
    for t,r in ((.81,1.5),(.7,1.62)):
        result = pair_comparison(case(), baseline, summary(t,r))
        assert result['comparable_at_preregistered_endpoint']
        assert not result['both_relative_metrics_improved_at_least_20_percent']
    assert exact['execution_authorized'] is False


@pytest.mark.parametrize('baseline,feedback', [
    (summary(1.,2.,steps=39,duration=1.95), summary(.1,.2)),
    (summary(1.,2.), summary(.1,.2,status='STOPPED_AT_FIRST_REJECTION',steps=39,duration=1.95)),
    (summary(0.,2.), summary(.1,.2)),
    (summary(1.,0.), summary(.1,.2)),
    (summary(1e-8,2.), summary(.1,.2)),
    (summary(1.,1e-8), summary(.1,.2)),
    (summary(1.,2.,duration=1.95), summary(.1,.2)),
    (summary(1.,2.), summary(.1,.2,duration=1.95)),
])
def test_incomplete_below_noise_zero_or_time_mismatch_is_not_comparable(baseline, feedback):
    result = pair_comparison(case(), baseline, feedback)
    assert not result['comparable_at_preregistered_endpoint']
    assert result['not_comparable_reason']
    assert result['feedback_over_baseline_relative_translation_ratio'] is None
    assert result['feedback_over_baseline_relative_rotation_ratio'] is None
    assert not result['both_relative_metrics_improved_at_least_20_percent']


def test_registered_matrix_gains_budgets_and_no_parameter_tuning():
    with open(ROOT/'offline/task28_p5_qp_2/evidence/preregistration.json') as f:
        cfg = json.load(f)
    cases = cfg['cases']
    assert len(cases) == 15
    assert len(cases)*2+2 == 32
    assert sum(c['purpose']=='primary_combined' for c in cases) == 6
    for seed in (28,29,30):
        primary = [c for c in cases if c['seed']==seed and c['purpose']=='primary_combined']
        assert {c['dt_s'] for c in primary} == {.02,.05}
        assert {c['planned_duration_s'] for c in primary} == {2.}
        assert all(c['max_steps']*c['dt_s']==pytest.approx(2.) for c in primary)
        assert all(c['methods']==['baseline','feedback'] for c in primary)
    gains = asdict(FeedbackConfig())
    expected = {'relative_translation_gain_s':2., 'relative_rotation_gain_s':2.,
                'object_translation_gain_s':.5, 'object_rotation_gain_s':.5}
    for key, value in expected.items():
        assert cfg['gains'][key] == gains[key] == value
    assert cfg['budget']['formal_batches'] == 1
    assert cfg['budget']['max_rollout_steps'] == 1922
    assert 'No post-run tuning' in cfg['primary_analysis']
    assert 'all primary comparable' in cfg['acceptance']


@pytest.mark.parametrize('t,r', [(float('nan'), .2), (.1, float('inf')), (-.1, .2), (.1, -.2)])
def test_nonfinite_or_negative_feedback_drift_is_never_comparable(t, r):
    result = pair_comparison(case(), summary(1.,2.), summary(t,r))
    assert not result['comparable_at_preregistered_endpoint']
    assert result['not_comparable_reason']
    assert result['feedback_over_baseline_relative_translation_ratio'] is None
    assert result['feedback_over_baseline_relative_rotation_ratio'] is None


def test_misregistered_dt_steps_and_planned_duration_is_not_comparable():
    bad = dict(case(), planned_duration_s=3.)
    result = pair_comparison(bad, summary(1.,2.), summary(.1,.2))
    assert not result['comparable_at_preregistered_endpoint']
    assert result['not_comparable_reason']
