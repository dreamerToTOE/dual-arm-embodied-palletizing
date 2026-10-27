"""Synthetic contracts only; these are not Isaac or physical observations."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
import pytest

from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.se3 import transform
from offline.task28_p2_1.grasp import GraspModel
from offline.task28_p2_2_r1.adapter import SOURCE, SEMANTIC, StaticWindow, StaticLimits, SignCalibration
from offline.task28_p2_3.integration import ArmPair, IntegrationContract, JOINT_NAMES, integrate_virtual_pair


def sample():
    root = Path(__file__).resolve().parents[3]
    evidence = json.loads((root / 'offline/task28_g1_1/evidence/validation_results.json').read_text())
    control = evidence['positive_full_pipeline_controls'][0]
    obj = transform(translation=control['object_position_m'])
    model = RobotModel()
    contact_on_robot = {'left': np.array([1., 2., 3., .2, -.3, .4]),
                        'right': np.array([-1., -.5, .25, -.1, .2, .05])}
    arms = []
    for side in ('left', 'right'):
        q = np.asarray(control['ik'][side]['q'])
        J = model.jacobian(side, q)
        bias = np.arange(7) * .2
        kw = dict(q_rad=np.tile(q, (3, 1)), dq_rad_s=np.zeros((3, 7)),
                  timestamps_s=[1., 1.05, 1.1], evaluation_timestamp_s=1.1,
                  joint_names=JOINT_NAMES, source=SOURCE)
        unloaded = StaticWindow(effort_Nm=np.tile(bias, (3, 1)), **kw)
        loaded = replace(unloaded, effort_Nm=np.tile(bias - J.T @ contact_on_robot[side], (3, 1)))
        identity = '/isolated/' + side
        arms.append(ArmPair(side, identity, unloaded, loaded, 'unloaded_reset', 'loaded_reset',
                            (100,101,102), (200,201,202), model.fk(side, q),
                            SignCalibration(SOURCE, SEMANTIC, True, 'a'*64), identity, True,
                            'synthetic fixture control; not real sign evidence', 'synthetic FK'))
    cfg = IntegrationContract('/isolated/left', '/isolated/right', 'Cube_01',
                              'left_fr3_side_suction_tcp', 'right_fr3_side_suction_tcp',
                              'SYNTHETIC_TEST', 0., 1e-7, 1e-7, 1e-6, 1e-6, .05)
    return arms[0], arms[1], obj, cfg, StaticLimits(.2, .01, .002, .05, .002), contact_on_robot


def test_independent_virtual_action_reaction_and_center_shift():
    left, right, obj, cfg, limits, truth = sample()
    got = integrate_virtual_pair(left, right, obj, cfg, limits)
    grasp = GraspModel.frozen(obj)
    # Independent mechanics: -w_on_robot, torque about center = tau_tcp + r x F.
    fsum, msum = np.zeros(3), np.zeros(3)
    for side, pair in (('left', left), ('right', right)):
        force, moment = -truth[side][:3], -truth[side][3:]
        lever = np.asarray(pair.live_world_tcp)[:3, 3] - obj[:3, 3]
        fsum += force
        msum += moment + np.cross(lever, force)
        np.testing.assert_allclose(got[side]['on_robot_world_tcp'], truth[side], atol=1e-10)
        np.testing.assert_allclose(grasp.contact_to_world(side, got[side]['on_cube_local_tcp']),
                                   got[side]['on_cube_world_nominal_tcp'], atol=1e-10)
    np.testing.assert_allclose(got['net_cube_wrench_world_center'], np.r_[fsum, msum], atol=1e-7)
    assert got['grasp_null_residual'] < 1e-12
    assert got['execution_authorized'] is False and got['physical_contact_measured'] is False
    assert got['unloaded_loaded_physically_synchronous'] is False


@pytest.mark.parametrize('mutation,reason', [
    ('arm', 'ARM_IDENTITY_MISMATCH'), ('order', 'JOINT_ORDER_MISMATCH'),
    ('sign', 'UNSUPPORTED_OR_UNCALIBRATED_SIGN'), ('cube', 'INVALID_CUBE_OR_GRASP_FRAME'),
    ('tcp', 'INVALID_CUBE_OR_GRASP_FRAME'), ('trial', 'MISMATCHED_BILATERAL_TRIAL'),
    ('steps', 'ASYNCHRONOUS_BILATERAL_STEPS'), ('stale', 'STALE_OBSERVATION'),
    ('nonfinite', 'INVALID_EFFORT'), ('uncalibrated', 'UNCALIBRATED_ARM_SOURCE'),
    ('fk', 'LIVE_FK_MISMATCH'), ('object', 'FROZEN_GRASP_MISMATCH'),
    ('real', 'REAL_CONTACT_NOT_ESTABLISHED'), ('missing', 'MISSING_OBSERVATION_WINDOW'),
    ('duplicate_id', 'DUPLICATE_PAIR_EVIDENCE'), ('duplicate_window', 'DUPLICATE_PAIR_EVIDENCE'),
])
def test_fail_closed_controls(mutation, reason):
    l,r,obj,cfg,lim,_ = sample()
    if mutation == 'arm': r = replace(r, articulation_id=l.articulation_id)
    elif mutation == 'order': r = replace(r, loaded=replace(r.loaded, joint_names=JOINT_NAMES[::-1]))
    elif mutation == 'sign': r = replace(r, calibration=replace(r.calibration, semantic='wrong'))
    elif mutation == 'cube': cfg = replace(cfg, cube_id='Cube_02')
    elif mutation == 'tcp': cfg = replace(cfg, right_tcp_id='flange')
    elif mutation == 'trial': r = replace(r, loaded_trial_id='different_reset')
    elif mutation == 'steps': r = replace(r, loaded_steps=(201,202,203))
    elif mutation == 'stale': r = replace(r, loaded=replace(r.loaded, evaluation_timestamp_s=2.))
    elif mutation == 'nonfinite': r = replace(r, loaded=replace(r.loaded, effort_Nm=np.full((3,7), np.nan)))
    elif mutation == 'uncalibrated': r = replace(r, calibration_verified_for_articulation=False)
    elif mutation == 'fk':
        m = r.live_world_tcp.copy(); m[0,3] += .01; r = replace(r, live_world_tcp=m)
    elif mutation == 'object': obj = transform(translation=(.65, 0, .5))
    elif mutation == 'real': cfg = replace(cfg, observation_scope='REAL_SUCTION_CONTACT')
    elif mutation == 'missing': r = replace(r, loaded_steps=())
    elif mutation == 'duplicate_id': r = replace(r, loaded_trial_id=r.unloaded_trial_id)
    elif mutation == 'duplicate_window': r = replace(r, loaded=r.unloaded)
    with pytest.raises(ValueError, match=reason):
        integrate_virtual_pair(l,r,obj,cfg,lim)


def test_rank_deficiency_rejected(monkeypatch):
    l,r,obj,cfg,lim,_ = sample()
    monkeypatch.setattr(RobotModel, 'jacobian', lambda *_: np.zeros((6,7)))
    with pytest.raises(ValueError, match='JACOBIAN_RANK_DEFICIENT'):
        integrate_virtual_pair(l,r,obj,cfg,lim)


def test_reference_point_shift_is_applied_before_local_rotation():
    left, right, obj, cfg, limits, truth = sample()
    # Deliberately displace measured TCP within the separately set FK bound.
    live = left.live_world_tcp.copy()
    live[:3, 3] += np.array([1e-5, -2e-5, 3e-5])
    left = replace(left, live_world_tcp=live)
    cfg = replace(cfg, max_fk_translation_m=1e-4)
    got = integrate_virtual_pair(left, right, obj, cfg, limits)
    nominal = obj @ GraspModel.frozen(obj).object_left
    delta = live[:3, 3] - nominal[:3, 3]
    expected_moment = -truth['left'][3:] + np.cross(delta, -truth['left'][:3])
    np.testing.assert_allclose(got['left']['reference_shift_actual_minus_nominal_m'], delta)
    np.testing.assert_allclose(got['left']['on_cube_world_nominal_tcp'][3:], expected_moment, atol=1e-12)


def test_small_nonzero_singular_value_rejected(monkeypatch):
    l,r,obj,cfg,lim,_ = sample()
    monkeypatch.setattr(RobotModel, 'jacobian', lambda *_: np.column_stack((np.diag([1.,1.,1.,1.,1.,.049]), np.zeros(6))))
    with pytest.raises(ValueError, match='JACOBIAN_SINGULAR_VALUE_BELOW_THRESHOLD'):
        integrate_virtual_pair(l,r,obj,cfg,lim)


def test_relative_closure_rejected_even_when_each_side_is_near_object(monkeypatch):
    left, right, obj, cfg, limits, _ = sample()
    original_fk = RobotModel.fk
    def offset_fk(model, side, q):
        pose = original_fk(model, side, q).copy()
        pose[0, 3] += 7.5e-5 if side == 'left' else -7.5e-5
        return pose
    monkeypatch.setattr(RobotModel, 'fk', offset_fk)
    cfg = replace(cfg, max_grasp_translation_m=1e-4)
    with pytest.raises(ValueError, match='FROZEN_G1_CLOSURE_MISMATCH'):
        integrate_virtual_pair(left, right, obj, cfg, limits)


def test_unloaded_state_must_also_satisfy_closure():
    left, right, obj, cfg, limits, _ = sample()
    q = np.asarray(left.unloaded.q_rad).copy()
    q[:, 0] += 0.005
    left = replace(left, unloaded=replace(left.unloaded, q_rad=q))
    with pytest.raises(ValueError, match='UNLOADED_G1_CLOSURE_MISMATCH'):
        integrate_virtual_pair(left, right, obj, cfg, limits)
