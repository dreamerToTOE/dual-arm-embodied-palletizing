"""Synthetic mathematics and input-contract controls, not physical contact."""
from dataclasses import replace
import numpy as np
import pytest

from offline.task28_g1_1.task28_validator.se3 import transform
from offline.task28_p2_3.tests.test_integration import sample
from offline.task28_p2_3_r1.replay import replay_virtual_pair


def test_nonclosing_synthetic_object_still_produces_virtual_mechanics():
    left, right, obj, cfg, limits, truth = sample()
    # Keep fixed for this diagnostic: G1 must fail and replay must record that.
    obj = transform(translation=obj[:3, 3] + [0.02, -0.01, 0.03])
    got = replay_virtual_pair(left, right, obj, cfg, limits)
    assert not got['frozen_g1_geometry']['loaded']['valid']
    assert not got['frozen_g1_geometry']['unloaded']['valid']
    assert got['execution_authorized'] is False
    assert got['physical_contact_measured'] is False
    assert got['unloaded_loaded_physically_synchronous'] is False
    force = np.zeros(3)
    moment = np.zeros(3)
    for side, pair in (('left', left), ('right', right)):
        applied = truth[side]
        f, m = -applied[:3], -applied[3:]
        nominal = got[side]['nominal_world_tcp'][:3, 3]
        actual = pair.live_world_tcp[:3, 3]
        np.testing.assert_allclose(got[side]['on_robot_world_tcp'], applied, atol=1e-10)
        np.testing.assert_allclose(got[side]['on_cube_world_nominal_tcp'],
            np.r_[f, m + np.cross(actual - nominal, f)], atol=1e-10)
        force += f
        moment += m + np.cross(actual - obj[:3, 3], f)
    np.testing.assert_allclose(got['net_cube_wrench_world_center'], np.r_[force, moment], atol=1e-8)
    assert got['grasp_null_residual'] < 1e-10
    assert got['decomposition_reconstruction_residual'] < 1e-10


@pytest.mark.parametrize('mutation,reason', [
    ('arm', 'ARM_IDENTITY_MISMATCH'),
    ('frame', 'INVALID_CUBE_OR_GRASP_FRAME'),
    ('order', 'JOINT_ORDER_MISMATCH'),
    ('sign', 'UNSUPPORTED_OR_UNCALIBRATED_SIGN'),
    ('stale', 'STALE_OBSERVATION'),
    ('calibration', 'UNCALIBRATED_ARM_SOURCE'),
    ('missing_hash', 'MISSING_INDEPENDENT_SIGN_CALIBRATION'),
    ('async', 'ASYNCHRONOUS_BILATERAL_STEPS'),
    ('missing_pose', 'MISSING_LIVE_TCP_PROVENANCE'),
    ('wrong_live_pose', 'LIVE_FK_MISMATCH'),
    ('real_contact', 'REAL_CONTACT_NOT_ESTABLISHED'),
])
def test_replay_rejects_bad_provenance_and_state(mutation, reason):
    left, right, obj, cfg, limits, _ = sample()
    if mutation == 'arm': right = replace(right, articulation_id=left.articulation_id)
    elif mutation == 'frame': cfg = replace(cfg, right_tcp_id='')
    elif mutation == 'order': right = replace(right, loaded=replace(right.loaded, joint_names=right.loaded.joint_names[::-1]))
    elif mutation == 'sign': right = replace(right, calibration=replace(right.calibration, semantic='wrong'))
    elif mutation == 'stale': right = replace(right, loaded=replace(right.loaded, evaluation_timestamp_s=100.))
    elif mutation == 'calibration': right = replace(right, calibration_provenance='')
    elif mutation == 'missing_hash': right = replace(right, calibration=replace(right.calibration, evidence_sha256=''))
    elif mutation == 'async': right = replace(right, loaded_steps=(201, 202, 203))
    elif mutation == 'missing_pose': right = replace(right, live_tcp_provenance='')
    elif mutation == 'wrong_live_pose':
        bad = right.live_world_tcp.copy(); bad[0, 3] += .01
        right = replace(right, live_world_tcp=bad)
    elif mutation == 'real_contact': cfg = replace(cfg, observation_scope='REAL_SUCTION_CONTACT')
    with pytest.raises(ValueError, match=reason):
        replay_virtual_pair(left, right, obj, cfg, limits)


def test_wrong_moment_arm_cannot_fake_the_correct_center_result():
    left, right, obj, cfg, limits, truth = sample()
    shifted = transform(translation=obj[:3, 3] + [.02, -.01, .03])
    got = replay_virtual_pair(left, right, shifted, cfg, limits)
    bad = np.zeros(3)
    for side in ('left', 'right'):
        # Wrongly treating the measured force as applied at the nominal point.
        f, m = -truth[side][:3], -truth[side][3:]
        nominal = got[side]['nominal_world_tcp'][:3, 3]
        bad += m + np.cross(nominal - shifted[:3, 3], f)
    assert np.linalg.norm(bad - got['net_cube_wrench_world_center'][3:]) > 1e-3


def test_archived_reference_is_fixed_not_fitted_to_live_pose():
    left, right, obj, cfg, limits, _ = sample()
    cfg = replace(cfg, source_kind='ISAAC_TWO_ARTICULATIONS_STANDALONE')
    shifted = transform(translation=[0.65, 0., 0.401])
    with pytest.raises(ValueError, match='ARCHIVED_FIXED_OBJECT_REFERENCE_MISMATCH'):
        replay_virtual_pair(left, right, shifted, cfg, limits)


def test_wrong_action_reaction_is_rejected_by_independent_center_oracle():
    left, right, obj, cfg, limits, truth = sample()
    result = replay_virtual_pair(left, right, obj, cfg, limits)
    independent = np.zeros(6)
    for side, pair in (('left', left), ('right', right)):
        robot = truth[side]
        on_cube_f, on_cube_m = -robot[:3], -robot[3:]
        r = pair.live_world_tcp[:3, 3] - obj[:3, 3]
        independent += np.r_[on_cube_f, on_cube_m + np.cross(r, on_cube_f)]
    np.testing.assert_allclose(result['net_cube_wrench_world_center'], independent, atol=1e-8)
    # Incorrectly retaining on-robot action instead of negating it.
    faulty_net = -result['net_cube_wrench_world_center']
    rejection_reasons = []
    if np.linalg.norm(faulty_net[:3] - independent[:3]) > 0.01:
        rejection_reasons.append('OBJECT_FORCE_MISMATCH')
    if np.linalg.norm(faulty_net[3:] - independent[3:]) > 0.01:
        rejection_reasons.append('OBJECT_MOMENT_MISMATCH')
    assert 'OBJECT_FORCE_MISMATCH' in rejection_reasons
    assert 'OBJECT_MOMENT_MISMATCH' in rejection_reasons
