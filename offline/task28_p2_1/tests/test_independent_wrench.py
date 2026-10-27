"""Independent, hand-derived oracles for nominal ±61 mm TCP wrenches.

Inputs are synthetic and all tests are CPU-only. These do not establish
physical contact wrench observability or a suction capacity.
"""
import numpy as np
import pytest

from offline.task28_p2_1.grasp import (
    GraspModel, build_observation, metric_matrix, request_physical_estimate,
)


# These explicitly encode the already frozen G1-1 convention; they are not
# obtained from GraspModel or nominal_grasps().  Columns map local axes to O.
RL = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
RR = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
RZ = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


def transform(rotation=RZ, origin=(1., 2., 3.)):
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = origin
    return t


def from_object_axes(left, right):
    """Independent conversion of two object-axis TCP wrenches to local."""
    left, right = np.asarray(left), np.asarray(right)
    return np.r_[RL.T @ left[:3], RL.T @ left[3:],
                 RR.T @ right[:3], RR.T @ right[3:]]


def oracle_object(left, right):
    """Explicit r×f expansion with left r=(0,-.061,0), right r=(0,.061,0)."""
    f_l, f_r = np.asarray(left[:3]), np.asarray(right[:3])
    tau = np.asarray(left[3:]) + np.asarray(right[3:])
    torque = tau + np.array([-.061*f_l[2] + .061*f_r[2], 0.,
                             .061*f_l[0] - .061*f_r[0]])
    return np.r_[f_l + f_r, torque]


@pytest.mark.parametrize('left,right', [
    ([0., 0., 0., 0., 0., 0.], [0., 0., 0., 0., 0., 0.]),
    ([1., 0., 0., 0., 0., 0.], [1., 0., 0., 0., 0., 0.]),
    ([0., 0., 0., 1., 2., 3.], [0., 0., 0., -2., 3., 4.]),
    ([4., 2., 3., .2, -.4, .1], [-1., -3., -2., -.1, .6, .2]),
    ([0., 1., 0., 0., 0., 0.], [0., -1., 0., 0., 0., 0.]),
])
def test_independent_force_and_moment_oracle(left, right):
    model = GraspModel.frozen(transform())
    local = from_object_axes(left, right)
    obj = oracle_object(left, right)
    np.testing.assert_allclose(model.net(local, 'object'), obj, atol=1e-14)
    np.testing.assert_allclose(model.net(local, 'world'),
                               np.r_[RZ @ obj[:3], RZ @ obj[3:]], atol=1e-14)


def test_offset_moment_and_force_torque_order_are_not_interchangeable():
    model = GraspModel.frozen(np.eye(4))
    left = np.array([2., 0., 0., 0., 0., 0.])
    x = from_object_axes(left, np.zeros(6))
    np.testing.assert_allclose(model.net(x), [2., 0., 0., 0., 0., .122], atol=1e-15)
    wrong = np.r_[x[3:6], x[:3], np.zeros(6)]
    assert not np.allclose(model.net(wrong), model.net(x))
    assert not np.allclose(model.net(-x), model.net(x))


def test_opposing_axial_forces_are_internal_but_equal_forces_load_object():
    model = GraspModel.frozen(np.eye(4))
    plus_y = np.array([0., 5., 0., 0., 0., 0.])
    internal = from_object_axes(plus_y, -plus_y)
    same = from_object_axes(plus_y, plus_y)
    np.testing.assert_allclose(model.net(internal), np.zeros(6), atol=1e-14)
    np.testing.assert_allclose(model.net(same), [0., 10., 0., 0., 0., 0.], atol=1e-14)
    d = model.decompose(internal)
    np.testing.assert_allclose(d.external, np.zeros(12), atol=1e-13)
    np.testing.assert_allclose(d.internal, internal, atol=1e-13)


def test_mirrored_rotated_frame_roundtrip_and_center_shift():
    model = GraspModel.frozen(transform())
    sample = np.array([1., 2., 3., 4., 5., 6.])
    for side, r in [('left', RL), ('right', RR)]:
        expected = np.r_[RZ @ r @ sample[:3], RZ @ r @ sample[3:]]
        actual = model.contact_to_world(side, sample)
        np.testing.assert_allclose(actual, expected, atol=1e-15)
        np.testing.assert_allclose(model.world_to_contact(side, actual), sample, atol=1e-15)
    # Changing world origin cannot change a wrench about the Cube center.
    other = GraspModel.frozen(transform(origin=(-20., 7., 44.)))
    np.testing.assert_allclose(model.matrix(), other.matrix(), atol=1e-15)


def test_independent_contact_to_center_moment_and_inverse():
    model = GraspModel.frozen(transform())
    left = np.array([2., 3., 4., .4, .5, .6])
    right = np.array([-5., -6., 7., .7, .8, .9])
    x = from_object_axes(left, right)
    for side, local, obj in [('left', x[:6], oracle_object(left, np.zeros(6))),
                             ('right', x[6:], oracle_object(np.zeros(6), right))]:
        world = np.r_[RZ @ obj[:3], RZ @ obj[3:]]
        np.testing.assert_allclose(model.contact_to_center(side, local, 'object'), obj, atol=1e-14)
        np.testing.assert_allclose(model.contact_to_center(side, local, 'world'), world, atol=1e-14)
        np.testing.assert_allclose(model.center_to_contact(side, obj, 'object'), local, atol=1e-14)
        np.testing.assert_allclose(model.center_to_contact(side, world, 'world'), local, atol=1e-14)


def test_rank_and_weighted_projection_against_independent_kkt_solution():
    model = GraspModel.frozen(transform())
    g = model.matrix()
    assert np.linalg.matrix_rank(g) == 6
    x = np.array([1., -3., .2, .4, .1, -.8, -.5, 2., .9, -.7, .3, 1.1])
    # Full SPD metric, including off-diagonal terms, avoids assuming a
    # Euclidean projection. Solve the KKT stationarity equations directly.
    z = np.arange(144, dtype=float).reshape(12, 12) / 200.
    w = z.T @ z + np.diag(np.linspace(.8, 3., 12))
    wi_gt = np.linalg.solve(w, g.T)
    independently_solved = wi_gt @ np.linalg.solve(g @ wi_gt, g @ x)
    d = model.decompose(x, metric=w)
    np.testing.assert_allclose(d.external, independently_solved, atol=2e-12)
    np.testing.assert_allclose(g @ d.internal, np.zeros(6), atol=2e-12)
    np.testing.assert_allclose(d.external + d.internal, x, atol=1e-14)
    assert abs(d.external @ w @ d.internal) < 2e-12
    assert d.rank == 6
    # Internal components depend on the metric, not a unique physical split.
    assert np.linalg.norm(d.internal - model.decompose(x).internal) > 1e-3


def test_symmetric_allocation_hand_derived():
    model = GraspModel.frozen(np.eye(4))
    result = model.allocate(np.array([2., 0., 0., 0., 0., 0.]))
    expected = from_object_axes([1., 0., 0., 0., 0., 0.],
                                [1., 0., 0., 0., 0., 0.])
    np.testing.assert_allclose(result.contacts, expected, atol=2e-14)
    assert result.residual < 1e-13


def test_asymmetric_weighted_allocation_checked_by_kkt_and_oracle():
    model = GraspModel.frozen(np.eye(4))
    w = metric_matrix()
    w[:6, :6] *= 3.
    target = np.array([3., -2., 5., .2, -.6, .4])
    result = model.allocate(target, metric=w)
    l, r = result.contacts[:6], result.contacts[6:]
    left = np.r_[RL @ l[:3], RL @ l[3:]]
    right = np.r_[RR @ r[:3], RR @ r[3:]]
    np.testing.assert_allclose(oracle_object(left, right), target, atol=2e-13)
    # Feasible alternative from a nonzero null vector cannot improve objective.
    internal = from_object_axes([0., 1., 0., 0., 0., 0.],
                                [0., -1., 0., 0., 0., 0.])
    alternative = result.contacts + internal
    assert .5 * alternative @ w @ alternative > result.objective


@pytest.mark.parametrize('args', [
    ('left', '', 'left_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE', 'force_then_torque_on_cube'),
    ('left', 'Cube_01', '', 'SYNTHETIC_OFFLINE', 'force_then_torque_on_cube'),
    ('left', 'Cube_01', 'left_fr3_side_suction_tcp', '', 'force_then_torque_on_cube'),
    ('left', 'Cube_01', 'left_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE', 'torque_then_force'),
    ('left', 'Cube_02', 'left_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE', 'force_then_torque_on_cube'),
    ('left', 'Cube_01', 'right_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE', 'force_then_torque_on_cube'),
])
def test_missing_or_inconsistent_observation_metadata_is_rejected(args):
    with pytest.raises(ValueError):
        build_observation(*args, np.zeros(6))


def test_only_synthetic_observations_and_physical_request_blocked():
    l = build_observation('left', 'Cube_01', 'left_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE',
                          'force_then_torque_on_cube', np.zeros(6))
    r = build_observation('right', 'Cube_01', 'right_fr3_side_suction_tcp', 'SYNTHETIC_OFFLINE',
                          'force_then_torque_on_cube', np.zeros(6))
    np.testing.assert_array_equal(GraspModel.frozen(np.eye(4)).net_observations(l, r), np.zeros(6))
    with pytest.raises(ValueError):
        GraspModel.frozen(np.eye(4)).net_observations(r, l)
    physical = request_physical_estimate(l, r)
    assert physical['status'] == 'BLOCKED'
    assert physical['physical_estimate'] is None
    assert physical['execution_authorized'] is False


@pytest.mark.parametrize('value', [np.nan, np.inf, -np.inf])
def test_invalid_numerical_inputs_rejected(value):
    model = GraspModel.frozen(np.eye(4))
    wrench = np.zeros(12)
    wrench[1] = value
    with pytest.raises(ValueError):
        model.decompose(wrench)
    with pytest.raises(ValueError):
        model.allocate(np.full(6, value))


def test_unsupported_rank_deficient_allocation_and_metric_rejected():
    model = GraspModel.frozen(np.eye(4))
    # Deliberately high rcond suppresses a genuine singular subspace. This is
    # numerical unsupported-input testing, not a claim of physical infeasibility.
    with pytest.raises(ValueError, match='UNSUPPORTED_REQUIRED_WRENCH'):
        model.allocate([0., 0., 0., 1., 0., 0.], rcond=.99)
    with pytest.raises(ValueError, match='TRUNCATED_DECOMPOSITION_NOT_IN_NULL_SPACE'):
        model.decompose(np.arange(12.), rcond=.99)
    with pytest.raises(ValueError):
        model.allocate(np.zeros(6), metric=np.zeros((12, 12)))
    with pytest.raises(ValueError):
        model.matrix('tcp')
