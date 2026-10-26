"""Independent mathematical controls for the interval certificate primitives."""
from decimal import Decimal, localcontext
from fractions import Fraction as F
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from continuous_verifier import (I, SCALE, VERIFIED, REJECTED, ProofModel,
                                 bernstein, closure_bounds, identity, polynomial_range,
                                 quaternion_matrix, rz, same_rotation, separation,
                                 split_controls, SampledEdgeValidator)


def _power_value(coefficients, u):
    # Horner evaluation is independent of the Bernstein restriction machinery.
    a, b, c, d = (F.from_float(float(value)) for value in coefficients)
    return ((a*u+b)*u+c)*u+d


def _bernstein_value(controls, u):
    return ((1-u)**3*controls[0] + 3*(1-u)**2*u*controls[1]
            + 3*(1-u)*u*u*controls[2] + u**3*controls[3])


def test_bernstein_ranges_enclose_independent_exact_horner_values():
    rng = random.Random(28101)
    coefficients = [(1e16, -1e16, 3., -2.), (0., 0., -1., 2.),
                    (.1, -.3, .7, -.2), (1e-200, -1e-200, 0., 0.)]
    coefficients += [tuple(rng.uniform(-8, 8) for _ in range(4)) for _ in range(25)]
    intervals = [(F(0), F(1)), (F(1, 4), F(3, 4)), (F(7, 8), F(1)),
                 (F(0), F(1, 16)), (F(1, 3), F(2, 3))]
    for coefficient in coefficients:
        controls = bernstein(coefficient)
        for lower, upper in intervals:
            bound = polynomial_range(controls, lower, upper)
            for sample in range(17):
                u = lower+(upper-lower)*sample/16
                assert bound.contains(_power_value(coefficient, u))
        for point in (F(0), F(1, 7), F(1, 2), F(1)):
            bound = polynomial_range(controls, point, point)
            assert bound.contains(_power_value(coefficient, point))
            assert bound.width <= F(1, SCALE)


def test_de_casteljau_subcurves_have_exact_original_parameterization():
    controls = (F(-2, 3), F(17, 13), F(-19, 11), F(4, 7))
    for split in (F(1, 4), F(1, 3), F(3, 4)):
        left, right = split_controls(controls, split)
        for u in (F(0), F(1, 5), F(1, 2), F(4, 5), F(1)):
            assert _bernstein_value(left, u) == _bernstein_value(controls, split*u)
            assert _bernstein_value(right, u) == _bernstein_value(controls, split+(1-split)*u)


def _box(x, y=(-1, 1), z=(-1, 1)):
    return [I.from_bounds(*bounds) for bounds in (x, y, z)]


def test_aabb_strict_separation_never_accepts_touch_or_overlap():
    unit = _box((-1, 1))
    positive = separation(unit, _box((2, 3)))
    assert positive == {'axis': 0, 'direction': 1, 'gap_lower_scaled': SCALE}
    negative = separation(unit, _box((-4, -3)))
    assert negative == {'axis': 0, 'direction': -1, 'gap_lower_scaled': 2*SCALE}
    assert separation(unit, _box((1, 3))) is None
    assert separation(unit, _box((F(1, 2), 3))) is None
    assert separation(unit, unit) is None
    # A real but sub-grid gap must be inconclusive, not rounded into a proof.
    assert separation(unit, _box((1+F(1, 2**100), 3))) is None
    assert separation(unit, _box((-1, 1), (2, 3)))['axis'] == 1
    assert separation(unit, _box((-1, 1), (-1, 1), (-3, -2)))['axis'] == 2


def test_quaternion_normalization_and_exact_proportionality():
    assert same_rotation((0., 0., 1., 1.), (0., 0., -2., -2.))
    assert not same_rotation((0., 0., 1., 1.), (0., 0., 1., 1.0000000000000002))
    quarter = quaternion_matrix((0., 0., 1., 1.))
    expected = [[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    for r in range(4):
        for c in range(4):
            assert quarter[r][c].lo == quarter[r][c].hi == expected[r][c]
    with pytest.raises(ValueError):
        quaternion_matrix((0., 0., 0., 0.))


def test_all_three_full_se3_closure_pairs_are_required():
    base = identity()
    assert all(row['status'] == VERIFIED for row in closure_bounds(base, base, base))
    below = identity()
    below[0][3] = I(9e-5)
    assert all(row['status'] == VERIFIED for row in closure_bounds(base, base, below))
    above = identity()
    above[0][3] = I(11e-5)
    rows = closure_bounds(above, above, base)
    assert rows[0]['status'] == VERIFIED
    assert rows[1]['status'] == rows[2]['status'] == REJECTED
    for angle, expected in ((.0009, VERIFIED), (.0011, REJECTED)):
        rotated = rz(I(angle))
        rows = closure_bounds(rotated, rotated, base)
        assert rows[0]['status'] == VERIFIED
        assert rows[1]['status'] == rows[2]['status'] == expected


def _decimal_binary_float(value):
    numerator, denominator = F.from_float(float(value)).as_integer_ratio()
    return Decimal(numerator)/Decimal(denominator)


def _decimal_sine_cosine(value):
    # This independent 100-digit oracle uses 120 series terms and Decimal;
    # the production interval implementation uses 32 dyadic terms.
    sine_term, cosine_term = value, Decimal(1)
    sine, cosine = sine_term, cosine_term
    for index in range(1, 120):
        sine_term = -sine_term*value*value/Decimal(2*index*(2*index+1))
        cosine_term = -cosine_term*value*value/Decimal((2*index-1)*2*index)
        sine += sine_term
        cosine += cosine_term
    return sine, cosine


def _decimal_identity():
    return [[Decimal(int(r == c)) for c in range(4)] for r in range(4)]


def _decimal_matmul(left, right):
    return [[sum((left[r][k]*right[k][c] for k in range(4)), Decimal(0))
             for c in range(4)] for r in range(4)]


def _decimal_rotation(axis, angle):
    sine, cosine = _decimal_sine_cosine(angle)
    result = _decimal_identity()
    first, second = ((1, 2), (2, 0), (0, 1))[axis]
    result[first][first] = result[second][second] = cosine
    result[first][second], result[second][first] = -sine, sine
    return result


def _decimal_origin(element):
    if element is None:
        return _decimal_identity()
    angles = [_decimal_binary_float(value) for value in element.get('rpy', '0 0 0').split()]
    result = _decimal_identity()
    # Compose individual rotations, independently of the production expanded
    # RPY formula.  Column-vector convention is Rz @ Ry @ Rx.
    for axis in (2, 1, 0):
        result = _decimal_matmul(result, _decimal_rotation(axis, angles[axis]))
    for axis, value in enumerate(element.get('xyz', '0 0 0').split()):
        result[axis][3] = _decimal_binary_float(value)
    return result


def _decimal_urdf_tcp(urdf_path, side, q):
    root = ET.parse(urdf_path).getroot()
    by_child = {node.find('child').get('link'): node for node in root.findall('joint')}
    chain, link = [], side+'_fr3_side_suction_tcp'
    while link != 'world':
        joint = by_child[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    values = {f'{side}_fr3_joint{k+1}': _decimal_binary_float(value) for k, value in enumerate(q)}
    transform = _decimal_identity()
    for joint in reversed(chain):
        transform = _decimal_matmul(transform, _decimal_origin(joint.find('origin')))
        if joint.get('type') == 'revolute':
            assert joint.find('axis').get('xyz') == '0 0 1'
            transform = _decimal_matmul(transform, _decimal_rotation(2, values[joint.get('name')]))
    return transform


def test_frozen_interval_fk_encloses_independent_high_precision_urdf_chain():
    checker = SampledEdgeValidator()
    model = ProofModel(checker)
    left = tuple(float(value) for value in checker.model.home)
    right = tuple(float(value)/2 for value in checker.model.home)
    frames = model.forward(list(map(I, left)), list(map(I, right)))
    urdf_path = checker.contact.collision.reference.description/'urdf/task28_single_cube_frozen.urdf'
    with localcontext() as context:
        context.prec = 100
        for side, q in (('left', left), ('right', right)):
            reference = _decimal_urdf_tcp(urdf_path, side, q)
            actual = frames[side+'_fr3_side_suction_tcp']
            for r in range(4):
                for c in range(4):
                    expected = F(reference[r][c])
                    assert actual[r][c].contains(expected)
                    assert actual[r][c].width < F(1, 10**17)
