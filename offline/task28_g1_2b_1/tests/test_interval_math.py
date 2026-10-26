"""Independent exact-rational checks for the dyadic enclosure arithmetic."""
from decimal import Decimal, localcontext
from fractions import Fraction
import math
from pathlib import Path
import random
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from interval_math import I, SCALE


def test_constructor_outward_exact_float_fraction_and_decimal_literal():
    for value in (0, 1, -2, .1, -.1, Fraction(1, 3), Fraction(-1, 7), "1.23e-17"):
        interval = I(value)
        exact = Fraction.from_float(value) if isinstance(value, float) else Fraction(value)
        assert interval.lo <= exact <= interval.hi
        assert interval.width <= Fraction(1, SCALE)
    assert I(1).lo == I(1).hi == 1
    assert I(Fraction(1, SCALE)).width == 0
    assert I(I(Fraction(1, 3))) == I(Fraction(1, 3))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, Decimal("NaN"), Decimal("Infinity")])
def test_nonfinite_rejected(value):
    with pytest.raises(ValueError):
        I(value)


@pytest.mark.parametrize("value", [True, False, 1j, object()])
def test_nonreal_or_boolean_rejected(value):
    with pytest.raises(TypeError):
        I(value)


def test_subnormal_tiny_and_cancellation_are_not_lost():
    smallest = math.nextafter(0., 1.)
    for value in (smallest, -smallest, Fraction(1, 2 ** 10000), Fraction(-1, 2 ** 10000)):
        interval = I(value)
        assert interval.contains(value)
        assert interval.lo != interval.hi
    exact = Fraction(1, 2 ** 10000)
    expression = I(1 + exact) - I(1)
    assert expression.contains(exact)
    assert (I(10 ** 300) - I(10 ** 300)).lo == 0
    assert (I(10 ** 300) - I(10 ** 300)).hi == 0


def test_every_arithmetic_corner_enclosed_with_rational_oracle():
    rng = random.Random(2801)
    for _ in range(160):
        a, b = sorted(Fraction(rng.randint(-1000, 1000), rng.randint(1, 1000)) for _ in range(2))
        c, d = sorted(Fraction(rng.randint(-1000, 1000), rng.randint(1, 1000)) for _ in range(2))
        left, right = I.from_bounds(a, b), I.from_bounds(c, d)
        for x in (a, b, (a + b) / 2):
            for y in (c, d, (c + d) / 2):
                assert (left + right).contains(x + y)
                assert (left - right).contains(x - y)
                assert (left * right).contains(x * y)
                if not right.lo <= 0 <= right.hi:
                    assert (left / right).contains(x / y)


def test_negative_rounding_and_reflected_operators():
    negative = I(Fraction(-1, 3))
    assert negative.lo <= Fraction(-1, 3) <= negative.hi
    assert negative.lo * SCALE == -((SCALE + 2) // 3)
    assert (3 - negative).contains(Fraction(10, 3))
    assert (3 / negative).contains(-9)
    assert (negative / -2).contains(Fraction(1, 6))
    assert (negative * -2).contains(Fraction(2, 3))
    assert (sum([negative, negative, negative])).contains(-1)


@pytest.mark.parametrize("denominator", [I(0), I.from_bounds(-1, 1), I.from_bounds(0, 1), I.from_bounds(-1, 0)])
def test_zero_containing_divisor_rejected(denominator):
    with pytest.raises(ZeroDivisionError):
        I(1) / denominator


def test_square_abs_powers_and_hulls():
    crossing = I.from_bounds(-2, 3)
    assert abs(crossing).lo == 0 and abs(crossing).hi == 3
    assert crossing.squared().lo == 0 and crossing.squared().hi == 9
    assert I.from_bounds(-3, -2).squared() == I.from_bounds(4, 9)
    assert I.from_bounds(2, 3).squared() == I.from_bounds(4, 9)
    assert crossing ** 2 == crossing.squared()
    assert (I(2) ** -3).contains(Fraction(1, 8))
    assert I(0) ** 0 == I(1)
    assert crossing.max_abs == 3 and crossing.min_abs == 0
    assert I.hull(-5, crossing, 8) == I.from_bounds(-5, 8)
    assert I.maximum(crossing, 1) == I.from_bounds(1, 3)
    assert I.minimum(crossing, 1) == I.from_bounds(-2, 1)
    with pytest.raises(ValueError):
        I.from_bounds(1, -1)
    with pytest.raises(ValueError):
        I.hull()
    with pytest.raises(TypeError):
        I(2) ** .5


def _decimal_sine_cosine(value: Fraction):
    """Independent 110-digit series, with 120 terms instead of library trig."""
    with localcontext() as context:
        context.prec = 110
        x = Decimal(value.numerator) / Decimal(value.denominator)
        sine_term, cosine_term = x, Decimal(1)
        sine, cosine = sine_term, cosine_term
        for index in range(1, 120):
            sine_term = -sine_term * x * x / Decimal((2 * index) * (2 * index + 1))
            cosine_term = -cosine_term * x * x / Decimal((2 * index - 1) * (2 * index))
            sine += sine_term
            cosine += cosine_term
        return Fraction(sine), Fraction(cosine)


@pytest.mark.parametrize("value", [Fraction(0), Fraction(1, 7), Fraction(-1, 3), Fraction(1), Fraction(-3), Fraction(7), Fraction(-16), Fraction(16), Fraction(1, 2 ** 200)])
def test_trig_encloses_independent_high_precision_series(value):
    interval = I(value)
    sine, cosine = _decimal_sine_cosine(value)
    assert interval.sin().contains(sine)
    assert interval.cos().contains(cosine)
    assert interval.sin().lo >= -1 and interval.sin().hi <= 1
    assert interval.cos().lo >= -1 and interval.cos().hi <= 1


def test_trig_lipschitz_expansion_encloses_interval_samples():
    for lower, upper in ((Fraction(-4), Fraction(-3)), (Fraction(-1, 8), Fraction(1, 9)),
                         (Fraction(15), Fraction(16)), (Fraction(-20), Fraction(20))):
        interval = I.from_bounds(lower, upper)
        for numerator in range(9):
            point = lower + (upper - lower) * numerator / 8
            sine, cosine = _decimal_sine_cosine(point)
            assert interval.sin().contains(sine)
            assert interval.cos().contains(cosine)
    assert I(100).sin() == I.from_bounds(-1, 1)
    assert I(-100).cos() == I.from_bounds(-1, 1)


def test_known_trig_values_and_library_crosscheck_tolerance():
    assert I(0).sin() == I(0)
    assert I(0).cos() == I(1)
    # math.sin/cos are a diagnostic crosscheck, not the reference used in proof.
    # They can lie outside an interval tighter than one binary64 rounding unit.
    for value in (-6.9, -3.14, -.001, .2, 2.8, 6.7):
        sine, cosine = I(value).sin(), I(value).cos()
        assert abs(float(sine.mid) - math.sin(value)) <= 2e-15
        assert abs(float(cosine.mid) - math.cos(value)) <= 2e-15
        assert sine.width < Fraction(1, 10 ** 17)
        assert cosine.width < Fraction(1, 10 ** 17)


def test_float_reporting_remains_outward_and_does_not_affect_exact_bounds():
    for interval in (I(Fraction(1, 3)), I(Fraction(-1, 3)), I(Fraction(1, 2 ** 10000)),
                     I.from_bounds(Fraction(-1, 2 ** 10000), 0), I(10 ** 500), I(-(10 ** 500))):
        lower, upper = interval.lower_float, interval.upper_float
        assert lower == -math.inf or Fraction.from_float(lower) <= interval.lo
        assert upper == math.inf or Fraction.from_float(upper) >= interval.hi
