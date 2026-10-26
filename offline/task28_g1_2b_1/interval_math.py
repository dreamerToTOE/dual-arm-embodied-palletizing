"""Outward dyadic intervals for the new offline G1-2B-1 verifier.

Each endpoint is an integer divided by 2**80.  Finite binary floats are
interpreted as their exact rational values; no nearest-rounded float is used in
an arithmetic or trigonometric enclosure.  Every non-exact grid operation is
rounded outwards using integer floor/ceiling division.

Sine and cosine use 32 Maclaurin terms at a dyadic center.  The polynomial is
evaluated with the same outward interval operations.  Both polynomials are the
degree-63 Taylor polynomial (the cosine coefficient of degree 63 is zero), so
the Lagrange error is at most abs(center)**64 / 64!.  The input radius is then
added using the global derivative bound 1.  Centers outside [-16,16] return the
unconditional [-1,1] enclosure.  There is no libm trigonometry assumption.
"""
from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from functools import lru_cache
import math


SCALE = 1 << 80
TAYLOR_TERMS = 32
TAYLOR_REMAINDER_ORDER = 64
_FACTORIAL_64 = math.factorial(TAYLOR_REMAINDER_ORDER)


def _fraction(value) -> Fraction:
    if isinstance(value, bool):
        raise TypeError("BOOLEAN_IS_NOT_AN_INTERVAL_NUMBER")
    if isinstance(value, Fraction):
        return value
    if isinstance(value, int):
        return Fraction(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("NONFINITE_INTERVAL_INPUT")
        return Fraction.from_float(value)
    if isinstance(value, str):
        # Useful for exact frozen URDF decimal literals, including exponents.
        try:
            return Fraction(value)
        except (ValueError, ZeroDivisionError) as error:
            raise ValueError("INVALID_INTERVAL_LITERAL") from error
    if hasattr(value, "as_integer_ratio"):
        try:
            numerator, denominator = value.as_integer_ratio()
            return Fraction(numerator, denominator)
        except (ValueError, OverflowError, TypeError, ZeroDivisionError) as error:
            raise ValueError("INVALID_OR_NONFINITE_INTERVAL_INPUT") from error
    raise TypeError("EXPECTED_EXACT_REAL_INTERVAL_INPUT")


def _floor(value: Fraction) -> int:
    return value.numerator // value.denominator


def _ceil(value: Fraction) -> int:
    return -((-value.numerator) // value.denominator)


def _float_outward(value: Fraction, upper: bool) -> float:
    """Reporting conversion only; preserve enclosure even at subnormal scale."""
    try:
        result = float(value)
    except OverflowError:
        result = math.inf if value > 0 else -math.inf
    if result == math.inf:
        return result if upper else math.nextafter(result, -math.inf)
    if result == -math.inf:
        return math.nextafter(result, math.inf) if upper else result
    exact = Fraction.from_float(result)
    if upper and exact < value:
        return math.nextafter(result, math.inf)
    if not upper and exact > value:
        return math.nextafter(result, -math.inf)
    return result


@dataclass(frozen=True, init=False)
class I:
    """Closed interval; ``lo`` and ``hi`` expose exact Fraction endpoints."""

    _lower: int
    _upper: int

    def __init__(self, value=0):
        if isinstance(value, I):
            lower, upper = value._lower, value._upper
        else:
            scaled = _fraction(value) * SCALE
            lower, upper = _floor(scaled), _ceil(scaled)
        object.__setattr__(self, "_lower", lower)
        object.__setattr__(self, "_upper", upper)

    @classmethod
    def _scaled(cls, lower: int, upper: int) -> I:
        if lower > upper:
            raise ValueError("REVERSED_INTERVAL_BOUNDS")
        result = object.__new__(cls)
        object.__setattr__(result, "_lower", lower)
        object.__setattr__(result, "_upper", upper)
        return result

    @classmethod
    def from_bounds(cls, lower, upper) -> I:
        lower, upper = _fraction(lower), _fraction(upper)
        if lower > upper:
            raise ValueError("REVERSED_INTERVAL_BOUNDS")
        return cls._scaled(_floor(lower * SCALE), _ceil(upper * SCALE))

    @classmethod
    def hull(cls, *values) -> I:
        """Smallest grid interval enclosing all given numbers/intervals."""
        if not values:
            raise ValueError("EMPTY_INTERVAL_HULL")
        intervals = tuple(cls(value) for value in values)
        return cls._scaled(min(v._lower for v in intervals),
                           max(v._upper for v in intervals))

    @classmethod
    def maximum(cls, *values) -> I:
        """Enclose the pointwise maximum of the given intervals."""
        if not values:
            raise ValueError("EMPTY_INTERVAL_MAXIMUM")
        intervals = tuple(cls(value) for value in values)
        return cls._scaled(max(v._lower for v in intervals),
                           max(v._upper for v in intervals))

    @classmethod
    def minimum(cls, *values) -> I:
        """Enclose the pointwise minimum of the given intervals."""
        if not values:
            raise ValueError("EMPTY_INTERVAL_MINIMUM")
        intervals = tuple(cls(value) for value in values)
        return cls._scaled(min(v._lower for v in intervals),
                           min(v._upper for v in intervals))

    @property
    def lo(self) -> Fraction:
        return Fraction(self._lower, SCALE)

    @property
    def hi(self) -> Fraction:
        return Fraction(self._upper, SCALE)

    @property
    def mid(self) -> Fraction:
        return Fraction(self._lower + self._upper, 2 * SCALE)

    @property
    def radius(self) -> Fraction:
        return Fraction(self._upper - self._lower, 2 * SCALE)

    @property
    def width(self) -> Fraction:
        return Fraction(self._upper - self._lower, SCALE)

    @property
    def max_abs(self) -> Fraction:
        return Fraction(max(abs(self._lower), abs(self._upper)), SCALE)

    @property
    def min_abs(self) -> Fraction:
        if self._lower <= 0 <= self._upper:
            return Fraction(0)
        return Fraction(min(abs(self._lower), abs(self._upper)), SCALE)

    @property
    def lower_float(self) -> float:
        return _float_outward(self.lo, False)

    @property
    def upper_float(self) -> float:
        return _float_outward(self.hi, True)

    def contains(self, value) -> bool:
        if isinstance(value, I):
            return self._lower <= value._lower and value._upper <= self._upper
        number = _fraction(value)
        return self.lo <= number <= self.hi

    def __add__(self, other) -> I:
        other = I(other)
        return I._scaled(self._lower + other._lower,
                         self._upper + other._upper)

    __radd__ = __add__

    def __neg__(self) -> I:
        return I._scaled(-self._upper, -self._lower)

    def __sub__(self, other) -> I:
        return self + (-I(other))

    def __rsub__(self, other) -> I:
        return I(other) - self

    def __mul__(self, other) -> I:
        other = I(other)
        products = (self._lower * other._lower,
                    self._lower * other._upper,
                    self._upper * other._lower,
                    self._upper * other._upper)
        return I._scaled(min(products) // SCALE,
                         -((-max(products)) // SCALE))

    __rmul__ = __mul__

    def __truediv__(self, other) -> I:
        other = I(other)
        if other._lower <= 0 <= other._upper:
            raise ZeroDivisionError("INTERVAL_DIVISOR_CONTAINS_ZERO")
        if other._lower == other._upper:
            # Taylor denominators are exact positive integers.  This path
            # avoids constructing/reducing four equal rational quotients.
            denominator = other._lower
            lower, upper = self._lower * SCALE, self._upper * SCALE
            if denominator < 0:
                lower, upper = upper, lower
            return I._scaled(lower // denominator,
                             -((-upper) // denominator))
        quotients = (Fraction(self._lower, other._lower),
                     Fraction(self._lower, other._upper),
                     Fraction(self._upper, other._lower),
                     Fraction(self._upper, other._upper))
        return I.from_bounds(min(quotients), max(quotients))

    def __rtruediv__(self, other) -> I:
        return I(other) / self

    def __abs__(self) -> I:
        if self._lower >= 0:
            return self
        if self._upper <= 0:
            return -self
        return I._scaled(0, max(-self._lower, self._upper))

    def squared(self) -> I:
        if self._lower >= 0:
            lower, upper = self._lower ** 2, self._upper ** 2
        elif self._upper <= 0:
            lower, upper = self._upper ** 2, self._lower ** 2
        else:
            lower, upper = 0, max(self._lower ** 2, self._upper ** 2)
        return I._scaled(lower // SCALE, -((-upper) // SCALE))

    def __pow__(self, exponent: int) -> I:
        if type(exponent) is not int:
            raise TypeError("INTERVAL_POWER_REQUIRES_INTEGER")
        if exponent < 0:
            return I(1) / (self ** (-exponent))
        result, power = I(1), self
        while exponent:
            if exponent & 1:
                result = result * power
            exponent //= 2
            if exponent:
                power = power.squared()
        return result

    def _trig(self, cosine: bool) -> I:
        center_twice_scaled = self._lower + self._upper
        if abs(center_twice_scaled) > 32 * SCALE:
            return I._scaled(-SCALE, SCALE)
        sine, cosine_value = _taylor_center(center_twice_scaled)
        center_enclosure = cosine_value if cosine else sine
        # The exact center can lie between adjacent grid points; the Taylor
        # evaluation already encloses it.  This radius is for the input itself.
        lower = max(Fraction(-1), center_enclosure.lo - self.radius)
        upper = min(Fraction(1), center_enclosure.hi + self.radius)
        return I.from_bounds(lower, upper)

    def sin(self) -> I:
        return self._trig(False)

    def cos(self) -> I:
        return self._trig(True)


@lru_cache(maxsize=32768)
def _taylor_center(center_twice_scaled: int) -> tuple[I, I]:
    center = Fraction(center_twice_scaled, 2 * SCALE)
    x = I(center)
    square = x.squared()
    sine_term, cosine_term = x, I(1)
    sine, cosine = sine_term, cosine_term
    for index in range(1, TAYLOR_TERMS):
        sine_term = -(sine_term * square) / ((2 * index) * (2 * index + 1))
        cosine_term = -(cosine_term * square) / ((2 * index - 1) * (2 * index))
        sine = sine + sine_term
        cosine = cosine + cosine_term
    remainder = abs(center) ** TAYLOR_REMAINDER_ORDER / _FACTORIAL_64
    error = I.from_bounds(-remainder, remainder)
    return sine + error, cosine + error
