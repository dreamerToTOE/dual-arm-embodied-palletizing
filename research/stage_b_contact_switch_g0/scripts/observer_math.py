"""Small pure helpers for G0 evidence semantics."""
from __future__ import annotations
import math

def finite_vector(values, size=3):
    vals = [float(v) for v in values]
    if len(vals) != size or not all(math.isfinite(v) for v in vals):
        raise ValueError('nonfinite_or_wrong_length')
    return vals

def impulse_to_force(impulse, dt):
    vals = finite_vector(impulse)
    dt = float(dt)
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('invalid_physics_dt')
    return [v / dt for v in vals]

def aligned_observation(step, time_s, dt, *, previous_step=None, previous_time=None):
    step = int(step); time_s = float(time_s); dt = float(dt)
    if step <= 0 or not all(math.isfinite(v) for v in (time_s, dt)) or dt <= 0:
        return False
    if previous_step is not None and step != previous_step + 1:
        return False
    if previous_time is not None and time_s <= previous_time:
        return False
    return True

def normal_component(force, normal):
    f, n = finite_vector(force), finite_vector(normal)
    norm = math.sqrt(sum(v*v for v in n))
    if abs(norm - 1.0) > 1e-3:
        raise ValueError('normal_not_unit')
    return sum(a*b for a,b in zip(f,n))
