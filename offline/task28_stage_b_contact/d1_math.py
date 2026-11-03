"""Momentum accounting for one observed physics step."""
import math

import numpy as np


def balance(mass, gravity, dt, before, after, normal, friction):
    values = [mass, dt, *gravity, *before, *after, *normal, *friction]
    if not all(math.isfinite(value) for value in values) or mass <= 0 or dt <= 0:
        raise ValueError('invalid physics sample')
    dv = mass * (np.asarray(after) - np.asarray(before))
    contact = np.asarray(normal) + np.asarray(friction)
    gravitational = mass * np.asarray(gravity) * dt
    inferred = dv - gravitational
    return {'momentum_change_Ns': dv.tolist(), 'gravity_impulse_Ns': gravitational.tolist(),
            'reported_contact_impulse_Ns': contact.tolist(), 'motion_inferred_contact_impulse_Ns': inferred.tolist(),
            'unexplained_impulse_Ns': (inferred - contact).tolist()}
