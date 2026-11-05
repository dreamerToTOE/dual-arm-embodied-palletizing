"""Pure G0-R momentum-ledger helpers."""
from __future__ import annotations
import math
import numpy as np

def _vec(v):
    a=np.asarray(v,dtype=float)
    if a.shape != (3,) or not np.isfinite(a).all(): raise ValueError('invalid_vector')
    return a

def momentum_ledger(mass, gravity, dt, before, after, contact_impulse, friction_impulse, applied_force):
    vals=[mass,dt]
    if not all(math.isfinite(float(x)) for x in vals) or mass <= 0 or dt <= 0: raise ValueError('invalid_mass_or_dt')
    g=_vec(gravity); b=_vec(before); a=_vec(after); jc=_vec(contact_impulse); jf=_vec(friction_impulse); fa=_vec(applied_force)
    dv=float(mass)*(a-b); jg=float(mass)*g*float(dt); ja=fa*float(dt)
    known=jc+jf+jg+ja
    residual=dv-known
    return {'momentum_change_Ns':dv.tolist(),'gravity_impulse_Ns':jg.tolist(),'contact_normal_impulse_Ns':jc.tolist(),'friction_impulse_Ns':jf.tolist(),'applied_force_world_N':fa.tolist(),'applied_force_impulse_Ns':ja.tolist(),'known_impulse_sum_Ns':known.tolist(),'unexplained_impulse_Ns':residual.tolist()}

def force_from_impulse(impulse, dt):
    i=_vec(impulse); dt=float(dt)
    if not math.isfinite(dt) or dt <= 0: raise ValueError('invalid_dt')
    return (i/dt).tolist()
