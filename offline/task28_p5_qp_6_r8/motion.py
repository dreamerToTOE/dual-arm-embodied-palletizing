"""Conditional short C2 reference, frozen only after formal static PASS."""
import numpy as np
from offline.task28_p5_qp_6_r8.controller import reference
DELTA=np.array([0.,.002,0.,0.,0.,0.,0.])
START_S=1.
DURATION_S=2.
STEPS=210

def moving_reference(q0,dq0,t,strategy='frozen_mass_smooth'):
    if t<START_S:return reference(q0,dq0,t,strategy)
    s=min(1.,max(0.,(t-START_S)/DURATION_S))
    h=10*s**3-15*s**4+6*s**5
    dh=(30*s*s-60*s**3+30*s**4)/DURATION_S
    return np.asarray(q0)+DELTA*h,DELTA*dh
