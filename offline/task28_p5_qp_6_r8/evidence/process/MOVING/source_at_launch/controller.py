"""Preregistered frozen-mass impedance, never a certified inverse-dynamics model."""
import numpy as np

KP_R7 = np.array([1.2,1.2,1.,.8,.5,.4,.3])
KD_R7 = np.array([.18,.18,.16,.14,.10,.08,.06])
DT = 1/60
WN = 4.
ZETA = 1.
TRANSITION_S = .15

def gains(mass, strategy):
    m = np.asarray(mass, float)[:7,:7]
    if m.shape != (7,7) or not np.isfinite(m).all() or not np.allclose(m,m.T,atol=1e-5,rtol=0):
        raise ValueError('INVALID_MASS')
    m = (m+m.T)/2
    if np.linalg.eigvalsh(m).min() <= 0:
        raise ValueError('NON_POSITIVE_MASS')
    if strategy == 'r7_pd':
        return np.diag(KP_R7),np.diag(KD_R7)
    if strategy not in ('frozen_mass_hold','frozen_mass_smooth'):
        raise ValueError('UNKNOWN_STRATEGY')
    return WN**2*m,2*ZETA*WN*m

def reference(q0, dq0, t, strategy):
    q0,dq0=np.asarray(q0,float),np.asarray(dq0,float)
    if strategy != 'frozen_mass_smooth' or t >= TRANSITION_S:
        return q0.copy(),np.zeros(7)
    s=max(0.,t)/TRANSITION_S
    # C2 position reference returns to actual initial q; initial dq matches native dq.
    f=s-6*s**3+8*s**4-3*s**5
    df=1-18*s**2+32*s**3-15*s**4
    return q0+dq0*TRANSITION_S*f,dq0*df

def torque(state,qref,dqref,kp,kd):
    p=kp@(qref-np.asarray(state['q'][:7],float))
    d=kd@(dqref-np.asarray(state['dq'][:7],float))
    g=np.asarray(state['g'][:7],float)
    return g+p+d,{'gravity_Nm':g.tolist(),'P_Nm':p.tolist(),'D_Nm':d.tolist(),'feedback_Nm':(p+d).tolist(),'q_ref':qref.tolist(),'dq_ref':dqref.tolist()}

def discrete_matrix(m,kp,kd,dt=DT):
    """Semi-implicit rigid-body approximation; not a TGS equivalence claim."""
    a=np.linalg.solve(m,kp); b=np.linalg.solve(m,kd); eye=np.eye(len(m))
    return np.block([[eye-dt*dt*a,dt*(eye-dt*b)],[-dt*a,eye-dt*b]])
