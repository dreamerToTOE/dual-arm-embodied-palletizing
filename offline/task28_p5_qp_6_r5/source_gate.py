"""R5 source/control gate and deterministic negative controls.

The gate acknowledges TGS end-step velocity semantics; it never replaces dq by
finite differences. A solver explanation does not bypass identity, timestamp,
collision, dynamics or actuator ownership checks.
"""
from __future__ import annotations
import numpy as np

class Rejected(RuntimeError):
    pass

class R5Gate:
    def __init__(self):
        self.latched=False; self.reason=None
    def reject(self,reason):
        self.latched=True;self.reason=reason;raise Rejected(reason)
    def validate(self,state,previous=None,identity='left',dt=1/60):
        if self.latched: raise Rejected('LATCHED:'+str(self.reason))
        if state.get('identity')!=identity:self.reject('WRONG_IDENTITY')
        if state.get('competing_sink'):self.reject('COMPETING_SINK')
        if state.get('unexpected_contact'):self.reject('UNEXPECTED_CONTACT')
        if state.get('effort_mode')!='force':self.reject('INVALID_EFFORT_MODE')
        if state.get('tcp_frame')!='original_side_suction_tcp':self.reject('WRONG_TCP')
        q=np.asarray(state.get('q',[]),float);dq=np.asarray(state.get('dq',[]),float)
        M=np.asarray(state.get('M',[]),float)
        if q.shape!=(9,) or dq.shape!=(9,) or not np.isfinite(q).all() or not np.isfinite(dq).all():self.reject('INVALID_Q_DQ')
        if M.shape!=(9,9) or not np.isfinite(M).all() or np.min(np.linalg.eigvalsh((M+M.T)/2))<=0:self.reject('INVALID_MASS')
        if previous is not None:
            if state.get('step')!=previous.get('step',-1)+1:self.reject('TIME_STEP_SYNC')
            if abs(float(state.get('time'))-float(previous.get('time'))-dt)>1e-8:self.reject('TIME_SYNC')
            if state.get('delayed_buffer'):self.reject('DELAYED_SOURCE')
        if state.get('stale_solver_state'):self.reject('STALE_SOLVER_STATE')
        if state.get('fresh_copy') is not True:self.reject('NOT_FRESH_COPY')
        if state.get('solver_semantics_ack') is not True:self.reject('SEMANTICS_UNDECLARED')
        return {'valid':True,'velocity_semantics':'END_STEP_SOLVER_VELOCITY','fd_role':'ESTIMATOR_ONLY'}
    def require(self):
        if self.latched:raise Rejected('LATCHED:'+str(self.reason))
        return True

def torque_attempt(gate):
    gate.require();return {'commands':0,'executed':False}
