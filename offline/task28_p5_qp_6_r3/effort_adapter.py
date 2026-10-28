"""Research-only bilateral Isaac effort sink with explicit prerequisite gates.

No activation happens at import or construction. A source certificate and a
caller-owned gate/journal are required. This adapter has not been physically
validated in R3: the registered original-scene source never qualified.
"""
from dataclasses import dataclass
import builtins
import json
import hashlib
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r2.common import DOF_NAMES,LEFT_ROOT,RIGHT_ROOT,FR3_SHA256


class Reject(RuntimeError):
    pass


@dataclass(frozen=True)
class SourceCertificate:
    status: str
    asset_sha256: str
    dt_s: float
    runtime_scene_identity: str
    finger_policy: str
    evidence_sha256: str
    evidence_path: str

    def require_valid(self):
        if (self.status!='ARM_SUBSYSTEM_SOURCE_VALIDATED' or self.asset_sha256!=FR3_SHA256
                or abs(self.dt_s-1/60)>1e-8 or self.runtime_scene_identity!='task28_g1_0_new_single_cube_v0'
                or self.finger_policy!='PRESERVE_NATIVE_DRIVES_AND_APPLIED_EFFORT'
                or len(self.evidence_sha256)!=64 or any(c not in '0123456789abcdef' for c in self.evidence_sha256)):
            raise Reject('UNQUALIFIED_SOURCE_CERTIFICATE')
        try:
            data=Path(self.evidence_path).read_bytes()
            evidence=json.loads(data)
        except (OSError,ValueError,TypeError):
            raise Reject('SOURCE_EVIDENCE_MISSING_OR_INVALID')
        required=('state_freshness','arm_residual','finger_coupling','negative_control','no_unexpected_contact','scene_identity','physics_dt')
        if (hashlib.sha256(data).hexdigest()!=self.evidence_sha256
                or evidence.get('status')!=self.status
                or evidence.get('asset_sha256')!=self.asset_sha256
                or evidence.get('finger_policy')!=self.finger_policy
                or any(evidence.get('checks',{}).get(key)!='PASS' for key in required)):
            raise Reject('SOURCE_EVIDENCE_MISMATCH')


def vec(v,n,label):
    x=np.asarray(v,dtype=float)
    if x.shape!=(n,) or not np.isfinite(x).all(): raise Reject('INVALID_'+label)
    return x


def gravity_pd(state,target,kp,kd):
    """Return candidate seven-arm efforts; caller must pass qualified state."""
    q=vec(state['q'],9,'Q');dq=vec(state['dq'],9,'DQ');g=vec(state['g'],9,'G')
    return g[:7]+vec(kp,7,'KP')*(vec(target,7,'TARGET')-q[:7])-vec(kd,7,'KD')*dq[:7]


class ExclusiveEffortAdapter:
    """Own both arm sinks, preserving existing native finger drives and inputs.

    ``gate`` must independently check source, scene, actual bilateral state,
    FREE-space collision and bounds. ``journal`` persists records before writes.
    No simulation step is performed by this class. Caller must abort the isolated
    simulation if any exception occurs after ownership changes or a partial write.
    """
    def __init__(self, arms, stage, certificate, gate, journal, limits, rates):
        self.arms=arms;self.stage=stage;self.certificate=certificate;self.gate=gate;self.journal=journal
        self.limits=vec(limits,7,'LIMITS');self.rates=vec(rates,7,'RATES')
        if np.any(self.limits<=0) or np.any(self.rates<=0):raise Reject('INVALID_LIMITS')
        self.enabled=False;self.pending=False;self.finger_gains={};self.pre_cursor=None

    def _identity(self):
        self.certificate.require_valid()
        if set(self.arms)!={'left','right'}:raise Reject('PAIR_REQUIRED')
        if getattr(builtins,'_task28_g1_0_single_cube_bridge',None) is not None:raise Reject('LEGACY_BRIDGE_PRESENT')
        root=self.stage.GetPrimAtPath('/World/Task28G10SingleCube')
        if (not root.IsValid() or root.GetCustomDataByKey('task28_g1_0_fixture_id')!='task28_g1_0_new_single_cube_v0'
                or root.GetCustomDataByKey('task28_g1_0_fr3_usd_sha256')!=FR3_SHA256
                or root.GetCustomDataByKey('task28_g1_0_active_cube_count')!=1
                or not self.stage.GetPrimAtPath('/World/Task28G10SingleCube/Supply/Cube_01').IsValid()):
            raise Reject('SCENE_IDENTITY')
        graph=self.stage.GetPrimAtPath('/World/Task28G10SingleCube/ROSGraph')
        if not graph.IsValid() or graph.IsActive():raise Reject('ROS_GRAPH_MISSING_OR_ACTIVE')
        for side,path in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]:
            a=self.arms[side]
            if str(a.prim_path)!=path or list(a.dof_names)!=DOF_NAMES or not a.handles_initialized:raise Reject('ARTICULATION_IDENTITY:'+side)

    def _check_pair(self,states):
        if set(states)!={'left','right'}:raise Reject('STATE_PAIR')
        a,b=states['left'],states['right']
        if a['physics_step']!=b['physics_step'] or a['time_s']!=b['time_s']:raise Reject('STATE_SYNCHRONIZATION')
        for side in ['left','right']:
            s=states[side];vec(s['q'],9,'Q');vec(s['dq'],9,'DQ');vec(s['g'],9,'G');vec(s['c'],9,'C')
            mass=np.asarray(s['M'],dtype=float)
            if mass.shape!=(9,9) or not np.isfinite(mass).all() or not np.allclose(mass,mass.T,atol=1e-5,rtol=0) or np.linalg.eigvalsh((mass+mass.T)/2).min()<=0:
                raise Reject('STATE_MASS_MATRIX')
            if not np.isfinite(float(s['time_s'])) or type(s['physics_step']) is not int or s['physics_step']<0:raise Reject('STATE_TIME')
        verdict=self.gate(states)
        if not isinstance(verdict,dict) or verdict.get('valid') is not True:raise Reject('PRE_COMMAND_GATE')
        return verdict

    def activate(self,states):
        if self.enabled:raise Reject('ALREADY_ACTIVE')
        self._identity();verdict=self._check_pair(states)
        before={}
        for side in ['left','right']:
            view=self.arms[side]._articulation_view; modes=view.get_effort_modes()[0];kp,kd=view.get_gains();kp=np.asarray(kp).reshape(-1);kd=np.asarray(kd).reshape(-1)
            if len(modes)!=9 or modes[:7]!=['force']*7 or kp.shape!=(9,) or kd.shape!=(9,):raise Reject('DRIVE_MODE_OR_GAINS')
            before[side]={'kp':kp.copy(),'kd':kd.copy(),'modes':list(modes)}
        self.journal({'event':'OWNERSHIP_PREPARED','states':states,'gate':verdict,'before':before})
        # No PhysX step is permitted between left/right changes. Finger gains and
        # mode are left exactly as the original scene provided them.
        try:
            for side in ['left','right']:
                view=self.arms[side]._articulation_view
                view.set_gains(kps=np.zeros(7),kds=np.zeros(7),joint_indices=np.arange(7,dtype=np.int32),save_to_usd=False)
            for side in ['left','right']:
                view=self.arms[side]._articulation_view; kp,kd=map(lambda x:np.asarray(x).reshape(-1),view.get_gains());modes=view.get_effort_modes()[0]
                if np.any(kp[:7]!=0) or np.any(kd[:7]!=0) or modes[:7]!=['force']*7:raise Reject('ARM_DRIVE_NOT_EXCLUSIVE')
                if not np.array_equal(kp[7:],before[side]['kp'][7:]) or not np.array_equal(kd[7:],before[side]['kd'][7:]) or modes[7:]!=before[side]['modes'][7:]:raise Reject('FINGER_DRIVE_CHANGED')
                self.finger_gains[side]=(kp[7:].copy(),kd[7:].copy(),list(modes[7:]))
        except BaseException:
            self.journal({'event':'OWNERSHIP_TRANSITION_FAILED_ABORT_WITHOUT_STEPPING'});raise
        self.enabled=True;self.journal({'event':'OWNERSHIP_ACTIVE'})

    def apply_pair(self,states,arm_efforts,dt):
        if not self.enabled or self.pending:raise Reject('ADAPTER_NOT_READY')
        self._identity();verdict=self._check_pair(states)
        if abs(float(dt)-1/60)>1e-8:raise Reject('PHYSICS_DT')
        full={}; previous={}
        for side in ['left','right']:
            a=self.arms[side];v=a._articulation_view;kp,kd=map(lambda x:np.asarray(x).reshape(-1),v.get_gains());modes=v.get_effort_modes()[0]
            if np.any(kp[:7]!=0) or np.any(kd[:7]!=0) or modes[:7]!=['force']*7:raise Reject('ARM_DRIVE_NOT_EXCLUSIVE')
            fkp,fkd,fm=self.finger_gains[side]
            if not np.array_equal(kp[7:],fkp) or not np.array_equal(kd[7:],fkd) or modes[7:]!=fm:raise Reject('FINGER_DRIVE_CHANGED')
            prev=vec(a.get_applied_joint_efforts(),9,'APPLIED_COMMAND');u=vec(arm_efforts[side],7,'ARM_EFFORT')
            if np.any(np.abs(u)>self.limits) or np.any(np.abs(u-prev[:7])>self.rates*dt):raise Reject('EFFORT_OR_RATE')
            previous[side]=prev.copy();full[side]=np.r_[u,prev[7:]]
        # Indexed Isaac writes construct a zero buffer internally. We explicitly
        # send the full vector with the currently read-back two finger actuation
        # inputs preserved; this is not a measured finger reaction.
        self.journal({'event':'PRE_COMMAND','states':states,'gate':verdict,'previous_applied':previous,'full_effort':full})
        try:
            for side in ['left','right']:self.arms[side].set_joint_efforts(full[side],joint_indices=np.arange(9,dtype=np.int32))
        except BaseException:
            self.enabled=False;self.journal({'event':'PARTIAL_WRITE_ABORT_NO_STEP'});raise
        self.pending=True;self.pre_cursor=(states['left']['physics_step'],states['left']['time_s']);return full

    def post_step(self,states):
        if not self.pending:raise Reject('NO_PENDING_STEP')
        try:
            verdict=self._check_pair(states)
            step,t=self.pre_cursor
            if states['left']['physics_step']!=step+1 or abs(float(states['left']['time_s'])-float(t)-1/60)>1e-8:
                raise Reject('POST_STEP_CURSOR')
        except BaseException:
            self.enabled=False;self.journal({'event':'POST_STEP_GATE_FAILURE','states':states});raise
        self.journal({'event':'POST_STEP','states':states,'gate':verdict});self.pending=False
