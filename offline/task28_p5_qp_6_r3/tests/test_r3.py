import hashlib,json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p5_qp_6_r2.common import DOF_NAMES,FR3_SHA256,LEFT_ROOT,RIGHT_ROOT
from offline.task28_p5_qp_6_r3.effort_adapter import Reject,SourceCertificate,ExclusiveEffortAdapter,gravity_pd
from offline.task28_p5_qp_6_r3.audit import audit

ROOT=Path(__file__).resolve().parents[1]

class Prim:
    def IsValid(self):return True
    def IsActive(self):return False
    def GetCustomDataByKey(self,key):
        return {'task28_g1_0_fixture_id':'task28_g1_0_new_single_cube_v0','task28_g1_0_fr3_usd_sha256':FR3_SHA256,'task28_g1_0_active_cube_count':1}.get(key)
class Stage:
    def GetPrimAtPath(self,path):return Prim()
class View:
    def __init__(self):self.kp=np.array([[10.]*7+[3.,4.]]);self.kd=np.array([[2.]*7+[5.,6.]]);self.mode=['force']*9
    def get_gains(self):return self.kp.copy(),self.kd.copy()
    def get_effort_modes(self):return [self.mode.copy()]
    def set_gains(self,kps,kds,joint_indices,save_to_usd):
        assert save_to_usd is False
        self.kp[0,joint_indices]=kps;self.kd[0,joint_indices]=kds
class Arm:
    def __init__(self,path):self.prim_path=path;self.dof_names=DOF_NAMES;self.handles_initialized=True;self._articulation_view=View();self.applied=np.array([0.]*7+[.25,-.5]);self.calls=[]
    def get_applied_joint_efforts(self):return self.applied.copy()
    def set_joint_efforts(self,efforts,joint_indices):self.applied=np.asarray(efforts).copy();self.calls.append((self.applied.copy(),np.asarray(joint_indices).copy()))

def states(t=0.,step=0):
    return {s:{'q':np.zeros(9),'dq':np.zeros(9),'g':np.zeros(9),'c':np.zeros(9),'M':np.eye(9),'time_s':t,'physics_step':step} for s in ['left','right']}

def cert(tmp):
    keys=('state_freshness','arm_residual','finger_coupling','negative_control','no_unexpected_contact','scene_identity','physics_dt')
    evidence={'status':'ARM_SUBSYSTEM_SOURCE_VALIDATED','asset_sha256':FR3_SHA256,'finger_policy':'PRESERVE_NATIVE_DRIVES_AND_APPLIED_EFFORT','checks':{k:'PASS' for k in keys}}
    p=tmp/'synthetic_unit_test_only.json';p.write_text(json.dumps(evidence));digest=hashlib.sha256(p.read_bytes()).hexdigest()
    return SourceCertificate(evidence['status'],FR3_SHA256,1/60,'task28_g1_0_new_single_cube_v0',evidence['finger_policy'],digest,str(p))

def create(tmp,gate=lambda s:{'valid':True}):
    arms={'left':Arm(LEFT_ROOT),'right':Arm(RIGHT_ROOT)}; events=[]
    ad=ExclusiveEffortAdapter(arms,Stage(),cert(tmp),gate,events.append,np.full(7,20),np.full(7,1000))
    return ad,arms,events

def test_default_no_writes_and_unqualified_evidence_rejected(tmp_path):
    ad,arms,_=create(tmp_path)
    with pytest.raises(Reject):ad.apply_pair(states(),{'left':np.zeros(7),'right':np.zeros(7)},1/60)
    ad.certificate=SourceCertificate('ARM_SUBSYSTEM_SOURCE_PARTIAL',FR3_SHA256,1/60,'task28_g1_0_new_single_cube_v0','PRESERVE_NATIVE_DRIVES_AND_APPLIED_EFFORT','0'*64,'missing')
    with pytest.raises(Reject,match='UNQUALIFIED_SOURCE'):ad.activate(states())
    assert all(not arm.calls for arm in arms.values())

def test_pair_actuation_preserves_native_finger_inputs_and_drives(tmp_path):
    ad,arms,events=create(tmp_path);ad.activate(states());full=ad.apply_pair(states(),{'left':np.ones(7),'right':np.ones(7)*2},1/60)
    assert [e['event'] for e in events]==['OWNERSHIP_PREPARED','OWNERSHIP_ACTIVE','PRE_COMMAND']
    for side in arms:
        assert np.array_equal(full[side][7:],[.25,-.5]);assert np.array_equal(arms[side]._articulation_view.kp[0,7:],[3,4]);assert len(arms[side].calls)==1
    ad.post_step(states(1/60,1));assert not ad.pending and events[-1]['event']=='POST_STEP'

def test_reject_pair_before_either_write_and_require_post_step(tmp_path):
    ad,arms,events=create(tmp_path);ad.activate(states())
    with pytest.raises(Reject,match='EFFORT_OR_RATE'):ad.apply_pair(states(),{'left':np.ones(7),'right':np.full(7,21)},1/60)
    assert all(not arm.calls for arm in arms.values())
    ad.apply_pair(states(),{'left':np.ones(7),'right':np.ones(7)},1/60)
    with pytest.raises(Reject,match='ADAPTER_NOT_READY'):ad.apply_pair(states(),{'left':np.ones(7),'right':np.ones(7)},1/60)
    with pytest.raises(Reject,match='POST_STEP_CURSOR'):ad.post_step(states(1/60,2))
    assert not ad.enabled

def test_gate_and_bad_matrix_reject_before_mode_change(tmp_path):
    ad,arms,_=create(tmp_path,lambda s:{'valid':False})
    with pytest.raises(Reject,match='PRE_COMMAND_GATE'):ad.activate(states())
    assert arms['left']._articulation_view.kp[0,0]==10
    ad,arms,_=create(tmp_path);bad=states();bad['right']['M'][0,0]=-1
    with pytest.raises(Reject,match='STATE_MASS_MATRIX'):ad.activate(bad)
    assert arms['left']._articulation_view.kp[0,0]==10

def test_malformed_evidence_and_actuator_source():
    x=SourceCertificate('ARM_SUBSYSTEM_SOURCE_PARTIAL',FR3_SHA256,1/60,'task28_g1_0_new_single_cube_v0','PRESERVE_NATIVE_DRIVES_AND_APPLIED_EFFORT','0'*64,'missing')
    with pytest.raises(Reject):x.require_valid()
    assert np.allclose(gravity_pd(states()['left'],np.zeros(7),np.ones(7),np.ones(7)),0)

def test_real_archived_development_failures_and_actual_rate():
    for run,reason in [('dev_01','NATIVE_DRIVE_SETTLE_TIMEOUT'),('dev_02','FROZEN_POSITION_NONZERO_VELOCITY_SOURCE_MISMATCH'),('dev_03','NATIVE_DRIVE_SETTLE_TIMEOUT')]:
        x=audit(ROOT/'evidence'/run)
        assert x['first_failure']['message']==reason and x['arm_subsystem_status']=='PARTIAL_NOT_QUALIFIED'
        assert max(abs(t-1/60) for t in x['callback_dt_unique_s'])<1e-8
        assert x['sides']['left']['position_velocity_inconsistency_inf_rad_s']>.01

def test_inertia_and_collision_diagnostic_not_grasp_pass():
    d=json.loads((ROOT/'evidence/dev_04/result.json').read_text());assert d['status']=='MOUNTED_INERTIA_READ_COMPLETED'
    for side in ['left','right']:
        t=d['tool_mass_inertia_inspection'][side];assert t['link8_effective_mass_kg']>0;assert t['tool_inertia_increment']=='UNKNOWN_FROM_SINGLE_CONFIGURATION'
        assert sum(x['collision_api'] for x in t['tool_prims'])==7
    a=json.loads((ROOT/'evidence/approach_audit.json').read_text())
    assert not a['validated_continuous_approach'] and a['straight_joint_interpolation']['first_unexpected_collision'] is not None
