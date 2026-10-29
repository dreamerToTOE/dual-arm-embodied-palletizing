import json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p5_qp_6_r8.controller import gains,reference,discrete_matrix,torque
from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate,Rejected
from offline.task28_p5_qp_6_r8.effort_adapter import R8ExclusiveEffortAdapter,Reject
from offline.task28_p5_qp_6_r8.evaluate import accepted_metrics
from offline.task28_p5_qp_6_r8.process_runner import competing
ROOT=Path(__file__).resolve().parents[1]

def original_state():
 f=Path('offline/task28_p5_qp_6_r7/evidence/diagnostic/S3/frames.jsonl')
 for line in f.open():
  x=json.loads(line)
  if x['arms']['left']['position_drives_disabled']:
   for a in x['arms'].values():a['within_original_joint_limits']=True
   return x['arms']

def test_coupled_model_discrete_stable_and_full_gain():
 m=np.array(original_state()['left']['M'])[:7,:7];K,D=gains(m,'frozen_mass_hold')
 assert abs(K[1,3])>.1
 assert np.max(abs(np.linalg.eigvals(discrete_matrix(m,K,D))))<.95
 assert np.allclose(np.linalg.solve(m,K),16*np.eye(7))

def test_transition_initial_position_velocity_are_consistent():
 q=np.arange(7)/10;v=np.arange(7)/100
 a,b=reference(q,v,0,'frozen_mass_smooth');assert np.array_equal(a,q);assert np.array_equal(b,v)
 h=1e-6;plus=reference(q,v,h,'frozen_mass_smooth')[0]
 assert np.allclose((plus-a)/h,v,atol=1e-7)

def test_transition_returns_to_same_reference_c2():
 q=np.ones(7);v=np.ones(7)*.01;h=1e-6
 end=reference(q,v,.15,'frozen_mass_smooth');near=reference(q,v,.15-h,'frozen_mass_smooth')
 assert np.array_equal(end[0],q) and np.array_equal(end[1],np.zeros(7))
 assert np.max(abs(near[1]/h))<1e-4
 assert np.max(abs((reference(q,v,h,'frozen_mass_smooth')[1]-v)/h))<1e-4

def test_smooth_initial_feedback_zero_native_dq_preserved():
 a=original_state()['left'];q=np.array(a['q'][:7]);v=np.array(a['dq'][:7]);qr,vr=reference(q,v,0,'frozen_mass_smooth');u,c=torque(a,qr,vr,*gains(a['M'],'frozen_mass_smooth'))
 assert np.array_equal(u,np.array(a['g'][:7]));assert not np.allclose(v,0)

def test_source_failure_latches_and_rejects_later_good_state():
 a=original_state();g=R8DiagnosticGate();a['left']['within_original_joint_limits']=False
 with pytest.raises(Rejected,match='ORIGINAL_JOINT_LIMIT'):g.validate_pair(a)
 with pytest.raises(Rejected,match='LATCHED'):g.validate_pair(original_state())

def test_wrong_tcp_and_competing_sink_rejected():
 for field,value in [('tcp_fk_error_m',.001),('competing_sink',True),('position_drives_disabled',False)]:
  a=original_state();a['left'][field]=value
  with pytest.raises(Rejected):R8DiagnosticGate().validate_pair(a)

def test_non_spd_mass_is_rejected_before_gain_creation():
 m=np.eye(9);m[1,1]=-1
 with pytest.raises(ValueError,match='NON_POSITIVE'):gains(m,'frozen_mass_hold')

def test_performance_checks_peak_and_empty_not_pass():
 a={'tcp_peak_translation_m':.0011,'tcp_peak_rotation_rad':0.,'joint_peak_drift_rad':0.,'native_dq_peak_rad_s':0.,'tcp_final_translation_m':0.}
 assert not accepted_metrics({'left':a,'right':a});assert not accepted_metrics({})
 a['tcp_peak_translation_m']=.001;assert accepted_metrics({'left':a,'right':a})

def test_source_cap_cannot_expand():
 with pytest.raises(Reject,match='RESEARCH_LIMIT_EXCEEDS_SOURCE'):
  R8ExclusiveEffortAdapter({},None,None,None,None,np.array([35,12,8,35,13,4,4]),np.ones(7))

def test_R7_replay_is_red_for_exact_1mm_symptom():
 r=json.loads((ROOT/'evidence/r7_drift_analysis.json').read_text())
 for c in r['cells'].values():
  assert all(s['tcp_peak_translation_m']>.001 for s in c['sides'].values())
  assert all(s['command_reconstruction_max_error_Nm']<1e-12 for s in c['sides'].values())

def test_process_filter_finds_live_worker_and_ignores_zombie():
 rows=[{'pid':999999,'state':'S','argv':'/x/isaacsim-4.5.0/kit/python/bin/python3 -m task'}, {'pid':999998,'state':'Z','argv':'ros2 run test'}, {'pid':999997,'state':'S','argv':'ordinary-python'}]
 assert [r['pid'] for r in competing(rows)]==[999999]

def test_preregistration_preserves_caps_and_four_cells():
 p=json.loads((ROOT/'preregistration.json').read_text())
 assert p['max_development_cells']==4
 assert p['limits']['research_effort_abs_max_Nm']==[35,12,8,35,4,4,4]
 assert p['acceptance']['all_samples_tcp_translation_m']==.001

def test_moving_reference_c2_at_motion_boundaries():
 from offline.task28_p5_qp_6_r8.motion import moving_reference,DELTA
 q=np.zeros(7);v=np.ones(7)*.01;h=1e-6
 for t,target in ((1.,q),(3.,q+DELTA)):
  a,b=moving_reference(q,v,t);assert np.allclose(a,target) and np.max(abs(b))<1e-12
  for sign in (-1,1):assert np.max(abs(moving_reference(q,v,t+sign*h)[1]/h))<1e-6
 assert np.allclose(moving_reference(q,v,3.5)[0],q+DELTA)

def test_object_center_jacobian_transport_matches_frozen_fk_difference():
 from offline.task28_g1_1.task28_validator.model import RobotModel
 from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps
 from offline.task28_g1_1.task28_validator.se3 import inverse
 from offline.task28_p5_qp_6_r8.cooperative_readiness import skew
 model=RobotModel();grasp=nominal_grasps();h=1e-6
 for side in ('left','right'):
  q=np.array(original_state()[side]['q'][:7]);T=model.fk(side,q);G=getattr(grasp,side);O=T@inverse(G);J=model.jacobian(side,q);linear=J[:3]-skew(O[:3,3]-T[:3,3])@J[3:]
  for k in range(7):
   step=np.zeros(7);step[k]=h
   fd=((model.fk(side,q+step)@inverse(G))[:3,3]-(model.fk(side,q-step)@inverse(G))[:3,3])/(2*h)
   assert np.allclose(linear[:,k],fd,atol=1e-8,rtol=0)

def test_free_home_cannot_be_promoted_to_frozen_grasp_closure():
 from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
 a=original_state();T=np.eye(4)
 closure=evaluate_closure(T,np.array(a['left']['tcp_world']),np.array(a['right']['tcp_world']))
 assert not closure.valid and closure.translation_error_m>.0001
