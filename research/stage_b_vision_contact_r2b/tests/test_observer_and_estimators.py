import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from observer.contract import *
from estimator.contact_estimators import *
from estimator.stage_b_contact import ContactEstimate, ContactState, E0ContactEstimator, E1TemporalContactEstimator
from observer.stage_b_adapter import causal_payload, frame_from_records
from observer.runtime_adapter import frame_from_synchronized_records, latest_past_runtime, runtime_receipt_from_trace
from shadow.mode_selector import ShadowMode, ShadowModeSelector

def frame(step):
 return StageBObserverFrame(step,step/60,1/60,{'q':SourceStamp('sim',step/60,0,'articulation',Health.PRESENT)},
  {'left':(0.,)*7,'right':(0.,)*7},{'left':(0.,)*7,'right':(0.,)*7},
  {'left':(0.,0.,0.),'right':(0.,0.,0.)},{'owner':'test'},None,None,None,
  VisualPose((0.,0.,.26),(1.,0.,0.,0.),.9,1.,.01,23),{'left':True,'right':True},{'left':True,'right':True},
  ({'colliders':['Cube','Table']},),{'visual':Health.PRESENT},{'visual':.01})

def test_contract_separates_oracle():
 f=frame(1); d=f.to_dict(); p=f.estimator_payload(); assert 'oracle_contact_pairs' not in p; assert 'oracle_contact_pairs' in d

def test_buffer_owner_and_monotonicity():
 b=StageBFrameBuffer(); b.append(frame(1),'runner')
 try: b.append(frame(1),'runner'); assert False
 except ValueError: pass
 try: b.append(frame(2),'other'); assert False
 except RuntimeError: pass

def test_multilabel_and_e2_occlusion():
 e0=E0MultiLabel(); f={'physics_step':1,'time_s':.1,'cube_position_m':(0,0,.26),'visual_health':'PRESENT','visual_confidence':.9,'left_tool_distance_m':.05,'right_tool_distance_m':.3,'table_gap_m':.0,'deep_wall_gap_m':.5,'cube_velocity_x_m_s':.02,'suction_closed_count':2,'commanded_advancement_m':None}
 r=e0.estimate(f); assert r.edges[Edge.LEFT_TOOL_CUBE]>.5 and r.edges[Edge.RIGHT_TOOL_CUBE]<.5 and r.mode is Mode.DUAL_ATTACHED
 e2=E2PhysicsConstrained(); r=e2.estimate(f); assert r.causes==('bayesian_update',)
 f['physics_step']=2; f['visual_health']='UNKNOWN'; f['visual_confidence']=0.; r=e2.estimate(f); assert r.causes==('prediction_only',) and r.uncertainty>.01

def test_causal_adapter_excludes_oracle_channel():
 f=frame_from_records({'physics_step':1,'time_s':.1,'status':'PRESENT','confidence':.9,'estimated_cube_world_m':[.5,0,.26],'marker_id':23}, runtime_record={'oracle_contact_pairs':[{'pair':'Cube/Table'}], 'attachment_confidence':{'left':.8}})
 assert 'oracle_contact_pairs' not in causal_payload(f)
 assert f.to_dict()['oracle_contact_pairs'] == [{'pair':'Cube/Table'}]
 assert f.estimator_payload()['attachment_confidence']['left'] == .8

def test_new_e0_distinguishes_possible_from_confirmed_contact():
 e=E0ContactEstimator()
 r=e.estimate({'physics_step':1,'time_s':.1,'cube_position_m':(.80,0,.26),'visual_available':True,'visual_confidence':.95,'tcp_world_m':{'left':(.74,0,.26),'right':(.90,0,.26)},'commanded_advancement_m':.01,'observed_cube_velocity_x_m_s':.02})
 assert r.state is ContactState.PUSH_PROGRESS
 assert r.contact_possible and not r.contact_confirmed_from_available_evidence

def test_missing_tcp_is_unknown_not_free():
 e=E0ContactEstimator()
 r=e.estimate({'physics_step':1,'time_s':.1,'cube_position_m':(.80,0,.26),'visual_available':True,'visual_confidence':.95,'tcp_world_m':{'left':None,'right':None}})
 assert r.state is ContactState.UNKNOWN
 assert 'tcp_missing_or_stale' in r.causes

def test_e1_dropout_prediction_then_unknown_and_shadow_rejection():
 e=E1TemporalContactEstimator(dwell_frames=2,max_prediction_s=.20)
 first={'physics_step':1,'time_s':.0,'cube_position_m':(.80,0,.26),'visual_available':True,'visual_confidence':.95,'tcp_world_m':{'left':(.74,0,.26),'right':(.90,0,.26)},'commanded_advancement_m':.01}
 assert e.estimate(first).state is ContactState.UNKNOWN
 pending=e.estimate({**first,'physics_step':2,'time_s':.1,'cube_position_m':(.802,0,.26),'observed_cube_velocity_x_m_s':.02})
 assert pending.causes and 'candidate_pending' in pending.causes
 predicted=e.estimate({'physics_step':3,'time_s':.2,'visual_available':False,'visual_confidence':0.0,'tcp_world_m':{'left':(.74,0,.26),'right':(.90,0,.26)},'commanded_advancement_m':.01})
 assert 'short_horizon_causal_prediction' in predicted.causes
 stale=e.estimate({'physics_step':4,'time_s':.6,'visual_available':False,'visual_confidence':0.0,'tcp_world_m':{'left':(.74,0,.26),'right':(.90,0,.26)},'commanded_advancement_m':.01})
 decision=ShadowModeSelector().select(stale)
 assert decision.mode is ShadowMode.HOLD_UNKNOWN

def test_shadow_side_mode_requires_preflight():
 estimate=ContactEstimate(1,.1,ContactState.SIDE_CONTACT_POSSIBLE,{},True,False,.8,.2)
 selector=ShadowModeSelector()
 context={'skill_id':'auxiliary_side_compaction','controller_owner':'Task26BatchedFeedBridge','sensor_health':{'camera':'PRESENT','q':'PRESENT','dq':'PRESENT','tcp':'PRESENT'}}
 assert selector.select(estimate).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, context).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, {**context,'side_preflight_passed':True}).mode is ShadowMode.SIDE_APPROACH_READY

def _push_context(skill='single_arm_insertion', attachment=True):
 return {'skill_id':skill,'controller_owner':'Task26BatchedFeedBridge','sensor_health':{'camera':'PRESENT','q':'PRESENT','dq':'PRESENT','tcp':'PRESENT','command':'PRESENT'},'attachment_confidence':{'left':.9,'right':.9} if attachment else {'left':None,'right':None}}

def test_shadow_push_is_phase_and_owner_gated():
 estimate=ContactEstimate(1,.1,ContactState.SINGLE_ARM_PUSHING,{},True,False,.8,.2)
 selector=ShadowModeSelector()
 assert selector.select(estimate, _push_context('dual_arm_approach')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context('dual_arm_attachment')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context('coordinated_transport')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context('placement_release')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context('release_retreat')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context('single_arm_insertion')).mode is ShadowMode.CONTINUE_PUSH
 assert selector.select(estimate, _push_context('auxiliary_side_compaction')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, {**_push_context(), 'controller_owner':None}).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, _push_context(attachment=False)).mode is ShadowMode.HOLD_UNKNOWN

def test_shadow_deep_wall_and_side_require_phase_receipts():
 selector=ShadowModeSelector()
 wall=ContactEstimate(1,.1,ContactState.DEEP_WALL_CONTACT_POSSIBLE,{},True,False,.8,.2)
 assert selector.select(wall, _push_context('dual_arm_approach')).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(wall, _push_context('single_arm_insertion')).mode is ShadowMode.REQUEST_DEEP_WALL_CHECK
 assert selector.select(wall, _push_context('deep_wall_arrival')).mode is ShadowMode.REQUEST_DEEP_WALL_CHECK
 side=ContactEstimate(1,.1,ContactState.SIDE_CONTACT_POSSIBLE,{},True,False,.8,.2)
 assert selector.select(side, {**_push_context('auxiliary_side_compaction'), 'side_preflight_passed':False}).mode is ShadowMode.HOLD_UNKNOWN

def test_shadow_rejects_contradictory_or_stale_receipts():
 estimate=ContactEstimate(1,.1,ContactState.PUSH_PROGRESS,{},True,False,.8,.2)
 selector=ShadowModeSelector()
 context={**_push_context(), 'current_skill':'dual_arm_approach'}
 assert selector.select(estimate, context).mode is ShadowMode.HOLD_UNKNOWN
 assert selector.select(estimate, {**_push_context(), 'sensor_health':{'camera':'PRESENT','q':'STALE','dq':'PRESENT','tcp':'PRESENT','command':'PRESENT'}}).mode is ShadowMode.HOLD_UNKNOWN

def test_runtime_join_never_selects_future_and_preserves_age():
 visual={'physics_step':10,'time_s':1.0,'status':'PRESENT','confidence':.9,'estimated_cube_world_m':[.5,0,.26]}
 records=[{'physics_step':9,'time_s':.9,'q_actual_rad':{'left':(0.,)*7,'right':(0.,)*7},'dq_actual_rad_s':{'left':(0.,)*7,'right':(0.,)*7},'tcp_world_m':{'left':(.4,0,.26),'right':(.6,0,.26)},'suction_command':{'left':False,'right':False},'suction_state':{'left':False,'right':False}}, {'physics_step':11,'time_s':1.1}]
 assert latest_past_runtime(visual, records)['physics_step'] == 9
 f=frame_from_synchronized_records(visual, records)
 assert abs(f.source_stamps['q'].age_s - .1) < 1e-9
 assert f.sensor_health['q'] is Health.PRESENT

def test_missing_runtime_fields_keep_unknown_health():
 visual={'physics_step':1,'time_s':1.0,'status':'PRESENT','confidence':.9,'estimated_cube_world_m':[.5,0,.26]}
 receipt=runtime_receipt_from_trace(visual, {'physics_step':1,'time_s':1.0})
 assert receipt['source_stamps']['q']['health'] == 'UNKNOWN'
 assert receipt['source_stamps']['command']['health'] == 'UNKNOWN'
