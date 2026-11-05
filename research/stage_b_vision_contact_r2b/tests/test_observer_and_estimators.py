import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from observer.contract import *
from estimator.contact_estimators import *

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
