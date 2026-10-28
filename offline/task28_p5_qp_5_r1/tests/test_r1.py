import json
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_4.reference import load_qp3_positive, build_reference_sequence
from offline.task28_p5_qp_5.controller import sample_reference
from offline.task28_p5_qp_5_r1.contract import sample, check_time_pair
from offline.task28_p5_qp_5_r1.runtime_gate import stop_after_first_failure
from offline.task28_p5_qp_5_r1.controls import negative_controls

def test_same_time_joint_and_object_contract():
 refs=build_reference_sequence(load_qp3_positive()); initial=np.asarray(load_qp3_positive()['raw_qp2']['request']['object_pose'])
 a=sample(refs,initial,1/120); b=sample(refs,initial,1/120)
 assert a.time_s==b.time_s and np.array_equal(a.q_ref_14,b.q_ref_14) and np.array_equal(a.object_pose_world,b.object_pose_world)
def test_post_step_time_requires_dt():
 check_time_pair(0.,1/120,1/120)
 try: check_time_pair(0.,0.02,1/120)
 except ValueError as e: assert str(e)=='TEMPORAL_STEP_MISMATCH'
 else: raise AssertionError('offset accepted')
def test_twenty_ms_is_not_three_steps_by_assumption():
 refs=build_reference_sequence(load_qp3_positive()); q1=sample_reference(refs,0.02)[0]; q3=sample_reference(refs,3/120)[0]
 assert not np.array_equal(q1,q3)
def test_fail_closed_stops_after_first_rejection():
 out=stop_after_first_failure([{'valid':True},{'valid':False},{'valid':True},{'valid':True}]); assert out=={'status':'REJECTED','executed_steps':2,'first_failure_index':1,'continued_after_failure':False}
def test_negative_controls():
 c=negative_controls(); assert len(c)==11 and all(x['passed'] for x in c)
def test_original_archived_forensic_numbers_are_preserved():
 p=Path('offline/task28_p5_qp_5_r1/evidence/forensic_replay/replay.json')
 if p.is_file():
  d=json.loads(p.read_text()); x=d['rows'][2]['closure_matched_object']; assert abs(x['left_absolute_translation_m']-0.00015331409253391698)<1e-12
