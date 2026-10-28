#!/usr/bin/env python3
"""Read-only forensic replay of the archived QP-5 moving frames."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
from offline.task28_g1_1.task28_validator.se3 import checked_transform
from offline.task28_p5_qp_4.reference import load_qp3_positive, build_reference_sequence
from offline.task28_p5_qp_5.controller import sample_reference, interpolate_pose
from offline.task28_p5_qp_5_r1.contract import sample
ROOT=Path(__file__).resolve().parents[2]
DT=1/120

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def arr(x,n):
 a=np.asarray(x,float); assert a.shape==(n,) and np.isfinite(a).all(); return a

def closure_dict(cl):
 return {'valid':cl.valid,'reasons':list(cl.reasons),'relative_translation_m':cl.translation_error_m,'relative_rotation_rad':cl.rotation_error_rad,'left_absolute_translation_m':cl.left_object_translation_error_m,'left_absolute_rotation_rad':cl.left_object_rotation_error_rad,'right_absolute_translation_m':cl.right_object_translation_error_m,'right_absolute_rotation_rad':cl.right_object_rotation_error_rad}

def first_violations(rows,c):
 out={}
 for row in rows:
  for key,ok in row['gate_checks'].items():
   if not ok and key not in out: out[key]={'frame_index':row['frame_index'],'time_s':row['time_s'],'reason':row['gate_reasons']}
 return out

def replay(path, output):
 path,output=Path(path),Path(output); raw=json.loads((path/'tracking_observations.json').read_text()); rows=raw['observations']; req=load_qp3_positive()['raw_qp2']['request']; initial=np.asarray(req['object_pose']); refs=build_reference_sequence(load_qp3_positive()); model=RobotModel(); c=json.loads((ROOT/'offline/task28_p5_qp_5_r1/preregistration.json').read_text())
 report={'schema':'task28.g2_p5_qp_5_r1.forensic.v1','source_dir':str(path),'source_tracking_sha256':sha(path/'tracking_observations.json'),'frame_count':len(rows),'rows':[],'first_violations':{},'hypotheses':{},'source_integrity':{'qp4_reference_source':'HASH_BOUND_ARCHIVED_QP4','original_qp5_unchanged':True}}
 for i,r in enumerate(rows):
  t=float(r['sim_time_s']); t_joint=max(0.,t-DT); t_obj=t
  orig_q=np.asarray(r['reference']['q_ref_14'],float); orig_qd=np.asarray(r['reference']['qdot_ref_14'],float)
  mixed=sample(refs,initial,t_joint); matched=sample(refs,initial,t); offset=sample(refs,initial,max(0.,t+DT))
  obj_orig=np.asarray(r['object_pose_world'],float); obj_matched=matched.object_pose_world
  tcp_l=checked_transform(r['arms']['left']['tcp_world']); tcp_r=checked_transform(r['arms']['right']['tcp_world'])
  cm=evaluate_closure(obj_orig,tcp_l,tcp_r); cc=evaluate_closure(obj_matched,tcp_l,tcp_r)
  q_actual=np.r_[arr(r['arms']['left']['q_rad'],9)[:7],arr(r['arms']['right']['q_rad'],9)[:7]]; dq_actual=np.r_[arr(r['arms']['left']['dq_rad_s'],9)[:7],arr(r['arms']['right']['dq_rad_s'],9)[:7]]
  fk_l=model.fk('left',matched.q_ref_14[:7]); fk_r=model.fk('right',matched.q_ref_14[7:]); ref_geom=evaluate_closure(obj_matched,fk_l,fk_r)
  reasons=[]; checks={}
  checks['relative_translation']=cm.translation_error_m<=c['closure_translation_tolerance_m']; checks['relative_rotation']=cm.rotation_error_rad<=c['closure_rotation_tolerance_rad']; checks['left_absolute_translation']=cm.left_object_translation_error_m<=c['closure_translation_tolerance_m']; checks['right_absolute_translation']=cm.right_object_translation_error_m<=c['closure_translation_tolerance_m']; checks['left_absolute_rotation']=cm.left_object_rotation_error_rad<=c['closure_rotation_tolerance_rad']; checks['right_absolute_rotation']=cm.right_object_rotation_error_rad<=c['closure_rotation_tolerance_rad']
  checks['position_tracking']=bool(np.max(np.abs(q_actual-matched.q_ref_14))<=c['tracking_position_tolerance_rad']); checks['velocity_tracking']=bool(np.max(np.abs(dq_actual-matched.qdot_ref_14))<=c['tracking_velocity_tolerance_rad_s']); checks['command_readback']=bool(max(np.max(np.abs(arr(r['arms']['left']['commanded_effort'],9)-arr(r['arms']['left']['applied_effort'],9))),np.max(np.abs(arr(r['arms']['right']['commanded_effort'],9)-arr(r['arms']['right']['applied_effort'],9))))<=c['command_readback_tolerance']); checks['saturation']=not (r['arms']['left']['saturated'] or r['arms']['right']['saturated']); checks['rate_limit']=not (r['arms']['left']['rate_limited'] or r['arms']['right']['rate_limited']); checks['reference_geometry']=ref_geom.valid
  reasons=[k for k,v in checks.items() if not v]
  report['rows'].append({'frame_index':i,'time_s':t,'collector_joint_sample_time_s':t_joint,'collector_object_sample_time_s':t_obj,'matched_reference_time_s':t,'one_step_later_reference_time_s':max(0.,t+DT),'recorded_reference_q14':orig_q.tolist(),'recorded_reference_qdot14':orig_qd.tolist(),'mixed_time_q14':mixed.q_ref_14.tolist(),'matched_time_q14':matched.q_ref_14.tolist(),'actual_q14':q_actual.tolist(),'actual_dq14':dq_actual.tolist(),'object_pose_recorded':obj_orig.tolist(),'object_pose_matched':obj_matched.tolist(),'actual_tcp_left':tcp_l.tolist(),'actual_tcp_right':tcp_r.tolist(),'closure_recorded_object':closure_dict(cm),'closure_matched_object':closure_dict(cc),'reference_geometry_at_matched_time':closure_dict(ref_geom),'position_error_recorded_inf':float(np.max(np.abs(q_actual-orig_q))),'position_error_matched_inf':float(np.max(np.abs(q_actual-matched.q_ref_14))),'velocity_error_matched_inf':float(np.max(np.abs(dq_actual-matched.qdot_ref_14))),'command_saturation':bool(r['arms']['left']['saturated'] or r['arms']['right']['saturated']),'rate_limited':bool(r['arms']['left']['rate_limited'] or r['arms']['right']['rate_limited']),'gate_checks':checks,'gate_reasons':reasons})
 report['first_violations']=first_violations(report['rows'],c)
 report['hypotheses']={'mixed_collector_time':'q_ref(max(0,t-dt)) versus object_ref(t), reproduces original collector semantics; diagnostic only','matched_same_time':'q_ref(t) and object_ref(t), corrected temporal hypothesis; not a retrospective acceptance override','post_step_one_step_later':'q_ref(t+dt) and object_ref(t+dt), required for a post-step state; diagnostic only','reference_geometry':'matched-time interpolated QP reference FK against matched-time virtual object; tests mathematical reference itself'}
 report['summary']={'first_recorded_closure_failure_frame':report['first_violations'].get('relative_translation'),'first_left_absolute_translation_failure':report['first_violations'].get('left_absolute_translation'),'first_right_absolute_translation_failure':report['first_violations'].get('right_absolute_translation'),'max_relative_translation_m':max(x['closure_matched_object']['relative_translation_m'] for x in report['rows']),'max_left_absolute_translation_m':max(x['closure_matched_object']['left_absolute_translation_m'] for x in report['rows']),'max_right_absolute_translation_m':max(x['closure_matched_object']['right_absolute_translation_m'] for x in report['rows']),'max_reference_geometry_translation_m':max(x['reference_geometry_at_matched_time']['relative_translation_m'] for x in report['rows']),'original_recorded_q_mismatch_max':max(x['position_error_recorded_inf'] for x in report['rows']),'matched_q_mismatch_max':max(x['position_error_matched_inf'] for x in report['rows'])}
 output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n'); return report
if __name__=='__main__':
 ap=argparse.ArgumentParser(); ap.add_argument('--input',type=Path,required=True); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args(); r=replay(a.input,a.output); print(json.dumps(r['summary'],sort_keys=True))
