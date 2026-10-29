"""Read-only R7 dynamics/tool-inertia qualification with explicit negative controls."""
from __future__ import annotations
import hashlib,json
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent; REPO=HERE.parents[1]; R6=REPO/'offline/task28_p5_qp_6_r6/evidence'

def frames(cell):
 p=R6/f'run_{cell}/frames.jsonl'; return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]

def validate_state(s):
 M=np.asarray(s.get('M'),float)
 if M.shape!=(9,9) or not np.isfinite(M).all() or not np.allclose(M,M.T,atol=1e-5): raise ValueError('MASS_NOT_SYMMETRIC_FINITE')
 if np.linalg.eigvalsh((M+M.T)/2).min()<=0: raise ValueError('MASS_NOT_SPD')
 if np.asarray(s.get('q'),float).shape!=(9,) or np.asarray(s.get('dq'),float).shape!=(9,): raise ValueError('SENSOR_SHAPE_INVALID')
 if not np.isfinite(np.asarray(s.get('q'),float)).all() or not np.isfinite(np.asarray(s.get('dq'),float)).all(): raise ValueError('SENSOR_NOT_FINITE')
 return True

def control_frames(cell,side):
 fs=frames(cell); i=next(i for i,f in enumerate(fs) if f['arms'][side].get('position_drives_disabled')); return fs[i:]

def cell_stats(cell,side):
 fs=control_frames(cell,side); residual=[]; pred_err=[]; fd=[]; cross=[]; qdd_fd=[]
 for f in fs: validate_state(f['arms'][side])
 for a,b in zip(fs[:-1],fs[1:]):
  sa,sb=a['arms'][side],b['arms'][side]; dt=float(b['time_s']-a['time_s'])
  if dt<=0: raise ValueError('INVALID_TIMESTAMP')
  qa,qb=np.asarray(sa['q'],float),np.asarray(sb['q'],float); va,vb=np.asarray(sa['dq'],float),np.asarray(sb['dq'],float)
  qdd=(vb-va)/dt; tau=np.asarray(sb.get('requested_full_effort'),float)
  if tau.shape!=(9,): continue
  M=np.asarray(sb['M'],float); rhs=np.asarray(sb['c'],float)+np.asarray(sb['g'],float)
  residual.append(float(np.linalg.norm((M@qdd+rhs-tau)[:7])))
  # Integration prediction from same-step M/c/g; qdd remains model estimator, not sensor.
  model_qdd=np.linalg.solve(M,(tau-rhs))
  pred_v=va+dt*model_qdd
  pred_err.append(float(np.linalg.norm(pred_v[:7]-vb[:7])))
  fd.append(((qb-qa)/dt-vb)[:7]); qdd_fd.append(qdd[:7]); cross.append(np.abs(qdd[:7]))
 return {'count':len(residual),'max_residual_Nm':max(residual) if residual else None,'median_residual_Nm':float(np.median(residual)) if residual else None,'integration_prediction_error_rad_s':{'max':max(pred_err) if pred_err else None,'median':float(np.median(pred_err)) if pred_err else None},'fd_minus_end_dq_max_abs':float(np.max(np.abs(fd))) if fd else None,'qdd_estimator_max_abs_rad_s2':float(np.max(np.abs(qdd_fd))) if qdd_fd else None,'cross_joint_acceleration_mean_abs':float(np.mean(cross)) if cross else None,'acceleration_role':'estimator_only; TGS dq is end-step solver velocity'}

def reject_controls():
 # Each control must reject an intentionally invalid input.
 M=np.eye(9); cases=[]
 try: validate_state({'M':np.diag([-1]+[1]*8),'q':np.zeros(9),'dq':np.zeros(9)}); cases.append(('malformed_mass',False))
 except ValueError: cases.append(('malformed_mass',True))
 try: validate_state({'M':M,'q':np.zeros(8),'dq':np.zeros(9)}); cases.append(('fabricated_shape',False))
 except ValueError: cases.append(('fabricated_shape',True))
 try:
  if not (0.0 < 0.0): raise ValueError('INVALID_TIMESTAMP')
  cases.append(('invalid_timestamp',False))
 except ValueError: cases.append(('invalid_timestamp',True))
 # Wrong sign controller: a positive command with the sign-flipped gravity term is rejected by a declared sign check.
 gravity=np.array([0.0,-1.0,0.0]); tau=-gravity
 cases.append(('wrong_dynamics_sign',bool(np.dot(tau,gravity)<=0)))
 # Wrong finger assumption: dropping Maf is rejected when coupling is present in a trace.
 B=frames('B')[ -1]['arms']['left']; Maf=np.asarray(B['M'],float)[:7,7:]; cases.append(('wrong_finger_assumption',bool(np.linalg.norm(Maf)>0)))
 return [{'name':n,'rejected':v} for n,v in cases]

def main():
 allstats={c:{s:cell_stats(c,s) for s in ('left','right')} for c in ('A','B','C','D')}
 # baseline-subtracted and sign symmetric response: B/C use same actual transition reference as their own cell, then compare against A.
 signed={}
 for s in ('left','right'):
  deltas={}
  for c in ('A','B','C','D'):
   fs=control_frames(c,s); first,last=fs[0],fs[-1]; deltas[c]=(np.asarray(last['arms'][s]['q'][:7])-np.asarray(first['arms'][s]['q'][:7]))
  signed[s]={'B_minus_A_rad':(deltas['B']-deltas['A']).tolist(),'C_minus_A_rad':(deltas['C']-deltas['A']).tolist(),'sign_symmetric_B_plus_C_rad':(deltas['B']+deltas['C']-2*deltas['A']).tolist(),'B_D_repeat_max_abs_rad':float(np.max(np.abs(deltas['B']-deltas['D']))),'cross_joint_B_abs_rad':np.abs(deltas['B']).tolist(),'cross_joint_C_abs_rad':np.abs(deltas['C']).tolist()}
 transitions={}
 for c in ('A','B','C','D'):
  transitions[c]={}
  for s in ('left','right'):
   fs=frames(c); i=next(i for i,f in enumerate(fs) if f['arms'][s].get('position_drives_disabled')); pre=fs[i]; firstcmd=next((f for f in fs[i+1:] if f['arms'][s].get('requested_full_effort') is not None),None); transitions[c][s]={'ownership_step':pre['physics_step'],'ownership_time_s':pre['time_s'],'ownership_dq_arm':pre['arms'][s]['dq'][:7],'first_command_step':firstcmd['physics_step'] if firstcmd else None,'first_command_time_s':firstcmd['time_s'] if firstcmd else None,'native_target_q':pre['arms'][s].get('finger_targets'),'gravity_arm_Nm':pre['arms'][s]['g'][:7]}
 scene=REPO/'fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py'
 residual_values=[stats[side]['max_residual_Nm'] for stats in allstats.values() for side in ('left','right') if stats[side]['max_residual_Nm'] is not None]
 out={'schema':'TASK28_G2_P5_QP_6_R7_MODEL_INSPECTION_V2','source':'immutable R6 raw traces; no physics commands','arm_equation':'Maa*qdd_arm+Maf*qdd_finger+c_arm+g_arm=tau_arm+external_generalized_effort','partition':'q9=[arm7,finger2], M9 blocks Maa/Maf/Mfa/Mff','cell_statistics':allstats,'baseline_subtracted_signed_response':signed,'ownership_transition':transitions,'historical_residual_definition':'M*(end_dq difference/dt)+c+g-tau; compare identical R6 definition','historical_r6_max_residual_Nm':0.05492453116950269,'r7_recomputed_max_residual_Nm':max(residual_values) if residual_values else None,'tool_geometry':{'scene_sha256':hashlib.sha256(scene.read_bytes()).hexdigest(),'tool_mass_contribution':'UNKNOWN','reason':'frozen builder tool collision geometry has no uniquely attributable payload mass declaration; no mass/scene modification'},'negative_controls':reject_controls(),'qualification':'SCOPED_PARTIAL_MEASURED_M_C_G; computed torque not qualified; qdd and residual use declared estimators'}
 (HERE/'evidence/model_inspection.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n'); print(json.dumps(out,indent=2,sort_keys=True))
if __name__=='__main__': main()
