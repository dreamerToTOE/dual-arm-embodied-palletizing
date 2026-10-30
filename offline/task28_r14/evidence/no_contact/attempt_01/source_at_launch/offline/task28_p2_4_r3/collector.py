#!/usr/bin/env python3
"""Isolated R3 gravity+PD stabilized explicit-effort collector."""
from __future__ import annotations
import argparse, hashlib, json, os, time, traceback
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent; CONFIG=HERE/'experiment_preregistration.json'
def sha(p):
 h=hashlib.sha256();
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def dump(p,x):
 with Path(p).open('x') as f: json.dump(x,f,indent=2,sort_keys=True,allow_nan=False); f.write('\n'); f.flush(); os.fsync(f.fileno())
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--asset',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--prereg-sha256',required=True); ap.add_argument('--stage',choices=['stability','online'],required=True); ap.add_argument('--stability-gate',type=Path); a=ap.parse_args()
 if sha(CONFIG)!=a.prereg_sha256: raise SystemExit('PREREGISTRATION_HASH_MISMATCH')
 c=json.loads(CONFIG.read_text());
 if not a.asset.is_file() or sha(a.asset)!=c['fixture_usd_sha256']: raise SystemExit('MISSING_OR_WRONG_ASSET')
 if a.stage=='online':
  if not a.stability_gate or not a.stability_gate.is_file(): raise SystemExit('STABILITY_GATE_REQUIRED')
  gate=json.loads(a.stability_gate.read_text())
  if gate.get('status')!='STABILITY_GATE_PASS' or gate.get('preregistration_sha256')!=a.prereg_sha256: raise SystemExit('STABILITY_GATE_NOT_PASS')
 a.output.mkdir(parents=False,exist_ok=False); status={'schema':'task28.g3_p2_4_r3.collector.v1','stage':a.stage,'status':'STARTED','preregistration_sha256':a.prereg_sha256,'collector_sha256':sha(Path(__file__)),'asset_sha256':sha(a.asset),'asset':str(a.asset.resolve()),'completed_cases':[],'execution_authorized':False,'physical_robot_tested':False,'frozen_usd_modified':False}
 app=None; total=0; started=time.monotonic(); active=None
 try:
  from isaacsim import SimulationApp
  app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
  from isaacsim.core.api import World
  from isaacsim.core.prims import SingleArticulation,RigidPrim
  from isaacsim.core.utils.stage import add_reference_to_stage
  world=World(stage_units_in_meters=1.0,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
  add_reference_to_stage(usd_path=str(a.asset.resolve()),prim_path='/World/ProbeFR3')
  robot=world.scene.add(SingleArticulation('/World/ProbeFR3',name='fr3',position=np.asarray(c['base_world_m'])))
  link=world.scene.add(RigidPrim('/World/ProbeFR3/fr3_link7',name='link7')); world.reset(); view=robot._articulation_view; names=list(robot.dof_names)
  if names!=c['expected_all_dof_names']: raise RuntimeError('DOF_ORDER_MISMATCH')
  idx=np.arange(9,dtype=np.int32); qref=np.asarray(c['configuration_rad']+c['finger_q_m'],float); kp=np.asarray(c['pd_kp'],float); kd=np.asarray(c['pd_kd'],float); bounds=np.asarray(c['effort_bounds'],float); rate=np.asarray(c['effort_rate_bounds'],float)
  view.set_effort_modes('force',joint_indices=idx); view.set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
  status['runtime']={'all_dof_names':names,'effort_mode':view.get_effort_modes()[0],'pre_gains':{'kps':np.asarray(view.get_gains()[0])[0].tolist(),'kds':np.asarray(view.get_gains()[1])[0].tolist()},'runtime_drive_gains_zero':True,'controller_kp':kp.tolist(),'controller_kd':kd.tolist(),'effort_bounds':bounds.tolist(),'effort_rate_bounds':rate.tolist(),'physics_dt_s':float(world.get_physics_dt()),'set_effort_api':'set_joint_efforts -> set_dof_actuation_forces','applied_readback_api':'get_applied_joint_efforts -> get_dof_actuation_forces','projected_readback_api':'get_measured_joint_efforts -> get_dof_projected_joint_forces','source_asset_unchanged':True}
  cases=c['stability_cases'] if a.stage=='stability' else c['online_cases']; steps=c['stability_steps'] if a.stage=='stability' else c['online_steps']
  for cid in cases:
   world.reset(); robot.set_joint_positions(qref,joint_indices=idx); robot.set_joint_velocities(np.zeros(9),joint_indices=idx); view.set_effort_modes('force',joint_indices=idx); view.set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
   force=np.asarray([0.,0.,0.] if a.stage=='stability' else c['online_force_world_n'][cid],float); active={'case_id':cid,'stage':a.stage,'preregistration_sha256':a.prereg_sha256,'samples':[]}; oracle={'case_id':cid,'force_world_n':force.tolist(),'application_points_world_m':[]}; prev_cmd=None
   for k in range(steps):
    q_pre=np.asarray(robot.get_joint_positions(),float); dq_pre=np.asarray(robot.get_joint_velocities(),float); g_pre=np.asarray(view.get_generalized_gravity_forces(),float)[0]; fb=kp*(qref-q_pre)-kd*dq_pre; raw=g_pre+fb; cmd=np.clip(raw, -bounds,bounds)
    if prev_cmd is not None: cmd=np.clip(cmd,prev_cmd-rate,prev_cmd+rate)
    sat=bool(np.any(np.abs(cmd-raw)>1e-9)); prev_cmd=cmd.copy(); robot.set_joint_efforts(cmd,joint_indices=idx)
    point=np.zeros(3)
    if np.any(force): pos,_=link.get_world_poses(); point=np.asarray(pos[0],float); link.apply_forces_and_torques_at_pos(forces=force[None,:],torques=np.zeros((1,3)),positions=point[None,:],is_global=True)
    world.step(render=False); total+=1; read_start=time.monotonic(); q=np.asarray(robot.get_joint_positions(),float); dq=np.asarray(robot.get_joint_velocities(),float); applied=np.asarray(robot.get_applied_joint_efforts(),float); projected=np.asarray(robot.get_measured_joint_efforts(),float); M=np.asarray(view.get_mass_matrices(),float); cor=np.asarray(view.get_coriolis_and_centrifugal_forces(),float); grav=np.asarray(view.get_generalized_gravity_forces(),float); read_end=time.monotonic(); t=float(world.current_time)
    if not np.isfinite(np.r_[q,dq,cmd,applied,projected,M.ravel(),cor.ravel(),grav.ravel()]).all(): raise RuntimeError('NONFINITE_STATE')
    if abs(q[0])>c['q1_abs_limit_rad'] or np.max(np.abs(dq[:7]))>c['arm_speed_limit_rad_s']: raise RuntimeError('BOUNDED_MOTION_LIMIT')
    active['samples'].append({'step':k,'sim_time_s':t,'q_rad_or_m':q.tolist(),'dq_rad_s_or_m_s':dq.tolist(),'commanded_effort':cmd.tolist(),'gravity_feedforward_pre':g_pre.tolist(),'feedback_pre':fb.tolist(),'applied_effort_readback':applied.tolist(),'measured_projected_effort':projected.tolist(),'mass_matrix':M[0].tolist(),'coriolis_compensation':cor[0].tolist(),'gravity_compensation':grav[0].tolist(),'position_error':(qref-q).tolist(),'velocity_error':(-dq).tolist(),'saturated':sat,'source_read_duration_s':read_end-read_start,'physics_step_index':int(world.current_time_step_index)})
   oracle['application_points_world_m']=[point.tolist() for _ in active['samples']]
   dump(a.output/(cid+'_observations.json'),active); dump(a.output/(cid+'_oracle.json'),oracle); active=None; status['completed_cases'].append(cid)
  status['status']='COLLECTION_COMPLETE_NOT_EVALUATED'
 except BaseException as exc:
  status['status']='COLLECTION_FAILED'; status['failure']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
  if active is not None: dump(a.output/'partial_observations.json',active)
 finally:
  status['physics_steps']=total; status['elapsed_wall_s']=time.monotonic()-started; dump(a.output/'collector_status.json',status)
  if app is not None: app.close()
 return 0 if status['status']=='COLLECTION_COMPLETE_NOT_EVALUATED' else 1
if __name__=='__main__': raise SystemExit(main())
