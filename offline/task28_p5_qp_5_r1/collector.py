#!/usr/bin/env python3
"""R1 time-aligned bilateral torque collector with runtime fail-closed stop."""
from __future__ import annotations
import argparse, hashlib, json, os, time, traceback
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[1]; CONFIG=HERE/'preregistration.json'
def sha(p):
 h=hashlib.sha256();
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def dump(p,x):
 p=Path(p); p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('w') as f: json.dump(x,f,indent=2,sort_keys=True,allow_nan=False); f.write('\n'); f.flush(); os.fsync(f.fileno())
class GateStop(RuntimeError): pass
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--asset',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--prereg-sha256',required=True); a=ap.parse_args()
 if sha(CONFIG)!=a.prereg_sha256: raise SystemExit('PREREGISTRATION_HASH_MISMATCH')
 c=json.loads(CONFIG.read_text())
 if not a.asset.is_file() or sha(a.asset)!=c['fixture_usd_sha256']: raise SystemExit('MISSING_OR_WRONG_ASSET')
 out=Path(a.output); out.mkdir(parents=False,exist_ok=False)
 req=json.loads((ROOT/'offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json').read_text())['positive']['raw_qp2']['request']; q14=np.asarray(req['q14'],float); initial_pose=np.asarray(req['object_pose'],float)
 status={'schema':'task28.g2_p5_qp_5_r1.collector.v1','status':'STARTED','preregistration_sha256':a.prereg_sha256,'collector_sha256':sha(Path(__file__)),'asset_sha256':sha(a.asset),'asset':str(a.asset.resolve()),'execution_authorized':False,'physical_robot_tested':False,'scope':'VIRTUAL_GEOMETRY_ONLY','source_q14':q14.tolist(),'source_object_pose_world':initial_pose.tolist(),'completed_files':[],'tracking_steps_executed':0}
 app=None; total=0; started=time.monotonic(); active=None; phase=None; previous_cmd={'left':None,'right':None}
 try:
  from isaacsim import SimulationApp
  app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
  from isaacsim.core.api import World
  from isaacsim.core.prims import SingleArticulation,RigidPrim
  from isaacsim.core.utils.stage import add_reference_to_stage
  from scipy.spatial.transform import Rotation
  import sys; sys.path.insert(0,str(ROOT))
  from offline.task28_g1_1.task28_validator.model import RobotModel
  from offline.task28_p5_qp_4.reference import load_qp3_positive,build_reference_sequence
  from offline.task28_p5_qp_5.controller import explicit_effort,sample_reference,interpolate_pose
  from offline.task28_p5_qp_5_r1.runtime_gate import validate_post
  model=RobotModel(); refs=build_reference_sequence(load_qp3_positive())
  world=World(stage_units_in_meters=1.0,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
  arms={}; links={}; views={}
  for side in ('left','right'):
   root=f'/World/Probe_{side}'; add_reference_to_stage(usd_path=str(a.asset.resolve()),prim_path=root)
   arms[side]=world.scene.add(SingleArticulation(root,name='fr3_'+side,position=np.asarray([0.65,-0.6,0.] if side=='left' else [0.65,0.6,0.])))
   links[side]=world.scene.add(RigidPrim(root+'/fr3_link7',name='link7_'+side))
  world.reset(); idx=np.arange(9,dtype=np.int32); kp=np.asarray(c['pd_kp']); kd=np.asarray(c['pd_kd']); bounds=np.asarray(c['effort_bounds']); rates=np.asarray(c['effort_rate_bounds_per_step'])
  for side in ('left','right'):
   names=list(arms[side].dof_names)
   if names!=c['expected_all_dof_names']: raise RuntimeError('DOF_ORDER_MISMATCH:'+side)
   views[side]=arms[side]._articulation_view; views[side].set_effort_modes('force',joint_indices=idx); views[side].set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
   qside=q14[:7] if side=='left' else q14[7:]; arms[side].set_joint_positions(np.r_[qside,c['finger_q_m']],joint_indices=idx); arms[side].set_joint_velocities(np.zeros(9),joint_indices=idx)
  status['runtime']={'physics_dt_s':float(world.get_physics_dt()),'gravity_m_s2':c['gravity_m_s2'],'arms':{side:{'all_dof_names':list(arms[side].dof_names),'ordered_dof_indices':idx.tolist(),'articulation_path':'/World/Probe_'+side,'link7_path':'/World/Probe_'+side+'/fr3_link7','effort_mode':views[side].get_effort_modes()[0],'runtime_drive_gains_zero':True} for side in ('left','right')},'controller':{'kp':kp.tolist(),'kd':kd.tolist(),'effort_bounds':bounds.tolist(),'effort_rate_bounds_per_step':rates.tolist(),'finger_policy':'explicit gravity+PD full 9D'},'temporal_contract':c['temporal_contract']}
  tracking_origin=0.0
  def cursor_now(stage): return float(world.current_time) if stage=='hold' else float(world.current_time)-tracking_origin
  def step_once(stage):
   nonlocal total,previous_cmd
   pre_t=cursor_now(stage); moving=stage in ('tracking','final_hold')
   if moving: qref_pre,qdref_pre,source_step_pre,phase_pre=sample_reference(refs,pre_t)
   else: qref_pre,qdref_pre,source_step_pre,phase_pre=q14,np.zeros(14),-1,'hold'
   pre_states={}; commands={}; prior_cmd={side:(None if previous_cmd[side] is None else previous_cmd[side].copy()) for side in ('left','right')}
   for side,sl in (('left',slice(0,7)),('right',slice(7,14))):
    q=np.asarray(arms[side].get_joint_positions(),float); dq=np.asarray(arms[side].get_joint_velocities(),float); g=np.asarray(views[side].get_generalized_gravity_forces(),float)[0]
    pre_states[side]={'q_rad':q.tolist(),'dq_rad_s':dq.tolist()}; ctrl=explicit_effort(q,dq,np.r_[qref_pre[sl],c['finger_q_m']],np.r_[qdref_pre[sl],np.zeros(2)],g,kp,kd,bounds,rates,previous_cmd[side]); commands[side]=ctrl; views[side].set_effort_modes('force',joint_indices=idx); arms[side].set_joint_efforts(ctrl['command'],joint_indices=idx); previous_cmd[side]=ctrl['command'].copy()
   world.step(render=False); total+=1; post_t=cursor_now(stage); row={'physics_step_index':int(world.current_time_step_index),'sim_time_s':post_t,'physics_time_s':float(world.current_time),'controller_time_s':pre_t,'post_step_time_s':post_t,'phase':stage,'source_q14':q14.tolist(),'source_object_pose_world':initial_pose.tolist(),'pre_step':{'time_s':pre_t,'states':pre_states,'reference_q14':np.asarray(qref_pre).tolist(),'reference_qdot14':np.asarray(qdref_pre).tolist(),'source_step_index':source_step_pre,'phase':phase_pre},'reference':{},'object_pose_world':None,'arms':{}}
   if moving: qref_post,qdref_post,k,phase_post=sample_reference(refs,post_t); obj_post=interpolate_pose(refs,initial_pose,post_t)
   else: qref_post,qdref_post,k,phase_post=q14,np.zeros(14),-1,'hold'; obj_post=initial_pose
   row['reference']={'q_ref_14':np.asarray(qref_post).tolist(),'qdot_ref_14':np.asarray(qdref_post).tolist(),'source_step_index':k,'phase':phase_post,'source_identity':'task28_qp3_seed28_transport40_5step'}; row['object_pose_world']=np.asarray(obj_post).tolist()
   for side,sl in (('left',slice(0,7)),('right',slice(7,14))):
    q=np.asarray(arms[side].get_joint_positions(),float); dq=np.asarray(arms[side].get_joint_velocities(),float); pos,quat=links[side].get_world_poses(); pos=np.asarray(pos[0],float); quat=np.asarray(quat[0],float); link=np.eye(4); link[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix(); link[:3,3]=pos; q7=q[:7]; offset=np.linalg.inv(model._chain(side,q14[sl])[0][side+'_fr3_link7'])@model.fk(side,q14[sl]); tcp_live=link@offset; ctrl=commands[side]
    row['arms'][side]={'dof_names':list(arms[side].dof_names),'q_rad':q.tolist(),'dq_rad_s':dq.tolist(),'commanded_effort':ctrl['command'].tolist(),'raw_effort':ctrl['raw'].tolist(),'gravity_pre':np.asarray(ctrl['raw']-np.asarray(kp)*(np.r_[qref_pre[sl],c['finger_q_m']]-np.r_[pre_states[side]['q_rad'][:7],c['finger_q_m']])-np.asarray(kd)*(np.r_[qdref_pre[sl],np.zeros(2)]-np.asarray(pre_states[side]['dq_rad_s']))).tolist(),'applied_effort':np.asarray(arms[side].get_applied_joint_efforts(),float).tolist(),'measured_projected_effort':np.asarray(arms[side].get_measured_joint_efforts(),float).tolist(),'link7_world':link.tolist(),'tcp_world':tcp_live.tolist(),'model_tcp_world':model.fk(side,q7).tolist(),'saturated':ctrl['saturated'],'rate_limited':ctrl['rate_limited']}
   gate=validate_post(row,expected_time=post_t,config=c,model=model,refs=refs,initial_pose=initial_pose,previous_cmd=prior_cmd,phase='tracking' if moving else 'hold'); row['runtime_gate']=gate; return row
  active={'stage':'static_hold','observations':[]}; phase='hold'
  for _ in range(int(c['static_hold_steps'])):
   row=step_once('hold'); active['observations'].append(row)
   if not row['runtime_gate']['valid']: dump(out/'static_hold_observations.json',active); status['status']='REJECTED'; status['failure_phase']='static_hold'; status['first_failure']=row; raise GateStop('STATIC_GATE_FAILED:'+','.join(row['runtime_gate']['reasons']))
  dump(out/'static_hold_observations.json',active); status['completed_files'].append('static_hold_observations.json'); status['static_hold_pass']=True
  tracking_origin=float(world.current_time); phase='tracking'; active={'stage':'tracking','observations':[]}
  for _ in range(12):
   row=step_once('tracking'); active['observations'].append(row); status['tracking_steps_executed']+=1
   if not row['runtime_gate']['valid']:
    dump(out/'tracking_observations.json',active); status['status']='REJECTED'; status['failure_phase']='tracking'; status['first_failure']=row; raise GateStop('TRACKING_GATE_FAILED:'+','.join(row['runtime_gate']['reasons']))
  for _ in range(int(c['final_hold_steps'])):
   row=step_once('final_hold'); active['observations'].append(row)
   if not row['runtime_gate']['valid']:
    dump(out/'tracking_observations.json',active); status['status']='REJECTED'; status['failure_phase']='final_hold'; status['first_failure']=row; raise GateStop('FINAL_HOLD_GATE_FAILED:'+','.join(row['runtime_gate']['reasons']))
  dump(out/'tracking_observations.json',active); status['completed_files'].append('tracking_observations.json'); status['status']='COLLECTION_COMPLETE_NOT_EVALUATED'
 except BaseException as exc:
  if status.get('status')=='STARTED': status['status']='COLLECTION_FAILED'
  status['failure']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
  if active is not None and status.get('failure_phase') not in ('static_hold','tracking','final_hold'): dump(out/'partial_observations.json',active)
 finally:
  status['physics_steps']=total; status['elapsed_wall_s']=time.monotonic()-started; dump(out/'collector_status.json',status)
  if app is not None: app.close()
 return 0 if status['status']=='COLLECTION_COMPLETE_NOT_EVALUATED' else 1
if __name__=='__main__': raise SystemExit(main())
