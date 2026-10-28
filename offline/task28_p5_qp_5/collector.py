#!/usr/bin/env python3
"""Bounded two-articulation explicit-effort experiment; no production bridge."""
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
def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--asset',required=True,type=Path); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--prereg-sha256',required=True); a=ap.parse_args()
 if sha(CONFIG)!=a.prereg_sha256: raise SystemExit('PREREGISTRATION_HASH_MISMATCH')
 c=json.loads(CONFIG.read_text());
 if not a.asset.is_file() or sha(a.asset)!=c['fixture_usd_sha256']: raise SystemExit('MISSING_OR_WRONG_ASSET')
 out=Path(a.output); out.mkdir(parents=False,exist_ok=False)
 source=json.loads((ROOT/'offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json').read_text())['positive']['raw_qp2']['request']
 q14=np.asarray(source['q14'],float); object_pose=np.asarray(source['object_pose'],float)
 status={'schema':'task28.g2_p5_qp_5.collector.v1','status':'STARTED','preregistration_sha256':a.prereg_sha256,'collector_sha256':sha(Path(__file__)),'asset_sha256':sha(a.asset),'asset':str(a.asset.resolve()),'execution_authorized':False,'physical_robot_tested':False,'scope':'VIRTUAL_GEOMETRY_ONLY','completed_files':[],'source_q14':q14.tolist(),'source_object_pose_world':object_pose.tolist()}
 app=None; active=None; total=0; previous={'left':None,'right':None}; started=time.monotonic()
 try:
  from isaacsim import SimulationApp
  app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
  from isaacsim.core.api import World
  from isaacsim.core.prims import SingleArticulation,RigidPrim
  from isaacsim.core.utils.stage import add_reference_to_stage
  import sys; sys.path.insert(0,str(ROOT))
  from offline.task28_g1_1.task28_validator.model import RobotModel
  from offline.task28_p5_qp_4.reference import load_qp3_positive, build_reference_sequence
  from offline.task28_p5_qp_5.controller import explicit_effort, sample_reference, interpolate_pose
  from offline.task28_p5_qp_5.evaluate import evaluate_rows
  model=RobotModel(); refs=build_reference_sequence(load_qp3_positive())
  world=World(stage_units_in_meters=1.0,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
  arms={}; links={}; views={}
  for side in ('left','right'):
   root=f'/World/Probe_{side}'
   add_reference_to_stage(usd_path=str(a.asset.resolve()),prim_path=root)
   arms[side]=world.scene.add(SingleArticulation(root,name='fr3_'+side,position=np.asarray(c['base_world_m'][side])))
   links[side]=world.scene.add(RigidPrim(root+'/fr3_link7',name='link7_'+side))
  world.reset()
  idx=np.arange(9,dtype=np.int32); kp=np.asarray(c['pd_kp']); kd=np.asarray(c['pd_kd']); bounds=np.asarray(c['effort_bounds']); rates=np.asarray(c['effort_rate_bounds_per_step'])
  for side in ('left','right'):
   names=list(arms[side].dof_names)
   if names != c['expected_all_dof_names']: raise RuntimeError('DOF_ORDER_MISMATCH:'+side)
   views[side]=arms[side]._articulation_view; views[side].set_effort_modes('force',joint_indices=idx); views[side].set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
   arms[side].set_joint_positions(np.r_[q14[:7] if side=='left' else q14[7:],c['finger_q_m']],joint_indices=idx); arms[side].set_joint_velocities(np.zeros(9),joint_indices=idx)
  status['runtime']={'physics_dt_s':float(world.get_physics_dt()),'gravity_m_s2':c['gravity_m_s2'],'arms':{side:{'all_dof_names':list(arms[side].dof_names),'ordered_dof_indices':idx.tolist(),'articulation_path':'/World/Probe_'+side,'link7_path':'/World/Probe_'+side+'/fr3_link7','effort_mode':views[side].get_effort_modes()[0],'runtime_drive_gains_zero':True} for side in ('left','right')},'controller':{'kp':kp.tolist(),'kd':kd.tolist(),'effort_bounds':bounds.tolist(),'effort_rate_bounds_per_step':rates.tolist(),'finger_policy':'explicit gravity+PD hold at q=[0.04,0.04], full 9D command'} ,'reference_semantics':c['sampler']}
  # Initialization is position-only; first recorded frame is pre-step and receives gravity+PD effort.
  for side in ('left','right'): views[side].set_effort_modes('force',joint_indices=idx)
  tracking_origin=0.0
  def collect_frame(step,phase,prev_cmd):
   nonlocal total
   cursor=float(world.current_time) if phase=='hold' else float(world.current_time)-tracking_origin; qrefs,qdrefs,k,refphase=sample_reference(refs,max(0.,cursor- c['physics_dt_s'])) if phase=='tracking' else (q14,np.zeros(14),-1,'hold')
   row={'physics_step_index':int(world.current_time_step_index),'sim_time_s':cursor,'phase':phase,'object_pose_world':(object_pose if phase=='hold' else interpolate_pose(refs,object_pose,cursor)).tolist(),'reference':{'q_ref_14':np.asarray(qrefs).tolist(),'qdot_ref_14':np.asarray(qdrefs).tolist(),'source_step_index':k,'phase':refphase},'arms':{}}
   for side, sl in (('left',slice(0,7)),('right',slice(7,14))):
    q=np.asarray(arms[side].get_joint_positions(),float); dq=np.asarray(arms[side].get_joint_velocities(),float); g=np.asarray(views[side].get_generalized_gravity_forces(),float)[0]
    qref=np.r_[qrefs[sl],c['finger_q_m']]; dqref=np.r_[qdrefs[sl],np.zeros(2)]
    ctrl=explicit_effort(q,dq,qref,dqref,g,kp,kd,bounds,rates,prev_cmd[side]); views[side].set_effort_modes('force',joint_indices=idx); arms[side].set_joint_efforts(ctrl['command'],joint_indices=idx); prev_cmd[side]=ctrl['command'].copy()
    pos,quat=links[side].get_world_poses(); pos=np.asarray(pos[0],float); quat=np.asarray(quat[0],float)
    from scipy.spatial.transform import Rotation
    link= np.eye(4); link[:3,:3]=Rotation.from_quat(quat[[1,2,3,0]]).as_matrix(); link[:3,3]=pos
    q7=q[:7]; tcp=model.fk(side,q7); offset=np.linalg.inv(model._chain(side,q14[sl])[0][side+'_fr3_link7']) @ model.fk(side,q14[sl]); tcp_live=link@offset
    row['arms'][side]={'dof_names':list(arms[side].dof_names),'q_rad':q.tolist(),'dq_rad_s':dq.tolist(),'commanded_effort':ctrl['command'].tolist(),'raw_effort':ctrl['raw'].tolist(),'gravity_pre':g.tolist(),'applied_effort':np.asarray(arms[side].get_applied_joint_efforts(),float).tolist(),'measured_projected_effort':np.asarray(arms[side].get_measured_joint_efforts(),float).tolist(),'link7_world':link.tolist(),'tcp_world':tcp_live.tolist(),'model_tcp_world':tcp.tolist(),'saturated':ctrl['saturated'],'rate_limited':ctrl['rate_limited']}
   return row
  # Hold 240 steps. Commands are applied before each step; collect post-step state.
  for k in range(int(c['static_hold_steps'])):
   row=collect_frame(k,'hold',previous); active.setdefault('observations',[]).append(row) if active else None
   # apply commanded torques from row then advance; collect_frame applies command for next step
   world.step(render=False); total+=1
   if active is None: active={'stage':'static_hold','observations':[row]}
  dump(out/'static_hold_observations.json',active)
  gate=evaluate_rows(active['observations'],c,moving=False); dump(out/'static_hold_evaluation.json',gate); status['static_gate']=gate
  if gate['status']!='PASS': status['status']='STATIC_HOLD_REJECTED'; status['failure_stop']='STATIC_GATE_FAILED'; raise RuntimeError('STATIC_GATE_FAILED')
  # Track only after static gate, preserving same world state; reference clock restarts at this boundary.
  tracking_origin=float(world.current_time)
  active={'stage':'tracking','observations':[]}
  for k in range(int(c['tracking_intervals']*3 + c['final_hold_steps'])):
   row=collect_frame(k,'tracking',previous); active['observations'].append(row); world.step(render=False); total+=1
  dump(out/'tracking_observations.json',active); status['tracking_evaluation']=evaluate_rows(active['observations'],c,moving=True); status['completed_files']=['static_hold_observations.json','tracking_observations.json']; status['status']='COLLECTION_COMPLETE_NOT_EVALUATED'
 except BaseException as exc:
  if status.get('status')=='STARTED': status['status']='COLLECTION_FAILED'
  status['failure']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
  if active is not None: dump(out/'partial_observations.json',active); status['partial_file']='partial_observations.json'
 finally:
  status['physics_steps']=total; status['elapsed_wall_s']=time.monotonic()-started; dump(out/'collector_status.json',status)
  if app is not None: app.close()
 return 0 if status['status']=='COLLECTION_COMPLETE_NOT_EVALUATED' else 1
if __name__=='__main__': raise SystemExit(main())
