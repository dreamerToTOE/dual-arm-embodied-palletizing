#!/usr/bin/env python3
"""G0-R DeepWall probe with explicit active-command momentum accounting."""
from __future__ import annotations
import argparse, hashlib, json, re, sys, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
from offline.task28_stage_b_contact.c2_control import json_record
from research.stage_b_contact_switch_g1a.scripts.g0r_math import momentum_ledger

CUBE='/World/Cube_01'; TABLE='/World/Table'; WALL='/World/DeepWall'

def xyz(v): return [float(v.x), float(v.y), float(v.z)]

def vec(v): return np.asarray(v, dtype=float)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run-id',required=True); ap.add_argument('--output-root',type=Path,default=REPO/'research/stage_b_contact_switch_g1a/evidence'); ap.add_argument('--config',type=Path,default=REPO/'research/stage_b_contact_switch_g1a/configs/g0r_c2_protocol.json'); args=ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.run_id): raise ValueError('invalid run ID')
    config=json.loads(args.config.read_text()); config_sha=hashlib.sha256(args.config.read_bytes()).hexdigest(); script_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out=args.output_root/args.run_id; out.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'sync_loads':True})
    rows=[]; events=[]; subs=[]
    result={'status':'FAILED','reason':'not_started','scene':'minimal Cube/table/deep-wall','cube_pose_writes_after_play':0,'command_semantics':'force selected after post-step sample is active during the following physics interval','protocol_id':config['protocol_id'],'protocol_sha256':config_sha,'script_sha256':script_sha,'config_path':str(args.config)}
    row_file=(out/'timeseries.jsonl').open('x'); event_file=(out/'contact_events.jsonl').open('x')
    try:
        import omni.kit.app, omni.physx, omni.timeline, omni.usd
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdShade, UsdUtils
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage=omni.usd.get_context().get_stage(); UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage,1.0); UsdGeom.Xform.Define(stage,'/World')
        scene=UsdPhysics.Scene.Define(stage,'/physicsScene'); scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr().Set(9.81)
        phys_scene=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()); phys_scene.CreateEnableCCDAttr().Set(True); phys_scene.CreateEnableStabilizationAttr().Set(True); phys_scene.CreateSolverTypeAttr().Set('TGS')
        material=UsdShade.Material.Define(stage,'/World/Material'); mat=UsdPhysics.MaterialAPI.Apply(material.GetPrim()); mat.CreateStaticFrictionAttr().Set(.9); mat.CreateDynamicFrictionAttr().Set(.75); mat.CreateRestitutionAttr().Set(0.)
        def box(path,center,size,dynamic=False):
            b=UsdGeom.Cube.Define(stage,path); b.CreateSizeAttr().Set(1.0); x=UsdGeom.Xformable(b.GetPrim()); x.AddTranslateOp(opSuffix='g0r_pose').Set(Gf.Vec3d(*center)); x.AddScaleOp(opSuffix='g0r_size').Set(Gf.Vec3f(*size)); p=b.GetPrim(); UsdPhysics.CollisionAPI.Apply(p); UsdShade.MaterialBindingAPI.Apply(p).Bind(material)
            if dynamic:
                UsdPhysics.RigidBodyAPI.Apply(p); UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(.8); PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr().Set(False)
            return p
        cube=box(CUBE,(.5,0.,.26),(.12,.12,.12),True); box(TABLE,(.55,0.,.1),(1.5,.8,.2)); box(WALL,(.70,0.,.275),(.02,.8,.15))
        table_view=RigidContactView(CUBE,[TABLE],name='g0r_cube_table',disable_stablization=False,max_contact_count=128)
        wall_view=RigidContactView(CUBE,[WALL],name='g0r_cube_wall',disable_stablization=False,max_contact_count=128)
        manager=omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control',True): raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc=_dynamic_control.acquire_dynamic_control_interface(); sim=omni.physx.get_physx_simulation_interface(); stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt(); cube_path=PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        zero=np.zeros(3)
        sched=config['force_schedule']; acceptance=config['acceptance']
        state={'step':0,'time':0.0,'error':None,'before':None,'before_dt':None,'active_force':zero.copy(),'active_dt':None,'next_force':zero.copy(),'pending':[],'phase':'PRECONTACT','loaded_step':None,'loaded_time_s':None,'withdraw_step':None,'withdraw_issued':0,'separation_count':0,'cumulative':zero.copy(),'event_step_ok':True}
        def report(headers,data):
            try:
                for h in headers:
                    paths=[str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0,h.collider1)]
                    if CUBE not in paths: continue
                    points=[]
                    for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
                        d=data[i]; points.append({'normal_world':list(d.normal),'impulse_world_Ns':list(d.impulse),'position_world_m':list(d.position),'separation_m':float(d.separation)})
                    ev={'step_at_callback':state['step'],'time_at_callback_s':state['time'],'wall_ns':time.time_ns(),'colliders':paths,'points':points}; state['pending'].append(ev); events.append(ev); event_file.write(json_record(ev)+'\n'); event_file.flush()
            except Exception as exc: state['error']=f'CONTACT_EVENT:{exc!r}'
        def pre_step(dt):
            try:
                state['before']=xyz(dc.get_rigid_body_linear_velocity(handle)); state['before_dt']=float(dt); state['active_dt']=float(dt); state['active_force']=state['next_force'].copy()
            except Exception as exc: state['error']=f'PRE_STEP:{exc!r}'
        def matrix(view,dt):
            m=np.asarray(view.get_contact_force_matrix(dt=float(dt)),dtype=float)
            if m.shape!=(1,1,3) or not np.isfinite(m).all(): raise RuntimeError('INVALID_CONTACT_MATRIX')
            return m[0,0]
        def friction(view,dt):
            ff,pts,counts,starts=view.get_friction_data(dt=float(dt)); i,n=int(np.asarray(starts)[0,0]),int(np.asarray(counts)[0,0]); f=np.asarray(ff,dtype=float)[i:i+n]; return f.sum(axis=0) if n else np.zeros(3), np.asarray(pts,dtype=float)[i:i+n].tolist()
        def choose_next_command():
            if state['phase']=='EXCITE': return vec(sched['excite_force_world_N'])
            if state['phase']=='LOADED': return vec(sched['loaded_hold_force_world_N'])
            if state['phase']=='WITHDRAW' and state['withdraw_issued'] < int(sched['withdraw_pulse_steps']):
                state['withdraw_issued'] += 1; return vec(sched['withdraw_pulse_force_world_N'])
            return vec(sched['post_withdraw_force_world_N'])
        def post_step(dt):
            try:
                state['step']+=1; state['time']+=float(dt)
                if state['before'] is None or abs(state['before_dt']-dt)>1e-8: raise RuntimeError('PRE_STEP_MISMATCH')
                cube_p=dc.get_rigid_body_pose(handle); cube_v=dc.get_rigid_body_linear_velocity(handle); cp=xyz(cube_p.p); cv=xyz(cube_v)
                wf=matrix(wall_view,dt); tf=matrix(table_view,dt); wi=np.asarray(wall_view.get_contact_force_matrix(dt=1.),dtype=float)[0,0]; ti=np.asarray(table_view.get_contact_force_matrix(dt=1.),dtype=float)[0,0]
                events_step=list(state['pending']); state['pending'].clear(); wall_ev=np.zeros(3); table_ev=np.zeros(3); other_ev=np.zeros(3)
                for e in events_step:
                    if e['step_at_callback']+1 != state['step']: state['event_step_ok']=False
                    imp=sum((np.asarray(p['impulse_world_Ns'],float) for p in e['points']),np.zeros(3)); sign=1 if e['colliders'][0]==CUBE else -1; imp*=sign
                    if WALL in e['colliders']: wall_ev+=imp
                    elif TABLE in e['colliders']: table_ev+=imp
                    else: other_ev+=imp
                if state['phase']=='PRECONTACT' and state['time']>=float(sched['precontact_until_s']): state['phase']='EXCITE'
                wall_loaded=abs(float(wf[0]))>float(acceptance['loaded_wall_force_min_N']) or np.linalg.norm(wall_ev)>.0002
                if state['phase']=='EXCITE' and wall_loaded: state['phase']='LOADED'; state['loaded_step']=state['step']; state['loaded_time_s']=state['time']
                if state['phase']=='LOADED' and state['time']-float(state['loaded_time_s'])>=float(sched['loaded_hold_s']): state['phase']='WITHDRAW'; state['withdraw_step']=state['step']+1
                if state['phase']=='WITHDRAW' and state['withdraw_issued']>=int(sched['withdraw_pulse_steps']): state['phase']='POST_WITHDRAW'
                if state['phase']=='POST_WITHDRAW':
                    if not wall_loaded and float(.69-(cp[0]+.06))>float(acceptance['separation_gap_min_m']): state['separation_count']+=1
                    else: state['separation_count']=0
                    if state['separation_count']>=int(sched['separation_confirmation_steps']): state['phase']='DONE'
                fr_t,pts_t=friction(table_view,dt); fr_w,pts_w=friction(wall_view,dt); fr=(fr_t+fr_w)*float(dt)
                corrected=momentum_ledger(mass,gravity_vec,float(dt),state['before'],cv,table_ev+wall_ev,fr,state['active_force'])
                uncorrected=momentum_ledger(mass,gravity_vec,float(dt),state['before'],cv,table_ev+wall_ev,fr,zero)
                state['cumulative']+=np.asarray(corrected['unexplained_impulse_Ns'])
                sleeping=bool(sim.is_sleeping(stage_id,cube_path)); next_force=choose_next_command(); state['next_force']=next_force
                row={'step':state['step'],'simulation_time_s':state['time'],'dt_s':float(dt),'wall_ns':time.time_ns(),'phase':state['phase'],'cube_xyz_m':cp,'pre_velocity_world_m_s':state['before'],'velocity_world_m_s':cv,'cube_quaternion_xyzw':[float(cube_p.r.x),float(cube_p.r.y),float(cube_p.r.z),float(cube_p.r.w)],'cube_sleeping':sleeping,'cube_table_force_world_N':tf.tolist(),'cube_wall_force_world_N':wf.tolist(),'cube_table_impulse_world_Ns':ti.tolist(),'cube_wall_impulse_world_Ns':wi.tolist(),'event_table_impulse_world_Ns':table_ev.tolist(),'event_wall_impulse_world_Ns':wall_ev.tolist(),'event_other_impulse_world_Ns':other_ev.tolist(),'table_friction_force_world_N':fr_t.tolist(),'wall_friction_force_world_N':fr_w.tolist(),'table_friction_points_world_m':pts_t,'wall_friction_points_world_m':pts_w,'contact_pairs':[e['colliders'] for e in events_step],'contact_event_count':len(events_step),'wall_loaded':wall_loaded,'active_force_world_N':state['active_force'].tolist(),'active_force_dt_s':float(state['active_dt']),'active_force_impulse_world_Ns':(state['active_force']*float(dt)).tolist(),'next_commanded_force_world_N':next_force.tolist(),'event_step_alignment_ok':state['event_step_ok'],'separation_confirmation_count':state['separation_count'],**corrected,'uncorrected_unexplained_impulse_Ns':uncorrected['unexplained_impulse_Ns'],'cumulative_unexplained_impulse_Ns':state['cumulative'].tolist(),'wall_gap_m':float(.69-(cp[0]+.06)),'cube_table_contact':bool(np.linalg.norm(tf)>.02),'cube_wall_contact':wall_loaded}
                row_file.write(json_record(row)+'\n'); row_file.flush(); rows.append(row); state['before']=None
                dc.apply_body_force(handle,tuple(next_force.tolist()),(float(cp[0]),float(cp[1]),float(cp[2])),True)
                if np.linalg.norm(np.asarray(cp)-np.asarray(initial))>float(acceptance['motion_displacement_max_m']) or np.linalg.norm(np.asarray(cv))>float(acceptance['motion_speed_max_m_s']): raise RuntimeError('MOTION_BOUND_EXCEEDED')
            except Exception as exc: state['error']=f'POST_STEP:{exc!r}'
        timeline=omni.timeline.get_timeline_interface(); timeline.play()
        for _ in range(90): app.update()
        handle=dc.get_rigid_body(CUBE)
        if handle==_dynamic_control.INVALID_HANDLE: raise RuntimeError('INVALID_CUBE_HANDLE')
        table_view.initialize(); wall_view.initialize()
        if any(not v.is_physics_handle_valid() or (v.num_shapes,v.num_filters)!=(1,1) for v in (table_view,wall_view)): raise RuntimeError('INVALID_CONTACT_VIEW')
        mass=float(dc.get_rigid_body_properties(handle).mass); gravity_vec=xyz(table_view._physics_sim_view.get_gravity()); gravity=float(np.linalg.norm(gravity_vec))
        if abs(mass-.8)>.001 or not np.allclose(gravity_vec,[0,0,-9.81],atol=.001): raise RuntimeError('MASS_OR_GRAVITY_MISMATCH')
        initial=dc.get_rigid_body_pose(handle).p; dc.wake_up_rigid_body(handle)
        result.update(runtime_mass_kg=mass,gravity_world_m_s2=gravity_vec,gravity_m_s2=gravity,geometry={'cube_m':[.12,.12,.12],'table_m':[1.5,.8,.2],'wall_m':[.02,.8,.15],'wall_left_face_x_m':.69},friction={'static':.9,'dynamic':.75,'restitution':0},solver=phys_scene.GetSolverTypeAttr().Get(),stabilization=phys_scene.GetEnableStabilizationAttr().Get(),initial_cube_xyz_m=xyz(initial),physics_dt_nominal_s=1/60,force_schedule=sched,acceptance=acceptance)
        subs.extend([sim.subscribe_contact_report_events(report),omni.physx.get_physx_interface().subscribe_physics_on_step_events(pre_step,True,0),omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step,False,0)])
        start=time.monotonic()
        while len(rows)<360 and state['phase']!='DONE' and not state['error'] and time.monotonic()-start<180: app.update()
        timeline.pause()
        if state['error']: raise RuntimeError(state['error'])
        wall_forces=[abs(r['cube_wall_force_world_N'][0]) for r in rows]; loaded_rows=[r for r in rows if r['wall_loaded']]; separated_rows=[r for r in rows if r['phase'] in ('POST_WITHDRAW','DONE') and not r['wall_loaded']]
        corrected_max=max((max(abs(x) for x in r['unexplained_impulse_Ns']) for r in rows),default=0.0); uncorrected_max=max((max(abs(x) for x in r['uncorrected_unexplained_impulse_Ns']) for r in rows),default=0.0)
        result.update(status='RECORDED',reason=None,samples=len(rows),wall_loaded=bool(loaded_rows),withdrawal_started=any(r['phase'] in ('WITHDRAW','POST_WITHDRAW','DONE') for r in rows),done_phase=state['phase'],first_loaded_step=loaded_rows[0]['step'] if loaded_rows else None,last_loaded_step=loaded_rows[-1]['step'] if loaded_rows else None,first_separated_step=separated_rows[0]['step'] if separated_rows else None,peak_wall_normal_force_N=max(wall_forces,default=0.0),max_cube_displacement_m=max((np.linalg.norm(np.asarray(r['cube_xyz_m'])-np.asarray(rows[0]['cube_xyz_m'])) for r in rows),default=0.0),sleeping_steps=sum(bool(r['cube_sleeping']) for r in rows),corrected_residual_linf_Ns=corrected_max,uncorrected_residual_linf_Ns=uncorrected_max,event_step_alignment_ok=state['event_step_ok'],separation_samples=len(separated_rows),contact_events_after_separation=sum(1 for r in separated_rows if np.linalg.norm(np.asarray(r['event_wall_impulse_world_Ns']))>.0002),raw_wall_pair_steps_after_separation=sum(1 for r in separated_rows if any(WALL in pair for pair in r['contact_pairs'])),loaded_contact_acceptance=bool(loaded_rows and min(r['wall_gap_m'] for r in loaded_rows)<=float(acceptance['loaded_gap_max_m']) and max(abs(r['cube_wall_force_world_N'][0]) for r in loaded_rows)>float(acceptance['loaded_wall_force_min_N'])),unloaded_contact_acceptance=bool(len(separated_rows)>=int(sched['separation_confirmation_steps']) and max((abs(r['cube_wall_force_world_N'][0]) for r in separated_rows),default=999)>-1 and all(abs(r['cube_wall_force_world_N'][0])<=float(acceptance['unloaded_wall_force_max_N']) and np.linalg.norm(np.asarray(r['event_wall_impulse_world_Ns']))<=.0002 for r in separated_rows)))
    except Exception as exc: result['reason']=repr(exc); result['samples']=len(rows); result['events']=len(events)
    finally:
        for sub in subs:
            try: sub.unsubscribe()
            except Exception: pass
        row_file.close(); event_file.close(); result.setdefault('samples',len(rows)); result.setdefault('events',len(events)); (out/'summary.json').write_text(json_record(result)+'\n'); print('G0R_C2_WALL',json_record(result),flush=True); app.close()
    return 0 if result.get('status')=='RECORDED' else 1
if __name__=='__main__': raise SystemExit(main())
