#!/usr/bin/env python3
"""Fresh minimal Cube/table/deep-wall probe with physical force excitation.
No Cube pose or velocity is written after play; only a bounded world-frame force is applied.
"""
from __future__ import annotations
import argparse, json, re, sys, time
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.c2_control import json_record
from offline.task28_stage_b_contact.d1_math import balance

CUBE='/World/Cube_01'; TABLE='/World/Table'; WALL='/World/DeepWall'

def xyz(v): return [float(v.x), float(v.y), float(v.z)]

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run-id',required=True); ap.add_argument('--output-root',type=Path,default=ROOT/'research/stage_b_contact_switch_g0/evidence'); args=ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.run_id): raise ValueError('invalid run ID')
    out=args.output_root/args.run_id; out.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'sync_loads':True})
    rows=[]; events=[]; subs=[]; result={'status':'FAILED','reason':'not_started','scene':'minimal Cube/table/deep-wall','cube_pose_writes_after_play':0,'applied_force_mode':'bounded world +X excitation'}
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
            b=UsdGeom.Cube.Define(stage,path); b.CreateSizeAttr().Set(1.0); x=UsdGeom.Xformable(b.GetPrim()); x.AddTranslateOp(opSuffix='g0_pose').Set(Gf.Vec3d(*center)); x.AddScaleOp(opSuffix='g0_size').Set(Gf.Vec3f(*size)); p=b.GetPrim(); UsdPhysics.CollisionAPI.Apply(p); UsdShade.MaterialBindingAPI.Apply(p).Bind(material)
            if dynamic:
                UsdPhysics.RigidBodyAPI.Apply(p); UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(.8); PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr().Set(False)
            return p
        cube=box(CUBE,(.5,0.,.26),(.12,.12,.12),True); table=box(TABLE,(.55,0.,.1),(1.5,.8,.2)); wall=box(WALL,(.70,0.,.275),(.02,.8,.15))
        cube_phys=PhysxSchema.PhysxRigidBodyAPI(cube)
        table_view=RigidContactView(CUBE,[TABLE],name='g0_cube_table',disable_stablization=False,max_contact_count=128)
        wall_view=RigidContactView(CUBE,[WALL],name='g0_cube_wall',disable_stablization=False,max_contact_count=128)
        manager=omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control',True): raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc=_dynamic_control.acquire_dynamic_control_interface(); sim=omni.physx.get_physx_simulation_interface(); stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt(); cube_path=PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        state={'step':0,'time':0.0,'frame':0,'error':None,'before':None,'before_dt':None,'pending':[],'phase':'PRECONTACT','loaded_step':None,'withdraw_step':None,'done_count':0,'cumulative':np.zeros(3)}
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
            try: state['before']=xyz(dc.get_rigid_body_linear_velocity(handle)); state['before_dt']=float(dt)
            except Exception as exc: state['error']=f'PRE_STEP:{exc!r}'
        def matrix(view,dt):
            m=np.asarray(view.get_contact_force_matrix(dt=float(dt)),dtype=float)
            if m.shape!=(1,1,3) or not np.isfinite(m).all(): raise RuntimeError('INVALID_CONTACT_MATRIX')
            return m[0,0]
        def friction(view,dt):
            ff,pts,counts,starts=view.get_friction_data(dt=float(dt)); i,n=int(np.asarray(starts)[0,0]),int(np.asarray(counts)[0,0]); f=np.asarray(ff,dtype=float)[i:i+n]; return f.sum(axis=0) if n else np.zeros(3), np.asarray(pts,dtype=float)[i:i+n].tolist()
        def post_step(dt):
            try:
                state['step']+=1; state['time']+=float(dt); state['frame']+=1
                if state['before'] is None or abs(state['before_dt']-dt)>1e-8: raise RuntimeError('PRE_STEP_MISMATCH')
                cube_p=dc.get_rigid_body_pose(handle); cube_v=dc.get_rigid_body_linear_velocity(handle); cp=xyz(cube_p.p); cv=xyz(cube_v)
                wf=matrix(wall_view,dt); tf=matrix(table_view,dt); wi=np.asarray(wall_view.get_contact_force_matrix(dt=1.),dtype=float)[0,0]; ti=np.asarray(table_view.get_contact_force_matrix(dt=1.),dtype=float)[0,0]
                events_step=list(state['pending']); state['pending'].clear(); wall_ev=np.zeros(3); table_ev=np.zeros(3); other_ev=np.zeros(3)
                for e in events_step:
                    imp=sum((np.asarray(p['impulse_world_Ns'],float) for p in e['points']),np.zeros(3)); sign=1 if e['colliders'][0]==CUBE else -1; imp*=sign
                    if WALL in e['colliders']: wall_ev+=imp
                    elif TABLE in e['colliders']: table_ev+=imp
                    else: other_ev+=imp
                if state['phase']=='PRECONTACT' and state['time']>=.5: state['phase']='EXCITE'
                wall_loaded=abs(float(wf[0]))>.02 or np.linalg.norm(wall_ev)>.0002
                if state['phase']=='EXCITE' and wall_loaded: state['phase']='LOADED'; state['loaded_step']=state['step']
                if state['phase']=='LOADED' and state['time']-state['loaded_step']*float(dt)>.6: state['phase']='WITHDRAW'
                if state['phase']=='WITHDRAW' and not wall_loaded and state['step']-(state['withdraw_step'] or state['step'])>10: state['phase']='DONE'
                if state['phase']=='WITHDRAW' and state['withdraw_step'] is None: state['withdraw_step']=state['step']
                fr_t,pts_t=friction(table_view,dt); fr_w,pts_w=friction(wall_view,dt); fr=fr_t+fr_w
                accounting=balance(mass,gravity_vec,float(dt),state['before'],cv,table_ev+wall_ev,fr*float(dt)); state['cumulative']+=np.asarray(accounting['unexplained_impulse_Ns'])
                sleeping=bool(sim.is_sleeping(stage_id,cube_path)); row={'step':state['step'],'simulation_time_s':state['time'],'dt_s':float(dt),'wall_ns':time.time_ns(),'phase':state['phase'],'cube_xyz_m':cp,'pre_velocity_world_m_s':state['before'],'velocity_world_m_s':cv,'cube_quaternion_xyzw':[float(cube_p.r.x),float(cube_p.r.y),float(cube_p.r.z),float(cube_p.r.w)],'cube_sleeping':sleeping,'cube_table_force_world_N':tf.tolist(),'cube_wall_force_world_N':wf.tolist(),'cube_table_impulse_world_Ns':ti.tolist(),'cube_wall_impulse_world_Ns':wi.tolist(),'event_table_impulse_world_Ns':table_ev.tolist(),'event_wall_impulse_world_Ns':wall_ev.tolist(),'event_other_impulse_world_Ns':other_ev.tolist(),'table_friction_force_world_N':fr_t.tolist(),'wall_friction_force_world_N':fr_w.tolist(),'table_friction_points_world_m':pts_t,'wall_friction_points_world_m':pts_w,'contact_pairs':[e['colliders'] for e in events_step],'contact_event_count':len(events_step),'wall_loaded':wall_loaded,'applied_force_world_N':None,**accounting,'cumulative_unexplained_impulse_Ns':state['cumulative'].tolist()}
                # Force is applied only through the PhysX API after the sample; no pose/velocity teleport.
                if state['phase']=='EXCITE': applied=8.0
                elif state['phase']=='LOADED': applied=1.0
                elif state['phase']=='WITHDRAW': applied=-12.0
                else: applied=0.0
                row['applied_force_world_N']=[applied,0.,0.]; row['wall_gap_m']=float(.69-(cp[0]+.06)); row['cube_table_contact']=bool(np.linalg.norm(tf)>.02); row['cube_wall_contact']=wall_loaded
                row_file.write(json_record(row)+'\n'); row_file.flush(); rows.append(row); state['before']=None
                dc.apply_body_force(handle,(applied,0.,0.),(float(cp[0]),float(cp[1]),float(cp[2])),True)
                if np.linalg.norm(np.asarray(cp)-np.asarray(initial))>.30 or np.linalg.norm(np.asarray(cv))>.8: raise RuntimeError('MOTION_BOUND_EXCEEDED')
            except Exception as exc: state['error']=f'POST_STEP:{exc!r}'
        timeline=omni.timeline.get_timeline_interface(); timeline.play()
        for _ in range(90): app.update()
        handle=dc.get_rigid_body(CUBE)
        if handle==_dynamic_control.INVALID_HANDLE: raise RuntimeError('INVALID_CUBE_HANDLE')
        table_view.initialize(); wall_view.initialize();
        if any(not v.is_physics_handle_valid() or (v.num_shapes,v.num_filters)!=(1,1) for v in (table_view,wall_view)): raise RuntimeError('INVALID_CONTACT_VIEW')
        mass=float(dc.get_rigid_body_properties(handle).mass); gravity_vec=xyz(table_view._physics_sim_view.get_gravity()); gravity=float(np.linalg.norm(gravity_vec));
        if abs(mass-.8)>.001 or not np.allclose(gravity_vec,[0,0,-9.81],atol=.001): raise RuntimeError('MASS_OR_GRAVITY_MISMATCH')
        initial=dc.get_rigid_body_pose(handle).p; dc.wake_up_rigid_body(handle)
        result.update(runtime_mass_kg=mass,gravity_world_m_s2=gravity_vec,gravity_m_s2=gravity,geometry={'cube_m':[.12,.12,.12],'table_m':[1.5,.8,.2],'wall_m':[.02,.8,.15],'wall_left_face_x_m':.69},friction={'static':.9,'dynamic':.75,'restitution':0},solver=phys_scene.GetSolverTypeAttr().Get(),stabilization=phys_scene.GetEnableStabilizationAttr().Get(),initial_cube_xyz_m=xyz(initial),physics_dt_nominal_s=1/60)
        subs.extend([sim.subscribe_contact_report_events(report),omni.physx.get_physx_interface().subscribe_physics_on_step_events(pre_step,True,0),omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step,False,0)])
        start=time.monotonic()
        while len(rows)<300 and state['phase']!='DONE' and not state['error'] and time.monotonic()-start<180: app.update()
        timeline.pause()
        if state['error']: raise RuntimeError(state['error'])
        wall_forces=[abs(r['cube_wall_force_world_N'][0]) for r in rows]; wall_loaded=any(r['wall_loaded'] for r in rows); withdrew=any(r['phase']=='WITHDRAW' for r in rows)
        result.update(status='RECORDED',reason=None,samples=len(rows),wall_loaded=wall_loaded,withdrawal_started=withdrew,done_phase=state['phase'],first_loaded_step=next((r['step'] for r in rows if r['wall_loaded']),None),peak_wall_normal_force_N=max(wall_forces,default=0.0),max_cube_displacement_m=max((np.linalg.norm(np.asarray(r['cube_xyz_m'])-np.asarray(rows[0]['cube_xyz_m'])) for r in rows),default=0.0),sleeping_steps=sum(bool(r['cube_sleeping']) for r in rows))
    except Exception as exc: result['reason']=repr(exc); result['samples']=len(rows); result['events']=len(events)
    finally:
        for sub in subs:
            try: sub.unsubscribe()
            except Exception: pass
        row_file.close(); event_file.close(); result.setdefault('samples',len(rows)); result.setdefault('events',len(events)); (out/'summary.json').write_text(json_record(result)+'\n'); print('G0_C2_WALL',json_record(result),flush=True); app.close()
    return 0 if result.get('status')=='RECORDED' else 1
if __name__=='__main__': raise SystemExit(main())
