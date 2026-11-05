#!/usr/bin/env python3
"""G1-A physical proxy: geometry/motion-only control switching toward a deep wall."""
from __future__ import annotations
import argparse, hashlib, json, re, sys, time
from pathlib import Path
import numpy as np
REPO=Path(__file__).resolve().parents[3]; sys.path.insert(0,str(REPO))
from offline.task28_stage_b_contact.c2_control import json_record
from research.stage_b_contact_switch_g1a.scripts.g0r_math import momentum_ledger
CUBE='/World/Cube_01'; TABLE='/World/Table'; WALL='/World/DeepWall'

def xyz(v): return [float(v.x),float(v.y),float(v.z)]
def vec(v): return np.asarray(v,dtype=float)
def clamp(x,lo,hi): return max(lo,min(hi,x))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run-id',required=True); ap.add_argument('--controller',choices=('C0','C1','C2'),required=True); ap.add_argument('--output-root',type=Path,default=REPO/'research/stage_b_contact_switch_g1a/evidence'); ap.add_argument('--config',type=Path,default=REPO/'research/stage_b_contact_switch_g1a/configs/g1a_proxy_protocol.json'); args=ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+',args.run_id): raise ValueError('invalid run ID')
    config=json.loads(args.config.read_text()); cfg=config['controllers'][args.controller]; common=config['common']; cfg_sha=hashlib.sha256(args.config.read_bytes()).hexdigest(); script_sha=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    out=args.output_root/args.run_id; out.mkdir(parents=True,exist_ok=False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'sync_loads':True})
    rows=[]; events=[]; subs=[]; result={'status':'FAILED','reason':'not_started','experiment_class':'PHYSICAL_PROXY_ONLY','controller':args.controller,'controller_definition':cfg,'protocol_id':config['protocol_id'],'protocol_sha256':cfg_sha,'script_sha256':script_sha,'control_observation_basis':'geometry_gap_and_cube_velocity_only','fr3_execution':False,'cube_pose_writes_after_play':0}
    row_file=(out/'timeseries.jsonl').open('x'); event_file=(out/'contact_events.jsonl').open('x')
    try:
        import omni.kit.app,omni.physx,omni.timeline,omni.usd
        from pxr import Gf,PhysxSchema,PhysicsSchemaTools,Sdf,UsdGeom,UsdPhysics,UsdShade,UsdUtils
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage=omni.usd.get_context().get_stage(); UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage,1.0); UsdGeom.Xform.Define(stage,'/World')
        scene=UsdPhysics.Scene.Define(stage,'/physicsScene'); scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr().Set(9.81); phys_scene=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()); phys_scene.CreateEnableCCDAttr().Set(True); phys_scene.CreateEnableStabilizationAttr().Set(True); phys_scene.CreateSolverTypeAttr().Set('TGS')
        material=UsdShade.Material.Define(stage,'/World/Material'); mat=UsdPhysics.MaterialAPI.Apply(material.GetPrim()); mat.CreateStaticFrictionAttr().Set(.9); mat.CreateDynamicFrictionAttr().Set(.75); mat.CreateRestitutionAttr().Set(0.)
        def box(path,center,size,dynamic=False):
            b=UsdGeom.Cube.Define(stage,path); b.CreateSizeAttr().Set(1.0); x=UsdGeom.Xformable(b.GetPrim()); x.AddTranslateOp(opSuffix='g1a_pose').Set(Gf.Vec3d(*center)); x.AddScaleOp(opSuffix='g1a_size').Set(Gf.Vec3f(*size)); p=b.GetPrim(); UsdPhysics.CollisionAPI.Apply(p); UsdShade.MaterialBindingAPI.Apply(p).Bind(material)
            if dynamic: UsdPhysics.RigidBodyAPI.Apply(p); UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(.8); PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr().Set(False)
            return p
        box(CUBE,(.5,0.,.26),(.12,.12,.12),True); box(TABLE,(.55,0.,.1),(1.5,.8,.2)); box(WALL,(.70,0.,.275),(.02,.8,.15))
        table_view=RigidContactView(CUBE,[TABLE],name=f'g1a_{args.controller}_table',disable_stablization=False,max_contact_count=128); wall_view=RigidContactView(CUBE,[WALL],name=f'g1a_{args.controller}_wall',disable_stablization=False,max_contact_count=128)
        manager=omni.kit.app.get_app().get_extension_manager();
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control',True): raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc=_dynamic_control.acquire_dynamic_control_interface(); sim=omni.physx.get_physx_simulation_interface(); stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt(); cube_path=PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        state={'step':0,'time':0.,'before':None,'before_dt':None,'active_force':np.zeros(3),'next_force':np.zeros(3),'pending':[],'error':None,'mode':'APPROACH','transition_step':None,'transition_time_s':None,'target_x':float(cfg['target_center_x_m']),'current_target_x':float(cfg['target_center_x_m']),'blend_index':0,'previous_command':0.0,'target_reached_step':None}
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
            try: state['before']=xyz(dc.get_rigid_body_linear_velocity(handle)); state['before_dt']=float(dt); state['active_force']=state['next_force'].copy()
            except Exception as exc: state['error']=f'PRE_STEP:{exc!r}'
        def matrix(view,dt):
            m=np.asarray(view.get_contact_force_matrix(dt=float(dt)),dtype=float)
            if m.shape!=(1,1,3) or not np.isfinite(m).all(): raise RuntimeError('INVALID_CONTACT_MATRIX')
            return m[0,0]
        def friction(view,dt):
            ff,pts,counts,starts=view.get_friction_data(dt=float(dt)); i,n=int(np.asarray(starts)[0,0]),int(np.asarray(counts)[0,0]); f=np.asarray(ff,dtype=float)[i:i+n]; return f.sum(axis=0) if n else np.zeros(3)
        def controller_command(cp,cv,wall_gap):
            if args.controller=='C1' and state['mode']=='APPROACH' and wall_gap<=float(cfg['trigger_gap_m']) and cv[0]>=-float(cfg['trigger_speed_m_s']):
                state['mode']='GEOMETRY_HOLD'; state['transition_step']=state['step']; state['transition_time_s']=state['time']; state['target_x']=cp[0]
            elif args.controller=='C2' and state['mode']=='APPROACH' and wall_gap<=float(cfg['trigger_gap_m']) and cv[0]>=-float(cfg['trigger_speed_m_s']):
                state['mode']='BLEND'; state['transition_step']=state['step']; state['transition_time_s']=state['time']; state['blend_index']=0
            if args.controller=='C0':
                state['current_target_x']=float(cfg['target_center_x_m'])
                return float(cfg['approach_force_N'])
            if state['mode']=='APPROACH':
                state['current_target_x']=float(cfg['target_center_x_m'])
                return float(cfg['approach_force_N'])
            if args.controller=='C1':
                state['current_target_x']=state['target_x']
                return 0.0
            n=max(1,int(cfg['blend_steps'])); alpha=min(1.0,state['blend_index']/n); state['blend_index']+=1
            state['current_target_x']=(1-alpha)*float(cfg['target_center_x_m'])+alpha*(float(common['wall_left_face_x_m'])-float(common['cube_half_extent_m'])-float(common['target_gap_m']))
            if alpha>=1.0: state['mode']='REDUCED_HOLD'; state['target_x']=state['current_target_x']
            return (1-alpha)*float(cfg['approach_force_N'])
        def post_step(dt):
            try:
                state['step']+=1; state['time']+=float(dt)
                if state['before'] is None or abs(state['before_dt']-dt)>1e-8: raise RuntimeError('PRE_STEP_MISMATCH')
                pose=dc.get_rigid_body_pose(handle); vel=dc.get_rigid_body_linear_velocity(handle); cp=np.asarray(xyz(pose.p)); cv=np.asarray(xyz(vel)); gap=float(common['wall_left_face_x_m']-float(common['cube_half_extent_m'])-cp[0])
                events_step=list(state['pending']); state['pending'].clear(); wall_ev=np.zeros(3); table_ev=np.zeros(3); other_ev=np.zeros(3); wall_pair=False
                for e in events_step:
                    if e['step_at_callback']+1 != state['step']: raise RuntimeError('CONTACT_REPORT_STEP_MISMATCH')
                    imp=sum((np.asarray(p['impulse_world_Ns'],float) for p in e['points']),np.zeros(3)); sign=1 if e['colliders'][0]==CUBE else -1; imp*=sign
                    if WALL in e['colliders']: wall_ev+=imp; wall_pair=True
                    elif TABLE in e['colliders']: table_ev+=imp
                    else: other_ev+=imp
                wf=matrix(wall_view,dt); tf=matrix(table_view,dt); fr=(friction(table_view,dt)+friction(wall_view,dt))*float(dt)
                wall_loaded=abs(float(wf[0]))>.02 or np.linalg.norm(wall_ev)>.0002
                command=controller_command(cp,cv,gap); state['next_force']=np.array([command,0.,0.]); target_x=float(state['current_target_x'])
                if state['target_reached_step'] is None and abs(target_x-cp[0])<=.001: state['target_reached_step']=state['step']
                led=momentum_ledger(mass,gravity_vec,float(dt),state['before'],cv,table_ev+wall_ev,fr,state['active_force'])
                row={'step':state['step'],'simulation_time_s':state['time'],'dt_s':float(dt),'phase':state['mode'],'controller_mode':state['mode'],'cube_xyz_m':cp.tolist(),'cube_velocity_world_m_s':cv.tolist(),'cube_quaternion_xyzw':[float(pose.r.x),float(pose.r.y),float(pose.r.z),float(pose.r.w)],'cube_wall_gap_m':gap,'target_center_x_m':target_x,'commanded_force_next_world_N':state['next_force'].tolist(),'active_force_world_N':state['active_force'].tolist(),'command_discontinuity_N':abs(command-state['previous_command']),'contact_pairs':[e['colliders'] for e in events_step],'raw_wall_pair_present':wall_pair,'wall_loaded':wall_loaded,'wall_force_world_N':wf.tolist(),'wall_impulse_world_Ns':wall_ev.tolist(),'table_force_world_N':tf.tolist(),'momentum_ledger':led,'cube_sleeping':bool(sim.is_sleeping(stage_id,cube_path)),'transition_step':state['transition_step'],'transition_time_s':state['transition_time_s'],'target_reached':state['target_reached_step'] is not None,'target_reached_step':state['target_reached_step']}
                row_file.write(json_record(row)+'\n'); row_file.flush(); rows.append(row); state['previous_command']=command; state['before']=None; dc.apply_body_force(handle,(command,0.,0.),(float(cp[0]),float(cp[1]),float(cp[2])),True)
                if np.linalg.norm(cp-np.asarray(initial))>float(common['motion_displacement_max_m']) or np.linalg.norm(cv)>float(common['motion_speed_max_m_s']): raise RuntimeError('MOTION_BOUND_EXCEEDED')
            except Exception as exc: state['error']=f'POST_STEP:{exc!r}'
        timeline=omni.timeline.get_timeline_interface(); timeline.play();
        for _ in range(90): app.update()
        handle=dc.get_rigid_body(CUBE); table_view.initialize(); wall_view.initialize();
        if handle==_dynamic_control.INVALID_HANDLE or not table_view.is_physics_handle_valid() or not wall_view.is_physics_handle_valid(): raise RuntimeError('INVALID_PHYSICS_HANDLE_OR_VIEW')
        if (table_view.num_shapes,table_view.num_filters)!=(1,1) or (wall_view.num_shapes,wall_view.num_filters)!=(1,1): raise RuntimeError('INVALID_CONTACT_VIEW')
        mass=float(dc.get_rigid_body_properties(handle).mass); gravity_vec=xyz(table_view._physics_sim_view.get_gravity());
        if abs(mass-.8)>.001 or not np.allclose(gravity_vec,[0,0,-9.81],atol=.001): raise RuntimeError('MASS_OR_GRAVITY_MISMATCH')
        initial=np.asarray(xyz(dc.get_rigid_body_pose(handle).p)); dc.wake_up_rigid_body(handle)
        result.update(runtime_mass_kg=mass,gravity_world_m_s2=gravity_vec,geometry={'cube_m':[.12,.12,.12],'table_m':[1.5,.8,.2],'wall_m':[.02,.8,.15],'wall_left_face_x_m':.69},friction={'static':.9,'dynamic':.75,'restitution':0},solver=phys_scene.GetSolverTypeAttr().Get(),stabilization=phys_scene.GetEnableStabilizationAttr().Get(),initial_cube_xyz_m=initial.tolist(),physics_dt_nominal_s=1/60)
        subs.extend([sim.subscribe_contact_report_events(report),omni.physx.get_physx_interface().subscribe_physics_on_step_events(pre_step,True,0),omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step,False,0)])
        start=time.monotonic(); max_steps=int(common['trial_max_steps'])
        while len(rows)<max_steps and not state['error'] and time.monotonic()-start<180: app.update()
        timeline.pause()
        if state['error']: raise RuntimeError(state['error'])
        last=rows[-1] if rows else {}
        final_gap=float(last.get('cube_wall_gap_m',999)); loaded=any(r['wall_loaded'] for r in rows); force_linf=max((max(abs(x) for x in r['momentum_ledger']['unexplained_impulse_Ns']) for r in rows),default=0.0)
        result.update(status='RECORDED',reason=None,samples=len(rows),transition_step=state['transition_step'],transition_time_s=state['transition_time_s'],transition_observed=state['transition_step'] is not None,final_cube_wall_gap_m=final_gap,geometric_acceptance=bool(-float(common['penetration_limit_m'])<=final_gap<=float(common['gap_acceptance_m'])),no_undocumented_penetration=bool(final_gap>=-float(common['penetration_limit_m'])),loaded_wall_contact=loaded,wall_contact_steps=sum(r['wall_loaded'] for r in rows),max_wall_force_N=max((abs(r['wall_force_world_N'][0]) for r in rows),default=0.0),max_command_discontinuity_N=max((r['command_discontinuity_N'] for r in rows),default=0.0),max_command_discontinuity_after_initial_N=max((r['command_discontinuity_N'] for r in rows[1:]),default=0.0),transition_command_discontinuity_max_N=max((r['command_discontinuity_N'] for r in rows if state['transition_step'] is not None and r['step']>=state['transition_step']),default=0.0),tracking_error_final_m=abs(float(cfg['target_center_x_m'])-float(last.get('cube_xyz_m',[0])[0])) if last else None,target_reached=bool(state['target_reached_step'] is not None),target_reached_step=state['target_reached_step'],max_corrected_residual_linf_Ns=force_linf,execution_time_s=float(last.get('simulation_time_s',0.0)),mode_sequence=list(dict.fromkeys(r['controller_mode'] for r in rows)))
    except Exception as exc: result['reason']=repr(exc); result['samples']=len(rows); result['events']=len(events)
    finally:
        for sub in subs:
            try: sub.unsubscribe()
            except Exception: pass
        row_file.close(); event_file.close(); result.setdefault('samples',len(rows)); result.setdefault('events',len(events)); (out/'summary.json').write_text(json_record(result)+'\n'); print('G1A_PROXY',json_record(result),flush=True); app.close()
    return 0 if result.get('status')=='RECORDED' else 1
if __name__=='__main__': raise SystemExit(main())
