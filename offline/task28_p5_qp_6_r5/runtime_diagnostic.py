"""Bounded original-scene source diagnostics, one pre-registered variation.

No robot/suction command; graphs disabled only in fresh anonymous stage. All
frame values are copied and journaled immediately, before further physics or
further reads of the native tensor buffers.
"""
import argparse,builtins,hashlib,importlib.util,json,os,traceback
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r2.common import FR3_ASSET,FR3_SHA256,FIXTURE,TASK_ROOT,LEFT_ROOT,RIGHT_ROOT,CUBE_PATH,DOF_NAMES,sha256
from offline.task28_p5_qp_6_r2.runtime_probe import _runtime_identity
HERE=Path(__file__).resolve().parent

def write(path,v):
    def conv(x):
        if isinstance(x,np.ndarray):return x.tolist()
        if isinstance(x,np.generic):return x.item()
        raise TypeError(str(type(x)))
    path.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False,default=conv)+'\n')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--configuration',choices=list('ABCDE'),required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True); cfg=json.loads((HERE/'source_contract.json').read_text());reg=json.loads((HERE/'registrations.json').read_text())[a.configuration]; app=None; subs=[]; count=[]; callback=[];rows=0;retained={};last=None;stale=0
    result={'status':'REJECTED','phase':'BOOT','first_failure':None,'configuration':a.configuration,'registration':reg,'explicit_physics_steps':0,'robot_command_writes':0,'suction_commands':0,'ros_graph_suppressed':False,'result_written_before_close':True}
    def record(row):
        nonlocal rows
        with (a.output/'frames.jsonl').open('a') as f:
            f.write(json.dumps(row,sort_keys=True,allow_nan=False)+'\n');f.flush()
        rows+=1
    def save():
        result['callback_dts_s']=count.copy();result['recorded_frames']=rows;result['post_callback_count']=len(callback)
        write(a.output/'result.json',result)
    os.environ['TASK28_FR3_USD']=str(FR3_ASSET);os.environ['TASK28_FR3_USD_SHA256']=FR3_SHA256
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        import omni.usd,omni.timeline,omni.physx,omni.kit.app
        from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema
        from scipy.spatial.transform import Rotation
        from isaacsim.core.api.simulation_context import SimulationContext
        from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
        from offline.task28_g1_1.task28_validator.model import RobotModel,DEFAULT_FIXTURE
        from offline.task28_g1_1.task28_validator.collision import CollisionValidator
        from offline.task28_g1_1.task28_validator.se3 import residual
        if a.configuration in 'BE':
            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('omni.isaac.dynamic_control',True)
            from omni.isaac.dynamic_control import _dynamic_control
            dc=_dynamic_control.acquire_dynamic_control_interface()
        ctx=omni.usd.get_context();ctx.new_stage();stage=ctx.get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.0);UsdGeom.Xform.Define(stage,'/World')
        spec=importlib.util.spec_from_file_location('frozen_r5_scene',FIXTURE/'scene/task28_single_cube_scene.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);result['phase']='BUILD';mod.build_scene()
        result['scene_before']=_runtime_identity(stage,mod)
        if result['scene_before']['status']!='SCENE_IDENTITY_VERIFIED' or sha256(FR3_ASSET)!=FR3_SHA256:raise RuntimeError('SCENE_IDENTITY')
        def physics_config():
            p=stage.GetPrimAtPath(TASK_ROOT+'/physicsScene');names=['physxScene:timeStepsPerSecond','physxScene:solverType','physxScene:enableCCD','physxScene:enableStabilization','physics:gravityMagnitude','physxScene:broadphaseType','physxScene:enableGPUDynamics','physxScene:enableExternalForcesEveryIteration','physxScene:minPositionIterationCount','physxScene:maxPositionIterationCount','physxScene:minVelocityIterationCount','physxScene:maxVelocityIterationCount'];return {n:str(p.GetAttribute(n).Get()) if p.GetAttribute(n).IsValid() else None for n in names}
        result['physics_before_context']=physics_config();result['backend_requested']='torch/cuda' if a.configuration=='D' else 'numpy/cpu';result['non_baseline_diagnostic']=a.configuration in 'CD';result['cube_mass_kg']=float(UsdPhysics.MassAPI(stage.GetPrimAtPath(CUBE_PATH)).GetMassAttr().Get())
        if abs(result['cube_mass_kg']-.8)>1e-6:raise RuntimeError('CUBE_MASS')
        graph=stage.GetPrimAtPath(TASK_ROOT+'/ROSGraph');
        if not graph.IsValid() or not graph.IsActive():raise RuntimeError('ROS_GRAPH_IDENTITY')
        graph.SetActive(False);result['ros_graph_suppressed']=not graph.IsActive();result['graph_suppression']='fresh anonymous stage only; original builder still loads ROS2 bridge extension; no bridge/node/command graph intentionally run'
        if a.configuration=='C':
            scene_api=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(TASK_ROOT+'/physicsScene'));attr=scene_api.GetEnableExternalForcesEveryIterationAttr();result['tgs_force_flag_supported']=bool(attr.IsValid());result['tgs_force_flag_before']=str(attr.Get());
            if not attr.IsValid(): raise RuntimeError('TGS_FORCE_FLAG_UNAVAILABLE')
            attr.Set(True);result['tgs_force_flag_set_before_context']=True
        if getattr(builtins,'_task28_g1_0_single_cube_bridge',None) is not None:raise RuntimeError('POSITION_BRIDGE_PRESENT')
        model=RobotModel();collision=CollisionValidator(DEFAULT_FIXTURE)
        result['phase']='CONTEXT';sim=SimulationContext(physics_prim_path=TASK_ROOT+'/physicsScene',set_defaults=False,backend='torch' if a.configuration=='D' else 'numpy',device='cuda' if a.configuration=='D' else 'cpu')
        result['physics_after_context']=physics_config(); scenes=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)];result['physics_scenes']=scenes
        if scenes!=[TASK_ROOT+'/physicsScene']:raise RuntimeError('PHYSICS_SCENE_COUNT')
        before=result['physics_before_context'];after=result['physics_after_context'];
        if any(before[k]!=after[k] for k in ['physxScene:timeStepsPerSecond','physxScene:solverType','physxScene:enableCCD','physxScene:enableStabilization','physics:gravityMagnitude']):raise RuntimeError('PHYSICS_CONFIGURATION_CHANGED')
        result['physics_backend_adaptation']={
            'observed': {'before': before, 'after': after},
            'source':'installed isaacsim.core.api PhysicsContext device branch',
            'scene_source_unchanged':True,
        }
        if a.configuration=='C':
            result['tgs_force_flag_after_context']=str(PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(TASK_ROOT+'/physicsScene')).GetEnableExternalForcesEveryIterationAttr().Get())
        result['effective_backend']='GPU_CUDA' if a.configuration=='D' else 'CPU_NUMPY'
        result['configured_dt_s']=float(sim.get_physics_dt())
        if abs(result['configured_dt_s']-cfg['physics_dt_s'])>cfg['dt_tolerance_s']:raise RuntimeError('PHYSICS_DT_CONFIGURATION')
        subs.append(omni.physx.get_physx_interface().subscribe_physics_step_events(lambda d:count.append(float(d))) )
        result['phase']='NATIVE_DRIVES_PLAY';sim.play();arms={};links={};views={}
        for side,path in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]:
            arm=SingleArticulation(path);arm.initialize();arms[side]=arm;views[side]=arm._articulation_view
            if not arm.handles_initialized or list(arm.dof_names)!=DOF_NAMES:raise RuntimeError('DOF_IDENTITY:'+side)
            links[side]={}
            for link in ['fr3_link7','fr3_link8']:
                o=SingleRigidPrim(path+'/'+link);o.initialize();links[side][link]=o
        cube=SingleRigidPrim(CUBE_PATH);cube.initialize()
        if a.configuration in 'BE':
            handles={s:dc.get_articulation(p) for s,p in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]}
            if any(h==_dynamic_control.INVALID_HANDLE for h in handles.values()):raise RuntimeError('DYNAMIC_CONTROL_HANDLE')
        local={}
        for side,sign in [('left',-1),('right',1)]:
            mat=np.eye(4);mat[:3,:3]=Rotation.from_euler('z',sign*np.pi/2).as_matrix();mat[:3,3]=[0,sign*.155,.08];local[side]=mat
        def tf(prim):
            p,q=prim.get_world_pose();T=np.eye(4);T[:3,3]=np.asarray(p,float).copy();T[:3,:3]=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();return T
        def meta(x):
            y=np.asarray(x);return {'dtype':str(y.dtype),'shape':list(y.shape),'pointer':int(y.__array_interface__['data'][0]),'object_id':id(y),'owns_data':bool(y.flags.owndata),'writeable':bool(y.flags.writeable)}
        def callback_sample(dt):
            try:
                snapshot={s:{'q':np.array(arms[s].get_joint_positions(),copy=True).tolist(),'dq':np.array(arms[s].get_joint_velocities(),copy=True).tolist()} for s in ['left','right']}
                callback.append({'dt_s':float(dt),'reported_sim_time_s':float(sim.current_time),'reported_sim_step':int(sim.current_time_step_index),'snapshot':snapshot})
            except BaseException as exc: callback.append({'dt_s':float(dt),'exception':type(exc).__name__+':'+str(exc)})
        if a.configuration=='C':subs.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(callback_sample,False,0))
        def read():
            obj=tf(cube);v={};timestamp=float(sim.current_time);tick=int(sim.current_time_step_index)
            for side in ['left','right']:
                arm=arms[side];view=views[side];qbuf=arm.get_joint_positions();vbuf=arm.get_joint_velocities();q=np.array(qbuf,dtype=np.float64,copy=True);dq=np.array(vbuf,dtype=np.float64,copy=True)
                L7=tf(links[side]['fr3_link7']);L8=tf(links[side]['fr3_link8']);T=L8@local[side];kp,kd=view.get_gains()
                M=np.array(view.get_mass_matrices(),dtype=float,copy=True)[0];c=np.array(view.get_coriolis_and_centrifugal_forces(),dtype=float,copy=True)[0];g=np.array(view.get_generalized_gravity_forces(),dtype=float,copy=True)[0]
                if any(not np.isfinite(x).all() for x in [q,dq,L7,L8,T,M,c,g]):raise RuntimeError('NONFINITE:'+side)
                fp,fr=residual(model.fk(side,q[:7]),T)
                viewq=view._physics_view.get_dof_positions();tensorq=np.array(viewq,dtype=float,copy=True).reshape(-1); viewv=view._physics_view.get_dof_velocities();tensorv=np.array(viewv,dtype=float,copy=True).reshape(-1)
                meta_q=meta(qbuf);meta_v=meta(vbuf);native_meta_q=meta(viewq);native_meta_v=meta(viewv)
                row={'names':list(arm.dof_names),'q':q.tolist(),'dq':dq.tolist(),'q_source':meta_q,'dq_source':meta_v,'native_tensor_q':tensorq.tolist(),'native_tensor_dq':tensorv.tolist(),'native_q_buffer':native_meta_q,'native_dq_buffer':native_meta_v,'native_wrapper_q_shares_memory':bool(np.shares_memory(np.asarray(qbuf),np.asarray(viewq))),'native_wrapper_dq_shares_memory':bool(np.shares_memory(np.asarray(vbuf),np.asarray(viewv))),'link7_world':L7.tolist(),'link8_world':L8.tolist(),'tcp_world':T.tolist(),'tcp_fk_error_m':fp,'tcp_fk_error_rad':fr,'M':M.tolist(),'c':c.tolist(),'g':g.tolist(),'drive_kp':np.asarray(kp).tolist(),'drive_kd':np.asarray(kd).tolist(),'drive_position_targets':np.array(view._physics_view.get_dof_position_targets(),copy=True).reshape(-1).tolist(),'drive_velocity_targets':np.array(view._physics_view.get_dof_velocity_targets(),copy=True).reshape(-1).tolist(),'effort_modes':view.get_effort_modes()[0],'applied_command_not_sensor':np.array(arm.get_applied_joint_efforts(),copy=True).tolist(),'solver_projected_not_sensor':np.array(arm.get_measured_joint_efforts(),copy=True).tolist()}
                if a.configuration in 'BE':
                    ds=dc.get_articulation_dof_states(handles[side],_dynamic_control.STATE_ALL);row['dynamic_control_q']=np.array(ds['pos'],copy=True).tolist();row['dynamic_control_dq']=np.array(ds['vel'],copy=True).tolist();row['dynamic_control_dtype']=str(ds.dtype)
                if a.configuration=='D':
                    if side in retained:
                        prior=retained[side];row['previous_borrowed_q_after_new_read']=np.array(prior['q'],copy=True).reshape(-1).tolist();row['previous_borrowed_dq_after_new_read']=np.array(prior['dq'],copy=True).reshape(-1).tolist();row['previous_immediate_q']=prior['copyq'];row['previous_immediate_dq']=prior['copyv']
                    retained[side]={'q':viewq,'dq':viewv,'copyq':tensorq.tolist(),'copyv':tensorv.tolist()}
                v[side]=row
            hit=collision.validate(model.forward(np.asarray(v['left']['q'][:7]),np.asarray(v['right']['q'][:7])),obj,'detached');unexpected=[z for z in hit.reasons if z!='COLLISION:Cube_01/box|task28_table/box']
            return {'physics_step':tick,'time_s':timestamp,'timeline_playing':bool(omni.timeline.get_timeline_interface().is_playing()),'callback_count':len(count),'cube_world':obj.tolist(),'arms':v,'collision':{'valid':not unexpected,'reasons':list(hit.reasons),'unexpected':unexpected,'checked_pairs':hit.checked_shape_pairs,'fcl_scope':'discrete conservative; expected Cube/table support only'},'state':'FREE_HOME_OR_SETTLING'}
        result['phase']='OBSERVE';row=read();record(row);last=row;save()
        for k in range(cfg['max_explicit_steps_per_live_cell']):
            if not last['collision']['valid']:raise RuntimeError('UNEXPECTED_COLLISION')
            if not last['timeline_playing']:raise RuntimeError('TIMELINE_NOT_PLAYING')
            if any(last['arms'][s]['tcp_fk_error_m']>cfg['fk_translation_tolerance_m'] or last['arms'][s]['tcp_fk_error_rad']>cfg['fk_rotation_tolerance_rad'] for s in ['left','right']):raise RuntimeError('TCP_FK_MISMATCH')
            prior_callbacks=len(count);sim.step(render=False);result['explicit_physics_steps']+=1
            if a.configuration=='F':
                before_refresh=len(count);sim.render();after_refresh=len(count)
                if before_refresh!=after_refresh:raise RuntimeError('REFRESH_UNEXPECTED_PHYSICS_STEP')
            row=read();row['post_callback']=callback[-1] if a.configuration=='C' and callback else None
            if last is not None:
                row['derived']={}
                dt=row['time_s']-last['time_s']
                for side in ['left','right']:
                    q0=np.asarray(last['arms'][side]['q'],float);q1=np.asarray(row['arms'][side]['q'],float);v0=np.asarray(last['arms'][side]['dq'],float);v1=np.asarray(row['arms'][side]['dq'],float)
                    row['derived'][side]={'delta_q':(q1-q0).tolist(),'whole_step_fd_dq':((q1-q0)/dt).tolist(),'reported_end_dq':v1.tolist(),'fd_minus_reported':((q1-q0)/dt-v1).tolist(),'trapezoid_integrated_dq':((v0+v1)*dt/2).tolist(),'integral_error':(q1-q0-(v0+v1)*dt/2).tolist(),'link8_delta_m':float(np.linalg.norm(np.asarray(row['arms'][side]['link8_world'])[:3,3]-np.asarray(last['arms'][side]['link8_world'])[:3,3]))}
            record(row);save()
            if row['physics_step']!=last['physics_step']+1 or len(count)!=prior_callbacks+1:raise RuntimeError('STEP_CURSOR')
            dt=row['time_s']-last['time_s']
            if abs(dt-cfg['physics_dt_s'])>cfg['dt_tolerance_s'] or abs(count[-1]-cfg['physics_dt_s'])>cfg['dt_tolerance_s']:raise RuntimeError('PHYSICS_DT')
            if not row['collision']['valid']:raise RuntimeError('UNEXPECTED_COLLISION')
            if a.configuration=='C' and (not callback or 'exception' in callback[-1]):raise RuntimeError('POST_CALLBACK_READ_FAILURE')
            last=row
        result['status']='BOUNDED_OBSERVATION_COMPLETE_PENDING_EVALUATION';result['phase']='COMPLETE';result['solver_semantics_observation']='END_STEP_DQ_RECORDED_NOT_REJECTED_BY_FINITE_DIFFERENCE'
    except BaseException as e:
        result['status']='REJECTED';result['first_failure']={'phase':result['phase'],'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc(),'last_frame_index':rows-1}
    finally:
        save()
        for sub in subs:
            try:sub.unsubscribe()
            except Exception:pass
        if app is not None:app.close()
    return 0
if __name__=='__main__':raise SystemExit(main())
