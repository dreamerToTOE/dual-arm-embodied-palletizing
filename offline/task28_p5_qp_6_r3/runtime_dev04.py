"""Development-only source run: original builder and native startup drives.

No effort/position writes. Persist before SimulationApp shutdown. The only stage
mutation outside the builder is disabling its publishing-only ROSGraph on the
fresh anonymous research stage so no ROS graph executes.
"""
import argparse, builtins, importlib.util, json, os, sys, traceback
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r2.common import FR3_ASSET,FR3_SHA256,FIXTURE,TASK_ROOT,LEFT_ROOT,RIGHT_ROOT,CUBE_PATH,TCP_PATHS,DOF_NAMES,sha256
from offline.task28_p5_qp_6_r2.runtime_probe import _runtime_identity

HERE=Path(__file__).resolve().parent

def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    def conv(x):
        if isinstance(x,np.ndarray): return x.tolist()
        if isinstance(x,np.generic): return x.item()
        raise TypeError(str(type(x)))
    path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False,default=conv)+'\n')

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    if a.output.exists(): raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    cfg=json.loads((HERE/'preregistration.json').read_text()); rows=[]; callback_dts=[]; app=None; sub=None
    result={'status':'REJECTED','phase':'BOOT','first_failure':None,'robot_command_writes':0,'suction_commands':0,'physics_steps_commanded':0,'development_run':True,'scene_source':'frozen build_scene','ros_graph_session_disabled':False}
    def save():
        result['callback_dts_s']=callback_dts.copy(); result['recorded_frames']=len(rows)
        write(a.output/'result.json',result); write(a.output/'raw.json',{'rows':rows})
    os.environ['TASK28_FR3_USD']=str(FR3_ASSET);os.environ['TASK28_FR3_USD_SHA256']=FR3_SHA256
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        import omni.usd,omni.timeline,omni.physx
        from pxr import Usd,UsdGeom,UsdPhysics
        from scipy.spatial.transform import Rotation
        from isaacsim.core.api.simulation_context import SimulationContext
        from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
        import omni.kit.app
        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate('omni.isaac.dynamic_control',True)
        from omni.isaac.dynamic_control import _dynamic_control
        dc=_dynamic_control.acquire_dynamic_control_interface()
        from offline.task28_g1_1.task28_validator.model import RobotModel,DEFAULT_FIXTURE
        from offline.task28_g1_1.task28_validator.collision import CollisionValidator
        from offline.task28_g1_1.task28_validator.se3 import residual
        from offline.task28_g1_1.task28_validator.closed_chain import evaluate_closure
        ctx=omni.usd.get_context();ctx.new_stage();stage=ctx.get_stage();UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.0);UsdGeom.Xform.Define(stage,'/World')
        spec=importlib.util.spec_from_file_location('frozen_task28_r3',FIXTURE/'scene/task28_single_cube_scene.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
        result['phase']='SCENE_BUILD';mod.build_scene();scene=_runtime_identity(stage,mod);result['scene_before']=scene
        if scene['status']!='SCENE_IDENTITY_VERIFIED': raise RuntimeError('SCENE_IDENTITY')
        scenes=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)];result['physics_scenes_before']=scenes
        if scenes!=[TASK_ROOT+'/physicsScene']: raise RuntimeError('PHYSICS_SCENE_COUNT')
        graph=stage.GetPrimAtPath(TASK_ROOT+'/ROSGraph');result['ros_graph_before_active']=bool(graph.IsActive())
        if not graph.IsValid() or not graph.IsActive(): raise RuntimeError('ROS_GRAPH_IDENTITY')
        graph.SetActive(False);result['ros_graph_session_disabled']=not bool(graph.IsActive());result['ros_graph_mutation_scope']='fresh anonymous stage only; no saved USD'
        if getattr(builtins,'_task28_g1_0_single_cube_bridge',None) is not None: raise RuntimeError('LEGACY_BRIDGE_PRESENT')
        model=RobotModel(); collision=CollisionValidator(DEFAULT_FIXTURE)
        # set_defaults=False and the exact existing physics prim path avoid creating a second scene.
        result['phase']='SIMULATION_CONTEXT';sim=SimulationContext(physics_prim_path=TASK_ROOT+'/physicsScene',set_defaults=False,backend='numpy',device='cpu')
        scenes=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)];result['physics_scenes_after_context']=scenes
        if scenes!=[TASK_ROOT+'/physicsScene']: raise RuntimeError('PHYSICS_SCENE_COUNT_AFTER_CONTEXT')
        def onstep(dt): callback_dts.append(float(dt))
        sub=omni.physx.get_physx_interface().subscribe_physics_step_events(onstep)
        result['phase']='PLAY_NATIVE_DRIVES';sim.play()
        arms={};links={}
        for side,path in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]:
            arms[side]=SingleArticulation(path);arms[side].initialize();links[side]=SingleRigidPrim(path+'/fr3_link8');links[side].initialize()
            if not arms[side].handles_initialized or list(arms[side].dof_names)!=DOF_NAMES: raise RuntimeError('DOF_IDENTITY:'+side)
        dc_handles={side:dc.get_articulation(path) for side,path in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]}
        if any(handle==_dynamic_control.INVALID_HANDLE for handle in dc_handles.values()): raise RuntimeError('DYNAMIC_CONTROL_HANDLE')
        cube=SingleRigidPrim(CUBE_PATH);cube.initialize()
        local={}
        for side,sign in [('left',-1),('right',1)]:
            mat=np.eye(4);mat[:3,:3]=Rotation.from_euler('z',sign*np.pi/2).as_matrix();mat[:3,3]=[0,sign*.155,.08];local[side]=mat
        def transform(prim):
            p,q=prim.get_world_pose();T=np.eye(4);T[:3,3]=np.asarray(p);T[:3,:3]=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();return T
        def read():
            obj=transform(cube); values={};now=float(sim.current_time);tick=int(sim.current_time_step_index)
            for side in ['left','right']:
                arm=arms[side];v=arm._articulation_view; q=np.asarray(arm.get_joint_positions(),float);dq=np.asarray(arm.get_joint_velocities(),float);L=transform(links[side]);tcp=L@local[side]; kp,kd=v.get_gains()
                M=np.asarray(v.get_mass_matrices(),float)[0];c=np.asarray(v.get_coriolis_and_centrifugal_forces(),float)[0];g=np.asarray(v.get_generalized_gravity_forces(),float)[0]
                if any(not np.isfinite(x).all() for x in [q,dq,L,tcp,M,c,g]): raise RuntimeError('NONFINITE:'+side)
                fk=model.fk(side,q[:7]); fp,fr=residual(fk,tcp)
                dc_state=dc.get_articulation_dof_states(dc_handles[side],_dynamic_control.STATE_ALL)
                values[side]={'dynamic_control_q':np.asarray(dc_state['pos'],float),'dynamic_control_dq':np.asarray(dc_state['vel'],float),'q':q,'dq':dq,'link8_world':L,'tcp_world':tcp,'fk_tcp_error_m':fp,'fk_tcp_error_rad':fr,'M':M,'c':c,'g':g,'kp':np.asarray(kp),'kd':np.asarray(kd),'effort_modes':v.get_effort_modes()[0],'applied_command_not_sensor':np.asarray(arm.get_applied_joint_efforts(),float),'solver_projected_not_actuator_sensor':np.asarray(arm.get_measured_joint_efforts(),float),'names':list(arm.dof_names)}
            hit=collision.validate(model.forward(values['left']['q'][:7],values['right']['q'][:7]),obj,'detached');unexpected=[x for x in hit.collisions if {x['owner_a'],x['owner_b']}!={'Cube_01','task28_table'}]
            reasons=[r for r in hit.reasons if not r.startswith('COLLISION:Cube_01/box|task28_table/box')]
            cl=evaluate_closure(obj,values['left']['tcp_world'],values['right']['tcp_world'])
            return {'physics_step':tick,'time_s':now,'cube_world':obj,'arms':values,'collision':{'valid':not unexpected and not reasons,'unexpected':unexpected,'all_reasons':hit.reasons,'expected_support_collision_count':len(hit.collisions)-len(unexpected),'checked_pairs':hit.checked_shape_pairs,'backend':hit.backend,'clearance_m':None},'closure_diagnostic_not_gate':{'valid':cl.valid,'left_m':cl.left_object_translation_error_m,'right_m':cl.right_object_translation_error_m,'relative_m':cl.translation_error_m},'state':'FREE_HOME_OR_SETTLING'}
        result['tool_mass_inertia_inspection']={}
        for side,path in [('left',LEFT_ROOT),('right',RIGHT_ROOT)]:
            l8=stage.GetPrimAtPath(path+'/fr3_link8'); tool=stage.GetPrimAtPath(path+'/fr3_link8/side_suction_tool')
            mass=UsdPhysics.MassAPI(l8) if l8.HasAPI(UsdPhysics.MassAPI) else None
            def authored(attr):
                val=attr.Get() if attr and attr.IsValid() else None
                if val is None:return None
                try: return [float(x) for x in val]
                except TypeError:return float(val)
            children=[]
            for prim in Usd.PrimRange(tool):
                children.append({'path':str(prim.GetPath()),'type':str(prim.GetTypeName()),'collision_api':prim.HasAPI(UsdPhysics.CollisionAPI),'rigid_body_api':prim.HasAPI(UsdPhysics.RigidBodyAPI),'mass_api':prim.HasAPI(UsdPhysics.MassAPI)})
            result['tool_mass_inertia_inspection'][side]={'link8_mass_api':bool(mass),'link8_authored_mass_kg':authored(mass.GetMassAttr()) if mass else None,'link8_authored_diagonal_inertia':authored(mass.GetDiagonalInertiaAttr()) if mass else None,'link8_effective_mass_kg':float(links[side].get_mass()),'link8_effective_inertia':np.asarray(links[side]._rigid_prim_view.get_inertias(),float),'tool_prims':children,'tool_inertia_increment':'UNKNOWN_FROM_SINGLE_CONFIGURATION'}
        initial=read();rows.append(initial);save();stable=0
        for k in range(cfg['maximum_explicit_steps']):
            previous=rows[-1]
            if not previous['collision']['valid']: raise RuntimeError('UNEXPECTED_COLLISION')
            if any(previous['arms'][s]['fk_tcp_error_m']>.001 or previous['arms'][s]['fk_tcp_error_rad']>.002 for s in ['left','right']): raise RuntimeError('TCP_FK_MISMATCH')
            if callback_dts and any(abs(x-1/60)>cfg['physics_dt_tolerance_s'] for x in callback_dts): raise RuntimeError('PHYSICS_DT')
            sim.step(render=False);result['physics_steps_commanded']+=1;row=read();rows.append(row);save()
            if not row['collision']['valid']: raise RuntimeError('UNEXPECTED_COLLISION')
            result['status']='MOUNTED_INERTIA_READ_COMPLETED';break
            if len(rows)>=6:
                frozen=all(np.max(np.abs(np.asarray(rows[-j]['arms'][side]['q'])-np.asarray(rows[-j-1]['arms'][side]['q'])))<1e-10 for side in ['left','right'] for j in range(1,5))
                nonzero=max(np.max(np.abs(row['arms'][side]['dq'][:7])) for side in ['left','right'])>.01
                if frozen and nonzero: raise RuntimeError('FROZEN_POSITION_NONZERO_VELOCITY_SOURCE_MISMATCH')
            speeds=[np.max(np.abs(row['arms'][s]['dq'][:7])) for s in ['left','right']]
            errors=[np.max(np.abs(row['arms'][s]['q'][:7]-model.home)) for s in ['left','right']]
            stable=stable+1 if max(speeds)<cfg['settle_max_arm_speed_rad_s'] and max(errors)<cfg['settle_max_home_error_rad'] else 0
            if stable>=cfg['settle_consecutive_samples']:
                result['status']='NATIVE_DRIVE_SETTLE_COMPLETE';result['settled_consecutive']=stable;break
        else: raise RuntimeError('NATIVE_DRIVE_SETTLE_TIMEOUT')
        result['phase']='COMPLETE'
    except BaseException as e:
        result['first_failure']={'phase':result['phase'],'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc(),'last_frame_index':len(rows)-1}; result['status']='REJECTED'
    finally:
        save()
        if sub is not None: sub.unsubscribe()
        if app is not None: app.close()
    return 0
if __name__=='__main__': raise SystemExit(main())
