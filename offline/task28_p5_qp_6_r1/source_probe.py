"""One preregistered source configuration per fresh Isaac process."""
import sys
import argparse
import json
import time
import traceback
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT.parent/'qp6_deps'))
from offline.task28_p5_qp_6_r1.common import HERE,ISAAC,ASSET,ASSET_SHA,NAMES,sha,dump,config
# Pin numerical backend before SimulationApp prepends its old pip_prebundle.
import osqp
from offline.task28_p5_qp_6_r1.dynamics import constrained_prediction,arm_effort_with_fingers,evaluate_R_step
if osqp.__version__!='1.0.5': raise RuntimeError('PINNED_OSQP_VERSION_REQUIRED')


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--configuration',choices=list('ABCD'),required=True); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--registration-sha',required=True); args=ap.parse_args()
    regfile=HERE/'source_registration_v2.json'
    if sha(regfile)!=args.registration_sha: raise ValueError('REGISTRATION_HASH')
    reg=json.loads(regfile.read_text())
    for path,h in reg['files'].items():
        if sha(ROOT/path)!=h: raise ValueError('REGISTERED_SOURCE_HASH:'+path)
    if sha(ASSET)!=ASSET_SHA: raise ValueError('FROZEN_USD_HASH')
    c=config(); variant=c['source_configs'][args.configuration]; args.output.mkdir(exist_ok=False,parents=True)
    app=None; rows=[]; previous=None; result={'status':'SOURCE_CONFIGURATION_REJECTED','configuration':args.configuration,'identity':variant['identity'],'registration_sha':args.registration_sha,'controlled_steps':0,'initialization_physics_steps':None,'source_kind':'BOUNDED_SOURCE_ONLY_NOT_FORMAL_CONTROLLER','Cube_D6_ROS_hardware':False}
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation,RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from scipy.spatial.transform import Rotation
        from pxr import PhysxSchema,UsdPhysics
        import omni.usd
        from offline.task28_g1_1.task28_validator.model import RobotModel
        from offline.task28_p5_qp_6.profiles import Profile,full_joint_reference
        from offline.task28_p5_qp_6_r1.dynamics import constrained_prediction,arm_effort_with_fingers,evaluate_R_step
        from offline.task28_p5_qp_6_r1.safety import check
        model=RobotModel(); initial=Profile('S05').sample(0); q0,_,_=full_joint_reference(initial)
        q0[[7,8,16,17]]=variant['finger_initial_m']
        world=World(stage_units_in_meters=1.,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
        arms={}; links={}; views={}; offsets={}; idx=np.arange(9,dtype=np.int32); schemas={}
        for side,y,part in [('left',-.6,q0[:7]),('right',.6,q0[9:16])]:
            path='/World/QP6R1_'+side; add_reference_to_stage(str(ASSET),path)
            arms[side]=world.scene.add(SingleArticulation(path,name=side,position=np.array([.65,y,0.])))
            links[side]=world.scene.add(RigidPrim(path+'/fr3_link7',name='link_'+side))
            offsets[side]=np.linalg.inv(model._chain(side,part)[0][side+'_fr3_link7'])@model.fk(side,part)
        world.reset(); result['initialization_physics_steps']=int(world.current_time_step_index); dt=float(world.get_physics_dt()); stage=omni.usd.get_context().get_stage()
        for side,part in [('left',q0[:9]),('right',q0[9:])]:
            v=arms[side]._articulation_view; views[side]=v
            if list(arms[side].dof_names)!=NAMES: raise ValueError('DOF_ORDER')
            v.set_effort_modes('force',joint_indices=idx); v.set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
            arms[side].set_joint_positions(part,joint_indices=idx); arms[side].set_joint_velocities(np.zeros(9),joint_indices=idx)
            base='/World/QP6R1_'+side+'/fr3_hand/'; joint=stage.GetPrimAtPath(base+'fr3_finger_joint2'); mimic=PhysxSchema.PhysxMimicJointAPI(joint,'rotX')
            gearing=float(mimic.GetGearingAttr().Get()); offset=float(mimic.GetOffsetAttr().Get()); targets=[str(t) for t in mimic.GetReferenceJointRel().GetTargets()]
            if gearing!=-1 or offset!=0 or targets!=[base+'fr3_finger_joint1']: raise ValueError('ACTUAL_MIMIC_IDENTITY')
            lo=[]; hi=[]
            for i in [1,2]:
                j=UsdPhysics.PrismaticJoint(stage.GetPrimAtPath(base+f'fr3_finger_joint{i}')); lo.append(float(j.GetLowerLimitAttr().Get())); hi.append(float(j.GetUpperLimitAttr().Get()))
            schemas[side]={'gearing':gearing,'offset_m':offset,'reference_joint':targets,'finger_lower':lo,'finger_upper':hi,'mimic_schema_metadata':str(joint.GetMetadata('apiSchemas'))}
        result.update(osqp_version=osqp.__version__,osqp_file=osqp.__file__,actual_mimic_and_limits=schemas,asset_sha=sha(ASSET),runtime_version=(ISAAC/'VERSION').read_text(),python=sys.version,numpy=np.__version__,actual_physics_dt_s=dt,wrapper_sha=sha(ISAAC/'exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py'))
        def read():
            states={}
            for side in ['left','right']:
                tick=time.perf_counter(); a=arms[side]; v=views[side]; q=np.asarray(a.get_joint_positions(),float); dq=np.asarray(a.get_joint_velocities(),float)
                p,quat=links[side].get_world_poses(); T=np.eye(4); T[:3,3]=p[0]; T[:3,:3]=Rotation.from_quat(np.asarray(quat[0])[[1,2,3,0]]).as_matrix(); gains=v.get_gains()
                states[side]={'identity':'/World/QP6R1_'+side,'names':list(a.dof_names),'time_s':float(world.current_time),'physics_step':int(world.current_time_step_index),'q':q,'dq':dq,'M':np.asarray(v.get_mass_matrices(),float)[0],'c':np.asarray(v.get_coriolis_and_centrifugal_forces(),float)[0],'g':np.asarray(v.get_generalized_gravity_forces(),float)[0],'tcp':T@offsets[side],'effort_modes':v.get_effort_modes()[0],'drive_kp':np.asarray(gains[0]),'drive_kd':np.asarray(gains[1]),'finger_lower':schemas[side]['finger_lower'],'finger_upper':schemas[side]['finger_upper'],'read_wall_s':time.perf_counter()-tick,'source_timestamp_semantics':'shared synchronous World CPU cursor, no native independent hardware sample timestamp'}
            return states
        for k in range(variant['maximum_controlled_steps']):
            pre=read(); row={'step_in_configuration':k,'pre':pre,'executed':False,'arms':{}}
            gate=check(pre,initial['pose'],float(world.current_time),int(world.current_time_step_index),c,model); row['pre_state_gate']=gate
            if not gate['valid']: rows.append(row); result['first_failure']=gate; raise RuntimeError('PRE_STATE')
            cmds=[]
            for side,part in [('left',q0[:9]),('right',q0[9:])]:
                s=pre[side]; q=s['q']; dq=s['dq']; M=s['M']; h=s['c']+s['g']; requested=np.zeros(9)
                if args.configuration in 'AB':
                    if k>=12:
                        axis=(k-12)//2; requested[axis]=(.05 if axis<7 else .001)*(1 if k%2==0 else -1)
                    tau=h+M@requested
                elif args.configuration=='C':
                    requested[:7]=c['source_arm_kp_s2']*(part[:7]-q[:7])-c['source_arm_kd_s']*dq[:7]
                    if 120<=k<148:
                        index=k-120; requested[index//4]+=c['source_pulse_acceleration_rad_s2']*(1 if index%2==0 else -1)
                    tau=arm_effort_with_fingers(M,s['c'],s['g'],q,dq,requested[:7],c)
                else:
                    delta=np.zeros(9)
                    if k>=12: delta[7:]=c['source_negative_null_force_N']*(1 if k%2==0 else -1)*np.array([1.,-1.])
                    tau=h+delta; requested=np.linalg.solve(M,delta)
                pred,pdiag=constrained_prediction(M,h,tau,q,dq,dt,s['finger_lower'],s['finger_upper'])
                row['arms'][side]={'requested_free_qdd':requested,'proposed_effort':tau,'K_candidate_prediction':pred,'K_candidate_solver':pdiag,'constraint_fit_is_not_reaction_sensor':True}
                cmds.append(tau)
            u=np.r_[cmds[0],cmds[1]]; gate=check(pre,initial['pose'],float(world.current_time),int(world.current_time_step_index),c,model,u,previous); row['pre_command_gate']=gate
            if not gate['valid']: rows.append(row); result['first_failure']=gate; raise RuntimeError('PRE_COMMAND')
            for side,cmd in [('left',u[:9]),('right',u[9:])]: arms[side].set_joint_efforts(cmd,joint_indices=idx)
            buffers={s:np.asarray(arms[s].get_applied_joint_efforts(),float) for s in ['left','right']}; row['command_buffer_pre_step']=buffers
            if np.max(np.abs(np.r_[buffers['left'],buffers['right']]-u))>c['command_buffer_tolerance']: rows.append(row); result['first_failure']={'reasons':['COMMAND_BUFFER']}; raise RuntimeError('COMMAND_BUFFER')
            world.step(render=False); result['controlled_steps']+=1; row['executed']=True; post=read(); row['post']=post
            gate=check(post,initial['pose'],float(world.current_time),int(world.current_time_step_index),c,model); row['post_safety_gate']=gate; model_fail=[]
            for side in ['left','right']:
                info=row['arms'][side]; s=pre[side]; obs=(post[side]['dq']-s['dq'])/dt; request=info['requested_free_qdd']; applied=np.asarray(arms[side].get_applied_joint_efforts(),float)
                info.update(observed_qdd=obs,applied_command_readback=applied,solver_projected_effort_not_actuator_sensor=np.asarray(arms[side].get_measured_joint_efforts(),float))
                F=bool(np.max(np.abs(obs[:7]-request[:7]))<=c['arm_acceleration_error_rad_s2'] and np.max(np.abs(obs[7:]-request[7:]))<=c['finger_free_acceleration_error_m_s2']); info['F_free_model_pass']=F
                pred=info['K_candidate_prediction']; K=bool(pred is not None and np.max(np.abs(obs[:7]-pred[:7]))<=c['arm_acceleration_error_rad_s2'] and np.max(np.abs(obs[7:]-pred[7:]))<=c['finger_free_acceleration_error_m_s2']); info['K_prediction_pass']=K
                if args.configuration=='C':
                    R=evaluate_R_step(s,post[side],applied,request[:7],dt,s['finger_upper'],c); info['R_step']=R
                    if not R['valid']: model_fail.extend([side+':'+r for r in R['reasons']])
                if args.configuration in 'AB' and not F: model_fail.append(side+':ORIGINAL_F_ACCELERATION_GATE')
                if args.configuration=='D' and not K: model_fail.append(side+':K_NEGATIVE_CONTROL_RESPONSE')
                if np.max(np.abs(applied-info['proposed_effort']))>c['command_buffer_tolerance']: model_fail.append(side+':POST_COMMAND_BUFFER')
                if post[side]['physics_step']!=s['physics_step']+1 or abs(post[side]['time_s']-s['time_s']-dt)>1e-9: model_fail.append(side+':SOURCE_TIMING')
            row['registered_model_gate']={'valid':not model_fail,'reasons':model_fail}; rows.append(row); previous=u.copy()
            if not gate['valid'] or model_fail:
                result['first_failure']={'reasons':gate['reasons']+model_fail,'row_index':len(rows)-1}; raise RuntimeError('POST_GATE_LATCHED')
        result['status']='SOURCE_CONFIGURATION_COMPLETE_PENDING_INDEPENDENT_EVALUATION'
    except BaseException as e:
        result['exception']={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
    finally:
        result['unexecuted_steps']=variant['maximum_controlled_steps']-result['controlled_steps']; dump(args.output/'raw.json',{'rows':rows}); dump(args.output/'result.json',result)
        if app is not None: app.close()
    return 0 if result['status']=='SOURCE_CONFIGURATION_COMPLETE_PENDING_INDEPENDENT_EVALUATION' else 1


if __name__=='__main__': raise SystemExit(main())
