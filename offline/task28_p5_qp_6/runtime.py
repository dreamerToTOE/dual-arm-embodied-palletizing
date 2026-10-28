"""One registered isolated no-contact cell per fresh Isaac process."""
import argparse
import json
import sys
import time
import traceback
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT)); sys.path.insert(0,str(ROOT.parent/'qp6_deps'))
from offline.task28_p5_qp_6.common import HERE,ASSET,ASSET_SHA,NAMES,sha,dump,config


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--controller',choices=['A','B','C'],required=True); ap.add_argument('--profile',choices=['S05','S10'],required=True); ap.add_argument('--output',required=True,type=Path); ap.add_argument('--registration-sha',required=True)
    a=ap.parse_args(); c=config(); registration=HERE/'formal_registration.json'
    if sha(registration)!=a.registration_sha: raise ValueError('REGISTRATION_IDENTITY')
    reg=json.loads(registration.read_text())
    for path,digest in reg['files'].items():
        if sha(ROOT/path)!=digest: raise ValueError('SOURCE_HASH:'+path)
    if sha(ASSET)!=ASSET_SHA: raise ValueError('FROZEN_USD')
    eligible=reg['cell_eligibility'][a.controller+'/'+a.profile]
    if not eligible['eligible']: raise ValueError('CELL_NOT_ELIGIBLE:'+eligible['reason'])
    a.output.mkdir(exist_ok=False,parents=True); app=None; rows=[]; previous=None; total=0
    result={'status':'REJECTED','cell':a.controller+'/'+a.profile,'model_level':'A_NO_CUBE_VIRTUAL_OBJECT','registration_sha':a.registration_sha,'prereg_sha':sha(HERE/'preregistration.json'),'physics_steps':0,'moving_steps':0,'final_hold_steps':0,'initial_hold_steps':0,'physical_cube_present':False,'physical_contact':False,'internal_wrench_observed':False,'limits_label':c['limits_label'],'source_time_scope':'sequential reads at shared synchronous World CPU physics cursor; no API native per-state timestamp'}
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation,RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from scipy.linalg import block_diag
        from scipy.spatial.transform import Rotation
        from offline.task28_g1_1.task28_validator.model import RobotModel
        from offline.task28_p5_qp_6.profiles import Profile,full_joint_reference
        from offline.task28_p5_qp_6.controllers import no_contact
        from offline.task28_p5_qp_6.gates import RuntimeGate,Rejection
        model=RobotModel(); profile=Profile(a.profile); q0,_,_=full_joint_reference(profile.sample(0.)); gate=RuntimeGate(c,model)
        world=World(stage_units_in_meters=1.,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
        arms={}; links={}; views={}; idx=np.arange(9,dtype=np.int32); offsets={}
        for side,y,part in [('left',-.6,q0[:7]),('right',.6,q0[9:16])]:
            path='/World/QP6_'+side; add_reference_to_stage(str(ASSET),path)
            arms[side]=world.scene.add(SingleArticulation(path,name=side,position=np.array([.65,y,0.])))
            links[side]=world.scene.add(RigidPrim(path+'/fr3_link7',name='link_'+side))
            offsets[side]=np.linalg.inv(model._chain(side,part)[0][side+'_fr3_link7'])@model.fk(side,part)
        world.reset(); actual_dt=float(world.get_physics_dt())
        if abs(actual_dt-c['physics_dt_s'])>1e-9: raise ValueError('PHYSICS_DT')
        for side,part in [('left',q0[:9]),('right',q0[9:])]:
            v=arms[side]._articulation_view; views[side]=v
            v.set_effort_modes('force',joint_indices=idx); v.set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
            arms[side].set_joint_positions(part,joint_indices=idx); arms[side].set_joint_velocities(np.zeros(9),joint_indices=idx)
        def read_pair():
            states={}
            for side in ['left','right']:
                tick=time.perf_counter(); arm=arms[side]; v=views[side]
                q=np.asarray(arm.get_joint_positions(),float); dq=np.asarray(arm.get_joint_velocities(),float)
                pos,quat=links[side].get_world_poses(); link=np.eye(4); link[:3,3]=pos[0]; link[:3,:3]=Rotation.from_quat(np.asarray(quat[0])[[1,2,3,0]]).as_matrix()
                gains=v.get_gains()
                states[side]={'q':q,'dq':dq,'M':np.asarray(v.get_mass_matrices(),float)[0],'c':np.asarray(v.get_coriolis_and_centrifugal_forces(),float)[0],'g':np.asarray(v.get_generalized_gravity_forces(),float)[0],
                    'time_s':float(world.current_time),'step':int(world.current_time_step_index),'source_identity':'/World/QP6_'+side,'names':list(arm.dof_names),'effort_modes':v.get_effort_modes()[0],'drive_kp':np.asarray(gains[0]),'drive_kd':np.asarray(gains[1]),'tcp':link@offsets[side],'model_tcp':model.fk(side,q[:7]),'acquisition_wall_s':time.perf_counter()-tick}
            return states
        origin=None; nmove=round(profile.duration/actual_dt)
        phases=['initial_hold']*c['initial_hold_steps']+['movement']*nmove+['final_hold']*c['final_hold_steps']
        for j,phase in enumerate(phases):
            if phase=='movement' and origin is None: origin=float(world.current_time)
            cursor=0. if origin is None else float(world.current_time)-origin
            rpre=profile.sample(cursor); states=read_pair(); pretime=float(world.current_time); prestep=int(world.current_time_step_index)
            row={'phase':phase,'cursor_pre_s':cursor,'reference_identity':profile.identity,'states_pre':states,'reference_pre':rpre,'previous_effort':previous,'executed':False}
            pregate=gate.check(states,rpre,pretime,prestep); row['pre_state_gate']=pregate
            if not pregate['valid']: rows.append(row); result['first_failure']=pregate; raise Rejection('PRE_STATE')
            q=np.r_[states['left']['q'],states['right']['q']]; dq=np.r_[states['left']['dq'],states['right']['dq']]
            M=block_diag(states['left']['M'],states['right']['M']); h={'g':np.r_[states['left']['g'],states['right']['g']],'c':np.r_[states['left']['c'],states['right']['c']],'source_full_pass':reg['full_dynamics_source_pass']}
            u,ctrl=no_contact(a.controller,q,dq,M,h,rpre,previous,c,model); row['attempted_effort']=u; row['controller']=ctrl
            cmdgate=gate.check(states,rpre,pretime,prestep,effort=u,previous=previous,dt=actual_dt); row['pre_command_gate']=cmdgate
            if not cmdgate['valid']: rows.append(row); result['first_failure']=cmdgate; raise Rejection('PRE_COMMAND')
            # A coordinated pair is sent only after BOTH complete state/gain/effort gates.
            for side,cmd in [('left',u[:9]),('right',u[9:])]: arms[side].set_joint_efforts(cmd,joint_indices=idx)
            buffers={side:np.asarray(arms[side].get_applied_joint_efforts(),float) for side in ['left','right']}; row['command_buffer_before_step']=buffers
            if np.max(np.abs(np.r_[buffers['left'],buffers['right']]-u))>c['command_readback_tolerance']:
                rows.append(row); result['first_failure']={'reasons':['COMMAND_BUFFER_PRE_STEP']}; raise Rejection('COMMAND_BUFFER')
            world.step(render=False); total+=1; row['executed']=True
            post=read_pair(); postcursor=0. if phase=='initial_hold' else float(world.current_time)-origin
            rpost=profile.sample(postcursor); row.update(cursor_post_s=postcursor,states_post=post,reference_post=rpost)
            row['applied_command_readback']={side:np.asarray(arms[side].get_applied_joint_efforts(),float) for side in ['left','right']}
            row['solver_projected_effort']={side:np.asarray(arms[side].get_measured_joint_efforts(),float) for side in ['left','right']}
            postgate=gate.check(post,rpost,float(world.current_time),int(world.current_time_step_index)); row['post_gate']=postgate
            timing_ok=abs(float(world.current_time)-pretime-actual_dt)<1e-9 and int(world.current_time_step_index)==prestep+1
            if not timing_ok: postgate['valid']=False; postgate['reasons'].append('STEP_TIMING')
            if np.max(np.abs(np.r_[row['applied_command_readback']['left'],row['applied_command_readback']['right']]-u))>c['command_readback_tolerance']: postgate['valid']=False; postgate['reasons'].append('COMMAND_BUFFER_POST_STEP')
            rows.append(row); previous=u.copy()
            result[{'initial_hold':'initial_hold_steps','movement':'moving_steps','final_hold':'final_hold_steps'}[phase]]+=1
            if not postgate['valid']: result['first_failure']=postgate; raise Rejection('POST_STEP')
        result['status']='PASS'; result['completed_movement']=True; result['completed_final_hold']=True
    except BaseException as e:
        result['exception']={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
    finally:
        result['physics_steps']=total; result['unexecuted_registered_steps']=c['initial_hold_steps']+round(5*c['profiles'][a.profile]/c['physics_dt_s'])+c['final_hold_steps']-total
        dump(a.output/'raw.json',{'rows':rows}); dump(a.output/'result.json',result)
        if app is not None: app.close()
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__': raise SystemExit(main())
