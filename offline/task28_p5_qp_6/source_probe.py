"""Bounded isolated source characterization; never formal controller evidence."""
import argparse
import sys
import time
import traceback
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]; sys.path.insert(0,str(ROOT))
from offline.task28_p5_qp_6.common import HERE,ISAAC,ASSET,ASSET_SHA,NAMES,sha,dump,config


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True); ap.add_argument('--prereg-sha',required=True)
    args=ap.parse_args(); c=config()
    if sha(HERE/'preregistration.json')!=args.prereg_sha or sha(ASSET)!=ASSET_SHA: raise ValueError('SOURCE_IDENTITY_REJECTED')
    args.output.mkdir(exist_ok=False,parents=True); app=None; rows=[]; result={'status':'SOURCE_BLOCKED','prereg_sha':args.prereg_sha,'asset_sha':sha(ASSET),'collector_sha':sha(__file__),'physics_steps':0,'kind':'BOUNDED_SOURCE_CHARACTERIZATION_NOT_DEVELOPMENT_OR_FORMAL','time_scope':'single synchronous World CPU backend; API has no independent hardware source timestamp'}
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        from offline.task28_p5_qp_6.profiles import Profile,full_joint_reference
        q0,_,_=full_joint_reference(Profile('S05').sample(0))
        world=World(stage_units_in_meters=1.,physics_dt=c['physics_dt_s'],rendering_dt=c['physics_dt_s'],backend='numpy',device='cpu')
        arms={}; views={}; idx=np.arange(9,dtype=np.int32)
        for side,y in [('left',-.6),('right',.6)]:
            path='/World/Source_'+side; add_reference_to_stage(str(ASSET),path)
            arms[side]=world.scene.add(SingleArticulation(path,name=side,position=np.array([.65,y,0.])))
        world.reset()
        result['runtime_version']=(ISAAC/'VERSION').read_text(); result['wrapper_sha']=sha(ISAAC/'exts/isaacsim.core.prims/isaacsim/core/prims/impl/articulation.py')
        for side,part in [('left',q0[:9]),('right',q0[9:])]:
            a=arms[side]; v=a._articulation_view; views[side]=v
            if list(a.dof_names)!=NAMES: raise ValueError('DOF_ORDER:'+side)
            v.set_effort_modes('force',joint_indices=idx); v.set_gains(kps=np.zeros(9),kds=np.zeros(9),joint_indices=idx,save_to_usd=False)
            a.set_joint_positions(part,joint_indices=idx); a.set_joint_velocities(np.zeros(9),joint_indices=idx)
        errors=[]; readback=[]; hold_speed=[]
        for k in range(c['source_gravity_steps']+c['source_inverse_dynamics_steps']):
            row={'physics_step_pre':int(world.current_time_step_index),'sim_time_pre':float(world.current_time),'arms':{}}
            for side in ('left','right'):
                a=arms[side]; v=views[side]; tick=time.perf_counter()
                q=np.asarray(a.get_joint_positions(),float); dq=np.asarray(a.get_joint_velocities(),float)
                Mraw=np.asarray(v.get_mass_matrices(),float); craw=np.asarray(v.get_coriolis_and_centrifugal_forces(),float); graw=np.asarray(v.get_generalized_gravity_forces(),float)
                if Mraw.shape!=(1,9,9) or craw.shape!=(1,9) or graw.shape!=(1,9): raise ValueError('DYNAMICS_SHAPE:'+side)
                M,cc,g=Mraw[0],craw[0],graw[0]
                if not all(np.isfinite(x).all() for x in (q,dq,M,cc,g)): raise ValueError('NONFINITE_SOURCE')
                if np.max(np.abs(M-M.T))>c['source_mass_symmetry_inf'] or np.linalg.eigvalsh(M).min()<c['source_min_mass_eigenvalue']: raise ValueError('MASS_NOT_SPD')
                des=np.zeros(9)
                if k>=c['source_gravity_steps']:
                    axis=(k-c['source_gravity_steps'])//2; des[axis]=(.05 if axis<7 else .001)*(1 if k%2==0 else -1)
                tau=g+cc+M@des
                if np.any(np.abs(tau)>c['effort_bounds_per_robot']): raise ValueError('SOURCE_PROPOSED_EFFORT_BOUND')
                row['arms'][side]={'q_pre':q,'dq_pre':dq,'M_pre':M,'c_pre':cc,'g_pre':g,'qdd_requested':des,'effort_proposed':tau,'acquisition_wall_s':time.perf_counter()-tick,'source_sim_time':float(world.current_time),'source_physics_step':int(world.current_time_step_index),'names':list(a.dof_names),'units':['rad']*7+['m']*2,'effort_units':['N.m']*7+['N']*2,'mass_shape':list(Mraw.shape),'c_shape':list(craw.shape),'g_shape':list(graw.shape),'drive_modes':v.get_effort_modes()[0],'gains':[np.asarray(z).tolist() for z in v.get_gains()]}
            # Both source/proposed commands validated before either actuator write.
            for side in ('left','right'): arms[side].set_joint_efforts(row['arms'][side]['effort_proposed'],joint_indices=idx)
            for side in ('left','right'):
                rb=np.asarray(arms[side].get_applied_joint_efforts(),float); row['arms'][side]['command_buffer_before_step']=rb
                if np.max(np.abs(rb-row['arms'][side]['effort_proposed']))>c['command_readback_tolerance']: raise ValueError('COMMAND_BUFFER_REJECTED')
            world.step(render=False); result['physics_steps']+=1
            row['sim_time_post']=float(world.current_time); row['physics_step_post']=int(world.current_time_step_index)
            for side in ('left','right'):
                a=arms[side]; rr=row['arms'][side]; post=np.asarray(a.get_joint_velocities(),float)
                observed=(post-rr['dq_pre'])/float(world.get_physics_dt()); rr['q_post']=np.asarray(a.get_joint_positions(),float); rr['dq_post']=post; rr['qdd_backward_step']=observed
                rr['inverse_dynamics_residual_observed']=rr['M_pre']@observed+rr['c_pre']+rr['g_pre']-rr['effort_proposed']
                rr['applied_command_readback']=np.asarray(a.get_applied_joint_efforts(),float); rr['solver_projected_effort']=np.asarray(a.get_measured_joint_efforts(),float)
                rr['acceleration_error']=observed-rr['qdd_requested']; errors.append(np.abs(rr['acceleration_error']))
                readback.append(float(np.max(np.abs(rr['applied_command_readback']-rr['effort_proposed']))))
                if k<c['source_gravity_steps']: hold_speed.append(np.abs(post))
            rows.append(row)
        peak=np.max(errors,axis=0); speed=np.max(hold_speed,axis=0)
        result.update(max_acceleration_error=peak,max_gravity_hold_speed=speed,max_command_buffer_error=max(readback),same_pre_step_dynamics=True,full9_per_robot=True)
        gravity_ok=bool(np.max(speed[:7])<=c['source_gravity_hold_speed_arm_rad_s'] and np.max(speed[7:])<=c['source_gravity_hold_speed_finger_m_s'] and max(readback)<=c['command_readback_tolerance'])
        dynamics_ok=bool(np.max(peak[:7])<=c['source_acceleration_error_arm_rad_s2'] and np.max(peak[7:])<=c['source_acceleration_error_finger_m_s2'])
        result.update(gravity_state_command_source_pass=gravity_ok,full_dynamics_source_pass=gravity_ok and dynamics_ok,status='SOURCE_PASS' if gravity_ok and dynamics_ok else 'SOURCE_BLOCKED')
    except BaseException as e:
        result['failure']={'type':type(e).__name__,'message':str(e),'traceback':traceback.format_exc()}
    finally:
        dump(args.output/'observations.json',{'rows':rows}); dump(args.output/'result.json',result)
        if app is not None: app.close()
    print(result['status']); return 0 if result['status']=='SOURCE_PASS' else 1


if __name__=='__main__': raise SystemExit(main())
