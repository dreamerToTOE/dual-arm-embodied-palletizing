"""Independent raw-data evaluator. Does not call collector or runtime gate."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .common import HERE,dump,sha,config
from .profiles import Profile,full_joint_reference
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps


def source_evaluation():
    c=config(); rows=json.loads((HERE/'evidence/source_v1/observations.json').read_text())['rows']
    first=None; errors=[]; hold=[]; allres=[]; matrices=[]; buffers=[]; order=[]
    for k,r in enumerate(rows):
        dt=r['sim_time_post']-r['sim_time_pre']
        if abs(dt-c['physics_dt_s'])>1e-9 or r['physics_step_post']!=r['physics_step_pre']+1: order.append(k)
        for side,v in r['arms'].items():
            M=np.asarray(v['M_pre']); cc=np.asarray(v['c_pre']); g=np.asarray(v['g_pre']); tau=np.asarray(v['effort_proposed'])
            dd=(np.array(v['dq_post'])-np.array(v['dq_pre']))/dt
            er=dd-np.array(v['qdd_requested']); residual=M@dd+cc+g-tau
            errors.append(np.abs(er)); allres.append(np.abs(residual)); matrices.append(np.linalg.eigvalsh(M))
            buffers.append(float(np.max(np.abs(np.asarray(v['applied_command_readback'])-tau))))
            if k<c['source_gravity_steps']: hold.append(np.abs(v['dq_post']))
            failed=np.any(np.abs(er[:7])>c['source_acceleration_error_arm_rad_s2']) or np.any(np.abs(er[7:])>c['source_acceleration_error_finger_m_s2'])
            if failed and first is None: first={'row_index':k,'side':side,'physics_step_post':r['physics_step_post'],'post_time_s':r['sim_time_post'],'acceleration_error':er,'empirical_dynamics_residual':residual,'raw_frame':r}
    speed=np.max(hold,axis=0); peak=np.max(errors,axis=0)
    gravity=bool(np.max(speed[:7])<c['source_gravity_hold_speed_arm_rad_s'] and np.max(speed[7:])<c['source_gravity_hold_speed_finger_m_s'] and max(buffers)<c['command_readback_tolerance'] and not order)
    return {'status':'SOURCE_PASS' if gravity and first is None else 'SOURCE_BLOCKED','gravity_state_command_pass':gravity,'full_18d_dynamics_pass':gravity and first is None,'first_rejected_frame':first,'peak_acceleration_error_per_robot_dof':peak,'peak_empirical_dynamics_residual_per_robot_dof':np.max(allres,axis=0),'minimum_mass_eigenvalue':float(np.min(matrices)),'maximum_mass_eigenvalue':float(np.max(matrices)),'max_command_buffer_error':max(buffers),'timing_invalid_indices':order,'raw_rows':len(rows),'source_only_posthoc_rejection':True,'later_source_rows_after_first_rejection':0 if first is None else len(rows)-first['row_index']-1,'limitation':'Source characterization collected its predetermined bounded pulse sequence before offline acceleration acceptance; later source frames are diagnostic only. Formal cell runtime has a per-step rejection latch. No source rerun or 14DOF bypass.'}


def cell_evaluation(name):
    path=HERE/'evidence'/('formal_'+name.replace('/','_')); result=json.loads((path/'result.json').read_text()); rows=json.loads((path/'raw.json').read_text())['rows']
    profile=Profile(name.split('/')[1]); grip=nominal_grasps(); metrics=[]; efforts=[]; rates=[]; qerr=[]; derr=[]; dyn=[]; first=None; previous=None
    for k,r in enumerate(rows):
        if not r['executed']: continue
        post=r['states_post']; ref=profile.sample(r['cursor_post_s']); qr,dr,_=full_joint_reference(ref)
        q=np.r_[post['left']['q'],post['right']['q']]; dq=np.r_[post['left']['dq'],post['right']['dq']]
        qerr.append(np.abs(q-qr)); derr.append(np.abs(dq-dr)); implied=[]
        for side,g in [('left',grip.left),('right',grip.right)]:
            tcp=np.asarray(post[side]['tcp']); X=np.eye(4); X[:3,:3]=tcp[:3,:3]@g[:3,:3].T; X[:3,3]=tcp[:3,3]-X[:3,:3]@g[:3,3]; implied.append(X)
        e=[X[:3,3]-ref['pose'][:3,3] for X in implied]
        rot=[float(np.linalg.norm(Rotation.from_matrix(ref['pose'][:3,:3].T@X[:3,:3]).as_rotvec())) for X in implied]
        relative_rotation=float(np.linalg.norm(Rotation.from_matrix(implied[0][:3,:3].T@implied[1][:3,:3]).as_rotvec()))
        m={'time_s':r['cursor_post_s'],'phase':r['phase'],'physics_step':post['left']['step'],'left_translation_m':float(np.linalg.norm(e[0])),'right_translation_m':float(np.linalg.norm(e[1])),'relative_translation_m':float(np.linalg.norm(e[0]-e[1])),'common_translation_m':float(np.linalg.norm((e[0]+e[1])/2)),'left_rotation_rad':rot[0],'right_rotation_rad':rot[1],'relative_rotation_rad':relative_rotation,'common_error_vector_m':(e[0]+e[1])/2,'differential_error_vector_m':e[0]-e[1]}
        failure=[]
        for key in ['left_translation_m','right_translation_m','relative_translation_m']:
            if m[key]>1e-4: failure.append(key)
        for key in ['left_rotation_rad','right_rotation_rad','relative_rotation_rad']:
            if m[key]>1e-3: failure.append(key)
        if failure and first is None: first={'row':k,'metrics':m,'reasons':failure}
        metrics.append(m); u=np.array(r['attempted_effort']); efforts.append(np.abs(u))
        dt=post['left']['time_s']-r['states_pre']['left']['time_s']
        if previous is not None: rates.append(np.abs(u-previous)/dt)
        previous=u
        rr=[]
        for side,cmd in [('left',u[:9]),('right',u[9:])]:
            pre=r['states_pre'][side]; dd=(np.asarray(post[side]['dq'])-np.asarray(pre['dq']))/dt
            rr.extend(np.abs(np.asarray(pre['M'])@dd+np.asarray(pre['c'])+np.asarray(pre['g'])-cmd))
        dyn.append(rr)
    stop_correct=first is None or first['row']==len(rows)-1
    return {'cell':name,'status':result['status'],'independent_geometry_status':'REJECTED' if first else 'NO_GEOMETRY_FAILURE_IN_RECORDED_PREFIX','first_failure':first,'first_failure_is_last_recorded_frame':stop_correct,'actual_physics_steps':result['physics_steps'],'movement_steps':result['moving_steps'],'initial_hold_steps':result['initial_hold_steps'],'final_hold_steps':result['final_hold_steps'],'unexecuted_steps':result['unexecuted_registered_steps'],'completed_movement':result.get('completed_movement',False),'completed_final_hold':result.get('completed_final_hold',False),'peaks':{key:max(m[key] for m in metrics) for key in ['left_translation_m','right_translation_m','relative_translation_m','common_translation_m','left_rotation_rad','right_rotation_rad','relative_rotation_rad']},'q_error_peak_per_dof':np.max(qerr,axis=0),'dq_error_peak_per_dof':np.max(derr,axis=0),'effort_peak_per_dof':np.max(efforts,axis=0),'effort_rate_peak_per_dof_per_s':np.max(rates,axis=0) if rates else None,'empirical_dynamics_residual_peak_per_dof':np.max(dyn,axis=0),'residual_scope':'evaluator diagnostic only; complete free 18DOF dynamics source was not accepted','QP_solve_time_peak_and_percentiles':None,'QP_feasibility':'NOT_APPLICABLE_BASELINE_A_NO_QP','physical_cube_tracking':None,'virtual_object_tracking':metrics,'contact_wrench_allocation':None,'grasp_map_residual':None,'internal_wrench_prediction_or_observation':None,'contact_detach_and_finite_load_margins':None,'raw_sha':sha(path/'raw.json'),'result_sha':sha(path/'result.json')}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',required=True,type=Path); a=ap.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
    source=source_evaluation(); dump(a.output/'source.json',source)
    cells=[]
    for ctrl in 'ABCDE':
        for p in ['S05','S10']:
            name=ctrl+'/'+p
            if ctrl=='A':
                record=cell_evaluation(name); dump(a.output/('evaluation_'+ctrl+'_'+p+'.json'),record)
                cells.append({k:record[k] for k in ['cell','status','actual_physics_steps','movement_steps','final_hold_steps','completed_movement','completed_final_hold','first_failure']})
            else: cells.append({'cell':name,'status':'SKIPPED_WITH_REASON','actual_physics_steps':0,'reason':'SOURCE_BLOCKED_FULL_18DOF_DYNAMICS; finger acceleration gate failed'+('; actual Cube/D6 attachment and contact-wrench source not independently established; no contact attempt eligible' if ctrl in 'DE' else ''),'physical_level':'A_NO_CUBE' if ctrl in 'BC' else 'B_IDEAL_ATTACHMENT'})
    dump(a.output/'matrix.json',cells)
    print({r['cell']:r['status'] for r in cells})


if __name__=='__main__': main()
