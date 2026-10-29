"""Independent evaluator for R10 virtual cooperative-QP cells."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.se3 import residual, inverse

HERE=Path(__file__).resolve().parent

def read_jsonl(p): return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]

def eval_cell(name):
    d=HERE/'evidence/diagnostic'/name; result=json.loads((d/'result.json').read_text())
    if result.get('status') != 'NO_CONTACT_R10_VIRTUAL_COOPERATION_COMPLETE':
        return {'status':'NOT_ELIGIBLE','runtime_status':result.get('status'),'commands':result.get('actual_effort_commands',0),'first_failure':result.get('first_failure')}
    frames=read_jsonl(d/'frames.jsonl'); vf=result['virtual_frame']; Tvl=np.asarray(vf['T_VL'],float); Tvr=np.asarray(vf['T_VR'],float)
    baseline=result['ownership_initial_state']; cube0=np.asarray(baseline['cube_world'],float)
    records=[]; max_obj=0.; max_rel=0.; max_rot=0.; max_rel_rot=0.; max_q=0.; max_qdot=0.; max_native_dq=0.; max_qp=0.; first_bad=None
    for row in frames:
        if 'r10_qp' not in row: continue
        Tl=np.asarray(row['arms']['left']['tcp_world'],float); Tr=np.asarray(row['arms']['right']['tcp_world'],float)
        goal=np.asarray(row['r10_qp']['virtual_goal'],float); Il=Tl@inverse(Tvl); Ir=Tr@inverse(Tvr)
        el=residual(Il,goal); er=residual(Ir,goal); ep=residual(Il,Ir)
        max_obj=max(max_obj,el[0],er[0]); max_rot=max(max_rot,el[1],er[1]); max_rel=max(max_rel,ep[0]); max_rel_rot=max(max_rel_rot,ep[1]);
        q=np.r_[np.asarray(row['arms']['left']['q'][:7]),np.asarray(row['arms']['right']['q'][:7])]
        qref=np.asarray(row['r10_qp']['q_ref'],float); qd=np.asarray(row['r10_qp']['qdot'],float)
        native_dq=np.r_[np.asarray(row['arms']['left']['dq'][:7]),np.asarray(row['arms']['right']['dq'][:7])]
        max_q=max(max_q,float(np.max(np.abs(q-qref)))); max_qdot=max(max_qdot,float(np.max(np.abs(qd)))); max_native_dq=max(max_native_dq,float(np.max(np.abs(native_dq))))
        qpd=row['r10_qp']['diagnostics']; max_qp=max(max_qp,float(qpd['hard_relative_residual_inf']))
        bad=list(row['collision'].get('unexpected',[]))
        if bad and first_bad is None: first_bad={'physics_step':row['physics_step'],'reasons':bad}
        records.append({'physics_step':row['physics_step'],'object_left_error_m':el[0],'object_right_error_m':er[0],'object_left_rotation_rad':el[1],'object_right_rotation_rad':er[1],'relative_error_m':ep[0],'relative_rotation_rad':ep[1]})
    final=frames[-1]; cubef=np.asarray(final['cube_world'],float); cube_disp=float(np.linalg.norm(cubef[:3,3]-cube0[:3,3]))
    final_obj_l=np.asarray(final['arms']['left']['tcp_world'])@inverse(Tvl); final_obj_r=np.asarray(final['arms']['right']['tcp_world'])@inverse(Tvr)
    virtual_disp=float(np.linalg.norm(final_obj_l[:3,3]-np.asarray(vf['T_WV'])[:3,3]))
    efforts=[]; rates=[]; previous={}
    for cmd in result.get('command_sequence',[]):
        for side in ('left','right'):
            effort=np.asarray(cmd['effort_arm'][side],float)
            efforts.append(effort)
            if side in previous:
                rates.extend(np.abs((effort-previous[side])*60.).ravel())
            previous[side]=effort
    prereg=json.loads((HERE/'preregistration.json').read_text())['acceptance']
    accuracy={
        'virtual_object_tracking_peak_m': {'value': max_obj, 'limit': prereg['virtual_object_tracking_peak_m'], 'pass': max_obj <= prereg['virtual_object_tracking_peak_m']},
        'relative_tcp_translation_peak_m': {'value': max_rel, 'limit': prereg['relative_tcp_translation_peak_m'], 'pass': max_rel <= prereg['relative_tcp_translation_peak_m']},
        'relative_tcp_rotation_peak_rad': {'value': max_rel_rot, 'limit': prereg['relative_tcp_rotation_peak_rad'], 'pass': max_rel_rot <= prereg['relative_tcp_rotation_peak_rad']},
        'joint_tracking_peak_rad': {'value': max_q, 'limit': prereg['joint_tracking_peak_rad'], 'pass': max_q <= prereg['joint_tracking_peak_rad']},
        'native_dq_peak_rad_s': {'value': max_native_dq, 'limit': prereg['native_dq_peak_rad_s'], 'pass': max_native_dq <= prereg['native_dq_peak_rad_s']},
    }
    no_contact=not first_bad and result.get('suction_commands')==0 and result.get('cube_contact_commands')==0 and cube_disp <= prereg['cube_translation_m']
    accuracy_pass=all(v['pass'] for v in accuracy.values())
    if no_contact and accuracy_pass: status='PASS_NO_CONTACT_ACCURACY_GATE_PASS'
    elif no_contact: status='PASS_NO_CONTACT_ACCURACY_GATE_FAIL'
    else: status='FAIL_NO_CONTACT_GATE'
    eligible = bool(no_contact and accuracy_pass and result.get('level1_source') == 'PASS' and result.get('level2_source') != 'BLOCKED_AFTER_FAILURE')
    return {'status':status, 'eligible_for_v10d':eligible, 'no_contact_status':'PASS' if no_contact else 'FAIL', 'accuracy_status':'PASS' if accuracy_pass else 'FAIL_FROZEN_PREREG_THRESHOLDS', 'accuracy':accuracy,
            'runtime_status':result['status'],'commands_per_arm':result['actual_effort_commands'],'robot_command_writes':result['robot_command_writes'],'physics_steps':result['explicit_physics_steps'],'frames':len(frames),
            'virtual_object_displacement_m':virtual_disp,'virtual_object_tracking_peak_m':max_obj,'virtual_object_tracking_peak_rotation_rad':max_rot,'relative_tcp_translation_peak_m':max_rel,'relative_tcp_rotation_peak_rad':max_rel_rot,'joint_tracking_peak_rad':max_q,'qp_hard_residual_peak':max_qp,'qdot_peak_rad_s':max_qdot,'native_dq_peak_rad_s':max_native_dq,
            'cube_displacement_m':cube_disp,'cube_unchanged':cube_disp<=1e-8,'unexpected_collision':first_bad,'suction_commands':result['suction_commands'],'D6_commands':result.get('virtual_execution',{}).get('D6_commands',0),'effort_peak_Nm':float(np.max(np.abs(efforts),initial=0.)) if efforts else None,'effort_rate_peak_Nm_s':float(max(rates,default=0.)),
            'virtual_tracking_records':records,'process_result':{'level1_source':result['level1_source'],'level2_source':result['level2_source'],'effective_backend':result.get('effective_backend'),'controller':result.get('controller')}}

def main():
    cells={name:eval_cell(name) for name in ('V10-A','V10-B','V10-C','V10-D','FORMAL') if (HERE/'evidence/diagnostic'/name/'result.json').exists()}
    (HERE/'evidence').mkdir(exist_ok=True); (HERE/'evidence/evaluation.json').write_text(json.dumps({'schema':'TASK28_R10_EVALUATION_V1','cells':cells,'acceptance':'independent all-sample virtual/object/Cube/FCL/process gates; no physical Cube or G1 closure promotion'},indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:v.get('status') for k,v in cells.items()},ensure_ascii=False))
if __name__=='__main__': main()
