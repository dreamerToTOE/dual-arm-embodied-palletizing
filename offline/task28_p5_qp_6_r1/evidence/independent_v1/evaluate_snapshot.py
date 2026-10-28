"""Source-isolated evaluator: analytic reaction decomposition and active faces.

No call to production evaluate_R_step, constrained_prediction, qp or old QP6
evaluator. Projected effort remains an unused evaluator field, not a lambda.
"""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np
from .common import HERE,dump,sha,config


def active_face_oracle(M,h,tau,q,dq,dt,lower,upper):
    J=np.zeros(9); J[7:]=[-1,1]
    b0=-(dq[8]-dq[7])/dt-.9*(q[8]-q[7])/dt**2
    low=(np.asarray(lower)-q[7:]-dt*dq[7:])/dt**2
    high=(np.asarray(upper)-q[7:]-dt*dq[7:])/dt**2
    feasible=[]
    for face in itertools.product([-1,0,1],repeat=2):
        rows=[J]; b=[b0]
        for i,f in enumerate(face):
            if f:
                row=np.zeros(9);row[7+i]=1;rows.append(row);b.append(low[i] if f<0 else high[i])
        E=np.asarray(rows)
        if np.linalg.matrix_rank(E)!=len(rows): continue
        K=np.block([[M,-E.T],[E,np.zeros((len(rows),len(rows)))]])
        sol=np.linalg.solve(K,np.r_[tau-h,b]); a=sol[:9]; lam=sol[9:]
        if np.any(a[7:]<low-1e-8) or np.any(a[7:]>high+1e-8): continue
        j=1; sign=True
        for f in face:
            if f:
                if (f<0 and lam[j]<-1e-8) or (f>0 and lam[j]>1e-8): sign=False
                j+=1
        if sign: feasible.append((float(.5*a@M@a+(h-tau)@a),a,face))
    if not feasible: return None
    return min(feasible,key=lambda t:t[0])[1]


def independent_R(pre,post,tau,requested,dt,c):
    q=np.asarray(pre['q']);v=np.asarray(pre['dq']); qp=np.asarray(post['q']); vp=np.asarray(post['dq']);M=np.asarray(pre['M']);h=np.asarray(pre['c'])+np.asarray(pre['g'])
    dd=(vp-v)/dt; residual=M@dd+h-tau; upper=np.asarray(pre['finger_upper']); active=upper-q[7:]<=c['near_upper_m']
    normal=max(0.,-float(residual[7]+residual[8])) if np.any(active) else 0.
    # Pick one supported upper normal. Multiple active normals are redundant,
    # so this assignment is an algebraic witness, not a measured force split.
    upper_force=np.zeros(2)
    if np.any(active): upper_force[np.flatnonzero(active)[0]]=normal
    lm=(residual[8]+upper_force[1]-residual[7]-upper_force[0])/2
    fitted=np.array([-lm,lm])-upper_force; unexplained=residual[7:]-fitted
    checks={
      'ARM_ACCELERATION_ERROR':np.max(abs(dd[:7]-requested))<=c['arm_acceleration_error_rad_s2'],
      'ARM_EQUATION_WITH_COUPLING':np.max(abs(residual[:7]))<=c['arm_residual_Nm'],
      'FINGER_POLICY_POSITION':np.max(abs(qp[7:]-.04))<=c['finger_hold_position_m'],
      'FINGER_POLICY_VELOCITY':np.max(abs(vp[7:]))<=c['finger_hold_velocity_m_s'],
      'MIMIC_POSITION':abs(qp[8]-qp[7])<=c['mimic_position_m'],
      'MIMIC_VELOCITY':abs(vp[8]-vp[7])<=c['mimic_velocity_m_s'],
      'MIMIC_ACCELERATION':abs(dd[8]-dd[7])<=c['finger_free_acceleration_error_m_s2'],
      'STRUCTURAL_ARM_FINGER_CANCELLATION':np.max(abs(M[:7,7]+M[:7,8]))<=c['M_af_common_bound'],
      'ACTUAL_ARM_FINGER_COUPLING_TERM':np.max(abs(M[:7,7:]@dd[7:]))<=c['arm_coupling_term_Nm'],
      'CONSTRAINT_RESIDUAL_STRUCTURE':np.max(abs(unexplained))<=c['finger_reaction_unexplained_N'],
      'UNILATERAL_GAP_COMPLEMENTARITY':np.max(abs(upper_force*(upper-qp[7:])))<=c['gap_complementarity_Nm']}
    return {'valid':all(checks.values()),'checks':checks,'observed_qdd':dd,'free_residual':residual,'upper_candidates':active,'upper_gap_pre_m':upper-q[7:],'upper_gap_post_m':upper-qp[7:],'mimic_force_algebraic_witness_N':lm,'upper_force_algebraic_witness_N':upper_force,'unexplained_finger_residual_N':unexplained,'arm_finger_coupling_term':M[:7,7:]@dd[7:],'M_af_common_column':M[:7,7]+M[:7,8]}


def evaluate_configuration(cfg):
    c=config(); folder=HERE/'evidence'/('source_'+cfg+'_v2'); raw=json.loads((folder/'raw.json').read_text())['rows']; result=json.loads((folder/'result.json').read_text()); records=[]; first=None
    for k,row in enumerate(raw):
        if not row['executed']: continue
        for side in ['left','right']:
            pre=row['pre'][side];post=row['post'][side];info=row['arms'][side]
            M=np.asarray(pre['M']);h=np.asarray(pre['c'])+pre['g'];tau=np.asarray(info['applied_command_readback']); requested=np.asarray(info['requested_free_qdd']);dt=post['time_s']-pre['time_s']
            q=np.asarray(pre['q']);v=np.asarray(pre['dq']);obs=(np.asarray(post['dq'])-v)/dt
            oracle=active_face_oracle(M,h,np.asarray(info['proposed_effort']),q,v,result['actual_physics_dt_s'],pre['finger_lower'],pre['finger_upper'])
            stored=info['K_candidate_prediction']; odiff=None if oracle is None or stored is None else float(np.max(abs(oracle-np.asarray(stored))))
            F=bool(np.max(abs(obs[:7]-requested[:7]))<=c['arm_acceleration_error_rad_s2'] and np.max(abs(obs[7:]-requested[7:]))<=c['finger_free_acceleration_error_m_s2'])
            K=bool(oracle is not None and np.max(abs(obs[:7]-oracle[:7]))<=c['arm_acceleration_error_rad_s2'] and np.max(abs(obs[7:]-oracle[7:]))<=c['finger_free_acceleration_error_m_s2'])
            R=independent_R(pre,post,tau,requested[:7],dt,c)
            temporal=(post['physics_step']==pre['physics_step']+1 and abs(dt-c['physics_dt_s'])<1e-9)
            if cfg in 'AB': valid=F
            elif cfg=='C': valid=R['valid']
            else: valid=K
            valid=bool(valid and temporal and row['post_safety_gate']['valid'] and np.max(abs(tau-info['proposed_effort']))<=c['command_buffer_tolerance'])
            rec={'row':k,'side':side,'physics_step_post':post['physics_step'],'post_time_s':post['time_s'],'F_pass':F,'K_prediction_pass':K,'K_oracle_difference_inf':odiff,'F_acceleration_error':obs-requested,'K_acceleration_error':None if oracle is None else obs-oracle,'R_independent':R,'registered_configuration_gate_valid':valid,'temporal_valid':temporal,'free_requested_mimic_compatibility':float(requested[8]-requested[7]),'source_identity':pre['identity']}
            records.append(rec)
            if not valid and first is None: first={**rec,'exact_raw_frame':row}
    expected=config()['source_configs'][cfg]['maximum_controlled_steps']; complete=result['controlled_steps']==expected and first is None
    all_oracle=[r['K_oracle_difference_inf'] for r in records if r['K_oracle_difference_inf'] is not None]
    if max(all_oracle,default=0)>2e-6: raise ValueError('INDEPENDENT_QP_ORACLE_MISMATCH')
    return {'configuration':cfg,'identity':result['identity'],'status':'COMPLETE' if complete else 'REJECTED','controlled_steps':result['controlled_steps'],'initialization_steps':result['initialization_physics_steps'],'expected_steps':expected,'unexecuted_steps':expected-result['controlled_steps'],'first_failure':first,'first_failure_is_last_recorded_step':first is None or first['row']==len(raw)-1,'maximum_K_oracle_difference_inf':max(all_oracle,default=0),'F_fail_records':sum(not r['F_pass'] for r in records),'K_fail_records':sum(not r['K_prediction_pass'] for r in records),'peak_F_acceleration_error_per_dof':np.max(abs([r['F_acceleration_error'] for r in records]),axis=0),'peak_arm_residual_Nm':max(np.max(abs(r['R_independent']['free_residual'][:7])) for r in records),'peak_M_af_common_column':max(np.max(abs(r['R_independent']['M_af_common_column'])) for r in records),'peak_actual_arm_finger_coupling_Nm':max(np.max(abs(r['R_independent']['arm_finger_coupling_term'])) for r in records),'raw_sha':sha(folder/'raw.json'),'records':records}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cases={}
    for cfg in 'ABCD':
        r=evaluate_configuration(cfg);cases[cfg]=r;dump(a.output/('configuration_'+cfg+'.json'),r)
    eligible=cases['C']['status']=='COMPLETE' and cases['D']['status']=='COMPLETE'
    source={'status':'ARM14_REDUCED_DYNAMICS_VALIDATED_WITH_FINGER_POLICY' if eligible else 'SOURCE_PARTIAL_FORMAL_BLOCKED','original_unconstrained_full18_status':'UNCONSTRAINED_FULL_18D_SOURCE_STILL_BLOCKED','full18_constrained_status':'NOT_VALIDATED_FOR_ORIGINAL_CONFIGURATION','arm14_reduced_status':'QUALIFIED_WITH_DECLARED_POLICY' if eligible else 'NOT_QUALIFIED_REGISTERED_C_GATE_FAILED','formal_eligible':eligible,'required_configurations':['C','D'],'source_configuration_statuses':{k:{key:v[key] for key in ['status','controlled_steps','expected_steps','F_fail_records','K_fail_records','maximum_K_oracle_difference_inf','peak_arm_residual_Nm','peak_M_af_common_column','peak_actual_arm_finger_coupling_Nm']} for k,v in cases.items()},'finger_constraint_reaction_source':'No independent reaction sensor; algebraic fits are not physical observations.','original_finger_target_m':.04,'interior_B_D_never_promoted_to_original':True}
    dump(a.output/'source_qualification.json',source)
    cells=[{'controller':ctrl,'profile':p,'status':'ELIGIBLE_NOT_RUN' if eligible else 'SKIPPED_WITH_REASON','reason':None if eligible else 'Shared original-configuration source C failed CONSTRAINT_RESIDUAL_STRUCTURE at controlled step140; no validated applicable full/reduced source.','moving_steps':0,'final_hold_steps':0,'first_invalid_gate':'SOURCE_PREREQUISITE' if not eligible else None,'tracking_errors':None,'efforts_and_rates':None,'dynamic_constraint_residuals':None,'HQP_priority_and_solver_wall_time':None,'cooperative_benefit':None,'Cube_D6_contact':False} for ctrl in ['B','C'] for p in ['S05','S10']]
    dump(a.output/'formal_matrix.json',cells);print(source)


if __name__=='__main__':main()
