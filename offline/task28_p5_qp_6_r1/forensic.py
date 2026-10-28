"""Independent reconstruction; no import of QP-6 evaluator/controller."""
import argparse
from pathlib import Path
import json
import numpy as np
from .common import ROOT,HERE,dump,sha


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True); args=ap.parse_args()
    old=ROOT/'offline/task28_p5_qp_6/evidence/source_v1/observations.json'
    data=json.loads(old.read_text())['rows']; details=[]; first=None
    upper=float(np.float32(.04)); Jm=np.zeros((1,9)); Jm[0,7:]=[-1,1]
    # Full-rank basis for mimic plus symmetric upper-stop reaction. Splitting
    # the latter between two redundant stops is not uniquely identifiable.
    Ju=np.zeros((1,9)); Ju[0,7:]=[-1,-1]; J=np.vstack([Jm,Ju])
    N=np.zeros((9,8)); N[:7,:7]=np.eye(7); N[7:,7]=1/np.sqrt(2)
    for k,row in enumerate(data):
        dt=row['sim_time_post']-row['sim_time_pre']
        for side,a in row['arms'].items():
            q=np.array(a['q_pre']); dq=np.array(a['dq_pre']); postq=np.array(a['q_post']); postv=np.array(a['dq_post'])
            dd=(postv-dq)/dt; req=np.array(a['qdd_requested']); M=np.array(a['M_pre']); h=np.array(a['c_pre'])+np.array(a['g_pre']); applied=np.array(a['applied_command_readback']); proposed=np.array(a['effort_proposed'])
            residual=M@dd+h-applied
            fit=np.linalg.lstsq(J.T,residual,rcond=1e-12)[0]
            mimic=np.linalg.lstsq(Jm.T,residual,rcond=1e-12)[0]
            v32=np.r_[dq,postv].astype(np.float32); velerr=float(np.max(np.abs(np.spacing(v32))))/dt*2
            r={'row':k,'side':side,'physics_step_post':row['physics_step_post'],'pre_time_s':row['sim_time_pre'],'post_time_s':row['sim_time_post'],'dt_s':dt,'q_pre':q,'q_post':postq,'dq_pre':dq,'dq_post':postv,'requested_qdd':req,'observed_qdd':dd,'acceleration_error':dd-req,'M':M,'c':a['c_pre'],'g':a['g_pre'],'applied_command_readback':applied,'proposed':proposed,'solver_projected_effort_not_reaction_sensor':a['solver_projected_effort'],'free_residual_using_applied':residual,'free_residual_using_proposed':M@dd+h-proposed,'arm_finger_mass_coupling':M[:7,7:],'finger_arm_mass_coupling':M[7:,:7],'finger_mass_block':M[7:,7:],'upper_limit_m':upper,'pre_upper_gap_m':upper-q[7:],'post_upper_gap_m':upper-postq[7:],'mimic_q_residual_m':q[8]-q[7],'mimic_dq_residual_m_s':dq[8]-dq[7],'mimic_acceleration_residual_m_s2':dd[8]-dd[7],'mimic_only_unexplained_residual':residual-Jm.T@mimic,'permitted_mimic_motion_residual':N.T@residual,'mimic_plus_upper_fitted_force_coefficients_N':fit,'mimic_plus_upper_unexplained_residual':residual-J.T@fit,'upper_reaction_sign_compatible':bool(fit[1]>=0),'fitted_upper_gap_products_Nm':fit[1]*(upper-postq[7:]),'fitted_upper_gap_acceleration_products':fit[1]*(-dd[7:]),'velocity_quantization_acceleration_bound':velerr,'position_quantization_acceleration_bound_m_s2':float(4*np.spacing(np.float32(.04))/dt**2),'model_finger_coupling_term_in_arm_equation':M[:7,7:]@dd[7:],'constraint_fit_status':'ALGEBRAIC_COMPATIBILITY_ONLY_NOT_MEASURED_LAMBDA_OR_COMPLEMENTARITY_VALIDATION'}
            details.append(r)
            if first is None and (np.max(np.abs(dd[:7]-req[:7]))>.02 or np.max(np.abs(dd[7:]-req[7:]))>.002): first=r
    assert first['row']==18 and first['side']=='right' and first['physics_step_post']==21
    result={'status':'HISTORICAL_FREE_DYNAMICS_FAILURE_REPRODUCED','input_sha':sha(old),'first_failure':first,'frames_examined':len(data),'mimic_constraint_jacobian':Jm,'upper_symmetric_basis':Ju,'permitted_basis':N,'mimic_offset_assumption':'Frozen schema default 0; independently verify via installed schema before source experiments','inference':'Mimic alone cannot explain the symmetric finger residual. Adding upper-stop direction algebraically explains finger residual but does not validate causal reaction, active set or complementarity.','historical_status_preserved':'UNCONSTRAINED_FULL_18D_SOURCE_STILL_BLOCKED'}
    args.output.mkdir(parents=True,exist_ok=False); dump(args.output/'all_frames.json',details); dump(args.output/'summary.json',result)
    print(first['row'],first['side'],first['observed_qdd'][7:],first['free_residual_using_applied'][7:])


if __name__=='__main__': main()
