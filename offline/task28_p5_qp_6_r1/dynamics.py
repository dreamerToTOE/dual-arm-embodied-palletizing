"""Constraint signs, reduced virtual work and bounded source qualifications.

R is an arm subsystem with a retained physical finger policy, not a claim that
all nine free accelerations are valid. F never changes its historical gates.
"""
import numpy as np
from offline.task28_p5_qp_6.solver import qp


def mimic_basis():
    S=np.zeros((9,8)); S[:7,:7]=np.eye(7); S[7:,7]=1.
    J=np.zeros((1,9)); J[0,7:]=[-1,1]
    return S,J


def reduced_mimic_model(M,h):
    S,J=mimic_basis()
    return S.T@M@S,S.T@h,S,J


def constrained_prediction(M,h,tau,q,dq,dt,lower,upper):
    """Discrete mimic correction plus unilateral next-position bounds.

Mimic ERP=.9 is from installed TGS test example. This is a candidate solver
model tested independently, never presumed identical to the PhysX constraint
iteration/warm-start model. Finger limits are inequalities, not permanent locks.
"""
    _,J=mimic_basis(); b=-float(J@dq)/dt-.9*float(J@q)/dt**2
    C=np.zeros((2,9)); C[:,7:]=np.eye(2)
    lo=(np.asarray(lower)-q[7:]-dt*dq[7:])/dt**2
    hi=(np.asarray(upper)-q[7:]-dt*dq[7:])/dt**2
    A=np.vstack([J,C]); l=np.r_[b,lo]; u=np.r_[b,hi]
    a,diag=qp(M,h-tau,A,l,u)
    return a,{'solver':diag,'J_mimic':J,'mimic_acceleration_rhs':b,'finger_acceleration_lower':lo,'finger_acceleration_upper':hi}


def arm_effort_with_fingers(M,cc,g,q,dq,arm_acceleration,c,finger_target=.04):
    """R policy: refreshed full M/h; explicit finger force servo, all 9 outputs.

The arm equation retains its coupling term in every evaluator. Omitting it
from the command is eligible only under the validated mimic cancellation bound.
"""
    tau=np.empty(9); tau[:7]=M[:7,:7]@arm_acceleration+cc[:7]+g[:7]
    tau[7:]=g[7:]+c['finger_kp_N_m']*(finger_target-q[7:])-c['finger_kd_Ns_m']*dq[7:]
    return tau


def reaction_structure(M,cc,g,applied,q,dq,qpost,dqpost,dt,upper,c):
    a=(dqpost-dq)/dt; r=M@a+cc+g-applied; S,Jm=mimic_basis()
    active=np.asarray(upper)-q[7:]<=c['near_upper_m']
    rows=[Jm[0]]
    for k in np.flatnonzero(active):
        row=np.zeros(9); row[7+k]=-1; rows.append(row)
    J=np.vstack(rows)
    # Enumerate which unilateral force coefficients are positive. Mimic force
    # is signed; no reaction is read from get_measured_joint_efforts.
    best=None
    import itertools
    for mask in itertools.product([False,True],repeat=len(rows)-1):
        ids=[0]+[j+1 for j,on in enumerate(mask) if on]
        lam=np.zeros(len(rows)); lam[ids]=np.linalg.lstsq(J[ids].T,r,rcond=1e-12)[0]
        if np.any(lam[1:]<-1e-12): continue
        diff=r-J.T@lam; score=np.linalg.norm(diff)
        if best is None or score<best[0]: best=(score,lam,diff)
    _,lam,unexplained=best
    gap_post=np.asarray(upper)-qpost[7:]
    comp=[abs(lam[j+1]*gap_post[k]) for j,k in enumerate(np.flatnonzero(active))]
    return {'observed_qdd':a,'free_residual':r,'mimic_only_permitted_residual':S.T@r,'candidate_constraint_J':J,'algebraically_fitted_lambda_not_sensor':lam,'constraint_unexplained_residual':unexplained,'upper_gap_products_Nm':comp,'upper_active_candidates':active,'M_af_common_column':M[:7,7:].sum(axis=1),'full_arm_finger_inertia_term':M[:7,7:]@a[7:],'mimic_position_residual_m':qpost[8]-qpost[7],'mimic_velocity_residual_m_s':dqpost[8]-dqpost[7],'mimic_acceleration_residual_m_s2':a[8]-a[7],'status':'REACTION_STRUCTURE_COMPATIBILITY_NOT_MEASURED_REACTION'}


def evaluate_R_step(pre,post,command,arm_requested,dt,upper,c):
    q=np.asarray(pre['q']); dq=np.asarray(pre['dq']); qpost=np.asarray(post['q']); vpost=np.asarray(post['dq'])
    M=np.asarray(pre['M']); cc=np.asarray(pre['c']); g=np.asarray(pre['g'])
    r=reaction_structure(M,cc,g,np.asarray(command),q,dq,qpost,vpost,dt,upper,c)
    aa=r['observed_qdd']; reasons=[]
    tests={
      'ARM_ACCELERATION_ERROR':np.max(np.abs(aa[:7]-arm_requested))<=c['arm_acceleration_error_rad_s2'],
      'ARM_EQUATION_WITH_COUPLING':np.max(np.abs(r['free_residual'][:7]))<=c['arm_residual_Nm'],
      'FINGER_POLICY_POSITION':np.max(np.abs(qpost[7:]-.04))<=c['finger_hold_position_m'],
      'FINGER_POLICY_VELOCITY':np.max(np.abs(vpost[7:]))<=c['finger_hold_velocity_m_s'],
      'MIMIC_POSITION':abs(r['mimic_position_residual_m'])<=c['mimic_position_m'],
      'MIMIC_VELOCITY':abs(r['mimic_velocity_residual_m_s'])<=c['mimic_velocity_m_s'],
      'MIMIC_ACCELERATION':abs(r['mimic_acceleration_residual_m_s2'])<=c['finger_free_acceleration_error_m_s2'],
      'STRUCTURAL_ARM_FINGER_CANCELLATION':np.max(np.abs(r['M_af_common_column']))<=c['M_af_common_bound'],
      'ACTUAL_ARM_FINGER_COUPLING_TERM':np.max(np.abs(r['full_arm_finger_inertia_term']))<=c['arm_coupling_term_Nm'],
      'CONSTRAINT_RESIDUAL_STRUCTURE':np.max(np.abs(r['constraint_unexplained_residual'][7:]))<=c['finger_reaction_unexplained_N'],
      'UNILATERAL_GAP_COMPLEMENTARITY':max(r['upper_gap_products_Nm'],default=0)<=c['gap_complementarity_Nm']}
    reasons=[k for k,v in tests.items() if not v]
    return {'valid':not reasons,'reasons':reasons,'checks':tests,'diagnostics':r,'scope':'ACTIVE_ARM_REDUCTION_WITH_OBSERVED_FINGER_POLICY_NOT_FULL_9D_FREE_ACCELERATION_PASS'}
