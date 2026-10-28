"""Research A/B/C and conditional coupled D/E formulations.

Only A is eligible if the complete source gate fails. B/C never silently
discard finger coupling; D/E are mathematical-only without attachment sources.
"""
import numpy as np
from scipy.linalg import block_diag
from scipy.spatial.transform import Rotation
from .model import virtual_centres,centre_jdot,coupled_equalities
from .solver import hqp
from .profiles import full_joint_reference


def bounds(q,dq,M,h,previous,c,model):
    n=len(q); dt=c['physics_dt_s']; lo=np.r_[model.joint_lower,[0,0],model.joint_lower,[0,0]]
    hi=np.r_[model.joint_upper,[.04,.04],model.joint_upper,[.04,.04]]
    vel=np.tile(np.r_[model.manifest['joint_limits']['velocity_max_rad_s'],[.2,.2]],2)
    # Semi-implicit one-step prediction: dq+=dt*qdd; q+=dt*dq_next.
    low=np.maximum((-vel-dq)/dt,(lo-q-dt*dq)/dt**2)
    high=np.minimum((vel-dq)/dt,(hi-q-dt*dq)/dt**2)
    tau=np.tile(c['effort_bounds_per_robot'],2); tl=-tau; th=tau
    if previous is not None:
        rate=np.tile(c['effort_rate_bounds_per_s_per_robot'],2)*dt
        tl=np.maximum(tl,previous-rate); th=np.minimum(th,previous+rate)
    return np.r_[low,tl],np.r_[high,th]


def no_contact(kind,q,dq,M,h,ref,previous,c,model):
    qref,dref,ddref=full_joint_reference(ref)
    if kind=='A':
        tau=h['g']+np.tile(c['baseline_kp'],2)*(qref-q)+np.tile(c['baseline_kd'],2)*(dref-dq)
        return tau,{'priority_structure':'historical gravity+joint PD, no QP','qdd_model_prediction':np.linalg.solve(M,tau-h['g']-h['c']),'qp':[],'f_contact':None}
    if not h['source_full_pass']: raise ValueError('SOURCE_BLOCKED_FULL_18_DOF')
    total=h['g']+h['c']
    if kind=='B':
        dd=ddref+np.tile(c['joint_acceleration_kp'],2)*(qref-q)+np.tile(c['joint_acceleration_kd'],2)*(dref-dq)
        return M@dd+total,{'priority_structure':'independent joint-space computed torque','qdd_model_prediction':dd,'qp':[],'f_contact':None}
    if kind!='C': raise ValueError('UNKNOWN_NO_CONTACT_CONTROLLER')
    poses,B=virtual_centres(model,q); jd=centre_jdot(model,q,dq)
    if min(np.linalg.svd(b[:,:7],compute_uv=False)[-1] for b in B)<.01: raise ValueError('TASK_RANK_DEFICIENT')
    velocities=[b@v for b,v in zip(B,(dq[:9],dq[9:]))]; targets=[]
    for X,v in zip(poses,velocities):
        err=np.r_[ref['pose'][:3,3]-X[:3,3],Rotation.from_matrix(ref['pose'][:3,:3]@X[:3,:3].T).as_rotvec()]
        targets.append(ref['acceleration']+c['object_acceleration_kp']*err+c['object_acceleration_kd']*(ref['twist']-v))
    mean=.5*np.hstack(B); relative=np.hstack([B[0],-B[1]])
    erel=np.r_[poses[0][:3,3]-poses[1][:3,3],Rotation.from_matrix(poses[0][:3,:3]@poses[1][:3,:3].T).as_rotvec()]
    # Local geometric acceleration servo. For rotation, Log PD is explicitly a
    # small-error research servo, not claimed exact global log-error linearization.
    trel=-c['relative_acceleration_kp']*erel-c['relative_acceleration_kd']*(velocities[0]-velocities[1])-(jd[0]-jd[1])
    scale=np.diag([1]*3+[c['task_rotation_length_m']]*3)
    p1=np.c_[scale@mean,np.zeros((6,18))]; t1=scale@(.5*(targets[0]+targets[1]-jd[0]-jd[1]))
    p2=np.c_[scale@relative,np.zeros((6,18))]; t2=scale@trel
    # Tie-break effort, effort change and posture acceleration together at P3.
    p3=np.vstack([np.c_[.01*np.eye(18),np.zeros((18,18))],np.c_[np.zeros((18,18)),.001*np.eye(18)],np.c_[np.zeros((18,18)),.001*np.eye(18)]])
    t3=np.r_[.01*(ddref+100*(qref-q)+20*(dref-dq)),np.zeros(18),.001*(total if previous is None else previous)]
    E=np.c_[M,-np.eye(18)]; lower,upper=bounds(q,dq,M,h,previous,c,model)
    x,records=hqp(36,E,-total,np.eye(36),lower,upper,[(p1,t1),(p2,t2),(p3,t3)])
    if x is None: raise ValueError('QP_REJECTED:'+str(records))
    return x[18:],{'priority_structure':'lexicographic common acceleration > relative coordination > posture/effort/change','qdd_model_prediction':x[:18],'qp':records,'dynamics_equality_inf':float(np.max(np.abs(E@x+total))),'f_contact':None,'virtual_only':True}


def coupled_hqp(M,h,J,jdot,Mo,ho,G,H,hdot,aref,metric,limits,internal_reference=None,contact_constraints=None):
    """Mathematical D/E solver; caller must establish physical source gates.

P2 relative acceleration is redundant on the exact ideal rigid manifold.
Its explicit presence does not create an additional physical control freedom.
No calibrated suction restriction is supplied or inferred.
"""
    E,b=coupled_equalities(M,h,J,jdot,Mo,ho,G,H,hdot)
    p1=np.zeros((6,54)); p1[:,48:]=np.eye(6)
    invH=np.linalg.pinv(H); compatibility=J-H@invH@J
    p2=np.zeros((12,54)); p2[:,:18]=compatibility
    t2=-(np.eye(12)-H@invH)@(jdot-hdot)
    # Metric-orthogonal internal projection N=I-W^-1 G^T(... )^-1 G.
    W=np.asarray(metric,float); np.linalg.cholesky(W)
    Gi=np.linalg.solve(W,G.T)@np.linalg.inv(G@np.linalg.solve(W,G.T)); N=np.eye(12)-Gi@G
    rows=[]; targets=[]
    if internal_reference is not None:
        L=np.linalg.cholesky(W).T; p=np.zeros((12,54)); p[:,36:48]=L@N
        rows.append(p); targets.append(L@N@internal_reference)
    effort=np.zeros((18,54)); effort[:,18:36]=.01*np.eye(18); rows.append(effort); targets.append(np.zeros(18))
    accel=np.zeros((18,54)); accel[:,:18]=.001*np.eye(18); rows.append(accel); targets.append(np.zeros(18))
    C=np.eye(54); lower,upper=limits
    if contact_constraints is not None:
        Cf,lf,uf=contact_constraints; extra=np.zeros((len(lf),54)); extra[:,36:48]=Cf
        C=np.vstack([C,extra]); lower=np.r_[lower,lf]; upper=np.r_[upper,uf]
    return hqp(54,E,b,C,lower,upper,[(p1,aref),(p2,t2),(np.vstack(rows),np.concatenate(targets))])
