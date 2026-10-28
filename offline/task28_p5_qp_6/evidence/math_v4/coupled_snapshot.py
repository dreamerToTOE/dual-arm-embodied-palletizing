"""Final offline coupled HQP assembly, unavailable as a physical actuator sink.

The original controllers.py prototype is frozen as a formal-run dependency.
This final mathematical assembly adds explicit effort-change regularization
and validates model rank/metric independently before solving synthetic cases.
"""
import numpy as np
import osqp
from scipy import sparse
import time
from .model import coupled_equalities
from .solver import hqp


def solve(M,h,J,jdot,Mo,ho,G,H,hdot,aref,W,lower,upper,previous_tau,internal_ref=None,contact_feasibility=None):
    arrays=[np.asarray(v,float) for v in [M,h,J,jdot,Mo,ho,G,H,hdot,aref,W,lower,upper,previous_tau]]
    if not all(np.isfinite(v).all() for v in arrays): return None,{'status':'NONFINITE_MODEL_REJECTED'}
    M,h,J,jdot,Mo,ho,G,H,hdot,aref,W,lower,upper,previous_tau=arrays
    if M.shape!=(18,18) or Mo.shape!=(6,6) or G.shape!=(6,12) or H.shape!=(12,6) or W.shape!=(12,12) or J.shape!=(12,18): return None,{'status':'MODEL_SHAPE_REJECTED'}
    for a in [M,Mo,W]:
        if not np.allclose(a,a.T,atol=1e-10,rtol=0) or np.linalg.eigvalsh(a).min()<=0: return None,{'status':'NON_SPD_MODEL_REJECTED'}
    if np.linalg.matrix_rank(G)!=6 or np.linalg.matrix_rank(H)!=6 or np.linalg.matrix_rank(J)!=12: return None,{'status':'RANK_DEFICIENT_MODEL_REJECTED'}
    if not np.allclose(G,H.T,rtol=0,atol=1e-9): return None,{'status':'VIRTUAL_POWER_REJECTED'}
    if previous_tau.shape!=(18,) or lower.shape!=(54,) or upper.shape!=(54,) or np.any(lower>upper): return None,{'status':'LIMITS_REJECTED'}
    E,b=coupled_equalities(M,h,J,jdot,Mo,ho,G,H,hdot)
    p1=np.zeros((6,54)); p1[:,48:]=np.eye(6)
    invH=np.linalg.pinv(H); projector=np.eye(12)-H@invH
    p2=np.zeros((12,54)); p2[:,:18]=projector@J; t2=-projector@(jdot-hdot)
    # At an exact rigid grasp P2 lies in the hard-constraint row space; its
    # optimum is already fixed. Never soften contact to manufacture freedom.
    WGi=np.linalg.solve(W,G.T); Gi=WGi@np.linalg.solve(G@WGi,np.eye(6)); N=np.eye(12)-Gi@G
    rows=[]; targets=[]
    if internal_ref is not None:
        internal_ref=np.asarray(internal_ref,float)
        if internal_ref.shape!=(12,) or not np.isfinite(internal_ref).all(): return None,{'status':'INTERNAL_REFERENCE_REJECTED'}
        P=np.zeros((12,54)); L=np.linalg.cholesky(W).T; P[:,36:48]=L@N; rows.append(P); targets.append(L@N@internal_ref)
    for factor,target in [(1e-2,np.zeros(18)),(1e-2,previous_tau)]:
        P=np.zeros((18,54)); P[:,18:36]=factor*np.eye(18); rows.append(P); targets.append(factor*target)
    # A tiny objective at P3 is an explicit tie-break, not P1/P2 regularization.
    P=np.zeros((18,54)); P[:,:18]=1e-3*np.eye(18); rows.append(P); targets.append(np.zeros(18))
    C=np.eye(54); l=lower.copy(); u=upper.copy()
    if contact_feasibility is not None:
        Cf,lf,uf=map(lambda x:np.asarray(x,float),contact_feasibility)
        if Cf.ndim!=2 or Cf.shape[1]!=12 or not np.isfinite(Cf).all(): return None,{'status':'CONTACT_CONSTRAINT_REJECTED'}
        P=np.zeros((Cf.shape[0],54)); P[:,36:48]=Cf; C=np.vstack([C,P]); l=np.r_[l,lf]; u=np.r_[u,uf]
    A3=np.vstack(rows); t3=np.concatenate(targets)
    x,records=hqp(54,E,b,C,l,u,[(p1,aref),(p2,t2),(A3,t3)])
    # Full-scale wrench metrics make weak P3 acceleration directions sensitive
    # to an absolute dual tolerance. Refine P3 at stricter numerical precision,
    # retaining exactly the achieved P1/P2 task vectors. No weights are changed.
    refinement=None
    if x is not None:
        tick=time.perf_counter(); A=np.vstack([E,p1,p2,C]); low=np.r_[b,p1@x,p2@x,l]; high=np.r_[b,p1@x,p2@x,u]
        P=A3.T@A3; linear=-A3.T@t3; backend=osqp.OSQP()
        backend.setup(P=sparse.csc_matrix(np.triu(P)),q=linear,A=sparse.csc_matrix(A),l=low,u=high,verbose=False,eps_abs=1e-12,eps_rel=1e-12,max_iter=100000,polishing=True)
        backend.warm_start(x=x); r=backend.solve(raise_error=False)
        if r.info.status_val==1 and np.isfinite(r.x).all():
            z=A@r.x; violation=max(float(np.max(np.maximum(low-z,0))),float(np.max(np.maximum(z-high,0))))
            stationarity=float(np.max(np.abs(P@r.x+linear+A.T@r.y)))
            # All limits here are finite; independently check bound-side dual
            # complementarity as well as equality feasibility and stationarity.
            complementarity=max((abs(y*(zz-(uu if y>=0 else ll))) for y,zz,ll,uu in zip(r.y,z,low,high)),default=0.)
            refinement={'status':r.info.status,'iterations':int(r.info.iter),'feasibility_inf':violation,'stationarity_inf':stationarity,'complementarity_inf':float(complementarity),'solve_wall_s':time.perf_counter()-tick,'eps_abs':1e-12,'eps_rel':1e-12}
            if max(violation,stationarity,complementarity)<=2e-7: x=r.x
            else: x=None
        else:
            refinement={'status':r.info.status}; x=None
    diagnostics={'status':'MATH_ACCEPTED' if x is not None else 'QP_REJECTED','priorities':records,'rank_G':int(np.linalg.matrix_rank(G)),'rank_J':int(np.linalg.matrix_rank(J)),'rank_equalities':int(np.linalg.matrix_rank(E)),'metric':W,'internal_projection':N,'physical_contact_established':False,'contact_feasibility_label':'IDEAL_UNBOUNDED_6D_CONTACT' if contact_feasibility is None else 'DECLARED_SYNTHETIC_LINEAR_CONTACT_CONSTRAINTS_NOT_CALIBRATED_SUCTION'}
    diagnostics['priority3_numerical_refinement']=refinement
    if x is not None:
        f=x[36:48]; diagnostics.update(equality_residual_inf=float(np.max(np.abs(E@x-b))),grasp_map_residual_inf=float(np.max(np.abs(G@f-Mo@x[48:]-ho))),internal_prediction=N@f,internal_null_residual=float(np.linalg.norm(G@N@f)))
    return x,diagnostics
