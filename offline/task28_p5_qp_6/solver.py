"""Lexicographic convex least-squares QPs, OSQP, independent KKT gate.

After each priority, freeze its achieved task vector A*x. For convex least
squares that vector is unique over the convex minimizer set (strict convexity
in task space). Thus these equalities preserve the entire preceding optimum.
There is no epsilon regularizer or silent equality relaxation.
"""
import time
import numpy as np
from scipy import sparse
import osqp


def qp(P,c,A,l,u,tol=2e-7):
    started=time.perf_counter(); P=np.asarray(P,float); c=np.asarray(c,float)
    A=np.asarray(A,float); l=np.asarray(l,float); u=np.asarray(u,float)
    if not all(np.isfinite(v).all() for v in (P,c,A)) or np.isnan(l).any() or np.isnan(u).any() or np.any(l>u):
        return None,{'status':'INPUT_REJECTED'}
    if not np.allclose(P,P.T,atol=1e-10,rtol=0) or np.linalg.eigvalsh(P).min() < -1e-9:
        return None,{'status':'NONCONVEX_REJECTED'}
    backend=osqp.OSQP()
    backend.setup(P=sparse.csc_matrix(np.triu(P)),q=c,A=sparse.csc_matrix(A),l=l,u=u,
                  verbose=False,eps_abs=1e-9,eps_rel=1e-9,max_iter=20000,polishing=True,
                  adaptive_rho=True,scaled_termination=False)
    r=backend.solve(raise_error=False)
    d={'status':r.info.status,'iterations':int(r.info.iter),'backend_primal_residual':float(r.info.prim_res),
       'backend_dual_residual':float(r.info.dual_res),'solve_wall_s':time.perf_counter()-started,'osqp_version':osqp.__version__}
    if r.info.status_val!=1 or r.x is None or not np.isfinite(r.x).all(): return None,d
    x=r.x; y=r.y; z=A@x
    feasibility=max(float(np.max(np.maximum(l-z,0),initial=0)),float(np.max(np.maximum(z-u,0),initial=0)))
    stationarity=float(np.max(np.abs(P@x+c+A.T@y),initial=0))
    # OSQP dual convention: positive upper-bound, negative lower-bound.
    comp=[]
    for j in range(len(y)):
        if y[j]>0: comp.append(abs(y[j]*(z[j]-u[j])) if np.isfinite(u[j]) else abs(y[j]))
        elif y[j]<0: comp.append(abs(y[j]*(z[j]-l[j])) if np.isfinite(l[j]) else abs(y[j]))
    complementarity=max(comp,default=0.)
    d.update(feasibility_inf=feasibility,stationarity_inf=stationarity,complementarity_inf=float(complementarity),objective=float(.5*x@P@x+c@x))
    if max(feasibility,stationarity,complementarity)>tol:
        d['status']='INDEPENDENT_KKT_REJECTED'; return None,d
    return x,d


def hqp(n,E,b,C,lower,upper,tasks):
    """Tasks are (A,target). Hard constraints C*x in [lower,upper]."""
    E=np.asarray(E,float).reshape(-1,n); b=np.asarray(b,float)
    C=np.asarray(C,float).reshape(-1,n); lower=np.asarray(lower,float); upper=np.asarray(upper,float)
    records=[]; achieved=[]; x=None
    for priority,(A,target) in enumerate(tasks,1):
        A=np.asarray(A,float).reshape(-1,n); target=np.asarray(target,float)
        x,d=qp(A.T@A,-A.T@target,np.vstack([E,C]),np.r_[b,lower],np.r_[b,upper])
        d['priority']=priority; records.append(d)
        if x is None: return None,records
        achieved.append((A,A@x)); E=np.vstack([E,A]); b=np.r_[b,A@x]
    if x is None: return None,[{'status':'NO_TASKS_REJECTED'}]
    if any(np.max(np.abs(A@x-v),initial=0)>2e-7 for A,v in achieved):
        return None,records+[{'status':'PRIORITY_PRESERVATION_REJECTED'}]
    return x,records
