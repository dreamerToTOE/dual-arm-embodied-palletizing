"""Synthetic coupled HQP versus independent null-space least-squares oracles."""
import argparse
from pathlib import Path
import time
import numpy as np
from scipy.linalg import block_diag, null_space
from scipy.spatial.transform import Rotation
from .common import dump
from .model import contact_maps,object_dynamics
from .coupled import solve
from offline.task28_p2_1.grasp import GraspModel,metric_matrix


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--output',type=Path,required=True); a=ap.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
    rng=np.random.default_rng(280600); records=[]
    for k in range(12):
        T=np.eye(4); T[:3,:3]=Rotation.from_rotvec(rng.normal(size=3)*.3).as_matrix()
        H,hd=contact_maps(T,np.zeros(6)); G=GraspModel.frozen(T).matrix()
        J=block_diag(np.c_[np.eye(6),np.zeros((6,3))],np.c_[np.eye(6),np.zeros((6,3))])
        M=np.diag(rng.uniform(.5,2,18)); h=rng.normal(size=18)*.1
        Mo,ho=object_dynamics(T,np.zeros(6),.8,np.eye(3)*.00192)
        W=metric_matrix(); target=rng.normal(size=6)*.1; previous=np.zeros(18)
        internal=GraspModel.frozen(T).decompose(rng.normal(size=12),W).internal
        started=time.perf_counter()
        x,d=solve(M,h,J,np.zeros(12),Mo,ho,G,H,hd,target,W,np.full(54,-100.),np.full(54,100.),previous,internal_ref=internal)
        if x is None: raise ValueError('SYNTHETIC_QP_REJECTED:'+str(d))
        wall=time.perf_counter()-started
        # Oracle equations are assembled directly from three physical balances.
        E=np.block([[M,-np.eye(18),J.T,np.zeros((18,6))],
                    [np.zeros((6,18)),np.zeros((6,18)),-G,Mo],
                    [J,np.zeros((12,18)),np.zeros((12,12)),-H]])
        b=np.r_[-h,-ho,np.zeros(12)]
        A1=np.zeros((6,54)); A1[:,48:]=np.eye(6)
        # Hard contact makes relative acceleration an exact redundant task.
        U=np.eye(12)-H@np.linalg.pinv(H); A2=np.zeros((12,54)); A2[:,:18]=U@J
        # Independent metric decomposition with a direct saddle-system solve.
        saddle=np.block([[W,G.T],[G,np.zeros((6,6))]])
        Gi=np.linalg.solve(saddle,np.vstack([np.zeros((12,6)),np.eye(6)]))[:12]
        N=np.eye(12)-Gi@G; L=np.linalg.cholesky(W).T
        A3=np.zeros((66,54)); A3[:12,36:48]=L@N; A3[12:30,18:36]=.01*np.eye(18); A3[30:48,18:36]=.01*np.eye(18); A3[48:,:18]=.001*np.eye(18)
        t3=np.r_[L@internal,np.zeros(54)]
        oracle_tasks=[(A1,target),(A2,np.zeros(12)),(A3,t3)]
        Ee=E.copy(); bb=b.copy(); oracles=[]
        for A,t in oracle_tasks:
            # Do not square the condition number in a redundant full KKT.
            xp=np.linalg.lstsq(Ee,bb,rcond=1e-11)[0]
            Z=null_space(Ee,rcond=1e-11)
            sol=xp+Z@np.linalg.lstsq(A@Z,t-A@xp,rcond=1e-11)[0]
            oracles.append({'task_vector':A@sol,'task_squared_error':float(np.linalg.norm(A@sol-t)**2),'equality_inf':float(np.max(np.abs(Ee@sol-bb)))})
            Ee=np.vstack([Ee,A]); bb=np.r_[bb,A@sol]
        diff=float(np.max(np.abs(sol-x)))
        if diff>2e-6 or np.max(np.abs(E@x-b))>2e-7: raise ValueError('INDEPENDENT_ORACLE_MISMATCH:'+str(diff))
        records.append({'case':k,'provenance':'SYNTHETIC_SPD_ROBOTS_IDEAL_RIGID_CONTACT_NOT_ISAAC_MODEL','full_dimension':54,'equalities':36,'solution':x,'independent_nullspace_oracle':sol,'oracle_difference_inf':diff,'oracle_priorities':oracles,'solver':d,'total_hqp_wall_s':wall,'minimum_inactive_box_margin':float(100-np.max(np.abs(x))),'M_robot':M,'h_robot':h,'M_object':Mo,'h_object':ho,'J_contact_local':J,'H':H,'G':G,'object_acceleration_reference':target,'internal_reference':internal})
    times=[r['total_hqp_wall_s'] for r in records]
    summary={'status':'QP6_COUPLED_MATH_PASS','scope':'IDEAL_MATHEMATICAL_MODEL_ONLY_NOT_ACTUAL_ARTICULATION_DYNAMICS_PASS','cases':len(records),'maximum_nullspace_oracle_difference_inf':max(r['oracle_difference_inf'] for r in records),'maximum_equality_inf':max(r['solver']['equality_residual_inf'] for r in records),'maximum_internal_null_residual':max(r['solver']['internal_null_residual'] for r in records),'offline_hqp_wall_s':{'peak':max(times),'p50':float(np.percentile(times,50)),'p95':float(np.percentile(times,95)),'p99':float(np.percentile(times,99))},'real_time_certification':False,'observed_physical_contact_wrench':False}
    dump(a.output/'cases.json',records); dump(a.output/'summary.json',summary); print(summary)


if __name__=='__main__': main()
