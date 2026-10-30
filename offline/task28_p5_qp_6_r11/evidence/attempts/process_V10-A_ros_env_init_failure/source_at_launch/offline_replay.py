"""Read-only, CPU semi-implicit counterfactual replay of immutable R10 V10-A.

The replay is diagnostic evidence only. It never starts Isaac, writes commands, or
changes a gate. It varies only the startup reference velocity term while keeping
recorded QP qdot, mass, gains, geometry and target sequence fixed.
"""
from __future__ import annotations
import argparse, json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r10.qp_math import native_velocity_handoff

DT=1/60

def run(source: Path, output: Path):
    rows=[json.loads(x) for x in (source/'frames.jsonl').read_text().splitlines() if x.strip()]
    result=json.loads((source/'result.json').read_text())
    rows=[r for r in rows if 'r10_qp' in r]
    vf=result['virtual_frame']; Tvl=np.asarray(vf['T_VL'],float); Tvr=np.asarray(vf['T_VR'],float)
    model=RobotModel(DEFAULT_FIXTURE)
    qhold=np.r_[result['q_hold_actual']['left'],result['q_hold_actual']['right']].astype(float)
    dq0=np.r_[result['gains']['left']['dq0'],result['gains']['right']['dq0']].astype(float)
    Kp=np.block([[np.asarray(result['gains']['left']['Kp'],float),np.zeros((7,7))],[np.zeros((7,7)),np.asarray(result['gains']['right']['Kp'],float)]])
    Kd=np.block([[np.asarray(result['gains']['left']['Kd'],float),np.zeros((7,7))],[np.zeros((7,7)),np.asarray(result['gains']['right']['Kd'],float)]])
    qdot=np.asarray([r['r10_qp']['qdot'] for r in rows],float)
    qpath=np.zeros(14); paths=[]
    for x in qdot: paths.append(qpath.copy()); qpath += x*DT
    def hand_native(t):
        p,v,_=native_velocity_handoff(t,dq0); return p,v
    def reference(kind,t):
        if kind=='baseline': return hand_native(t)
        if kind=='immediate_zero': return np.zeros(14),np.zeros(14)
        tau=float(kind.split(':')[1]); return np.zeros(14),-dq0*np.exp(-t/tau)
    def one(kind):
        q=qhold.copy(); v=dq0.copy(); records=[]
        for k,row in enumerate(rows):
            t=k*DT; pos,vel=reference(kind,t); qr=qhold+paths[k]+pos; qdr=qdot[k]+vel
            M=np.block([[np.asarray(row['arms']['left']['M'],float)[:7,:7],np.zeros((7,7))],[np.zeros((7,7)),np.asarray(row['arms']['right']['M'],float)[:7,:7]]])
            acc=np.linalg.solve(M,Kp@(qr-q)+Kd@(qdr-v)); v=v+acc*DT; q=q+v*DT
            goal=np.asarray(row['r10_qp']['virtual_goal'],float)
            Il=model.fk('left',q[:7])@inverse(Tvl); Ir=model.fk('right',q[7:])@inverse(Tvr)
            obj=max(residual(Il,goal)[0],residual(Ir,goal)[0]); rel=residual(Il,Ir)[0]
            records.append({'physics_step':int(row['physics_step']),'object_error_m':float(obj),'relative_error_m':float(rel),'joint_error_inf_rad':float(np.max(np.abs(q-qr))),'velocity_inf_rad_s':float(np.max(np.abs(v)))})
        return {'kind':kind,'peak_object_error_m':max(x['object_error_m'] for x in records),'peak_relative_error_m':max(x['relative_error_m'] for x in records),'peak_joint_error_inf_rad':max(x['joint_error_inf_rad'] for x in records),'peak_velocity_inf_rad_s':max(x['velocity_inf_rad_s'] for x in records),'records':records}
    variants=[one('baseline'),one('immediate_zero')]+[one(f'capture:{tau}') for tau in (0.05,0.075,0.10,0.125)]
    out={'schema':'TASK28_R11_SINGLE_VARIABLE_CPU_REPLAY_V1','source':str(source),'source_runtime_status':result.get('status'),'execution_authorized':False,'physical_execution':False,'model':{'integrator':'semi_implicit_euler','dt_s':DT,'plant':'M^-1[Kp(q_ref-q)+Kd(dq_ref-dq)]','gravity':'omitted_for_counterfactual_only','recorded_inputs':['R10 qdot','R10 M','R10 Kp/Kd','R10 qhold/dq0','R10 virtual goals']},'selection_rule':'tau_s=0.10 was frozen before R11 Isaac runs from the fixed set {0.05,0.075,0.10,0.125}; it is predicted below 0.25 mm and below 0.15 mm relative error without changing gates.','variants':variants}
    output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n'); return out

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--source',type=Path,default=Path('offline/task28_p5_qp_6_r10/evidence/diagnostic/V10-A')); ap.add_argument('--output',type=Path,default=Path('offline/task28_p5_qp_6_r11/evidence/single_variable_replay.json')); args=ap.parse_args(); o=run(args.source,args.output); print(json.dumps({'variants':[{k:v for k,v in x.items() if k!='records'} for x in o['variants']]},indent=2))
