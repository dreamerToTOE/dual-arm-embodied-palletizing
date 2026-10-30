"""Read-only proxy sensitivity for the globally frozen feedback scale.

This uses immutable R11 qdot records and does not re-solve or execute the QP.
It is a pre-registration aid only; R12 Isaac runs remain the acceptance test.
"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import DEFAULT_FIXTURE, RobotModel
from offline.task28_g1_1.task28_validator.se3 import inverse, residual
from offline.task28_p5_qp_6_r11.virtual_frames import VirtualCooperativeFrame

DT=1/60

def one(cell: str, scale: float):
    d=Path('offline/task28_p5_qp_6_r11/evidence/diagnostic')/cell
    result=json.loads((d/'result.json').read_text())
    rows=[json.loads(x) for x in (d/'frames.jsonl').read_text().splitlines() if 'r11_qp' in x]
    vf=result['virtual_frame']; frame=VirtualCooperativeFrame(np.asarray(vf['T_WV']),np.asarray(vf['T_VL']),np.asarray(vf['T_VR']))
    model=RobotModel(DEFAULT_FIXTURE)
    qhold=np.r_[result['q_hold_actual']['left'],result['q_hold_actual']['right']].astype(float)
    dq0=np.r_[result['gains']['left']['dq0'],result['gains']['right']['dq0']].astype(float)
    Kp=np.block([[np.asarray(result['gains']['left']['Kp'],float),np.zeros((7,7))],[np.zeros((7,7)),np.asarray(result['gains']['right']['Kp'],float)]])
    Kd=np.block([[np.asarray(result['gains']['left']['Kd'],float),np.zeros((7,7))],[np.zeros((7,7)),np.asarray(result['gains']['right']['Kd'],float)]])
    qdot=np.asarray([r['r11_qp']['qdot'] for r in rows],float); path=np.zeros(14); qpaths=[]
    for x in qdot: qpaths.append(path.copy()); path += scale*x*DT
    q=qhold.copy(); v=dq0.copy(); records=[]
    for k,row in enumerate(rows):
        qref=qhold+qpaths[k]; dqref=scale*qdot[k]-dq0*np.exp(-(k*DT)/0.10)
        M=np.block([[np.asarray(row['arms']['left']['M'],float)[:7,:7],np.zeros((7,7))],[np.zeros((7,7)),np.asarray(row['arms']['right']['M'],float)[:7,:7]]])
        a=np.linalg.solve(M,Kp@(qref-q)+Kd@(dqref-v)); v += a*DT; q += v*DT
        goal=np.asarray(row['r11_qp']['virtual_goal'],float)
        Il=model.fk('left',q[:7])@inverse(frame.T_VL); Ir=model.fk('right',q[7:])@inverse(frame.T_VR)
        records.append({'physics_step':int(row['physics_step']),'object_error_m':max(residual(Il,goal)[0],residual(Ir,goal)[0]),'relative_error_m':residual(Il,Ir)[0]})
    p=max(records,key=lambda x:x['object_error_m'])
    return {'cell':cell,'scale':scale,'peak_object_error_m':max(x['object_error_m'] for x in records),'peak_relative_error_m':max(x['relative_error_m'] for x in records),'peak_physics_step':p['physics_step'],'records':records}

def main(output=Path('offline/task28_p5_qp_6_r12/evidence/feedback_gain_sensitivity.json')):
    scales=[1.0,0.75,0.50,0.25]
    out={'schema':'TASK28_R12_FEEDBACK_GAIN_SENSITIVITY_V1','source':'R11 V10-A/B/C immutable archives','execution_authorized':False,'physical_execution':False,'model':'semi-implicit M^-1[Kp(qref-q)+Kd(dqref-dq)] with recorded R11 qdot/M/Kp/Kd and retained tau=0.10 velocity capture','caveat':'Proxy scales recorded pose-feedback qdot and does not re-solve the QP; it is diagnostic support, not an acceptance result.','selection_rule':'Freeze the single global scale 0.75 before R12 Isaac runs; no post-run tuning.','cells':{c:[one(c,s) for s in scales] for c in ('V10-A','V10-B','V10-C')}}
    output.parent.mkdir(parents=True,exist_ok=True); output.write_text(json.dumps(out,indent=2,sort_keys=True)+'\n'); print(json.dumps({c:[{k:v for k,v in x.items() if k not in ('records','cell')} for x in xs] for c,xs in out['cells'].items()},indent=2))
if __name__=='__main__': main()
