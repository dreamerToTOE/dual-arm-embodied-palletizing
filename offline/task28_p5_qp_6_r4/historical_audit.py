"""Frame-wise read-only reconstruction of R3 records; no simulator calls."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parent
R3=ROOT.parent/'task28_p5_qp_6_r3/evidence'

def audit(run):
    rows=json.loads((R3/run/'raw.json').read_text())['rows']; records=[]
    for i in range(1,len(rows)):
        prev,cur=rows[i-1],rows[i];dt=cur['time_s']-prev['time_s']; rec={'row':i,'physics_step':cur['physics_step'],'previous_step':prev['physics_step'],'time_s':cur['time_s'],'previous_time_s':prev['time_s'],'dt_s':dt,'sides':{}}
        if dt<=0: raise ValueError('NON_MONOTONIC_TIME')
        for side in ('left','right'):
            a=prev['arms'][side];b=cur['arms'][side];q0=np.asarray(a['q']);q1=np.asarray(b['q']);dq0=np.asarray(a['dq']);dq1=np.asarray(b['dq']); L0=np.asarray(a['link8_world']);L1=np.asarray(b['link8_world']);T0=np.asarray(a['tcp_world']);T1=np.asarray(b['tcp_world'])
            rec['sides'][side]={'delta_q':(q1-q0).tolist(),'finite_difference_velocity':((q1-q0)/dt).tolist(),'reported_dq':dq1.tolist(),'previous_reported_dq':dq0.tolist(),'trapezoid_integrated_dq':((dq0+dq1)*dt/2).tolist(),'difference_reported_vs_fd':(dq1-(q1-q0)/dt).tolist(),'single_step_integral_error':(q1-q0-(dq0+dq1)*dt/2).tolist(),'link8_translation_change_m':float(np.linalg.norm(L1[:3,3]-L0[:3,3])),'link8_rotation_change_rad':float(np.linalg.norm(Rotation.from_matrix(L0[:3,:3].T@L1[:3,:3]).as_rotvec())),'tcp_translation_change_m':float(np.linalg.norm(T1[:3,3]-T0[:3,3])),'tcp_rotation_change_rad':float(np.linalg.norm(Rotation.from_matrix(T0[:3,:3].T@T1[:3,:3]).as_rotvec())),'dynamic_control_minus_wrapper_q':(np.asarray(b['dynamic_control_q'])-q1).tolist(),'dynamic_control_minus_wrapper_dq':(np.asarray(b['dynamic_control_dq'])-dq1).tolist(),'drive_kp':b['kp'],'drive_kd':b['kd'],'drive_target':'NOT_RECORDED','source_dtype_and_pointers':'NOT_RECORDED_IN_ARCHIVE'}
        records.append(rec)
    return {'run':run,'source_raw_sha256':__import__('hashlib').sha256((R3/run/'raw.json').read_bytes()).hexdigest(),'records':records,'archive_aliasing_verdict':'UNKNOWN_NO_ARCHIVED_POINTERS; installed wrapper clone and float64 conversion suggest copies, must test live','physical_source_independence':'Dynamic Control and tensor have not been established as independent physics solvers','link7':'NOT_RECORDED_IN_R3'}

def main():
    p=ROOT/'evidence/historical';p.mkdir(parents=True,exist_ok=True)
    for run in ('dev_02','dev_03'):
        x=audit(run);(p/(run+'.json')).write_text(json.dumps(x,indent=2)+'\n'); print(run,len(x['records']))
if __name__=='__main__':main()
