"""Re-evaluate saved native-drive observations; never commands the simulator."""
import argparse,json
from pathlib import Path
import numpy as np


def audit(path):
    data=json.loads((path/'raw.json').read_text())['rows']; result=json.loads((path/'result.json').read_text()); dt=1/60
    analysis={}
    for side in ('left','right'):
        f=data[-1]['arms'][side]; M=np.asarray(f['M']); q=np.asarray(f['q']); v=np.asarray(f['dq']); vp=np.asarray(data[-2]['arms'][side]['dq']) if len(data)>1 else v; qp=np.asarray(data[-2]['arms'][side]['q']) if len(data)>1 else q
        acc=(v-vp)/dt; qfd=(q-qp)/dt; proj=np.asarray(f['solver_projected_not_actuator_sensor']); c=np.asarray(f['c']);g=np.asarray(f['g'])
        # Projected joint effort and finite differences are recorded at a post-step
        # state; their timing is not certified actuator effort. Diagnostic only.
        residual=M[:7,:7]@acc[:7]+M[:7,7:]@acc[7:]+c[:7]+g[:7]-proj[:7]
        analysis[side]={'last_q':q.tolist(),'last_dq':v.tolist(),'last_position_finite_difference':qfd.tolist(),'last_acceleration_finite_difference':acc.tolist(),'M_symmetry_inf':float(np.max(np.abs(M-M.T))),'M_min_eigenvalue':float(np.linalg.eigvalsh((M+M.T)/2).min()),'Maa_shape':[7,7],'Maf_shape':[7,2],'Maf_times_finger_acceleration_Nm':(M[:7,7:]@acc[7:]).tolist(),'arm_equation_diagnostic_residual_using_solver_projected_effort_not_torque_sensor_Nm':residual.tolist(),'residual_scope':'post-state finite difference and solver-projected joint effort; timing/sensor independence unqualified','position_velocity_inconsistency_inf_rad_s':float(np.max(np.abs(qfd[:7]-v[:7])))}
    return {'run':str(path),'frames':len(data),'status':result['status'],'first_failure':result['first_failure'],'callback_dt_unique_s':sorted(set(result['callback_dts_s'])),'sides':analysis,'arm_subsystem_status':'PARTIAL_NOT_QUALIFIED','finger_constraint_model':'UNVALIDATED','full_articulation_constrained_dynamics':'UNVALIDATED'}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(audit(a.run),indent=2)+'\n')
