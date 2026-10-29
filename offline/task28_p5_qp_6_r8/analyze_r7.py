"""Read-only R7 replay and coupled rigid-body predictions; no Isaac import."""
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.linalg import eigh,expm
from scipy.spatial.transform import Rotation
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_p5_qp_6_r8.controller import gains,reference,torque,discrete_matrix,KP_R7,KD_R7
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'task28_p5_qp_6_r7'

def main():
 model=RobotModel();out={'scope':'Observed source M and native end-step dq; predictions assume frozen unconstrained arm block, not TGS certified inverse dynamics. No tool inertia attribution.','cells':{},'design':{}}
 for name in ('S1','S3'):
  folder=OLD/'evidence/diagnostic'/name
  frames=[json.loads(x) for x in (folder/'frames.jsonl').read_text().splitlines()]
  control=frames[next(i for i,f in enumerate(frames) if f['arms']['left']['position_drives_disabled']):]
  r=json.loads((folder/'result.json').read_text());cell={'source_sha256':hashlib.sha256((folder/'frames.jsonl').read_bytes()).hexdigest(),'steps':len(control)-1,'physics_start':control[0]['physics_step'],'physics_end':control[-1]['physics_step'],'duration_s':control[-1]['time_s']-control[0]['time_s'],'sides':{}}
  for side in ('left','right'):
   states=[f['arms'][side] for f in control];a=states[0];q=np.array([x['q'][:7] for x in states]);v=np.array([x['dq'][:7] for x in states]);g=np.array([x['g'][:7] for x in states]);dt=np.diff([f['time_s'] for f in control]);fd=np.diff(q,axis=0)/dt[:,None];tcp=np.array([x['tcp_world'] for x in states]);delta=q[-1]-q[0];jac=model.jacobian(side,q[0]);M=np.array(a['M'])[:7,:7];M=(M+M.T)/2
   kp=np.diag(KP_R7 if name=='S3' else np.zeros(7));kd=np.diag(KD_R7)
   p=(q[0]-q[:-1])@kp.T;d=-v[:-1]@kd.T;feedback=p+d
   u=np.array([c['effort_arm'][side] for c in r['command_sequence']]);lam,V=eigh(np.diag(KP_R7),M);freq=np.sqrt(lam);modal_d=V.T@np.diag(KD_R7)@V
   A=np.block([[np.zeros((7,7)),np.eye(7)],[-np.linalg.solve(M,np.diag(KP_R7)),-np.linalg.solve(M,np.diag(KD_R7))]])
   poles=np.linalg.eigvals(A)
   contrib=jac[:3]*delta[None,:]
   cell['sides'][side]={'initial_q_rad':q[0].tolist(),'initial_native_dq_rad_s':v[0].tolist(),'final_q_delta_rad':delta.tolist(),'peak_native_dq_per_joint':np.max(abs(v),axis=0).tolist(),'mean_fd_minus_end_dq_per_joint':np.mean(fd-v[1:],axis=0).tolist(),'max_abs_fd_minus_end_dq_per_joint':np.max(abs(fd-v[1:]),axis=0).tolist(),'integral_end_dq_minus_delta_q_rad':(np.sum(v[1:]*dt[:,None],axis=0)-delta).tolist(),'gravity_start_Nm':g[0].tolist(),'gravity_final_Nm':g[-1].tolist(),'max_P_Nm_per_joint':np.max(abs(p),axis=0).tolist(),'max_D_Nm_per_joint':np.max(abs(d),axis=0).tolist(),'max_feedback_Nm_per_joint':np.max(abs(feedback),axis=0).tolist(),'command_reconstruction_max_error_Nm':float(np.max(abs(u-g[:-1]-feedback))),'tcp_final_translation_m':float(np.linalg.norm(tcp[-1,:3,3]-tcp[0,:3,3])),'tcp_peak_translation_m':float(np.max(np.linalg.norm(tcp[:,:3,3]-tcp[0,:3,3],axis=1))),'tcp_final_rotation_rad':float(Rotation.from_matrix(tcp[0,:3,:3].T@tcp[-1,:3,:3]).magnitude()),'jacobian_translation_contributions_m':contrib.T.tolist(),'jacobian_translation_sum_m':contrib.sum(axis=1).tolist(),'linearized_tcp_residual_m':float(np.linalg.norm(contrib.sum(axis=1)-(tcp[-1,:3,3]-tcp[0,:3,3]))),'M_arm':M.tolist(),'M_eigenvalues':np.linalg.eigvalsh(M).tolist(),'M_offdiagonal_frobenius_fraction':float(np.linalg.norm(M-np.diag(np.diag(M)))/np.linalg.norm(M)),'R7_PD_undamped_modal_wn_rad_s':freq.tolist(),'R7_PD_modal_projected_zeta':(np.diag(modal_d)/(2*freq)).tolist(),'R7_PD_modal_damping_offdiag_max':float(np.max(abs(modal_d-np.diag(np.diag(modal_d))))),'R7_PD_full_damped_poles':[[float(z.real),float(z.imag)] for z in poles]}
   if name=='S3':
    design={}
    for strategy in ('r7_pd','frozen_mass_hold','frozen_mass_smooth'):
     K,D=gains(a['M'],strategy);x=q[0].copy();dv=v[0].copy();qs=[x.copy()];us=[];fbs=[]
     for k in range(60):
      qr,vr=reference(q[0],v[0],k/60,strategy);fb=K@(qr-x)+D@(vr-dv);us.append(g[0]+fb);fbs.append(fb);dv=dv+np.linalg.solve(M,fb)/60;x=x+dv/60;qs.append(x.copy())
     us=np.array(us);qs=np.array(qs);rates=np.abs(np.diff(np.vstack([np.array(a['applied_command_not_sensor'][:7]),us]),axis=0))*60
     limits=np.array([35,12,8,35,4,4,4]);rate_limits=np.array([2400,600,240,2400,240,240,240]);eig=np.linalg.eigvals(discrete_matrix(M,K,D));pred_tcp=np.array([model.fk(side,x)[:3,3] for x in qs]);design[strategy]={'Kp':K.tolist(),'Kd':D.tolist(),'discrete_poles':[[float(z.real),float(z.imag)] for z in eig],'spectral_radius':float(max(abs(eig))),'predicted_peak_TCP_30step_m':float(max(np.linalg.norm(pred_tcp[:31]-pred_tcp[0],axis=1))),'predicted_peak_TCP_60step_m':float(max(np.linalg.norm(pred_tcp-pred_tcp[0],axis=1))),'predicted_abs_effort_peak_Nm':np.max(abs(us),axis=0).tolist(),'predicted_rate_peak_Nm_s':np.max(rates,axis=0).tolist(),'predicted_effort_and_rate_feasible':bool(np.all(abs(us)<=limits) and np.all(rates<=rate_limits)),'first_feedback_Nm':np.array(fbs[0]).tolist()}
    out['design'][side]=design
  out['cells'][name]=cell
 out['R7_replay_verdict']='FAIL_1MM_BOTH_S1_S3'
 (HERE/'evidence/r7_drift_analysis.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
 for name,c in out['cells'].items():
  print(name,{s:round(v['tcp_final_translation_m']*1000,9) for s,v in c['sides'].items()})
 for side,d in out['design'].items():print(side,{k:{'radius':v['spectral_radius'],'pred_mm':v['predicted_peak_TCP_30step_m']*1000,'feasible':v['predicted_effort_and_rate_feasible']} for k,v in d.items()})
 return out
if __name__=='__main__':main()
