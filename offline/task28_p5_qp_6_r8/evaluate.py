"""Independent all-frame performance and source audit, CPU only."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from offline.task28_g1_1.task28_validator.model import RobotModel,DEFAULT_FIXTURE
from offline.task28_g1_1.task28_validator.collision import CollisionValidator
from offline.task28_g1_1.task28_validator.se3 import residual
from offline.task28_p5_qp_6_r8.controller import reference,torque,gains
from offline.task28_p5_qp_6_r8.motion import moving_reference
from offline.task28_p5_qp_6_r8.source_gate import R8DiagnosticGate
HERE=Path(__file__).resolve().parent

def accepted_metrics(sides):
 return bool(len(sides)==2 and all(v['tcp_peak_translation_m']<=.001 and v['tcp_peak_rotation_rad']<=.01 and v['joint_peak_drift_rad']<=.02 and v['native_dq_peak_rad_s']<=.3 for v in sides.values()))

def evaluate(name):
 folder=HERE/'evidence/diagnostic'/name
 if not (folder/'result.json').exists():return {'status':'NOT_RUN','pass':False,'execution_valid':False}
 ref_fn=moving_reference if name=="MOVING" else reference
 r=json.loads((folder/'result.json').read_text());frames=[json.loads(x) for x in (folder/'frames.jsonl').read_text().splitlines()] if (folder/'frames.jsonl').exists() else []
 life=json.loads((HERE/'evidence/process'/name/'lifecycle.json').read_text())
 control=frames[next((i for i,f in enumerate(frames) if f['arms']['left']['position_drives_disabled']),len(frames)):]
 events=[json.loads(x) for x in (folder/'command_journal.jsonl').read_text().splitlines()] if (folder/'command_journal.jsonl').exists() else []
 pre=[e for e in events if e['event']=='PRE_COMMAND'];immediate=[e for e in events if e['event']=='IMMEDIATE_READBACK'];post=[e for e in events if e['event']=='POST_STEP']
 out={'status':r['status'],'commands':r.get('actual_effort_commands',0),'writes':r.get('robot_command_writes',0),'strategy':r.get('controller'),'sides':{},'failures':[],'first_runtime_failure':r.get('first_failure'),'process_isolation':life.get('isolation_verified',False),'process_clean_exit':life.get('clean_exit',False),'source_files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in folder.iterdir() if p.is_file()},'unexpected_collisions':[x for f in frames for x in f['collision']['unexpected']]}
 gate=R8DiagnosticGate();model=RobotModel();cfg=json.loads((HERE/'source_contract.json').read_text());limits=np.array(cfg['limits']['research_effort_abs_max_Nm']);rates=np.array(cfg['limits']['arm_effort_rate_max_Nm_s'])
 if control:
  out['actual_physics_duration_s']=control[-1]['time_s']-control[0]['time_s'];out['physics_steps']=[control[0]['physics_step'],control[-1]['physics_step']]
  cv=CollisionValidator(DEFAULT_FIXTURE)
  out['independent_geometry']={'frames':len(control),'max_fk_translation_m':0.,'max_fk_rotation_rad':0.,'unexpected':[]}
  for f in control:
   for side in ('left','right'):
    x=f['arms'][side];m,rad=residual(model.fk(side,np.array(x['q'][:7])),np.array(x['tcp_world']))
    out['independent_geometry']['max_fk_translation_m']=max(out['independent_geometry']['max_fk_translation_m'],float(m))
    out['independent_geometry']['max_fk_rotation_rad']=max(out['independent_geometry']['max_fk_rotation_rad'],float(rad))
    if m>2e-5 or rad>2e-5:out['failures'].append('INDEPENDENT_FK')
   hit=cv.validate(model.forward(np.array(f['arms']['left']['q'][:7]),np.array(f['arms']['right']['q'][:7])),np.array(f['cube_world']),'detached')
   unexpected=[x for x in hit.reasons if x!='COLLISION:Cube_01/box|task28_table/box']
   out['independent_geometry']['unexpected']+=unexpected
   if unexpected:out['failures'].append('INDEPENDENT_COLLISION')
   try:gate.validate_pair(f['arms'])
   except Exception as exc:out['failures'].append(str(exc));break
  if len(pre)!=len(control)-1 or len(pre)!=len(immediate) or len(pre)!=len(post):out['failures'].append('JOURNAL_FRAME_COUNTS')
  for side in ('left','right'):
   states=[f['arms'][side] for f in control];q=np.array([a['q'][:7] for a in states]);v=np.array([a['dq'][:7] for a in states]);tcp=np.array([a['tcp_world'] for a in states]);trans=np.linalg.norm(tcp[:,:3,3]-tcp[0,:3,3],axis=1);rot=np.array([Rotation.from_matrix(tcp[0,:3,:3].T@T[:3,:3]).magnitude() for T in tcp]);dq=np.abs(q-q[0]);a=states[0];K,D=gains(a['M'],r['controller']);feedback=[];efforts=[];rate_values=[];errors=[];readback=[];fingers=[]
   for k,e in enumerate(pre):
    if k>=len(states)-1:break
    qr,vr=ref_fn(q[0],v[0],k/60,r['controller']);u,c=torque(states[k],qr,vr,K,D)
    applied=np.array(e['full_effort'][side]);previous=np.array(e['previous_applied'][side]);efforts.append(applied[:7]);rate_values.append(abs(applied[:7]-previous[:7])*60);feedback.append(c['feedback_Nm']);errors.append(float(np.max(abs(applied[:7]-u))))
    readback.append(float(np.max(abs(applied-np.array(states[k+1]['applied_command_not_sensor'])))))
    fingers.append(bool(np.array_equal(applied[7:],previous[7:]) and all(states[k+1][key]==a[key] for key in ('finger_kp','finger_kd','finger_targets'))))
    if e['states'][side]['physics_step']!=control[k]['physics_step'] or control[k+1]['physics_step']!=control[k]['physics_step']+1:out['failures'].append('STEP_CURSOR')
    if abs(control[k+1]['time_s']-control[k]['time_s']-1/60)>1e-8:out['failures'].append('DT')
   effort_peak=np.max(abs(np.array(efforts)),axis=0) if efforts else np.zeros(7);rate_peak=np.max(np.array(rate_values),axis=0) if rate_values else np.zeros(7)
   out['sides'][side]={'tcp_peak_translation_m':float(max(trans)),'tcp_final_translation_m':float(trans[-1]),'tcp_peak_rotation_rad':float(max(rot)),'tcp_final_rotation_rad':float(rot[-1]),'joint_peak_drift_rad':float(np.max(dq)),'joint_final_delta_rad':(q[-1]-q[0]).tolist(),'native_dq_peak_rad_s':float(np.max(abs(v))),'initial_q_rad':q[0].tolist(),'initial_dq_rad_s':v[0].tolist(),'feedback_peak_Nm_per_joint':np.max(abs(np.array(feedback)),axis=0).tolist() if feedback else [],'effort_peak_Nm_per_joint':effort_peak.tolist(),'rate_peak_Nm_s_per_joint':rate_peak.tolist(),'command_reconstruction_error_Nm':max(errors,default=0),'readback_max_error_Nm':max(readback,default=0),'finger_preserved':bool(fingers and all(fingers)),'finger_q_peak_change_m':float(np.max(abs(np.array([a['q'][7:] for a in states])-np.array(states[0]['q'][7:])))),'torque_limit_exceeded':bool(np.any(effort_peak>limits)),'rate_limit_exceeded':bool(np.any(rate_peak>rates)),'first_1mm_exceed_step':next((control[i]['physics_step'] for i,t in enumerate(trans) if t>.001),None)}
   if name=='MOVING':
    expected_q=np.array([ref_fn(q[0],v[0],k/60,r['controller'])[0] for k in range(len(states))]);expected_tcp=np.array([model.fk(side,x) for x in expected_q]);tracking=np.linalg.norm(tcp[:,:3,3]-expected_tcp[:,:3,3],axis=1);rotation_tracking=np.array([Rotation.from_matrix(T[:3,:3].T@U[:3,:3]).magnitude() for T,U in zip(expected_tcp,tcp)])
    out['sides'][side].update(TCP_tracking_peak_m=float(max(tracking)),TCP_tracking_moving_peak_m=float(max(tracking[60:])),TCP_tracking_final_m=float(tracking[-1]),TCP_rotation_tracking_peak_rad=float(max(rotation_tracking)),joint_tracking_peak_rad=float(np.max(abs(q-expected_q))),joint_tracking_final_rad=float(np.max(abs(q[-1]-expected_q[-1]))),actual_joint2_displacement_rad=float(q[-1,1]-q[0,1]),motion_nonzero_reference_commands=sum(bool(np.linalg.norm(ref_fn(q[0],v[0],k/60,r['controller'])[1])>0) for k in range(60,180)))
   if max(errors,default=0)>1e-6 or max(readback,default=0)>1e-6:out['failures'].append('TORQUE_RECONSTRUCTION_OR_READBACK')
   if not all(fingers) or not fingers:out['failures'].append('FINGER_POLICY')
   if np.any(effort_peak>limits) or np.any(rate_peak>rates):out['failures'].append('EFFORT_RATE')
   if not all(model.within_limits(x) for x in q):out['failures'].append('JOINT_LIMIT')
  out['worst_tcp_peak_m']=max(s['tcp_peak_translation_m'] for s in out['sides'].values())
 if out['unexpected_collisions']:out['failures'].append('COLLISION')
 expected=r.get('registration',{}).get('steps',-1)
 out['execution_valid']=bool(r['status'] in ('DIAGNOSTIC_STABILIZATION_COMPLETE','FORMAL_STATIC_HOLD_COMPLETE','MOVING_TRACKING_COMPLETE') and len(control)==expected+1 and len(pre)==expected and out['commands']==expected and not out['failures'] and out['process_isolation'] and out['process_isolation'])
 out['performance_pass']=accepted_metrics(out['sides'])
 if name=='MOVING':
  out['performance_pass']=bool(len(out['sides'])==2 and all(s.get('TCP_tracking_peak_m',1)<=.001 and s.get('TCP_tracking_moving_peak_m',1)<=.0002 and s.get('TCP_tracking_final_m',1)<=.0001 and s.get('TCP_rotation_tracking_peak_rad',1)<=.01 and s.get('joint_tracking_peak_rad',1)<=.002 and s.get('joint_tracking_final_rad',1)<=.0002 and s.get('actual_joint2_displacement_rad',0)>=.0018 and s['native_dq_peak_rad_s']<=.3 for s in out['sides'].values()))
 out['pass']=out['execution_valid'] and out['performance_pass']
 out['verdict']='PASS' if out['pass'] else 'FAIL_PERFORMANCE' if out['execution_valid'] else 'FAIL_EXECUTION_OR_SOURCE'
 return out

def main():
 p=argparse.ArgumentParser();p.add_argument('--select',action='store_true');a=p.parse_args()
 cells={n:evaluate(n) for n in ('D1','D2','D3','D4','FORMAL','MOVING')}
 if a.select:
  eligible=[n for n in ('D2','D3') if cells[n]['execution_valid']]
  if not eligible:raise RuntimeError('NO_REPEAT_CANDIDATE')
  name=min(eligible,key=lambda n:cells[n]['worst_tcp_peak_m']);selection={'selected_cell':name,'strategy':cells[name]['strategy'],'metric':'worst-arm all-sample TCP translation','value_m':cells[name]['worst_tcp_peak_m'],'selection_frozen_before_D4':True}
  (HERE/'evidence/selection.json').write_text(json.dumps(selection,indent=2)+'\n')
 selected=json.loads((HERE/'evidence/selection.json').read_text()) if (HERE/'evidence/selection.json').exists() else None
 ready=bool(selected and cells[selected['selected_cell']]['pass'] and cells['D4']['pass'] and cells['D4']['strategy']==selected['strategy'])
 prerequisite={'eligible':ready,'selected':selected,'reason':'independent diagnostic repeat PASS' if ready else 'selected diagnostic and independent D4 repeat have not both passed all-frame acceptance and source/process gates'}
 (HERE/'evidence/formal_prerequisite.json').write_text(json.dumps(prerequisite,indent=2)+'\n')
 (HERE/'evidence/evaluation.json').write_text(json.dumps({'cells':cells,'formal_prerequisite':prerequisite},indent=2,sort_keys=True)+'\n')
 print(json.dumps({n:{k:c.get(k) for k in ('verdict','pass','commands','worst_tcp_peak_m')} for n,c in cells.items()},indent=2))
if __name__=='__main__':main()
