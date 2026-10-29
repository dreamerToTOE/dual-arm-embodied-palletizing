"""CPU-only frozen-G1 object-space readiness, never an actuator or HQP claim."""
import hashlib,json
from pathlib import Path
import numpy as np
from offline.task28_g1_1.task28_validator.model import RobotModel
from offline.task28_g1_1.task28_validator.closed_chain import nominal_grasps,evaluate_closure,ClosurePolicy
from offline.task28_g1_1.task28_validator.se3 import inverse
HERE=Path(__file__).resolve().parent

def skew(r):
 x,y,z=r;return np.array([[0,-z,y],[z,0,-x],[-y,x,0]])

def main():
 evaluation=json.loads((HERE/'evidence/evaluation.json').read_text())
 if not evaluation['cells']['MOVING']['pass']:raise RuntimeError('MOVING_PREREQUISITE_NOT_PASS')
 path=HERE/'evidence/diagnostic/MOVING/frames.jsonl';frames=[json.loads(x) for x in path.read_text().splitlines()];frames=[f for f in frames if f['arms']['left']['position_drives_disabled']]
 model=RobotModel();grasps=nominal_grasps();rows=[]
 for f in frames:
  left=np.array(f['arms']['left']['tcp_world']);right=np.array(f['arms']['right']['tcp_world']);obj=np.array(f['cube_world']);closure=evaluate_closure(obj,left,right,grasps,ClosurePolicy());j=[]
  for side,T,G in (('left',left,grasps.left),('right',right,grasps.right)):
   J=model.jacobian(side,np.array(f['arms'][side]['q'][:7]));O=T@inverse(G);r=O[:3,3]-T[:3,3];j.append(np.vstack((J[:3]-skew(r)@J[3:],J[3:])))
  relative=np.hstack((-j[0],j[1]));common=.5*np.hstack((j[0],j[1]));A=np.vstack((common,relative));sv=np.linalg.svd(A,compute_uv=False)
  rows.append({'physics_step':f['physics_step'],'time_s':f['time_s'],'relative_object_translation_m':closure.translation_error_m,'relative_object_rotation_rad':closure.rotation_error_rad,'left_absolute_grasp_m':closure.left_object_translation_error_m,'right_absolute_grasp_m':closure.right_object_translation_error_m,'closure_valid':closure.valid,'closure_reasons':closure.reasons,'object_task_jacobian_rank':int(np.linalg.matrix_rank(A)),'object_task_jacobian_singular_values':sv.tolist()})
 out={'status':'OFFLINE_READINESS_EVALUATED_NOT_READY','moving_prerequisite':'PASS','source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'frozen_object_to_TCP':{'left':grasps.left.tolist(),'right':grasps.right.tolist()},'original_closure_tolerances':{'translation_m':1e-4,'rotation_rad':1e-3},'rows':rows,'all_closure_valid':all(x['closure_valid'] for x in rows),'reason':'Executed reference is free-home joint-space motion. Frozen object-centered grasp closure is not satisfied; applicable inverse dynamics remains unqualified. Stop before a cooperative QP/HQP solve or execution.','QP_solve_count':0,'HQP_runtime_commands':0,'Cube_suction_commands':0,'D6_attachments':0,'qualification_not_inferred_from_J_rank':True}
 (HERE/'evidence/cooperative_readiness.json').write_text(json.dumps(out,indent=2,sort_keys=True)+'\n')
 print(json.dumps({'status':out['status'],'frames':len(rows),'first':rows[0],'last':rows[-1]},indent=2))
if __name__=='__main__':main()
