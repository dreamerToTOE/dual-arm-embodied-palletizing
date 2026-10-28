from __future__ import annotations
import hashlib,json,platform,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def sha(p):
 h=hashlib.sha256();
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1<<20),b''): h.update(b)
 return h.hexdigest()
def main():
 e=ROOT/'offline/task28_p5_qp_5/evidence'; formal=e/'formal_run_v1'; status=json.loads((formal/'collector_status.json').read_text())
 sources=['offline/task28_p5_qp_5/preregistration.json','offline/task28_p5_qp_5/collector.py','offline/task28_p5_qp_5/controller.py','offline/task28_p5_qp_5/evaluate.py','offline/task28_p5_qp_5/controls.py','offline/task28_p5_qp_5/tests/test_qp5.py','offline/task28_p5_qp_5/tests/test_controls.py','offline/task28_p5_qp_4/reference.py','offline/task28_p5_qp_4/torque_backend_contract.json','offline/task28_p5_qp_3/evidence/experiments_v1/handoff_records.json','offline/task28_g1_1/task28_validator/model.py','offline/task28_g1_1/task28_validator/closed_chain.py']
 hashes={p:sha(ROOT/p) for p in sources}; hashes['fixture_usd']=status['asset_sha256']
 stat=json.loads((formal/'static_hold_evaluation.json').read_text()); track=status['tracking_evaluation']
 failed=[]
 for d in sorted((e/'failed_runs').glob('run_*')):
  s=json.loads((d/'collector_status.json').read_text()); failed.append({'directory':d.name,'status':s.get('status'),'failure':s.get('failure',{}),'static_gate':json.loads((d/'static_hold_evaluation.json').read_text()) if (d/'static_hold_evaluation.json').is_file() else None})
 result={'schema':'task28.g2_p5_qp_5.evidence.v1','status':'G2_P5_QP_5_BILATERAL_HOLD_ONLY','scope':'ISOLATED_CPU/ISAAC_SIM_VIRTUAL_GEOMETRY_ONLY','execution_authorized':False,'physical_robot_tested':False,'continuous_safety_certified':False,'formal_run':{'collector_status_sha256':sha(formal/'collector_status.json'),'static_hold_evaluation_sha256':sha(formal/'static_hold_evaluation.json'),'tracking_observation_sha256':sha(formal/'tracking_observations.json'),'static_hold_observation_sha256':sha(formal/'static_hold_observations.json'),'static_hold':stat,'tracking':track,'physics_steps':status['physics_steps'],'elapsed_wall_s':status['elapsed_wall_s'],'reference_horizon_s':0.1,'tracking_observation_count':75,'interpolation':'piecewise SE(3) endpoint interpolation for virtual reference only; q interval-start velocity semantics preserved'},'rejected_formal_runs':failed,'negative_controls':json.loads((ROOT/'offline/task28_p5_qp_5/controls.py').read_text().split('def negative_controls')[0] and 'null' or 'null'),'source_hashes':hashes,'reproduction':{'collector_command':'OMNI_KIT_ACCEPT_EULA=YES PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONPATH=.:offline /home/ubuntu2004/isaacsim-4.5.0/python.sh offline/task28_p5_qp_5/collector.py --asset <fr3.usd> --output <fresh-directory> --prereg-sha256 <sha256(preregistration.json)>','historical_tests':'see regression_historical_789.log; 789 passed','qp4_tests':'see regression_qp4_53.log; 53 passed','qp5_tests':'see regression_qp5_5.log; 5 passed','platform':platform.platform(),'python':sys.version,'timestamp_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())},'limitations':['no Cube/suction/D6/table or payload contact','no ROS/production bridge or physical robot','static hold passes but moving reference rejected at frame 2 by frozen G1 grasp translation gate','no internal-force feedback or continuous collision certificate']}
 # replace placeholder with deterministic controls output
 from offline.task28_p5_qp_5.controls import negative_controls
 result['negative_controls']=negative_controls()
 (e/'summary.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')
 (e/'source_hashes.json').write_text(json.dumps(hashes,indent=2,sort_keys=True)+'\n')
 print(json.dumps({'status':result['status'],'static':stat['status'],'tracking':track['status'],'negative_controls':len(result['negative_controls'])}))
if __name__=='__main__': main()
