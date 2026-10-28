import json
from offline.task28_p2_4_r3.evaluate import evaluate, sha, CONFIG
def _records(tmp,n=240):
 cfg=json.loads(CONFIG.read_text()); rows=[]
 for i in range(n): rows.append({'sim_time_s':i*cfg['physics_dt_s'],'q_rad_or_m':[0.0]*9,'dq_rad_s_or_m_s':[0.0]*9,'commanded_effort':[0.0]*9,'applied_effort_readback':[0.0]*9,'source_read_duration_s':0.0001})
 (tmp/'zero_load_hold_observations.json').write_text(json.dumps({'samples':rows})); (tmp/'collector_status.json').write_text(json.dumps({'status':'COLLECTION_COMPLETE_NOT_EVALUATED','preregistration_sha256':sha(CONFIG),'runtime':{'all_dof_names':cfg['expected_all_dof_names'],'effort_mode':['force']*9,'runtime_drive_gains_zero':True}}))
def test_stability_gate_accepts_valid_contract(tmp_path):
 _records(tmp_path); assert evaluate(tmp_path)['status']=='STABILITY_GATE_PASS'
def test_stability_gate_rejects_readback_mismatch(tmp_path):
 _records(tmp_path); p=tmp_path/'zero_load_hold_observations.json'; d=json.loads(p.read_text()); d['samples'][4]['applied_effort_readback'][0]=1.0; p.write_text(json.dumps(d)); r=evaluate(tmp_path); assert r['status']=='STABILITY_GATE_REJECTED' and 'COMMAND_READBACK_MISMATCH' in r['reasons']
