import json
from pathlib import Path
import numpy as np
from offline.task28_p2_4_r2.evaluate import evaluate, sha, CONFIG

ROOT = Path(__file__).resolve().parents[1]
CFG = json.loads(CONFIG.read_text())

def _run(tmp, cid, delta):
    n=CFG['source_steps']; rows=[]
    for i in range(n):
        q=[delta*i/(n-1),0,0,0,0,0,0,0.04,0.04]
        dq=[delta/(n-1)/CFG['physics_dt_s'],0,0,0,0,0,0,0,0]
        cmd=CFG['commands']['zero'] if cid=='zero' else CFG['commands']['plus_q1' if cid=='plus_q1' else 'minus_q1']
        rows.append({'sim_time_s':i*CFG['physics_dt_s'],'q_rad_or_m':q,'dq_rad_s_or_m_s':dq,
                     'commanded_effort':cmd,'applied_effort_readback':cmd,
                     'mass_matrix':np.eye(9).tolist(),'coriolis_compensation':[0.]*9,
                     'gravity_compensation':[0.]*9})
    (tmp/(cid+'_observations.json')).write_text(json.dumps({'case_id':cid,'samples':rows}))
    (tmp/(cid+'_oracle.json')).write_text(json.dumps({'case_id':cid,'application_points_world_m':[[0,0,0]]*n}))

def _status(tmp):
    runtime={'all_dof_names':CFG['expected_all_dof_names'],'post_effort_modes':['force']*9,
             'post_stiffness':[0.]*9,'post_damping':[0.]*9}
    (tmp/'collector_status.json').write_text(json.dumps({'status':'COLLECTION_COMPLETE_NOT_EVALUATED',
       'preregistration_sha256':sha(CONFIG),'collector_sha256':'a'*64,'runtime':runtime}))

def test_source_gate_accepts_explicit_command_readback(tmp_path):
    _status(tmp_path); _run(tmp_path,'zero',0.0001); _run(tmp_path,'plus_q1',0.02); _run(tmp_path,'minus_q1',-0.02)
    out=tmp_path/'eval.json'; result=evaluate(tmp_path,out,'source')
    assert result['status']=='SOURCE_GATE_PASS'

def test_source_gate_rejects_command_readback_mismatch(tmp_path):
    _status(tmp_path); _run(tmp_path,'zero',0.0001); _run(tmp_path,'plus_q1',0.02); _run(tmp_path,'minus_q1',-0.02)
    p=tmp_path/'plus_q1_observations.json'; d=json.loads(p.read_text()); d['samples'][3]['applied_effort_readback'][0]+=1e-3; p.write_text(json.dumps(d))
    result=evaluate(tmp_path,tmp_path/'eval.json','source')
    assert result['status']=='SOURCE_GATE_REJECTED' and 'COMMAND_READBACK_MISMATCH' in result['reasons']

def test_preregistration_freezes_zero_drive_force_mode():
    assert CFG['mode_configuration']=={'effort_mode':'force','stiffness':0.0,'damping':0.0,'runtime_only':True,'source_usd_unchanged':True}
    assert CFG['boundaries']['frozen_usd_modified'] is False
