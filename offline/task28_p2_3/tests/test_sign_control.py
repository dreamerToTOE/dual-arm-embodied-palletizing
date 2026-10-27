"""Mock raw data to exercise sign-control gates; not Isaac observations."""
import json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p2_3.sign_control import CONFIG, COLLECTOR, sha256, verify_sign_controls


def make_files(tmp):
    cfg = json.loads(CONFIG.read_text())
    runtime = {'physics_dt_s':cfg['physics_dt_s'], 'physics_gravity':[[0,0,-1],9.81], 'arms':{}, 'observation_source':'SingleArticulation.get_measured_joint_efforts',
               'force_frame':'world', 'torque_frame':'world',
               'force_application_api':'RigidPrim.apply_forces_and_torques_at_pos(is_global=True)'}
    names = cfg['joint_names']
    for side in ('left','right'):
        runtime['arms'][side] = {'articulation_path':'/World/Probe_'+side, 'all_dof_names':names,
                                 'ordered_joint_names':names, 'ordered_dof_indices':list(range(7)),
                                 'base_world_position_m':cfg['base_world_m'][side],
                                 'base_world_orientation_wxyz':cfg['base_world_orientation_wxyz'],
                                 'dof_stiffness':[1.]*7, 'dof_damping':[1.]*7}
    status = {'status':'COLLECTION_COMPLETE_NOT_EVALUATED',
              'source_kind':'ISAAC_TWO_ARTICULATIONS_STANDALONE', 'config_sha256':sha256(CONFIG),
              'asset_sha256':cfg['fixture_usd_sha256'], 'collector_sha256':sha256(COLLECTOR), 'runtime':runtime}
    (tmp/'collector_status.json').write_text(json.dumps(status))
    step=0
    for side in ('left','right'):
        for direction in ('positive','negative'):
            case_id = side + '_' + direction + '_fx'
            case = next(x for x in cfg['cases'] if x['id']==case_id)
            for condition in ('unloaded','loaded'):
                rows=[]
                active = condition=='loaded'
                force = np.array(case[side]['force_world_n'] if active else [0,0,0])
                origin = np.array(cfg['base_world_m'][side]); point = origin+np.array([.3,-.2,.1])
                moment = np.cross(point-origin, force)[2]
                q = np.array(cfg['state'][side+'_q']); q[0] += moment * .0001
                effort = np.zeros(7); effort[0] = -moment
                tcp = np.eye(4); tcp[:3,3]=point
                for i in range(cfg['observation_window_steps']):
                    step += 1
                    row = {'physics_step':i+1, 'global_collector_step':step,
                           'simulation_time_s':(i+1)*cfg['physics_dt_s'],
                           'arms': {side:{'source_id':'probe_'+side, 'joint_names':names,
                               'q_rad':q.tolist(), 'dq_rad_s':[0]*7, 'effort_nm':effort.tolist(),
                               'load_oracle':{'q_pre_step_rad':q.tolist(), 'tcp_pre_step_world':tcp.tolist(),
                                   'application_point_world_m':point.tolist(), 'applied_force_world_n':force.tolist(),
                                   'applied_pure_torque_world_nm':[0,0,0]}}}}
                    rows.append(row)
                data={'case':case,'condition':condition,'observations':rows,
                      'source_kind':'ISAAC_TWO_ARTICULATIONS_STANDALONE','config_sha256':sha256(CONFIG)}
                (tmp/(case_id+'_'+condition+'.json')).write_text(json.dumps(data))
    return tmp


def test_all_four_direct_sign_controls(tmp_path):
    result = verify_sign_controls(make_files(tmp_path))
    assert result['status']=='SIGN_CONTROL_PASS'
    assert len(result['cases'])==4
    assert all(x['equilibrium_residual_nm'] < 1e-12 for x in result['cases'])
    assert all(x['input_sha256']['unloaded'] != x['input_sha256']['loaded'] for x in result['cases'])


@pytest.mark.parametrize('mutation,reason', [('sign','SIGN_EQUILIBRIUM_CONTROL_FAILED'),
    ('order','ARM_SOURCE_OR_JOINT_ORDER_MISMATCH'), ('stale','STALE_DUPLICATE_OR_OUT_OF_ORDER_OBSERVATION'),
    ('point','SIGN_CONTROL_LOAD_OR_REFERENCE_POINT_MISMATCH'), ('nonfinite','INVALID_EFFORT')])
def test_one_bad_raw_record_cannot_calibrate(tmp_path, mutation, reason):
    make_files(tmp_path)
    p = tmp_path/'right_positive_fx_loaded.json'; data = json.loads(p.read_text())
    rows = data['observations']; a = rows[0]['arms']['right']
    if mutation=='sign':
        for row in rows: row['arms']['right']['effort_nm'][0] *= -1
    elif mutation=='order': a['joint_names'] = a['joint_names'][::-1]
    elif mutation=='stale': rows[1]['simulation_time_s'] = rows[0]['simulation_time_s']
    elif mutation=='point': a['load_oracle']['application_point_world_m'][0] += 1
    elif mutation=='nonfinite': a['effort_nm'][0] = float('nan')
    p.write_text(json.dumps(data))
    result=verify_sign_controls(tmp_path)
    assert result['status']=='REJECTED'
    assert any(reason in failure for case in result['cases'] for failure in case['failures'])
