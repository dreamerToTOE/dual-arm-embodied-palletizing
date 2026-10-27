"""Offline synthetic negative controls, never Isaac or calibration evidence."""
import json
from pathlib import Path
import numpy as np
import pytest
from offline.task28_p2_2_r1 import sign_control as sc


def write(path, data):
    path.write_text(json.dumps(data))


@pytest.fixture
def synthetic_sign_pair(tmp_path):
    cfg = json.loads(sc.CONFIG.read_text())
    names = cfg['joint_names'] + ['finger1', 'finger2']
    status = {'mode':'sign', 'status':'COLLECTION_COMPLETE_NOT_EVALUATED',
        'source_kind':'ISAAC_SINGLE_ARTICULATION_STANDALONE',
        'config_sha256':sc.sha(sc.CONFIG), 'asset_sha256':cfg['fixture_usd_sha256'],
        'collector_sha256':sc.sha(sc.COLLECTOR),
        'runtime': {'all_dof_names':names, 'ordered_joint_names':cfg['joint_names'],
            'ordered_dof_indices':list(range(7)), 'dof_stiffness':[1]*9, 'dof_damping':[1]*9,
            'observation_source':'SingleArticulation.get_measured_joint_efforts only',
            'force_frame':'world', 'position_argument_frame':'world',
            'physics_gravity':[[0,0,-1],9.81], 'physics_dt_s':cfg['physics_dt_s']}}
    write(tmp_path/'collector_status.json', status)
    for load in cfg['loads']:
        for condition in ['unloaded', 'loaded']:
            force = np.asarray(load['force_world_n'],float) if condition == 'loaded' else np.zeros(3)
            point = np.array([1.05,-.4,.5])
            moment = np.cross(point - np.asarray(cfg['base_world_m']), force)[2]
            rows = []
            for i in range(cfg['observation_window_steps']):
                q = list(cfg['configurations_rad']['home']); q[0] += moment * 1e-4
                rows.append({'physics_step':i+541, 'simulation_time_s':(i+541)*cfg['physics_dt_s'],
                    'q_rad':q, 'dq_rad_s':[0.0]*7, 'effort_nm':[-moment]+[0.0]*6,
                    'application_point_world_m':point.tolist(), 'tcp_pre_step_world_m':point.tolist(),
                    'applied_force_world_n':force.tolist(), 'applied_torque_world_nm':[0,0,0]})
            write(tmp_path/f"home_{load['id']}_{condition}.json", {'configuration':'home','load':load,
                'condition':condition, 'target_q_rad':cfg['configurations_rad']['home'],
                'observations':rows, 'config_sha256':sc.sha(sc.CONFIG)})
    return tmp_path


def test_known_axis_equilibrium_and_both_signs(synthetic_sign_pair):
    result = sc.evaluate(synthetic_sign_pair)
    assert result['status'] == 'SIGN_CONTROL_PASS'
    assert result['independent_equilibrium_confirmed']
    assert all(c['equilibrium_residual_nm'] < 1e-10 for c in result['cases'])


def test_wrong_effort_sign_is_rejected(synthetic_sign_pair):
    p = synthetic_sign_pair/'home_positive_fx_loaded.json'
    doc = json.loads(p.read_text())
    for row in doc['observations']:
        row['effort_nm'][0] *= -1
    write(p, doc)
    r = sc.evaluate(synthetic_sign_pair)
    assert r['status'] == 'REJECTED'
    assert not r['independent_equilibrium_confirmed']
    assert 'MEASURED_EFFORT_NOT_OPPOSED_TO_MOMENT' in r['cases'][0]['failures']


def test_duplicate_timestamp_is_rejected(synthetic_sign_pair):
    p = synthetic_sign_pair/'home_positive_fx_loaded.json'
    doc = json.loads(p.read_text())
    doc['observations'][1]['simulation_time_s'] = doc['observations'][0]['simulation_time_s']
    write(p, doc)
    r = sc.evaluate(synthetic_sign_pair)
    assert r['status'] == 'REJECTED'
    assert 'STALE_DUPLICATE_OR_ASYNCHRONOUS_SAMPLE' in r['cases'][0]['failures']


def test_wrong_dof_order_and_missing_data_rejected(synthetic_sign_pair):
    p = synthetic_sign_pair/'collector_status.json'
    doc = json.loads(p.read_text()); doc['runtime']['ordered_dof_indices'][0:2] = [1,0]; write(p,doc)
    (synthetic_sign_pair/'home_negative_fy_loaded.json').unlink()
    r = sc.evaluate(synthetic_sign_pair)
    assert r['status'] == 'REJECTED'
    assert 'INVALID_JOINT_ORDER_SOURCE_OR_RUNTIME' in r['global_failures']
    assert r['cases'][-1]['status'] == 'REJECTED'
