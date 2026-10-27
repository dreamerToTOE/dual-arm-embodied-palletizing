"""Synthetic tests of evaluation barriers, never real simulator evidence."""
import hashlib
import json
import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from task28_validator.model import RobotModel
from offline.task28_p2_2_r1.adapter import SEMANTIC, SOURCE
from offline.task28_p2_2_r1.evaluate_six import evaluate, ROOT
import offline.task28_p2_2_r1.evaluate_six as evaluator


def h(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.fixture
def synthetic(tmp_path, monkeypatch):
    cfg = json.loads((ROOT / 'offline/task28_p2_2_r1/evidence/static_preregistration.json').read_text())
    cfg['configurations_rad'] = {'home': cfg['configurations_rad']['home']}
    cfg['loads'] = [next(x for x in cfg['loads'] if x['id'] == 'positive_fx')]
    cfg['observation_window_steps'] = 3
    config = tmp_path / 'cfg.json'
    config.write_text(json.dumps(cfg))
    # Narrow synthetic matrix is explicitly test-scoped; production hash remains frozen.
    monkeypatch.setattr(evaluator, 'EXPECTED_CONFIG_SHA256', h(config))
    monkeypatch.setattr(evaluator, 'EXPECTED_CONFIGURATIONS', ('home',))
    monkeypatch.setattr(evaluator, 'EXPECTED_LOADS', ('positive_fx',))
    input_dir = tmp_path / 'input'
    input_dir.mkdir()
    sign_input = tmp_path / 'sign_input'
    sign_input.mkdir()
    sign_cfg = json.loads((ROOT / 'offline/task28_p2_2_r1/evidence/sign_preregistration.json').read_text())
    (sign_input / 'collector_status.json').write_text(json.dumps({
        'collector_sha256': h(ROOT / 'offline/task28_p2_2_r1/static_isaac_probe_r1.py'),
        'asset_sha256': cfg['fixture_usd_sha256'], 'mode': 'sign',
        'config_sha256': h(ROOT / 'offline/task28_p2_2_r1/evidence/sign_preregistration.json'),
        'source_kind': 'SYNTHETIC_TEST'}))
    sign_cases = []
    for load in sign_cfg['loads']:
        case = {'status': 'PASS', 'load_id': load['id'], 'input_sha256': {}}
        for kind in ('unloaded', 'loaded'):
            path = sign_input / f"home_{load['id']}_{kind}.json"
            path.write_text('{}')
            case['input_sha256'][kind] = h(path)
        sign_cases.append(case)
    calibration = tmp_path / 'calibration.json'
    calibration.write_text(json.dumps({'status': 'SIGN_CONTROL_PASS', 'source': SOURCE,
                                     'semantic': SEMANTIC, 'independent_equilibrium_confirmed': True,
                                     'source_kind': 'SYNTHETIC_TEST', 'global_failures': [],
                                     'cases': sign_cases,
                                     'collector_status_sha256': h(sign_input / 'collector_status.json'),
                                     'script_sha256': h(ROOT / 'offline/task28_p2_2_r1/sign_control.py'),
                                     'config_sha256': h(ROOT / 'offline/task28_p2_2_r1/evidence/sign_preregistration.json')}))
    synthetic_control = json.loads(calibration.read_text())
    monkeypatch.setattr(evaluator, 'evaluate_sign_control', lambda _: copy.deepcopy(synthetic_control))
    names = cfg['joint_names']
    state = {'status': 'COLLECTION_COMPLETE_NOT_EVALUATED', 'source_kind': 'SYNTHETIC_TEST',
             'mode': 'six', 'config_sha256': h(config), 'asset_sha256': cfg['fixture_usd_sha256'],
             'collector_sha256': h(ROOT / 'offline/task28_p2_2_r1/static_isaac_probe_r1.py'),
             'runtime': {'ordered_joint_names': names, 'all_dof_names': names,
                         'ordered_dof_indices': list(range(7)), 'dof_stiffness': [1.]*7,
                         'dof_damping': [1.]*7, 'physics_dt_s': cfg['physics_dt_s'],
                         'physics_gravity': [[0, 0, -1], 9.81],
                         'observation_source': 'SingleArticulation.get_measured_joint_efforts only'}}
    (input_dir / 'collector_status.json').write_text(json.dumps(state))
    model = RobotModel()
    q = np.array(cfg['configurations_rad']['home'])
    J = model.jacobian('left', q)
    force = np.array([1., 0., 0.])
    link = model._chain('left', q)[0]['left_fr3_link7']
    quat = Rotation.from_matrix(link[:3, :3]).as_quat()[[3, 0, 1, 2]]
    for condition in ('unloaded', 'loaded'):
        f = force if condition == 'loaded' else np.zeros(3)
        tau = -J.T @ np.r_[f, np.zeros(3)]
        rows = [{'physics_step': i + 1, 'simulation_time_s': (i+1)*cfg['physics_dt_s'],
                 'q_rad': q.tolist(), 'dq_rad_s': [0.]*7, 'effort_nm': tau.tolist(),
                 'link7_position_world_m': link[:3, 3].tolist(), 'link7_orientation_world_wxyz': quat.tolist(),
                 'applied_force_world_n': f.tolist(), 'applied_torque_world_nm': [0.]*3,
                 'application_point_world_m': model.fk('left', q)[:3, 3].tolist(),
                 'q_pre_step_rad': q.tolist()} for i in range(3)]
        doc = {'configuration': 'home', 'load': cfg['loads'][0], 'condition': condition,
               'observations': rows, 'config_sha256': h(config), 'target_q_rad': q.tolist()}
        (input_dir / f'home_positive_fx_{condition}.json').write_text(json.dumps(doc))
    return input_dir, config, calibration, sign_input


def test_only_synthetic_pass(synthetic):
    result = evaluate(*synthetic)
    assert result['status'] == 'SYNTHETIC_GATES_PASS_NOT_ACTUAL_EVIDENCE'
    assert result['cases'][0]['status'] == 'GATES_PASS'
    assert result['cases'][0]['max_abs_torque_error_nm'] < 1e-12


def test_rejects_uncalibrated_without_estimating(synthetic):
    inp, cfg, cal, sign_input = synthetic
    doc = json.loads(cal.read_text())
    doc['independent_equilibrium_confirmed'] = False
    cal.write_text(json.dumps(doc))
    result = evaluate(inp, cfg, cal, sign_input)
    assert result['status'] == 'INCOMPLETE'
    assert result['cases'] == []
    assert 'INDEPENDENT_SIGN_CALIBRATION_MISSING_OR_MISMATCH' in result['global_failures']


def test_wrong_sign_and_force_log_are_rejected(synthetic):
    inp, cfg, cal, sign_input = synthetic
    path = inp / 'home_positive_fx_loaded.json'
    doc = json.loads(path.read_text())
    for row in doc['observations']:
        row['effort_nm'] = [-x for x in row['effort_nm']]
        row['applied_force_world_n'] = [0, 1, 0]
    path.write_text(json.dumps(doc))
    result = evaluate(inp, cfg, cal, sign_input)
    assert result['status'] == 'REJECTED'
    assert 'JOINT_TORQUE_ORACLE_MISMATCH' in result['cases'][0]['failures']
    assert 'LOADED_APPLIED_FORCE_MISMATCH' in result['cases'][0]['failures']


def test_incomplete_and_stale_data_are_not_pass(synthetic):
    inp, cfg, cal, sign_input = synthetic
    path = inp / 'home_positive_fx_loaded.json'
    doc = json.loads(path.read_text())
    doc['observations'].pop()
    path.write_text(json.dumps(doc))
    result = evaluate(inp, cfg, cal, sign_input)
    assert result['status'] != 'ALL_STATIC_GATES_PASS_PENDING_INDEPENDENT_REVIEW'
    assert result['cases'][0]['status'] != 'GATES_PASS'


def test_sign_evidence_tampering_and_missing_case_is_rejected(synthetic):
    inp, cfg, cal, sign_input = synthetic
    sign = json.loads(cal.read_text())
    sign['cases'].pop()
    cal.write_text(json.dumps(sign))
    (sign_input / 'collector_status.json').write_text('{"tampered":true}')
    result = evaluate(inp, cfg, cal, sign_input)
    assert 'SIGN_CONTROL_CASE_IDENTITIES_MISMATCH' in result['global_failures']
    assert 'SIGN_RAW_EVIDENCE_HASH_MISMATCH_OR_MISSING' in result['global_failures']
    assert result['cases'] == []


def test_empty_or_changed_formal_matrix_never_passes(synthetic):
    inp, cfg, cal, sign_input = synthetic
    altered = json.loads(cfg.read_text())
    altered['loads'] = []
    cfg.write_text(json.dumps(altered))
    result = evaluate(inp, cfg, cal, sign_input)
    assert 'FORMAL_CONFIGURATION_HASH_MISMATCH' in result['global_failures']
    assert 'FORMAL_EXPERIMENT_MATRIX_MISMATCH' in result['global_failures']
    assert not result['cases']


@pytest.mark.parametrize('field', ['target_q_rad', 'load'])
def test_raw_metadata_is_checked_against_registration(synthetic, field):
    inp, cfg, cal, sign_input = synthetic
    path = inp / 'home_positive_fx_loaded.json'
    raw = json.loads(path.read_text())
    if field == 'load':
        raw[field]['force_world_n'] = [0, 2, 0]
    else:
        raw[field][0] += .001
    path.write_text(json.dumps(raw))
    result = evaluate(inp, cfg, cal, sign_input)
    assert result['cases'][0]['status'] == 'REJECTED'
    assert 'INVALID_OR_NUMERICAL:RAW_LOAD_OR_TARGET_METADATA_MISMATCH' in result['cases'][0]['failures']


def test_recomputed_calibration_must_match_archived_calibration(synthetic):
    inp, cfg, cal, sign_input = synthetic
    sign = json.loads(cal.read_text())
    sign['semantic_explanation'] = 'forged'
    cal.write_text(json.dumps(sign))
    result = evaluate(inp, cfg, cal, sign_input)
    assert 'SIGN_CONTROL_RECOMPUTATION_MISMATCH' in result['global_failures']
    assert not result['cases']
