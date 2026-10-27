#!/usr/bin/env python3
"""Evaluate frozen R1 static records, keeping the load oracle out of estimation.

Input files are immutable; output goes to a fresh directory. This evaluator does
not launch Isaac or produce observations. Sign calibration must be a separate,
independently reviewed on-disk record, never a fit to the cases evaluated here.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'offline/task28_g1_1'))
from task28_validator.model import RobotModel
from offline.task28_p2_2.estimator import Contract
from offline.task28_p2_2.evaluate_probe import _read_condition
from offline.task28_p2_2_r1.sign_control import evaluate as evaluate_sign_control
from offline.task28_p2_2_r1.adapter import (
    SEMANTIC, SOURCE, SignCalibration, StaticLimits, StaticWindow,
    static_actuator_pair_to_wrench,
)

# Registered before this formal acquisition; altered or empty matrices cannot pass.
EXPECTED_CONFIG_SHA256 = 'afa406a646cc844552c2e7822be18c35fabca9ffa7eac527c33bd8a34379e93d'
EXPECTED_SIGN_CONFIG_SHA256 = '9f27b78b1223cae4efcb0d8006a6587f826b872f3158fdf4f12882f10287dad7'
EXPECTED_CONFIGURATIONS = ('home', 'offset')
EXPECTED_LOADS = ('zero', 'positive_fx', 'positive_fy', 'positive_fz', 'positive_mx',
                  'positive_my', 'positive_mz', 'negative_fx', 'negative_fz', 'negative_mz')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def evaluate(input_dir, config_path, calibration_path, sign_input_dir):
    config = json.loads(config_path.read_text())
    config_sha = digest(config_path)
    calibration_record = json.loads(calibration_path.read_text())
    calibration_sha = digest(calibration_path)
    result = {'schema': 'task28.p2_2_r1.static_6d_evaluation.v1',
              'status': 'INCOMPLETE', 'config_sha256': config_sha,
              'calibration_sha256': calibration_sha,
              'evaluator_sha256': digest(Path(__file__)),
              'adapter_sha256': digest(ROOT / 'offline/task28_p2_2_r1/adapter.py'),
              'input_directory': str(input_dir.resolve()), 'flags': config['flags'],
              'oracle_access': 'only after SVD estimate; never passed to estimator or adapter',
              'cases': [], 'global_failures': []}
    global_failures = result['global_failures']
    sign_config_path = ROOT / 'offline/task28_p2_2_r1/evidence/sign_preregistration.json'
    sign_config = json.loads(sign_config_path.read_text())
    if config_sha != EXPECTED_CONFIG_SHA256:
        global_failures.append('FORMAL_CONFIGURATION_HASH_MISMATCH')
    if digest(sign_config_path) != EXPECTED_SIGN_CONFIG_SHA256:
        global_failures.append('SIGN_CONFIGURATION_HASH_MISMATCH')
    if (set(config.get('configurations_rad', {})) != set(EXPECTED_CONFIGURATIONS)
            or len(config.get('loads', [])) != len(EXPECTED_LOADS)
            or {entry.get('id') for entry in config.get('loads', [])} != set(EXPECTED_LOADS)):
        global_failures.append('FORMAL_EXPERIMENT_MATRIX_MISMATCH')
    shared_settings = ('fixture_usd_sha256', 'joint_names', 'base_world_m',
                       'physics_dt_s', 'gravity_m_s2', 'control')
    if any(sign_config.get(k) != config.get(k) for k in shared_settings):
        global_failures.append('SIGN_CONTROL_CONFIGURATION_MISMATCH')
    if (calibration_record.get('status') != 'SIGN_CONTROL_PASS'
            or calibration_record.get('source') != SOURCE
            or calibration_record.get('semantic') != SEMANTIC
            or calibration_record.get('independent_equilibrium_confirmed') is not True
            or calibration_record.get('config_sha256') != digest(sign_config_path)
            or calibration_record.get('script_sha256') != digest(ROOT / 'offline/task28_p2_2_r1/sign_control.py')
            or calibration_record.get('global_failures') != []
            or not calibration_record.get('cases')
            or any(c.get('status') != 'PASS' for c in calibration_record['cases'])):
        global_failures.append('INDEPENDENT_SIGN_CALIBRATION_MISSING_OR_MISMATCH')
    sign_cases = calibration_record.get('cases', [])
    expected_ids = [load['id'] for load in sign_config['loads']]
    if len(sign_cases) != len(expected_ids) or sorted(c.get('load_id') for c in sign_cases) != sorted(expected_ids):
        global_failures.append('SIGN_CONTROL_CASE_IDENTITIES_MISMATCH')
    try:
        if digest(sign_input_dir / 'collector_status.json') != calibration_record['collector_status_sha256']:
            raise ValueError('SIGN_COLLECTOR_HASH_MISMATCH')
        for case in sign_cases:
            for kind in ('unloaded', 'loaded'):
                path = sign_input_dir / f"home_{case['load_id']}_{kind}.json"
                if digest(path) != case['input_sha256'][kind]:
                    raise ValueError('SIGN_INPUT_HASH_MISMATCH')
    except (OSError, KeyError, TypeError, ValueError):
        global_failures.append('SIGN_RAW_EVIDENCE_HASH_MISMATCH_OR_MISSING')
    try:
        recalculated = evaluate_sign_control(sign_input_dir)
        if recalculated != calibration_record:
            raise ValueError('SIGN_CONTROL_RECOMPUTATION_MISMATCH')
    except (OSError, KeyError, TypeError, ValueError):
        global_failures.append('SIGN_CONTROL_RECOMPUTATION_MISMATCH')
    status_path = input_dir / 'collector_status.json'
    if not status_path.is_file():
        global_failures.append('MISSING_COLLECTOR_STATUS')
        return result
    state = json.loads(status_path.read_text())
    result['collector_status_sha256'] = digest(status_path)
    result['source_kind'] = state.get('source_kind', 'MISSING')
    if calibration_record.get('source_kind') != result['source_kind']:
        global_failures.append('SIGN_CALIBRATION_SOURCE_KIND_MISMATCH')
    if state.get('config_sha256') != config_sha:
        global_failures.append('COLLECTOR_CONFIG_HASH_MISMATCH')
    if state.get('asset_sha256') != config['fixture_usd_sha256']:
        global_failures.append('COLLECTOR_ASSET_HASH_MISMATCH')
    collector = ROOT / 'offline/task28_p2_2_r1/static_isaac_probe_r1.py'
    if not collector.exists() or state.get('collector_sha256') != digest(collector):
        global_failures.append('COLLECTOR_SOURCE_HASH_MISMATCH')
    try:
        sign_state = json.loads((sign_input_dir / 'collector_status.json').read_text())
        if (sign_state.get('collector_sha256') != state.get('collector_sha256')
                or sign_state.get('asset_sha256') != state.get('asset_sha256')
                or sign_state.get('config_sha256') != digest(sign_config_path)
                or sign_state.get('source_kind') != state.get('source_kind')
                or sign_state.get('mode') != 'sign'):
            raise ValueError('SIGN_COLLECTOR_IDENTITY_MISMATCH')
    except (OSError, json.JSONDecodeError, ValueError):
        global_failures.append('SIGN_COLLECTOR_IDENTITY_MISMATCH')
    if state.get('mode') != 'six':
        global_failures.append('INCORRECT_COLLECTION_MODE')
    if result['source_kind'] not in ('ISAAC_SINGLE_ARTICULATION_STANDALONE', 'SYNTHETIC_TEST'):
        global_failures.append('SOURCE_KIND_NOT_VERIFIED')
    runtime = state.get('runtime', {})
    names = tuple(config['joint_names'])
    all_names, indices = runtime.get('all_dof_names', []), runtime.get('ordered_dof_indices', [])
    if (runtime.get('ordered_joint_names') != list(names) or len(indices) != 7
            or len(set(indices)) != 7 or len(set(all_names)) != len(all_names)
            or any(not isinstance(i, int) or i < 0 or i >= len(all_names)
                   or all_names[i] != n for i, n in zip(indices, names))
            or runtime.get('observation_source') != 'SingleArticulation.get_measured_joint_efforts only'):
        global_failures.append('JOINT_ORDER_OR_EFFORT_SOURCE_UNVERIFIED')
    if state.get('status') != 'COLLECTION_COMPLETE_NOT_EVALUATED':
        global_failures.append('COLLECTION_NOT_COMPLETE')
    try:
        stiff = np.asarray(runtime['dof_stiffness'], float)
        damp = np.asarray(runtime['dof_damping'], float)
        gravity = runtime['physics_gravity']
        g = np.asarray(gravity[0], float) * float(gravity[1])
        if (stiff.shape != (len(all_names),) or damp.shape != (len(all_names),)
                or not np.isfinite(np.r_[stiff, damp, g]).all() or np.any(stiff[indices] <= 0)
                or abs(float(runtime['physics_dt_s']) - config['physics_dt_s']) > 1e-12
                or not np.allclose(g, config['gravity_m_s2'], rtol=0, atol=1e-6)):
            raise ValueError('RUNTIME_SETTINGS_MISMATCH')
    except (KeyError, ValueError, TypeError, IndexError):
        global_failures.append('RUNTIME_SETTINGS_UNVERIFIED')
    # Refuse to convert at all when sign, source, DOFs or physics are unproven.
    if global_failures:
        return result
    calibration = SignCalibration(SOURCE, SEMANTIC, True, calibration_sha)
    th = config['thresholds']
    limits = StaticLimits(config['observation_window_steps'] * config['physics_dt_s'],
                          th['max_abs_velocity_rad_s'], th['max_window_q_range_rad'],
                          th['max_window_effort_std_nm'], th['max_pair_mean_q_difference_rad'])
    model = RobotModel()
    n, dt = config['observation_window_steps'], config['physics_dt_s']
    incomplete = False
    for config_id in config['configurations_rad']:
        for load in config['loads']:
            load_id = load['id']
            case = {'configuration': config_id, 'load_id': load_id,
                    'status': 'INCOMPLETE', 'failures': []}
            result['cases'].append(case)
            paths = {kind: input_dir / f'{config_id}_{load_id}_{kind}.json'
                     for kind in ('unloaded', 'loaded')}
            if not all(path.is_file() for path in paths.values()):
                incomplete = True
                case['failures'].append('MISSING_CONDITION_FILE')
                continue
            case['input_sha256'] = {k: digest(p) for k, p in paths.items()}
            try:
                # Check complete raw identity including applied load and target.
                for path in paths.values():
                    raw = json.loads(path.read_text())
                    if raw.get('load') != load or raw.get('target_q_rad') != config['configurations_rad'][config_id]:
                        raise ValueError('RAW_LOAD_OR_TARGET_METADATA_MISMATCH')
                pair = {k: _read_condition(p, config_sha, config_id, load_id, k, n, dt, model)
                        for k, p in paths.items()}
                for item in pair.values():
                    if any(not model.within_limits(qi) for qi in item['q']):
                        case['failures'].append('OBSERVATION_Q_OUTSIDE_LIMITS')
                u, l = pair['unloaded'], pair['loaded']
                q = l['q'].mean(axis=0)
                J = model.jacobian('left', q)
                def window(item):
                    return StaticWindow(item['q'], item['dq'], item['tau'], item['time'],
                                        float(item['time'][-1]), names, SOURCE)
                converted = static_actuator_pair_to_wrench(
                    window(u), window(l), J, contract=Contract(names, 'world',
                    'left_fr3_side_suction_tcp', 0, 0), limits=limits,
                    calibration=calibration, jacobian_q=q.copy(), jacobian_joint_names=names,
                    frame='world', tcp_point='left_fr3_side_suction_tcp',
                    jacobian_provenance='frozen G1 left URDF world-frame virtual TCP Jacobian')
                converted['estimate'] = asdict(converted['estimate'])
                case.update(converted)
                case['input_sha256'] = {k: v['sha256'] for k, v in pair.items()}
                case['max_pair_mean_q_difference_rad'] = float(np.max(np.abs(l['q'].mean(axis=0)-u['q'].mean(axis=0))))
                case['loaded_mean_q_minus_target_rad'] = (q - np.asarray(config['configurations_rad'][config_id], float)).tolist()
                case['unloaded_mean_q_minus_target_rad'] = (u['q'].mean(axis=0) - np.asarray(config['configurations_rad'][config_id], float)).tolist()
                stats = {}
                for k, item in pair.items():
                    stats[k] = {'max_abs_velocity_rad_s': float(np.max(np.abs(item['dq']))),
                                'max_window_q_range_rad': float(np.max(np.ptp(item['q'], axis=0))),
                                'max_window_effort_std_nm': float(np.max(np.std(item['tau'], axis=0))),
                                'max_link7_fk_position_difference_m': float(np.max(item['fk_p'])),
                                'max_link7_fk_rotation_difference_rad': float(np.max(item['fk_r']))}
                    for label, value in stats[k].items():
                        if value > th[label]:
                            case['failures'].append(k.upper() + '_' + label.upper())
                case['condition_stats'] = stats
                # No applied force/torque is consumed until AFTER the estimation above.
                F = np.asarray(load['force_world_n'], float)
                M = np.asarray(load['torque_world_nm'], float)
                r = np.asarray(load['offset_from_virtual_tcp_world_m'], float)
                oracle = np.r_[F, M + np.cross(r, F)]
                if not np.isfinite(oracle).all():
                    raise ValueError('NONFINITE_ORACLE')
                tau_expected = J.T @ oracle
                tau_error = np.asarray(converted['external_generalized_torque_on_robot_Nm']) - tau_expected
                err = np.asarray(converted['estimate']['wrench_minimum_norm']) - oracle
                for k, item in pair.items():
                    force = F if k == 'loaded' else np.zeros(3)
                    torque = M if k == 'loaded' else np.zeros(3)
                    offset = r if k == 'loaded' else np.zeros(3)
                    for row in item['rows']:
                        f = np.asarray(row['applied_force_world_n'], float)
                        m = np.asarray(row['applied_torque_world_nm'], float)
                        p = np.asarray(row['application_point_world_m'], float)
                        pre_q = np.asarray(row['q_pre_step_rad'], float)
                        if (f.shape != (3,) or m.shape != (3,) or p.shape != (3,) or pre_q.shape != (7,)
                                or not np.isfinite(np.r_[f, m, p, pre_q]).all()):
                            raise ValueError('MALFORMED_APPLICATION_LOG')
                        if not np.allclose(f, force, rtol=0, atol=1e-12):
                            case['failures'].append(k.upper() + '_APPLIED_FORCE_MISMATCH')
                            break
                        if not np.allclose(m, torque, rtol=0, atol=1e-12):
                            case['failures'].append(k.upper() + '_APPLIED_TORQUE_MISMATCH')
                            break
                        if not np.allclose(p, model.fk('left', pre_q)[:3, 3] + offset, rtol=0, atol=1e-10):
                            case['failures'].append(k.upper() + '_APPLICATION_POINT_MISMATCH')
                            break
                est = converted['estimate']
                if est['rank'] != 6 or min(est['singular_values']) < th['minimum_jacobian_singular_value']:
                    case['failures'].append('JACOBIAN_RANK_OR_CONDITION_GATE')
                if np.max(np.abs(tau_error)) > th['max_abs_torque_error_nm']:
                    case['failures'].append('JOINT_TORQUE_ORACLE_MISMATCH')
                if np.linalg.norm(err[:3]) > th['max_force_error_n']:
                    case['failures'].append('FORCE_ORACLE_MISMATCH')
                if np.linalg.norm(err[3:]) > th['max_moment_error_nm']:
                    case['failures'].append('MOMENT_ORACLE_MISMATCH')
                if not np.any(oracle) and np.max(np.abs(converted['delta_actuator_effort_Nm'])) > th['zero_delta_tau_inf_nm']:
                    case['failures'].append('ZERO_LOAD_TORQUE_MISMATCH')
                case.update({'oracle_wrench_on_robot_world_n_nm': oracle.tolist(),
                             'expected_external_generalized_torque_nm': tau_expected.tolist(),
                             'torque_error_nm': tau_error.tolist(),
                             'max_abs_torque_error_nm': float(np.max(np.abs(tau_error))),
                             'wrench_error_n_nm': err.tolist(),
                             'force_error_norm_n': float(np.linalg.norm(err[:3])),
                             'moment_error_norm_nm': float(np.linalg.norm(err[3:]))})
                case['status'] = 'REJECTED' if case['failures'] else 'GATES_PASS'
            except (KeyError, json.JSONDecodeError) as exc:
                incomplete = True
                case['failures'].append('MISSING_OR_TRUNCATED_DATA:' + str(exc))
            except (ValueError, TypeError, OverflowError) as exc:
                case['status'] = 'REJECTED'
                case['failures'].append('INVALID_OR_NUMERICAL:' + str(exc))
    if len(result['cases']) != len(EXPECTED_CONFIGURATIONS) * len(EXPECTED_LOADS):
        result['global_failures'].append('FORMAL_CASE_COUNT_MISMATCH')
        result['status'] = 'INCOMPLETE'
    elif incomplete:
        result['status'] = 'INCOMPLETE'
    elif any(c['status'] != 'GATES_PASS' for c in result['cases']):
        result['status'] = 'REJECTED'
    elif result['source_kind'] == 'SYNTHETIC_TEST':
        result['status'] = 'SYNTHETIC_GATES_PASS_NOT_ACTUAL_EVIDENCE'
    else:
        result['status'] = 'ALL_STATIC_GATES_PASS_PENDING_INDEPENDENT_REVIEW'
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--sign-input', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=False, exist_ok=False)
    result = evaluate(args.input, args.config, args.calibration, args.sign_input)
    with (args.output / 'evaluation.json').open('x') as f:
        json.dump(result, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')
    print(result['status'])
    return 0 if result['status'].startswith('ALL_STATIC') else 1


if __name__ == '__main__':
    raise SystemExit(main())
