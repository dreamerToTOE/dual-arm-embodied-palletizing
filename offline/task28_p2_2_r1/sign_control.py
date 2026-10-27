#!/usr/bin/env python3
"""Independent FR3 joint-1 static sign control; does not use a Jacobian.

Reads a separate preregistered sign-mode Isaac acquisition, not the six-axis
experiment.  External wrench is applied ON robot.  With world +Z joint axis,
virtual work gives tau_ext1 = ((p - o) x F).z.  At static equilibrium a
position-hold actuator counteracts the external moment.  No wrench estimator or
six-axis results are consulted.  Input and output paths must be distinct.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'evidence/sign_preregistration.json'
COLLECTOR = HERE / 'static_isaac_probe_r1.py'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def read_rows(path, config, h, load, condition):
    data = json.loads(path.read_text())
    if (data.get('config_sha256') != h or data.get('configuration') != 'home'
            or data.get('condition') != condition or data.get('load') != load
            or data.get('target_q_rad') != config['configurations_rad']['home']):
        raise ValueError('RAW_METADATA_MISMATCH')
    rows = data['observations']
    if len(rows) != config['observation_window_steps']:
        raise ValueError('INCOMPLETE_OBSERVATION_WINDOW')
    def arr(key, n):
        v = np.asarray([r[key] for r in rows], dtype=float)
        if v.shape != (len(rows), n) or not np.isfinite(v).all():
            raise ValueError('INVALID_' + key.upper())
        return v
    q, dq, effort = arr('q_rad',7), arr('dq_rad_s',7), arr('effort_nm',7)
    p, tcp, force = arr('application_point_world_m',3), arr('tcp_pre_step_world_m',3), arr('applied_force_world_n',3)
    torque = arr('applied_torque_world_nm',3)
    steps = np.asarray([r['physics_step'] for r in rows], dtype=float)
    times = np.asarray([r['simulation_time_s'] for r in rows], dtype=float)
    if (not np.isfinite(steps).all() or not np.isfinite(times).all()
            or np.any(np.diff(steps) != 1)
            or not np.allclose(np.diff(times), config['physics_dt_s'], atol=1e-9, rtol=0)):
        raise ValueError('STALE_DUPLICATE_OR_ASYNCHRONOUS_SAMPLE')
    expected_force = load['force_world_n'] if condition == 'loaded' else [0,0,0]
    if (not np.allclose(force, expected_force, atol=1e-8, rtol=0)
            or not np.allclose(torque, 0, atol=1e-8, rtol=0)
            or not np.allclose(p, tcp, atol=1e-8, rtol=0)):
        raise ValueError('SIGN_CONTROL_LOAD_OR_POINT_MISMATCH')
    return {'q': q, 'dq': dq, 'effort': effort, 'p': p, 'force': force,
            'times': times, 'steps': steps, 'sha256': sha(path)}

def evaluate(input_dir, config=CONFIG):
    cfg = json.loads(config.read_text())
    digest = sha(config)
    status_path = input_dir / 'collector_status.json'
    status = json.loads(status_path.read_text())
    result = {'schema': 'task28.p2_2_r1.sign_control.v1', 'status': 'REJECTED',
              'source': 'isaac_sim_4_5_single_articulation_measured_joint_efforts',
              'semantic': 'actuator_effort_loaded_minus_unloaded_opposes_wrench_on_robot',
              'independent_equilibrium_confirmed': False,
              'source_kind': status.get('source_kind'), 'config_sha256': digest,
              'collector_status_sha256': sha(status_path), 'script_sha256': sha(Path(__file__)),
              'method': 'direct world +Z lever-arm moment about joint1; no Jacobian or wrench estimation',
              'flags': cfg['flags'], 'global_failures': [], 'cases': []}
    failures = result['global_failures']
    if (status.get('mode') != 'sign' or status.get('status') != 'COLLECTION_COMPLETE_NOT_EVALUATED'
            or status.get('source_kind') != 'ISAAC_SINGLE_ARTICULATION_STANDALONE'
            or status.get('config_sha256') != digest or status.get('asset_sha256') != cfg['fixture_usd_sha256']
            or status.get('collector_sha256') != sha(COLLECTOR)):
        failures.append('COLLECTOR_PROVENANCE_MISMATCH_OR_INCOMPLETE')
    runtime = status.get('runtime', {})
    names, indices = runtime.get('all_dof_names', []), runtime.get('ordered_dof_indices', [])
    try:
        if (runtime.get('ordered_joint_names') != cfg['joint_names'] or len(set(names)) != len(names)
                or len(indices) != 7 or len(set(indices)) != 7
                or any(type(i) is not int or i < 0 or i >= len(names) or names[i] != name
                       for i, name in zip(indices, cfg['joint_names']))
                or runtime.get('observation_source') != 'SingleArticulation.get_measured_joint_efforts only'
                or runtime.get('force_frame') != 'world' or runtime.get('position_argument_frame') != 'world'):
            raise ValueError
        gains = np.asarray(runtime['dof_stiffness'], float)
        damping = np.asarray(runtime['dof_damping'], float)
        gravity = runtime['physics_gravity']
        if (gains.shape != (len(names),) or damping.shape != (len(names),)
                or not np.isfinite(np.r_[gains, damping]).all() or np.any(gains[indices] <= 0)
                or not np.allclose(np.asarray(gravity[0]) * gravity[1], cfg['gravity_m_s2'], atol=1e-6, rtol=0)
                or abs(runtime['physics_dt_s'] - cfg['physics_dt_s']) > 1e-12):
            raise ValueError
    except (KeyError, ValueError, TypeError, IndexError):
        failures.append('INVALID_JOINT_ORDER_SOURCE_OR_RUNTIME')
    sc, th = cfg['independent_sign_control'], cfg['thresholds']
    origin = np.asarray(sc['axis_origin_world_m'], dtype=float)
    axis = np.asarray(sc['q1_axis_world'], dtype=float)
    if not np.allclose(axis, [0,0,1], atol=0, rtol=0) or not np.allclose(origin, cfg['base_world_m'], atol=0, rtol=0):
        failures.append('AXIS_ORIGIN_CONFIGURATION_INVALID')
    for load in cfg['loads']:
        case = {'load_id': load['id'], 'status': 'REJECTED', 'failures': []}
        result['cases'].append(case)
        try:
            pair = {kind: read_rows(input_dir / f"home_{load['id']}_{kind}.json", cfg, digest, load, kind)
                    for kind in ('unloaded','loaded')}
            u, l = pair['unloaded'], pair['loaded']
            moment = float(np.mean(np.cross(l['p'] - origin, l['force']) @ axis))
            delta_effort = float(np.mean(l['effort'][:,0]) - np.mean(u['effort'][:,0]))
            delta_q = float(np.mean(l['q'][:,0]) - np.mean(u['q'][:,0]))
            case.update({'moment_external_on_robot_q1_nm': moment, 'delta_measured_effort_q1_nm': delta_effort,
                         'delta_q1_rad': delta_q, 'equilibrium_residual_nm': abs(delta_effort + moment),
                         'opposite_hypothesis_residual_nm': abs(delta_effort - moment),
                         'input_sha256': {kind: item['sha256'] for kind,item in pair.items()},
                         'observation_times_s': {kind: [float(item['times'][0]), float(item['times'][-1])] for kind,item in pair.items()}})
            cf = case['failures']
            if max(np.max(np.abs(item['dq'])) for item in pair.values()) > sc['max_abs_velocity_rad_s']:
                cf.append('DYNAMIC_STATE')
            if np.max(np.abs(np.mean(l['q'], axis=0) - np.mean(u['q'], axis=0))) > th['max_pair_mean_q_difference_rad']:
                cf.append('UNMATCHED_POSTURE')
            if max(np.max(np.ptp(item['q'],axis=0)) for item in pair.values()) > th['max_window_q_range_rad']:
                cf.append('UNSTABLE_Q_WINDOW')
            if max(np.max(np.std(item['effort'],axis=0)) for item in pair.values()) > th['max_window_effort_std_nm']:
                cf.append('UNSTABLE_EFFORT_WINDOW')
            if abs(moment) < sc['min_abs_moment_nm']:
                cf.append('SIGNAL_TOO_SMALL')
            if delta_q * moment <= 0:
                cf.append('POSITION_DISPLACEMENT_DISAGREES_WITH_APPLIED_MOMENT')
            if delta_effort * moment >= 0:
                cf.append('MEASURED_EFFORT_NOT_OPPOSED_TO_MOMENT')
            if abs(delta_effort + moment) > sc['max_q1_effort_plus_moment_error_nm']:
                cf.append('EQUILIBRIUM_RESIDUAL_TOO_LARGE')
            if abs(delta_effort - moment) <= abs(delta_effort + moment):
                cf.append('ALTERNATIVE_SIGN_NOT_EXCLUDED')
            if not cf:
                case['status'] = 'PASS'
        except (KeyError, OSError, TypeError, ValueError) as exc:
            case['failures'].append(str(exc))
    if not failures and result['cases'] and all(case['status'] == 'PASS' for case in result['cases']):
        result['status'] = 'SIGN_CONTROL_PASS'
        result['independent_equilibrium_confirmed'] = True
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise SystemExit('OUTPUT_ALREADY_EXISTS')
    result = evaluate(args.input)
    with args.output.open('x', encoding='utf8') as f:
        json.dump(result, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')
    return 0 if result['status'] == 'SIGN_CONTROL_PASS' else 1

if __name__ == '__main__':
    raise SystemExit(main())
