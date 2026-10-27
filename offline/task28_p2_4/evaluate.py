#!/usr/bin/env python3
"""Causal, oracle-separated evaluation of frozen P2-4 collector records.

The sensor/model pass opens observations only. The independent applied-wrench
oracle is loaded only after all observer calls have completed. Candidate
estimates are never control commands or proof of real FR3 force sensing.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'offline/task28_g1_1'))
from task28_validator.model import RobotModel
from offline.task28_p2_4.observer import ObserverContract, Sample, step

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'evidence/experiment_preregistration.json'
PREFLIGHT = HERE / 'evidence/api_preflight.json'
FLAGS = {'execution_authorized': False, 'physical_robot_tested': False,
         'wrist_ft_sensor_required': False, 'dual_arm_force_control_implemented': False,
         'production_bridge_modified': False}


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def _read_json(p):
    return json.loads(Path(p).read_text(encoding='utf-8'))


def estimate_observations(obs, config, model, source_hash):
    """Run causally with no case load or oracle argument. Retain every frame."""
    names = tuple(config['expected_all_dof_names'])
    arm = tuple(config['joint_names'])
    dt = config['physics_dt_s']
    # Inclusive tiny float tolerance only for binary timestep representation.
    contract = ObserverContract(names, arm, 'world', 'left_side_tcp', dt + 1e-8,
        dt + 1e-8, dt - 1e-6, dt + 1e-6, 1e-10, 1e6,
        'SOURCE_UNVALIDATED_CANDIDATE', source_hash,
        ('revolute',)*7 + ('prismatic',)*2, 0.1)
    units = ('rad,rad/s,rad/s2,N*m',)*7 + ('m,m/s,m/s2,N',)*2
    source = {'q':'Isaac.get_joint_positions','dq':'Isaac.get_joint_velocities',
        'effort':'Isaac.get_measured_joint_efforts (projected solver force)',
        'mass':'Isaac.get_mass_matrices',
        'coriolis':'Isaac.get_coriolis_and_centrifugal_forces',
        'gravity':'Isaac.get_generalized_gravity_forces',
        'jacobian':'frozen G1 RobotModel.jacobian at current q'}
    history = None
    rows = []
    for x in obs['samples']:
        started = time.perf_counter()
        row = {'case_step': x.get('case_step'), 'sim_time_s': x.get('sim_time_s'),
               'rejection_reasons': []}
        try:
            q = np.asarray(x['q_rad_or_m'], float)
            dq = np.asarray(x['dq_rad_s_or_m_s'], float)
            if q.shape != (9,) or dq.shape != (9,):
                raise ValueError('SOURCE_SHAPE_INVALID')
            J = model.jacobian('left', q[:7])
            time_s = float(x['sim_time_s'])
            start_s = float(x['effort_interval_start_s'])
            if abs(float(x['effort_interval_end_s']) - time_s) > 1e-8:
                raise ValueError('EFFORT_INTERVAL_END_MISMATCH')
            if abs((time_s - start_s) - dt) > 1e-6:
                raise ValueError('EFFORT_INTERVAL_DURATION_MISMATCH')
            sample = Sample(names, arm, q, dq, x['measured_projected_effort'],
                x['mass_matrix'], x['coriolis_compensation'], x['gravity_compensation'],
                J, q.copy(), dq.copy(), q[:7].copy(), time_s, time_s, start_s,
                time_s, time_s, time_s, str(x['case_step']), source, 'world',
                'left_side_tcp', ('revolute',)*7+('prismatic',)*2, units,
                'effort_i/acceleration_j')
            result = step(sample, contract, history)
            history = result.history
            row.update({'observer_status': result.status, 'sample_age_s': result.sample_age_s,
                'sample_skew_s': result.max_sample_skew_s, 'derivative_dt_s': result.derivative_dt_s,
                'acceleration': result.acceleration_SI, 'inertial_term': result.inertial_term_SI,
                'tau_external_on_robot_nm': result.arm_external_torque_on_robot_Nm,
                'predicted_wrench_on_robot': (result.wrench.wrench_minimum_norm if result.wrench else None),
                'singular_values': (result.wrench.singular_values if result.wrench else None),
                'jacobian_rank': (result.wrench.rank if result.wrench else None),
                'jacobian_condition': (result.wrench.condition_number if result.wrench else None),
                'jacobian_residual_nm': (result.wrench.reconstruction_residual_Nm if result.wrench else None),
                'source_read_duration_s': x['source_read_duration_s'],
                'link7_fk_position_error_m': x['link7_fk_position_error_m'],
                'link7_fk_rotation_error_rad': x['link7_fk_rotation_error_rad'],
                'max_arm_velocity_rad_s': float(np.max(np.abs(dq[:7]))),
                'tcp_position_world_m': model.fk('left',q[:7])[:3,3].tolist(),
                'jacobian': J.tolist()})
            if result.status not in ('CANDIDATE_ESTIMATE', 'WARMUP'):
                row['rejection_reasons'].append(result.status)
            if result.wrench and result.wrench.singular_values[-1] < config['thresholds']['minimum_jacobian_singular_value']:
                row['rejection_reasons'].append('JACOBIAN_SIGMA_BELOW_THRESHOLD')
            if x['link7_fk_position_error_m'] > config['thresholds']['max_link7_fk_position_difference_m']:
                row['rejection_reasons'].append('LINK7_FK_POSITION_ERROR')
            if x['link7_fk_rotation_error_rad'] > config['thresholds']['max_link7_fk_rotation_difference_rad']:
                row['rejection_reasons'].append('LINK7_FK_ROTATION_ERROR')
        except (ValueError, TypeError, KeyError, IndexError, np.linalg.LinAlgError) as exc:
            row['observer_status'] = 'REJECTED'
            row['rejection_reasons'].append(str(exc))
            # A failed frame interrupts consecutive derivative evidence.
            history = None
        row['observer_wall_s'] = time.perf_counter() - started
        if 'source_read_duration_s' in row:
            row['read_plus_observer_wall_s'] = row['source_read_duration_s'] + row['observer_wall_s']
            if row['read_plus_observer_wall_s'] > config['latency_budget_s']:
                row['rejection_reasons'].append('LATENCY_BUDGET_EXCEEDED')
        rows.append(row)
    return rows


def add_oracle_scores(rows, oracle, config, motion):
    """Evaluate after observer calls; applied wrench is never an input to step."""
    force = np.asarray(oracle['force_world_n'], float)
    torque = np.asarray(oracle['pure_torque_world_nm'], float)
    points = oracle['application_points_world_m']
    if force.shape != (3,) or torque.shape != (3,) or len(points) != len(rows):
        raise ValueError('ORACLE_SHAPE_INVALID')
    if not np.isfinite(np.r_[force,torque]).all():
        raise ValueError('ORACLE_NONFINITE')
    limits = config['thresholds']
    for i, row in enumerate(rows):
        if row.get('predicted_wrench_on_robot') is None:
            continue
        point = np.asarray(points[i], float)
        if point.shape != (3,) or not np.isfinite(point).all():
            row['rejection_reasons'].append('ORACLE_POINT_INVALID')
            continue
        # Isaac load is applied at pre-step p; wrench moment is shifted to
        # the measured post-step candidate TCP point, in the same world frame.
        w = np.r_[force, torque + np.cross(point - row['tcp_position_world_m'], force)]
        J = np.asarray(row['jacobian'])
        expected_tau = J.T @ w
        error_tau = np.asarray(row['tau_external_on_robot_nm']) - expected_tau
        error_wrench = np.asarray(row['predicted_wrench_on_robot']) - w
        row.update({'oracle_wrench_on_robot': w.tolist(), 'oracle_tau_nm': expected_tau.tolist(),
                    'joint_torque_error_nm': error_tau.tolist(),
                    'max_abs_joint_torque_error_nm': float(np.max(np.abs(error_tau))),
                    'force_error_n': float(np.linalg.norm(error_wrench[:3])),
                    'moment_error_nm': float(np.linalg.norm(error_wrench[3:]))})
        if row['max_abs_joint_torque_error_nm'] > limits['max_joint_torque_error_nm']:
            row['rejection_reasons'].append('TORQUE_ERROR')
        if row['force_error_n'] > limits['max_force_error_n']:
            row['rejection_reasons'].append('FORCE_ERROR')
        if row['moment_error_nm'] > limits['max_moment_error_nm']:
            row['rejection_reasons'].append('MOMENT_ERROR')
        if not motion and row['max_arm_velocity_rad_s'] > limits['max_static_joint_velocity_rad_s']:
            row['rejection_reasons'].append('NOT_STATIONARY')


def evaluate(input_dir, output, stage):
    config = _read_json(CONFIG)
    status = _read_json(input_dir/'collector_status.json')
    expected_config_hash = sha(CONFIG)
    if status.get('preregistration_sha256') != expected_config_hash or status.get('stage') != stage:
        raise ValueError('RUN_OR_PREREGISTRATION_ID_MISMATCH')
    runtime = status.get('runtime', {})
    names_ok = runtime.get('all_dof_names') == config['expected_all_dof_names']
    idx_ok = runtime.get('arm_indices') == [runtime.get('all_dof_names',[]).index(n) for n in config['joint_names']] if names_ok else False
    gains_ok = True
    for key in ('gains_stiffness', 'gains_damping'):
        try:
            gains = np.asarray(runtime[key], float).reshape(-1)
            gains_ok &= gains.shape == (9,) and bool(np.isfinite(gains).all()) and bool((gains >= 0).all())
        except (KeyError, ValueError, TypeError):
            gains_ok = False
    prerequisites = {'collection_complete': status.get('status') == 'COLLECTION_COMPLETE_NOT_EVALUATED',
        'dof_names_match':names_ok, 'arm_indices_match':idx_ok, 'gains_finite_nonnegative_and_shape_9':gains_ok,
        'physics_dt_match':abs(float(runtime.get('physics_dt_s',-1))-config['physics_dt_s']) < 1e-12}
    model = RobotModel()
    results = {}
    for case in config['cases']:
        if (stage == 'static' and case['motion']) or (stage != 'static' and case['id'] != stage):
            continue
        cid = case['id']
        observation_file = input_dir/(cid+'_observations.json')
        oracle_file = input_dir/(cid+'_oracle.json')
        if not observation_file.is_file() or not oracle_file.is_file():
            results[cid] = {'pass':False,'rejection_reasons':['MISSING_OBSERVATION_OR_ORACLE'],'rows':[]}
            continue
        obs = _read_json(observation_file)
        if obs.get('case_id') != cid or obs.get('preregistration_sha256') != expected_config_hash:
            results[cid] = {'pass':False,'rejection_reasons':['OBSERVATION_ID_MISMATCH'],'rows':[]}
            continue
        rows = estimate_observations(obs, config, model, sha(PREFLIGHT))
        # Access the oracle only after the causal observer has finished this case.
        oracle = _read_json(oracle_file)
        if oracle.get('case_id') != cid:
            results[cid] = {'pass':False,'rejection_reasons':['ORACLE_ID_MISMATCH'],'rows':rows}
            continue
        try:
            add_oracle_scores(rows, oracle, config, case['motion'])
        except (ValueError, KeyError, TypeError) as exc:
            results[cid] = {'pass':False,'rejection_reasons':[str(exc)],'rows':rows}
            continue
        eval_rows = rows[config['static_evaluation_start_step']:] if not case['motion'] else rows[1:]
        all_source_failures = [r for r in rows if r.get('observer_status') == 'REJECTED']
        errors = sorted({reason for r in eval_rows for reason in r['rejection_reasons']})
        if all_source_failures:
            errors.append('SOURCE_SAMPLE_FAILURE')
        if len(rows) != (config['motion_steps'] if case['motion'] else config['static_steps']):
            errors.append('SAMPLE_COUNT_MISMATCH')
        if not eval_rows or any(r.get('predicted_wrench_on_robot') is None for r in eval_rows):
            errors.append('MISSING_CANDIDATE')
        if not all(prerequisites.values()):
            errors.append('SOURCE_PREREQUISITE_FAILED')
        if case['motion']:
            errors.append('INDEPENDENT_DYNAMIC_SOURCE_SEMANTICS_NOT_VALIDATED')
        results[cid] = {'pass':not errors, 'rejection_reasons':sorted(set(errors)),
                        'eval_start_step':config['static_evaluation_start_step'] if not case['motion'] else 1,
                        'observation_sha256':sha(observation_file), 'oracle_sha256':sha(oracle_file), 'rows':rows}
    gate = all(prerequisites.values()) and bool(results) and all(x['pass'] for x in results.values())
    result = {'stage':stage, 'status':'STATIC_SOURCE_AND_ORACLE_GATE_PASS' if gate and stage == 'static' else 'GATE_REJECTED_OR_INCONCLUSIVE',
              'gate_pass':gate and stage == 'static', 'preregistration_sha256':expected_config_hash,
              'preflight_sha256':sha(PREFLIGHT), 'source_prerequisites':prerequisites,
              'boundaries':FLAGS, 'cases':results,
              'interpretation':'Source-gated candidate estimates only; online physical, dynamic and bilateral control claims are not implied.'}
    with output.open('x',encoding='utf-8') as f:
        json.dump(result,f,allow_nan=False,indent=2)
        f.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stage', choices=('static','motion_zero','motion_plus'), required=True)
    args = parser.parse_args()
    result=evaluate(args.input,args.output,args.stage)
    print(json.dumps({'status':result['status'],'gate_pass':result['gate_pass'],
          'cases':{k:{'pass':v['pass'],'reasons':v['rejection_reasons']} for k,v in result['cases'].items()}},indent=2))
    return 0 if result['gate_pass'] else 1


if __name__=='__main__':
    sys.exit(main())
