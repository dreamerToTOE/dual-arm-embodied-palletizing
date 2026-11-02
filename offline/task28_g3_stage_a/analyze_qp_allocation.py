"""Analyze target torque allocation and replay one logged QP with an assumed load."""
import argparse
import json
import lzma
from collections import defaultdict
from dataclasses import replace
from pathlib import Path

import numpy as np
from offline.task28_task27_live_qp.torque_qp import CooperativeTorqueQPConfig, solve_cooperative_torque_qp
from offline.task28_p5_qp_6_r12.qp_math import pair_tcp_geometric_relative_matrix


def analyze(run_dir):
    metrics = json.loads((run_dir / 'reaction_metrics.json').read_text())
    windows = [s['steady_time_s'] for s in metrics['segments']]
    rows = defaultdict(list)
    selected = None
    raw = run_dir / 'task27_torque_bridge.jsonl'
    path = raw if raw.exists() else run_dir / 'task27_torque_bridge.jsonl.xz'
    opener = open if raw.exists() else lzma.open
    with opener(path, 'rt') as stream:
        for line in stream:
            x = json.loads(line)
            if x.get('event') != 'TORQUE_FRAME':
                continue
            q = x['qp']; g = q.get('g3_internal_force') or {}
            if selected is None and x['time_s'] > 110 and g.get('target_N') == 2:
                selected = x
            for i, (lo, hi) in enumerate(windows):
                if not lo <= x['time_s'] <= hi:
                    continue
                target = q.get('internal_torque_target14_Nm')
                if target is None:
                    continue
                tau = np.asarray(q['qp_tau14']); target = np.asarray(target)
                it = np.asarray(g['internal_robot_torque_target_Nm']); bias = target - it
                applied = np.r_[x['applied_effort_command_Nm']['left'], x['applied_effort_command_Nm']['right']]
                rows[i].append({'target_N': g['target_N'], 'effective_N': g['effective_target_N'],
                                'target_internal_l2_Nm': float(np.linalg.norm(it)),
                                'torque_residual_l2_Nm': float(np.linalg.norm(tau - target)),
                                'torque_residual_inf_Nm': float(np.max(np.abs(tau - target))),
                                'tau_applied_diff_inf_Nm': float(np.max(np.abs(tau - applied))),
                                'target_direction_projection': float((tau - bias) @ it / (it @ it)),
                                'torque_saturated': bool(q['torque_saturated_indices']),
                                'torque_rate_active': bool(q['torque_rate_active_indices'])})
    segments = {}
    for i, items in rows.items():
        segments[str(i)] = {'samples': len(items)}
        for key in items[0]:
            values = np.asarray([r[key] for r in items], dtype=float)
            segments[str(i)][key + '_mean'] = float(values.mean())
            segments[str(i)][key + '_max'] = float(values.max())
    if selected is None:
        raise ValueError('NO_2N_FRAME')
    x = selected; qp = x['qp']; g = qp['g3_internal_force']
    if qp.get('g3_qp_contact_load_model') == 'assumed_commanded_wrench_not_measured':
        return {'source_run': str(run_dir), 'windows': segments,
                'counterfactual': None,
                'limitation': 'torque projection is not measured contact force'}
    J = x['reaction_diagnostic']['jacobian_world_at_tcp']; JL = np.asarray(J['left']); JR = np.asarray(J['right'])
    poses = x['tcp_pose_world']
    Jrel = pair_tcp_geometric_relative_matrix(np.asarray(poses['left']), np.asarray(poses['right'])) @ np.block([
        [JL, np.zeros((6, 7))], [np.zeros((6, 7)), JR]])
    mass = np.zeros((14, 14))
    for index, side in enumerate(('left', 'right')):
        mass[7*index:7*(index+1), 7*index:7*(index+1)] = np.asarray(qp['full_mass_matrix_used'][side])[:7, :7]
    target = np.asarray(qp['internal_torque_target14_Nm'])
    it = np.asarray(g['internal_robot_torque_target_Nm']); bias = target - it
    cfg = replace(CooperativeTorqueQPConfig(), torque_abs_Nm=(87.,87.,87.,87.,12.,12.,12.),
                  torque_rate_Nm_s=(2400.,600.,240.,2400.,2400.,2400.,2400.),
                  joint_posture_weight=1e4, internal_torque_weight=1000.)
    args = dict(q14=np.r_[x['q_rad']['left'], x['q_rad']['right']],
                dq14=np.r_[x['dq_rad_s']['left'], x['dq_rad_s']['right']], mass14=mass,
                jacobian_left=JL, jacobian_right=JR, jacobian_relative=Jrel,
                jdotdq_left=qp['jdotdq_left'], jdotdq_right=qp['jdotdq_right'],
                jdotdq_relative=qp['jdotdq_relative'], common_accel_desired=qp['common_accel_desired'],
                relative_accel_desired=qp['relative_accel_desired'], joint_accel_desired=qp['joint_accel_desired'],
                previous_tau14=np.r_[x['reaction_diagnostic']['applied_effort_completed_interval_Nm']['left'],
                                      x['reaction_diagnostic']['applied_effort_completed_interval_Nm']['right']],
                config=cfg, model_validated=True, feedback_fresh=True, dt_s=1/60, torque_target14=target)
    replay = {}
    for name, model_bias in (('baseline', bias), ('assumed_commanded_contact_load', target)):
        result = solve_cooperative_torque_qp(**args, bias14=model_bias)
        if result.tau14_Nm is None:
            raise RuntimeError('COUNTERFACTUAL_QP_FAILED:' + result.status)
        tau = np.asarray(result.tau14_Nm)
        replay[name] = {'status': result.status,
                        'target_direction_projection': float((tau - bias) @ it / (it @ it)),
                        'torque_target_residual_inf_Nm': result.diagnostics['internal_torque_target_residual_inf_Nm'],
                        'relative_acceleration_residual_inf': result.diagnostics['relative_acceleration_residual_inf'],
                        'recorded_tau_difference_inf_Nm': float(np.max(np.abs(tau - qp['qp_tau14'])))}
    if replay['baseline']['recorded_tau_difference_inf_Nm'] > 1e-4:
        raise ValueError('BASELINE_REPLAY_DOES_NOT_MATCH')
    return {'source_run': str(run_dir), 'windows': segments,
            'counterfactual_step': x['physics_step'], 'counterfactual_time_s': x['time_s'],
            'counterfactual': replay,
            'limitation': 'assumed contact wrench is not a measured force and offline replay is not plant validation'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = analyze(args.run_dir)
    payload = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(payload)
    print(payload, end='')
