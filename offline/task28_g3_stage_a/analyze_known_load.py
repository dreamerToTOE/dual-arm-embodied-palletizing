"""Compare injected known forces with the next completed-step reaction and model residual."""
from __future__ import annotations

import argparse
import json
import lzma
from collections import defaultdict
from pathlib import Path
import numpy as np
from offline.task28_g3_stage_a.analyze_reaction import joint_world_wrench


def analyze(run_dir):
    path = run_dir / 'task27_torque_bridge.jsonl'
    opener = open
    if not path.exists():
        path = run_dir / 'task27_torque_bridge.jsonl.xz'
        opener = lzma.open
    metadata = None
    prev_command = None
    samples = defaultdict(list)
    statuses = defaultdict(int)
    with opener(path, 'rt') as stream:
        for line in stream:
            frame = json.loads(line)
            event = frame.get('event')
            if event == 'REACTION_SOURCE_METADATA':
                metadata = frame['sides']
            elif event == 'KNOWN_LOAD_COMMAND':
                prev_command = frame
            elif event == 'TORQUE_FRAME' and prev_command is not None:
                step = frame['physics_step']
                if prev_command['physics_step'] != step - 1:
                    raise ValueError('KNOWN_LOAD_COMMAND_STEP_MISMATCH')
                command, prev_command = prev_command, None
                status = frame['qp']['status']
                statuses[status] += 1
                side = command['body_path'].split('/')[2].split('_')[0]
                data = frame['reaction_diagnostic']['sides'][side]
                if not data['valid']:
                    raise ValueError('INVALID_REACTION')
                force = np.asarray(command['force_world_N'])
                point = np.asarray(command['application_point_world_m'])
                tcp = np.asarray(frame['tcp_pose_world'][side])[:3, 3]
                ref_wrench = np.r_[force, np.cross(point - tcp, force)]
                qp = frame['qp']
                qdd = frame['measured_qdd14_rad_s2']
                if qdd is None:
                    continue
                arm_slice = slice(0, 7) if side == 'left' else slice(7, 14)
                M = np.asarray(qp['full_mass_matrix_used'][side])[:7, :7]
                bias = np.asarray(qp['full_coriolis_Nm'][side])[:7] + np.asarray(qp['full_gravity_Nm'][side])[:7]
                applied = np.asarray(frame['reaction_diagnostic']['applied_effort_completed_interval_Nm'][side])
                residual = M @ np.asarray(qdd)[arm_slice] + bias - applied
                J = np.asarray(frame['reaction_diagnostic']['jacobian_world_at_tcp'][side])
                estimate = np.linalg.lstsq(J.T, residual, rcond=1e-8)[0]
                incoming, _ = joint_world_wrench(metadata[side], data, 'fr3_joint8')
                samples[command['case_index']].append({
                    'time_s': frame['time_s'], 'side': side, 'force_N': force.tolist(),
                    'reference_wrench_at_tcp_N_Nm': ref_wrench.tolist(),
                    'model_wrench_at_tcp_N_Nm': estimate.tolist(),
                    'joint8_wrench_world_N_Nm': incoming.tolist(),
                    'joint_torque_residual_error_inf_Nm': float(np.max(np.abs(residual - J.T @ ref_wrench))),
                    'q_rad': frame['q_rad'][side], 'dq_rad_s': frame['dq_rad_s'][side],
                    'tau_box_violation': qp['tau_box_violation'],
                    'torque_saturated': bool(qp['torque_saturated_indices']),
                    'rate_active': bool(qp['torque_rate_active_indices']),
                })
    cases = []
    for index, rows in sorted(samples.items()):
        last = rows[-1]['time_s']
        steady = [row for row in rows if last - 1.0 <= row['time_s'] <= last]
        a = lambda key: np.asarray([row[key] for row in steady], dtype=float)
        cases.append({'index': index, 'side': rows[0]['side'], 'force_N': rows[0]['force_N'],
                      'samples': len(rows), 'steady_samples': len(steady),
                      'steady_time_s': [steady[0]['time_s'], steady[-1]['time_s']],
                      'model_wrench_mean': a('model_wrench_at_tcp_N_Nm').mean(axis=0).tolist(),
                      'injected_wrench_mean': a('reference_wrench_at_tcp_N_Nm').mean(axis=0).tolist(),
                      'joint8_wrench_mean': a('joint8_wrench_world_N_Nm').mean(axis=0).tolist(),
                      'joint8_wrench_std': a('joint8_wrench_world_N_Nm').std(axis=0).tolist(),
                      'joint_residual_error_inf_mean_Nm': float(a('joint_torque_residual_error_inf_Nm').mean()),
                      'max_abs_dq_rad_s': float(np.max(np.abs(a('dq_rad_s')))),
                      'q_mean_rad': a('q_rad').mean(axis=0).tolist(),
                      'torque_saturation_frames': sum(r['torque_saturated'] for r in rows),
                      'rate_active_frames': sum(r['rate_active'] for r in rows)})
    result = {'classification': 'KNOWN_INJECTED_LOAD_NOT_SUCTION_CONTACT', 'cases': cases,
              'qp_status': dict(statuses), 'reaction_is_controller_input': False,
              'timing': 'known load command at step k paired to measured reaction at k+1'}
    (run_dir / 'known_load_metrics.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.run_dir), indent=2))
