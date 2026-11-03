"""Produce compact Stage B0 evidence from the unmodified 60 Hz trace."""
import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace


def frame_receipt(frame):
    cube = frame['cube_runtime']
    qp = frame['qp']
    closed = (frame.get('reaction_diagnostic') or {}).get('suctions_closed') or {}
    return {
        'time_s': frame['time_s'], 'mode': (qp.get('trajectory_adapter') or {}).get('task_mode'), 'qp_status': qp.get('status'),
        'cube_xyz_m': cube['pose_world']['position_m'],
        'cube_quaternion_wxyz': cube['pose_world']['orientation_xyzw'],
        'cube_velocity_m_s': cube['linear_velocity_m_s'],
        'table_force_world_N': cube.get('table_contact_force_world_N'),
        'table_bottom_gap_m': cube.get('table_bottom_gap_m'),
        'tcp_xyz_m': frame['tcp_world_m'], 'grippers_closed': closed,
        'relative_objective_enabled': qp.get('relative_objective_enabled'),
        'independent_joint_tracking': qp.get('independent_joint_tracking'),
        'b0_pusher_tracking': qp.get('b0_pusher_tracking'),
        'common_weight': qp.get('common_weight'), 'relative_weight': qp.get('relative_weight'),
        'object_reference_active': (qp.get('object_reference') or {}).get('active', False),
        'g3_load_model': qp.get('g3_qp_contact_load_model'),
        'payload_active': (qp.get('payload_gravity_compensation') or {}).get('active', False),
        'q_error_rad': frame['q_error_rad'],
        'applied_torque_Nm': frame['canonical_command_effort_Nm'],
    }


def analyze(run):
    summary = json.loads((run / 'summary.json').read_text())
    executor = (run / 'task27_executor.log').read_text(errors='replace')
    events = []; samples = []; per_mode = Counter(); status = Counter(); collision_events = Counter()
    dual = release = single = last_single = final_release = last = None
    max_tau = np.zeros(14); max_rate = np.zeros(14); max_velocity = 0.; max_accel = 0.
    previous_tau = None; tracking = []; single_qp_count = 0; single_independent = True
    with open_trace(run) as stream:
        for line in stream:
            frame = json.loads(line)
            event = frame.get('event')
            if event == 'TASK_MODE_EVENT_TRANSITION':
                events.append(frame)
            if event == 'PLACEMENT_SHAPE_CONTACT':
                collision_events[' <-> '.join(frame['colliders'])] += 1
            if event != 'TORQUE_FRAME':
                continue
            row = frame_receipt(frame); qp = frame['qp']; closed = row['grippers_closed']
            both = all(closed.get(s) is True for s in ('left', 'right'))
            opened = all(closed.get(s) is False for s in ('left', 'right'))
            one = sum(closed.get(s) is True for s in ('left', 'right')) == 1
            if both and dual is None: dual = row
            if dual and opened and release is None: release = row
            if release and one and row['mode'] == 'SINGLE_ARM_PUSH':
                if single is None: single = row
                last_single = row
                single_qp_count += 1
                single_independent &= (row['independent_joint_tracking'] is True
                    and not row['relative_objective_enabled'] and not row['object_reference_active']
                    and not row['payload_active'] and row['g3_load_model'] == 'no_internal_contact_load_in_dynamics')
                tracking.extend(abs(x) for q in row['q_error_rad'].values() for x in q)
            if single and opened and final_release is None: final_release = row
            tau = np.r_[frame['canonical_command_effort_Nm']['left'], frame['canonical_command_effort_Nm']['right']]
            max_tau = np.maximum(max_tau, np.abs(tau))
            if previous_tau is not None:
                max_rate = np.maximum(max_rate, np.abs(tau - previous_tau) * 60.)
            previous_tau = tau
            max_velocity = max(max_velocity, max(abs(v) for side in ('left', 'right') for v in frame['dq_rad_s'][side]))
            if qp.get('qp_qdd14') is not None: max_accel = max(max_accel, max(abs(v) for v in qp['qp_qdd14']))
            per_mode[row['mode']] += 1
            status[row['qp_status']] += 1
            if frame['physics_step'] % 6 == 0: samples.append(row)
            last = row
    tau_limits = np.array([87., 87., 87., 87., 12., 12., 12.] * 2)
    rate_limits = np.array([2400., 600., 240., 2400., 240., 240., 240.] * 2)
    displacement = None if not single or not last_single else (np.array(last_single['cube_xyz_m']) - single['cube_xyz_m']).tolist()
    result = {
        'run_id': run.name, 'summary_status': summary.get('status'), 'executor_exit': summary.get('executor_exit'),
        'torque_latched_error': summary.get('torque_latched_error'), 'position_arm_write_count': summary.get('position_arm_write_count'),
        'qp_status_counts_summary': summary.get('qp_status_counts'), 'sampled_status_counts': dict(status),
        'torque_frames_by_mode': dict(per_mode), 'transitions': events,
        'first_dual_attachment': dual, 'stage_a_first_both_open': release,
        'first_single_attachment': single, 'last_single_attachment': last_single,
        'first_final_both_open': final_release, 'final': last,
        'single_attachment_cube_displacement_xyz_m': displacement,
        'deep_wall_gap_m': None if not last else 1.1 - last['cube_xyz_m'][0],
        'side_wall_gap_m': None if not last else .243 - last['cube_xyz_m'][1],
        'single_attachment_qp_frames': single_qp_count,
        'single_attachment_independent_no_bilateral_model': bool(single_qp_count and single_independent),
        'single_attachment_joint_tracking_peak_rad': max(tracking) if tracking else None,
        'single_attachment_joint_tracking_rms_rad': float(np.sqrt(np.mean(np.square(tracking)))) if tracking else None,
        'applied_torque_peak_per_joint_Nm': max_tau.tolist(), 'torque_limit_per_joint_Nm': tau_limits.tolist(),
        'torque_rate_peak_per_joint_Nm_s': max_rate.tolist(), 'torque_rate_limit_per_joint_Nm_s': rate_limits.tolist(),
        'torque_bounds_observed': bool(np.all(max_tau <= tau_limits + 1e-5) and np.all(max_rate <= rate_limits + 1e-4)),
        'max_measured_joint_velocity_rad_s': max_velocity, 'max_qp_acceleration_rad_s2': max_accel,
        'contact_diagnostic_event_counts': dict(collision_events),
        'executor_b0_pass_marker': 'TASK28_B0_ONLY PASS' in executor,
    }
    return result, samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    result, samples = analyze(args.run)
    (args.output / 'b0_metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    with (args.output / 'b0_series.jsonl').open('x') as stream:
        for sample in samples: stream.write(json.dumps(sample, allow_nan=False) + '\n')
    print(json.dumps({k: result[k] for k in ('run_id', 'summary_status', 'executor_exit',
        'single_attachment_cube_displacement_xyz_m', 'deep_wall_gap_m', 'executor_b0_pass_marker')}, indent=2))


if __name__ == '__main__':
    main()
