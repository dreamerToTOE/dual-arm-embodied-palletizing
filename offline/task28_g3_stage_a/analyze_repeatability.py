"""Compare Stage-A runs using full-rate measured and object-reference frames."""
import argparse
from collections import Counter
import json
from pathlib import Path
import re

import numpy as np

from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace


def phase_boundaries(rows):
    """Infer phases from object reference geometry, not ROS wall-clock stamps."""
    if not rows:
        return None
    high = max(r['desired_xyz_m'][2] for r in rows)
    initial = rows[0]['desired_xyz_m'][2]
    lift_end = next((i for i, r in enumerate(rows) if r['desired_xyz_m'][2] >= high - .005), None)
    if high - initial < .05 or lift_end is None:
        return {'lift_end': len(rows), 'transport_end': len(rows), 'terminal_start': len(rows)}
    descent_start = next((i for i in range(lift_end, len(rows))
                          if rows[i]['desired_xyz_m'][2] < high - .005), len(rows))
    terminal_start = next((i for i in range(descent_start, len(rows))
                           if rows[i]['terminal_active']), len(rows))
    return {'lift_end': lift_end, 'transport_end': descent_start, 'terminal_start': terminal_start}


def phase_statistics(rows):
    boundaries = phase_boundaries(rows)
    if boundaries is None:
        return {}, None
    a, b, c = (boundaries[k] for k in ('lift_end', 'transport_end', 'terminal_start'))
    phases = {}
    for name, subset in (('lift', rows[:a]), ('transport', rows[a:b]),
                         ('descent_and_short_push', rows[b:c]), ('terminal_placement', rows[c:])):
        if not subset:
            phases[name] = None
            continue
        errors = np.array([r['error_mm'] for r in subset])
        peak = max(subset, key=lambda r: r['error_mm'])
        phases[name] = {'start_time_s': subset[0]['time_s'], 'end_time_s': subset[-1]['time_s'],
                        'sample_count': len(subset), 'rms_mm': float(np.sqrt(np.mean(errors**2))),
                        'peak_mm': float(peak['error_mm']), 'peak_time_s': peak['time_s']}
    return phases, boundaries


def process_status(log, node):
    died = re.findall(rf'\[ERROR\] \[{re.escape(node)}-\d+\]: process has died \[pid \d+, exit code (-?\d+)', log)
    if died:
        return {'status': 'exited', 'code': int(died[-1])}
    if re.search(rf'\[INFO\] \[{re.escape(node)}-\d+\]: process has finished cleanly', log):
        return {'status': 'exited', 'code': 0}
    return {'status': 'not_observed', 'code': None}


def analyze(run):
    summary = json.loads((run / 'summary.json').read_text())
    executor = (run / 'task27_executor.log').read_text(errors='replace')
    moveit = (run / 'moveit.log').read_text(errors='replace')
    loaded = []; sampled = []; after_release = []; contact_pairs = Counter(); contact_times = []
    prior = None; first_open = None; last = None; was_closed = False
    max_tau = np.zeros(14); max_rate = np.zeros(14); prev_tau = None
    max_step_m = 0.; prev_loaded_pos = None; max_qdd = 0.; max_dq = 0.
    post_release_ff_disabled = True; post_release_grippers_open = True; saw_post_release = False
    with open_trace(run) as stream:
        for line in stream:
            frame = json.loads(line)
            if frame.get('event') == 'PLACEMENT_SHAPE_CONTACT':
                paths = frame['colliders']
                if any(p == '/World/Table' or p.startswith('/World/Table/') for p in paths):
                    contact_pairs[' <-> '.join(paths)] += 1
                    contact_times.append(frame['time_s'])
                continue
            if frame.get('event') != 'TORQUE_FRAME':
                continue
            qp = frame['qp']; cube = frame['cube_runtime']; pose = cube['pose_world']
            xyz = np.array(pose['position_m']); vel = np.array(cube['linear_velocity_m_s'])
            closed = (frame.get('reaction_diagnostic') or {}).get('suctions_closed') or {}
            closed_both = all(closed.get(s) is True for s in ('left', 'right'))
            open_both = all(closed.get(s) is False for s in ('left', 'right'))
            tau = np.r_[frame['canonical_command_effort_Nm']['left'], frame['canonical_command_effort_Nm']['right']]
            max_tau = np.maximum(max_tau, np.abs(tau))
            if prev_tau is not None:
                max_rate = np.maximum(max_rate, np.abs(tau - prev_tau) * 60.)
            prev_tau = tau
            max_dq = max(max_dq, float(np.max(np.abs(np.r_[frame['dq_rad_s']['left'], frame['dq_rad_s']['right']]))))
            if qp.get('qp_qdd14') is not None:
                max_qdd = max(max_qdd, float(np.max(np.abs(qp['qp_qdd14']))))
            row = {'time_s': frame['time_s'], 'cube_xyz_m': xyz.tolist(),
                   'cube_velocity_m_s': vel.tolist(), 'cube_vertical_acceleration_m_s2': (cube.get('linear_acceleration_m_s2') or [None]*3)[2],
                   'bottom_gap_m': cube.get('table_bottom_gap_m'), 'table_force_N': cube.get('table_contact_force_world_N'),
                   'support_evidence': cube.get('table_support_evidence'), 'grippers_closed': closed,
                   'load_model': qp.get('g3_qp_contact_load_model'),
                   'payload_active': (qp.get('payload_gravity_compensation') or {}).get('active', False),
                   'position_arm_write_count': frame.get('position_arm_write_count')}
            if closed_both:
                was_closed = True
            elif was_closed and open_both:
                if first_open is None:
                    first_open = row.copy()
            if first_open is not None:
                saw_post_release = True
                post_release_grippers_open &= open_both
                post_release_ff_disabled &= (not row['payload_active'] and row['load_model'] == 'no_internal_contact_load_in_dynamics')
                after_release.append(row)
            obj = qp.get('object_reference') or {}
            if obj.get('active') and closed_both:
                desired = np.asarray(obj['T_WO_des'])[:3, 3]
                error = 1000. * (desired - xyz)
                current_twist = np.mean([qp['predicted_tcp_twist_world'][s] for s in ('left', 'right')], axis=0)
                actual_common = None
                if prior is not None and frame['time_s'] > prior['time_s']:
                    actual_common = (current_twist - prior['common_tcp_velocity']) / (frame['time_s'] - prior['time_s'])
                model_previous = None if prior is None else prior['common_predicted_accel']
                common_accel = np.asarray(qp['common_accel_desired'])
                residual = np.asarray(qp['common_acceleration_residual'])
                row.update(desired_xyz_m=desired.tolist(), error_vector_mm=error.tolist(), error_mm=float(np.linalg.norm(error)),
                           desired_velocity=obj.get('object_reference_velocity_world'),
                           desired_acceleration=obj.get('object_reference_acceleration_world'),
                           actual_closure_mm=1000. * float(np.linalg.norm(obj['actual_closure_error'][:3])),
                           desired_closure_mm=1000. * float(np.linalg.norm(obj['desired_closure_error'][:3])),
                           terminal_active=bool(obj.get('terminal_placement_correction', {}).get('active')),
                           integral_accel=obj.get('terminal_placement_correction', {}).get('acceleration_world_m_s2'),
                           common_error=qp['common_tcp_error'], common_command=common_accel.tolist(),
                           common_qp_residual=residual.tolist(), relative_qp_residual=qp['relative_acceleration_residual'],
                           realized_common_accel=None if actual_common is None else actual_common.tolist(),
                           previous_model_common_accel=None if model_previous is None else model_previous.tolist(),
                           weights={k: qp[k] for k in ('common_weight', 'relative_weight', 'joint_posture_weight', 'internal_torque_weight')},
                           torque_saturated=qp['torque_saturated_indices'], torque_rate_active=qp['torque_rate_active_indices'],
                           qref_max_error_rad=max(max(abs(np.asarray(v))) for v in frame['q_error_rad'].values()),
                           g3=qp.get('g3_internal_force', {}), payload=qp.get('payload_gravity_compensation', {}))
                if prev_loaded_pos is not None:
                    max_step_m = max(max_step_m, float(np.linalg.norm(xyz - prev_loaded_pos)))
                prev_loaded_pos = xyz
                prior = {'time_s': frame['time_s'], 'common_tcp_velocity': current_twist,
                         'common_predicted_accel': (common_accel + residual)}
                loaded.append(row)
            if frame['physics_step'] % 6 == 0:
                sampled.append({k: v for k, v in row.items() if k != 'g3'})
            last = row
    phases, bounds = phase_statistics(loaded)
    peak = max(loaded, key=lambda r: r['error_mm']) if loaded else None
    terminal = loaded[bounds['terminal_start']:] if bounds else []
    premature_unload = [r for r in terminal if r['bottom_gap_m'] is not None
                        and r['bottom_gap_m'] > .0015 and r['table_force_N'] is not None
                        and r['table_force_N'][2] < .2 and r['payload'].get('active')
                        and r['payload'].get('load_fraction', 1.) < .99]
    worst_unload = min(premature_unload, key=lambda r: r['payload']['load_fraction']) if premature_unload else None
    model_error = [r['realized_common_accel'][2] - r['previous_model_common_accel'][2]
                   for r in terminal if r['realized_common_accel'] is not None and r['previous_model_common_accel'] is not None]
    terminal_rz = [r['common_qp_residual'][2] for r in terminal]
    position_writes = summary.get('position_arm_write_count')
    force_z = None if not last or not last['table_force_N'] else last['table_force_N'][2]
    final_error = None if not last else (1000. * (np.array(last['cube_xyz_m']) - [.77, .06, .26])).tolist()
    all_optimal = summary.get('qp_status_counts') == {'QP_OPTIMAL': summary.get('qp_solve_count')}
    completed = bool('STAGE_A_ONLY PASS' in executor and summary.get('status') == 'COMPLETED'
        and summary.get('executor_exit') == 0 and not summary.get('torque_latched_error')
        and first_open and last and all(not x for x in last['grippers_closed'].values())
        and abs(last['bottom_gap_m']) <= .0015 and np.linalg.norm(last['cube_velocity_m_s']) <= .002
        and saw_post_release and post_release_grippers_open and post_release_ff_disabled and all_optimal and position_writes == 0)
    result = {'run_id': run.name, 'configuration': {k: summary.get(k) for k in (
        'mode', 'live_qp', 'stage_a_only', 'consistent_object_reference', 'assumed_internal_contact_load',
        'internal_force_target_N', 'internal_force_weight', 'internal_force_kp', 'execution_time_scale',
        'local_sweep_limit_m', 'max_batches', 'headless', 'record_joint_reactions', 'scene_entry_sha256',
        'original_task26_scene_source_sha256', 'original_task26_bridge_source_sha256')},
        'physical_cycle_complete': completed, 'executor_exit': summary.get('executor_exit'),
        'torque_latched_error': summary.get('torque_latched_error'), 'position_arm_write_count': position_writes,
        'lift_gt_log': bool(re.search(r'COMMON_LIFT GT=', executor)),
        'transport_gt_log': bool(re.search(r'COMMON_X_TRAVEL GT=', executor)),
        'both_attached_observed': bool(loaded), 'first_both_open': first_open, 'final': last,
        'grippers_remained_open_after_release': bool(saw_post_release and post_release_grippers_open),
        'load_feedforward_disabled_after_release': bool(saw_post_release and post_release_ff_disabled),
        'final_xyz_error_mm': final_error,
        'final_xy_error_mm': None if final_error is None else float(np.linalg.norm(final_error[:2])),
        'last_both_closed': loaded[-1] if loaded else None,
        'release_to_final_height_mm': None if not first_open or not loaded else 1000. * (loaded[-1]['cube_xyz_m'][2] - last['cube_xyz_m'][2]),
        'first_open_to_final_height_mm': None if not first_open else 1000. * (first_open['cube_xyz_m'][2] - last['cube_xyz_m'][2]),
        'post_release_max_xy_drift_mm': None if not first_open else 1000. * max(float(np.linalg.norm(np.array(r['cube_xyz_m'][:2]) - first_open['cube_xyz_m'][:2])) for r in after_release),
        'post_release_duration_s': None if not first_open else last['time_s'] - first_open['time_s'],
        'release_velocity_mm_s': None if not first_open or not loaded else (1000. * np.array(loaded[-1]['cube_velocity_m_s'])).tolist(),
        'pre_release_table_force_N': None if not first_open or not loaded else loaded[-1]['table_force_N'],
        'final_table_force_z_N': force_z, 'phases': phases,
        'terminal_convergence_duration_s': None if not first_open or not terminal else first_open['time_s'] - terminal[0]['time_s'],
        'cube_tracking_peak_mm': None if not peak else peak['error_mm'],
        'cube_tracking_rms_mm': None if not loaded else float(np.sqrt(np.mean([r['error_mm']**2 for r in loaded]))),
        'actual_closure_peak_mm': None if not loaded else max(r['actual_closure_mm'] for r in loaded),
        'desired_closure_peak_mm': None if not loaded else max(r['desired_closure_mm'] for r in loaded),
        'peak_detail': peak, 'terminal_qp_z_residual_rms_m_s2': None if not terminal_rz else float(np.sqrt(np.mean(np.array(terminal_rz)**2))),
        'terminal_realized_vs_previous_qp_z_accel_rms_m_s2': None if not model_error else float(np.sqrt(np.mean(np.array(model_error)**2))),
        'terminal_vertical_accel_cap_fraction': None if not terminal else float(np.mean([abs(r['common_command'][2]) >= .1499 for r in terminal])),
        'terminal_unsupported_partial_load_samples': len(premature_unload),
        'terminal_unsupported_partial_load_duration_s': len(premature_unload) / 60.,
        'terminal_unsupported_partial_load_worst': worst_unload,
        'max_attached_cube_step_m': max_step_m, 'qp_status_counts': summary.get('qp_status_counts'),
        'applied_torque_peak_per_joint_Nm': max_tau.tolist(), 'applied_torque_rate_peak_per_joint_Nm_s': max_rate.tolist(),
        'torque_limits_Nm': [87.,87.,87.,87.,12.,12.,12.] * 2,
        'torque_rate_limits_Nm_s': [2400.,600.,240.,2400.,240.,240.,240.] * 2,
        'max_measured_joint_velocity_rad_s': max_dq, 'max_qp_joint_accel_rad_s2': max_qdd,
        'moveit_launch_exit': summary.get('moveit_exit'), 'move_group_child': process_status(moveit, 'move_group'),
        'joint_bridge_child': process_status(moveit, 'dual_joint_state_bridge.py'),
        'shutdown_excerpt': [line for line in moveit.splitlines() if ('move_group-' in line or 'dual_joint_state_bridge.py-' in line) and ('process has died' in line or 'process has finished cleanly' in line)],
        'robot_table_contact_event_counts': dict(contact_pairs),
        'robot_table_contact_time_range_s': [min(contact_times), max(contact_times)] if contact_times else None}
    return result, sampled


def plot(all_data, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), layout='constrained')
    for result, rows in all_data:
        terminal = (result['phases'].get('terminal_placement') or {}).get('start_time_s')
        rows = [r for r in rows if 'error_mm' in r and terminal is not None and terminal - 3 <= r['time_s'] <= terminal + 10]
        if not rows:
            continue
        t = [r['time_s'] - terminal for r in rows]
        axes[0].plot(t, [r['error_mm'] for r in rows], label=result['run_id'])
        axes[1].plot(t, [r['error_vector_mm'][2] for r in rows], label=result['run_id'])
        axes[2].plot(t, [r['common_qp_residual'][2] for r in rows], label=result['run_id'])
    for ax, title in zip(axes, ('Cube position tracking error (mm)', 'Desired minus actual Cube Z (mm)',
                                'QP common Z acceleration residual (m/s²)')):
        ax.axvline(0, color='black', alpha=.4, linewidth=.7)
        ax.set_title(title); ax.set_xlabel('Simulation time from terminal controller activation (s)')
        ax.grid(True); ax.legend()
    fig.savefig(output / 'terminal_tracking_comparison.png', dpi=150)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    all_data = [analyze(run) for run in args.runs]
    for result, data in all_data:
        (args.output / (result['run_id'] + '.json')).write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        with (args.output / (result['run_id'] + '_series.jsonl')).open('w') as stream:
            for row in data:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    plot(all_data, args.output)
    print(json.dumps([{'run_id': r['run_id'], 'physical_cycle_complete': r['physical_cycle_complete'],
                       'peak_mm': r['cube_tracking_peak_mm'], 'rms_mm': r['cube_tracking_rms_mm'],
                       'final_xy_error_mm': r['final_xy_error_mm'], 'phases': r['phases'],
                       'moveit_launch_exit': r['moveit_launch_exit'], 'move_group_child': r['move_group_child']}
                      for r, _ in all_data], indent=2))


if __name__ == '__main__':
    main()
