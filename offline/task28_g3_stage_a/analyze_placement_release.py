"""Compact physical Stage A receipts, including the terminal 3 s and release."""
import argparse
from collections import Counter, deque
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from offline.task28_g3_stage_a.analyze_suspended_probe import open_trace


def analyze(run):
    loaded = []
    terminal = deque(maxlen=181)
    sampled = []
    statuses = Counter()
    max_tau = np.zeros(14)
    max_rate = np.zeros(14)
    previous_tau = None
    max_qdd = 0.0
    max_velocity = 0.0
    before_release = first_open = final = binding = None
    initial = []
    ff_after_release = []
    for_line = open_trace(run)
    with for_line as stream:
        for line in stream:
            frame = json.loads(line)
            if frame.get('event') == 'TASK28_SUPPORT_BINDING':
                binding = frame
            if frame.get('event') != 'TORQUE_FRAME':
                continue
            qp = frame['qp']; cube = frame['cube_runtime']
            pose = cube['pose_world']
            # Legacy field says xyzw but the SingleRigidPrim API stores wxyz.
            rotation = Rotation.from_quat(np.asarray(pose['orientation_xyzw'])[[1, 2, 3, 0]])
            gap = pose['position_m'][2] - .06 * sum(abs(rotation.as_matrix()[2])) - .2
            closed = (frame.get('reaction_diagnostic') or {}).get('suctions_closed', {})
            obj = qp.get('object_reference') or {}
            tau = np.r_[frame['canonical_command_effort_Nm']['left'], frame['canonical_command_effort_Nm']['right']]
            max_tau = np.maximum(max_tau, abs(tau))
            if previous_tau is not None:
                max_rate = np.maximum(max_rate, abs(tau - previous_tau) * 60.0)
            previous_tau = tau
            if qp.get('qp_qdd14') is not None:
                max_qdd = max(max_qdd, float(np.max(abs(np.asarray(qp['qp_qdd14'])))))
            max_velocity = max(max_velocity, float(np.max(abs(np.r_[frame['dq_rad_s']['left'], frame['dq_rad_s']['right']]))))
            statuses[qp['status']] += 1
            row = dict(time_s=frame['time_s'], cube_xyz_m=pose['position_m'],
                       cube_quaternion_wxyz=pose['orientation_xyzw'],
                       cube_velocity_m_s=cube['linear_velocity_m_s'],
                       bottom_gap_m=gap, table_force_N=cube.get('table_contact_force_world_N'),
                       support_evidence=cube.get('table_support_evidence', 'force_unvalidated'),
                       grippers_closed=closed, payload=qp.get('payload_gravity_compensation'),
                       g3_load_model=qp.get('g3_qp_contact_load_model'),
                       torque_command_Nm=tau.tolist(), position_arm_write_count=frame['position_arm_write_count'])
            if frame['time_s'] < 1.0:
                initial.append(row)
            if obj.get('active') and all(closed.get(s) for s in ('left', 'right')):
                row.update(desired_cube_pose=obj['T_WO_des'], actual_cube_pose=obj['T_WO_actual'],
                           desired_tcp_poses=obj['tcp_desired_world'], actual_tcp_poses=frame['tcp_pose_world'],
                           reference_twist=obj['tcp_twist_desired_world'], reference_acceleration=obj['tcp_acceleration_desired_world'],
                           desired_closure_mm=1000 * float(np.linalg.norm(obj['desired_closure_error'][:3])),
                           actual_closure_mm=1000 * float(np.linalg.norm(obj['actual_closure_error'][:3])),
                           actual_closure_rotation_rad=float(np.linalg.norm(obj['actual_closure_error'][3:])),
                           cube_tracking_mm=1000 * float(np.linalg.norm(np.asarray(obj['T_WO_des'])[:3, 3] - pose['position_m'])),
                           cube_rotation_error_rad=float(np.linalg.norm(Rotation.from_matrix(
                               np.asarray(obj['T_WO_des'])[:3, :3] @ rotation.as_matrix().T).as_rotvec())),
                           common_accel=qp['common_accel_desired'], common_residual=qp['common_acceleration_residual'],
                           relative_residual=qp['relative_acceleration_residual'],
                           weights={k: qp[k] for k in ('common_weight', 'relative_weight', 'joint_posture_weight', 'internal_torque_weight')},
                           q=frame['q_rad'], dq=frame['dq_rad_s'], qref=frame['q_ref_rad'], dqref=frame['dq_ref_rad_s'],
                           internal_force=qp.get('g3_internal_force'),
                           torque_target_residual_Nm=qp.get('internal_torque_target_residual_inf_Nm'))
                loaded.append(row); terminal.append(row); before_release = row
                last_loaded_frame = frame
            elif before_release is not None and not any(closed.values()):
                if first_open is None:
                    first_open = row
                ff_after_release.append(bool((qp.get('payload_gravity_compensation') or {}).get('active')))
                ff_after_release.append(qp.get('g3_qp_contact_load_model') == 'assumed_commanded_wrench_not_measured')
            if frame['physics_step'] % 6 == 0:
                sampled.append(row)
            final = row
    if not loaded or final is None:
        raise ValueError('NO_LOADED_EVIDENCE')
    summary = json.loads((run / 'summary.json').read_text())
    log = (run / 'task27_executor.log').read_text()
    result = dict(run_id=run.name, support_binding=binding,
                  desired_closure_peak_mm=max(r['desired_closure_mm'] for r in loaded),
                  actual_closure_peak_mm=max(r['actual_closure_mm'] for r in loaded),
                  actual_closure_rotation_peak_rad=max(r['actual_closure_rotation_rad'] for r in loaded),
                  cube_tracking_peak_mm=max(r['cube_tracking_mm'] for r in loaded),
                  terminal_mean_cube_tracking_mm=float(np.mean([r['cube_tracking_mm'] for r in terminal])),
                  terminal_mean_common_accel_residual=float(np.mean([max(abs(np.array(r['common_residual']))) for r in terminal])),
                  final_xyz_error_mm=(1000 * (np.asarray(final['cube_xyz_m']) - [.77, .06, .26])).tolist(),
                  release_to_settled_height_mm=None if first_open is None else 1000 * (before_release['cube_xyz_m'][2] - final['cube_xyz_m'][2]),
                  final=final, pre_release=before_release, first_both_open=first_open,
                  initial_supported_force_max_N=max(max(abs(np.array(r['table_force_N'] or [0]))) for r in initial),
                  initial_geometry_gap_max_abs_m=max(abs(r['bottom_gap_m']) for r in initial),
                  qp_status_counts=dict(statuses), applied_torque_peak_per_joint_Nm=max_tau.tolist(),
                  applied_torque_rate_peak_per_joint_Nm_s=max_rate.tolist(),
                  qp_joint_acceleration_peak_rad_s2=max_qdd,
                  torque_limits_Nm=[87, 87, 87, 87, 12, 12, 12] * 2,
                  measured_joint_velocity_peak_rad_s=max_velocity,
                  load_feedforward_disabled_after_release=bool(first_open is not None and not any(ff_after_release)),
                  executor_exit=summary['executor_exit'], executor_pass='STAGE_A_ONLY PASS' in log)
    result['terminal_objective_replay'] = replay_weights(last_loaded_frame)
    result['physical_cycle_complete'] = bool(
        result['executor_pass'] and summary['executor_exit'] == 0 and first_open is not None
        and not any(final['grippers_closed'].values()) and abs(final['bottom_gap_m']) <= .0015
        and np.linalg.norm(final['cube_velocity_m_s']) <= .002
        and result['load_feedforward_disabled_after_release']
        and np.all(max_tau < result['torque_limits_Nm'])
        and summary['qp_status_counts'] == {'QP_OPTIMAL': summary['qp_solve_count']})
    (run / 'placement_release_metrics.json').write_text(json.dumps(result, indent=2) + '\n')
    with (run / 'terminal_3s.jsonl').open('w') as stream:
        for row in terminal:
            stream.write(json.dumps(row) + '\n')
    with (run / 'placement_release_series.jsonl').open('w') as stream:
        keys = ('time_s', 'cube_xyz_m', 'bottom_gap_m', 'table_force_N', 'grippers_closed',
                'cube_tracking_mm', 'actual_closure_mm', 'cube_rotation_error_rad', 'desired_cube_pose')
        for row in sampled:
            stream.write(json.dumps({k: row[k] for k in keys if k in row}) + '\n')
    return result, sampled


def replay_weights(frame):
    """Hold one archived plant sample fixed while changing only soft weights."""
    from offline.task28_g1_1.task28_validator.model import RobotModel
    from offline.task28_p5_qp_6_r12.qp_math import pair_tcp_geometric_relative_matrix
    from offline.task28_task27_live_qp.torque_qp import CooperativeTorqueQPConfig, solve_cooperative_torque_qp
    model = RobotModel(); qp = frame['qp']
    q = np.r_[frame['q_rad']['left'], frame['q_rad']['right']]
    dq = np.r_[frame['dq_rad_s']['left'], frame['dq_rad_s']['right']]
    jl = model.jacobian('left', q[:7]); jr = model.jacobian('right', q[7:])
    relative = pair_tcp_geometric_relative_matrix(model.fk('left', q[:7]), model.fk('right', q[7:]))
    jrel = relative @ np.block([[jl, np.zeros((6, 7))], [np.zeros((6, 7)), jr]])
    mass = np.zeros((14, 14))
    for index, side in enumerate(('left', 'right')):
        mass[index*7:(index+1)*7, index*7:(index+1)*7] = np.asarray(qp['full_mass_matrix_used'][side])[:7, :7]
    result = {}
    for label, posture, internal in (('v6_weights', 1e4, 1000.), ('terminal_weights', 100., 100.)):
        config = CooperativeTorqueQPConfig(torque_abs_Nm=(87., 87., 87., 87., 12., 12., 12.),
            common_weight=3e4, relative_weight=1e4, joint_posture_weight=posture, internal_torque_weight=internal)
        solved = solve_cooperative_torque_qp(q14=q, dq14=dq, mass14=mass,
            bias14=qp['internal_torque_target14_Nm'], jacobian_left=jl, jacobian_right=jr,
            jacobian_relative=jrel, jdotdq_left=qp['jdotdq_left'], jdotdq_right=qp['jdotdq_right'],
            jdotdq_relative=qp['jdotdq_relative'], common_accel_desired=qp['common_accel_desired'],
            relative_accel_desired=qp['relative_accel_desired'], joint_accel_desired=qp['joint_accel_desired'],
            previous_tau14=np.r_[frame['canonical_command_effort_Nm']['left'], frame['canonical_command_effort_Nm']['right']],
            config=config, model_validated=True, torque_target14=qp['internal_torque_target14_Nm'])
        result[label] = {k: solved.diagnostics.get(k) for k in (
            'status', 'reason', 'common_acceleration_residual_inf', 'relative_acceleration_residual_inf')}
    return result


def plot(run, data, before_data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), layout='constrained')
    for rows, label in ((before_data, 'v6'), (data, run.name)):
        loaded = [r for r in rows if 'desired_cube_pose' in r]
        if not loaded:
            continue
        t = [r['time_s'] - loaded[0]['time_s'] for r in loaded]
        for ax, key in zip(axes[:3], ('cube_tracking_mm', 'actual_closure_mm', 'cube_rotation_error_rad')):
            ax.plot(t, [r[key] for r in loaded], label=label)
        end_attached = loaded[-1]['time_s']
        tail = [r for r in rows if end_attached - 10 <= r['time_s'] <= end_attached + 2]
        axes[3].plot([r['time_s'] - end_attached for r in tail],
                     [1000 * r['bottom_gap_m'] for r in tail], label=label)
    for ax, title in zip(axes, ('Cube tracking (mm)', 'Actual bilateral closure (mm)',
                               'Object orientation tracking (rad)', 'Bottom-to-table gap near release / failed stop (mm)')):
        ax.set_title(title); ax.set_xlabel('Simulation time from attach (s)'); ax.grid(True); ax.legend()
    axes[3].set_xlabel('Time from last attached sample (s)')
    fig.savefig(run / 'placement_release_comparison.png', dpi=150)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--before', type=Path)
    parser.add_argument('--require-complete', action='store_true')
    args = parser.parse_args()
    result, data = analyze(args.run)
    if args.before:
        _, before_data = analyze(args.before)
        plot(args.run, data, before_data)
    print(json.dumps({k: v for k, v in result.items() if k not in ('final', 'pre_release', 'first_both_open')}, indent=2))
    if args.require_complete:
        assert result['physical_cycle_complete'], 'STAGE_A_PHYSICAL_CYCLE_INCOMPLETE'


if __name__ == '__main__':
    main()
