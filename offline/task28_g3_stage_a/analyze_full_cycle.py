"""Streaming receipts for matched probes and the physical Stage-A-only cycle."""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import tempfile

import numpy as np
from scipy.spatial.transform import Rotation

from offline.task28_g3_stage_a.analyze_reaction import joint_world_wrench
from offline.task28_g3_stage_a.analyze_suspended_probe import analyze, open_trace
from offline.task28_g3_stage_a.suspended_probe import END_S


def matched(run):
    """Reuse the probe analyzer, separating cases without duplicating archived raw traces."""
    resets, metadata, motions = {}, [], {}
    with tempfile.TemporaryDirectory() as tmp:
        outputs = []
        end_time = None
        with open_trace(run) as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('event') == 'REACTION_SOURCE_METADATA': metadata.append(line)
                if row.get('event') == 'MATCHED_PROBE_RESET': resets[row['case']] = row
                if row.get('event') == 'SUSPENDED_PROBE_MOTION_START':
                    motions['off' if len(outputs) == 1 else 'on'] = row['ik']
                if row.get('event') == 'SUSPENDED_PROBE_START':
                    if outputs: outputs[-1].close()
                    path = Path(tmp) / ('off' if not outputs else 'on')
                    path.mkdir()
                    outputs.append((path / 'task27_torque_bridge.jsonl').open('w'))
                    outputs[-1].writelines(metadata)
                    end_time = row['time_s'] + END_S
                if outputs and not outputs[-1].closed:
                    outputs[-1].write(line)
                    if row.get('event') == 'TORQUE_FRAME' and row['time_s'] >= end_time - 1e-8:
                        outputs[-1].close()
        for output in outputs: output.close()
        result = {'cases': {}, 'reset_states': resets}
        if len(outputs) != 2 or set(resets) != {'off', 'on'}:
            raise RuntimeError('MATCHED_CASES_INCOMPLETE')
        for case in ('off', 'on'):
            result['cases'][case] = analyze(Path(tmp) / case)
            result['cases'][case]['run_id'] = run.name + '/' + case
        result['shared_q_reference_identical'] = resets['off']['reference_q_rad'] == resets['on']['reference_q_rad']
        result['motion_joint_references_identical'] = all(motions['off'][s][k] == motions['on'][s][k]
            for s in ('left','right') for k in ('q_start','q_end'))
        result['reset_q_max_difference_rad'] = max(float(np.max(np.abs(np.asarray(resets['off']['q_rad'][s]) - resets['on']['q_rad'][s]))) for s in ('left','right'))
        result['reset_cube_difference_mm'] = float(np.linalg.norm(np.asarray(resets['off']['cube_world_m']) - resets['on']['cube_world_m'])*1000)
        from offline.task28_g3_stage_a.plot_suspended_probe import plot
        plot(Path(tmp)/'off', Path(tmp)/'on', run/'matched_comparison.png')
    (run / 'matched_metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    return result


def distribution(values):
    if not values: return None
    a = np.asarray(values, float)
    return {'count': len(a), 'mean': np.mean(a, axis=0).tolist(), 'min': np.min(a, axis=0).tolist(),
            'max': np.max(a, axis=0).tolist(), 'p95': np.percentile(a, 95, axis=0).tolist()}


def cycle(run):
    metadata = None
    status, modes = Counter(), Counter()
    data = defaultdict(list)
    first = last = placed = None
    grasp0 = None
    relative0 = None
    peak_frame = None
    peak_relative_mm = -1.0
    fail_events = []
    active = release_active = saturation = rate = reaction_valid = 0
    count = 0
    fields = ['time_s','mode','cube_x_m','cube_y_m','cube_z_m','left_closed','right_closed',
              'load_feedforward_active','internal_target_N','internal_candidate_N',
              'torque_target_residual_Nm','relative_tcp_error_mm','common_tcp_error_mm',
              'left_joint8_Fy_N','right_joint8_Fy_N']
    with (run / 'cycle_timeseries.csv').open('w') as output, open_trace(run) as stream:
        writer = csv.writer(output); writer.writerow(fields)
        for line in stream:
            row = json.loads(line)
            event = row.get('event')
            if event == 'REACTION_SOURCE_METADATA': metadata = row['sides']
            if event in ('FAIL_CLOSED','FAIL_STOP_REQUESTED','FAIL_STOP_UNAVAILABLE'): fail_events.append(row)
            if event != 'TORQUE_FRAME': continue
            count += 1
            qp = row.get('qp') or {}
            status[str(qp.get('status'))] += 1
            mode = (qp.get('trajectory_adapter') or {}).get('task_mode', qp.get('task_mode', 'UNKNOWN'))
            modes[mode] += 1
            diag = row.get('reaction_diagnostic') or {}
            closed = diag.get('suctions_closed') or {}
            force = qp.get('g3_internal_force') or {}
            ff = qp.get('g3_qp_contact_load_model') == 'assumed_commanded_wrench_not_measured'
            if ff: active += 1
            if mode != 'DUAL_ATTACHED' and ff: release_active += 1
            saturation += bool(qp.get('torque_saturated_indices'))
            rate += bool(qp.get('torque_rate_active_indices'))
            cube = (row.get('cube_runtime') or {}).get('pose_world')
            if not cube: continue
            last = {'time_s': row['time_s'], 'position_m': cube['position_m'],
                    'orientation_wxyz': cube.get('orientation_xyzw')}
            if first is None: first = last
            if grasp0 is not None and placed is None and not any(closed.values()): placed = last
            left, right = (np.asarray(row['tcp_pose_world'][s]) for s in ('left','right'))
            rel = None if qp.get('relative_tcp_error') is None else float(np.linalg.norm(qp['relative_tcp_error'][:3])*1000)
            common = None if qp.get('common_tcp_error') is None else float(np.linalg.norm(qp['common_tcp_error'][:3])*1000)
            reactions = {}
            if metadata and all((diag.get('sides',{}).get(s) or {}).get('valid') for s in ('left','right')):
                reaction_valid += 1
                for side in ('left','right'):
                    reactions[side] = joint_world_wrench(metadata[side], diag['sides'][side], 'fr3_joint8')[0]
            loaded = mode == 'DUAL_ATTACHED' and all(closed.get(s) is True for s in ('left','right'))
            if loaded:
                if rel is not None and rel > peak_relative_mm:
                    peak_relative_mm, peak_frame = rel, row
                relative = np.linalg.inv(left) @ right
                if relative0 is None: relative0 = relative.copy()
                data['loaded_actual_relative_translation_change_mm'].append(float(np.linalg.norm(relative[:3,3]-relative0[:3,3])*1000))
                data['loaded_actual_relative_rotation_change_rad'].append(float(np.linalg.norm(Rotation.from_matrix(relative0[:3,:3].T@relative[:3,:3]).as_rotvec())))
                cube_point = np.r_[cube['position_m'], 1]
                offsets = {s: (np.linalg.inv(T) @ cube_point)[:3] for s,T in (('left',left),('right',right))}
                if grasp0 is None: grasp0 = offsets
                for side in ('left','right'):
                    data['loaded_grasp_'+side+'_translation_change_mm'].append(float(np.linalg.norm(offsets[side] - grasp0[side])*1000))
                    if side in reactions: data['loaded_joint8_'+side+'_world_N_Nm'].append(reactions[side].tolist())
                for key,value in (('relative_tcp_error_mm',rel),('common_tcp_error_mm',common),
                    ('internal_candidate_N',force.get('estimated_internal_force_N')),
                    ('internal_target_N',force.get('target_N')),
                    ('torque_target_residual_Nm',qp.get('internal_torque_target_residual_inf_Nm'))):
                    if value is not None: data['loaded_'+key].append(value)
                efforts = diag.get('applied_effort_completed_interval_Nm')
                if efforts: data['loaded_applied_effort14_Nm'].append(efforts['left'] + efforts['right'])
                if ff: data['loaded_ff_active'].append(1)
                else: data['loaded_ff_active'].append(0)
            data['cube_xyz_m'].append(cube['position_m'])
            writer.writerow([row['time_s'],mode,*cube['position_m'],closed.get('left'),closed.get('right'),ff,
                force.get('target_N'),force.get('estimated_internal_force_N'),qp.get('internal_torque_target_residual_inf_Nm'),rel,common,
                reactions.get('left',[None]*6)[1],reactions.get('right',[None]*6)[1]])
    executor = (run/'task27_executor.log').read_text()
    summary = json.loads((run/'summary.json').read_text())
    result = {'classification':'G3_JOINT_REACTION_CANDIDATE','frames':count,'qp_status':dict(status),
        'modes':dict(modes),'ff_active_frames':active,'ff_outside_dual_attached_frames':release_active,
        'torque_saturation_frames':saturation,'torque_rate_active_frames':rate,'reaction_valid_frames':reaction_valid,
        'first_cube_pose':first,'first_open_after_attachment_cube_pose':placed,'final_cube_pose':last,
        'failure_events':fail_events,'statistics':{k:distribution(v) for k,v in data.items()},
        'stage_a_only_pass_receipt':'STAGE_A_ONLY PASS:' in executor,
        'executor_exit':summary.get('executor_exit'),'runtime_status':summary.get('status'),
        'limitations':['incoming joint8 transmission is not D6 contact wrench',
            'logged QP reference errors refer to the upcoming control interval',
            'grasp translation change is relative to first bilateral closed sample; not contact force or CCD']}
    result['physical_cycle_completed'] = (result['stage_a_only_pass_receipt'] and summary.get('executor_exit') == 0
        and summary.get('status') == 'COMPLETED' and active > 0 and release_active == 0 and not fail_events
        and placed is not None)
    if peak_frame is not None:
        from offline.task28_g1_1.task28_validator.model import RobotModel
        from offline.task28_p5_qp_6_r12.qp_math import se3_log
        model = RobotModel()
        actual = {s:np.asarray(peak_frame['tcp_pose_world'][s]) for s in ('left','right')}
        desired = {s:model.fk(s,peak_frame['q_ref_rad'][s]) for s in ('left','right')}
        ar = np.linalg.inv(actual['left'])@actual['right']
        dr = np.linalg.inv(desired['left'])@desired['right']
        receipt = {'physics_step':peak_frame['physics_step'],'time_s':peak_frame['time_s'],
            'relative_reference_error_translation_mm':peak_relative_mm,
            'fk_recomputed_error_translation_mm':float(np.linalg.norm(se3_log(np.linalg.inv(ar)@dr)[:3])*1000),
            'actual_tcp_world':{s:t.tolist() for s,t in actual.items()},
            'fk_qref_tcp_world':{s:t.tolist() for s,t in desired.items()},
            'actual_relative_transform':ar.tolist(),'fk_qref_relative_transform':dr.tolist(),
            'cube_pose_world':peak_frame['cube_runtime']['pose_world'],
            'trajectory_adapter':peak_frame['qp']['trajectory_adapter'],
            'limitation':'upcoming-interval reference comparison, not a grasp-slip measurement or root cause attribution'}
        (run/'reference_error_peak.json').write_text(json.dumps(receipt,indent=2)+'\n')
        result['relative_reference_peak'] = {k:receipt[k] for k in ('physics_step','time_s','relative_reference_error_translation_mm','fk_recomputed_error_translation_mm')}
    (run/'cycle_metrics.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


def plot_cycle(run):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with (run/'cycle_timeseries.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    def values(key):
        return [float(r[key]) if r[key] else np.nan for r in rows]
    time = values('time_s')
    fig, axes = plt.subplots(4,1,figsize=(11,9),sharex=True,layout='constrained')
    for key in ('cube_x_m','cube_y_m','cube_z_m'): axes[0].plot(time,values(key),label=key)
    axes[1].plot(time,values('relative_tcp_error_mm'),label='relative TCP reference error')
    for key in ('internal_target_N','internal_candidate_N','left_joint8_Fy_N','right_joint8_Fy_N'):
        axes[2].plot(time,values(key),label=key)
    for key in ('left_closed','right_closed','load_feedforward_active'):
        axes[3].plot(time,[int(r[key]=='True') for r in rows],label=key)
    for ax,unit in zip(axes,('Cube world position (m)','Translation error (mm)','Candidate / transmission (N)','Active / closed')):
        ax.set_ylabel(unit); ax.grid(alpha=.3); ax.legend(loc='best')
    axes[-1].set_xlabel('Isaac physics time (s)')
    fig.suptitle('Task27 Stage A: incoming joint transmission and model candidate are not D6 contact measurements')
    fig.savefig(run/'cycle_overview.png',dpi=150); plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('run_dir',type=Path)
    parser.add_argument('--matched',action='store_true')
    args=parser.parse_args()
    result=(matched if args.matched else cycle)(args.run_dir)
    if not args.matched: plot_cycle(args.run_dir)
    print(json.dumps({k:v for k,v in result.items() if k not in ('statistics','cases')},indent=2))
