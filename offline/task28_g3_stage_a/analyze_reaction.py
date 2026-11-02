"""Recompute diagnostic window statistics from a real Stage-A reaction trace."""
import argparse
import gzip
import lzma
import json
from pathlib import Path
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation


def joint_world_wrench(meta, sample, joint_name):
    """Transmission wrench at the child joint anchor; no contact attribution."""
    row = meta['joint_rows'][joint_name]
    joint = meta['usd_joints'][joint_name]
    child_name = joint['body1'][0].split('/')[-1]
    child_row = meta['body_names_by_row'].index(child_name)
    if row != child_row:
        raise ValueError('JOINT_CHILD_ROW_MISMATCH')
    pose = np.asarray(sample['get_link_transforms'])[0, child_row]
    Rc = Rotation.from_quat(pose[3:7]).as_matrix()
    Rlocal = Rotation.from_quat(np.asarray(joint['local_rot1_wxyz'])[[1, 2, 3, 0]]).as_matrix()
    Rjoint = Rc @ Rlocal
    wrench = np.asarray(sample['incoming_child_joint_N_Nm'])[row]
    return np.r_[Rjoint @ wrench[:3], Rjoint @ wrench[3:]], pose[:3] + Rc @ joint['local_pos1_m']


def analyze(run_dir):
    path = run_dir / 'task27_torque_bridge.jsonl'
    opener = open
    if not path.exists():
        path = path.with_suffix('.jsonl.gz'); opener = gzip.open
    if not path.exists():
        path = run_dir / 'task27_torque_bridge.jsonl.xz'; opener = lzma.open
    meta = None; frames = []; events = []; status = Counter(); last = None
    with opener(path, 'rt') as stream:
        for line in stream:
            d = json.loads(line); event = d.get('event')
            if event == 'REACTION_SOURCE_METADATA': meta = d['sides']
            if event.startswith('G3_FIXED_FORCE_SEGMENT_'):
                events.append(dict(d, time_s=d.get('time_s', last)))
            if event != 'TORQUE_FRAME': continue
            last = d['time_s']; q = d.get('qp') or {}; status[q.get('status')] += 1
            diag = d.get('reaction_diagnostic')
            if not diag or not all(v.get('valid') for v in diag['sides'].values()): continue
            g = q.get('g3_internal_force') or {}
            f = {'time': last, 'step': d['physics_step'], 'target': g.get('target_N', 0),
                 'estimate': g.get('estimated_internal_force_N'), 'effective': g.get('effective_target_N'),
                 'q': np.r_[d['q_rad']['left'], d['q_rad']['right']],
                 'dq': np.r_[d['dq_rad_s']['left'], d['dq_rad_s']['right']],
                 'effort': np.r_[diag['applied_effort_completed_interval_Nm']['left'], diag['applied_effort_completed_interval_Nm']['right']],
                 'cube': d['cube_runtime']['pose_world']['position_m'],
                 'relative_mm': float(np.linalg.norm(q.get('relative_tcp_error', [0]*6)[:3])*1000),
                 'closed': diag['suctions_closed'], 'reactions': {},
                 'internal_target_residual_Nm': q.get('internal_torque_target_residual_inf_Nm'),
                 'active_torque': q.get('torque_saturated_indices'), 'active_rate': q.get('torque_rate_active_indices'),
                 'qdd_error': d.get('qdd_tracking_error14_rad_s2')}
            for side in ('left', 'right'):
                m = meta[side]; sample = diag['sides'][side]
                f['reactions'][side] = {}
                for name in ('fr3_joint7', 'fr3_joint8'):
                    if name in m['joint_rows']:
                        wrench, anchor = joint_world_wrench(m, sample, name)
                        f['reactions'][side][name] = wrench.tolist()
                f['reactions'][side]['local_all'] = sample['incoming_child_joint_N_Nm']
            # Recompute using arm-only model bias and the completed-interval effort.
            if d.get('measured_qdd14_rad_s2') is not None and diag.get('jacobian_world_at_tcp'):
                J = np.zeros((14,12)); M = np.zeros((14,14))
                for si, side in enumerate(('left','right')):
                    J[7*si:7*(si+1),6*si:6*(si+1)] = np.asarray(diag['jacobian_world_at_tcp'][side]).T
                    M[7*si:7*(si+1),7*si:7*(si+1)] = np.asarray(q['full_mass_matrix_used'][side])[:7,:7]
                bias=np.r_[np.asarray(q['full_coriolis_Nm']['left'])[:7]+np.asarray(q['full_gravity_Nm']['left'])[:7],
                           np.asarray(q['full_coriolis_Nm']['right'])[:7]+np.asarray(q['full_gravity_Nm']['right'])[:7]]
                r=M@d['measured_qdd14_rad_s2']+bias-f['effort']
                contacts=np.linalg.lstsq(J,-r,rcond=1e-8)[0]
                axis=np.asarray(g.get('grasp_axis_world',[0,1,0]))
                f['arm_only_estimate_N']=float(.5*(axis@contacts[:3]-axis@contacts[6:9]))
                r_new=M@d['measured_qdd14_rad_s2']+bias-np.r_[d['canonical_command_effort_Nm']['left'],d['canonical_command_effort_Nm']['right']]
                cn=np.linalg.lstsq(J,-r_new,rcond=1e-8)[0]
                f['new_vs_completed_effort_estimate_delta_N']=float(.5*(axis@(cn[:3]-contacts[:3])-axis@(cn[6:9]-contacts[6:9])))
            for side in ('left', 'right'):
                sample=diag['sides'][side]; m=meta[side]
                # Entire subtree beyond joint8: includes hand, fingers and hand_tcp.
                distal=[i for i,name in enumerate(m['body_names_by_row']) if name in ('fr3_link8','fr3_hand','fr3_leftfinger','fr3_rightfinger','fr3_hand_tcp')]
                mass=np.asarray(m['link_inertial_runtime']['get_masses'])[0]
                acceleration=np.asarray(sample['get_link_accelerations'])[0]
                velocity=np.asarray(sample['get_link_velocities'])[0]
                com=np.asarray(m['link_inertial_runtime']['get_coms'])[0]
                f['reactions'][side]['subtree_mass_kg']=float(mass[distal].sum())
                # Cover both COM-origin and actor-origin linear acceleration interpretations.
                f['reactions'][side]['horizontal_inertia_bound_N']=float(sum(mass[i]*(
                    np.linalg.norm(acceleration[i,:3]) + np.linalg.norm(acceleration[i,3:])*np.linalg.norm(com[i,:3])
                    + np.linalg.norm(velocity[i,3:])**2*np.linalg.norm(com[i,:3])) for i in distal))
            frames.append(f)
    result = {'run_id': run_dir.name, 'classification': 'G3_JOINT_REACTION_CANDIDATE',
              'qp_status': dict(status), 'reaction_valid_frames': len(frames),
              'metadata': meta, 'segments': [], 'new_reaction_used_by_controller': False}
    unloaded = [f for f in frames if 1.0 <= f['time'] <= 3.0 and not any(f['closed'].values())]
    result['startup_unloaded'] = {}
    for side in ('left', 'right'):
        if unloaded:
            result['startup_unloaded'][side] = {
                'joint8_force_world_mean_N': np.mean([f['reactions'][side]['fr3_joint8'][:3] for f in unloaded], axis=0).tolist(),
                'distal_mass_kg': unloaded[0]['reactions'][side]['subtree_mass_kg'],
                'gravity_magnitude_N': 9.81 * unloaded[0]['reactions'][side]['subtree_mass_kg'],
            }
    starts = [x for x in events if x['event'].endswith('_START')]
    ends = [x for x in events if x['event'].endswith('_COMPLETE')]
    scalar_keys = ('target', 'estimate', 'effective', 'relative_mm', 'internal_target_residual_Nm', 'arm_only_estimate_N', 'new_vs_completed_effort_estimate_delta_N')
    for start, end in zip(starts, ends):
        lo = start['time_s'] + start['transition_s'] + 1.0
        hi = end['time_s'] - 0.25
        window = [f for f in frames if lo <= f['time'] <= hi]
        if not window: raise ValueError('EMPTY_STEADY_WINDOW')
        s = {'index': start['segment_index'], 'target_N': start['target_N'],
             'steady_time_s': [window[0]['time'], window[-1]['time']], 'count': len(window)}
        for key in scalar_keys:
            a = np.array([f[key] for f in window if f.get(key) is not None])
            s[key] = {'mean': float(a.mean()), 'min': float(a.min()), 'max': float(a.max())} if len(a) else None
        for key in ('q', 'dq', 'effort', 'cube'):
            a = np.array([f[key] for f in window]); s[key] = {'mean': a.mean(axis=0).tolist(), 'min': a.min(axis=0).tolist(), 'max': a.max(axis=0).tolist()}
        s['transmission_world_at_joint_N_Nm'] = {}
        for side in ('left', 'right'):
            s['transmission_world_at_joint_N_Nm'][side] = {}
            for joint in ('fr3_joint7', 'fr3_joint8'):
                if joint not in window[0]['reactions'][side]: continue
                a = np.array([f['reactions'][side][joint] for f in window])
                s['transmission_world_at_joint_N_Nm'][side][joint] = {'mean': a.mean(axis=0).tolist(), 'std': a.std(axis=0).tolist()}
        s['subtree_dynamics'] = {side: {
            'mass_kg': window[0]['reactions'][side]['subtree_mass_kg'],
            'max_inertial_force_bound_N': max(f['reactions'][side]['horizontal_inertia_bound_N'] for f in window)
        } for side in ('left', 'right')}
        s['all_suctions_closed'] = all(all(f['closed'].values()) for f in window)
        result['segments'].append(s)
    fixed = [f for f in frames if starts and starts[0]['time_s'] <= f['time'] <= ends[-1]['time_s']]
    if fixed:
        poses = np.array([f['cube'] for f in fixed]); q0=np.array(fixed[0]['q'])
        result['fixed_cube_peak_drift_mm'] = float(np.linalg.norm(poses-poses[0],axis=1).max()*1000)
        result['fixed_relative_peak_mm'] = max(f['relative_mm'] for f in fixed)
        result['fixed_q_max_change_rad'] = float(max(np.abs(f['q']-q0).max() for f in fixed))
        result['fixed_torque_saturation_frames'] = sum(bool(f['active_torque']) for f in fixed)
        result['fixed_torque_rate_active_frames'] = sum(bool(f['active_rate']) for f in fixed)
    (run_dir/'reaction_metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
    for key in ('target', 'estimate', 'effective'):
        axes[0].plot([f['time'] for f in fixed], [f[key] for f in fixed], label=key+' (N)')
    axes[0].legend(); axes[0].set_ylabel('G3 target/candidate N')
    for side in ('left', 'right'):
        axes[1].plot([f['time'] for f in fixed], [f['reactions'][side]['fr3_joint8'][1] for f in fixed], label=side+' joint8 world Fy')
        axes[2].plot([f['time'] for f in fixed], [f['reactions'][side]['fr3_joint8'][2] for f in fixed], label=side+' joint8 world Fz')
    for ax in axes[1:3]: ax.legend(); ax.set_ylabel('Transmission N')
    axes[3].plot([f['time'] for f in fixed], [f['effort'][1] for f in fixed], label='left J2')
    axes[3].plot([f['time'] for f in fixed], [f['effort'][8] for f in fixed], label='right J2')
    axes[3].legend(); axes[3].set_ylabel('Applied effort Nm')
    for ax in axes: ax.grid(alpha=.25)
    axes[-1].set_xlabel('Isaac physics time (s)')
    fig.suptitle('Joint transmission is a candidate signal, not D6 contact force')
    fig.savefig(run_dir/'reaction_comparison.png', dpi=150)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('run_dir', type=Path)
    r=analyze(parser.parse_args().run_dir)
    print(json.dumps({k:v for k,v in r.items() if k not in ('metadata','segments')}, indent=2))
    for s in r['segments']:
        print(s['index'],s['target_N'],s['estimate'],s['transmission_world_at_joint_N_Nm'])
