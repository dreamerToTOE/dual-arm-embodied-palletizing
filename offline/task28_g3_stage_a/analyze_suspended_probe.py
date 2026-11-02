"""Analyze the Stage-A suspended-Cube probe without treating reactions as contact force."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import gzip
import json
import lzma
from pathlib import Path

import numpy as np
from offline.task28_g3_stage_a.analyze_reaction import joint_world_wrench
from offline.task28_g3_stage_a.suspended_probe import END_S, MOTION_START_S, MOVE_S, HOLD_S, motion_progress


def paired_commands(rows):
    """Pair command at state k with the immediately following completed state k+1."""
    pending = None
    for row in rows:
        event = row.get('event')
        if event == 'SUSPENDED_PROBE_COMMAND':
            pending = row
        elif event == 'TORQUE_FRAME' and pending is not None:
            if row.get('physics_step') == pending.get('physics_step', -2) + 1:
                yield pending, row
            pending = None


def open_trace(run_dir):
    for suffix, opener in (('', open), ('.gz', gzip.open), ('.xz', lzma.open)):
        path = run_dir / ('task27_torque_bridge.jsonl' + suffix)
        if path.exists():
            return opener(path, 'rt')
    raise FileNotFoundError('task27_torque_bridge.jsonl[.gz|.xz]')


def phase(elapsed):
    t = elapsed - MOTION_START_S
    if t < 0: return 'disturbance_or_settle'
    if t < MOVE_S: return 'outbound'
    if t < MOVE_S + HOLD_S: return 'outbound_hold'
    if t < 2 * MOVE_S + HOLD_S: return 'return'
    return 'return_hold'


def stats(values):
    if not values: return None
    a = np.asarray(values, dtype=float)
    return {'count': len(a), 'mean': float(a.mean()), 'min': float(a.min()), 'max': float(a.max())}


def analyze(run_dir):
    run_dir = Path(run_dir)
    meta = None
    start = None
    motion = None
    pending = None
    pairs = []
    unmatched = 0
    stop_events = []
    with open_trace(run_dir) as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                stop_events.append({'event': 'INVALID_JSON', 'line': line_number})
                break
            event = row.get('event')
            if event == 'REACTION_SOURCE_METADATA': meta = row.get('sides')
            elif event == 'SUSPENDED_PROBE_START': start = row
            elif event == 'SUSPENDED_PROBE_MOTION_START': motion = row
            elif event in ('FAIL_STOP_REQUESTED', 'FAIL_STOP_UNAVAILABLE'):
                stop_events.append(row)
            elif event == 'SUSPENDED_PROBE_COMMAND':
                if pending is not None: unmatched += 1
                pending = row
            elif event == 'TORQUE_FRAME' and pending is not None:
                if row.get('physics_step') == pending.get('physics_step', -2) + 1:
                    pairs.append((pending, row))
                else:
                    unmatched += 1
                pending = None
    trailing_unpaired = pending is not None
    if trailing_unpaired: unmatched += 1
    result = {
        'run_id': run_dir.name, 'probe_started': start is not None,
        'motion_started': motion is not None, 'paired_frames': len(pairs),
        'unmatched_commands': unmatched, 'trailing_command_unpaired': trailing_unpaired, 'stop_events': stop_events,
        'alignment': 'force command physics_step k matched completed TORQUE_FRAME k+1; ideal target evaluated at frame.time_s',
        'reaction_semantics': 'joint8 incoming transmission wrench, not independent suction contact force',
        'model_candidate_semantics': 'model residual estimate, not measured suction force',
        'force_semantics': 'known force injected on Cube during interval k to k+1',
        'cube_orientation_used': False, 'motion': None, 'disturbance_windows': [],
    }
    elapsed_last = pairs[-1][0].get('elapsed_s') if pairs else None
    result['last_elapsed_s'] = elapsed_last
    step_gaps = sum(b[0]['physics_step'] != a[0]['physics_step'] + 1 for a, b in zip(pairs, pairs[1:]))
    result['paired_step_gaps'] = step_gaps
    result['complete_by_trace'] = bool(start and motion and pairs and pairs[0][0].get('elapsed_s', 1) <= 0.05
                                       and elapsed_last >= END_S - 0.05 and not stop_events
                                       and unmatched == int(trailing_unpaired) and not step_gaps)
    if not pairs:
        (run_dir / 'suspended_probe_metrics.json').write_text(json.dumps(result, indent=2) + '\n')
        return result
    qps, forces, phases = Counter(), defaultdict(list), defaultdict(list)
    model = None
    start_fk_inverse = {}
    if motion and start and motion.get('ik') and start.get('time_s') is not None:
        from offline.task28_g1_1.task28_validator.model import RobotModel
        model = RobotModel()
        for side in ('left', 'right'):
            start_fk_inverse[side] = np.linalg.inv(model.fk(side, motion['ik'][side]['q_start']))
    samples = []
    relative0 = None
    cube0 = None
    for command, frame in pairs:
        elapsed = float(command['elapsed_s'])
        qp = frame.get('qp') or {}
        diag = frame.get('reaction_diagnostic') or {}
        cube = ((frame.get('cube_runtime') or {}).get('pose_world') or {}).get('position_m')
        target = command.get('cube_target_world_m')
        if cube is None: continue
        cube = np.asarray(cube, float)
        if cube0 is None: cube0 = cube.copy()
        tcp = frame.get('tcp_pose_world') or {}
        rel_change_mm = None
        if all(side in tcp for side in ('left', 'right')):
            left, right = (np.asarray(tcp[s], float) for s in ('left', 'right'))
            rel = (np.linalg.inv(left) @ right)[:3, 3]
            if relative0 is None: relative0 = rel.copy()
            rel_change_mm = float(np.linalg.norm(rel - relative0) * 1000)
        g3 = qp.get('g3_internal_force') or {}
        adapter = qp.get('trajectory_adapter') or {}
        frame_time = frame.get('time_s')
        reference_age = adapter.get('reference_age') or {}
        ref_sources = {side: (reference_age.get(side) or {}).get('source') for side in ('left', 'right')}
        state_elapsed = float(frame_time - start['time_s']) if frame_time is not None and start and start.get('time_s') is not None else None
        ideal_target = None
        fk_target = None
        if motion and state_elapsed is not None and frame_time >= motion.get('time_s', float('inf')):
            initial = np.asarray(motion['cube_start_world_m'], float)
            amplitude = motion['amplitude_world_x_m']
            ideal_target = initial + np.array([amplitude * motion_progress(state_elapsed)[0], 0, 0])
            refs = frame.get('q_ref_rad') or adapter.get('q_ref_rad') or {}
            if (model is not None and all(side in refs for side in ('left', 'right'))
                    and all(ref_sources[side] == 'suspended_probe_quintic_joint_ik' for side in ('left', 'right'))):
                p0 = np.r_[initial, 1.0]
                fk_target = np.mean([(model.fk(side, refs[side]) @ start_fk_inverse[side] @ p0)[:3]
                                     for side in ('left', 'right')], axis=0)
        sample = {'elapsed_s': elapsed, 'step': frame.get('physics_step'),
                  'cube': cube, 'target': None if target is None else np.asarray(target, float),
                  'state_time_s': frame_time, 'state_elapsed_s': state_elapsed,
                  'ideal_target': ideal_target, 'fk_target': fk_target,
                  'tcp_targets_active': adapter.get('tcp_targets_active'), 'ref_sources': ref_sources,
                  'active_arms': adapter.get('active_arms'),
                  'relative_change_mm': rel_change_mm, 'closed': diag.get('suctions_closed'),
                  'relative_tracking_mm': (float(np.linalg.norm(qp['relative_tcp_error'][:3])*1000)
                                           if qp.get('relative_tcp_error') is not None else None),
                  'relative_tracking_rotation_rad': (float(np.linalg.norm(qp['relative_tcp_error'][3:]))
                                                     if qp.get('relative_tcp_error') is not None else None),
                  'estimate': g3.get('estimated_internal_force_N'),
                  'target_force': g3.get('target_N'),
                  'target_residual_Nm': qp.get('internal_torque_target_residual_inf_Nm'),
                  'qp_status': qp.get('status'),
                  'torque_saturation': bool(qp['torque_saturated_indices']) if 'torque_saturated_indices' in qp else None,
                  'rate_active': bool(qp['torque_rate_active_indices']) if 'torque_rate_active_indices' in qp else None, 'reactions': {}}
        qps[str(sample['qp_status'])] += 1
        if meta:
            for side in ('left', 'right'):
                try:
                    reaction = diag['sides'][side]
                    if reaction.get('valid'):
                        sample['reactions'][side] = joint_world_wrench(meta[side], reaction, 'fr3_joint8')[0].tolist()
                except (KeyError, IndexError, TypeError, ValueError):
                    pass
        samples.append(sample)
        phases[phase(elapsed)].append(sample)
        if 4 <= elapsed < 18:
            index = int((elapsed - 4) / 2)
            forces[index].append((command, sample))
    result['qp_status'] = dict(qps)
    for name, key in (('torque_saturation', 'torque_saturation'), ('torque_rate_active', 'rate_active')):
        observed = [s[key] for s in samples if s[key] is not None]
        result[name + '_frames'] = sum(observed) if observed else None
        result[name + '_observed_frames'] = len(observed)
    result['gripper_closed_samples'] = sum(isinstance(s['closed'], dict) and all(s['closed'].get(k) is True for k in ('left', 'right')) for s in samples)
    result['gripper_not_closed_or_unknown_samples'] = len(samples) - result['gripper_closed_samples']
    rels = [s['relative_change_mm'] for s in samples if s['relative_change_mm'] is not None]
    result['relative_tcp_translation_change_max_mm'] = max(rels) if rels else None
    tracking = [s['relative_tracking_mm'] for s in samples if s['relative_tracking_mm'] is not None]
    rotations = [s['relative_tracking_rotation_rad'] for s in samples if s['relative_tracking_rotation_rad'] is not None]
    result['relative_tcp_tracking_translation_max_mm'] = max(tracking) if tracking else None
    result['relative_tcp_tracking_rotation_max_rad'] = max(rotations) if rotations else None
    result['model_candidate_by_phase_N'] = {key: stats([s['estimate'] for s in group if s['estimate'] is not None]) for key, group in phases.items()}
    result['target_torque_residual_by_phase_Nm'] = {key: stats([s['target_residual_Nm'] for s in group if s['target_residual_Nm'] is not None]) for key, group in phases.items()}
    adapters = Counter((str(s['tcp_targets_active']), ','.join(s['active_arms']) if isinstance(s['active_arms'], list) else 'unknown') for s in samples)
    result['trajectory_adapter_states'] = [{'tcp_targets_active': None if active == 'None' else active == 'True',
                                             'active_arms': None if arms == 'unknown' else arms.split(',') if arms else [],
                                             'sample_count': count} for (active, arms), count in sorted(adapters.items())]
    motion_samples = [s for s in samples if s['ideal_target'] is not None]
    if motion_samples:
        initial = np.asarray(motion['cube_start_world_m'], float)
        ideal_errors = [float(np.linalg.norm(s['cube'] - s['ideal_target']) * 1000) for s in motion_samples]
        final = motion_samples[-1]
        fk_samples = [s for s in motion_samples if s['fk_target'] is not None]
        fk_errors = [float(np.linalg.norm(s['cube'] - s['fk_target']) * 1000) for s in fk_samples]
        deviations = [float(np.linalg.norm(s['fk_target'] - s['ideal_target']) * 1000) for s in fk_samples]
        active = [s for s in fk_samples if s['tcp_targets_active'] is True and set(s['active_arms'] or []) == {'left', 'right'}]
        result['motion'] = {'samples': len(motion_samples),
                            'target_semantics': 'ideal quintic straight line evaluated at each TORQUE_FRAME time_s',
                            'target_error_peak_mm': max(ideal_errors),
                            'target_error_rms_mm': float(np.sqrt(np.mean(np.square(ideal_errors)))),
                            'last_target_error_mm': ideal_errors[-1],
                            'last_cube_world_m': final['cube'].tolist(),
                            'last_target_world_m': final['ideal_target'].tolist(),
                            'last_error_world_mm': ((final['cube'] - final['ideal_target'])*1000).tolist(),
                            'return_error_mm': float(np.linalg.norm(final['cube'] - initial) * 1000) if phase(final['state_elapsed_s']) == 'return_hold' else None,
                            'max_displacement_from_motion_start_mm': float(max(np.linalg.norm(s['cube'] - initial) for s in motion_samples) * 1000),
                            'last_phase': phase(final['state_elapsed_s']),
                            'fk_qref_reference': {'semantics': 'average Cube position implied by each FK q_ref and initial rigid attachment; q_ref in state k+1 is the command for the following interval, not the command applied in interval k; not a direct Cube command or contact measurement',
                                                 'source_filter': 'both trajectory_adapter.reference_age sources must equal suspended_probe_quintic_joint_ik',
                                                 'samples': len(fk_samples),
                                                 'excluded_samples': len(motion_samples) - len(fk_samples),
                                                 'excluded_reference_sources': dict(Counter(str(s['ref_sources']) for s in motion_samples if s['fk_target'] is None)),
                                                 'bilateral_tcp_targets_active_samples': len(active),
                                                 'active_flag_limitation': 'adapter flag does not by itself prove no later hold override',
                                                 'tracking_error_peak_mm': max(fk_errors) if fk_errors else None,
                                                 'tracking_error_rms_mm': float(np.sqrt(np.mean(np.square(fk_errors)))) if fk_errors else None,
                                                 'fk_vs_ideal_peak_mm': max(deviations) if deviations else None,
                                                 'last_implied_cube_world_m': fk_samples[-1]['fk_target'].tolist() if fk_samples else None,
                                                 'active_tracking_error_peak_mm': max(float(np.linalg.norm(s['cube'] - s['fk_target'])*1000) for s in active) if active else None}}
    for index, group in sorted(forces.items()):
        # Late half of each constant-force interval; omitted if the run ended before it.
        lo = 4 + index * 2 + 1
        window = [(c, s) for c, s in group if s['elapsed_s'] >= lo]
        window_result = {'index': index, 'force_world_N': group[0][0].get('force_world_N'),
                         'sample_count': len(window), 'window_s': [lo, 4 + (index + 1) * 2],
                         'actual_elapsed_range_s': [window[0][1]['elapsed_s'], window[-1][1]['elapsed_s']] if window else None,
                         'cube_position_mean_world_m': np.mean([s['cube'] for _, s in window], axis=0).tolist() if window else None,
                         'cube_displacement_from_probe_start_mean_mm': ((np.mean([s['cube'] for _, s in window], axis=0) - cube0) * 1000).tolist() if window else None,
                         'model_candidate_N': stats([s['estimate'] for _, s in window if s['estimate'] is not None]),
                         'joint8_world_wrench_mean_N_Nm': {}}
        for side in ('left', 'right'):
            vals = [s['reactions'][side] for _, s in window if side in s['reactions']]
            window_result['joint8_world_wrench_mean_N_Nm'][side] = np.mean(vals, axis=0).tolist() if vals else None
        result['disturbance_windows'].append(window_result)
    (run_dir / 'suspended_probe_metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run_dir', type=Path)
    print(json.dumps(analyze(parser.parse_args().run_dir), indent=2))
