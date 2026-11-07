"""Same-run observable history. Time intervals stay intervals; FK never estimates Cube pose."""
import bisect
import math
import numpy as np
from research.stage_b_multisensor_r3.time_alignment import RobotHistory

IDENTITY = ('run_id', 'epoch_id', 'time_base_id')


def identity(row):
    if any(row.get(k) is None for k in IDENTITY):
        raise ValueError('missing_time_base_identity')
    return tuple(row[k] for k in IDENTITY)


def reject_oracle(value):
    if isinstance(value, dict):
        for k, v in value.items():
            if 'oracle' in k.lower() or 'physx_xyz' in k.lower() or 'tcp_physx' in k.lower() or k.lower().endswith('_gt'):
                raise ValueError('oracle_not_an_observable_input')
            reject_oracle(v)
    elif isinstance(value, list):
        for v in value:
            reject_oracle(v)


class SynchronizedHistory:
    """Immutable offline epochs; sparse interpolation requires an explicit expected stride."""
    def __init__(self, rows, fk, cfg, expected_step_stride=1):
        self.epochs = {}; self.cfg = cfg; self.seen = {}; self.stride = expected_step_stride
        assert expected_step_stride in (1, 2)
        for row in rows:
            reject_oracle(row)
            allowed = set(IDENTITY) | {'time_s','physics_step','dt_s','core_sim_time_s','timeline_s','q','dq','tcp_fk','joint_state_source','acquisition_stage','validity','suction_observation','suction_command','controller_command','contact_observation','force_observation'}
            if set(row)-allowed:
                raise ValueError('unsupported_robot_input_field')
            key = identity(row)
            if row.get('joint_state_source') != 'Isaac4.5 SingleArticulation -> live PhysX articulation joint tensors' or not all(row.get('validity', {}).get(k) is True for k in ('finite_q_dq', 'articulation_handles_initialized')):
                raise ValueError('invalid_measured_joint_source')
            if not math.isfinite(row['core_sim_time_s']) or abs(row['core_sim_time_s']-row['time_s']) > 1e-6:
                raise ValueError('inconsistent_robot_clock')
            if key not in self.epochs:
                self.epochs[key] = RobotHistory(fk, max_samples=cfg['history_max_samples'], max_bracket_s=cfg['max_interpolation_bracket_s'])
            history = self.epochs[key]
            if history.rows and row['physics_step'] <= history.rows[-1]['physics_step']:
                raise ValueError('duplicate_step_or_unmarked_reset')
            history.append(**{k: row[k] for k in ('time_s', 'physics_step', 'q', 'dq', 'suction_observation', 'suction_command', 'controller_command')})

    def at(self, clock_identity, time_s):
        try:
            key = identity(clock_identity)
        except ValueError as exc:
            return {'status': 'UNKNOWN', 'reason': str(exc)}
        history = self.epochs.get(key)
        if history is None:
            return {'status': 'UNKNOWN', 'reason': 'incompatible_run_epoch_or_clock'}
        if not isinstance(time_s, (float, int)) or not math.isfinite(time_s):
            return {'status': 'UNKNOWN', 'reason': 'invalid_candidate_time'}
        rows = list(history.rows); times = [r['time_s'] for r in rows]
        if times[0] <= time_s <= times[-1]:
            i = bisect.bisect_left(times, time_s)
            if i and abs(times[i]-time_s) > 1e-9 and rows[i]['physics_step']-rows[i-1]['physics_step'] != self.stride:
                return {'status': 'UNKNOWN', 'reason': 'missing_joint_sample'}
            if i and abs(times[i]-time_s) > 1e-9 and rows[i]['controller_command'] != rows[i-1]['controller_command']:
                return {'status': 'UNKNOWN', 'reason': 'command_transition_interpolation_unsupported'}
        result = history.at(time_s)
        if result['status'] == 'ALIGNED':
            result.update({k: clock_identity[k] for k in IDENTITY})
            result['tcp_convention'] = 'world_from_TCP, column vectors; translation m, rotation matrix; q rad, dq rad/s'
            result['value_source'] = 'MEASURED_Q_DQ_AND_RECOMPUTED_FK' if result['bracket_steps'][0] == result['bracket_steps'][1] else 'LINEAR_Q_DQ_AND_RECOMPUTED_FK'
            result['accuracy_status'] = 'EXACT_HISTORY_SAMPLE' if result['bracket_steps'][0] == result['bracket_steps'][1] else 'INTERPOLATION_ACCURACY_NOT_GUARANTEED'
        return result

    def associate(self, frame, physics_times, epoch_start_s=0.):
        """Return full causal interval and normalized grid references, never a point time.

        Grid references do not discretize away the interval. Call at() for any real
        candidate inside it; only adjacent valid samples support interpolation.
        """
        base = {k: frame.get(k) for k in IDENTITY}
        base.update(frame_id=frame.get('frame_id'), measurement_time_s=None,
                    timing_status='UNCERTAIN_CAUSAL_INTERVAL', online_fusion_eligible=False)
        def unknown(reason):
            return dict(base, status='UNKNOWN', reason=reason)
        try:
            key = identity(frame)
        except ValueError as exc:
            return unknown(str(exc))
        if key not in self.epochs:
            return unknown('incompatible_run_epoch_or_clock')
        if frame.get('callback_entry_step') not in physics_times:
            return unknown('missing_callback_physics_time')
        upper = physics_times[frame['callback_entry_step']]
        if not math.isfinite(upper) or not math.isfinite(epoch_start_s) or upper < epoch_start_s:
            return unknown('invalid_epoch_interval')
        sdk, fabric = frame.get('sdk_time_s'), frame.get('fabric_sim_time_s')
        if any(not isinstance(t, (int, float)) or not math.isfinite(t) for t in (sdk, fabric, frame.get('arrival_physics_time_s'), frame.get('callback_core_time_s'))):
            return unknown('missing_or_invalid_frame_clock')
        if abs(sdk-fabric) > 1e-6 or abs(frame['callback_core_time_s']-upper) > 1e-6 or sdk < epoch_start_s-1e-6 or sdk > upper+1e-6:
            return unknown('inconsistent_frame_clock')
        age = frame['arrival_physics_time_s']-upper
        if age < -1e-6:
            return unknown('arrival_precedes_callback')
        if age > self.cfg['max_stale_arrival_s']:
            return unknown('stale_arrival')
        frame_key = key+(frame['frame_id'],)
        content = (frame['rgb_sha256'], frame['depth_sha256'], sdk)
        if frame_key in self.seen:
            return unknown('duplicate_frame' if self.seen[frame_key] == content else 'conflicting_duplicate_frame')
        self.seen[frame_key] = content
        history = list(self.epochs[key].rows)
        grid = [{'time_s': t, 'physics_step': s, 'robot_state_ref': s if any(r['physics_step'] == s for r in history) else None}
                for s, t in physics_times.items() if epoch_start_s <= t <= upper+1e-9]
        domains = [[a['time_s'], b['time_s']] for a, b in zip(history, history[1:])
                   if b['physics_step']-a['physics_step'] == self.stride and b['controller_command']==a['controller_command'] and b['time_s']-a['time_s'] <= self.cfg['max_interpolation_bracket_s'] and b['time_s'] <= upper+1e-9]
        supported = sum(c['robot_state_ref'] is not None for c in grid)
        return dict(base, status='UNCERTAIN',
                    timestamp_source='SAME_EPOCH_CAUSAL_BOUND_NO_SCENE_TRANSFER_ASSUMPTION',
                    admissible_interval_s=[epoch_start_s, upper], interval_endpoint_policy='closed',
                    physics_grid_candidates=grid, interpolation_domains_s=domains,
                    supported_grid_candidates=supported, unsupported_grid_candidates=len(grid)-supported,
                    unsupported_prefix_s=[epoch_start_s, history[0]['time_s']],
                    unsupported_prefix_endpoint_policy='right_open',
                    broad_ambiguity=upper-epoch_start_s > self.cfg['broad_candidate_span_s'],
                    callback_to_reader_sim_s=age,
                    measurement_age_interval_s=[age, frame['arrival_physics_time_s']-epoch_start_s])
