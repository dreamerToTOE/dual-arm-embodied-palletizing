"""Bounded Stage-A Cube disturbance and small bilateral translation probe."""
from __future__ import annotations
import numpy as np
from offline.task28_p5_qp_6_r12.qp_math import quintic_profile

FORCES_N = (0.0, 0.1, 0.2, 0.0, -0.1, -0.2, 0.0)
CASE_S = 2.0
MOTION_START_S = 20.0
MOVE_S = 5.0
HOLD_S = 2.0
END_S = MOTION_START_S + 2 * (MOVE_S + HOLD_S)


def motion_progress(elapsed):
    """C2 out-and-back; returns position, velocity and acceleration scales."""
    t = elapsed - MOTION_START_S
    if t < 0:
        return 0.0, 0.0, 0.0
    if t < MOVE_S:
        return quintic_profile(t, MOVE_S, 1.0)
    if t < MOVE_S + HOLD_S:
        return 1.0, 0.0, 0.0
    if t < 2 * MOVE_S + HOLD_S:
        s, ds, dd = quintic_profile(t - MOVE_S - HOLD_S, MOVE_S, 1.0)
        return 1.0 - s, -ds, -dd
    return 0.0, 0.0, 0.0


def run_suspended_probe(app, torque, stage, *, amplitude_m=0.005, timeout_s=300, matched_reference=None):
    import time
    import omni.physx
    from pxr import Gf, PhysicsSchemaTools, UsdUtils
    if not 0 < amplitude_m <= 0.01:
        raise ValueError('SUSPENDED_PROBE_AMPLITUDE')
    if torque._reaction_recorder is None or not torque._surface_grippers_closed():
        raise RuntimeError('SUSPENDED_PROBE_REQUIRES_DUAL_ATTACHMENT_AND_REACTIONS')
    if torque._attached_hold_active or any(torque._trajectory_adapters[s].point_count < 1
                                           for s in ('left', 'right')):
        raise RuntimeError('SUSPENDED_PROBE_TRAJECTORY_REFERENCE_NOT_ACTIVE')
    cube = torque.feed.rigids[0]
    cube_start, _ = cube.get_world_pose()
    cube_start = np.asarray(cube_start, float)
    if cube_start[2] < 0.40:
        raise RuntimeError('SUSPENDED_PROBE_CUBE_NOT_LIFTED')
    path = '/World/Task27/Supply/Cube_01'
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError('SUSPENDED_PROBE_CUBE_MISSING')
    sim = omni.physx.get_physx_simulation_interface()
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    encoded = PhysicsSchemaTools.sdfPathToInt(path)
    start = float(torque._time_s)
    old_step, old_ref = torque._torque_step, torque._trajectory_reference
    hold_q = {s: np.asarray(torque.articulations[s].get_joint_positions(), float)[:7].copy()
              for s in ('left', 'right')}
    if matched_reference is not None:
        hold_q = matched_reference['q']
    tcp_offsets = {s: (np.linalg.inv(torque._tcp_transform(s)) @ np.r_[cube_start, 1.0])[:3]
                   for s in ('left', 'right')}
    motion = {}

    def reference(side, q, dq):
        if not motion:
            return hold_q[side].copy(), np.zeros(7), np.zeros(7), 0, 'suspended_probe_hold'
        s, ds, dd = motion_progress(torque._time_s - start)
        delta = motion['q_end'][side] - motion['q_start'][side]
        return (motion['q_start'][side] + s * delta, ds * delta, dd * delta,
                0, 'suspended_probe_quintic_joint_ik')

    def step(dt):
        elapsed = torque._time_s - start
        if not torque._surface_grippers_closed():
            raise RuntimeError('SUSPENDED_PROBE_GRIPPER_OPEN')
        pos, _ = cube.get_world_pose(); pos = np.asarray(pos, float)
        if not np.isfinite(pos).all() or np.linalg.norm(pos - cube_start) > 0.03 or pos[2] < 0.40:
            raise RuntimeError('SUSPENDED_PROBE_CUBE_ENVELOPE')
        for side in ('left', 'right'):
            offset = (np.linalg.inv(torque._tcp_transform(side)) @ np.r_[pos, 1.0])[:3]
            if np.linalg.norm(offset - tcp_offsets[side]) > 0.005:
                raise RuntimeError('SUSPENDED_PROBE_GRASP_DRIFT:' + side)
        if elapsed >= MOTION_START_S and not motion:
            model = torque._task27_fk_model
            motion['q_start'] = {s: np.asarray(torque.articulations[s].get_joint_positions(), float)[:7].copy()
                                 for s in ('left', 'right')}
            motion['cube_start'] = pos.copy()
            if matched_reference is not None:
                motion['q_start'] = {s: hold_q[s].copy() for s in ('left', 'right')}
                motion['cube_start'] = matched_reference['cube'].copy()
            motion['q_end'] = {}
            ik_diag = {}
            for side in ('left', 'right'):
                actual = torque._tcp_transform(side)
                desired = model.fk(side, motion['q_start'][side]); desired[0, 3] += amplitude_m
                measured_q = np.asarray(torque.articulations[side].get_joint_positions(), float)[:7]
                if np.linalg.norm(model.fk(side, measured_q)[:3, 3] - actual[:3, 3]) > 0.0002:
                    raise RuntimeError('SUSPENDED_PROBE_FK_MISMATCH:' + side)
                ik = model.ik(side, desired, seeds=[motion['q_start'][side]], max_attempts=1, max_nfev=300)
                if not ik.success:
                    raise RuntimeError('SUSPENDED_PROBE_IK_FAILED:' + side + ':' + ik.reason)
                motion['q_end'][side] = np.asarray(ik.q)
                if np.max(np.abs(motion['q_end'][side] - motion['q_start'][side])) > 0.10:
                    raise RuntimeError('SUSPENDED_PROBE_IK_JOINT_JUMP:' + side)
                ik_diag[side] = {'q_start': motion['q_start'][side].tolist(),
                                 'q_end': list(ik.q), 'translation_error_m': ik.translation_error_m,
                                 'rotation_error_rad': ik.rotation_error_rad}
            torque._log({'event': 'SUSPENDED_PROBE_MOTION_START', 'physics_step': torque._step,
                         'time_s': torque._time_s, 'cube_start_world_m': motion['cube_start'].tolist(),
                         'actual_cube_start_world_m': pos.tolist(),
                         'amplitude_world_x_m': amplitude_m, 'ik': ik_diag})
        torque.set_internal_force_target(min(1.0, max(0.0, elapsed / 2.0))) if elapsed <= 2.0 else None
        old_step(dt)
        fy = FORCES_N[min(int((elapsed - 4.0) / CASE_S), len(FORCES_N)-1)] if 4 <= elapsed < 18 else 0.0
        if fy:
            sim.apply_force_at_pos(stage_id, encoded, Gf.Vec3f(0, fy, 0), Gf.Vec3f(*pos), 'Force')
        progress = motion_progress(elapsed)[0]
        torque._log({'event': 'SUSPENDED_PROBE_COMMAND', 'physics_step': torque._step - 1,
                     'state_time_s': torque._time_s - float(dt), 'elapsed_s': elapsed,
                     'force_world_N': [0.0, fy, 0.0], 'force_point_world_m': pos.tolist(),
                     'force_semantics': 'injected_on_cube_next_interval_not_suction_force',
                     'motion_progress': progress,
                     'cube_target_world_m': (motion['cube_start'] + [amplitude_m*progress, 0, 0]).tolist() if motion else None})

    torque._trajectory_reference = reference
    torque._torque_step = step
    torque._log({'event': 'SUSPENDED_PROBE_START', 'time_s': start,
                 'force_schedule_N': FORCES_N, 'case_duration_s': CASE_S,
                 'motion_start_s': MOTION_START_S, 'move_s': MOVE_S, 'hold_s': HOLD_S,
                 'amplitude_m': amplitude_m, 'force_is_controller_input': False})
    deadline = time.monotonic() + timeout_s
    try:
        while torque._time_s - start < END_S - 1e-8 and not torque._latched_error:
            if time.monotonic() > deadline:
                raise RuntimeError('SUSPENDED_PROBE_TIMEOUT')
            app.update()
        if torque._latched_error:
            raise RuntimeError('SUSPENDED_PROBE_STOPPED:' + str(torque._latched_error))
    finally:
        torque._torque_step, torque._trajectory_reference = old_step, old_ref
    return {'start_s': start, 'end_s': torque._time_s, 'amplitude_m': amplitude_m,
            'cube_start_world_m': cube_start.tolist(), 'motion_started': bool(motion)}


def run_matched_comparison(app, torque, stage):
    """Two bounded probes share the same desired grasp; reset by torque motion, never teleport."""
    old_ref = torque._trajectory_reference
    old_ff = torque._assumed_internal_contact_load
    q = {s: np.asarray(torque.articulations[s].get_joint_positions(), float)[:7].copy()
         for s in ('left', 'right')}
    shared = {'q': q}
    resets, probes = {}, {}
    torque._trajectory_reference = lambda side, *_: (q[side].copy(), np.zeros(7), np.zeros(7),
                                                      0, 'matched_reset_hold')
    try:
        for case, enabled in (('off', False), ('on', True)):
            torque.set_internal_force_target(0.0)
            torque._assumed_internal_contact_load = False
            end = torque._time_s + 6.0
            while torque._time_s < end and not torque._latched_error:
                app.update()
            if torque._latched_error or not torque._surface_grippers_closed():
                raise RuntimeError('MATCHED_RESET_FAILED:' + str(torque._latched_error))
            pos = np.asarray(torque.feed.rigids[0].get_world_pose()[0], float)
            if 'cube' not in shared:
                shared['cube'] = pos.copy()
            receipt = {'event': 'MATCHED_PROBE_RESET', 'case': case, 'time_s': torque._time_s,
                       'q_rad': {s: np.asarray(torque.articulations[s].get_joint_positions(), float)[:7].tolist()
                                 for s in ('left', 'right')},
                       'dq_rad_s': {s: np.asarray(torque.articulations[s].get_joint_velocities(), float)[:7].tolist()
                                    for s in ('left', 'right')},
                       'cube_world_m': pos.tolist(), 'reference_q_rad': {s: q[s].tolist() for s in q}}
            resets[case] = receipt
            torque._log(receipt)
            if case == 'on':
                dq = max(np.max(np.abs(np.asarray(receipt['q_rad'][s]) - resets['off']['q_rad'][s])) for s in q)
                dp = float(np.linalg.norm(pos - shared['cube']))
                if dq > 0.001 or dp > 0.0003:
                    raise RuntimeError(f'MATCHED_RESET_MISMATCH:q={dq},cube={dp}')
            torque._assumed_internal_contact_load = enabled
            probes[case] = run_suspended_probe(app, torque, stage, matched_reference=shared)
    finally:
        torque.set_internal_force_target(0.0)
        torque._trajectory_reference = old_ref
        torque._assumed_internal_contact_load = old_ff
    return {'reset_states': resets, 'probes': probes,
            'shared_reference_q_rad': {s: q[s].tolist() for s in q},
            'shared_cube_reference_m': shared['cube'].tolist()}
