#!/usr/bin/env python3
"""Isolated pre-registered 9-DOF Isaac source collector, not an observer.

Known applied loads are isolated in separate oracle files. A sample is a current
state plus a *previous-step average* projected constraint effort. This temporal
mismatch and any solver contributions are not automatically corrected here.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[2]
CONFIG = Path(__file__).resolve().parent / 'evidence/experiment_preregistration.json'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def dump(path, obj):
    with path.open('x', encoding='utf-8') as f:
        json.dump(obj, f, sort_keys=True, indent=2, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--prereg-sha256', required=True)
    parser.add_argument('--stage', choices=('static', 'motion_zero', 'motion_plus'), required=True)
    parser.add_argument('--motion-source-gate', type=Path)
    args = parser.parse_args()
    cfg_hash = digest(CONFIG)
    if cfg_hash != args.prereg_sha256:
        raise SystemExit('PREREGISTRATION_HASH_MISMATCH')
    c = json.loads(CONFIG.read_text(encoding='utf-8'))
    if not args.asset.is_file() or digest(args.asset) != c['fixture_usd_sha256']:
        raise SystemExit('MISSING_OR_WRONG_ASSET')
    if args.stage == 'motion_plus':
        if args.motion_source_gate is None or not args.motion_source_gate.is_file():
            raise SystemExit('MOTION_SOURCE_GATE_REQUIRED')
        gate = json.loads(args.motion_source_gate.read_text(encoding='utf-8'))
        if gate.get('status') != 'INDEPENDENT_DYNAMIC_SOURCE_VALIDATED' or gate.get('preregistration_sha256') != cfg_hash:
            raise SystemExit('MOTION_SOURCE_GATE_NOT_VALIDATED')
    args.output.mkdir(parents=False, exist_ok=False)
    state = {'status': 'STARTED', 'command': sys.argv, 'preregistration_sha256': cfg_hash,
             'asset': str(args.asset.resolve()), 'asset_sha256': digest(args.asset),
             'collector_sha256': digest(Path(__file__)), 'flags': c['boundaries'],
             'completed_cases': [], 'stage': args.stage,
             'motion_source_gate_sha256': digest(args.motion_source_gate) if args.motion_source_gate else None}
    start = time.monotonic()
    app = None
    active_obs = active_oracle = None
    total = 0
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'multi_gpu': False,
                             'anti_aliasing': 0, 'sync_loads': False, 'fast_shutdown': True})
        import numpy as np
        from scipy.spatial.transform import Rotation
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        from isaacsim.core.utils.types import ArticulationAction
        sys.path.insert(0, str(ROOT / 'offline/task28_g1_1'))
        from task28_validator.model import RobotModel
        model = RobotModel()
        world = World(stage_units_in_meters=1.0, physics_dt=c['physics_dt_s'],
                      rendering_dt=c['physics_dt_s'], backend='numpy', device='cpu')
        add_reference_to_stage(usd_path=str(args.asset.resolve()), prim_path='/World/ProbeFR3')
        robot = world.scene.add(SingleArticulation('/World/ProbeFR3', name='fr3',
                                                   position=np.asarray(c['base_world_m'])))
        link = world.scene.add(RigidPrim('/World/ProbeFR3/fr3_link7', name='link7'))
        world.reset()
        names = list(robot.dof_names)
        if names != c['expected_all_dof_names']:
            raise ValueError('DOF_NAMES_OR_ORDER_MISMATCH:' + repr(names))
        indices = np.array([robot.get_dof_index(n) for n in c['joint_names']], dtype=np.int32)
        if len(set(indices.tolist())) != 7 or any(names[i] != n for i,n in zip(indices,c['joint_names'])):
            raise ValueError('ARM_INDEX_MAPPING_INVALID')
        view = robot._articulation_view  # 4.5 implementation; static inspection identifies this view.
        gains = robot.get_articulation_controller().get_gains()
        gravity = world.get_physics_context().get_gravity()
        if not np.allclose(np.asarray(gravity[0])*gravity[1], c['gravity_m_s2'], atol=1e-6, rtol=0):
            raise ValueError('GRAVITY_MISMATCH')
        if abs(world.get_physics_dt() - c['physics_dt_s']) > 1e-12:
            raise ValueError('PHYSICS_DT_MISMATCH')
        state['runtime'] = {'all_dof_names': names, 'arm_indices': indices.tolist(),
                            'gains_stiffness': np.asarray(gains[0]).tolist(),
                            'gains_damping': np.asarray(gains[1]).tolist(),
                            'physics_dt_s': world.get_physics_dt(), 'gravity': gravity,
                            'observation_view': 'SingleArticulation._articulation_view',
                            'axis_units': ['rad, rad/s, Nm'] * 7 + ['m, m/s, N'] * 2,
                            'model_units': 'M[i,j]: generalized force[i] / generalized acceleration[j] (mixed revolute/prismatic); c/g[i]: Nm for first seven, N for last two',
                            'effort_temporal_support': 'preceding solver step average; may include drive, friction and limits',
                            'dynamics_temporal_support': 'query after step at current q/dq; source semantics not independently validated'}
        for case in c['cases']:
            if args.stage == 'static' and case['motion']:
                continue
            if args.stage != 'static' and case['id'] != args.stage:
                continue
            if time.monotonic() - start >= c['wall_timeout_s']:
                raise TimeoutError('WALL_BUDGET')
            q0 = np.array(c['configurations_rad'][case['configuration']], dtype=float)
            if not model.within_limits(q0):
                raise ValueError('JOINT_LIMIT_CONFIG:' + case['id'])
            world.reset()
            robot.set_joint_positions(q0, joint_indices=indices)
            robot.set_joint_velocities(np.zeros(7), joint_indices=indices)
            force = np.array(case['force_world_n'], float)
            torque = np.array(case['torque_world_nm'], float)
            steps = c['motion_steps'] if case['motion'] else c['static_steps']
            active_obs = {'case_id': case['id'], 'preregistration_sha256': cfg_hash, 'samples': []}
            active_oracle = {'case_id': case['id'], 'force_world_n': force.tolist(),
                             'pure_torque_world_nm': torque.tolist(), 'application_points_world_m': []}
            prev_sim_time = None
            for step in range(steps):
                if total >= c['max_steps'] or time.monotonic() - start >= c['wall_timeout_s']:
                    raise TimeoutError('PRE_REGISTERED_BUDGET')
                target_q = q0.copy()
                if case['motion']:
                    phase = min(1.0, max(0.0, (step-c['motion_start_step']) /
                                          (c['motion_stop_step']-c['motion_start_step'])))
                    target_q[0] += c['motion_amplitude_rad'] * (1.0 - np.cos(np.pi * phase)) / 2.0
                robot.apply_action(ArticulationAction(joint_positions=target_q, joint_indices=indices))
                q_pre = np.asarray(robot.get_joint_positions(joint_indices=indices), float)
                point = model.fk('left', q_pre)[:3, 3]
                link.apply_forces_and_torques_at_pos(forces=force[None,:], torques=torque[None,:],
                                                     positions=point[None,:], is_global=True)
                world.step(render=False)
                total += 1
                read_start = time.monotonic()
                q = np.asarray(robot.get_joint_positions(), float)
                dq = np.asarray(robot.get_joint_velocities(), float)
                effort = np.asarray(robot.get_measured_joint_efforts(), float)
                mass = np.asarray(view.get_mass_matrices(), float)
                coriolis = np.asarray(view.get_coriolis_and_centrifugal_forces(), float)
                grav = np.asarray(view.get_generalized_gravity_forces(), float)
                pos, quat = link.get_world_poses()
                read_end = time.monotonic()
                if q.shape != (9,) or dq.shape != (9,) or effort.shape != (9,) or mass.shape != (1,9,9) or coriolis.shape != (1,9) or grav.shape != (1,9):
                    raise ValueError('SOURCE_SHAPE_INVALID:' + repr([q.shape,dq.shape,effort.shape,mass.shape,coriolis.shape,grav.shape]))
                if not np.isfinite(np.r_[q,dq,effort,mass.ravel(),coriolis.ravel(),grav.ravel()]).all():
                    raise ValueError('SOURCE_NONFINITE')
                t = float(world.current_time)
                if prev_sim_time is not None and (t <= prev_sim_time or abs((t-prev_sim_time)-c['physics_dt_s']) > 1e-6):
                    raise ValueError('SIM_TIME_NOT_MONOTONIC_OR_STEP_MISMATCH')
                prev_sim_time = t
                fk = model._chain('left', q[indices])[0]['left_fr3_link7']
                actual_R = Rotation.from_quat(np.asarray(quat[0])[[1,2,3,0]]).as_matrix()
                active_obs['samples'].append({'case_step': step, 'sim_time_s': t,
                    'physics_step_index': int(world.current_time_step_index),
                    'effort_interval_end_s': t, 'effort_interval_start_s': t-c['physics_dt_s'],
                    'state_model_query_time_s': t, 'source_read_duration_s': read_end-read_start,
                    'q_rad_or_m': q.tolist(), 'dq_rad_s_or_m_s': dq.tolist(),
                    'measured_projected_effort': effort.tolist(), 'mass_matrix': mass[0].tolist(),
                    'coriolis_compensation': coriolis[0].tolist(), 'gravity_compensation': grav[0].tolist(),
                    'target_arm_q_rad': target_q.tolist(), 'link7_position_world_m': np.asarray(pos[0]).tolist(),
                    'link7_orientation_world_wxyz': np.asarray(quat[0]).tolist(),
                    'link7_fk_position_error_m': float(np.linalg.norm(pos[0]-fk[:3,3])),
                    'link7_fk_rotation_error_rad': float(np.linalg.norm(Rotation.from_matrix(fk[:3,:3].T@actual_R).as_rotvec()))})
                active_oracle['application_points_world_m'].append(point.tolist())
            dump(args.output / (case['id'] + '_observations.json'), active_obs)
            dump(args.output / (case['id'] + '_oracle.json'), active_oracle)
            active_obs = active_oracle = None
            state['completed_cases'].append(case['id'])
        state['status'] = 'COLLECTION_COMPLETE_NOT_EVALUATED'
    except BaseException as e:
        state['status'] = 'COLLECTION_FAILED'
        state['failure'] = {'type': type(e).__name__, 'message': str(e), 'traceback': traceback.format_exc()}
        if active_obs is not None:
            dump(args.output / 'partial_observations.json', active_obs)
        if active_oracle is not None:
            dump(args.output / 'partial_oracle.json', active_oracle)
    finally:
        state['physics_steps'] = total
        state['elapsed_wall_s'] = time.monotonic() - start
        dump(args.output / 'collector_status.json', state)
        if app is not None:
            app.close()
    return 0 if state['status'] == 'COLLECTION_COMPLETE_NOT_EVALUATED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
