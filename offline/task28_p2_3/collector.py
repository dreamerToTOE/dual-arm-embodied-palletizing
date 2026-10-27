#!/usr/bin/env python3
"""Isolated TWO-FR3 virtual-contact collector; no estimator, Cube, ROS or bridge.

Use frozen preregistration and a fresh output directory.  Applied wrenches are
recorded only as independent evaluation oracles.  Each trial resets the entire
world; left/right records within a sample share one physics step, while paired
unloaded/loaded trials are NOT physically synchronous.
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
CONFIG = Path(__file__).resolve().parent / 'evidence/preregistration.json'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def dump(path, data):
    with path.open('x', encoding='utf-8') as f:
        json.dump(data, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')
        f.flush()
        os.fsync(f.fileno())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--asset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    if not args.asset.is_file() or digest(args.asset) != config['fixture_usd_sha256']:
        raise SystemExit('MISSING_OR_WRONG_FROZEN_ASSET')
    source_state = ROOT / 'offline/task28_g1_1/tests/control_states.json'
    saved = next(x for x in json.loads(source_state.read_text())['states'] if x['label'] == 'lifted_symmetric')
    if saved != config['state']:
        raise SystemExit('FROZEN_SAVED_G1_STATE_MISMATCH')
    args.output.mkdir(parents=False, exist_ok=False)
    config_hash = digest(CONFIG)
    start = time.monotonic()
    state = {'status': 'STARTED', 'source_kind': 'ISAAC_TWO_ARTICULATIONS_STANDALONE',
             'scope': 'VIRTUAL_CONTACT_STATIC', 'config_sha256': config_hash,
             'asset': str(args.asset.resolve()), 'asset_sha256': digest(args.asset),
             'collector_sha256': digest(Path(__file__)), 'g1_source_state_sha256': digest(source_state),
             'flags': config['flags'], 'command': sys.argv, 'completed_condition_files': []}
    app = active = None
    total_steps = 0
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
        from pxr import PhysxSchema
        sys.path.insert(0, str(ROOT))
        from offline.task28_g1_1.task28_validator.model import RobotModel
        model = RobotModel()
        world = World(stage_units_in_meters=1.0, physics_dt=config['physics_dt_s'],
                      rendering_dt=config['physics_dt_s'], backend='numpy', device='cpu')
        robots, links, indices, targets, link7_to_tcp = {}, {}, {}, {}, {}
        for side in ('left', 'right'):
            prim = '/World/Probe_' + side
            add_reference_to_stage(usd_path=str(args.asset.resolve()), prim_path=prim)
            robots[side] = world.scene.add(SingleArticulation(prim, name='probe_' + side,
                position=np.asarray(config['base_world_m'][side]),
                orientation=np.asarray(config['base_world_orientation_wxyz'])))
            links[side] = world.scene.add(RigidPrim(prim + '/fr3_link7', name='link7_' + side))
        world.reset()
        runtime = {}
        for side in ('left', 'right'):
            robot = robots[side]
            names = list(robot.dof_names)
            requested = config['joint_names']
            if len(names) != len(set(names)) or any(names.count(n) != 1 for n in requested):
                raise ValueError('DOF_NAMES_MISSING_OR_AMBIGUOUS:' + side + ':' + repr(names))
            idx = np.array([robot.get_dof_index(n) for n in requested], dtype=np.int32)
            if len(set(idx.tolist())) != 7 or any(names[i] != n for i,n in zip(idx, requested)):
                raise ValueError('DOF_INDEX_MAPPING_INVALID:' + side)
            indices[side] = idx
            q0 = np.asarray(config['state'][side + '_q'], dtype=float)
            if not model.within_limits(q0):
                raise ValueError('CONFIG_JOINT_LIMIT:' + side)
            targets[side] = ArticulationAction(joint_positions=q0, joint_indices=idx)
            frames = model._chain(side, q0)[0]
            link7_to_tcp[side] = np.linalg.inv(frames[side + '_fr3_link7']) @ model.fk(side, q0)
            stiffness, damping = robot.get_articulation_controller().get_gains()
            stiffness, damping = np.asarray(stiffness), np.asarray(damping)
            if (stiffness.shape != (len(names),) or damping.shape != (len(names),)
                    or not np.isfinite(np.r_[stiffness, damping]).all() or np.any(stiffness[idx] <= 0)):
                raise ValueError('POSITION_HOLD_GAINS_INVALID:' + side)
            pose, quat = robot.get_world_pose()
            if not np.allclose(pose, config['base_world_m'][side], atol=1e-6, rtol=0):
                raise ValueError('BASE_POSITION_MISMATCH:' + side + ':' + repr(pose))
            if not np.allclose(np.abs(np.dot(quat, config['base_world_orientation_wxyz'])), 1, atol=1e-6):
                raise ValueError('BASE_ORIENTATION_MISMATCH:' + side)
            api = PhysxSchema.PhysxArticulationAPI(world.stage.GetPrimAtPath('/World/Probe_' + side))
            props = {n: getattr(api,n)().Get() if hasattr(api,n) else None for n in
                     ('GetSolverPositionIterationCountAttr','GetSolverVelocityIterationCountAttr',
                      'GetEnabledSelfCollisionsAttr','GetSleepThresholdAttr','GetStabilizationThresholdAttr')}
            runtime[side] = {'articulation_path': '/World/Probe_' + side,
                'link7_path': '/World/Probe_' + side + '/fr3_link7', 'all_dof_names': names,
                'ordered_joint_names': requested, 'ordered_dof_indices': idx.tolist(),
                'dof_stiffness': stiffness.tolist(), 'dof_damping': damping.tolist(),
                'base_world_position_m': np.asarray(pose).tolist(), 'base_world_orientation_wxyz': np.asarray(quat).tolist(),
                'link7_to_frozen_virtual_tcp': link7_to_tcp[side].tolist(), 'asset_physx_articulation_properties': props}
        gravity = world.get_physics_context().get_gravity()
        gravity_vec = np.asarray(gravity[0], float) * float(gravity[1])
        if not np.allclose(gravity_vec, config['gravity_m_s2'], atol=1e-6, rtol=0):
            raise ValueError('RUNTIME_GRAVITY_MISMATCH:' + repr(gravity))
        if abs(world.get_physics_dt() - config['physics_dt_s']) > 1e-12:
            raise ValueError('PHYSICS_DT_MISMATCH')
        state['runtime'] = {'arms': runtime, 'physics_dt_s': world.get_physics_dt(),
            'physics_gravity': gravity, 'observation_source': 'SingleArticulation.get_measured_joint_efforts',
            'force_application_api': 'RigidPrim.apply_forces_and_torques_at_pos(is_global=True)',
            'force_frame': 'world', 'torque_frame': 'world',
            'reference_point': 'frozen nominal virtual TCP; applied to link7 without physical tool or Cube',
            'simultaneity': 'both arms same physics step per record; unloaded and loaded whole-world resets are separate'}

        def link_pose_and_fk(side, q):
            pos, quat = links[side].get_world_poses()
            rot = Rotation.from_quat(np.asarray(quat[0])[[1,2,3,0]]).as_matrix()
            expected = model._chain(side, q)[0][side + '_fr3_link7']
            pe = float(np.linalg.norm(pos[0] - expected[:3,3]))
            re = float(np.linalg.norm(Rotation.from_matrix(expected[:3,:3].T @ rot).as_rotvec()))
            if pe > config['thresholds']['max_link7_fk_position_difference_m'] or re > config['thresholds']['max_link7_fk_rotation_difference_rad']:
                raise ValueError('LIVE_FK_MISMATCH:%s:position=%.12g:rotation=%.12g' % (side, pe, re))
            measured = np.eye(4)
            measured[:3,:3], measured[:3,3] = rot, pos[0]
            tcp = measured @ link7_to_tcp[side]
            return pos[0], quat[0], tcp, pe, re

        for case in config['cases']:
            for condition in ('unloaded', 'loaded'):
                if time.monotonic() - start >= config['wall_timeout_s']:
                    raise TimeoutError('PREREGISTERED_WALL_BUDGET')
                world.reset()
                for side in ('left', 'right'):
                    q0 = np.asarray(config['state'][side + '_q'], float)
                    robots[side].set_joint_positions(q0, joint_indices=indices[side])
                    robots[side].set_joint_velocities(np.zeros(7), joint_indices=indices[side])
                    robots[side].apply_action(targets[side])
                # One unloaded step synchronizes PhysX link state with the set joint state.
                world.step(render=False)
                total_steps += 1
                prechecks = {}
                for side in ('left', 'right'):
                    q = np.asarray(robots[side].get_joint_positions(joint_indices=indices[side]), float)
                    pos, quat, tcp, pe, re = link_pose_and_fk(side, q)
                    prechecks[side] = {'physics_step': int(world.current_time_step_index),
                        'q_rad': q.tolist(), 'link7_position_world_m': pos.tolist(),
                        'link7_orientation_world_wxyz': quat.tolist(), 'tcp_world': tcp.tolist(),
                        'link7_fk_translation_error_m': pe, 'link7_fk_rotation_error_rad': re}
                obs = []
                active = {'case': case, 'condition': condition, 'prechecks': prechecks,
                          'observations': obs, 'config_sha256': config_hash}
                for step in range(config['settle_steps_each_condition']):
                    if total_steps >= config['max_steps'] or time.monotonic() - start >= config['wall_timeout_s']:
                        raise TimeoutError('PREREGISTERED_BUDGET')
                    before = {}
                    for side in ('left', 'right'):
                        robots[side].apply_action(targets[side])
                        qpre = np.asarray(robots[side].get_joint_positions(joint_indices=indices[side]), float)
                        _, _, tcp, _, _ = link_pose_and_fk(side, qpre)
                        spec = case[side] if condition == 'loaded' else {'force_world_n':[0,0,0], 'pure_torque_world_nm':[0,0,0], 'offset_from_tcp_world_m':[0,0,0]}
                        force = np.asarray(spec['force_world_n'], float)
                        torque = np.asarray(spec['pure_torque_world_nm'], float)
                        point = tcp[:3,3] + np.asarray(spec['offset_from_tcp_world_m'], float)
                        links[side].apply_forces_and_torques_at_pos(forces=force[None,:], torques=torque[None,:],
                                                                  positions=point[None,:], is_global=True)
                        before[side] = {'q_pre_step_rad': qpre.tolist(), 'tcp_pre_step_world': tcp.tolist(),
                            'applied_force_world_n': force.tolist(), 'applied_pure_torque_world_nm': torque.tolist(),
                            'application_point_world_m': point.tolist()}
                    world.step(render=False)
                    total_steps += 1
                    if step >= config['settle_steps_each_condition'] - config['observation_window_steps']:
                        pair = {'physics_step': int(world.current_time_step_index),
                                'simulation_time_s': float(world.current_time), 'global_collector_step': total_steps,
                                'arms': {}}
                        for side in ('left', 'right'):
                            robot, idx = robots[side], indices[side]
                            q = np.asarray(robot.get_joint_positions(joint_indices=idx), float)
                            dq = np.asarray(robot.get_joint_velocities(joint_indices=idx), float)
                            tau = np.asarray(robot.get_measured_joint_efforts(joint_indices=idx), float)
                            if any(x.shape != (7,) for x in (q,dq,tau)) or not np.isfinite(np.r_[q,dq,tau]).all():
                                raise ValueError('MALFORMED_OR_NONFINITE_ACTUAL_OBSERVATION:' + side)
                            pos, quat, tcp, pe, re = link_pose_and_fk(side, q)
                            pair['arms'][side] = {'source_id': 'probe_' + side, 'joint_names': config['joint_names'],
                                'q_rad': q.tolist(), 'dq_rad_s': dq.tolist(), 'effort_nm': tau.tolist(),
                                'link7_position_world_m': pos.tolist(), 'link7_orientation_world_wxyz': quat.tolist(),
                                'tcp_world': tcp.tolist(), 'link7_fk_translation_error_m': pe,
                                'link7_fk_rotation_error_rad': re, 'load_oracle': before[side]}
                        obs.append(pair)
                filename = case['id'] + '_' + condition + '.json'
                dump(args.output / filename, {'case':case, 'condition':condition, 'prechecks':prechecks,
                    'observations':obs, 'source_kind':'ISAAC_TWO_ARTICULATIONS_STANDALONE',
                    'oracle_separate_from_estimator':True, 'config_sha256':config_hash})
                state['completed_condition_files'].append(filename)
                active = None
        state['status'] = 'COLLECTION_COMPLETE_NOT_EVALUATED'
    except BaseException as exc:
        state['status'] = 'COLLECTION_FAILED'
        state['failure'] = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        if active is not None:
            dump(args.output / 'partial_condition.json', active)
            state['partial_condition_file'] = 'partial_condition.json'
    finally:
        state['elapsed_wall_s'] = time.monotonic() - start
        state['actual_physics_steps'] = total_steps
        dump(args.output / 'collector_status.json', state)
        if app is not None:
            app.close()
    return 0 if state['status'] == 'COLLECTION_COMPLETE_NOT_EVALUATED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
