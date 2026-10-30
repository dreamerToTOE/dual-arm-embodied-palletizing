"""R14 guarded reuse of the unchanged P2-4-R3 zero-load collector only.

The hook adds rejecting guards; it does not clip/change the frozen controller.
Its first gravity bootstrap retains the explicitly registered R3 exception.
Later steps additionally obey R8 per-second arm effort-rate ceilings.
No known force, contact geometry, Cube, suction or ROS command is introduced.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
R3 = ROOT / 'offline/task28_p2_4_r3'
R8 = ROOT / 'offline/task28_p5_qp_6_r8/source_contract.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def plan():
    cfg = json.loads((R3 / 'experiment_preregistration.json').read_text())
    limits = json.loads(R8.read_text())['limits']
    return {
        'schema': 'TASK28_R14_NO_CONTACT_PLAN_V1',
        'baseline_head': '72fbb8d5ea09ac9edb0a3869a07bed01e243741c',
        'collector': 'offline.task28_p2_4_r3.collector', 'stage': 'stability',
        'scene': 'isolated single /World/ProbeFR3, no Cube or ground plane',
        'steps': cfg['stability_steps'], 'physics_dt_s': cfg['physics_dt_s'],
        'controller_modified': False,
        'first_command_policy': 'P2-4-R3 registered full gravity bootstrap; per-second rate gate begins at second write, NOT R8 original-scene admission',
        'arm_absolute_limits': np.minimum(cfg['effort_bounds'][:7], np.minimum(limits['source_effort_abs_max_Nm'], limits['research_effort_abs_max_Nm'])).tolist(),
        'later_arm_rate_limits_Nm_s': limits['arm_effort_rate_max_Nm_s'],
        'legacy_finger_abs_and_per_step_limits': [cfg['effort_bounds'][7:], cfg['effort_rate_bounds'][7:]],
        'sources': {str(p.relative_to(ROOT)): sha(p) for p in (R3/'collector.py', R3/'evaluate.py', R3/'experiment_preregistration.json', R8, HERE/'no_contact.py')},
        'observer': 'postcollection strict P2-4 observer, raw unsymmetrized M; command effort candidate source; no feedback from estimate to actuation',
        'contact_stage_admission': False, 'hardware_commands': False,
    }


def guard_command(command, q, dq, gravity, previous, previous_dq, cfg, p):
    """Validate before the sink; a failure produces no command for this step."""
    u, q, dq, g = (np.asarray(x, float) for x in (command, q, dq, gravity))
    if any(x.shape != (9,) for x in (u, q, dq, g)) or not np.isfinite(np.r_[u, q, dq, g]).all():
        raise ValueError('INVALID_PRECOMMAND_STATE')
    if abs(q[0]) > cfg['q1_abs_limit_rad'] or np.max(abs(dq[:7])) > cfg['arm_speed_limit_rad_s']:
        raise ValueError('BOUNDED_MOTION_LIMIT')
    if np.any(abs(u[:7]) > p['arm_absolute_limits']) or np.any(abs(u[7:]) > np.array(cfg['effort_bounds'])[7:]):
        raise ValueError('EFFORT_LIMIT')
    dt = cfg['physics_dt_s']
    if previous is not None:
        delta = abs(u - np.asarray(previous))
        if np.any(delta > np.array(cfg['effort_rate_bounds'])) or np.any(delta[:7] > np.array(p['later_arm_rate_limits_Nm_s']) * dt):
            raise ValueError('EFFORT_RATE_LIMIT')
        if np.max(abs((dq[:7] - np.asarray(previous_dq)[:7]) / dt)) > cfg['arm_acceleration_limit_rad_s2']:
            raise ValueError('ACCELERATION_LIMIT')
    elif not np.allclose(u, g, rtol=0, atol=1e-6):
        raise ValueError('INITIAL_COMMAND_NOT_GRAVITY')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--portable-root', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args()
    registered = json.loads(args.plan.read_text())
    if registered != plan():
        raise RuntimeError('PLAN_OR_SOURCE_CHANGED')
    cfg = json.loads((R3/'experiment_preregistration.json').read_text())
    if sha(cfg['asset']) != cfg['fixture_usd_sha256']:
        raise RuntimeError('ASSET_HASH')
    info = {'plan_sha256': sha(args.plan), 'pid': os.getpid(), 'guarded_commands': 0,
            'external_force_calls': 0, 'scope': 'single-arm zero-load hold only'}
    infofile = args.output.parent / 'runtime_scope.json'
    journal = args.output.parent / 'precommand.jsonl'
    import isaacsim
    original_app = isaacsim.SimulationApp

    def isolated_app(options):
        options = dict(options)
        # Match the proven R12 headless startup contract.  The R3 collector's
        # default create_new_stage=True enters the viewport-wait path even in a
        # headless process; that is a startup policy issue, not an experiment
        # result.  World.reset/build owns the stage below.
        options['create_new_stage'] = False
        options['headless'] = True
        options['renderer'] = 'RaytracedLighting'
        options['extra_args'] = ['--portable-root', str(args.portable_root),
            '--/app/tokens/appcache=' + str(args.portable_root/'appcache'),
            '--/app/settings/persistent=0']
        app = original_app(options)
        try:
            import carb.tokens
            import omni.kit.app
            info['kit_tokens'] = {k: carb.tokens.get_tokens_interface().resolve('${'+k+'}') for k in ('appcache','cache','logs','data')}
            if not all(v.startswith(str(args.portable_root) + '/') for v in info['kit_tokens'].values()):
                raise RuntimeError('CACHE_NOT_ISOLATED')
            manager = omni.kit.app.get_app().get_extension_manager()
            for ext in ('isaacsim.ros2.bridge', 'omni.isaac.ros_bridge', 'omni.isaac.ros2_bridge'):
                if manager.is_extension_enabled(ext):
                    manager.set_extension_enabled_immediate(ext, False)
                if manager.is_extension_enabled(ext):
                    raise RuntimeError('ROS_BRIDGE_ENABLED')
            info['ros_bridge_disabled'] = True
            from isaacsim.core.prims import SingleArticulation, RigidPrim
            original_write = SingleArticulation.set_joint_efforts
            previous = {'u': None, 'dq': None, 'cursor': None}

            def no_force(*unused, **kwargs):
                raise RuntimeError('NO_EXTERNAL_FORCE_IN_STAGE1')

            RigidPrim.apply_forces_and_torques_at_pos = no_force

            def guarded(robot, efforts, joint_indices=None):
                from isaacsim.core.api import World
                world = World.instance()
                cursor = [int(world.current_time_step_index), float(world.current_time)]
                if previous['cursor'] is not None:
                    if cursor[0] != previous['cursor'][0]+1 or abs(cursor[1]-previous['cursor'][1]-cfg['physics_dt_s']) > 1e-6:
                        raise ValueError('PRECOMMAND_CURSOR')
                if str(robot.prim_path) != '/World/ProbeFR3' or list(robot.dof_names) != cfg['expected_all_dof_names']:
                    raise ValueError('ROBOT_IDENTITY')
                if not np.array_equal(joint_indices, np.arange(9)):
                    raise ValueError('DOF_WRITE_ORDER')
                view = robot._articulation_view
                kp, kd = (np.asarray(x).reshape(-1) for x in view.get_gains())
                if np.any(kp != 0) or np.any(kd != 0) or list(view.get_effort_modes()[0]) != ['force']*9:
                    raise ValueError('DRIVE_OWNERSHIP')
                if abs(world.get_physics_dt()-cfg['physics_dt_s']) > 1e-8:
                    raise ValueError('PHYSICS_DT')
                q = np.asarray(robot.get_joint_positions(), float).copy()
                dq = np.asarray(robot.get_joint_velocities(), float).copy()
                g = np.asarray(view.get_generalized_gravity_forces(), float)[0].copy()
                guard_command(efforts, q, dq, g, previous['u'], previous['dq'], cfg, registered)
                row = {'cursor': cursor, 'q': q.tolist(), 'dq': dq.tolist(), 'g': g.tolist(),
                       'command': np.asarray(efforts).tolist(), 'actual_kp': kp.tolist(), 'actual_kd': kd.tolist(),
                       'first_gravity_bootstrap': previous['u'] is None}
                with journal.open('a') as stream:
                    stream.write(json.dumps(row, allow_nan=False)+'\n'); stream.flush(); os.fsync(stream.fileno())
                original_write(robot, efforts, joint_indices=joint_indices)
                previous.update(u=np.asarray(efforts).copy(), dq=dq, cursor=cursor)
                info['guarded_commands'] += 1
                write(infofile, info)
            SingleArticulation.set_joint_efforts = guarded
            write(infofile, info)
            return app
        except BaseException:
            app.close()
            raise

    isaacsim.SimulationApp = isolated_app
    from offline.task28_p2_4_r3 import collector
    sys.argv = [str(R3/'collector.py'), '--asset', cfg['asset'], '--output', str(args.output),
                '--prereg-sha256', sha(R3/'experiment_preregistration.json'), '--stage', 'stability']
    try:
        return collector.main()
    finally:
        info['asset_unchanged'] = sha(cfg['asset']) == cfg['fixture_usd_sha256']
        write(infofile, info)


if __name__ == '__main__':
    raise SystemExit(main())
