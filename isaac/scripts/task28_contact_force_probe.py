#!/usr/bin/env python3
"""Isolated force-driven copy of one Task27 suction cup against parked Cube_01."""
import argparse
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from isaac.scripts.task28_task27_one_pass import load_scene

CUBE = '/World/Task27/Supply/Cube_01'
TOOL = '/World/Task28ContactProbe/Cup'


def target(t, initial, cube_displacement):
    if t < 0.5:
        return 'SEPARATED', initial
    if t < 1.7:
        return 'APPROACH', initial + 0.006 * (t - 0.5)
    if t < 2.1:
        return 'LOAD', initial + 0.006 + cube_displacement + 0.0005
    if t < 3.1:
        return 'RETRACT', initial + 0.006 - 0.011 * (t - 2.1)
    return 'SEPARATED_AFTER', initial - 0.005


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        raise ValueError('invalid run ID')
    out = ROOT / 'offline/task28_stage_b_contact/evidence' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': True, 'renderer': 'RaytracedLighting',
                         'multi_gpu': False, 'sync_loads': True})
    rows, events, subs = [], [], []
    result = {'status': 'FAILED', 'cube': CUBE, 'tool': TOOL,
              'tool_description': 'one force-driven copy of original suction cup collision cylinder',
              'tool_radius_m': 0.01, 'tool_height_m': 0.01, 'cube_pose_writes': 0}
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, UsdGeom, UsdPhysics
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, '/World')
        load_scene()
        # Cube_01 remains in its original gravity-disabled supply location; no pose is written.
        cup = UsdGeom.Cylinder.Define(stage, TOOL)
        cup.CreateRadiusAttr(0.01)
        cup.CreateHeightAttr(0.01)
        cup.CreateAxisAttr('Y')
        cup.AddTranslateOp().Set(Gf.Vec3d(-0.6, -0.371, -5.0))
        UsdPhysics.CollisionAPI.Apply(cup.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cup.GetPrim())
        UsdPhysics.MassAPI.Apply(cup.GetPrim()).CreateMassAttr().Set(0.2)
        cup_physics = PhysxSchema.PhysxRigidBodyAPI.Apply(cup.GetPrim())
        cup_physics.CreateDisableGravityAttr().Set(True)
        cup_physics.CreateLinearDampingAttr().Set(0.0)
        cube_physics = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(CUBE))
        cube_physics.CreateLinearDampingAttr().Set(0.0)
        view = RigidContactView(CUBE, [TOOL], name='task28_tool_cube_pair',
                                disable_stablization=False, max_contact_count=64)
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control', True):
            raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        cube_h, tool_h = dc.get_rigid_body(CUBE), dc.get_rigid_body(TOOL)
        if cube_h == _dynamic_control.INVALID_HANDLE or tool_h == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError('INVALID_PHYSX_HANDLE')
        view.initialize()
        if not view.is_physics_handle_valid() or view.num_shapes != 1 or view.num_filters != 1:
            raise RuntimeError(f'INVALID_CONTACT_VIEW:{view.num_shapes}:{view.num_filters}')
        initial = float(dc.get_rigid_body_pose(tool_h).p.y)
        state = {'step': 0, 'time': 0.0, 'error': None, 'last_force': 0.0}
        dc.wake_up_rigid_body(cube_h)
        dc.wake_up_rigid_body(tool_h)

        def contact_report(headers, data):
            for header in headers:
                paths = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (header.collider0, header.collider1)]
                if CUBE not in paths:
                    continue
                points = []
                for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                    d = data[i]
                    points.append({'normal_world': list(d.normal), 'impulse_world_Ns': list(d.impulse),
                                   'position_world_m': list(d.position), 'separation_m': float(d.separation)})
                events.append({'step_at_callback': state['step'], 'time_at_callback_s': state['time'],
                               'wall_ns': time.time_ns(), 'colliders': paths, 'points': points})

        def post_step(dt):
            try:
                state['step'] += 1
                state['time'] += float(dt)
                cube_pose, tool_pose = dc.get_rigid_body_pose(cube_h), dc.get_rigid_body_pose(tool_h)
                cp, tp = cube_pose.p, tool_pose.p
                cv, tv = dc.get_rigid_body_linear_velocity(cube_h), dc.get_rigid_body_linear_velocity(tool_h)
                phase, desired = target(state['time'], initial, float(cp.y) + 0.3)
                f = np.asarray(view.get_contact_force_matrix(dt=float(dt)), dtype=float).reshape(-1, 3)[0]
                if not np.isfinite(f).all():
                    raise RuntimeError('NONFINITE_CONTACT_FORCE')
                relative_velocity = tv.y - cv.y if phase == 'LOAD' else tv.y
                applied = float(np.clip(70.0 * (desired - tp.y) - 8.0 * relative_velocity, -1.0, 1.0))
                row = {'step': state['step'], 'simulation_time_s': state['time'], 'dt_s': float(dt),
                       'wall_ns': time.time_ns(), 'phase': phase, 'tool_target_y_m': desired,
                       'tool_xyz_m': [float(tp.x), float(tp.y), float(tp.z)],
                       'cube_xyz_m': [float(cp.x), float(cp.y), float(cp.z)],
                       'tool_quaternion_xyzw': [float(tool_pose.r.x), float(tool_pose.r.y), float(tool_pose.r.z), float(tool_pose.r.w)],
                       'cube_quaternion_xyzw': [float(cube_pose.r.x), float(cube_pose.r.y), float(cube_pose.r.z), float(cube_pose.r.w)],
                       'tool_velocity_m_s': [float(tv.x), float(tv.y), float(tv.z)],
                       'cube_velocity_m_s': [float(cv.x), float(cv.y), float(cv.z)],
                       'tool_cube_force_world_N': f.tolist(), 'contact': bool(np.linalg.norm(f) > 1e-5),
                       'next_tool_force_world_N': [0.0, applied, 0.0]}
                rows.append(row)
                if (np.linalg.norm(f) > 5.0 or abs(tp.y - initial) > 0.05 or
                        abs(cp.y + 0.3) > 0.05 or abs(tv.y) > 0.15 or abs(cv.y) > 0.15):
                    raise RuntimeError('FORCE_OR_MOTION_BOUND_EXCEEDED')
                # Apply at the tool centre in the world frame for the following physics step.
                dc.apply_body_force(tool_h, (0.0, applied, 0.0), (float(tp.x), float(tp.y), float(tp.z)), True)
            except Exception as exc:
                state['error'] = repr(exc)

        subs.append(omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(contact_report))
        subs.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step, False, 0))
        start = time.monotonic()
        while state['time'] < 3.5 and not state['error'] and time.monotonic() - start < 180:
            app.update()
        if state['error']:
            raise RuntimeError(state['error'])
        if state['time'] < 3.5:
            raise RuntimeError('PHYSICS_TIMEOUT')
        result.update(status='RECORDED', samples=len(rows),
                      pair_events=sum(set(e['colliders']) == {CUBE, TOOL} for e in events),
                      other_cube_events=sum(set(e['colliders']) != {CUBE, TOOL} for e in events),
                      simulation_time_s=state['time'], source='RigidContactView Cube_01 filtered by Cup; dt-scaled impulse',
                      state_source='dynamic_control PhysX rigid-body pose and velocity in same post-step callback')
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        for sub in subs:
            sub.unsubscribe()
        for name, records in [('timeseries.jsonl', rows), ('contact_events.jsonl', events)]:
            with (out / name).open('x') as stream:
                for record in records:
                    stream.write(json.dumps(record, allow_nan=False) + '\n')
        with (out / 'summary.json').open('x') as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write('\n')
        print('TASK28_CONTACT_PROBE', json.dumps(result), flush=True)
        app.close()
    return 0 if result['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
