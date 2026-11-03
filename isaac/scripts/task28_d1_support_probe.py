#!/usr/bin/env python3
"""Single bounded, robot-free Task27 Cube/table contact diagnostic."""
import argparse
import json
import os
import re
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.c2_control import json_record
from offline.task28_stage_b_contact.d1_math import balance

CUBE = '/World/Cube_01'
TABLE = '/World/Table'


def xyz(value):
    return [float(value.x), float(value.y), float(value.z)]


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
    row_stream = (out / 'timeseries.jsonl').open('x')
    event_stream = (out / 'contact_events.jsonl').open('x')
    result = {'status': 'FAILED', 'reason': 'not_started', 'scene': 'minimal Cube/table only',
              'external_force_or_pose_writes_after_play': 0}
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdShade, UsdUtils
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, '/World')
        scene = UsdPhysics.Scene.Define(stage, '/physicsScene')
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        phys_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        phys_scene.CreateEnableCCDAttr().Set(True)
        phys_scene.CreateEnableStabilizationAttr().Set(True)
        phys_scene.CreateSolverTypeAttr().Set('TGS')
        material = UsdShade.Material.Define(stage, '/World/Material')
        mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        mat.CreateStaticFrictionAttr().Set(.9)
        mat.CreateDynamicFrictionAttr().Set(.75)
        mat.CreateRestitutionAttr().Set(0.)

        def box(path, center, size, dynamic):
            box_prim = UsdGeom.Cube.Define(stage, path)
            box_prim.CreateSizeAttr(1.0)
            transform = UsdGeom.Xformable(box_prim.GetPrim())
            transform.AddTranslateOp(opSuffix='task26_pose').Set(Gf.Vec3d(*center))
            transform.AddScaleOp(opSuffix='task26_size').Set(Gf.Vec3f(*size))
            prim = box_prim.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
            if dynamic:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(.8)
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr().Set(False)
            return prim
        table = box(TABLE, (.55, 0., .1), (1.5, .8, .2), False)
        cube = box(CUBE, (.5, 0., .26), (.12, .12, .12), True)
        cube_phys = PhysxSchema.PhysxRigidBodyAPI(cube)
        view = RigidContactView(CUBE, [TABLE], name='d1_support', disable_stablization=False, max_contact_count=128)
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control', True):
            raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        sim = omni.physx.get_physx_simulation_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        cube_path = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        handle = dc.get_rigid_body(CUBE)
        if handle == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError('INVALID_CUBE_HANDLE')
        view.initialize()
        if not view.is_physics_handle_valid() or (view.num_shapes, view.num_filters) != (1, 1):
            raise RuntimeError('INVALID_CONTACT_VIEW')
        mass = float(dc.get_rigid_body_properties(handle).mass)
        gravity = xyz(view._physics_sim_view.get_gravity())
        if abs(mass - .8) > .001 or not np.allclose(gravity, [0, 0, -9.81], atol=.001):
            raise RuntimeError('RUNTIME_MASS_GRAVITY_MISMATCH')
        dc.wake_up_rigid_body(handle)
        initial = xyz(dc.get_rigid_body_pose(handle).p)
        state = {'step': 0, 'time_s': 0., 'frame': 0, 'error': None, 'before': None,
                 'before_dt': None, 'pending': [], 'cumulative': np.zeros(3)}
        result.update(cube=CUBE, table=TABLE, runtime_mass_kg=mass, gravity_world_m_s2=gravity,
                      geometry={'cube_m': [.12, .12, .12], 'table_m': [1.5, .8, .2]},
                      friction={'static': .9, 'dynamic': .75, 'restitution': 0},
                      solver=phys_scene.GetSolverTypeAttr().Get(),
                      stabilization=phys_scene.GetEnableStabilizationAttr().Get(),
                      ccd=phys_scene.GetEnableCCDAttr().Get(),
                      body_sleep_threshold=cube_phys.GetSleepThresholdAttr().Get(),
                      body_stabilization_threshold=cube_phys.GetStabilizationThresholdAttr().Get(),
                      linear_damping=cube_phys.GetLinearDampingAttr().Get(),
                      angular_damping=cube_phys.GetAngularDampingAttr().Get(),
                      locked_position_axes=cube_phys.GetLockedPosAxisAttr().Get(),
                      locked_rotation_axes=cube_phys.GetLockedRotAxisAttr().Get(),
                      initial_xyz_m=initial, prims=[str(p.GetPath()) for p in stage.Traverse()],
                      time_steps_per_second=phys_scene.GetTimeStepsPerSecondAttr().Get())

        def pre_step(dt):
            try:
                state['before'] = xyz(dc.get_rigid_body_linear_velocity(handle))
                state['before_dt'] = float(dt)
            except Exception as exc:
                state['error'] = f'PRE_STEP:{exc!r}'

        def report(headers, data):
            try:
                for h in headers:
                    paths = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0, h.collider1)]
                    points = []
                    for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                        d = data[i]
                        points.append({'normal_world': list(d.normal), 'impulse_world_Ns': list(d.impulse),
                                       'position_world_m': list(d.position), 'separation_m': float(d.separation)})
                    event = {'step_at_callback': state['step'], 'wall_ns': time.time_ns(), 'colliders': paths, 'points': points}
                    event_stream.write(json_record(event) + '\n')
                    event_stream.flush()
                    events.append(event)
                    state['pending'].append(event)
            except Exception as exc:
                state['error'] = f'CONTACT_EVENT:{exc!r}'

        def post_step(dt):
            try:
                state['step'] += 1
                state['time_s'] += float(dt)
                if state['before'] is None or abs(state['before_dt'] - dt) > 1e-8:
                    raise RuntimeError('MISSING_OR_MISMATCHED_PRE_STEP')
                contacts = list(state['pending'])
                state['pending'].clear()
                impulse = np.zeros(3)
                for event in contacts:
                    if event['step_at_callback'] != state['step'] - 1:
                        raise RuntimeError('CONTACT_REPORT_STEP_MISMATCH')
                    if CUBE not in event['colliders']:
                        continue
                    sign = 1 if event['colliders'][0] == CUBE else -1
                    impulse += sign * sum((np.asarray(p['impulse_world_Ns']) for p in event['points']), np.zeros(3))
                mat_force = np.asarray(view.get_contact_force_matrix(dt=float(dt)), dtype=float)
                mat_impulse = np.asarray(view.get_contact_force_matrix(dt=1.), dtype=float)
                net = np.asarray(view.get_net_contact_forces(dt=float(dt)), dtype=float)
                if mat_force.shape != (1, 1, 3) or mat_impulse.shape != (1, 1, 3) or net.shape != (1, 3):
                    raise RuntimeError('CONTACT_SHAPE_MISMATCH')
                f, points, counts, starts = view.get_friction_data(dt=float(dt))
                start, count = int(np.asarray(starts)[0, 0]), int(np.asarray(counts)[0, 0])
                friction_force = np.asarray(f, dtype=float)[start:start + count].sum(axis=0)
                normal_f, normal_p, normal_n, distances, ncounts, nstarts = view.get_contact_force_data(dt=float(dt))
                nstart, ncount = int(np.asarray(nstarts)[0, 0]), int(np.asarray(ncounts)[0, 0])
                post_velocity = xyz(dc.get_rigid_body_linear_velocity(handle))
                position = xyz(dc.get_rigid_body_pose(handle).p)
                friction_impulse = friction_force * float(dt)
                accounting = balance(mass, gravity, float(dt), state['before'], post_velocity, impulse, friction_impulse)
                state['cumulative'] += np.asarray(accounting['unexplained_impulse_Ns'])
                row = {'step': state['step'], 'sim_time_s': state['time_s'], 'dt_s': float(dt),
                       'pre_dt_s': state['before_dt'], 'app_update_index': state['frame'], 'wall_ns': time.time_ns(),
                       'runtime_mass_kg': mass, 'gravity_world_m_s2': gravity, 'cube_xyz_m': position,
                       'pre_velocity_world_m_s': state['before'], 'velocity_world_m_s': post_velocity,
                       'angular_velocity_world_rad_s': xyz(dc.get_rigid_body_angular_velocity(handle)),
                       'sleeping': bool(sim.is_sleeping(stage_id, cube_path)), 'contact_event_count': len(contacts),
                       'contact_pairs': [e['colliders'] for e in contacts],
                       'event_normal_impulse_world_Ns': impulse.tolist(),
                       'matrix_force_world_N': mat_force[0, 0].tolist(), 'matrix_impulse_world_Ns': mat_impulse[0, 0].tolist(),
                       'net_normal_force_world_N': net[0].tolist(), 'table_friction_force_world_N': friction_force.tolist(),
                       'friction_anchor_count': count, 'friction_points_world_m': np.asarray(points)[start:start + count].tolist(),
                       'normal_contact_count': ncount, 'normal_forces_N': np.asarray(normal_f)[nstart:nstart + ncount].tolist(),
                       'contact_points_world_m': np.asarray(normal_p)[nstart:nstart + ncount].tolist(),
                       'contact_normals_world': np.asarray(normal_n)[nstart:nstart + ncount].tolist(),
                       'contact_separations_m': np.asarray(distances)[nstart:nstart + ncount].tolist(),
                       **accounting, 'cumulative_unexplained_impulse_Ns': state['cumulative'].tolist()}
                row_stream.write(json_record(row) + '\n')
                row_stream.flush()
                rows.append(row)
                state['before'] = None
                if np.linalg.norm(np.asarray(position) - initial) > .05 or np.linalg.norm(post_velocity) > .5:
                    raise RuntimeError('MOTION_BOUND_EXCEEDED')
            except Exception as exc:
                state['error'] = f'POST_STEP:{exc!r}'

        subs.append(sim.subscribe_contact_report_events(report))
        physx = omni.physx.get_physx_interface()
        subs.append(physx.subscribe_physics_on_step_events(pre_step, True, 0))
        subs.append(physx.subscribe_physics_on_step_events(post_step, False, 0))
        start = time.monotonic()
        while len(rows) < 240 and not state['error'] and time.monotonic() - start < 180:
            state['frame'] += 1
            app.update()
        timeline.pause()
        if state['error']:
            raise RuntimeError(state['error'])
        if len(rows) < 240:
            raise RuntimeError('PHYSICS_TIMEOUT')
        result.update(status='RECORDED', reason=None)
    except Exception as exc:
        result['reason'] = repr(exc)
    finally:
        for sub in subs:
            sub.unsubscribe()
        row_stream.close()
        event_stream.close()
        result.update(samples=len(rows), events=len(events))
        with (out / 'summary.json').open('x') as stream:
            stream.write(json_record(result) + '\n')
        print('TASK28_D1', json_record(result), flush=True)
    return 0 if result['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    exit_status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_status)
