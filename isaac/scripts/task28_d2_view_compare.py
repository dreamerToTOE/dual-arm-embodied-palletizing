#!/usr/bin/env python3
"""One configuration in a fresh process for the matched D2 comparison."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from isaac.scripts.task28_task27_one_pass import load_scene
from offline.task28_stage_b_contact.c2_control import json_record
from offline.task28_stage_b_contact.d1_math import balance

CUBE = '/World/Task27/Supply/Cube_01'
TABLE = '/World/Table'
SOURCE = '/World/left_fr3/fr3_link8/side_suction_tool'
TOOL = '/World/Task28C2/Tool'
DIAG = '/World/Task28C2/SleepDiagnostic'


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', choices=['A', 'B'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': True, 'renderer': 'RaytracedLighting',
                         'multi_gpu': False, 'sync_loads': True})
    rows, events, subs = [], [], []
    samples_file = (args.output / 'timeseries.jsonl').open('x')
    event_file = (args.output / 'contact_events.jsonl').open('x')
    summary = {'config': args.config, 'status': 'FAILED', 'reason': 'not_started',
               'force_commands': 0, 'cube_state_writes_during_measurement': 0}
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdUtils
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        UsdGeom.Xform.Define(stage, '/World')
        load_scene()
        cube = stage.GetPrimAtPath(CUBE)
        table = stage.GetPrimAtPath(TABLE)
        if not cube.IsValid() or not table.IsValid() or not stage.GetPrimAtPath(SOURCE).IsValid():
            raise RuntimeError('ORIGINAL_SCENE_MISSING')
        cube.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(.5, 0., .26))
        cube_phys = PhysxSchema.PhysxRigidBodyAPI(cube)
        cube_phys.CreateDisableGravityAttr().Set(False)
        UsdGeom.Xform.Define(stage, '/World/Task28C2')
        layer = stage.GetRootLayer()
        if not Sdf.CopySpec(layer, Sdf.Path(SOURCE), layer, Sdf.Path(TOOL)):
            raise RuntimeError('TOOL_COPY_FAILED')
        if not Sdf.CopySpec(layer, Sdf.Path(CUBE), layer, Sdf.Path(DIAG)):
            raise RuntimeError('DIAGNOSTIC_CUBE_COPY_FAILED')
        diag = stage.GetPrimAtPath(DIAG)
        diag.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(-1.5, -.3, -5.))
        diag_phys = PhysxSchema.PhysxRigidBodyAPI(diag)
        diag_phys.CreateDisableGravityAttr().Set(True)
        diag_phys.CreateLinearDampingAttr().Set(0.)
        tool = stage.GetPrimAtPath(TOOL)
        tx = UsdGeom.Xformable(tool)
        tx.AddTranslateOp().Set(Gf.Vec3d(.5, -.218, .34))
        tx.AddOrientOp().Set(Gf.Quatf(0., Gf.Vec3f(1., 0., 0.)))
        UsdPhysics.RigidBodyAPI.Apply(tool)
        UsdPhysics.MassAPI.Apply(tool).CreateMassAttr().Set(1.)
        tool_phys = PhysxSchema.PhysxRigidBodyAPI.Apply(tool)
        tool_phys.CreateDisableGravityAttr().Set(True)
        tool_phys.CreateLockedPosAxisAttr().Set(5)
        tool_phys.CreateLockedRotAxisAttr().Set(7)
        shapes = ('vertical_support', 'lateral_support', 'vacuum_manifold',
                  'cup_r1_c1', 'cup_r1_c2', 'cup_r2_c1', 'cup_r2_c2')
        if (not cube.HasAPI(UsdPhysics.CollisionAPI) or not table.HasAPI(UsdPhysics.CollisionAPI) or
                any(not stage.GetPrimAtPath(f'{TOOL}/{name}').HasAPI(UsdPhysics.CollisionAPI) for name in shapes)):
            raise RuntimeError('COLLISION_GEOMETRY_MISSING')
        for side in ('left', 'right'):
            if not stage.GetPrimAtPath(f'/World/{side}_fr3').SetActive(False):
                raise RuntimeError('FR3_DEACTIVATION_FAILED')
        # Match the tool's contact reporter setup in both configurations.
        PhysxSchema.PhysxContactReportAPI.Apply(tool).CreateThresholdAttr().Set(0)
        tool_view = (RigidContactView(CUBE, [TOOL], name='d2_cube_tool', disable_stablization=False, max_contact_count=128)
                     if args.config == 'B' else None)
        table_view = RigidContactView(CUBE, [TABLE], name='d2_cube_table', disable_stablization=False, max_contact_count=128)
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control', True):
            raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        sim = omni.physx.get_physx_simulation_interface()
        physx = omni.physx.get_physx_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        cube_path = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        state = {'step': 0, 'sim_s': 0., 'frame': 0, 'error': None, 'before': None,
                 'events': [], 'early': None, 'warmup_steps': 0, 'observing': False}

        def warmup(dt):
            state['warmup_steps'] += 1

        def contact(headers, data):
            if not state['observing']:
                return
            try:
                for h in headers:
                    paths = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0, h.collider1)]
                    if CUBE not in paths and not any(p == TOOL or p.startswith(TOOL + '/') for p in paths):
                        continue
                    points = []
                    for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                        d = data[i]
                        points.append({'impulse_world_Ns': list(d.impulse), 'normal_world': list(d.normal),
                                       'position_world_m': list(d.position), 'separation_m': float(d.separation)})
                    event = {'step_before_post': state['step'], 'monotonic_ns': time.monotonic_ns(),
                             'paths': paths, 'points': points}
                    event_file.write(json_record(event) + '\n')
                    event_file.flush()
                    events.append(event)
                    state['events'].append(event)
            except Exception as exc:
                state['error'] = f'CONTACT_REPORT:{exc!r}'

        subs.append(sim.subscribe_contact_report_events(contact))
        warmup_sub = physx.subscribe_physics_on_step_events(warmup, False, 0)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        warmup_sub.unsubscribe()
        cube_h, tool_h, diag_h = (dc.get_rigid_body(p) for p in (CUBE, TOOL, DIAG))
        if any(h == _dynamic_control.INVALID_HANDLE for h in (cube_h, tool_h, diag_h)):
            raise RuntimeError('INVALID_RIGID_BODY_HANDLE')
        if tool_view:
            tool_view.initialize()
        table_view.initialize()
        views = [table_view] + ([tool_view] if tool_view else [])
        if any(not v.is_physics_handle_valid() or (v.num_shapes, v.num_filters) != (1, 1) for v in views):
            raise RuntimeError('INVALID_CONTACT_VIEW')
        mass = float(dc.get_rigid_body_properties(cube_h).mass)
        gravity = xyz(table_view._physics_sim_view.get_gravity())
        if abs(mass - .8) > .001 or not np.allclose(gravity, [0, 0, -9.81], atol=.001):
            raise RuntimeError('MASS_OR_GRAVITY_MISMATCH')
        dc.wake_up_rigid_body(diag_h)
        dc.set_rigid_body_linear_velocity(diag_h, (0, .0182887595, 0))
        dc.wake_up_rigid_body(cube_h)
        dc.wake_up_rigid_body(tool_h)
        initial_cube = xyz(dc.get_rigid_body_pose(cube_h).p)
        initial_tool = xyz(dc.get_rigid_body_pose(tool_h).p)
        initial_cube_v = xyz(dc.get_rigid_body_linear_velocity(cube_h))
        initial_cube_q = dc.get_rigid_body_pose(cube_h).r
        summary.update(runtime_mass_kg=mass, gravity_world_m_s2=gravity, warmup_app_updates=90,
                       warmup_physics_steps=state['warmup_steps'], initial_cube_xyz_m=initial_cube,
                       initial_cube_velocity_m_s=initial_cube_v,
                       initial_cube_quaternion_xyzw=[float(initial_cube_q.x), float(initial_cube_q.y), float(initial_cube_q.z), float(initial_cube_q.w)],
                       initial_tool_xyz_m=initial_tool, fr3_deactivated_in_diagnostic=True,
                       view_order=['tool', 'table'] if tool_view else ['table'],
                       solver=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/physicsScene')).GetSolverTypeAttr().Get(),
                       stabilization=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/physicsScene')).GetEnableStabilizationAttr().Get())
        state['observing'] = True

        def pre(dt):
            try:
                if state['step'] >= 60:
                    return
                state['before'] = {'velocity': xyz(dc.get_rigid_body_linear_velocity(cube_h)),
                                   'dt': float(dt), 'monotonic_ns': time.monotonic_ns()}
            except Exception as exc:
                state['error'] = f'PRE:{exc!r}'

        def matrix(view, dt):
            value = np.asarray(view.get_contact_force_matrix(dt=dt), dtype=float)
            if value.shape != (1, 1, 3) or not np.isfinite(value).all():
                raise RuntimeError('INVALID_MATRIX')
            return value[0, 0]

        def early(dt):
            try:
                if state['step'] >= 60:
                    return
                state['step'] += 1
                state['sim_s'] += float(dt)
                if state['before'] is None or abs(state['before']['dt'] - dt) > 1e-8:
                    raise RuntimeError('PRE_STEP_MISMATCH')
                es = list(state['events'])
                state['events'].clear()
                if any(e['step_before_post'] != state['step'] - 1 for e in es):
                    raise RuntimeError('EVENT_STEP_MISMATCH')
                table_impulse = np.zeros(3)
                other_impulse = np.zeros(3)
                for e in es:
                    if CUBE not in e['paths']:
                        continue
                    sign = 1 if e['paths'][0] == CUBE else -1
                    impulse = sign * sum((np.asarray(p['impulse_world_Ns']) for p in e['points']), np.zeros(3))
                    if TABLE in e['paths']:
                        table_impulse += impulse
                    else:
                        other_impulse += impulse
                # Preserve C2R read order: tool view first, then table view.
                tool_force = matrix(tool_view, float(dt)).tolist() if tool_view else None
                support = matrix(table_view, float(dt))
                support_raw = matrix(table_view, 1.)
                net = np.asarray((tool_view or table_view).get_net_contact_forces(dt=float(dt)), dtype=float)
                if net.shape != (1, 3) or not np.isfinite(net).all():
                    raise RuntimeError('INVALID_NET_MATRIX')
                ff, _, counts, starts = table_view.get_friction_data(dt=float(dt))
                i, n = int(np.asarray(starts)[0, 0]), int(np.asarray(counts)[0, 0])
                friction = np.asarray(ff)[i:i+n].sum(axis=0)
                cp, tp = dc.get_rigid_body_pose(cube_h).p, dc.get_rigid_body_pose(tool_h).p
                cv, tv = dc.get_rigid_body_linear_velocity(cube_h), dc.get_rigid_body_linear_velocity(tool_h)
                result = balance(mass, gravity, float(dt), state['before']['velocity'], xyz(cv),
                                 table_impulse + other_impulse, friction * dt)
                state['early'] = {'step': state['step'], 'sim_time_s': state['sim_s'], 'dt_s': float(dt),
                                  'app_update_index': state['frame'], 'pre_monotonic_ns': state['before']['monotonic_ns'],
                                  'early_monotonic_ns': time.monotonic_ns(), 'event_monotonic_ns': [e['monotonic_ns'] for e in es],
                                  'cube_xyz_m': xyz(cp), 'cube_velocity_m_s': xyz(cv),
                                  'pre_cube_velocity_m_s': state['before']['velocity'], 'tool_xyz_m': xyz(tp),
                                  'tool_velocity_m_s': xyz(tv), 'sleeping': bool(sim.is_sleeping(stage_id, cube_path)),
                                  'support_force_N': support.tolist(), 'support_impulse_Ns': support_raw.tolist(),
                                  'net_normal_force_N': net[0].tolist(),
                                  'tool_force_N': tool_force, 'event_table_impulse_Ns': table_impulse.tolist(),
                                  'event_other_impulse_Ns': other_impulse.tolist(), 'table_friction_force_N': friction.tolist(),
                                  'contact_pairs': [e['paths'] for e in es], 'residual_Ns': result['unexplained_impulse_Ns'],
                                  'gravity_impulse_Ns': result['gravity_impulse_Ns'],
                                  'momentum_change_Ns': result['momentum_change_Ns']}
                state['before'] = None
                if np.linalg.norm(np.array(xyz(tp)) - initial_tool) > .001 or np.linalg.norm(xyz(tv)) > .01:
                    raise RuntimeError('TOOL_MOVED')
                if np.linalg.norm(np.array(xyz(cp)) - initial_cube) > .01 or np.linalg.norm(xyz(cv)) > .05:
                    raise RuntimeError('CUBE_MOTION_BOUND')
            except Exception as exc:
                state['error'] = f'EARLY:{exc!r}'

        def late(dt):
            try:
                row = state['early']
                if row is None or row['step'] != state['step']:
                    return
                # Reverse read order to test timing/order inside the same physics step.
                row['late_support_force_N'] = matrix(table_view, float(dt)).tolist()
                row['late_tool_force_N'] = matrix(tool_view, float(dt)).tolist() if tool_view else None
                row['late_monotonic_ns'] = time.monotonic_ns()
                samples_file.write(json_record(row) + '\n')
                samples_file.flush()
                rows.append(row)
                state['early'] = None
                if len(rows) == 60:
                    state['observing'] = False
            except Exception as exc:
                state['error'] = f'LATE:{exc!r}'

        subs.extend([physx.subscribe_physics_on_step_events(pre, True, 0),
                     physx.subscribe_physics_on_step_events(early, False, 0),
                     physx.subscribe_physics_on_step_events(late, False, 900)])
        start = time.monotonic()
        while len(rows) < 60 and not state['error'] and time.monotonic() - start < 120:
            state['frame'] += 1
            app.update()
        timeline.pause()
        if state['error']:
            raise RuntimeError(state['error'])
        if len(rows) != 60:
            raise RuntimeError('INCOMPLETE_WINDOW')
        summary.update(status='RECORDED', reason=None)
    except Exception as exc:
        summary['reason'] = repr(exc)
    finally:
        for sub in subs:
            sub.unsubscribe()
        samples_file.close()
        event_file.close()
        summary.update(samples=len(rows), events=len(events))
        (args.output / 'summary.json').write_text(json_record(summary) + '\n')
        print('TASK28_D2_CONFIG', json_record(summary), flush=True)
    return 0 if summary['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)
