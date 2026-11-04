#!/usr/bin/env python3
"""One ROS-free D4 scene; wait for both preflights before sampling."""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from offline.task28_stage_b_contact.c2_control import json_record
from offline.task28_stage_b_contact.d1_math import balance
from offline.task28_stage_b_contact.d4_scene import diagnostic_scene_code

CUBE = '/World/Task27/Supply/Cube_01'
TABLE = '/World/Table'
SOURCE = '/World/left_fr3/fr3_link8/side_suction_tool'
TOOL = '/World/Task28C2/Tool'
DIAG = '/World/Task28C2/SleepDiagnostic'


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def mass_record(value):
    return {'mass': float(value['mass']), 'inertia': xyz(value['inertia']), 'com': xyz(value['com'])}


def validate_material_bindings(stage, prims, material_api):
    for prim in prims:
        targets = prim.GetRelationship('material:binding').GetTargets()
        if not targets or any(not stage.GetPrimAtPath(p).HasAPI(material_api) for p in targets):
            raise RuntimeError(f'PHYSICS_MATERIAL_BINDING_INVALID:{prim.GetPath()}:{targets}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', choices=['C', 'D'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gate-dir', type=Path, required=True)
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
        scene_path = ROOT / 'isaac/scripts/task26_truck_box_scene.py'
        namespace = {'__name__': '__task28_d4_scene__', '_SIDE_SUCTION_SCENARIO': 'task27'}
        exec(diagnostic_scene_code(scene_path.read_text(encoding='utf-8'), str(scene_path)), namespace, namespace)
        if stage.GetPrimAtPath('/World/Task27ROSGraph').IsValid():
            raise RuntimeError('UNEXPECTED_ROS_GRAPH')
        if omni.kit.app.get_app().get_extension_manager().is_extension_enabled('isaacsim.ros2.bridge'):
            raise RuntimeError('UNEXPECTED_ROS_BRIDGE')
        cube = stage.GetPrimAtPath(CUBE)
        table = stage.GetPrimAtPath(TABLE)
        if not cube.IsValid() or not table.IsValid() or not stage.GetPrimAtPath(SOURCE).IsValid():
            raise RuntimeError('ORIGINAL_SCENE_MISSING')
        validate_material_bindings(stage, (cube, table), UsdPhysics.MaterialAPI)
        cube.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(.5, 0., .26))
        cube_phys = PhysxSchema.PhysxRigidBodyAPI(cube)
        cube_phys.CreateDisableGravityAttr().Set(False)
        UsdGeom.Xform.Define(stage, '/World/Task28C2')
        layer = stage.GetRootLayer()
        has_tool = args.config != 'C'
        if has_tool and not Sdf.CopySpec(layer, Sdf.Path(SOURCE), layer, Sdf.Path(TOOL)):
            raise RuntimeError('TOOL_COPY_FAILED')
        if not Sdf.CopySpec(layer, Sdf.Path(CUBE), layer, Sdf.Path(DIAG)):
            raise RuntimeError('DIAGNOSTIC_CUBE_COPY_FAILED')
        diag = stage.GetPrimAtPath(DIAG)
        diag.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(-1.5, -.3, -5.))
        diag_phys = PhysxSchema.PhysxRigidBodyAPI(diag)
        diag_phys.CreateDisableGravityAttr().Set(True)
        diag_phys.CreateLinearDampingAttr().Set(0.)
        tool = stage.GetPrimAtPath(TOOL) if has_tool else None
        if has_tool:
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
                (has_tool and any(not stage.GetPrimAtPath(f'{TOOL}/{name}').HasAPI(UsdPhysics.CollisionAPI) for name in shapes))):
            raise RuntimeError('COLLISION_GEOMETRY_MISSING')
        separation = {}
        if has_tool:
            cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
            cb = cache.ComputeWorldBound(cube).ComputeAlignedRange()
            for name in shapes:
                tb = cache.ComputeWorldBound(stage.GetPrimAtPath(f'{TOOL}/{name}')).ComputeAlignedRange()
                gap = max(float(max(tb.GetMin()[i] - cb.GetMax()[i], cb.GetMin()[i] - tb.GetMax()[i])) for i in range(3))
                separation[name] = gap
                if gap <= 0:
                    raise RuntimeError(f'TOOL_NOT_SEPARATED:{name}:{gap}')
        for side in ('left', 'right'):
            if not stage.GetPrimAtPath(f'/World/{side}_fr3').SetActive(False):
                raise RuntimeError('FR3_DEACTIVATION_FAILED')
        # Match the tool's contact reporter setup in both configurations.
        tool_view = None
        table_view = RigidContactView(CUBE, [TABLE], name='d2_cube_table', disable_stablization=False, max_contact_count=128)
        material_path = '/World/Task27/HighFrictionMaterial'
        material_prim = stage.GetPrimAtPath(material_path)
        material_api = UsdPhysics.MaterialAPI(material_prim)
        material_values = [float(material_api.GetStaticFrictionAttr().Get()),
                           float(material_api.GetDynamicFrictionAttr().Get()),
                           float(material_api.GetRestitutionAttr().Get())]
        if not np.allclose(material_values, [.9, .75, 0.], atol=1e-6):
            raise RuntimeError(f'WRONG_AUTHORED_MATERIAL:{material_values}')
        authored = {path: {'attributes': {a.GetName(): str(a.Get()) for a in stage.GetPrimAtPath(path).GetAttributes()},
                           'relationships': {r.GetName(): [str(t) for t in r.GetTargets()] for r in stage.GetPrimAtPath(path).GetRelationships()}}
                    for path in (CUBE, TABLE, '/physicsScene', material_path)}
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
                 'events': [], 'early': None, 'warmup_steps': 0, 'observing': False, 'warmup_pairs': set()}

        def warmup(dt):
            state['warmup_steps'] += 1

        def contact(headers, data):
            try:
                for h in headers:
                    paths = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0, h.collider1)]
                    if not state['observing']:
                        if CUBE in paths:
                            state['warmup_pairs'].add(tuple(paths))
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
        while state['warmup_steps'] < 52:
            app.update()
        warmup_sub.unsubscribe()
        cube_h, diag_h = (dc.get_rigid_body(p) for p in (CUBE, DIAG))
        tool_h = dc.get_rigid_body(TOOL) if has_tool else None
        if any(h == _dynamic_control.INVALID_HANDLE for h in (cube_h, diag_h)) or (has_tool and tool_h == _dynamic_control.INVALID_HANDLE):
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
        unit = omni.physx.get_physxunittests_interface()
        material_paths = {p: list(unit.get_materials_paths(p)) for p in (CUBE, TABLE)}
        if any(paths != [material_path] for paths in material_paths.values()):
            raise RuntimeError(f'PHYSX_MATERIAL_PATH_MISMATCH:{material_paths}')
        body_view = table_view._physics_sim_view.create_rigid_body_view(CUBE)
        effective = np.asarray(body_view.get_material_properties(), dtype=float)
        if effective.shape != (1, 1, 3) or not np.allclose(effective[0, 0], material_values, atol=1e-6):
            raise RuntimeError(f'PHYSX_MATERIAL_VALUES_MISMATCH:{effective}')
        force_check = np.asarray(table_view.get_contact_force_matrix(dt=1. / 60.), dtype=float)
        if force_check.shape != (1, 1, 3) or not np.isfinite(force_check).all() or not any(TABLE in p for p in state['warmup_pairs']):
            raise RuntimeError('CONTACT_REPORT_NOT_OPERATIONAL')
        initial_cube = xyz(dc.get_rigid_body_pose(cube_h).p)
        initial_tool = xyz(dc.get_rigid_body_pose(tool_h).p) if has_tool else None
        initial_cube_v = xyz(dc.get_rigid_body_linear_velocity(cube_h))
        initial_cube_q = dc.get_rigid_body_pose(cube_h).r
        summary.update(runtime_mass_kg=mass, gravity_world_m_s2=gravity, warmup_app_updates=None,
                       warmup_physics_steps=state['warmup_steps'], initial_cube_xyz_m=initial_cube,
                       initial_cube_velocity_m_s=initial_cube_v,
                       initial_cube_quaternion_xyzw=[float(initial_cube_q.x), float(initial_cube_q.y), float(initial_cube_q.z), float(initial_cube_q.w)],
                       initial_tool_xyz_m=initial_tool, fr3_deactivated_in_diagnostic=True,
                       view_order=['tool', 'table'] if tool_view else ['table'],
                       solver=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/physicsScene')).GetSolverTypeAttr().Get(),
                       stabilization=PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath('/physicsScene')).GetEnableStabilizationAttr().Get())
        timeline.pause()
        summary.update(authored=authored, material_paths=material_paths, effective_material=effective.tolist(),
                       cube_mass_properties=mass_record(unit.get_mass_information(CUBE)), tool_separation_m=separation,
                       warmup_pairs=sorted([list(p) for p in state['warmup_pairs']]),
                       initial_cube_angular_velocity_rad_s=xyz(dc.get_rigid_body_angular_velocity(cube_h)),
                       initial_cube_sleeping=bool(sim.is_sleeping(stage_id, cube_path)),
                       ros_graph_absent=True, tool_shapes=list(shapes) if has_tool else [])
        temporary = args.output / 'preflight.tmp'
        temporary.write_text(json_record(summary) + '\n')
        temporary.replace(args.output / 'preflight.json')
        print('D4_PREFLIGHT_READY', args.config, flush=True)
        deadline = time.monotonic() + 180
        while not (args.gate_dir / 'go').exists() and not (args.gate_dir / 'stop').exists() and time.monotonic() < deadline:
            time.sleep(.1)
        if not (args.gate_dir / 'go').exists():
            raise RuntimeError('PREFLIGHT_NOT_APPROVED')
        dc.wake_up_rigid_body(cube_h)
        if has_tool:
            dc.wake_up_rigid_body(tool_h)
        timeline.play()
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
                cp = dc.get_rigid_body_pose(cube_h).p
                cv = dc.get_rigid_body_linear_velocity(cube_h)
                tp = dc.get_rigid_body_pose(tool_h).p if has_tool else None
                tv = dc.get_rigid_body_linear_velocity(tool_h) if has_tool else None
                result = balance(mass, gravity, float(dt), state['before']['velocity'], xyz(cv),
                                 table_impulse + other_impulse, friction * dt)
                state['early'] = {'step': state['step'], 'sim_time_s': state['sim_s'], 'dt_s': float(dt),
                                  'app_update_index': state['frame'], 'pre_monotonic_ns': state['before']['monotonic_ns'],
                                  'early_monotonic_ns': time.monotonic_ns(), 'event_monotonic_ns': [e['monotonic_ns'] for e in es],
                                  'cube_xyz_m': xyz(cp), 'cube_velocity_m_s': xyz(cv),
                                  'cube_angular_velocity_rad_s': xyz(dc.get_rigid_body_angular_velocity(cube_h)),
                                  'cube_quaternion_xyzw': [float(getattr(dc.get_rigid_body_pose(cube_h).r, k)) for k in ('x', 'y', 'z', 'w')],
                                  'pre_cube_velocity_m_s': state['before']['velocity'], 'tool_xyz_m': xyz(tp) if has_tool else None,
                                  'tool_velocity_m_s': xyz(tv) if has_tool else None, 'sleeping': bool(sim.is_sleeping(stage_id, cube_path)),
                                  'support_force_N': support.tolist(), 'support_impulse_Ns': support_raw.tolist(),
                                  'net_normal_force_N': net[0].tolist(),
                                  'tool_force_N': tool_force, 'event_table_impulse_Ns': table_impulse.tolist(),
                                  'event_other_impulse_Ns': other_impulse.tolist(), 'table_friction_force_N': friction.tolist(),
                                  'contact_pairs': [e['paths'] for e in es], 'residual_Ns': result['unexplained_impulse_Ns'],
                                  'gravity_impulse_Ns': result['gravity_impulse_Ns'],
                                  'momentum_change_Ns': result['momentum_change_Ns']}
                state['before'] = None
                if has_tool and (np.linalg.norm(np.array(xyz(tp)) - initial_tool) > .001 or np.linalg.norm(xyz(tv)) > .01):
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
    try:
        status = main()
    except BaseException:
        import traceback
        traceback.print_exc()
        status = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)
