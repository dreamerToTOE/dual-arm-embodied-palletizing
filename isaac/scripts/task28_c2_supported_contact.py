#!/usr/bin/env python3
"""One supported, guarded contact-force experiment in the Task27 scene."""
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
from offline.task28_stage_b_contact.c2_control import Controller, json_record
from offline.task28_stage_b_contact.c2_evidence import EvidenceWriter
from offline.task28_stage_b_contact.c2_preflight import SupportPreflight

CUBE = '/World/Task27/Supply/Cube_01'
TABLE = '/World/Table'
SOURCE_TOOL = '/World/left_fr3/fr3_link8/side_suction_tool'
TOOL = '/World/Task28C2/Tool'
DIAG = '/World/Task28C2/SleepDiagnostic'


def xyz(value):
    return [float(value.x), float(value.y), float(value.z)]


def attr(prim, name):
    value = prim.GetAttribute(name)
    return value.Get() if value else None


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
    rows, events, warmup_events, subs = [], [], [], []
    writer = EvidenceWriter(out)
    summary = {'status': 'FAILED', 'reason': 'not_started', 'preflight': 'NOT_STARTED', 'cube': CUBE, 'tool': TOOL,
               'table': TABLE, 'full_fr3_execution': False, 'cube_pose_writes_during_contact': 0}
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Gf, PhysxSchema, PhysicsSchemaTools, Sdf, UsdGeom, UsdPhysics, UsdUtils
        from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, '/World')
        load_scene()
        cube = stage.GetPrimAtPath(CUBE)
        table = stage.GetPrimAtPath(TABLE)
        source = stage.GetPrimAtPath(SOURCE_TOOL)
        if not all(p.IsValid() for p in (cube, table, source)):
            raise RuntimeError('ORIGINAL_SCENE_PRIM_MISSING')
        # One initial scene placement, before physics starts; no Cube position writes during the experiment.
        cube.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(.5, 0., .26))
        cube_phys = PhysxSchema.PhysxRigidBodyAPI(cube)
        cube_phys.CreateDisableGravityAttr().Set(False)
        UsdGeom.Xform.Define(stage, '/World/Task28C2')
        layer = stage.GetRootLayer()
        if not Sdf.CopySpec(layer, Sdf.Path(SOURCE_TOOL), layer, Sdf.Path(TOOL)):
            raise RuntimeError('ORIGINAL_TOOL_COPY_FAILED')
        if not Sdf.CopySpec(layer, Sdf.Path(CUBE), layer, Sdf.Path(DIAG)):
            raise RuntimeError('DIAGNOSTIC_CUBE_COPY_FAILED')
        diag = stage.GetPrimAtPath(DIAG)
        diag.GetAttribute('xformOp:translate:task26_pose').Set(Gf.Vec3d(-1.5, -.3, -5.0))
        diag_phys = PhysxSchema.PhysxRigidBodyAPI(diag)
        diag_phys.CreateDisableGravityAttr().Set(True)
        diag_phys.CreateLinearDampingAttr().Set(0.0)
        tool = stage.GetPrimAtPath(TOOL)
        xform = UsdGeom.Xformable(tool)
        xform.AddTranslateOp().Set(Gf.Vec3d(.5, -.218, .34))
        xform.AddOrientOp().Set(Gf.Quatf(0., Gf.Vec3f(1., 0., 0.)))
        UsdPhysics.RigidBodyAPI.Apply(tool)
        UsdPhysics.MassAPI.Apply(tool).CreateMassAttr().Set(1.0)
        tool_phys = PhysxSchema.PhysxRigidBodyAPI.Apply(tool)
        tool_phys.CreateDisableGravityAttr().Set(True)
        tool_phys.CreateLockedPosAxisAttr().Set(5)  # x and z: prismatic +Y proxy
        tool_phys.CreateLockedRotAxisAttr().Set(7)
        names = ('vertical_support', 'lateral_support', 'vacuum_manifold',
                 'cup_r1_c1', 'cup_r1_c2', 'cup_r2_c1', 'cup_r2_c2')
        if any(not stage.GetPrimAtPath(f'{TOOL}/{name}').HasAPI(UsdPhysics.CollisionAPI) for name in names):
            raise RuntimeError('COPIED_TOOL_COLLISION_GEOMETRY_MISSING')
        for name in names[-4:]:
            cup = UsdGeom.Cylinder(stage.GetPrimAtPath(f'{TOOL}/{name}'))
            if abs(cup.GetRadiusAttr().Get() - .01) > 1e-6 or abs(cup.GetHeightAttr().Get() - .01) > 1e-6:
                raise RuntimeError('COPIED_CUP_DIMENSIONS_MISMATCH')
        view = RigidContactView(CUBE, [TOOL], name='c2_cube_tool',
                                disable_stablization=False, max_contact_count=128)
        support_view = RigidContactView(CUBE, [TABLE], name='c2_cube_table',
                                        disable_stablization=False, max_contact_count=128)
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control', True):
            raise RuntimeError('DYNAMIC_CONTROL_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        dc = _dynamic_control.acquire_dynamic_control_interface()
        sim = omni.physx.get_physx_simulation_interface()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        diag_path = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(DIAG))
        cube_path = PhysicsSchemaTools.sdfPathToInt(Sdf.Path(CUBE))
        pending = []
        state = {'step': 0, 'time_s': 0.0, 'started': False, 'error': None}

        def report(headers, data):
            for h in headers:
                paths = [str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0, h.collider1)]
                if CUBE not in paths and not any(p == TOOL or p.startswith(TOOL + '/') for p in paths):
                    continue
                points = []
                for i in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    d = data[i]
                    points.append({'normal_world': list(d.normal), 'impulse_world_Ns': list(d.impulse),
                                   'position_world_m': list(d.position), 'separation_m': float(d.separation)})
                row = {'step_at_callback': state['step'], 'wall_ns': time.time_ns(), 'colliders': paths, 'points': points}
                if state['started']:
                    pending.append(row)
                    try:
                        writer.write('event', row)
                        events.append(row)
                    except Exception as exc:
                        state['error'] = f'CONTACT_EVIDENCE_WRITE:{exc!r}'
                else:
                    warmup_events.append(row)

        subs.append(sim.subscribe_contact_report_events(report))
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        def is_tool(path):
            return path == TOOL or path.startswith(TOOL + '/')
        for event in warmup_events:
            paths = event['colliders']
            if any(is_tool(p) for p in paths):
                if any(np.linalg.norm(p['impulse_world_Ns']) > 1e-7 or p['separation_m'] < -1e-4 for p in event['points']):
                    raise RuntimeError('PRECONTACT_PROXY_COLLISION')
            elif CUBE in paths and TABLE not in paths:
                if any(np.linalg.norm(p['impulse_world_Ns']) > 1e-7 for p in event['points']):
                    raise RuntimeError('PRECONTACT_UNEXPECTED_CUBE_COLLISION')
        cube_h, tool_h, diag_h = (dc.get_rigid_body(p) for p in (CUBE, TOOL, DIAG))
        if any(h == _dynamic_control.INVALID_HANDLE for h in (cube_h, tool_h, diag_h)):
            raise RuntimeError('INVALID_PHYSX_HANDLE')
        cube_mass = float(dc.get_rigid_body_properties(cube_h).mass)
        summary['cube_runtime_mass_kg'] = cube_mass
        view.initialize()
        support_view.initialize()
        if (not view.is_physics_handle_valid() or not support_view.is_physics_handle_valid() or
                view.num_shapes != 1 or view.num_filters != 1 or support_view.num_shapes != 1 or support_view.num_filters != 1):
            raise RuntimeError(f'INVALID_PAIR_VIEW:{view.num_shapes}:{view.num_filters}')
        gravity_value = view._physics_sim_view.get_gravity()
        gravity_vector = xyz(gravity_value) if gravity_value is not None else None
        summary['gravity_vector_m_s2'] = gravity_vector
        gravity = float(np.linalg.norm(gravity_vector)) if gravity_vector is not None else 0.0
        summary['gravity_m_s2'] = gravity
        if (not np.isfinite(cube_mass) or abs(cube_mass - .8) > .001 or not np.isfinite(gravity) or gravity <= 0 or
                not np.allclose(np.asarray(gravity_vector, dtype=float) / gravity, [0, 0, -1], atol=1e-6)):
            raise RuntimeError('CUBE_MASS_OR_GRAVITY_NOT_VERIFIED')
        if not cube.HasAPI(UsdPhysics.CollisionAPI) or not table.HasAPI(UsdPhysics.CollisionAPI):
            raise RuntimeError('CUBE_OR_TABLE_COLLISION_DISABLED')
        cube_scale = cube.GetAttribute('xformOp:scale:task26_size').Get()
        table_scale = table.GetAttribute('xformOp:scale:task26_size').Get()
        if (not np.allclose(np.asarray(cube_scale), [.12, .12, .12], atol=1e-6) or
                not np.allclose(np.asarray(table_scale), [1.5, .8, .2], atol=1e-6)):
            raise RuntimeError('CUBE_OR_TABLE_GEOMETRY_MISMATCH')
        dc.wake_up_rigid_body(diag_h)
        dc.set_rigid_body_linear_velocity(diag_h, (0.0, .0182887595, 0.0))
        dc.wake_up_rigid_body(cube_h)
        dc.wake_up_rigid_body(tool_h)
        cube_initial = dc.get_rigid_body_pose(cube_h).p
        tool_initial = dc.get_rigid_body_pose(tool_h).p
        if abs(cube_initial.y) > .002 or abs(cube_initial.z - .26) > .002:
            raise RuntimeError('CUBE_NOT_STABLY_SUPPORTED')
        controller = Controller()
        preflight = SupportPreflight(cube_mass, gravity_vector)
        summary['preflight'] = 'PENDING'
        state['started'] = True
        summary.update(scene='original Task27; proxy is a copied original left tool',
                       warmup_events=len(warmup_events), initial_cube_xyz_m=xyz(cube_initial),
                       initial_tool_xyz_m=xyz(tool_initial),
                       cube_linear_damping=cube_phys.GetLinearDampingAttr().Get(),
                       cube_angular_damping=cube_phys.GetAngularDampingAttr().Get(),
                       cube_sleep_threshold=cube_phys.GetSleepThresholdAttr().Get(),
                       cube_runtime_mass_kg=cube_mass, gravity_m_s2=gravity,
                       cube_stabilization_threshold=cube_phys.GetStabilizationThresholdAttr().Get(),
                       diag_sleep_threshold=diag_phys.GetSleepThresholdAttr().Get(),
                       solver_type=attr(stage.GetPrimAtPath('/physicsScene'), 'physxScene:solverType'),
                       enable_stabilization=attr(stage.GetPrimAtPath('/physicsScene'), 'physxScene:enableStabilization'),
                       cube_scale_m=list(cube_scale), table_scale_m=list(table_scale),
                       cube_contact_offset=attr(cube, 'physxCollision:contactOffset'),
                       cube_rest_offset=attr(cube, 'physxCollision:restOffset'),
                       table_contact_offset=attr(table, 'physxCollision:contactOffset'),
                       table_rest_offset=attr(table, 'physxCollision:restOffset'),
                       physics_material='/World/Task27/HighFrictionMaterial; static=0.90 dynamic=0.75 restitution=0')

        def post_step(dt):
            try:
                if state['error']:
                    controller.fail('evidence_write_failure')
                    return
                started_ns = time.monotonic_ns()
                state['step'] += 1
                state['time_s'] += float(dt)
                step_events = list(pending)
                pending.clear()
                cube_impulse = np.zeros(3)
                pair_impulse = np.zeros(3)
                support_impulse = np.zeros(3)
                unexpected = []
                pair = False
                penetration = 0.0
                for event in step_events:
                    if event['step_at_callback'] != state['step'] - 1:
                        raise RuntimeError('CONTACT_REPORT_STEP_MISMATCH')
                    paths = event['colliders']
                    impulses = sum((np.asarray(p['impulse_world_Ns'], dtype=float) for p in event['points']), np.zeros(3))
                    active = bool(np.linalg.norm(impulses) > 1e-8 or any(p['separation_m'] < -1e-4 for p in event['points']))
                    if CUBE in paths:
                        sign = 1 if paths[0] == CUBE else -1
                        on_cube = sign * impulses
                        cube_impulse += on_cube
                        other = paths[1] if paths[0] == CUBE else paths[0]
                        if is_tool(other):
                            pair_impulse += on_cube
                            pair |= active
                            penetration = max([penetration] + [-p['separation_m'] for p in event['points']])
                        elif other == TABLE:
                            support_impulse += on_cube
                            penetration = max([penetration] + [-p['separation_m'] for p in event['points']])
                        elif active:
                            unexpected.append(paths)
                    elif any(is_tool(p) for p in paths) and active:
                        unexpected.append(paths)
                pair_matrix = np.asarray(view.get_contact_force_matrix(dt=float(dt)), dtype=float)
                support_matrix = np.asarray(support_view.get_contact_force_matrix(dt=float(dt)), dtype=float)
                net_matrix = np.asarray(view.get_net_contact_forces(dt=float(dt)), dtype=float)
                if pair_matrix.shape != (1, 1, 3) or support_matrix.shape != (1, 1, 3) or net_matrix.shape != (1, 3):
                    raise RuntimeError('CONTACT_MATRIX_SHAPE_MISMATCH')
                force, support_force, net_force = pair_matrix[0, 0], support_matrix[0, 0], net_matrix[0]
                friction, _, counts, starts = support_view.get_friction_data(dt=float(dt))
                count, start_index = int(np.asarray(counts)[0, 0]), int(np.asarray(starts)[0, 0])
                friction_force = np.asarray(friction, dtype=float)[start_index:start_index + count].sum(axis=0)
                if not all(np.isfinite(x).all() for x in (force, support_force, net_force, friction_force)):
                    raise RuntimeError('NONFINITE_CONTACT_READING')
                cp, tp, dp = (dc.get_rigid_body_pose(h).p for h in (cube_h, tool_h, diag_h))
                cv, tv, dv = (dc.get_rigid_body_linear_velocity(h) for h in (cube_h, tool_h, diag_h))
                cube_angular, diag_angular = (dc.get_rigid_body_angular_velocity(h) for h in (cube_h, diag_h))
                obs = {'step': state['step'], 'sensor_step': state['step'], 'time_s': state['time_s'],
                       'sensor_time_s': state['time_s'], 'dt_s': float(dt), 'age_ns': time.monotonic_ns() - started_ns,
                       'frame': 'world', 'source': 'physx_pair',
                       'valid': bool(view.is_physics_handle_valid() and support_view.is_physics_handle_valid()),
                       'force_y': float(force[1]), 'force_norm': float(np.linalg.norm(force)),
                       'event_force_y': float(pair_impulse[1] / dt), 'tool_pair': pair,
                       'unexpected_pairs': unexpected, 'tool_y': float(tp.y - tool_initial.y),
                       'cube_y': float(cp.y - cube_initial.y), 'tool_velocity': float(tv.y),
                       'cube_velocity': float(cv.y), 'penetration_m': float(penetration),
                       'tool_displacement': float(np.linalg.norm(np.array(xyz(tp)) - xyz(tool_initial))),
                       'cube_displacement': float(np.linalg.norm(np.array(xyz(cp)) - xyz(cube_initial))),
                       'tool_speed': float(np.linalg.norm(xyz(tv))), 'cube_speed': float(np.linalg.norm(xyz(cv))),
                       'support_force_z': float(support_force[2]), 'support_event_z': float(support_impulse[2] / dt),
                       'expected_weight_N': cube_mass * gravity}
                if summary['preflight'] != 'PASS':
                    result = preflight.update(obs, xyz(cv), cube_impulse)
                    summary['preflight_diagnostics'] = result
                    if result['status'] == 'FAILED':
                        summary['preflight'] = 'FAILED'
                        action = controller.fail(result['reason'])
                    else:
                        if result['status'] == 'PASS':
                            summary['preflight'] = 'PASS'
                            controller.previous_step, controller.previous_time = state['step'], state['time_s']
                        action = {'state': 'PREFLIGHT', 'reference_N': 0.0, 'command_N': 0.0, 'reason': None}
                else:
                    action = controller.update(obs)
                row = {**obs, **action, 'wall_ns': time.time_ns(),
                             'cube_xyz_m': xyz(cp), 'tool_xyz_m': xyz(tp),
                             'cube_velocity_xyz_m_s': xyz(cv), 'tool_velocity_xyz_m_s': xyz(tv),
                             'tool_cube_force_world_N': force.tolist(), 'cube_contact_impulse_world_Ns': cube_impulse.tolist(),
                             'cube_tool_impulse_world_Ns': pair_impulse.tolist(), 'cube_table_impulse_world_Ns': support_impulse.tolist(),
                             'cube_table_force_world_N': support_force.tolist(),
                             'cube_net_contact_force_world_N': net_force.tolist(),
                             'cube_table_friction_force_world_N': friction_force.tolist(),
                             'cube_other_contact_impulse_world_Ns': (cube_impulse - pair_impulse - support_impulse).tolist(),
                             'contact_matrix_shape': list(pair_matrix.shape),
                             'diag_xyz_m': xyz(dp), 'diag_velocity_xyz_m_s': xyz(dv),
                             'cube_angular_velocity_rad_s': xyz(cube_angular), 'diag_angular_velocity_rad_s': xyz(diag_angular),
                             'cube_sleeping': bool(sim.is_sleeping(stage_id, cube_path)),
                             'diag_sleeping': bool(sim.is_sleeping(stage_id, diag_path))}
                writer.write('sample', row)
                rows.append(row)
                dc.apply_body_force(tool_h, (0., action['command_N'], 0.), (float(tp.x), float(tp.y), float(tp.z)), True)
            except Exception as exc:
                state['error'] = repr(exc)
                controller.fail('instrumentation_exception')

        subs.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step, False, 0))
        start = time.monotonic()
        while controller.state not in ('DONE', 'FAILED') and not state['error'] and time.monotonic() - start < 180:
            app.update()
        timeline.pause()
        if state['error']:
            raise RuntimeError(state['error'])
        if controller.state != 'DONE':
            raise RuntimeError(controller.reason or 'PHYSICS_TIMEOUT')
        summary.update(status='RECORDED', reason=None, samples=len(rows), events=len(events))
    except Exception as exc:
        summary['reason'] = repr(exc)
        summary['samples'] = len(rows)
        summary['events'] = len(events)
    finally:
        for sub in subs:
            sub.unsubscribe()
        summary = writer.finish(summary)
        print('TASK28_C2', json.dumps(summary), flush=True)
    return 0 if summary['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    import os
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)
