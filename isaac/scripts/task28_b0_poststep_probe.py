#!/usr/bin/env python3
"""Bounded Cube_01 state-source probe in the original Task27 scene."""
import argparse
import json
import re
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from isaac.scripts.task28_task27_one_pass import load_scene

PATH = '/World/Task27/Supply/Cube_01'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', required=True)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.run_id):
        raise ValueError('invalid run ID')
    out = ROOT / 'offline/task28_task27_integration/evidence' / args.run_id
    out.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'create_new_stage': True, 'renderer': 'RaytracedLighting',
                         'multi_gpu': False, 'sync_loads': True})
    rows = []
    sub = None
    result = {'status': 'FAILED', 'cube': PATH, 'scene': 'original Task27',
              'physical_cycle': False, 'motion': 'bounded velocity impulse in parked supply area'}
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import Usd, UsdGeom
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from isaac.scripts.task28_task27_torque_bridge import make_b0_snapshot
        manager = omni.kit.app.get_app().get_extension_manager()
        if not manager.set_extension_enabled_immediate('omni.isaac.dynamic_control', True):
            raise RuntimeError('DYNAMIC_CONTROL_EXTENSION_UNAVAILABLE')
        from omni.isaac.dynamic_control import _dynamic_control
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, '/World')
        load_scene()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(90):
            app.update()
        rigid = SingleRigidPrim(PATH, name='task28_b0_poststep_probe')
        rigid.initialize()
        arms = {side: SingleArticulation(f'/World/{side}_fr3') for side in ('left', 'right')}
        for arm in arms.values():
            arm.initialize()
        dc = _dynamic_control.acquire_dynamic_control_interface()
        handle = dc.get_rigid_body(PATH)
        if handle == _dynamic_control.INVALID_HANDLE or not rigid._prim_view.is_physics_handle_valid():
            raise RuntimeError('PHYSX_HANDLE_INVALID_OR_USD_FALLBACK')
        prim = stage.GetPrimAtPath(PATH)
        if not prim.IsValid():
            raise RuntimeError('CUBE_PRIM_INVALID')
        initial = dc.get_rigid_body_pose(handle).p
        x0 = float(initial.x)
        state = {'step': 0, 'time': 0.0, 'error': None}
        def post_step(dt):
            try:
                state['step'] += 1
                state['time'] += float(dt)
                valid = bool(rigid._prim_view.is_physics_handle_valid())
                if not valid:
                    raise RuntimeError('PHYSX_HANDLE_INVALID_OR_USD_FALLBACK')
                snapshot = make_b0_snapshot(rigid, arms, stage, state['step'], state['time'])
                tensor_p, tensor_q = snapshot['cube_xyz_m'], snapshot['cube_quaternion_wxyz']
                p = dc.get_rigid_body_pose(handle)
                v = dc.get_rigid_body_linear_velocity(handle)
                trans = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                u = trans.ExtractTranslation()
                row = {'step': state['step'], 'simulation_time_s': state['time'], 'dt_s': float(dt),
                       'wall_ns': time.time_ns(), 'physics_handle_valid': valid, 'snapshot': snapshot,
                       'tensor_xyz': np.asarray(tensor_p, dtype=float).tolist(),
                       'tensor_quaternion_wxyz': np.asarray(tensor_q, dtype=float).tolist(),
                       'dynamic_xyz': [float(p.p.x), float(p.p.y), float(p.p.z)],
                       'dynamic_quaternion_xyzw': [float(p.r.x), float(p.r.y), float(p.r.z), float(p.r.w)],
                       'usd_xyz': [float(u[0]), float(u[1]), float(u[2])],
                       'dynamic_velocity_xyz': [float(v.x), float(v.y), float(v.z)]}
                rows.append(row)
                if abs(float(p.p.x) - x0) > 0.04:
                    state['error'] = 'MOTION_BOUND_EXCEEDED'
            except Exception as exc:
                state['error'] = repr(exc)
        sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step, False, 0)
        dc.wake_up_rigid_body(handle)
        dc.set_rigid_body_linear_velocity(handle, (0.02, 0.0, 0.0))
        start = time.monotonic()
        while len(rows) < 100 and not state['error'] and time.monotonic() - start < 120:
            app.update()
        dc.set_rigid_body_linear_velocity(handle, (0.0, 0.0, 0.0))
        if state['error']:
            raise RuntimeError(state['error'])
        if len(rows) < 30:
            raise RuntimeError(f'INSUFFICIENT_POST_STEPS:{len(rows)}')
        tensor = np.array([r['tensor_xyz'] for r in rows])
        dynamic = np.array([r['dynamic_xyz'] for r in rows])
        usd = np.array([r['usd_xyz'] for r in rows])
        result.update(status='RECORDED', samples=len(rows),
                      duration_sim_s=rows[-1]['simulation_time_s']-rows[0]['simulation_time_s'],
                      displacement_dynamic_m=float(dynamic[-1, 0]-dynamic[0, 0]),
                      max_tensor_dynamic_m=float(np.max(np.abs(tensor-dynamic))),
                      max_usd_dynamic_m=float(np.max(np.abs(usd-dynamic))),
                      physics_handle_valid_all=all(r['physics_handle_valid'] for r in rows),
                      max_snapshot_capture_ms=max((r['snapshot']['sample_finished_ns'] - r['snapshot']['sample_started_ns']) / 1e6 for r in rows),
                      independent_api='omni.isaac.dynamic_control.get_rigid_body_pose',
                      tensor_api='SingleRigidPrim.get_world_pose -> RigidPrim physics tensor get_transforms')
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if sub is not None:
            sub.unsubscribe()
        with (out / 'poststep.jsonl').open('x') as f:
            for row in rows:
                f.write(json.dumps(row, allow_nan=False) + '\n')
        with (out / 'summary.json').open('x') as f:
            json.dump(result, f, indent=2, allow_nan=False)
            f.write('\n')
        print('TASK28_POSTSTEP_PROBE', json.dumps(result), flush=True)
        app.close()
    return 0 if result['status'] == 'RECORDED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
