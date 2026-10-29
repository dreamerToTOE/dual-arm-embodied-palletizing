"""Read-only composed USD and PhysX body-property inspection in original scene.

No joint state, drive gain, effort, mass, inertia, or collision property is set.
The frozen builder creates an anonymous stage; its ROS graph is deactivated
before native physics initialization. SimulationContext.play may perform an
internal handle-propagation step; callbacks and explicit steps are reported.
"""
from __future__ import annotations
import argparse
import importlib.util
import json
import os
import traceback
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r2.common import FIXTURE, FR3_ASSET, FR3_SHA256, LEFT_ROOT, RIGHT_ROOT, TASK_ROOT, sha256


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    out = {'schema': 'TASK28_G2_P5_QP_6_R7_TOOL_INERTIA_INSPECTION_V2',
           'status': 'INITIALIZING', 'explicit_physics_steps': 0,
           'effort_writes': 0, 'joint_state_writes': 0, 'mass_inertia_writes': 0,
           'tool_mass_contribution': 'UNKNOWN', 'errors': [], 'records': [],
           'runtime_bodies': {}, 'physics_callback_dts_s': []}
    app = None
    subscription = None

    def save():
        args.output.write_text(json.dumps(out, indent=2, sort_keys=True, allow_nan=False) + '\n')

    def serial(value):
        if value is None or isinstance(value, (str, bool, int, float)):
            return value
        try:
            return np.asarray(value).tolist()
        except (TypeError, ValueError):
            return str(value)

    os.environ['TASK28_FR3_USD'] = str(FR3_ASSET)
    os.environ['TASK28_FR3_USD_SHA256'] = FR3_SHA256
    try:
        if sha256(FR3_ASSET) != FR3_SHA256:
            raise RuntimeError('FROZEN_ASSET_HASH_MISMATCH')
        from isaacsim import SimulationApp
        app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting',
                             'multi_gpu': False, 'anti_aliasing': 0,
                             'sync_loads': False, 'fast_shutdown': True})
        import omni.usd
        import omni.physx
        from pxr import UsdGeom, UsdPhysics
        from isaacsim.core.api.simulation_context import SimulationContext
        from isaacsim.core.prims import SingleArticulation
        from offline.task28_p5_qp_6_r2.runtime_probe import _runtime_identity
        ctx = omni.usd.get_context()
        ctx.new_stage()
        stage = ctx.get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, '/World')
        builder = FIXTURE / 'scene/task28_single_cube_scene.py'
        spec = importlib.util.spec_from_file_location('frozen_r7_inertia_scene', builder)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.build_scene()
        out.update({'asset_path': str(FR3_ASSET), 'asset_sha256': sha256(FR3_ASSET),
                    'asset_expected_sha256': FR3_SHA256,
                    'scene_builder_path': str(builder), 'scene_builder_sha256': sha256(builder),
                    'scene_identity_before_suppression': _runtime_identity(stage, module),
                    'anonymous_stage': stage.GetRootLayer().anonymous})
        if out['scene_identity_before_suppression'].get('status') != 'SCENE_IDENTITY_VERIFIED':
            raise RuntimeError('SCENE_IDENTITY_NOT_VERIFIED')
        for side, root in (('left', LEFT_ROOT), ('right', RIGHT_ROOT)):
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if path != root + '/fr3_link8' and not path.startswith(root + '/fr3_link8/side_suction_tool'):
                    continue
                rec = {'side': side, 'path': path, 'type': prim.GetTypeName(),
                       'has_rigidbody': bool(prim.HasAPI(UsdPhysics.RigidBodyAPI)),
                       'has_mass': bool(prim.HasAPI(UsdPhysics.MassAPI)),
                       'has_collision': bool(prim.HasAPI(UsdPhysics.CollisionAPI)),
                       'applied_schemas': list(prim.GetAppliedSchemas()), 'attributes': {}}
                for attr in prim.GetAttributes():
                    name = attr.GetName()
                    if name.startswith(('physics:', 'physx', 'xformOp:')) or name in ('size', 'radius', 'height', 'axis'):
                        rec['attributes'][name] = {'value': serial(attr.Get()),
                            'authored': attr.HasAuthoredValueOpinion(),
                            'property_stack': [{'layer': p.layer.identifier, 'path': str(p.path)} for p in attr.GetPropertyStack()]}
                owner = prim
                while owner and not owner.IsPseudoRoot() and not owner.HasAPI(UsdPhysics.RigidBodyAPI):
                    owner = owner.GetParent()
                rec['nearest_rigid_body_ancestor'] = str(owner.GetPath()) if owner and not owner.IsPseudoRoot() else None
                out['records'].append(rec)
        out['status'] = 'COMPOSED_USD_INSPECTED'
        save()
        graph = stage.GetPrimAtPath(TASK_ROOT + '/ROSGraph')
        if not graph.IsValid() or not graph.IsActive():
            raise RuntimeError('ROS_GRAPH_IDENTITY')
        graph.SetActive(False)
        out['ros_graph_suppressed_before_play'] = not graph.IsActive()
        physics = stage.GetPrimAtPath(TASK_ROOT + '/physicsScene')
        attrs = ['physxScene:solverType', 'physxScene:enableGPUDynamics',
                 'physxScene:broadphaseType', 'physxScene:timeStepsPerSecond',
                 'physxScene:enableExternalForcesEveryIteration']
        def physics_config():
            return {name: serial(physics.GetAttribute(name).Get()) if physics.GetAttribute(name).IsValid() else None for name in attrs}
        out['physics_before_context'] = physics_config()
        sim = SimulationContext(physics_prim_path=TASK_ROOT + '/physicsScene', set_defaults=False,
                                backend='numpy', device='cpu')
        out['physics_after_context'] = physics_config()
        subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            lambda dt: out['physics_callback_dts_s'].append(float(dt)))
        sim.play()
        out['initialization_note'] = 'Installed SimulationContext.play documents one internal handle-propagation step; no explicit sim.step was invoked.'
        for side, root in (('left', LEFT_ROOT), ('right', RIGHT_ROOT)):
            arm = SingleArticulation(root)
            arm.initialize()
            view = arm._articulation_view
            names = list(view.body_names)
            masses = np.array(view.get_body_masses(clone=True), copy=True)[0]
            inertias = np.array(view.get_body_inertias(clone=True), copy=True)[0]
            coms = np.array(view._physics_view.get_coms(), copy=True)[0]
            if not all(np.isfinite(x).all() for x in (masses, inertias, coms)):
                raise RuntimeError('NONFINITE_BODY_PROPERTIES')
            bodies = [{'name': name, 'mass_kg': float(masses[i]),
                       'inertia_com_frame_kg_m2_row_major': inertias[i].tolist(),
                       'com_pose_local_xyzw': coms[i].tolist()}
                      for i, name in enumerate(names)]
            out['runtime_bodies'][side] = {'body_names': names, 'bodies': bodies,
                'link8_present': 'fr3_link8' in names,
                'link8': bodies[names.index('fr3_link8')] if 'fr3_link8' in names else None,
                'q9': np.array(arm.get_joint_positions(), copy=True).tolist(),
                'dq9': np.array(arm.get_joint_velocities(), copy=True).tolist()}
        out['physics_callbacks_count'] = len(out['physics_callback_dts_s'])
        out['current_time_s'] = float(sim.current_time)
        out['current_time_step_index'] = int(sim.current_time_step_index)
        out['status'] = 'COMPOSED_USD_AND_RUNTIME_BODY_PROPERTIES_READ'
        out['qualification'] = ('Runtime link mass/inertia and collider ownership are measured. '
            'No tool-free counterfactual or independent payload measurement was authorized/performed; '
            'the unique incremental L-tool mass/inertia contribution remains UNKNOWN. '
            'Absence of a tool MassAPI is not evidence of zero contribution.')
        save()
        print(json.dumps({'status': out['status'], 'output': str(args.output),
                          'physics_callbacks': len(out['physics_callback_dts_s']),
                          'runtime_link8': {s: v['link8'] for s, v in out['runtime_bodies'].items()}), flush=True)
    except BaseException as exc:
        out['status'] = 'INSPECTION_FAILED'
        out['errors'].append({'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()})
        save()
        traceback.print_exc()
    finally:
        save()  # Persist errors before fast_shutdown can terminate the interpreter.
        if app is not None:
            app.close()
    return 0 if out['status'] == 'COMPOSED_USD_AND_RUNTIME_BODY_PROPERTIES_READ' else 1


if __name__ == '__main__':
    raise SystemExit(main())
