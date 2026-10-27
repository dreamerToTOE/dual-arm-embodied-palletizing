#!/usr/bin/env python3
"""Isolated Isaac 4.5 explicit-effort source characterization.

This creates a runtime-only articulation configuration. The frozen USD is read
as an asset and is never saved. Position drives are disabled by setting all
runtime stiffness/damping to zero and effort modes to ``force`` before calling
``set_joint_efforts``. ``get_applied_joint_efforts`` is retained as an input
readback, while ``get_measured_joint_efforts`` is recorded only as projected
solver force and is not treated as actuator torque.
"""
from __future__ import annotations
import argparse, hashlib, json, os, sys, time, traceback
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'experiment_preregistration.json'
ROOT = HERE.parents[1]

def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''): h.update(b)
    return h.hexdigest()

def write_new(path: Path, value):
    with path.open('x', encoding='utf-8') as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--asset', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    ap.add_argument('--prereg-sha256', required=True)
    ap.add_argument('--stage', choices=('source', 'online'), required=True)
    ap.add_argument('--source-gate', type=Path)
    args = ap.parse_args()
    if sha(CONFIG) != args.prereg_sha256: raise SystemExit('PREREGISTRATION_HASH_MISMATCH')
    cfg = json.loads(CONFIG.read_text(encoding='utf-8'))
    if not args.asset.is_file() or sha(args.asset) != cfg['fixture_usd_sha256']:
        raise SystemExit('MISSING_OR_WRONG_FROZEN_ASSET')
    if args.stage == 'online':
        if args.source_gate is None or not args.source_gate.is_file(): raise SystemExit('SOURCE_GATE_REQUIRED')
        gate = json.loads(args.source_gate.read_text(encoding='utf-8'))
        if gate.get('status') != 'SOURCE_GATE_PASS' or gate.get('preregistration_sha256') != args.prereg_sha256:
            raise SystemExit('SOURCE_GATE_NOT_PASS')
    args.output.mkdir(parents=False, exist_ok=False)
    status = {'schema':'task28.g3_p2_4_r2.collector.v1', 'status':'STARTED', 'stage':args.stage,
              'preregistration_sha256':args.prereg_sha256, 'collector_sha256':sha(Path(__file__)),
              'asset':str(args.asset.resolve()), 'asset_sha256':sha(args.asset), 'completed_cases':[],
              'source_gate_sha256':sha(args.source_gate) if args.source_gate else None,
              'frozen_asset_modified':False, 'execution_authorized':False, 'physical_robot_tested':False}
    app = None; total = 0; started = time.monotonic()
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,
                             'anti_aliasing':0,'sync_loads':False,'fast_shutdown':True})
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, RigidPrim
        from isaacsim.core.utils.stage import add_reference_to_stage
        import omni
        world = World(stage_units_in_meters=1.0, physics_dt=cfg['physics_dt_s'],
                      rendering_dt=cfg['physics_dt_s'], backend='numpy', device='cpu')
        add_reference_to_stage(usd_path=str(args.asset.resolve()), prim_path='/World/ProbeFR3')
        robot = world.scene.add(SingleArticulation('/World/ProbeFR3', name='fr3', position=np.asarray(cfg['base_world_m'])))
        link = world.scene.add(RigidPrim('/World/ProbeFR3/fr3_link7', name='link7'))
        world.reset()
        names = list(robot.dof_names)
        if names != cfg['expected_all_dof_names']: raise RuntimeError('DOF_NAMES_OR_ORDER_MISMATCH:'+repr(names))
        indices = np.arange(9, dtype=np.int32)
        arm_indices = np.arange(7, dtype=np.int32)
        view = robot._articulation_view
        pre_modes = view.get_effort_modes()[0]
        pre_kp, pre_kd = [np.asarray(x, dtype=float)[0] for x in view.get_gains()]
        # Runtime-only explicit effort setup. No USD save and no source-asset edit.
        view.set_effort_modes('force', joint_indices=indices)
        view.set_gains(kps=np.zeros(9), kds=np.zeros(9), joint_indices=indices, save_to_usd=False)
        post_modes = view.get_effort_modes()[0]
        post_kp, post_kd = [np.asarray(x, dtype=float)[0] for x in view.get_gains()]
        if post_modes != ['force']*9: raise RuntimeError('FORCE_MODE_NOT_ESTABLISHED:'+repr(post_modes))
        if not (np.allclose(post_kp, 0.0) and np.allclose(post_kd, 0.0)): raise RuntimeError('DRIVES_NOT_ZERO')
        status['runtime'] = {'all_dof_names':names, 'arm_indices':arm_indices.tolist(),
            'pre_effort_modes':pre_modes, 'post_effort_modes':post_modes,
            'pre_stiffness':pre_kp.tolist(), 'pre_damping':pre_kd.tolist(),
            'post_stiffness':post_kp.tolist(), 'post_damping':post_kd.tolist(),
            'effort_configuration':'view.set_effort_modes(force); view.set_gains(kps=0,kds=0,save_to_usd=False)',
            'set_joint_efforts_api':'SingleArticulation.set_joint_efforts -> set_dof_actuation_forces',
            'applied_readback_api':'SingleArticulation.get_applied_joint_efforts -> get_dof_actuation_forces',
            'projected_readback_api':'SingleArticulation.get_measured_joint_efforts -> get_dof_projected_joint_forces',
            'physics_dt_s':float(world.get_physics_dt()), 'source_asset_unchanged':True,
            'dof_units':['rad,rad/s,Nm']*7+['m,m/s,N']*2}
        q0 = np.asarray(cfg['configuration_rad'] + [0.04,0.04], dtype=float)
        cases = cfg['source_cases'] if args.stage == 'source' else cfg['online_cases']
        for cid in cases:
            if time.monotonic()-started > 120: raise TimeoutError('WALL_TIMEOUT')
            world.reset(); robot.set_joint_positions(q0, joint_indices=indices); robot.set_joint_velocities(np.zeros(9), joint_indices=indices)
            # Reapply runtime mode/gains after reset; no physical mode transition is used.
            view.set_effort_modes('force', joint_indices=indices)
            view.set_gains(kps=np.zeros(9), kds=np.zeros(9), joint_indices=indices, save_to_usd=False)
            command = np.asarray(cfg['commands']['zero'] if cid == 'zero' or cid.startswith('force_') else cfg['commands'][cid], dtype=float)
            force = np.asarray([0.,0.,0.]) if args.stage == 'source' else np.asarray(cfg['online_force_world_n'][cid],dtype=float)
            obs = {'case_id':cid,'stage':args.stage,'preregistration_sha256':args.prereg_sha256,'samples':[]}
            oracle = {'case_id':cid,'stage':args.stage,'command_generalized_effort':command.tolist(),'force_world_n':force.tolist(),'application_points_world_m':[]}
            for step in range(cfg['source_steps'] if args.stage=='source' else cfg['online_steps']):
                robot.set_joint_efforts(command, joint_indices=indices)
                q_pre = np.asarray(robot.get_joint_positions(joint_indices=arm_indices),float)
                point = np.asarray(robot._articulation_view.get_link_transforms() if False else [0.,0.,0.]) # set below after physics
                if args.stage == 'online' and np.any(force):
                    # Apply known evaluation force to link 7, but keep it in the oracle only.
                    pos, _ = link.get_world_poses(); point = np.asarray(pos[0],float)
                    link.apply_forces_and_torques_at_pos(forces=force[None,:], torques=np.zeros((1,3)), positions=point[None,:], is_global=True)
                world.step(render=False); total += 1
                read_start=time.monotonic()
                q=np.asarray(robot.get_joint_positions(),float); dq=np.asarray(robot.get_joint_velocities(),float)
                applied=np.asarray(robot.get_applied_joint_efforts(),float); projected=np.asarray(robot.get_measured_joint_efforts(),float)
                mass=np.asarray(view.get_mass_matrices(),float); cor=np.asarray(view.get_coriolis_and_centrifugal_forces(),float); grav=np.asarray(view.get_generalized_gravity_forces(),float)
                read_end=time.monotonic(); t=float(world.current_time)
                if point.shape != (3,):
                    pos,_=link.get_world_poses(); point=np.asarray(pos[0],float)
                arr=np.r_[q,dq,applied,projected,mass.ravel(),cor.ravel(),grav.ravel()]
                if not np.isfinite(arr).all(): raise RuntimeError('NONFINITE_SOURCE')
                obs['samples'].append({'step':step,'sim_time_s':t,'physics_step_index':int(world.current_time_step_index),
                  'q_rad_or_m':q.tolist(),'dq_rad_s_or_m_s':dq.tolist(),'commanded_effort':command.tolist(),
                  'applied_effort_readback':applied.tolist(),'measured_projected_effort':projected.tolist(),
                  'mass_matrix':mass[0].tolist(),'coriolis_compensation':cor[0].tolist(),'gravity_compensation':grav[0].tolist(),
                  'source_read_duration_s':read_end-read_start,'q1_rad':float(q[0]),'dq1_rad_s':float(dq[0])})
                oracle['application_points_world_m'].append(point.tolist())
                if abs(q[0]) > cfg['limits']['max_abs_q1_rad'] or np.max(np.abs(dq[:7])) > cfg['limits']['max_abs_qd_rad_s']:
                    raise RuntimeError('BOUNDED_MOTION_LIMIT')
            write_new(args.output/(cid+'_observations.json'),obs); write_new(args.output/(cid+'_oracle.json'),oracle)
            status['completed_cases'].append(cid)
        status['status']='COLLECTION_COMPLETE_NOT_EVALUATED'
    except BaseException as exc:
        status['status']='COLLECTION_FAILED'; status['failure']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    finally:
        status['physics_steps']=total; status['elapsed_wall_s']=time.monotonic()-started
        write_new(args.output/'collector_status.json',status)
        if app is not None: app.close()
    return 0 if status['status']=='COLLECTION_COMPLETE_NOT_EVALUATED' else 1

if __name__ == '__main__': raise SystemExit(main())
