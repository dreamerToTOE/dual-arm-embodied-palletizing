#!/usr/bin/env python3
"""Read-only runtime effort-mode probe for the frozen isolated FR3 asset.

This does not set effort modes, apply forces, send commands, or run a motion.
It records the mode exposed by the installed wrapper and the accompanying
projected/applied force APIs.  It is not an actuator-torque calibration.
"""
from __future__ import annotations
import argparse, hashlib, json, os, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

def sha(p: Path) -> str:
    h = hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()

def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--asset', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    args = ap.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    out = {'schema':'task28.g3_p2_4_r1.mode_probe.v1', 'status':'STARTED',
           'asset':str(args.asset.resolve()), 'asset_sha256':sha(args.asset),
           'execution_authorized':False, 'physical_robot_tested':False,
           'control_mode_changed':False}
    app = None
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({'headless':True, 'renderer':'RaytracedLighting',
                             'multi_gpu':False, 'anti_aliasing':0, 'sync_loads':False,
                             'fast_shutdown':True})
        import numpy as np
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.stage import add_reference_to_stage
        world = World(stage_units_in_meters=1.0, physics_dt=1.0/120.0,
                      rendering_dt=1.0/120.0, backend='numpy', device='cpu')
        add_reference_to_stage(usd_path=str(args.asset.resolve()), prim_path='/World/ProbeFR3')
        robot = world.scene.add(SingleArticulation('/World/ProbeFR3', name='fr3',
                                                   position=np.asarray([0.65,-0.6,0.0])))
        world.reset()
        view = robot._articulation_view
        names = list(robot.dof_names)
        modes = view.get_effort_modes()
        measured = np.asarray(robot.get_measured_joint_efforts(), float)
        applied = np.asarray(robot.get_applied_joint_efforts(), float)
        out.update({'status':'PROBE_COMPLETE', 'physics_dt_s':float(world.get_physics_dt()),
                    'dof_names':names, 'effort_modes':modes,
                    'measured_projected_effort_initial':measured.tolist(),
                    'applied_effort_initial':applied.tolist(),
                    'read_semantics':{'effort_modes':'wrapper reads UsdPhysics.DriveAPI TypeAttr; no mode was set',
                                      'measured':'get_dof_projected_joint_forces',
                                      'applied':'get_dof_actuation_forces from explicit set_joint_efforts',
                                      'torque_validated':False}})
    except BaseException as exc:
        out.update({'status':'PROBE_FAILED','failure':{'type':type(exc).__name__,'message':str(exc)}})
    finally:
        args.output.write_text(json.dumps(out, indent=2, sort_keys=True) + '\n')
        if app is not None:
            app.close()
    return 0 if out['status'] == 'PROBE_COMPLETE' else 1

if __name__ == '__main__':
    raise SystemExit(main())
