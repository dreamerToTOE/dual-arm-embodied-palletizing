"""Known world-force probe on an original Task27 distal link, not a contact sensor."""
from __future__ import annotations

import numpy as np

# Each steady case has the same duration; opposite arm stays unloaded.
CASES = tuple((side, force) for side in ('left', 'right')
              for force in (0.0, 0.1, 0.2, 0.0, -0.1, -0.2, 0.0))
CASE_DURATION_S = 3.0


def calibration_case(elapsed_s):
    index = int(max(0.0, elapsed_s) / CASE_DURATION_S)
    return None if index >= len(CASES) else (index, *CASES[index])


def attach_known_load_probe(torque, stage):
    """Write known force after each state read, for the next physics interval."""
    from pxr import Gf, PhysicsSchemaTools, UsdUtils
    import omni.physx

    if torque._reaction_recorder is None:
        raise ValueError('KNOWN_LOAD_REACTION_RECORDER_REQUIRED')
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    sim = omni.physx.get_physx_simulation_interface()
    body_paths = {s: f'/World/{s}_fr3/fr3_hand_tcp' for s in ('left', 'right')}
    body_rows = {}
    for side, path in body_paths.items():
        meta = torque._reaction_recorder.metadata[side]
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError('KNOWN_LOAD_BODY_MISSING:' + path)
        body_rows[side] = meta['body_names_by_row'].index('fr3_hand_tcp')
    encoded = {s: PhysicsSchemaTools.sdfPathToInt(p) for s, p in body_paths.items()}
    original_step = torque._torque_step
    start = float(torque._time_s)

    def step(dt):
        if any(torque.feed.grippers[s].is_closed() for s in ('left', 'right')):
            raise RuntimeError('KNOWN_LOAD_REQUIRES_OPEN_GRIPPERS')
        original_step(dt)
        case = calibration_case(torque._time_s - start - float(dt))
        if case is None:
            return
        index, side, force_y = case
        view = torque.articulations[side]._articulation_view._physics_view
        pose = np.asarray(view.get_link_transforms(), dtype=float)[0, body_rows[side]]
        point = pose[:3]
        force = [0.0, float(force_y), 0.0]
        if not np.isfinite(np.r_[point, force]).all():
            raise RuntimeError('KNOWN_LOAD_NONFINITE')
        if force_y:
            sim.apply_force_at_pos(stage_id, encoded[side], Gf.Vec3f(*force),
                                   Gf.Vec3f(*point), 'Force')
        torque._log({'event': 'KNOWN_LOAD_COMMAND', 'physics_step': torque._step - 1,
                     'state_time_s': torque._time_s - float(dt), 'case_index': index,
                     'body_path': body_paths[side], 'force_world_N': force,
                     'application_point_world_m': point.tolist(),
                     'semantics': 'known_injected_force_for_following_physics_interval_not_contact_force'})

    torque._torque_step = step
    torque._log({'event': 'KNOWN_LOAD_START', 'time_s': start,
                 'duration_s': len(CASES) * CASE_DURATION_S,
                 'case_duration_s': CASE_DURATION_S, 'cases': CASES,
                 'application': 'force_at_current_body_origin_world',
                 'oracle_separate_from_controller': True})
    return start + len(CASES) * CASE_DURATION_S
