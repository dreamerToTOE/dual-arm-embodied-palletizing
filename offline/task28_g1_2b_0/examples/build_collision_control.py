#!/usr/bin/env python3
"""Generate a real frozen-model payload/table interior-collision control.

This is an explicitly prescribed unsafe three-waypoint mathematical trajectory,
not a physically executable trajectory or a planner result. No simulated or
measured data are used. Only collision_control.json beside this script is written.
"""
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO / 'offline/task28_g1_1'),
                str(REPO / 'offline/task28_g1_2a'),
                str(REPO / 'offline/task28_g1_1/tests')]

from reference_fk import usd_fk
from task28_validator.closed_chain import desired_tcps
from task28_validator.model import DEFAULT_FIXTURE
from task28_validator.se3 import pose_matrix, residual
from task28_validator.state import AttachmentMode
from task28_contact.examples import input_state
from task28_contact.modes import (AttachmentIntent, EvidenceSource,
                                  ObservedAttachment, OperationMode, Snapshot)
from task28_contact.validator import ContactModeValidator


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def main():
    gate = ContactModeValidator()
    model = gate.model
    source = REPO / 'offline/task28_g1_2a/examples/r1_configurations.json'
    prior = json.loads(source.read_text())['configurations']
    endpoint = prior['supply_gap_4000um']
    seed = prior['supply_gap_0000um']
    middle = {'object_position_m': [.65, 0., .24], 'object_yaw_rad': 0.,
              'object_quaternion_xyzw': [0., 0., 0., 1.]}
    solve = {}
    for side, target in zip(('left', 'right'), desired_tcps(
            pose_matrix(middle['object_position_m'], middle['object_quaternion_xyzw']))):
        result = model.ik(side, target, seeds=[seed[side + '_q']],
                          max_attempts=1, max_nfev=300)
        if not result.success:
            raise RuntimeError('COLLISION_CONTROL_IK_FAILED:' + side)
        middle[side + '_q'] = list(result.q)
        solve[side] = asdict(result)

    records = []
    # The unsafe interior point is prescribed explicitly, never hidden in an
    # interpolation callback. Endpoint identity is deliberate: all violations
    # arise strictly inside the edge rather than at either external endpoint.
    for index, (label, configuration) in enumerate((
            ('collision_endpoint_start', endpoint),
            ('deliberate_payload_table_intrusion', middle),
            ('collision_endpoint_finish', endpoint))):
        time_s = float(index)
        stamp_s, now_s = 50. + time_s, 50.005 + time_s
        state = input_state(model, configuration, stamp_s, True, True,
                            AttachmentMode.BILATERAL)
        independent = {}
        for side in ('left', 'right'):
            q = configuration[side + '_q']
            tcp = usd_fk(DEFAULT_FIXTURE, side, q)
            pos, rot = residual(model.fk(side, q), tcp)
            independent[side] = {'translation_error_m': pos,
                                  'rotation_error_rad': rot}
            if pos > 1e-6 or rot > 1e-6:
                raise RuntimeError('COLLISION_CONTROL_INDEPENDENT_FK_MISMATCH')
            observation = replace(getattr(state, side + '_tcp'),
                                  position_m=tuple(tcp[:3, 3]),
                                  quaternion_xyzw=tuple(Rotation.from_matrix(
                                      tcp[:3, :3]).as_quat()))
            state = replace(state, **{side + '_tcp': observation})
        snapshot = Snapshot(state, OperationMode.BILATERAL_TRANSITIONING, None,
                            AttachmentIntent('fixed-bilateral-request', 49., True, True),
                            ObservedAttachment('independent-synthetic-' + str(index),
                                               stamp_s, 'fixed-bilateral-request',
                                               True, True,
                                               EvidenceSource.INDEPENDENT_SYNTHETIC))
        result = gate.validate(snapshot, now_s)
        ql, qr = np.asarray(configuration['left_q']), np.asarray(configuration['right_q'])
        margin = float(min(np.min(ql - model.joint_lower),
                           np.min(model.joint_upper - ql),
                           np.min(qr - model.joint_lower),
                           np.min(model.joint_upper - qr)))
        if index in (0, 2) and not result.valid:
            raise RuntimeError('COLLISION_CONTROL_ENDPOINT_REJECTED:' + repr(result.reasons))
        if index == 1:
            if (result.valid or not result.closure.valid or margin < 0
                    or not any({record['owner_a'], record['owner_b']} ==
                               {'Cube_01', 'task28_table'}
                               for record in result.collision.remaining_collisions)):
                raise RuntimeError('COLLISION_CONTROL_INTERIOR_NOT_ISOLATED')
        records.append({
            'label': label, 'time_s': time_s,
            'source': 'INDEPENDENT_SYNTHETIC_TEST_INPUT',
            'scope': 'REAL_FROZEN_KINEMATICS_AND_FCL_POINT_CONTROL',
            'mode': snapshot.mode.value, 'support_site': None,
            'configuration': {key: configuration[key] for key in (
                'object_position_m', 'object_quaternion_xyzw', 'object_yaw_rad',
                'left_q', 'right_q')},
            'observation_stamp_s': stamp_s, 'now_s': now_s,
            'evidence_id': snapshot.observed.evidence_id,
            'full_single_state_gate_valid': result.valid,
            'reasons': result.reasons,
            'within_joint_limits': margin >= 0,
            'minimum_joint_limit_margin_rad': margin,
            'independent_usd_vs_urdf_fk': independent,
            'closure': asdict(result.closure),
            'fk_observation_errors': result.fk_observation_errors,
            'collision': asdict(result.collision),
        })
    output = {
        'schema': 'task28.g1_2b_0.real_collision_control.v1',
        'starting_commit': 'bd519c47d226c1fb66bd6e36ed273194dd4ebb8b',
        'generator': str(Path(__file__).relative_to(REPO)),
        'generator_sha256': digest(Path(__file__)),
        'fixture_manifest_sha256': digest(DEFAULT_FIXTURE / 'fixture_manifest.json'),
        'source_configurations_sha256': digest(source),
        'scope': 'Explicit unsafe interior waypoint. Both external endpoints pass full frozen single-state gates; interior independently passes joint limits, USD-vs-URDF FK and bilateral closure, but fails real frozen FCL payload/table gate. No physical reachability or continuous safety claim.',
        'numerical_environment': {'numpy': np.__version__, 'scipy': scipy.__version__, 'fcl': '0.7.0'},
        'solve_budget': {'max_attempts': 1, 'max_nfev': 300, 'random_seed': None,
                         'seed_source': 'preserved supply_gap_0000um configuration'},
        'interior_ik': solve,
        'waypoints': records,
    }
    target = Path(__file__).with_name('collision_control.json')
    target.write_text(json.dumps(plain(output), indent=2, allow_nan=False) + '\n')
    print(str(target))


if __name__ == '__main__':
    main()
