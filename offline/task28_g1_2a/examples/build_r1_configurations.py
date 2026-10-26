#!/usr/bin/env python3
"""Regenerate bounded, deterministic offline R1 kinematic controls.

The object poses, joint vectors and TCPs are synthetic mathematical inputs, not
measured observations. Each solve is a single-state IK query, never a planner.
Only this script's sibling r1_configurations.json is written.
"""
from dataclasses import asdict
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO/'offline/task28_g1_1'), str(REPO/'offline/task28_g1_2a'),
                str(REPO/'offline/task28_g1_1/tests')]
from task28_validator.model import RobotModel, DEFAULT_FIXTURE
from task28_validator.closed_chain import desired_tcps, evaluate_closure
from task28_validator.se3 import transform, residual
from task28_contact.validator import ContactModeValidator
from task28_contact.modes import OperationMode, SupportSite
from reference_fk import usd_fk

GAPS_M = (0., .0005, .0009, .001, .0011, .002, .0025, .0029, .003, .0031, .0035, .004)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def name(site, gap):
    return f'{site}_gap_{int(round(gap * 1e6)):04d}um'


def main():
    gate = ContactModeValidator()
    model = gate.model
    prior_path = Path(__file__).with_name('a_stage_sequence.json')
    prior = json.loads(prior_path.read_text())['configurations']
    result = {
        'schema': 'task28.g1_2a_r1.synthetic_kinematic_configurations.v1',
        'starting_commit': '7c7116a09315e2bebdad7bc2d1a12b2de0ca30af',
        'source': 'INDEPENDENT_SYNTHETIC',
        'scope': 'Synthetic single-state object poses and bounded deterministic IK; no sensor data, force evidence, commands, temporal or path validation.',
        'generator': str(Path(__file__).relative_to(REPO)),
        'generator_sha256': digest(Path(__file__)),
        'fixture_manifest_sha256': digest(DEFAULT_FIXTURE/'fixture_manifest.json'),
        'a_stage_seed_source_sha256': digest(prior_path),
        'numerical_environment': {'numpy': np.__version__, 'scipy': scipy.__version__, 'fcl': '0.7.0'},
        'solve_budget': {'max_attempts': 1, 'max_nfev': 300, 'seed': 'site-specific frozen A-stage supported configuration'},
        'configurations': {},
    }

    def generate(site, gap, rotation=None, key=None):
        rotation = np.eye(3) if rotation is None else rotation
        initial = prior[f'{site}_supported']
        half = gate.collision.support.half_size
        # Exact OBB support function: min z = center.z - sum_i |R_zi| h_i.
        half_vertical = float(np.abs(rotation[2, :]) @ half)
        center = np.array([*initial['object_position_m'][:2], gate.collision.support.table_top_z_m + half_vertical + gap])
        obj = transform(rotation, center)
        tcps = desired_tcps(obj)
        q, solutions, independent = {}, {}, {}
        for side, tcp in zip(('left', 'right'), tcps):
            if gap == 0. and np.array_equal(rotation, np.eye(3)):
                q[side] = tuple(initial[f'{side}_q'])
                pos, rot = residual(model.fk(side, q[side]), tcp)
                solutions[side] = {'success': True, 'translation_error_m': pos, 'rotation_error_rad': rot,
                                   'reason': 'PRESERVED_G1_2A_SUPPORTED_CONTROL', 'attempts': 0, 'function_evaluations': 0}
            else:
                solution = model.ik(side, tcp, seeds=[initial[f'{side}_q']], max_attempts=1, max_nfev=300)
                if not solution.success:
                    raise RuntimeError(f'IK failed {site} gap={gap} {side}: {solution}')
                q[side] = solution.q
                solutions[side] = {k:v for k,v in asdict(solution).items() if k != 'q'}
            if not model.within_limits(q[side]):
                raise RuntimeError('Generated joint limit violation')
            ip, ir = residual(model.fk(side,q[side]), usd_fk(DEFAULT_FIXTURE,side,q[side]))
            independent[side] = {'translation_error_m': ip, 'rotation_error_rad': ir}
            if ip > 1e-6 or ir > 1e-6:
                raise RuntimeError('Independent USD-FK disagreement')
        closure = evaluate_closure(obj, model.fk('left', q['left']), model.fk('right', q['right']))
        if not closure.valid:
            raise RuntimeError('Generated closure invalid')
        mode = OperationMode.BILATERAL_SUPPORTED if gap == 0. and np.array_equal(rotation,np.eye(3)) else OperationMode.BILATERAL_TRANSITIONING
        phase = gate.collision.validate(model.forward(q['left'],q['right']), obj, mode,
                                        SupportSite(site) if mode is OperationMode.BILATERAL_SUPPORTED else None)
        if not phase.valid:
            raise RuntimeError(f'Generated collision invalid {site}, gap={gap}: {phase.reasons}')
        local_vertices = np.array(list(itertools.product(*[(-h,h) for h in half])))
        world_vertices = (rotation @ local_vertices.T).T + center
        actual_gap = float(world_vertices[:,2].min()-gate.collision.support.table_top_z_m)
        configuration = {
            'site': site, 'requested_minimum_clearance_m': gap,
            'object_position_m': center.tolist(),
            'object_quaternion_xyzw': Rotation.from_matrix(rotation).as_quat().tolist(),
            'object_rotation_rpy_rad': Rotation.from_matrix(rotation).as_euler('xyz').tolist(),
            'object_yaw_rad': float(Rotation.from_matrix(rotation).as_euler('xyz')[2]),
            'left_q': list(q['left']), 'right_q': list(q['right']),
            'verification': {
                'within_joint_limits': True,
                'mode': mode.value,
                'minimum_clearance_m_all_8_corners': actual_gap,
                'minimum_clearance_error_m': actual_gap-gap,
                'ik': solutions,
                'independent_usd_vs_urdf_fk': independent,
                'closure_translation_error_m': closure.translation_error_m,
                'closure_rotation_error_rad': closure.rotation_error_rad,
                'left_object_translation_error_m': closure.left_object_translation_error_m,
                'left_object_rotation_error_rad': closure.left_object_rotation_error_rad,
                'right_object_translation_error_m': closure.right_object_translation_error_m,
                'right_object_rotation_error_rad': closure.right_object_rotation_error_rad,
                'collision_valid': phase.valid,
                'collision_reasons': list(phase.reasons),
                'remaining_collisions': len(phase.remaining_collisions),
                'accepted_strict_support_records': len(phase.accepted_support_contacts),
                'checked_shape_pairs': phase.reference.checked_shape_pairs,
                'skipped_unchanged_acm_shape_pairs': phase.reference.skipped_acm_shape_pairs,
            },
        }
        result['configurations'][key or name(site,gap)] = configuration

    for site in ('supply','target'):
        for gap in GAPS_M:
            generate(site,gap)
    for roll in (.01,.1):
        generate('supply',.003,Rotation.from_euler('x',roll).as_matrix(), f'supply_roll_{str(roll).replace(".","p")}_gap_3000um')
    destination = Path(__file__).with_name('r1_configurations.json')
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'configurations': len(result['configurations']), 'destination': str(destination),
                      'maximum_closure_translation_m': max(v['verification']['closure_translation_error_m'] for v in result['configurations'].values()),
                      'maximum_closure_rotation_rad': max(v['verification']['closure_rotation_error_rad'] for v in result['configurations'].values())}))


if __name__ == '__main__':
    main()
