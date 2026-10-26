#!/usr/bin/env python3
"""Bounded frozen-model single-state IK continuation for A-stage controls.

This deterministic construction evaluates a prescribed, finite list of object
poses. It neither searches paths nor projects interpolated joint trajectories.
The resulting waypoints are SYNTHETIC computational controls, never sensor data
or commands, and waypoint validity is not a continuous-motion certificate.
Only the sibling a_stage_waypoints.json is written by this script.
"""
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import scipy

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO / 'offline/task28_g1_1'),
                str(REPO / 'offline/task28_g1_2a'),
                str(REPO / 'offline/task28_g1_1/tests')]

from task28_validator.closed_chain import desired_tcps, evaluate_closure
from task28_validator.model import DEFAULT_FIXTURE
from task28_validator.se3 import transform
from task28_validator.state import AttachmentMode
from task28_contact.examples import input_state
from task28_contact.clearance import (ClearanceFrame, CubePoseSource, ConfirmationEvent,
                                      TemporalClearanceConfirmer)
from task28_contact.modes import (AttachmentIntent, EvidenceSource, ObservedAttachment,
                                  OperationMode, Snapshot, SupportSite)
from task28_contact.validator import ContactModeValidator
from reference_fk import usd_fk, residuals as independent_residuals


STARTING_COMMIT = 'bd519c47d226c1fb66bd6e36ed273194dd4ebb8b'
MAX_OBJECT_STEP_M = .002
IK_MAX_NFEV = 300


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _segment_positions(start, finish):
    """Prescribed equal Cartesian increments, not joint interpolation."""
    start, finish = np.asarray(start), np.asarray(finish)
    divisions = math.ceil(float(np.linalg.norm(finish - start)) / MAX_OBJECT_STEP_M)
    return [start + (finish - start) * (index / divisions)
            for index in range(divisions + 1)]


def main():
    started = time.perf_counter()
    gate = ContactModeValidator()
    model = gate.model
    prior_path = REPO / 'offline/task28_g1_2a/examples/a_stage_sequence.json'
    prior = json.loads(prior_path.read_text())['configurations']
    cached_rows = None
    if sys.argv[1:]:
        if sys.argv[1:] != ['--revalidate-waypoints']:
            raise SystemExit('Only optional --revalidate-waypoints is supported')
        cached_data = json.loads(Path(__file__).with_name('a_stage_waypoints.json').read_text())
        cached_rows = [row for segment in cached_data['segments'].values() for row in segment['waypoints']]
    output = {
        'schema': 'task28.g1_2b_0.a_stage_waypoints.v1',
        'starting_commit': STARTING_COMMIT,
        'scope': 'FINITE_SYNTHETIC_WAYPOINT_CONSTRUCTION_NOT_PATH_PLANNING',
        'provenance': 'SYNTHETIC_COMPUTATIONAL_CONTROL_NOT_MEASURED_OBSERVATION',
        'continuous_safety_certificate': False,
        'generator': str(Path(__file__).relative_to(REPO)),
        'generator_sha256': digest(Path(__file__)),
        'fixture_manifest_sha256': digest(DEFAULT_FIXTURE / 'fixture_manifest.json'),
        'seed_source': str(prior_path.relative_to(REPO)),
        'seed_source_sha256': digest(prior_path),
        'construction': {
            'object_position_rule': 'Prescribed piecewise linear XYZ list; fixed identity orientation.',
            'solution_mode': 'REVALIDATE_PREVIOUSLY_BOUNDED_SOLVED_Q' if cached_rows is not None else 'BOUNDED_IK_CONTINUATION',
            'joint_rule': 'Bounded single-state IK, previous solved state as the only seed; no interpolated-state projection.',
            'max_object_waypoint_spacing_m': MAX_OBJECT_STEP_M,
            'ik_max_attempts_per_arm_per_pose': 1,
            'ik_max_function_evaluations_per_arm_per_pose': IK_MAX_NFEV,
            'seed_randomness': 'No random candidate is used with max_attempts=1 and an explicit seed.',
            'timing': 'Synthetic 10 ms waypoint spacing; no velocity, acceleration, actuator or ROS timing claim.',
            'interpolation_warning': 'Linear joint interpolation plus separate object interpolation is not assumed to preserve closure; edge checker must evaluate intermediates.',
            'state_gates': ['frozen limits', 'independent USD-vs-URDF FK', 'full bilateral SE3 closure',
                            'unchanged FCL with phase-local strict support semantics'],
        },
        'environment': {'numpy': np.__version__, 'scipy': scipy.__version__},
        'segments': {},
        'blockers': [],
    }
    if cached_rows is not None:
        cached_q_data = [{key: row[key] for key in ('left_q', 'right_q', 'object_position_m', 'object_quaternion_xyzw')} for row in cached_rows]
        output['construction']['revalidated_q_input_sha256'] = hashlib.sha256(json.dumps(cached_q_data, sort_keys=True).encode()).hexdigest()
        output['construction']['original_bounded_solve_summary'] = cached_data['construction'].get('original_bounded_solve_summary', cached_data['summary'])
    counter = 0
    now = 100.
    q_previous = {side: tuple(prior['supply_supported'][side + '_q'])
                  for side in ('left', 'right')}
    attached_intent = AttachmentIntent('b0-a-stage-attach', 100.0325, True, True)
    release_intent = None

    def waypoint(position, mode, site=None, *, attached=True, event='observe', preserve_q=None):
        nonlocal counter, now, q_previous, release_intent
        counter += 1
        now = 100. + .01 * counter
        stamp = now - .005
        obj = transform(np.eye(3), position)
        desired = desired_tcps(obj)
        q, ik_results, independent = {}, {}, {}
        reasons = []
        for side, target in zip(('left', 'right'), desired):
            if cached_rows is not None:
                cached = cached_rows[counter - 1]
                if np.max(np.abs(np.asarray(cached['object_position_m']) - position)) > 1e-14:
                    raise RuntimeError('Cached object pose differs from prescribed sequence')
                q[side] = tuple(cached[side + '_q'])
                ik_results[side] = {**cached['verification']['ik'][side], 'revalidated_previous_solution': True}
            elif preserve_q is not None:
                q[side] = tuple(preserve_q[side])
                ik_results[side] = {'success': True, 'attempts': 0, 'function_evaluations': 0,
                                    'reason': 'PRESERVED_VALIDATED_BOUNDARY'}
            else:
                solution = model.ik(side, target, seeds=[q_previous[side]],
                                    max_attempts=1, max_nfev=IK_MAX_NFEV)
                q[side] = solution.q
                ik_results[side] = {key: value for key, value in asdict(solution).items() if key != 'q'}
                if not solution.success:
                    reasons.append(side.upper() + ':' + solution.reason)
            if len(q[side]) != 7:
                return None, reasons + ['MISSING_IK_CONFIGURATION']
            ip, ir = independent_residuals(model.fk(side, q[side]), usd_fk(DEFAULT_FIXTURE, side, q[side]))
            independent[side] = {'translation_error_m': ip, 'rotation_error_rad': ir}
            if ip > 1e-6 or ir > 1e-6:
                reasons.append(side.upper() + '_INDEPENDENT_FK_DISAGREEMENT')
        configuration = {'left_q': list(q['left']), 'right_q': list(q['right']),
                         'object_position_m': [float(v) for v in position], 'object_yaw_rad': 0.}
        state = input_state(model, configuration, stamp, attached, attached,
                            AttachmentMode.BILATERAL if attached else AttachmentMode.DETACHED)
        if mode is OperationMode.RELEASED_SUPPORTED and release_intent is None:
            release_intent = AttachmentIntent('b0-a-stage-release', stamp - .0025, False, False)
        intent = (attached_intent if attached else release_intent if mode is OperationMode.RELEASED_SUPPORTED
                  else AttachmentIntent('b0-a-stage-detached', 99.99, False, False))
        snapshot = Snapshot(state, mode, site, intent, ObservedAttachment(
            f'b0-a-stage-feedback-{counter}', stamp, intent.request_id, attached, attached,
            EvidenceSource.INDEPENDENT_SYNTHETIC))
        validation = gate.validate(snapshot, now)
        reasons.extend(validation.reasons)
        closure = evaluate_closure(obj, model.fk('left', q['left']), model.fk('right', q['right']))
        if not closure.valid:
            reasons.extend(closure.reasons)
        margins = [float(np.min(np.minimum(np.asarray(q[side]) - model.joint_lower,
                                           model.joint_upper - np.asarray(q[side])))) for side in ('left', 'right')]
        row = {
            'time_s': now, 'observation_stamp_s': stamp,
            'left_q': list(q['left']), 'right_q': list(q['right']),
            'object_position_m': configuration['object_position_m'],
            'object_quaternion_xyzw': [0., 0., 0., 1.],
            'mode': mode.value, 'support_site': None if site is None else site.value,
            'attached': attached, 'confirmation_event': event,
            'provenance': 'SYNTHETIC_COMPUTATIONAL_CONTROL_NOT_MEASURED_OBSERVATION',
            'cube_evidence_id': f'b0-a-stage-cube-{counter}',
            'attachment_evidence_id': snapshot.observed.evidence_id,
            'intent': asdict(intent),
            'verification': {
                'valid': not reasons, 'reasons': sorted(set(reasons)),
                'minimum_joint_limit_margin_rad': min(margins),
                'closure_translation_error_m': closure.translation_error_m,
                'closure_rotation_error_rad': closure.rotation_error_rad,
                'left_object_translation_error_m': closure.left_object_translation_error_m,
                'right_object_translation_error_m': closure.right_object_translation_error_m,
                'independent_usd_vs_urdf_fk': independent,
                'ik': ik_results,
                'fcl_collision_pairs': [] if validation.collision is None else list(validation.collision.remaining_collisions),
                'accepted_strict_support_contact_records': 0 if validation.collision is None else len(validation.collision.accepted_support_contacts),
                'checked_shape_pairs': 0 if validation.collision is None else validation.collision.reference.checked_shape_pairs,
            },
        }
        q_previous = q
        return row, reasons

    def add_segment(name, positions, mode, site=None, *, attached=True, event='observe', preserve_first=False):
        rows = []
        segment_started = time.perf_counter()
        for index, position in enumerate(positions):
            mode_here = mode(index, len(positions)) if callable(mode) else mode
            site_here = site(index, len(positions)) if callable(site) else site
            row, reasons = waypoint(position, mode_here, site_here, attached=attached,
                                    event=event if index == 0 else 'observe',
                                    preserve_q=q_previous if preserve_first and index == 0 else None)
            if row is not None:
                rows.append(row)
            if reasons:
                output['blockers'].append({'segment': name, 'waypoint_index': index,
                                           'position_m': [float(v) for v in position],
                                           'reasons': sorted(set(reasons)),
                                           'meaning': 'Bounded prescribed-waypoint construction failed here; no global infeasibility claim.'})
                break
        print(json.dumps({'completed_segment': name, 'waypoints': len(rows), 'rejected': bool(reasons)}), file=sys.stderr, flush=True)
        output['segments'][name] = {'construction_complete': len(rows) == len(positions) and not any(
            blocker['segment'] == name for blocker in output['blockers']),
            'requested_waypoint_count': len(positions), 'waypoints': rows,
            'construction_computation_time_s': time.perf_counter() - segment_started}

    supply = prior['supply_supported']['object_position_m']
    supply_air = prior['supply_airborne']['object_position_m']
    target_air = prior['target_airborne']['object_position_m']
    target = prior['target_supported']['object_position_m']
    add_segment('grasp_boundary', [supply] * 3, OperationMode.DETACHED_SUPPORTED,
                SupportSite.SUPPLY, attached=False, preserve_first=True)
    add_segment('attachment_confirmation', [supply] * 3, OperationMode.BILATERAL_SUPPORTED,
                SupportSite.SUPPLY, event='attach', preserve_first=True)
    add_segment('vertical_lift', _segment_positions(supply, supply_air),
                lambda index, size: OperationMode.BILATERAL_SUPPORTED if index == 0 else OperationMode.BILATERAL_TRANSITIONING,
                lambda index, size: SupportSite.SUPPLY if index == 0 else None,
                event='begin_lift', preserve_first=True)
    add_segment('bilateral_transport', _segment_positions(supply_air, target_air),
                OperationMode.BILATERAL_AIRBORNE, preserve_first=True)
    add_segment('vertical_placement', _segment_positions(target_air, target),
                lambda index, size: OperationMode.BILATERAL_SUPPORTED if index == size - 1 else OperationMode.BILATERAL_TRANSITIONING,
                lambda index, size: SupportSite.TARGET if index == size - 1 else None,
                event='begin_place', preserve_first=True)
    add_segment('support_confirmation', [target] * 3, OperationMode.BILATERAL_SUPPORTED,
                SupportSite.TARGET, preserve_first=True)
    add_segment('release_boundary', [target] * 3, OperationMode.RELEASED_SUPPORTED,
                SupportSite.TARGET, attached=False, event='release', preserve_first=True)
    rows = [row for segment in output['segments'].values() for row in segment['waypoints']]
    if cached_rows is not None and counter != len(cached_rows):
        raise RuntimeError('Cached waypoint count differs from prescribed sequence')
    confirmer = TemporalClearanceConfirmer(gate)
    temporal_rejections = []
    for row in rows:
        state = input_state(model, {**row, 'object_yaw_rad': 0.}, row['observation_stamp_s'], row['attached'], row['attached'],
                            AttachmentMode.BILATERAL if row['attached'] else AttachmentMode.DETACHED)
        intent = AttachmentIntent(**row['intent'])
        snapshot = Snapshot(state, OperationMode(row['mode']),
                            SupportSite(row['support_site']) if row['support_site'] is not None else None,
                            intent, ObservedAttachment(row['attachment_evidence_id'], row['observation_stamp_s'],
                            intent.request_id, row['attached'], row['attached'], EvidenceSource.INDEPENDENT_SYNTHETIC))
        frame = ClearanceFrame(snapshot, row['cube_evidence_id'], CubePoseSource.SYNTHETIC_TEST_INPUT)
        temporal = confirmer.observe(frame, row['time_s'], ConfirmationEvent(row['confirmation_event']))
        row['temporal_verification'] = {key: value.value if hasattr(value, 'value') else value
            for key, value in vars(temporal).items() if key not in ('geometry', 'validation')}
        if not temporal.valid:
            temporal_rejections.append({'time_s': row['time_s'], 'reasons': list(temporal.reasons)})
    output['temporal_session'] = {'valid': not temporal_rejections, 'rejections': temporal_rejections,
        'final_phase': confirmer.phase.value,
        'meaning': 'R1 confirmation applied only to explicitly declared synthetic waypoint frames; interpolation samples are not independent observations.'}
    output['summary'] = {
        'all_prescribed_waypoints_constructed': not output['blockers'],
        'total_waypoints': len(rows),
        'valid_waypoints': sum(row['verification']['valid'] for row in rows),
        'maximum_closure_translation_m': max(row['verification']['closure_translation_error_m'] for row in rows),
        'maximum_closure_rotation_rad': max(row['verification']['closure_rotation_error_rad'] for row in rows),
        'minimum_joint_limit_margin_rad': min(row['verification']['minimum_joint_limit_margin_rad'] for row in rows),
        'construction_computation_time_s': time.perf_counter() - started,
    }
    destination = Path(__file__).with_name('a_stage_waypoints.json')
    destination.write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'destination': str(destination), **output['summary'], 'blockers': output['blockers']}))


if __name__ == '__main__':
    main()
