"""Deterministic software fault controls; no new planner/controller run."""
from copy import deepcopy
from dataclasses import replace
from .reference import (build_reference_sequence, load_archive, load_qp3_positive,
                        validate_reference, validate_sequence)

# Faults to archived inputs. These are contract mutations, not simulated motion.
INPUT_FAULTS = [
 ('wrong_cube', ('raw_qp2','request','cube_id'), 'Cube_02', 'INCORRECT_CUBE_ID'),
 ('wrong_identity', ('identity',), 'unknown', 'SOURCE_IDENTITY_MISMATCH'),
 ('missing_identity', ('identity',), None, 'SOURCE_IDENTITY_MISMATCH'),
 ('qp_infeasible', ('raw_qp2','steps',0,'qp','status'), 'INFEASIBLE', 'UPSTREAM_QP_NOT_ACCEPTED'),
 ('qp_missing_qdot', ('raw_qp2','steps',0,'qp','qdot'), None, 'QP_QDOT_INVALID'),
 ('current_geometry_rejected', ('raw_qp2','steps',0,'current_state','valid'), False, 'UPSTREAM_GEOMETRY_INVALID:current_state'),
 ('predicted_geometry_rejected', ('raw_qp2','steps',0,'predicted_state','valid'), False, 'UPSTREAM_GEOMETRY_INVALID:predicted_state'),
 ('edge_inconclusive', ('raw_qp2','steps',0,'sampled_edge','status'), 'edge_inconclusive', 'UPSTREAM_EDGE_INVALID'),
 ('invalid_bounds', ('raw_qp2','steps',0,'frozen_bounds','valid'), False, 'UPSTREAM_BOUNDS_INVALID'),
 ('async_right_stamp', ('position_targets',0,'right','stamp_s'), .021, 'ASYNC_REFERENCE_PAIR'),
 ('async_right_source_time', ('position_targets',0,'right','time_s'), .001, 'ASYNC_REFERENCE_PAIR'),
 ('wrong_order', ('position_targets',0,'left','joint_names'), ['x']*7, 'JOINT_ORDER_MISMATCH'),
 ('wrong_frame', ('position_targets',0,'right','frame_id'), 'base', 'INCORRECT_REFERENCE_FRAME'),
 ('wrong_pair_step', ('position_targets',0,'right','source_step_index'), 1, 'PAIR_SOURCE_STEP_MISMATCH'),
 ('wrong_q', ('position_targets',0,'right','positions_rad',0), .5, 'Q_TARGET_MISMATCH'),
 ('wrong_qdot', ('position_targets',0,'right','source_qdot_rad_s',0), .5, 'QDOT_TARGET_MISMATCH'),
 ('broken_q_chain', ('raw_qp2','steps',1,'q_initial',0), .5, 'DISCONTINUOUS_Q_INITIAL'),
 ('changed_provenance', ('position_targets',0,'right','provenance'), 'MEASURED', 'UPSTREAM_ARCHIVE_CONTENT_MISMATCH'),
 ('false_phase', ('raw_qp2','steps',0,'predicted_state','waypoint','mode'), 'bilateral_airborne', 'UPSTREAM_CONTACT_MODE_MISMATCH'),
 ('fake_confirmation', ('raw_qp2','phase_confirmation_observations_generated'), 3, 'SYNTHETIC_PHASE_CONFIRMATION_FORBIDDEN'),
 ('execution_enabled', ('position_targets',0,'right','execution_authorized'), True, 'EXECUTION_MUST_REMAIN_DISABLED'),
]


def mutated_input(path, value):
    x = deepcopy(load_qp3_positive()); node = x
    for part in path[:-1]:
        node = node[part]
    node[path[-1]] = value
    return x


def negative_controls():
    cases = []
    for name, path, value, expected in INPUT_FAULTS:
        try:
            build_reference_sequence(mutated_input(path, value))
        except ValueError as exc:
            actual = str(exc)
        else:
            actual = 'UNEXPECTED_ACCEPTANCE'
        cases.append({'case':name,'scope':'ARCHIVE_CONTRACT_FAULT_INJECTION','mutation_path':list(path),
                      'mutation_value':value,'expected_rejection':expected,'observed_rejection':actual,'passed':actual==expected})
    refs = build_reference_sequence(load_qp3_positive())
    actions = [
        ('original_qp3_payload_table', lambda: build_reference_sequence(load_archive()['negative_payload_table']), 'UPSTREAM_STATUS_NOT_ACCEPTED'),
        ('acceleration_zeros', lambda: validate_reference(replace(refs[0],acceleration_rad_s2=(0.,)*14)), 'ACCELERATION_MUST_BE_UNAVAILABLE'),
        ('duplicate_stamp', lambda: validate_sequence([refs[0],replace(refs[1],stamp_s=refs[0].stamp_s)]), 'STALE_OR_OUT_OF_ORDER_REFERENCE'),
        ('stale_replay', lambda: validate_sequence(refs,now_s=.2,max_age_s=.01), 'STALE_OR_FUTURE_REFERENCE'),
        ('future_replay', lambda: validate_sequence(refs,now_s=.05,max_age_s=.01), 'STALE_OR_FUTURE_REFERENCE'),
    ]
    for name, action, expected in actions:
        try:
            action()
        except ValueError as exc:
            actual=str(exc)
        else:
            actual='UNEXPECTED_ACCEPTANCE'
        cases.append({'case':name,'scope':'ORIGINAL_ARCHIVED_GEOMETRY_FAILURE' if name.startswith('original') else 'REFERENCE_CONTRACT_FAULT_INJECTION',
                      'expected_rejection':expected,'observed_rejection':actual,'passed':actual==expected})
    return cases
