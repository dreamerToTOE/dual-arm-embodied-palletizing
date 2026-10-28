"""Actual fault injection into the archive diagnostic boundary."""
from copy import deepcopy
from offline.task28_p5_qp_5_r2.calculations import validate_row


def negative_controls(row, steps, config):
    cases = []
    def case(name, mutate, expected):
        candidate = deepcopy(row)
        mutate(candidate)
        reason = None
        try:
            validate_row(candidate, steps, config, r1=True)
        except ValueError as exc:
            reason = str(exc)
        cases.append(dict(id=name, expected_reason=expected, actual_reason=reason, passed=reason==expected,
                          scope='CPU_SYNTHETIC_ARCHIVE_FAULT_NOT_PHYSICS'))
    case('nonfinite_joint', lambda r: r['arms']['left']['q_rad'].__setitem__(0, float('nan')), 'NONFINITE_OR_WRONG_SHAPE')
    case('wrong_dof_order', lambda r: r['arms']['right']['dof_names'].reverse(), 'DOF_ORDER')
    case('source_identity', lambda r: r['reference'].__setitem__('source_identity', 'wrong'), 'SOURCE_IDENTITY')
    case('one_step_timestamp_offset', lambda r: r.__setitem__('controller_time_s', r['controller_time_s']+config['physics_dt_s']), 'TIME_ALIGNMENT')
    case('wrong_q_reference', lambda r: r['reference']['q_ref_14'].__setitem__(0, 0.), 'REFERENCE_MISMATCH')
    case('wrong_object_reference', lambda r: r['object_pose_world'][0].__setitem__(3, 0.), 'OBJECT_REFERENCE_MISMATCH')
    case('wrong_pre_reference', lambda r: r['pre_step']['reference_qdot14'].__setitem__(0, 0.), 'PRE_REFERENCE_MISMATCH')
    return cases
