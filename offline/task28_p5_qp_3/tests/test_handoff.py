import numpy as np
import pytest

from task28_p5_qp_3.handoff import (
    HandoffContract, PositionTarget, make_position_targets, validate_target_pair,
)


def qvals():
    q0 = np.zeros(14)
    qd = np.linspace(-.2, .2, 14)
    return q0, q0 + .02 * qd, qd


def test_split_is_synchronized_and_reconstructable():
    q0, q1, qd = qvals()
    pair = make_position_targets(q0, q1, time_s=0.0, stamp_s=.02, sequence=0,
                                 source_step_index=0, qdot=qd)
    result = validate_target_pair(pair, expected_sequence=0)
    assert result["valid"] is True
    assert pair[0].stamp_s == pair[1].stamp_s
    assert pair[0].joint_names == tuple(f"left_fr3_joint{i}" for i in range(1, 8))


def test_wrong_cube_and_joint_order_rejected():
    q0, q1, qd = qvals()
    with pytest.raises(ValueError, match="INCORRECT_CUBE_ID"):
        validate_target_pair(make_position_targets(q0, q1, time_s=0, stamp_s=.02,
                                                    sequence=0, source_step_index=0, qdot=qd),
                             contract=HandoffContract(cube_id="Cube_02"))
    pair = list(make_position_targets(q0, q1, time_s=0, stamp_s=.02, sequence=0,
                                      source_step_index=0, qdot=qd))
    pair[0] = PositionTarget(**{**pair[0].__dict__, "joint_names": tuple(reversed(pair[0].joint_names))})
    with pytest.raises(ValueError, match="JOINT_ORDER_MISMATCH"):
        validate_target_pair(tuple(pair))


def test_duplicate_or_asynchronous_stamp_rejected():
    q0, q1, qd = qvals()
    pair = make_position_targets(q0, q1, time_s=0, stamp_s=.02, sequence=0,
                                 source_step_index=0, qdot=qd)
    with pytest.raises(ValueError, match="NON_MONOTONIC"):
        validate_target_pair(pair, previous_stamp_s=.02)
    right = PositionTarget(**{**pair[1].__dict__, "stamp_s": .021})
    with pytest.raises(ValueError, match="SYNCHRONIZED"):
        validate_target_pair((pair[0], right))


def test_stale_feedback_and_discontinuity_rejected():
    q0, q1, qd = qvals()
    pair = make_position_targets(q0, q1, time_s=0, stamp_s=.02, sequence=0,
                                 source_step_index=0, qdot=qd)
    with pytest.raises(ValueError, match="STALE"):
        validate_target_pair(pair, max_feedback_age_s=.005, feedback_stamp_s=0.0)
    with pytest.raises(ValueError, match="Q_NEXT_NOT"):
        make_position_targets(q0, q1 + .001, time_s=0, stamp_s=.02, sequence=0,
                              source_step_index=0, qdot=qd)


def test_execution_is_always_disabled():
    assert HandoffContract().execution_authorized is False
