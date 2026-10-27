"""Synthetic-only negative and algebra controls; no calibration claim."""
from dataclasses import replace

import numpy as np
import pytest

from offline.task28_p2_2.estimator import Contract
from offline.task28_p2_2_r1.adapter import (
    SEMANTIC, SOURCE, SignCalibration, StaticLimits, StaticWindow,
    static_actuator_pair_to_wrench,
)

NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
CONTRACT = Contract(NAMES, "world", "left_fr3_side_suction_tcp", 0, 0)
LIMITS = StaticLimits(0.2, .01, .002, .05, .002)
CALIBRATION = SignCalibration(SOURCE, SEMANTIC, True, "a" * 64)
J = np.column_stack((np.eye(6), np.zeros(6)))
WRENCH = np.array([1., 2., 3., .4, -.5, .6])


def inputs():
    q = np.tile(np.arange(7) / 10, (3, 1))
    bias = np.tile(np.arange(7, dtype=float), (3, 1))
    common = dict(q_rad=q, dq_rad_s=np.zeros((3, 7)), timestamps_s=[1., 1.05, 1.1],
                  evaluation_timestamp_s=1.1, joint_names=NAMES, source=SOURCE)
    unloaded = StaticWindow(effort_Nm=bias, **common)
    # Synthetic static equilibrium: tau_actuator + J.T @ wrench = 0.
    loaded = replace(unloaded, effort_Nm=bias - J.T @ WRENCH)
    kwargs = dict(contract=CONTRACT, limits=LIMITS, calibration=CALIBRATION,
                  jacobian_q=q.mean(axis=0), jacobian_joint_names=NAMES,
                  frame="world", tcp_point=CONTRACT.tcp_point,
                  jacobian_provenance="synthetic matrix")
    return unloaded, loaded, kwargs


def test_explicit_action_reaction_conversion_and_svd():
    unloaded, loaded, kw = inputs()
    result = static_actuator_pair_to_wrench(unloaded, loaded, J, **kw)
    np.testing.assert_allclose(result["delta_actuator_effort_Nm"], -J.T @ WRENCH)
    np.testing.assert_allclose(result["external_generalized_torque_on_robot_Nm"], J.T @ WRENCH)
    np.testing.assert_allclose(result["estimate"].wrench_minimum_norm, WRENCH)
    assert result["estimate"].rank == 6
    assert result["estimate"].reconstruction_residual_Nm < 1e-12


@pytest.mark.parametrize("field,value,reason", [
    ("joint_names", NAMES[::-1], "JOINT_ORDER_MISMATCH"),
    ("source", "raw_hardware_tau_J", "UNSUPPORTED_EFFORT_SOURCE"),
    ("q_units", "deg", "UNSUPPORTED_UNITS"),
    ("dq_units", "deg/s", "UNSUPPORTED_UNITS"),
    ("effort_units", "N", "UNSUPPORTED_UNITS"),
    ("timestamps_s", [1., 1., 1.1], "INVALID_OR_OUT_OF_ORDER_TIMESTAMP"),
    ("timestamps_s", [1., 1.1, 1.05], "INVALID_OR_OUT_OF_ORDER_TIMESTAMP"),
    ("timestamps_s", [1., 1.05, float("nan")], "INVALID_WINDOW_TIMESTAMPS"),
    ("evaluation_timestamp_s", 1.4, "STALE_OBSERVATION"),
    ("effort_Nm", np.full((3, 7), np.nan), "INVALID_EFFORT"),
    ("q_rad", np.full((3, 7), np.inf), "INVALID_Q"),
    ("dq_rad_s", np.ones((3, 7)), "NOT_QUASISTATIC"),
    ("q_rad", np.ones((3, 7)), "PAIR_Q_MISMATCH"),
    ("effort_Nm", np.vstack([np.zeros(7), np.ones(7), np.zeros(7)]), "UNSTABLE_WINDOW_EFFORT"),
])
def test_reject_bad_windows(field, value, reason):
    u, l, kw = inputs()
    with pytest.raises(ValueError, match=reason):
        static_actuator_pair_to_wrench(u, replace(l, **{field: value}), J, **kw)


@pytest.mark.parametrize("change,reason", [
    ({"frame": "robot_base"}, "FRAME_OR_POINT_MISMATCH"),
    ({"tcp_point": "flange"}, "FRAME_OR_POINT_MISMATCH"),
    ({"jacobian_joint_names": NAMES[::-1]}, "JOINT_ORDER_MISMATCH"),
    ({"jacobian_provenance": ""}, "MISSING_JACOBIAN_PROVENANCE"),
    ({"jacobian_q": np.zeros(7)}, "JACOBIAN_CONFIGURATION_MISMATCH"),
    ({"calibration": replace(CALIBRATION, independent_equilibrium_confirmed=False)}, "MISSING_INDEPENDENT_SIGN_CALIBRATION"),
    ({"calibration": replace(CALIBRATION, evidence_sha256="unknown")}, "MISSING_INDEPENDENT_SIGN_CALIBRATION"),
    ({"calibration": replace(CALIBRATION, semantic="positive")}, "UNSUPPORTED_OR_UNCALIBRATED_SIGN"),
])
def test_reject_bad_contracts_or_unverified_sign(change, reason):
    u, l, kw = inputs()
    kw.update(change)
    with pytest.raises(ValueError, match=reason):
        static_actuator_pair_to_wrench(u, l, J, **kw)


def test_rank_deficiency_remains_visible():
    u, l, kw = inputs()
    deficient = J.copy()
    deficient[-1] = 0
    result = static_actuator_pair_to_wrench(u, l, deficient, **kw)
    assert result["estimate"].status == "RANK_DEFICIENT_NONUNIQUE"
    assert result["estimate"].rank == 5
    assert result["estimate"].reconstruction_residual_Nm > 0.5
