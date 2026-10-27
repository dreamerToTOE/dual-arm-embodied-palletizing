"""Synthetic controls only: not Isaac, not a physical calibration."""
from dataclasses import replace
import numpy as np
import pytest
from offline.task28_p2_4.observer import ObserverContract, Sample, step

NAMES = tuple(f"joint{i}" for i in range(1, 8)) + ("finger_left", "finger_right")
ARM = NAMES[:7]
DOF_TYPES = ("revolute",) * 7 + ("prismatic",) * 2
DOF_UNITS = ("rad,rad/s,rad/s2,N*m",) * 7 + ("m,m/s,m/s2,N",) * 2
PROVENANCE = {k: "synthetic test " + k for k in ("q", "dq", "effort", "mass", "coriolis", "gravity", "jacobian")}
CONTRACT = ObserverContract(NAMES, ARM, "world", "tcp", .1, .005, .001, .1, 1e-10, 1000., "SYNTHETIC_ONLY", "a" * 64, DOF_TYPES, 1e-6)
J = np.column_stack((np.diag([1., 2., 3., 4., 5., 6.]), np.ones(6) * .1))
M = np.eye(9) * 2
M[0, 7] = M[7, 0] = .5


def sample(t=1., ident="sample1", dq=None, ext=None):
    dq = np.zeros(9) if dq is None else np.asarray(dq)
    ext = np.zeros(9) if ext is None else np.asarray(ext)
    # Caller controls synthetic acceleration separately; default first state static.
    q = np.linspace(0, .8, 9)
    c = np.arange(9) * .01
    g = np.arange(9) * .1
    return Sample(NAMES, ARM, q, dq, c + g - ext, M, c, g, J, q.copy(), dq.copy(),
                  q[:7].copy(), t, t, t, t, t, t + .001, ident, PROVENANCE,
                  "world", "tcp", DOF_TYPES, DOF_UNITS, "effort_i/acceleration_j")


def test_first_sample_is_warmup_and_never_synthetic_zero_acceleration():
    result = step(sample(), CONTRACT)
    assert result.status == "WARMUP" and result.wrench is None
    assert result.acceleration_SI is None and result.external_generalized_force_on_robot_SI is None


def test_online_with_full_mass_coupling_and_known_synthetic_wrench():
    initial = sample()
    history = step(initial, CONTRACT).history
    dt = .01
    a = np.array([1., -2., .4, 0., 0., .1, 0., 3., -1.])
    w = np.array([.3, -.2, .1, .04, -.06, .08])
    external = np.r_[J.T @ w, [0., 0.]]
    current = sample(1. + dt, "sample2", a * dt, external)
    # Independent Newton equation generates actuator effort; no wrench passed to step.
    current = replace(current, actuator_generalized_effort_SI=M @ a + current.coriolis_lhs_SI + current.gravity_lhs_SI - external)
    result = step(current, CONTRACT, history)
    np.testing.assert_allclose(result.acceleration_SI, a, atol=2e-14)
    np.testing.assert_allclose(result.external_generalized_force_on_robot_SI, external, atol=2e-14)
    np.testing.assert_allclose(result.wrench.wrench_minimum_norm, w, atol=1e-13)
    assert result.status == "CANDIDATE_ESTIMATE" and result.source_status == "SYNTHETIC_ONLY"
    assert result.inertial_term_SI[0] == pytest.approx(3.5)


def test_disturbance_outside_tcp_column_space_exposes_residual():
    history = step(sample(), CONTRACT).history
    # Independent nullspace vector has zero dot product with every J.T column.
    _, _, vh = np.linalg.svd(J)
    disturbance = np.r_[vh[-1] * .25, 0., 0.]
    result = step(sample(1.01, "sample2", ext=disturbance), CONTRACT, history)
    assert result.status == "TORQUE_OUTSIDE_TCP_MODEL"
    assert result.wrench.reconstruction_residual_Nm == pytest.approx(.25)
    assert "TCP-equivalent" in result.diagnostics


def test_wrench_at_unsupported_contact_may_fit_and_cannot_be_identified():
    history = step(sample(), CONTRACT).history
    external = np.r_[J.T @ np.array([1., 0., 0., 0., 0., 0.]), 0., 0.]
    result = step(sample(1.01, "sample2", ext=external), CONTRACT, history)
    assert result.wrench.reconstruction_residual_Nm < 1e-12
    assert "contact location" in result.diagnostics


def test_rank_deficiency_and_condition_never_report_candidate():
    history = step(sample(), CONTRACT).history
    singular = J.copy(); singular[5] = 0
    result = step(replace(sample(1.01, "sample2"), jacobian=singular), CONTRACT, history)
    assert result.status == "RANK_DEFICIENT_NONUNIQUE" and result.wrench.rank == 5
    bad_cond = J.copy(); bad_cond[5] *= 1e-4
    result = step(replace(sample(1.01, "sample2"), jacobian=bad_cond), CONTRACT, history)
    assert result.status == "ILL_CONDITIONED"


@pytest.mark.parametrize("changes, reason", [
    ({"joint_names": NAMES[::-1]}, "JOINT_ORDER_MISMATCH"),
    ({"arm_joint_names": ARM[::-1]}, "JOINT_ORDER_MISMATCH"),
    ({"position_SI": np.zeros(7)}, "INVALID_Q"),
    ({"velocity_SI": np.full(9, np.nan)}, "INVALID_VELOCITY"),
    ({"actuator_generalized_effort_SI": np.zeros(7)}, "INVALID_EFFORT"),
    ({"mass_matrix": np.eye(7)}, "INVALID_MASS_MATRIX"),
    ({"mass_matrix": np.diag([-1.] + [1.] * 8)}, "NONPOSITIVE_MASS_MATRIX"),
    ({"mass_matrix": np.triu(np.ones((9, 9)))}, "NONSYMMETRIC_MASS_MATRIX"),
    ({"coriolis_lhs_SI": np.zeros(7)}, "INVALID_CORIOLIS"),
    ({"gravity_lhs_SI": np.zeros(7)}, "INVALID_GRAVITY"),
    ({"jacobian": np.zeros((7, 6))}, "INVALID_JACOBIAN"),
    ({"model_position_SI": np.ones(9)}, "MODEL_STATE_MISMATCH"),
    ({"model_velocity_SI": np.ones(9)}, "MODEL_STATE_MISMATCH"),
    ({"jacobian_position_SI": np.ones(7)}, "JACOBIAN_STATE_MISMATCH"),
    ({"dof_types": ("revolute",) * 9}, "DOF_TYPE_MISMATCH"),
    ({"dof_units": ("rad,rad/s,rad/s2,N*m",) * 9}, "DOF_UNITS_MISMATCH"),
    ({"mass_matrix_units": "kg*m2"}, "DOF_UNITS_MISMATCH"),
    ({"frame": "base"}, "FRAME_OR_POINT_MISMATCH"),
    ({"tcp_point": "wrong"}, "FRAME_OR_POINT_MISMATCH"),
    ({"effort_semantics": "reaction_on_actuator"}, "UNSUPPORTED_SIGN_UNITS_OR_ORDER"),
    ({"model_semantics": "gravity_on_rhs"}, "UNSUPPORTED_SIGN_UNITS_OR_ORDER"),
    ({"units": "degrees"}, "UNSUPPORTED_SIGN_UNITS_OR_ORDER"),
    ({"jacobian_order": "angular_then_linear"}, "UNSUPPORTED_SIGN_UNITS_OR_ORDER"),
    ({"provenance": {}}, "MISSING_PROVENANCE"),
    ({"evidence_id": ""}, "MISSING_PROVENANCE"),
    ({"effort_time_s": .5}, "STALE_SAMPLE"),
    ({"model_time_s": .99}, "ASYNCHRONOUS_SAMPLE"),
    ({"jacobian_time_s": 2.}, "FUTURE_OR_NEGATIVE_TIMESTAMP"),
    ({"q_time_s": float("nan")}, "INVALID_TIMESTAMP"),
])
def test_sample_contract_rejections(changes, reason):
    with pytest.raises(ValueError, match=reason):
        step(replace(sample(), **changes), CONTRACT)


def test_timestamp_identity_and_rejected_sample_do_not_advance_history():
    old = step(sample(), CONTRACT).history
    with pytest.raises(ValueError, match="DUPLICATE_EVIDENCE"):
        step(sample(1.01), CONTRACT, old)
    with pytest.raises(ValueError, match="NONMONOTONIC_TIMESTAMP"):
        step(sample(.99, "older"), CONTRACT, old)
    with pytest.raises(ValueError, match="DERIVATIVE_INTERVAL_OUT_OF_BOUNDS"):
        step(sample(1.2, "too_late"), CONTRACT, old)
    assert old.evidence_id == "sample1" and old.timestamp_s == 1.
    assert step(sample(1.01, "good"), CONTRACT, old).status == "CANDIDATE_ESTIMATE"


def test_dynamic_sampling_does_not_silently_neglect_acceleration():
    previous = step(sample(), CONTRACT).history
    dq = np.array([.01] + [0.] * 8)
    current = sample(1.01, "moving", dq=dq)
    out = step(current, CONTRACT, previous)
    assert out.acceleration_SI[0] == pytest.approx(1)
    assert out.external_generalized_force_on_robot_SI[0] == pytest.approx(2)


@pytest.mark.parametrize("changes,reason", [
    ({"joint_names": NAMES[:-1] + ("joint1",)}, "INVALID_JOINT_CONTRACT"),
    ({"arm_joint_names": ARM[:-1] + ("missing",)}, "ARM_NOT_IN_ARTICULATION"),
    ({"dof_types": ("revolute",) * 8}, "INVALID_DOF_TYPES"),
    ({"dof_types": ("prismatic",) + DOF_TYPES[1:]}, "NONREVOLUTE_ARM_JOINT"),
    ({"max_tcp_residual_Nm": -1}, "INVALID_LIMITS"),
    ({"max_age_s": -1}, "INVALID_LIMITS"),
    ({"source_status": "UNKNOWN"}, "UNSUPPORTED_OR_UNVALIDATED_SOURCE"),
    ({"source_evidence_sha256": "unverified"}, "MISSING_SOURCE_EVIDENCE"),
])
def test_invalid_configuration_rejected(changes, reason):
    with pytest.raises(ValueError, match=reason):
        step(sample(), replace(CONTRACT, **changes))


def test_unvalidated_source_is_only_candidate_never_calibration():
    c = replace(CONTRACT, source_status="SOURCE_UNVALIDATED_CANDIDATE")
    old = step(sample(), c).history
    result = step(sample(1.01, "next"), c, old)
    assert result.source_status == "SOURCE_UNVALIDATED_CANDIDATE"
    assert result.status == "CANDIDATE_ESTIMATE"
    assert "assumptions" in result.diagnostics


def test_reused_stale_component_identity_rejected_even_inside_age_window():
    old = step(sample(), CONTRACT).history
    replayed_effort = replace(sample(1.001, "new"), effort_time_s=1.)
    with pytest.raises(ValueError, match="NONMONOTONIC_TIMESTAMP"):
        step(replayed_effort, CONTRACT, old)
