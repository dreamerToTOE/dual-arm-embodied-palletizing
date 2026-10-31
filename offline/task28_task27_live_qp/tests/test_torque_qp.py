import numpy as np

from offline.task28_task27_live_qp.torque_qp import (
    CooperativeTorqueQPConfig,
    solve_cooperative_torque_qp,
)


def _state():
    # FR3 joint 4 has an upper limit below zero; use a valid interior state.
    q7 = np.array([-0.3, -0.4, 0.2, -1.5, 0.1, 1.2, 0.2])
    q = np.r_[q7, q7]
    dq = np.zeros(14)
    mass = np.diag(np.linspace(0.8, 2.2, 14))
    bias = np.linspace(-0.4, 0.4, 14)
    # Two simple, full-rank translational/spatial maps.  They represent
    # already-validated world-frame Jacobians, not a robot fixture claim.
    jl = np.zeros((6, 7)); jr = np.zeros((6, 7))
    jl[:6, :6] = np.eye(6); jr[:6, :6] = np.eye(6)
    return q, dq, mass, bias, jl, jr


def _solve(**kwargs):
    q, dq, m, h, jl, jr = _state()
    values = dict(q14=q, dq14=dq, mass14=m, bias14=h,
                  jacobian_left=jl, jacobian_right=jr,
                  model_validated=True, feedback_fresh=True)
    values.update(kwargs)
    return solve_cooperative_torque_qp(**values)


def test_dynamics_equality_and_bilateral_objective_are_centralized():
    result = _solve(common_accel_desired=np.r_[.2, 0, 0, 0, 0, 0],
                    relative_accel_desired=np.zeros(6))
    assert result.status == "QP_OPTIMAL"
    qdd = np.asarray(result.qdd14); tau = np.asarray(result.tau14_Nm)
    q, dq, m, h, *_ = _state()
    np.testing.assert_allclose(tau, m @ qdd + h, atol=3e-7)
    assert result.diagnostics["decision_dimension"] == 28
    assert result.diagnostics["dynamics_model"] == "tau=M*qdd+h"
    # Equal maps and zero relative target imply equal first-arm/second-arm
    # common motion for the unconstrained translational rows.
    np.testing.assert_allclose(qdd[:6], qdd[7:13], atol=2e-5)


def test_caller_verified_relative_jacobian_is_used_when_supplied():
    # Swapping one relative row changes the requested bilateral correction;
    # this catches silently falling back to a world-frame subtraction.
    jrel = np.zeros((6, 14)); jrel[1, 7] = 1.0
    result = _solve(jacobian_relative=jrel,
                    relative_accel_desired=np.r_[0., .4, 0., 0., 0., 0.])
    assert result.status == "QP_OPTIMAL"
    assert result.diagnostics["relative_jacobian_source"] == "caller_verified"
    assert result.diagnostics["relative_jacobian_shape"] == [6, 14]


def test_hard_common_task_constraint_is_enforced_or_fails_closed():
    cfg = CooperativeTorqueQPConfig(hard_common_acceleration=True)
    desired = np.r_[.01, 0, 0, 0, 0, 0]
    result = _solve(config=cfg, common_accel_desired=desired)
    assert result.status == "QP_OPTIMAL"
    assert result.diagnostics["hard_common_acceleration"] is True
    assert result.diagnostics["common_acceleration_residual_inf"] < 3e-7


def test_tracking_residual_rejection_keeps_failure_status_in_diagnostics():
    cfg = CooperativeTorqueQPConfig(common_residual_limit=1e-12)
    result = _solve(config=cfg, common_accel_desired=np.r_[.4, 0, 0, 0, 0, 0])
    assert result.status == "QP_TRACKING_REJECTED"
    assert result.diagnostics["status"] == "QP_TRACKING_REJECTED"
    assert result.qdd14 is None and result.tau14_Nm is None


def test_torque_rate_bound_is_hard_and_reported():
    prev = np.zeros(14)
    cfg = CooperativeTorqueQPConfig(
        torque_rate_Nm_s=(.06,) * 7,
        common_weight=1e6,
        relative_weight=1e6,
    )
    result = _solve(previous_tau14=prev, config=cfg,
                    common_accel_desired=np.r_[1., 0, 0, 0, 0, 0])
    assert result.status == "QP_OPTIMAL"
    tau = np.asarray(result.tau14_Nm)
    assert np.max(np.abs(tau)) <= .06 / 60. + 3e-7
    assert result.diagnostics["torque_rate_active_indices"]


def test_model_source_gate_is_fail_closed():
    q, dq, m, h, jl, jr = _state()
    result = solve_cooperative_torque_qp(
        q14=q, dq14=dq, mass14=m, bias14=h,
        jacobian_left=jl, jacobian_right=jr,
        model_validated=False,
    )
    assert result.status == "MODEL_GATE_REJECTED"
    assert result.qdd14 is None and result.tau14_Nm is None
    assert not result.execution_authorized


def test_stale_feedback_and_nonfinite_model_reject_without_command():
    stale = _solve(feedback_fresh=False)
    assert stale.status == "FEEDBACK_GATE_REJECTED"
    _, _, m, *_ = _state()
    m[0, 0] = np.nan
    bad = _solve(mass14=m)
    assert bad.status == "INPUT_REJECTED"
    assert bad.qdd14 is None and bad.tau14_Nm is None


def test_dynamics_or_limits_infeasible_is_fail_closed():
    # A large measured gravity term cannot be represented under a tiny torque
    # box; the affine dynamics and the box are genuinely disjoint.
    q, dq, m, h, jl, jr = _state()
    h = np.full(14, 100.)
    cfg = CooperativeTorqueQPConfig(torque_abs_Nm=(.1,) * 7)
    result = solve_cooperative_torque_qp(
        q14=q, dq14=dq, mass14=m, bias14=h,
        jacobian_left=jl, jacobian_right=jr,
        config=cfg, model_validated=True,
    )
    assert result.status == "QP_INFEASIBLE"
    assert result.tau14_Nm is None


def test_joint_velocity_and_position_lookahead_are_hard_bounds():
    q, dq, m, h, jl, jr = _state()
    q[0] = 2.8973 - 1e-5
    dq[0] = 0.1
    cfg = CooperativeTorqueQPConfig(qdd_abs_rad_s2=(100.,) * 7)
    result = solve_cooperative_torque_qp(
        q14=q, dq14=dq, mass14=m, bias14=h,
        jacobian_left=jl, jacobian_right=jr,
        config=cfg, model_validated=True,
    )
    assert result.status == "QP_OPTIMAL"
    qdd = np.asarray(result.qdd14)
    assert q[0] + dq[0] / 60. + .5 * qdd[0] / 60.**2 <= 2.8973 + 1e-8
    assert dq[0] + qdd[0] / 60. <= cfg.qdot_abs_rad_s[0] + 1e-8

