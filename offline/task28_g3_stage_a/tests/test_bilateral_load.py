import numpy as np
import pytest
from types import SimpleNamespace

from offline.task28_g3_stage_a.bilateral_load import (
    ArmObserverOutput, BilateralLoadObserver, RuntimeGeometry,
)


def txyz(x, y, z):
    t = np.eye(4); t[:3, 3] = [x, y, z]; return t


def candidate(side, wrench, t=1.0, source="SOURCE_UNVALIDATED_CANDIDATE"):
    est = SimpleNamespace(wrench_minimum_norm=np.asarray(wrench, dtype=float))
    history = SimpleNamespace(evidence_id=side + "-evidence")
    result = SimpleNamespace(status="CANDIDATE_ESTIMATE", source_status=source,
                             wrench=est, history=history)
    return ArmObserverOutput(side, result, "world", "side_suction_tcp", t)


def geometry():
    return RuntimeGeometry(txyz(.5, 0, .4),
                           {"left": txyz(.5, -.061, .4), "right": txyz(.5, .061, .4)},
                           "RUNTIME_MEASURED_SIMULATION", "geom-1")


def test_runtime_geometry_uses_measured_anchor_and_decomposes_candidate_pair():
    out = BilateralLoadObserver().observe(
        candidate("left", [0, 0, -2, 0, 0, 0]),
        candidate("right", [0, 0, -2, 0, 0, 0]), geometry())
    assert out.status == "CANDIDATE_BILATERAL_DECOMPOSITION"
    assert out.geometry_provenance == "RUNTIME_MEASURED_SIMULATION"
    assert out.physical_contact_wrench_validated is False
    assert out.decomposition.rank == 6
    assert out.residuals["null_space_N_Nm"] < 1e-10
    assert len(out.internal_axis_basis_local) == 12


def test_internal_target_is_bounded_and_has_zero_net_wrench():
    out = BilateralLoadObserver().observe(
        candidate("left", np.zeros(6)), candidate("right", np.zeros(6)), geometry())
    target = BilateralLoadObserver.bounded_internal_target(out, 3.)
    assert np.linalg.norm(out.decomposition.net_wrench) < 1e-12
    np.testing.assert_allclose(geometry().model.net(target), np.zeros(6), atol=1e-10)
    with pytest.raises(ValueError, match="OUT_OF_BOUNDS"):
        BilateralLoadObserver.bounded_internal_target(out, 51.)


def test_rejects_nominal_geometry_or_stale_pair():
    with pytest.raises(ValueError, match="GEOMETRY_PROVENANCE"):
        RuntimeGeometry(txyz(0, 0, 0), {"left": txyz(0, -.061, 0), "right": txyz(0, .061, 0)},
                        "G1_1_NOMINAL_FROZEN_CONTRACT", "g")
    with pytest.raises(ValueError, match="TIME_SKEW"):
        BilateralLoadObserver().observe(candidate("left", np.zeros(6)),
                                        candidate("right", np.zeros(6), t=1.01), geometry())


def test_torque_feasibility_is_diagnostic_only():
    out = BilateralLoadObserver().observe(
        candidate("left", np.zeros(6)), candidate("right", np.zeros(6)), geometry(),
        jacobians={"left": np.ones((6, 7)), "right": np.ones((6, 7))},
        torque_limits_Nm={"left": np.ones(7), "right": np.ones(7)})
    assert out.torque_feasibility["limit_source"].startswith("caller_declared")
    assert out.torque_feasibility["within_limits"] is True
