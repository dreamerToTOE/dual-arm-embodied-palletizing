import numpy as np
import pytest

from offline.task28_g3_p2_4_r4.source_adapter import (
    AdapterContract, AdapterRole, SourceAdapter, SourceAdapterError,
    SourceKind, SourceSample, SourceValidation, canonical_descriptor,
)

NAMES = tuple(f"panda_joint{i}" for i in range(1, 8))
CONTRACT = AdapterContract(NAMES, nominal_dt_s=0.01, dt_tolerance_s=1e-7,
                           max_age_s=0.002, max_abs_effort_Nm=20., max_step_Nm=5.)
HASH = "a" * 64


def descriptor(kind, **kw):
    return canonical_descriptor(kind, NAMES, evidence_sha256=HASH,
                                sample_period_s=0.01, **kw)


def sample(i=0, t=None, values=None, **kw):
    return SourceSample(np.zeros(7) if values is None else values,
                        i * 0.01 if t is None else t, i, **kw)


def test_simulation_drive_is_candidate_and_never_real_tau_j():
    a = SourceAdapter(CONTRACT)
    out = a.accept(descriptor(SourceKind.SIMULATION_DRIVE_EFFORT), sample(values=np.arange(7)))
    assert out.source_kind == SourceKind.SIMULATION_DRIVE_EFFORT.value
    assert out.is_simulation_source and out.is_candidate_estimate
    assert not out.is_real_fr3_tau_J
    assert out.source_validation == SourceValidation.SIMULATION_CANDIDATE.value
    assert out.sign_convention == "positive_on_robot" and out.frame == "joint"


def test_second_sample_checks_dt_and_step_and_keeps_state_on_rejection():
    a = SourceAdapter(CONTRACT); d = descriptor(SourceKind.SIMULATION_APPLIED_EFFORT)
    a.accept(d, sample(0, values=np.zeros(7)))
    with pytest.raises(SourceAdapterError, match="EFFORT_STEP_LIMIT"):
        a.accept(d, sample(1, values=np.full(7, 6.0)))
    assert a.last_sequence == 0
    out = a.accept(d, sample(1, values=np.ones(7)))
    assert out.dt_s == pytest.approx(.01)


@pytest.mark.parametrize("kind", [SourceKind.MODEL_INFERRED_TORQUE, SourceKind.ESTIMATED_EXTERNAL_TORQUE])
def test_model_and_external_estimate_cannot_be_actuator_effort(kind):
    a = SourceAdapter(CONTRACT)
    with pytest.raises(SourceAdapterError, match="INCOMPATIBLE_SOURCE_ROLE"):
        a.accept(descriptor(kind), sample())


def test_model_term_and_external_estimate_have_explicit_roles():
    model = SourceAdapter(CONTRACT)
    out = model.accept(descriptor(SourceKind.MODEL_INFERRED_TORQUE), sample(),
                       role=AdapterRole.MODEL_TERM_INPUT)
    assert out.role == AdapterRole.MODEL_TERM_INPUT.value
    est = SourceAdapter(CONTRACT).accept(
        descriptor(SourceKind.ESTIMATED_EXTERNAL_TORQUE), sample(),
        role=AdapterRole.EXTERNAL_TORQUE_ESTIMATE)
    assert est.role == AdapterRole.EXTERNAL_TORQUE_ESTIMATE.value
    assert est.is_candidate_estimate


def test_fake_tau_j_with_isaac_identity_is_rejected_at_descriptor_boundary():
    with pytest.raises(SourceAdapterError, match="SOURCE_IDENTITY_MISMATCH"):
        canonical_descriptor(SourceKind.REAL_FR3_TAU_J, NAMES,
                             evidence_sha256=HASH, sample_period_s=.01,
                             source_identity="isaacsim.get_applied_joint_efforts")


def test_tau_j_requires_franka_clock_and_api_validation():
    with pytest.raises(SourceAdapterError, match="REAL_TAU_J_CLOCK_REQUIRED"):
        canonical_descriptor(SourceKind.REAL_FR3_TAU_J, NAMES,
                             evidence_sha256=HASH, sample_period_s=.01,
                             timestamp_field="sim_time_s")
    d = canonical_descriptor(SourceKind.REAL_FR3_TAU_J, NAMES,
                             evidence_sha256=HASH, sample_period_s=.01,
                             timestamp_field="franka_control_time_s")
    out = SourceAdapter(CONTRACT).accept(d, sample())
    assert out.is_real_fr3_tau_J and not out.is_simulation_source
    assert not out.is_candidate_estimate


@pytest.mark.parametrize("bad", [
    dict(values=np.full(7, np.nan)),
    dict(values=np.full(7, 21.)),
    dict(valid=False),
    dict(quality_flags=("dropped",)),
    dict(t=-1.),
])
def test_bad_quality_values_fail_closed_without_advancing(bad):
    a = SourceAdapter(CONTRACT); d = descriptor(SourceKind.SIMULATION_DRIVE_EFFORT)
    with pytest.raises(SourceAdapterError):
        a.accept(d, sample(**bad))
    assert a.last_sequence is None


def test_stale_future_duplicate_and_dt_mismatch_reject():
    d = descriptor(SourceKind.SIMULATION_DRIVE_EFFORT)
    a = SourceAdapter(CONTRACT)
    with pytest.raises(SourceAdapterError, match="STALE_SAMPLE"):
        a.accept(d, sample(), now_s=.003)
    a.accept(d, sample())
    with pytest.raises(SourceAdapterError, match="NONMONOTONIC_SEQUENCE"):
        a.accept(d, sample(0, t=.01))
    with pytest.raises(SourceAdapterError, match="DT_OUT_OF_BOUNDS"):
        a.accept(d, sample(1, t=.02))
    with pytest.raises(SourceAdapterError, match="DECLARED_DT_MISMATCH"):
        a.accept(d, sample(1, dt_s=.02))


def test_unknown_or_mismatched_descriptor_rejects():
    with pytest.raises(SourceAdapterError, match="UNKNOWN_SOURCE_KIND"):
        canonical_descriptor("isaac_tau_J", NAMES, evidence_sha256=HASH, sample_period_s=.01)
    d = descriptor(SourceKind.SIMULATION_DRIVE_EFFORT)
    bad = d.__class__(**{**d.__dict__, "units": "Nm"})
    with pytest.raises(SourceAdapterError, match="SOURCE_SEMANTICS_MISMATCH"):
        SourceAdapter(CONTRACT).accept(bad, sample())
