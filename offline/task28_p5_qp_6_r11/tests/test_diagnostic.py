import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]


def test_v10a_single_variable_diagnostic_is_read_only_and_complete():
    d = json.loads((HERE / "evidence/single_variable_diagnostics.json").read_text())
    assert d["schema"] == "TASK28_R11_V10A_SINGLE_VARIABLE_DIAGNOSTICS_V1"
    assert d["execution_authorized"] is False
    assert d["physical_execution"] is False
    assert len(d["records"]) == 210
    assert d["qp_activation"]["all_samples_optimal"] is True
    assert d["qp_activation"]["constraints_active"] is False


def test_v10a_diagnostic_separates_reference_and_measured_lag():
    d = json.loads((HERE / "evidence/single_variable_diagnostics.json").read_text())
    p = d["peak_metrics"]
    assert p["actual_common_object_error_m"]["value"] > 5.7e-4
    assert p["reference_common_object_error_m"]["value"] < 2.5e-4
    assert p["actual_vs_reference_error_m"]["value"] > 5.7e-4
    assert p["relative_tcp_translation_error_m"]["value"] < 1.5e-4
    assert p["qp_hard_residual_inf"]["value"] < 2e-7
    assert p["qp_common_residual_inf"]["value"] < 2e-7
