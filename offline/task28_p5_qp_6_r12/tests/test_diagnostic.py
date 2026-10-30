import json
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]


def test_global_feedback_sensitivity_is_read_only_and_preregistered():
    d = json.loads((HERE / "evidence/feedback_gain_sensitivity.json").read_text())
    assert d["schema"] == "TASK28_R12_FEEDBACK_GAIN_SENSITIVITY_V1"
    assert d["execution_authorized"] is False
    assert d["physical_execution"] is False
    assert d["selection_rule"].find("0.50") == -1
    for cell in ("V10-A", "V10-B", "V10-C"):
        assert len(d["cells"][cell]) == 4


def test_frozen_scale_proxy_preserves_all_acceptance_gates():
    d = json.loads((HERE / "evidence/feedback_gain_sensitivity.json").read_text())
    selected = {x["cell"]: x for cell in d["cells"].values() for x in cell if x["scale"] == 0.75}
    assert set(selected) == {"V10-A", "V10-B", "V10-C"}
    for x in selected.values():
        assert x["peak_object_error_m"] <= 0.00025
        assert x["peak_relative_error_m"] <= 0.00015
