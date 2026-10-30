import json
import hashlib
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]
BASE=HERE.parent/'task28_p5_qp_6_r12'

def test_unqualified_previous_evidence_is_explicitly_revoked():
    d=json.loads((HERE/'evidence/methodology_correction.json').read_text())
    assert d['old_perturbation_peak_values_valid_as_robustness_result'] is False
    assert d['old_repetition_count_valid_as_fresh_runs'] is False
    for item in d['retained_artifacts']:
        assert hashlib.sha256((HERE/item['retained_path']).read_bytes()).hexdigest()==item['sha256']

def test_fresh_runs_invoke_frozen_r12_runtime_and_source():
    d=json.loads((HERE/'evidence/isaac_repetitions/plan.json').read_text())
    assert d['repeat_count_per_cell']==20
    assert d['controller_modified'] is False
    assert d['runtime_module']=='offline.task28_p5_qp_6_r12.runtime_virtual'
    for name in ['source_contract.json','registrations.json','runtime_virtual.py','qp_math.py']:
        key='offline/task28_p5_qp_6_r12/'+name
        assert hashlib.sha256((BASE/name).read_bytes()).hexdigest()==d['source_hashes'][key]
    reg=json.loads((BASE/'registrations.json').read_text())
    for cell in ['V10-A','V10-B','V10-C']:
        assert reg[cell]['feedback_gain_scale']==.75
        assert reg[cell]['velocity_capture']['tau_s']==.1
