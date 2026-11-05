import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / 'evidence'

def test_executor_status_is_not_task_pass():
    p=json.loads((ROOT/'r2b_rear_wall_physical05/summary.json').read_text())
    assert p['executor_exit']==1
    q=json.loads((ROOT/'r2b_rear_wall_physical05/execution_phase_receipts.json').read_text())
    assert q['status_separation']['manipulation_result']=='FAILED_OR_BLOCKED'

def test_phase_source_is_executor_receipt():
    q=json.loads((ROOT/'r2b_rear_wall_physical05/execution_phase_receipts.json').read_text())
    assert all('executor log event receipts' in r.get('phase_source','') for r in q['receipts'] if 'phase_source' in r)
    assert any(r['execution_status']=='PREFLIGHT_BLOCKED' for r in q['receipts'])

def test_sampling_gap_is_not_called_dropout():
    q=json.loads((ROOT/'r2b_rear_wall_physical05/visual_qualification.json').read_text())
    assert q['observation_timing']['max_intercapture_gap_s'] > 90.0
    assert q['longest_visual_dropout_s']==0.0
