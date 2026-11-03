import json

import numpy as np
import pytest
import sys

from offline.task28_stage_b_contact.c2_evidence import EvidenceWriter, verify_evidence
from offline.task28_stage_b_contact.run_c2r import run_child


def test_240_incremental_samples_and_events(tmp_path):
    writer = EvidenceWriter(tmp_path)
    phases = ['PREFLIGHT', 'APPROACH', 'CONTACT', 'FORCE_RAMP', 'EXIT', 'DONE']
    for i in range(240):
        state = phases[min(i // 40, 5)]
        writer.write('sample', {'step': i + 1, 'state': state, 'force': np.float32(i / 1000), 'valid': np.bool_(True)})
        writer.write('event', {'step_at_callback': np.int64(i), 'colliders': ['cube', 'table'], 'impulse': np.float64(.1)})
        assert len((tmp_path / 'timeseries.jsonl').read_text().splitlines()) == i + 1
        assert len((tmp_path / 'contact_events.jsonl').read_text().splitlines()) == i + 1
    result = writer.finish({'status': 'RECORDED', 'preflight': 'PASS'})
    assert result['evidence_complete'] and verify_evidence(tmp_path, 0)[0]
    assert not verify_evidence(tmp_path, 9)[0]
    (tmp_path / 'contact_events.jsonl').write_text('')
    assert not verify_evidence(tmp_path, 0)[0]


@pytest.mark.parametrize('bad', [np.float64(np.nan), np.float64(np.inf), np.float32(-np.inf)])
def test_nonfinite_record_fails_closed(tmp_path, bad):
    writer = EvidenceWriter(tmp_path)
    writer.write('sample', {'step': 1, 'state': 'APPROACH'})
    with pytest.raises(RuntimeError, match='EVIDENCE_WRITE_FAILED'):
        writer.write('event', {'force': bad})
    with pytest.raises(RuntimeError, match='EVIDENCE_ALREADY_FAILED'):
        writer.write('sample', {'step': 2, 'state': 'DONE'})
    result = writer.finish({'status': 'RECORDED', 'preflight': 'PASS'})
    assert result['status'] == 'FAILED' and not result['evidence_complete']
    assert not verify_evidence(tmp_path, 0)[0]


def test_injected_disk_failure_stops_and_summary_is_partial(tmp_path):
    writer = EvidenceWriter(tmp_path)
    writer.write('sample', {'step': 1, 'state': 'PREFLIGHT'})
    writer.streams['contact_events.jsonl'].close()
    with pytest.raises(RuntimeError, match='EVIDENCE_WRITE_FAILED'):
        writer.write('event', {'step': 1})
    summary = writer.finish({'status': 'RECORDED', 'preflight': 'PASS'})
    assert summary['status'] == 'FAILED'
    assert summary['evidence_failure'] is not None
    assert not verify_evidence(tmp_path, 0)[0]


def test_missing_summary_or_corrupt_event_never_passes(tmp_path):
    assert not verify_evidence(tmp_path, 0)[0]
    (tmp_path / 'summary.json').write_text(json.dumps({'status': 'RECORDED'}))
    (tmp_path / 'timeseries.jsonl').write_text('{}\n')
    (tmp_path / 'contact_events.jsonl').write_text('{"bad": NaN}\n')
    assert not verify_evidence(tmp_path, 0)[0]


def test_process_wrapper_rejects_zero_exit_with_missing_evidence(tmp_path):
    output = tmp_path / 'child_run'
    result = run_child([sys.executable, '-c', 'print("uncaught simulator failure")'], output)
    receipt = json.loads((output / 'process_exit.json').read_text())
    assert result == 1 and receipt['child_exit_code'] == 0 and receipt['wrapper_exit_code'] == 1
    assert receipt['evidence_check'] == 'MISSING_OR_INVALID_EVIDENCE'


def test_process_wrapper_preserves_actual_nonzero_exit(tmp_path):
    output = tmp_path / 'child_failure'
    assert run_child([sys.executable, '-c', 'raise SystemExit(7)'], output) == 1
    assert json.loads((output / 'process_exit.json').read_text())['child_exit_code'] == 7
