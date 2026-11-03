"""Incremental, fail-closed evidence for the supported-contact probe."""
import json
from pathlib import Path

from offline.task28_stage_b_contact.c2_control import json_record


class EvidenceWriter:
    def __init__(self, root):
        self.root = Path(root)
        self.streams = {name: (self.root / name).open('x', encoding='utf-8')
                        for name in ('timeseries.jsonl', 'contact_events.jsonl')}
        self.failed = None
        self.samples = self.events = 0
        self.last_state = None

    def write(self, kind, record):
        if self.failed:
            raise RuntimeError('EVIDENCE_ALREADY_FAILED')
        name = {'sample': 'timeseries.jsonl', 'event': 'contact_events.jsonl'}[kind]
        try:
            encoded = json_record(record) + '\n'
            self.streams[name].write(encoded)
            self.streams[name].flush()
        except Exception as exc:
            self.failed = f'{kind}: {exc!r}'
            raise RuntimeError('EVIDENCE_WRITE_FAILED') from exc
        if kind == 'sample':
            self.samples += 1
            self.last_state = record.get('state')
        else:
            self.events += 1

    def finish(self, summary):
        for stream in self.streams.values():
            stream.close()
        summary = dict(summary, samples=self.samples, events=self.events, evidence_failure=self.failed)
        complete = (self.failed is None and self.samples > 0 and self.events > 0 and
                    self.last_state == 'DONE' and summary.get('preflight') == 'PASS')
        if summary.get('status') == 'RECORDED' and not complete:
            summary.update(status='FAILED', reason='INCOMPLETE_EVIDENCE')
        summary['evidence_complete'] = bool(complete)
        with (self.root / 'summary.json').open('x', encoding='utf-8') as stream:
            stream.write(json_record(summary) + '\n')
        return summary


def verify_evidence(root, child_exit):
    root = Path(root)
    try:
        def reject_nonfinite(value):
            raise ValueError(f'non-finite JSON constant: {value}')
        def read(value):
            return json.loads(value, parse_constant=reject_nonfinite)
        summary = read((root / 'summary.json').read_text())
        rows = [read(x) for x in (root / 'timeseries.jsonl').read_text().splitlines()]
        events = [read(x) for x in (root / 'contact_events.jsonl').read_text().splitlines()]
        phases = list(dict.fromkeys(row['state'] for row in rows))
        valid = (child_exit == 0 and summary['status'] == 'RECORDED' and summary['evidence_complete'] is True and
                 summary['preflight'] == 'PASS' and len(rows) == summary['samples'] and len(events) == summary['events'] and
                 len(rows) > 0 and len(events) > 0 and rows[-1]['state'] == 'DONE' and
                 phases == ['PREFLIGHT', 'APPROACH', 'CONTACT', 'FORCE_RAMP', 'EXIT', 'DONE'] and
                 all(row['step'] == i + 1 for i, row in enumerate(rows)) and
                 all(0 <= event['step_at_callback'] < len(rows) for event in events))
        return bool(valid), 'COMPLETE' if valid else 'INCOMPLETE_OR_FAILED'
    except (OSError, ValueError, KeyError, TypeError):
        return False, 'MISSING_OR_INVALID_EVIDENCE'
