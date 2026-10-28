import json
from pathlib import Path

def test_known_load_evaluator_passes_independent_cases():
    p=Path(__file__).parents[1]/'evidence/online_known_load/evaluation.json'
    d=json.loads(p.read_text())
    assert d['status']=='ONLINE_EXPLICIT_CANDIDATE_PASS'
    assert d['gate_pass'] is True
    assert set(d['cases'])=={'zero_external','positive_fx','negative_fx'}
    assert d['global_metrics']['min_rank']==6

def test_known_load_errors_are_bounded():
    d=json.loads((Path(__file__).parents[1]/'evidence/online_known_load/evaluation.json').read_text())
    assert d['global_metrics']['max_force_error_n'] < 0.5
    assert d['global_metrics']['max_moment_error_nm'] < 0.2
