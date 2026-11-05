import gzip,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def load():
    with gzip.open(ROOT/'dataset/r2_timeline.jsonl.gz','rt') as f: return {int((x:=json.loads(l))['physics_step']):x for l in f if l.strip()}

def test_timeline_alignment_and_unknowns():
    d=load(); assert len(d)==15688
    assert d[1]['oracle_reference']['edge_states']['CUBE_TABLE']=='OBSERVED_ACTIVE'
    assert d[2152]['oracle_reference']['edge_states']['LEFT_TOOL_CUBE']=='OBSERVED_ACTIVE'
    assert d[9236]['oracle_reference']['edge_states']['CUBE_DEEP_WALL']=='OBSERVED_ACTIVE'
    for step in (1,2152,9236):
        r=d[step]; assert r['oracle_reference']['edge_states']['CUBE_SIDE_WALL']=='UNKNOWN'
        assert r['estimator_input']['edge_observations']['CUBE_SIDE_WALL']=='UNKNOWN'
        assert r['estimator_input']['suction_command'] is None
        assert r['estimator_input']['command_advance_m'] is None
        assert 'oracle_reference' not in r['estimator_input']
