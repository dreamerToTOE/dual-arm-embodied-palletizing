import json
from pathlib import Path

ROOT = Path(__file__).parents[3]
EVIDENCE = ROOT / 'offline/task28_r14/evidence'


def test_g2_closeout_is_frozen_and_limited():
    d = json.loads((EVIDENCE/'g2_results.json').read_text())
    assert d['baseline_head'] == '72fbb8d5ea09ac9edb0a3869a07bed01e243741c'
    assert d['episode_count'] == 8 and d['computed_qp_calls'] == 1680
    assert d['continuous_approach_safety'] == 'CONTINUOUS_SAFETY_NOT_ESTABLISHED'
    assert d['robustness_status'] == 'NOT_ESTABLISHED'
    assert d['runtime_acceptance_claim'] is False
    assert d['cube_contact_or_motion_commands'] == 0
    assert all(e['summary']['solver_status_counts'] == {'QP_OPTIMAL': 210} for e in d['episodes'])


def test_no_contact_candidate_is_source_bound_and_no_force():
    d = json.loads((EVIDENCE/'no_contact/attempt_01/evaluation.json').read_text())
    assert d['status'] == 'NO_CONTACT_SINGLE_POSE_CANDIDATE_PASS'
    assert d['fresh_simulation_runs'] == 1 and d['no_contact_pose_only'] is True
    assert d['external_load_tested'] is False
    assert d['contact_force_observation_validated'] is False
    assert d['closed_loop_force_control'] is False
    assert d['observer_status_counts'] == {'WARMUP': 1, 'CANDIDATE_ESTIMATE': 239}
    life = json.loads((EVIDENCE/'no_contact/attempt_01/lifecycle.json').read_text())
    scope = json.loads((EVIDENCE/'no_contact/attempt_01/runtime_scope.json').read_text())
    assert life['clean_exit'] and life['isolation_verified'] and life['portable_resources_removed']
    assert scope['guarded_commands'] == 240 and scope['external_force_calls'] == 0


def test_contact_and_transport_are_not_admitted():
    d = json.loads((EVIDENCE/'g3_stage_admission.json').read_text())
    by_id = {x['id']: x for x in d['stages']}
    assert all(by_id[x]['can_start_with_current_evidence'] is False for x in ('S2','S3','S4'))
    assert all(by_id[x]['may_apply_contact'] is False for x in ('S2','S3','S4'))
    assert by_id['S4']['original_approach_reuse_forbidden'] is True

