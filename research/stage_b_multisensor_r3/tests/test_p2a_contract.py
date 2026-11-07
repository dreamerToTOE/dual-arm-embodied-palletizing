import csv,json
from pathlib import Path
from research.stage_b_multisensor_r3.scripts.evaluate_p2a_alignment import evaluate
from research.stage_b_multisensor_r3.scripts.audit_p2a_occlusion import absence_intervals

R3=Path(__file__).resolve().parents[1]


def test_absence_duration_is_censored_and_not_a_missing_file_claim():
    frames=[{'physics_step':i,'time_s':i/6,'status':s} for i,s in enumerate(('PRESENT','UNKNOWN','PRESENT','UNKNOWN','UNKNOWN'))]
    result=absence_intervals(frames)
    assert not result[0]['right_censored'] and result[0]['observed_absence_span_s']==0
    assert result[1]['right_censored'] and result[1]['missing_acquired_images']==2
    reviewed=list(csv.DictReader((R3/'evidence/p2a_occlusion/107_frame_review.csv').open()))
    assert len(reviewed)==107 and all(r['frame_pair_ok']=='True' for r in reviewed)
    assert sum(r['cause']=='UNKNOWN' for r in reviewed)==19


def test_sdk_time_is_not_silently_certified_or_fed_oracle_for_p2b(tmp_path):
    result=evaluate(R3/'evidence/r3_p2a_dynamic07',tmp_path)
    assert result['paired_common_callback_checks_passed']==result['frames']==30
    assert result['uncertain_measurement_time_samples']==30
    def keys(value):
        if isinstance(value,dict):return set(value).union(*(keys(v) for v in value.values()))
        if isinstance(value,list):return set().union(*(keys(v) for v in value))
        return set()
    for row in map(json.loads,(tmp_path/'p2b_timestamped_interface.jsonl').read_text().splitlines()):
        assert row['measurement_time_s'] is None and not row['online_fusion_eligible']
        assert not {'cube_world_m','cube_quaternion_wxyz','cube_xyz_after_render_wait_m','tcp_physx','tcp_usd','oracle_contact_pairs','cube_velocity_m_s'}.intersection(keys(row))
        assert row['robot_at_reported_time']['status']=='ALIGNED'
