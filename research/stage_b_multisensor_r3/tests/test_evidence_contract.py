import json
from pathlib import Path
import shutil
import pytest
from research.stage_b_multisensor_r3.scripts.audit_r3_p0 import audit_run,count_files
from research.stage_b_multisensor_r3.scripts.evaluate_planar_pose import evaluate

R3=Path(__file__).resolve().parents[1]


def test_missing_raw_sources_are_explicitly_unavailable(tmp_path):
    compact=tmp_path/'compact';compact.mkdir()
    record=audit_run('absent',compact,None)
    assert record['raw_runtime_rows'] is None
    assert record['raw_rgb_files']==0
    assert record['runtime_sha256'] is None
    assert count_files(None)==0


def test_frozen_holdout_uses_separate_oracle_and_common_acceptance(tmp_path):
    result,rows,labels=evaluate(R3/'evidence/static_pose_dataset',R3/'config/p1_frozen.json',tmp_path)
    assert result['matched_acceptance_v0']['accepted_table_scored']==result['matched_acceptance_v1']['accepted_table_scored']==12
    assert result['matched_acceptance_v1']['xy_error_m']['p95'] < result['matched_acceptance_v0']['xy_error_m']['p95']
    assert result['v1']['invalid_support_false_accepts']>=1 # retained counterexample, not hidden success
    assert (tmp_path/'r3_static_holdout01_predictions.jsonl').exists()
    assert all('oracle' not in row and 'on_table' not in row for row in rows)


def test_edited_observation_is_rejected_by_manifest_hash(tmp_path):
    ds=tmp_path/'dataset';shutil.copytree(R3/'evidence/static_pose_dataset',ds)
    with (ds/'observations.jsonl').open('a') as f:f.write('\n')
    with pytest.raises(AssertionError,match='observation hash mismatch'):
        evaluate(ds,R3/'config/p1_frozen.json',tmp_path/'out')


def test_no_gt_velocity_or_gt_phase_in_r3_observations():
    for dataset in ('static_pose_dataset','physical06_holdout','physical07_development'):
        for line in (R3/'evidence'/dataset/'observations.jsonl').read_text().splitlines():
            row=json.loads(line)
            assert not {'cube_world_m','cube_velocity_m_s','observed_cube_velocity_x_m_s','phase','oracle_contact_pairs'}.intersection(row)
