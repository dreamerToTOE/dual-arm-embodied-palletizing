import numpy as np
from research.stage_b_multisensor_r3.r2_scene_binding import fit_box_points,scene_time_interface


def test_two_visible_cube_faces_recover_pixel_geometry_and_single_face_is_rejected():
    rng=np.random.default_rng(26);a=rng.uniform(-.055,.055,200);z=rng.uniform(.215,.305,200)
    xface=np.column_stack((np.full(200,.90),a+.06,z));yface=np.column_stack((a+.84,np.zeros(200),z))
    points=np.concatenate((xface,yface))+rng.normal(0,.00002,(400,3))
    fit=fit_box_points(points,[.842,.058,.02]);assert fit['status']=='PRESENT'
    assert np.linalg.norm(np.array(fit['xyyaw'])[:2]-[.84,.06])<.0001
    assert fit_box_points(xface,[.84,.06,0])['status']=='UNCERTAIN'


def test_timing_hypotheses_are_preserved_and_unpaired_or_unknown_epochs_reject():
    frame={'frame_id':1,'configuration_fingerprint':'a','cube_matches_known_stimulus':True};clock={s:{'paired':True,'rgb_code':10} for s in ('app','pre','post')}
    binding={'status':'FROZEN_CALIBRATION','configuration_fingerprint':'a','cube_minus_clock_candidate_ticks':[-2,0]}
    result=scene_time_interface(frame,clock,{8:8/60,10:10/60},binding)
    assert result['measurement_time_s'] is None and result['corresponding_physics_step'] is None
    assert [c['physics_step'] for c in result['candidates']]==[8,10]
    assert scene_time_interface(dict(frame,cube_matches_known_stimulus=False),clock,{10:10/60},binding)['timing_status']=='UNCERTAIN'
    clock['pre']['paired']=False
    assert scene_time_interface(frame,clock,{10:10/60},binding)['timing_status']=='UNCERTAIN'


def test_depth_pixel_center_is_verified_by_known_clock_geometry_not_cube_truth():
    import json
    from pathlib import Path
    r3=Path(__file__).resolve().parents[1];run=r3/'evidence/r2_calibration01';m=json.loads((run/'manifest.json').read_text())
    observations=[json.loads(s) for s in (run/'observations.jsonl').read_text().splitlines()]
    for index in (3,9,15):
        row=observations[index];depth=np.load(run/row['depth'])['distance_to_image_plane']
        for source in ('app','pre','post'):
            for inverse,(u,v) in enumerate(m['clock_layout'][source][15]):
                bit=((row['expected_pattern']>>15)&1)^inverse;expected=(.89 if bit else .85)-.001
                _,us=np.nonzero(abs(depth[v-9:v+10,u-9:u+10]-expected)<.00005)
                bounds=[us.min()+u-9+m['capture_roi'][0],us.max()+u-9+m['capture_roi'][0]]
                assert bounds==[632,647]
                assert sum(bounds)/2+.5==m['K'][0][2]


def test_real_holdout_predictions_and_hypotheses_exist_before_oracle_is_opened(tmp_path,monkeypatch):
    import hashlib,json,pytest
    from pathlib import Path
    from research.stage_b_multisensor_r3.scripts import evaluate_r2_scene_binding as evaluator
    r3=Path(__file__).resolve().parents[1];binding=r3/'config/r2_scene_binding_frozen.json';digest=hashlib.sha256(binding.read_bytes()).hexdigest()
    original=evaluator.rows
    class OracleBoundary(Exception):pass
    def stop_at_oracle(path):
        if path.name.startswith('oracle'):
            assert (tmp_path/'replay/pixel_predictions.jsonl').exists()
            assert (tmp_path/'replay/time_binding_interface.jsonl').exists()
            assert hashlib.sha256(binding.read_bytes()).hexdigest()==digest
            raise OracleBoundary()
        return original(path)
    monkeypatch.setattr(evaluator,'rows',stop_at_oracle)
    with pytest.raises(OracleBoundary):evaluator.evaluate(r3/'evidence/r2_holdout01',tmp_path/'replay',binding_path=binding)
    produced=original(tmp_path/'replay/time_binding_interface.jsonl');expected=original(r3/'evidence/r2_holdout01_analysis/time_binding_interface.jsonl')
    assert len(produced)==len(expected)==180
    assert sum(r['timing_status']=='MULTI_HYPOTHESIS_WITH_HISTORICAL_ALIASES' for r in produced)>100
    assert max(max(c['time_s'] for c in r['candidates'])-min(c['time_s'] for c in r['candidates']) for r in produced if r['candidates'])>6
    upper={r['frame_id']:r['callback_entry_step'] for r in original(tmp_path/'replay/pixel_predictions.jsonl')}
    for result,reference in zip(produced,expected):
        assert result['candidates']==reference['candidates']
        assert all(c['physics_step']<=upper[result['frame_id']] for c in result['candidates'])
        assert result['metadata_hypotheses']==reference['metadata_hypotheses']
        assert result['measurement_time_s'] is None and result['corresponding_physics_step'] is None
        if result['cube_pose'].get('cube_xyz_m') is not None:
            np.testing.assert_allclose(result['cube_pose']['cube_xyz_m'],reference['cube_pose']['cube_xyz_m'],atol=1e-10)
