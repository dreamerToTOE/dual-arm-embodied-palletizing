import numpy as np
import pytest
from research.stage_b_multisensor_r3.r1_time_code import code_bits,decode_clock,timestamp_interface,freeze_offset_interval,SOURCES


def synthetic(value):
    rgb=np.zeros((12,100,3),np.uint8);depth=np.zeros((12,100),np.float32);layout={}
    for source in SOURCES:
        pairs=[]
        for i in range(16):
            pair=[]
            for j in range(2):
                u,v=3+i*6,3+j*6;b=code_bits(value)[j,i]
                rgb[v-1:v+2,u-1:u+2]=240 if b else 5
                depth[v-1:v+2,u-1:u+2]=.889 if b else .849;pair.append([u,v])
            pairs.append(pair)
        layout[source]=pairs
    return rgb,depth,layout


@pytest.mark.parametrize('value',[0,65535,43690,21845,4660,32769,179,731])
def test_independent_rgb_depth_complement_decode(value):
    assert code_bits(value).sum()==16
    result=decode_clock(*synthetic(value))
    assert all(r['rgb_code']==r['depth_code']==value and r['paired'] for r in result.values())


def test_missing_contrast_and_cross_channel_mismatch_are_not_certified():
    rgb,depth,layout=synthetic(4);u,v=layout['app'][0][0];rgb[v-1:v+2,u-1:u+2]=240
    assert not decode_clock(rgb,depth,layout)['app']['paired']
    rgb,depth,layout=synthetic(4);depth[2:5,2:5]=np.nan
    assert decode_clock(rgb,depth,layout)['post']['depth_code'] is None
    assert decode_clock(rgb,depth,layout)['post']['depth_min_complement_difference_m'] is None
    with pytest.raises(ValueError):code_bits(65536)


def test_interface_requires_frozen_binding_fingerprint_and_finite_time():
    binding={'status':'FROZEN_CALIBRATION','configuration_fingerprint':'a','offset_interval_s':[.02,.03]}
    obs={'frame_id':1,'fabric_sim_time_s':1.,'configuration_fingerprint':'a','time_binding_ready':True}
    r=timestamp_interface(obs,binding)
    assert r['measurement_time_interval_s']==[.97,.98] and not r['online_fusion_eligible']
    assert timestamp_interface(dict(obs,configuration_fingerprint='b'),binding)['measurement_time_interval_s'] is None
    assert timestamp_interface(dict(obs,fabric_sim_time_s=float('nan')),binding)['measurement_time_interval_s'] is None
    assert timestamp_interface(dict(obs,time_binding_ready=False),binding)['measurement_time_interval_s'] is None
    assert r['measurement_time_s'] is None


def test_variable_delay_retains_quantization_and_does_not_become_constant_offset():
    tick=1/60
    assert freeze_offset_interval([0,2*tick,2*tick],tick)==[-tick,2*tick]
    with pytest.raises(ValueError):freeze_offset_interval([float('nan')],tick)
    with pytest.raises(ValueError):freeze_offset_interval([],tick)


def test_committed_independent_holdout_replays_without_oracle_input_to_predictions(tmp_path,monkeypatch):
    import hashlib,json
    from pathlib import Path
    from research.stage_b_multisensor_r3.scripts import evaluate_r1_time_binding as evaluator
    r3=Path(__file__).resolve().parents[1];run=r3/'evidence/r1_holdout01';binding=r3/'config/r1_time_binding_frozen.json'
    digest=hashlib.sha256(binding.read_bytes()).hexdigest();original=evaluator.rows
    def without_oracle(path):
        if path.name=='oracle.jsonl':
            assert (tmp_path/'replay/timestamp_interface.jsonl').exists()
            assert hashlib.sha256(binding.read_bytes()).hexdigest()==digest
            return []
        return original(path)
    monkeypatch.setattr(evaluator,'rows',without_oracle)
    result=evaluator.evaluate(run,tmp_path/'replay',binding_path=binding)
    assert result['decision']=='PARTIAL'
    assert result['full_phase_coverage'] and result['independent_holdout']
    assert result['prediction_sha256']==hashlib.sha256((r3/'evidence/r1_holdout01_analysis/timestamp_interface.jsonl').read_bytes()).hexdigest()
    assert result['valid_unique_dynamic_frames']==136
    assert result['empirical_interval_contains_visual_interval']=={'covered':136,'total':136}
    assert result['worst_midpoint_error_bound_ms']==pytest.approx(25,abs=.001)
    exported=[json.loads(line) for line in (tmp_path/'replay/timestamp_interface.jsonl').read_text().splitlines()]
    assert len(exported)==137 and sum(r['timing_status']=='UNCERTAIN' for r in exported)==1
    assert all(r['measurement_time_s'] is None and not r['online_fusion_eligible'] for r in exported)
