import json
from pathlib import Path
import numpy as np
import pytest
from research.stage_b_multisensor_r3.history_contract_r1 import ReadinessHistory, interpolate_rows
from research.stage_b_multisensor_r3.tests.test_history_contract import sample, fk, frame, KEY, TIMES

CFG=json.loads((Path(__file__).parents[1]/'config/p2b0_r1_frozen.json').read_text())


def epoch():
    return dict(KEY,camera_stream_id='fresh:0',birth_time_s=1/60,robot_first_time_s=1/60,
        birth_code=41809,fresh_render_product=True,render_product_created_time_s=1/60)


def test_robot_first_epoch_covers_entire_causal_range_without_fabricated_samples():
    h=ReadinessHistory([sample(i) for i in range(1,5)],fk,CFG);f=dict(frame(),camera_stream_id='fresh:0')
    r=h.associate_epoch(f,TIMES,epoch(),{'paired':True,'rgb_code':41809})
    assert r['admissible_interval_s']==[1/60,3/60] and r['full_grid_robot_coverage']
    assert r['robot_acquisition_unsupported_duration_s']==pytest.approx(0.)
    assert r['measurement_time_s'] is None and not r['online_fusion_eligible']
    assert h.exact_at(KEY,1.5/60)['reason']=='no_exact_measured_sample_no_snapping'
    assert h.exact_at(KEY,2/60)['status']=='ALIGNED'


@pytest.mark.parametrize('change,birth,reason',[
    ({'camera_stream_id':'retired'}, {'paired':True,'rgb_code':41809},'retired_or_incompatible_camera_stream'),
    ({'epoch_id':1}, {'paired':True,'rgb_code':41809},'retired_or_incompatible_camera_stream'),
    ({}, {'paired':True,'rgb_code':23718},'old_or_unproven_image_birth'),
    ({}, {'paired':False,'rgb_code':41809},'old_or_unproven_image_birth'),
    ({'sdk_time_s':0.,'fabric_sim_time_s':0.}, {'paired':True,'rgb_code':41809},'inconsistent_frame_clock'),
])
def test_old_buffers_and_relabelled_pixels_rejected(change,birth,reason):
    h=ReadinessHistory([sample(i) for i in range(1,5)],fk,CFG);f=dict(frame(),camera_stream_id='fresh:0');f.update(change)
    assert h.associate_epoch(f,TIMES,epoch(),birth)['reason']==reason


def test_no_lower_bound_without_independent_epoch_birth_evidence():
    h=ReadinessHistory([sample(i) for i in range(1,5)],fk,CFG);e=epoch();e['fresh_render_product']=False
    assert h.associate_epoch(dict(frame(),camera_stream_id='fresh:0'),TIMES,e,{'paired':True,'rgb_code':41809})['reason']=='unproven_epoch_lower_bound'


def test_missing_step_is_quantified_and_never_filled_by_exact_mode():
    h=ReadinessHistory([sample(1),sample(3),sample(4)],fk,CFG)
    r=h.associate_epoch(dict(frame(),camera_stream_id='fresh:0'),TIMES,epoch(),{'paired':True,'rgb_code':41809})
    assert not r['full_grid_robot_coverage'] and r['robot_acquisition_unsupported_duration_s']==pytest.approx(2/60)
    assert h.exact_at(KEY,2/60)['status']=='UNKNOWN'


def test_hermite_position_and_analytic_velocity_reproduce_cubic():
    a,b=sample(1),sample(3);dt=b['time_s']-a['time_s'];u=.37
    for r,x in ((a,0.),(b,dt)):
        r['q']={s:[x**3]*7 for s in ('left','right')};r['dq']={s:[3*x*x]*7 for s in ('left','right')}
    r=interpolate_rows(a,b,a['time_s']+u*dt,'hermite',fk)
    assert r['q']['left'][0]==pytest.approx((u*dt)**3)
    assert r['dq']['left'][0]==pytest.approx(3*(u*dt)**2)
    assert r['tcp_fk']['left'][0][3]==pytest.approx((u*dt)**6)


@pytest.mark.parametrize('method',['linear','hermite'])
def test_interpolation_retains_endpoints_and_rejects_extrapolation_reset(method):
    a,b=sample(1),sample(3)
    for row in (a,b):
        r=interpolate_rows(a,b,row['time_s'],method,fk)
        assert np.allclose(r['q']['left'],row['q']['left']) and np.allclose(r['dq']['left'],row['dq']['left'])
    assert interpolate_rows(a,b,0.,method,fk)['status']=='UNKNOWN'
    b['epoch_id']=1
    assert interpolate_rows(a,b,2/60,method,fk)['reason']=='epoch_boundary'


def test_existing_numerical_thresholds_unchanged():
    old=json.loads((Path(__file__).parents[1]/'config/p2b0_frozen.json').read_text())
    for k in old:
        if 'tolerance' in k:assert CFG[k]==old[k]
