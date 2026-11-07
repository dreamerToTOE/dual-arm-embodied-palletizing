import copy
import json
from pathlib import Path
import numpy as np
import pytest
from research.stage_b_multisensor_r3.history_contract import SynchronizedHistory

CFG = json.loads((Path(__file__).parents[1]/'config/p2b0_frozen.json').read_text())
KEY = dict(run_id='test', epoch_id=0, time_base_id='physics')


def sample(step, epoch=0):
    return dict(KEY, epoch_id=epoch, time_s=step/60, core_sim_time_s=step/60, physics_step=step,
        q={s:[step/100]*7 for s in ('left','right')}, dq={s:[.6]*7 for s in ('left','right')},
        validity={'finite_q_dq':True,'articulation_handles_initialized':True},
        joint_state_source='Isaac4.5 SingleArticulation -> live PhysX articulation joint tensors',
        suction_observation={'left':None,'right':None}, suction_command={'left':None,'right':None},controller_command=None)


def fk(side,q):
    m=np.eye(4);m[0,3]=q[0]**2;return m


def make(data=None, stride=1): return SynchronizedHistory(data or [sample(i) for i in range(1,5)], fk, CFG, stride)


def frame():
    return dict(KEY, frame_id=7, sdk_time_s=3/60, fabric_sim_time_s=3/60,
        callback_core_time_s=3/60, callback_entry_step=3, arrival_physics_time_s=4/60,
        rgb_sha256='a',depth_sha256='b')


TIMES = {i:i/60 for i in range(1,5)}


def test_interpolation_recomputes_fk_and_preserves_unknown_suction():
    r=make().at(KEY, 2.5/60)
    assert r['q']['left'][0] == pytest.approx(.025)
    assert r['tcp_fk']['left'][0][3] == pytest.approx(.025**2)
    assert r['suction_observation']['left'] is None


def test_causal_interval_includes_continuous_candidates_and_missing_prehistory():
    h=make();a=h.associate(frame(),TIMES)
    assert a['measurement_time_s'] is None and not a['online_fusion_eligible']
    assert a['admissible_interval_s']==[0,3/60]
    assert a['unsupported_prefix_s']==[0,1/60]
    assert a['interpolation_domains_s']==[[1/60,2/60],[2/60,3/60]]
    assert h.at(KEY,0)['status']=='UNKNOWN'


@pytest.mark.parametrize('change,reason',[
    ({'arrival_physics_time_s':1.},'stale_arrival'),
    ({'sdk_time_s':.9},'inconsistent_frame_clock'),
    ({'sdk_time_s':.9,'fabric_sim_time_s':.9},'inconsistent_frame_clock'),
    ({'fabric_sim_time_s':None},'missing_or_invalid_frame_clock'),
    ({'time_base_id':'different'},'incompatible_run_epoch_or_clock'),
    ({'epoch_id':1},'incompatible_run_epoch_or_clock'),
    ({'run_id':'other'},'incompatible_run_epoch_or_clock'),
    ({'time_base_id':None},'missing_time_base_identity'),
    ({'arrival_physics_time_s':0.},'arrival_precedes_callback'),
    ({'callback_entry_step':99},'missing_callback_physics_time'),
])
def test_reject_bad_frame(change,reason):
    f=frame();f.update(change)
    assert make().associate(f,TIMES)['reason']==reason


def test_duplicate_identity_and_conflicting_duplicate():
    h=make();f=frame();h.associate(f,TIMES)
    assert h.associate(f,TIMES)['reason']=='duplicate_frame'
    f['depth_sha256']='changed'
    assert h.associate(f,TIMES)['reason']=='conflicting_duplicate_frame'


def test_missing_joint_sample_is_not_silently_interpolated():
    h=make([sample(1),sample(3)])
    assert h.at(KEY,2/60)['reason']=='missing_joint_sample'
    assert h.at(KEY,3/60)['status']=='ALIGNED'
    assert make([sample(1),sample(3)],2).at(KEY,2/60)['status']=='ALIGNED'


def test_no_interpolation_across_command_transition_but_measured_endpoints_remain_valid():
    a,b=sample(1),sample(2);b['controller_command']={'sequence':1}
    h=make([a,b])
    assert h.at(KEY,1.5/60)['reason']=='command_transition_interpolation_unsupported'
    assert h.at(KEY,2/60)['status']=='ALIGNED'
    assert h.associate(frame(),TIMES)['interpolation_domains_s']==[]


def test_reset_requires_new_epoch_and_never_crosses_it():
    with pytest.raises(ValueError,match='unmarked_reset'):make([sample(2),sample(1)])
    h=make([sample(2),sample(3),sample(1,epoch=1),sample(2,epoch=1)])
    assert h.at(KEY,1/60)['status']=='UNKNOWN'
    assert h.at(dict(KEY,epoch_id=1),1/60)['status']=='ALIGNED'


@pytest.mark.parametrize('change',[
    {'cube_gt':[0,0,0]}, {'cube_world_m':[0,0,0]}, {'tcp_physx':[]},
    {'joint_state_source':'commanded_position'}, {'core_sim_time_s':9.},
    {'validity':{'finite_q_dq':False,'articulation_handles_initialized':True}},
    {'q':{'left':[float('nan')]*7,'right':[0]*7}},
])
def test_reject_oracle_nonfinite_commands_and_bad_source(change):
    r=sample(1);r.update(change)
    with pytest.raises(ValueError):make([r])


def test_dropout_does_not_mutate_robot_history_and_never_extrapolates():
    h=make();before=copy.deepcopy(list(h.epochs[tuple(KEY.values())].rows))
    assert h.at(KEY,5/60)['status']=='UNKNOWN'
    assert list(h.epochs[tuple(KEY.values())].rows)==before
    assert h.at(KEY,float('nan'))['status']=='UNKNOWN'
