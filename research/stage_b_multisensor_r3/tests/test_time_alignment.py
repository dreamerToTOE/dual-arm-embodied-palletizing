import numpy as np
import pytest
from research.stage_b_multisensor_r3.time_alignment import RobotHistory


def history(max_samples=240):
    return RobotHistory(lambda side,q: np.diag([1,1,1,1])+np.array([[0,0,0,q[0]**2],[0,0,0,0],[0,0,0,0],[0,0,0,0]]),max_samples=max_samples)


def add(h,t,q=0,state=False):
    h.append(time_s=t,physics_step=round(t*60),q={s:[q]*7 for s in ('left','right')},dq={s:[1.]*7 for s in ('left','right')},suction_observation={'left':state},suction_command={'left':state},controller_command={'sequence':q})


def test_interpolates_joints_recomputes_fk_and_holds_past_discrete_state():
    h=history();add(h,1.,0);add(h,1.02,2,True)
    r=h.at(1.01)
    assert r['status']=='ALIGNED' and r['q']['left'][0]==pytest.approx(1.)
    assert r['tcp_fk']['left'][0][3]==pytest.approx(1.) # not linear interpolation of endpoint FK (2)
    assert r['suction_observation']=={'left':False}
    assert h.at(1.02)['suction_observation']=={'left':True}


def test_no_extrapolation_no_old_evicted_state_and_no_large_gap():
    h=history(2);add(h,1.);add(h,1.02);add(h,1.04)
    assert h.at(1.)['status']=='UNKNOWN'
    assert h.at(1.05)['status']=='UNKNOWN'
    add(h,1.2)
    assert h.at(1.1)['reason']=='physics_history_gap'
    assert h.at(float('nan'))['status']=='UNKNOWN'


def test_rejects_time_reversal_nonfinite_joints_and_oracle_input():
    h=history();add(h,1.)
    with pytest.raises(ValueError):add(h,1.)
    with pytest.raises(ValueError):add(h,1.02,float('nan'))
    with pytest.raises(TypeError):h.append(cube_world_m=[.8,.06,.26])
