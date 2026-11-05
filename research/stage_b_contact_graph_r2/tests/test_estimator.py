import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from estimator.r2_contact_estimator import *

def obs(step, *, dist=.2, vx=0., command=None, gap=.5, state=ObservationState.OBSERVED_ACTIVE):
    return FrameObservation(step, step*.01, (0.,0.,.26), (vx,0.,0.), (dist,0.,.26), None,
        command_advance_m=command, observation_state=state,
        features={'tool_cube_distance_m':dist,'deep_wall_gap_m':gap})

def test_e0_normal_tool_contact_and_wall():
    e=E0GeometricMotionRules()
    assert e.estimate(obs(1,dist=.2)).state is TaskState.FREE_SPACE_APPROACH
    assert e.estimate(obs(2,dist=.05)).state is TaskState.TOOL_CONTACT
    assert e.estimate(obs(3,dist=.05,vx=.02)).state is TaskState.NORMAL_ADVANCEMENT
    assert e.estimate(obs(4,dist=.2,gap=.005)).state is TaskState.DEEP_WALL_ARRIVAL

def test_e0_unknown_and_no_false_jam_without_command():
    e=E0GeometricMotionRules()
    assert e.estimate(obs(1,state=ObservationState.UNKNOWN)).state is TaskState.OBSERVATION_UNAVAILABLE
    assert e.estimate(obs(2,dist=.05,vx=0.,command=None)).state is TaskState.TOOL_CONTACT
    assert e.estimate(obs(3,dist=.05,vx=0.,command=.1)).state is TaskState.SUSPECTED_PREMATURE_BLOCKAGE

def test_e1_dwell_is_causal():
    e=E1TemporalHysteresis(activation_frames=3, release_frames=2)
    assert e.estimate(obs(1)).state is TaskState.UNKNOWN
    assert e.estimate(obs(2,dist=.05)).state is TaskState.UNKNOWN
    assert e.estimate(obs(3,dist=.05)).state is TaskState.UNKNOWN
    assert e.estimate(obs(4,dist=.05)).state is TaskState.TOOL_CONTACT
    assert e.estimate(obs(5)).state is TaskState.TOOL_CONTACT
    assert e.estimate(obs(6)).state is TaskState.FREE_SPACE_APPROACH

def test_tracking_failure_preempts_state():
    e=E1TemporalHysteresis()
    o=obs(1,dist=.05); assert e.estimate(o).state is TaskState.UNKNOWN
    o=FrameObservation(2,.02,(0,0,.26),(0,0,0),(.05,0,.26),None,features={'tool_cube_distance_m':.05,'tracking_error_m':.1})
    assert e.estimate(o).state is TaskState.TRACKING_FAILURE

def test_live_adapter_ignores_oracle_reference():
    from estimator.live_adapter import estimate_runtime_row
    row={'physics_step':1,'time_s':0.1,'cube_world_m':[0,0,.26],'cube_velocity_m_s':[0,0,0],
         'tcp_world_m':{'left':[.05,0,.26],'right':[.4,0,.26]},
         'estimator_input':{'observation_state':'OBSERVED_ACTIVE','edge_observations':{'CUBE_SIDE_WALL':'UNKNOWN'},
                            'features':{'tool_cube_distance_m':.05,'deep_wall_gap_m':.5},'command_advance_m':None},
         'oracle_reference':{'edge_states':{'LEFT_TOOL_CUBE':'OBSERVED_ACTIVE'}}}
    assert estimate_runtime_row(row).state is TaskState.TOOL_CONTACT
