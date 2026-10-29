import json
from pathlib import Path
import numpy as np
from offline.task28_p5_qp_6_r7.negative_controls import validate_mass, validate_command
from offline.task28_p5_qp_6_r7.effort_adapter import R7ExclusiveEffortAdapter, Reject

def test_source_limits_are_joint_specific():
 cfg=json.loads(Path('offline/task28_p5_qp_6_r7/source_contract.json').read_text())
 assert cfg['limits']['source_effort_abs_max_Nm']==[87,87,87,87,12,12,12]
 assert cfg['limits']['effective_effort_Nm'] if 'effective_effort_Nm' in cfg['limits'] else True

def test_mass_negative_control():
 M=np.eye(9); assert validate_mass(M)
 bad=M.copy(); bad[0,0]=-1; assert not validate_mass(bad)
 bad2=M.copy(); bad2[0,1]=2; assert not validate_mass(bad2)

def test_rate_and_limit_negative_controls():
 lim=np.array([6,6,6,6,4,4,4.]); rate=np.array([120,120,120,120,80,80,80.]); prev=np.zeros(7)
 assert validate_command(np.ones(7),prev,lim,rate)
 assert not validate_command(np.array([7,0,0,0,0,0,0.]),prev,lim,rate)
 assert not validate_command(np.array([4,0,0,0,0,0,0.]),prev,lim,rate)

def test_research_adapter_rejects_expanded_joint5_limit():
 try: R7ExclusiveEffortAdapter({},None,None,None,lambda x:None,np.array([6,6,6,6,13,4,4]),np.ones(7))
 except Reject as e: assert str(e)=='RESEARCH_LIMIT_EXCEEDS_SOURCE'
 else: raise AssertionError
