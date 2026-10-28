"""Fault controls for stale/unsafe R5 states; no simulator or actuator writes."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from .source_gate import R5Gate,Rejected,torque_attempt
ROOT=Path(__file__).resolve().parent

def state(step=1,time=1/60):
 return {'identity':'left','step':step,'time':time,'q':np.zeros(9).tolist(),'dq':np.zeros(9).tolist(),'M':np.eye(9).tolist(),'tcp_frame':'original_side_suction_tcp','effort_mode':'force','competing_sink':False,'unexpected_contact':False,'fresh_copy':True,'solver_semantics_ack':True,'delayed_buffer':False}
def main():
 cases=[]
 def one(name,mutate,expected,prev=None):
  x=state();mutate(x);g=R5Gate()
  try:g.validate(x,prev);out,reason='UNEXPECTED_PASS',None
  except Rejected as e:out,reason='REJECTED',str(e)
  cases.append({'name':name,'expected':expected,'outcome':out,'reason':reason,'commands':0})
 prev=state(step=1,time=0)
 one('stale_solver_q',lambda x:x.update({'step':2,'dq':[.02]*9,'stale_solver_state':True}),'STALE_SOLVER_STATE',prev) # no semantics bypass if control source lacks declaration
 one('delayed_solver_state',lambda x:x.update({'step':2,'delayed_buffer':True}),'DELAYED_SOURCE',prev)
 one('broken_time_sync',lambda x:x.update({'step':2,'time':.2}),'TIME_SYNC',prev)
 one('wrong_identity',lambda x:x.update({'identity':'right'}),'WRONG_IDENTITY')
 one('invalid_effort_command_mode',lambda x:x.update({'effort_mode':'position'}),'INVALID_EFFORT_MODE')
 one('invalid_mass',lambda x:x.update({'M':[[(-1.0 if i==j==0 else float(i==j)) for j in range(9)] for i in range(9)]}),'INVALID_MASS')
 one('nonfinite_or_negative_mass',lambda x:x.update({'M':[[float('nan') if i==j==0 else float(i==j) for j in range(9)] for i in range(9)]}),'INVALID_MASS')
 one('wrong_tcp',lambda x:x.update({'tcp_frame':'link8'}),'WRONG_TCP')
 one('unexpected_contact',lambda x:x.update({'unexpected_contact':True}),'UNEXPECTED_CONTACT')
 one('competing_position_sink',lambda x:x.update({'competing_sink':True}),'COMPETING_SINK')
 one('nonfresh_copy',lambda x:x.update({'fresh_copy':False}),'NOT_FRESH_COPY')
 one('semantics_not_declared',lambda x:x.update({'solver_semantics_ack':False}),'SEMANTICS_UNDECLARED')
 g=R5Gate()
 try:g.reject('PRE_COMMAND_REJECTED')
 except Rejected:pass
 try:torque_attempt(g);latch={'outcome':'UNEXPECTED_PASS','reason':None,'commands':0}
 except Rejected as e:latch={'outcome':'REJECTED','reason':str(e),'commands':0}
 cases.append({'name':'actuation_after_latched_failure','expected':'LATCHED:PRE_COMMAND_REJECTED',**latch})
 result={'status':'ALL_R5_NEGATIVE_CONTROLS_REJECTED','cases':cases,'robot_command_writes':0,'suction_commands':0}
 if any(x['outcome']!='REJECTED' or x['reason']!=x['expected'] for x in cases):result['status']='NEGATIVE_CONTROL_FAILURE'
 out=ROOT/'evidence'/'negative_controls.json';out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(json.dumps(result,indent=2,sort_keys=True));return 0 if result['status']=='ALL_R5_NEGATIVE_CONTROLS_REJECTED' else 1
if __name__=='__main__':raise SystemExit(main())
