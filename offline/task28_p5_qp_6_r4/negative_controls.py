"""Execute R4 source fault-injection controls without a simulator or actuators."""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np
from .source_gate import LatchedSourceGate, SourceRejected, torque_attempt
ROOT=Path(__file__).resolve().parent
def base(step=1,time=1.0/60.0):
    return {"identity":"left","physics_step":step,"time_s":time,"q":np.zeros(9).tolist(),"dq":np.zeros(9).tolist(),"M":np.eye(9).tolist(),"tcp_frame":"original_side_suction_tcp","effort_mode":"force","position_sink_active":False,"unexpected_collision":False,"shared_mutable_buffer":False}
def main():
    cases=[]
    def run(name,mutate,expected,previous=None):
        current=base();mutate(current);gate=LatchedSourceGate()
        try: gate.validate(current,previous=previous,expected_identity="left");outcome,reason="UNEXPECTED_PASS",None
        except SourceRejected as exc: outcome,reason="REJECTED",str(exc)
        cases.append({"name":name,"expected":expected,"outcome":outcome,"reason":reason,"commands":0})
    prev=base(step=1,time=0.0);run("stale_joint_q",lambda x:x.update({"physics_step":2,"dq":[0.02]*9}),"STALE_Q_NONZERO_DQ",previous=prev)
    prev_dq=base(step=1,time=0.0);prev_dq["q"][0]=0.1;run("stale_joint_dq",lambda x:x.update({"physics_step":2,"q":[0.2]+[0.0]*8}),"STALE_DQ",previous=prev_dq)
    run("shared_mutable_source_array",lambda x:x.update({"shared_mutable_buffer":True}),"SHARED_MUTABLE_BUFFER")
    run("wrong_physics_step",lambda x:x.update({"physics_step":5}),"WRONG_PHYSICS_STEP",previous=base(step=3,time=0.0))
    run("wrong_physics_time",lambda x:x.update({"physics_step":2,"time_s":0.2}),"WRONG_PHYSICS_TIME",previous=base(step=1,time=0.0))
    run("wrong_robot_identity",lambda x:x.update({"identity":"right"}),"WRONG_ROBOT_IDENTITY")
    run("unexpected_physics_pause",lambda x:x.update({"physics_step":1}),"WRONG_PHYSICS_STEP",previous=base(step=1,time=0.0))
    run("position_sink_active",lambda x:x.update({"position_sink_active":True}),"POSITION_SINK_ACTIVE")
    run("joint_effort_mode_mismatch",lambda x:x.update({"effort_mode":"position"}),"JOINT_EFFORT_MODE_MISMATCH")
    run("invalid_mass_source",lambda x:x.update({"M":[[-1.0 if i==j==0 else float(i==j) for j in range(9)] for i in range(9)]}),"INVALID_MASS_SOURCE")
    run("wrong_tcp_frame",lambda x:x.update({"tcp_frame":"link8"}),"WRONG_TCP_FRAME")
    run("unexpected_collision",lambda x:x.update({"unexpected_collision":True}),"UNEXPECTED_COLLISION")
    gate=LatchedSourceGate()
    try: gate.reject("UNEXPECTED_COLLISION")
    except SourceRejected: pass
    try: torque_attempt(gate);latch={"outcome":"UNEXPECTED_PASS","reason":None,"commands":0}
    except SourceRejected as exc: latch={"outcome":"REJECTED","reason":str(exc),"commands":0}
    cases.append({"name":"torque_after_latched_rejection","expected":"LATCHED_REJECTION:UNEXPECTED_COLLISION",**latch})
    result={"status":"ALL_NEGATIVE_CONTROLS_REJECTED","cases":cases,"robot_command_writes":0,"suction_commands":0}
    if any(x["outcome"]!="REJECTED" or x.get("reason")!=x["expected"] for x in cases): result["status"]="NEGATIVE_CONTROL_FAILURE"
    out=ROOT/"evidence"/"negative_controls.json";out.write_text(json.dumps(result,indent=2,sort_keys=True)+"\n");print(json.dumps(result,indent=2,sort_keys=True));return 0 if result["status"]=="ALL_NEGATIVE_CONTROLS_REJECTED" else 1
if __name__=="__main__": raise SystemExit(main())
