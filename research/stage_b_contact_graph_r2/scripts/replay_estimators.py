#!/usr/bin/env python3
"""Causal replay of E0/E1 over the R2 timeline."""
from __future__ import annotations
import argparse, gzip, json, sys
from pathlib import Path
from collections import Counter
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from estimator.r2_contact_estimator import (E0GeometricMotionRules, E1TemporalHysteresis,
    FrameObservation, ObservationState, ContactEdge, TaskState)


def rows(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        for line in f:
            if line.strip(): yield json.loads(line)


def make_obs(r):
    ii = r["estimator_input"]
    return FrameObservation(step=int(r["physics_step"]), time_s=float(r["time_s"]),
        cube_position_m=tuple(r["cube_world_m"]), cube_velocity_m_s=tuple(r["cube_velocity_m_s"]),
        left_tcp_m=tuple(r["tcp_world_m"]["left"]), right_tcp_m=tuple(r["tcp_world_m"]["right"]),
        command_advance_m=ii.get("command_advance_m"),
        observation_state=ObservationState(ii["observation_state"]),
        edge_observations={ContactEdge(k): ObservationState(v) for k,v in ii.get("edge_observations",{}).items()},
        features={k:v for k,v in ii.get("features",{}).items() if v is not None})


def event_step(preds, states, wanted, start=0):
    for i, p in enumerate(preds[start:], start):
        if p.state in states: return i
    return None


def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--timeline", type=Path, default=ROOT/"dataset/r2_timeline.jsonl.gz"); ap.add_argument("--out-dir", type=Path, default=ROOT/"derived"); args=ap.parse_args()
    data=list(rows(args.timeline)); e0=E0GeometricMotionRules(); e1=E1TemporalHysteresis(e0)
    p0=[]; p1=[]
    args.out_dir.mkdir(parents=True, exist_ok=True)
    with gzip.open(args.out_dir/"estimator_predictions.jsonl.gz", "wt", encoding="utf-8") as out:
        for r in data:
            a=e0.estimate(make_obs(r)); b=e1.estimate(make_obs(r)); p0.append(a); p1.append(b)
            out.write(json.dumps({"physics_step":r["physics_step"],"time_s":r["time_s"],
                "e0_state":a.state.value,"e0_confidence":a.confidence,"e0_reason":a.reason,
                "e1_state":b.state.value,"e1_confidence":b.confidence,"e1_reason":b.reason},separators=(",",":"))+"\n")
    oracle=[r["oracle_reference"]["edge_states"] for r in data]
    tool_true=[any(x.get(k)=="OBSERVED_ACTIVE" for k in ("LEFT_TOOL_CUBE","RIGHT_TOOL_CUBE")) for x in oracle]
    wall_true=[x.get("CUBE_DEEP_WALL")=="OBSERVED_ACTIVE" for x in oracle]
    tool_states={TaskState.TOOL_CONTACT,TaskState.NORMAL_ADVANCEMENT}
    wall_states={TaskState.DEEP_WALL_ARRIVAL}
    def binary_metrics(truth,pred):
        tp=sum(t and p for t,p in zip(truth,pred)); tn=sum((not t) and (not p) for t,p in zip(truth,pred)); fp=sum((not t) and p for t,p in zip(truth,pred)); fn=sum(t and (not p) for t,p in zip(truth,pred))
        return {"accuracy":(tp+tn)/len(truth),"precision":tp/(tp+fp) if tp+fp else None,"recall":tp/(tp+fn) if tp+fn else None,"tp":tp,"fp":fp,"fn":fn,"tn":tn}
    e0_tool=[p.state in tool_states for p in p0]; e1_tool=[p.state in tool_states for p in p1]
    e0_wall=[p.state in wall_states for p in p0]; e1_wall=[p.state in wall_states for p in p1]
    def first_true(xs):
        return next((i for i,v in enumerate(xs) if v),None)
    def delay(true,pred):
        t=first_true(true); q=first_true(pred)
        return None if t is None or q is None else {"frames":q-t,"seconds":data[q]["time_s"]-data[t]["time_s"],"truth_step":data[t]["physics_step"],"pred_step":data[q]["physics_step"]}
    def counts(ps): return Counter(p.state.value for p in ps)
    edge_truth = {k: [x.get(k)=="OBSERVED_ACTIVE" for x in oracle] for k in
        ("LEFT_TOOL_CUBE", "RIGHT_TOOL_CUBE", "CUBE_TABLE", "CUBE_DEEP_WALL")}
    def edge_metrics(ps):
        result = {}
        for key, truth in edge_truth.items():
            edge = ContactEdge(key)
            pred = [p.edges.get(edge) is ObservationState.OBSERVED_ACTIVE for p in ps]
            result[key] = binary_metrics(truth, pred)
        return result
    result={"schema":"task28.r2.estimator_comparison.v1","episode_count":1,"frame_count":len(data),
      "input_policy":"oracle contact labels excluded from estimator input",
      "labels":"PhysX contact report reference labels; one correlated episode, not independent ground truth",
      "E0":{"state_counts":counts(p0),"edge_reference_agreement":edge_metrics(p0),"tool_contact_event":binary_metrics(tool_true,e0_tool),"deep_wall_event":binary_metrics(wall_true,e0_wall),"tool_delay":delay(tool_true,e0_tool),"deep_wall_delay":delay(wall_true,e0_wall)},
      "E1":{"state_counts":counts(p1),"edge_reference_agreement":edge_metrics(p1),"tool_contact_event":binary_metrics(tool_true,e1_tool),"deep_wall_event":binary_metrics(wall_true,e1_wall),"tool_delay":delay(tool_true,e1_tool),"deep_wall_delay":delay(wall_true,e1_wall)},
      "unknown_policy":{"side_wall":"UNKNOWN","suction":"UNKNOWN","commanded_advancement":"UNKNOWN","tracking_error":"UNKNOWN"},
      "premature_blockage":{"label_available":False,"observed_in_episode":False,"reason":"no synchronized commanded advancement and no independent jam label"},
      "complex_estimator":{"status":"UNPROVEN","reason":"one nominal episode and no independent abnormal-contact labels"}}
    (args.out_dir/"r2_comparison.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    print(json.dumps(result,indent=2))

if __name__ == "__main__": main()
