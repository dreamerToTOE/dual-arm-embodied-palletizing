#!/usr/bin/env python3
"""Replay E0/E1/E2 on the prior R2 geometry trace (oracle pose only for audit)."""
from __future__ import annotations
import argparse, gzip, json
from collections import Counter
from pathlib import Path
import numpy as np
from research.stage_b_vision_contact_r2b.estimator.contact_estimators import E0MultiLabel, E1Hysteresis, E2PhysicsConstrained, Edge

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('dataset',type=Path); ap.add_argument('--out',type=Path,required=True); args=ap.parse_args()
    e0,e1,e2=E0MultiLabel(),E1Hysteresis(dwell=3),E2PhysicsConstrained()
    modes={name:Counter() for name in ('E0','E1','E2')}; max_edges={name:{e.value:0.0 for e in Edge} for name in modes}; prediction_only=0; rows=0
    with gzip.open(args.dataset,'rt',encoding='utf-8') as stream:
      for line in stream:
        x=json.loads(line); p=np.asarray(x['cube_world_m']); vx=float(x['cube_velocity_m_s'][0]); tcp=x['tcp_world_m']
        f={'physics_step':x['physics_step'],'time_s':x['time_s'],'cube_position_m':tuple(p),'visual_health':'ORACLE','visual_confidence':1.0,
           'left_tool_distance_m':float(np.linalg.norm(np.asarray(tcp['left'])-p)),'right_tool_distance_m':float(np.linalg.norm(np.asarray(tcp['right'])-p)),
           'table_gap_m':float(p[2]-.26),'deep_wall_gap_m':float(1.16-.06-p[0]),'cube_velocity_x_m_s':vx,
           'suction_closed_count':0,'commanded_advancement_m':None}
        a,b,c=e0.estimate(f),e1.estimate(f),e2.estimate(f); rows+=1
        for name,r in (('E0',a),('E1',b),('E2',c)):
          modes[name][r.mode.value]+=1
          for e,v in r.edges.items(): max_edges[name][e.value]=max(max_edges[name][e.value],v)
        f['visual_health']='UNKNOWN'; f['visual_confidence']=0.0
        if e2.estimate(f).causes==('prediction_only',): prediction_only+=1
    result={'schema':'task28.r2b.estimator_replay.v1','frames':rows,'input_policy':'oracle pose replay for audit; no online claim','mode_counts':{k:dict(v) for k,v in modes.items()},'max_edge_probability':max_edges,'e2_prediction_only_occlusion_probe_frames':prediction_only}
    args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
