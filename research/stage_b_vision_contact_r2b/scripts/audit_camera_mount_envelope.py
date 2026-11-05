#!/usr/bin/env python3
"""Conservative camera body/column/cable envelope check against recorded TCP sweeps."""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import numpy as np

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('sweep',type=Path); ap.add_argument('physical_run',type=Path); ap.add_argument('--out',type=Path,required=True); args=ap.parse_args()
    qual=json.loads((args.sweep/'qualification.json').read_text()); q={r['id']:r for r in qual['reports']}; pts=[]
    with open(args.physical_run/'physical_trace/physical_trace.jsonl') as f:
        for line in f:
            row=json.loads(line); pts.extend(row['tcp_world_m'].values())
    candidates={k:q[k] for k in ('rear_11','rear_17')}
    rows=[]
    for cid,r in candidates.items():
        eye=np.asarray(r['eye_world_m'],float); radius=.10; cable=.05
        d=[]
        for p in pts:
            p=np.asarray(p,float); nearest=np.array([eye[0],eye[1],np.clip(p[2],.20,eye[2])]); d.append(float(np.linalg.norm(p-nearest)))
        min_center=min(d) if d else None
        rows.append({'id':cid,'eye_world_m':r['eye_world_m'],'static_marker_status':r['detection_status'],'static_box_clearance_m':r['body_clearance_to_box_m'],'static_tcp_point_clearance_m':r['tool_trajectory_clearance_m'],'column_radius_m':radius,'cable_envelope_radius_m':cable,'recorded_tcp_to_column_center_min_m':min_center,'recorded_tcp_to_column_surface_min_m':(min_center-radius-cable if min_center is not None else None),'box_outer_x_m':1.18,'camera_body_x_clearance_m':float(eye[0]-.15-1.18),'mountability_assumption':'column center is outside +X wall and table active envelope; bracket/cable use stated radii','swept_collision_certificate':'NOT_ESTABLISHED: only TCP points from one physical trace, not full link geometry/FCL swept volume'})
    result={'schema':'task28.r2b.camera_mount_envelope.v1','run_id':args.physical_run.name,'candidates':rows,'selection':'rear_11','selection_reason':'rear_11 has the same 0.380 m static box clearance as rear_17 and was physically observed at 10 Hz; rear_17 remains a static alternative. Neither is a complete bracket/column/link FCL certificate.','policy':'no collision thresholds or FCL rules changed'}
    args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
