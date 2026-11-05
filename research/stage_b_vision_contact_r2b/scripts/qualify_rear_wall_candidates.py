#!/usr/bin/env python3
"""Pixel/depth qualification and conservative clearance audit for rear-wall sites."""
from __future__ import annotations
import argparse, gzip, json, math
from pathlib import Path
import cv2, numpy as np
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco

def cv_to_world(m, v):
    m=np.asarray(m,dtype=float); v=np.asarray(v,dtype=float)
    return v[0]*m[0,:3]-v[1]*m[1,:3]-v[2]*m[2,:3]

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('sweep',type=Path); ap.add_argument('--out',type=Path); ap.add_argument('--trace',type=Path,default=Path('research/stage_b_contact_graph_r2/evidence/r4_physical_push02/physical_trace.jsonl.gz')); args=ap.parse_args()
    meta=json.loads((args.sweep/'candidates.json').read_text()); K=np.array([[20/36*1280,0,640],[0,20/24*720,360],[0,0,1]],float); K_cal=K.copy(); K_cal[1,2]=341.0; dist=np.zeros(5)
    (args.sweep/'annotated').mkdir(exist_ok=True)
    tcp=[]
    if args.trace.exists():
        with gzip.open(args.trace,'rt') as f:
            for line in f:
                x=json.loads(line); tcp += [x['tcp_world_m']['left'],x['tcp_world_m']['right']]
    reports=[]
    for c in meta['candidates']:
        im=cv2.imread(str(args.sweep/c['rgb'])); det=detect_aruco(im,K,dist,meta['camera']['marker_size_m'],meta['camera']['marker_id']); det_cal=detect_aruco(im,K_cal,dist,meta['camera']['marker_size_m'],meta['camera']['marker_id']); d=np.load(args.sweep/c['depth']); finite=np.isfinite(d)
        eye=np.asarray(c['eye'],float); target=np.asarray(c['target'],float); wall_x=1.17; wall_z=.35
        z_wall=eye[2]+(target[2]-eye[2])*(wall_x-eye[0])/(target[0]-eye[0]); body_clear=float(eye[0]-.15-wall_x)
        path_clear=float(min((np.linalg.norm(eye-np.asarray(p))-.15) for p in tcp)) if tcp else None
        row={"id":c['id'],"eye_world_m":c['eye'],"target_world_m":c['target'],"rgb":c['rgb'],"depth":c['depth'],"detection_status":det.status,"reprojection_error_px":det.reprojection_error_px,"confidence":det.confidence,"depth_finite_fraction":float(finite.mean()),"camera_to_cube_m":float(np.linalg.norm(eye-np.asarray(c['target_cube_world_m']))),"deep_wall_ray_z_m":float(z_wall),"deep_wall_occluded_by_ray":bool(z_wall<wall_z),"body_clearance_to_box_m":body_clear,"tool_trajectory_clearance_m":path_clear,"safe_external_envelope":bool(body_clear>0 and (path_clear is None or path_clear>0))}
        if det.status=='PRESENT':
            pts=np.asarray(det.corners_px); row['marker_pixel_coverage']=float(max(pts[:,0].ptp(),pts[:,1].ptp())/1280); row['marker_area_px2']=float(cv2.contourArea(pts.astype(np.float32)))
            m=np.asarray(c['world_from_camera']); p_marker=m[3,:3]+cv_to_world(m,det.tvec_m); p_cube=p_marker-np.array([0,0,.061]); row['cube_pose_eval_error_m']=float(np.linalg.norm(p_cube-np.asarray(c['target_cube_world_m'])))
            if det_cal.status=='PRESENT':
                p_marker_cal=m[3,:3]+cv_to_world(m,det_cal.tvec_m); p_cube_cal=p_marker_cal-np.array([0,0,.061]); row['cube_pose_eval_error_render_calibrated_cy341_m']=float(np.linalg.norm(p_cube_cal-np.asarray(c['target_cube_world_m'])))
            R,_=cv2.Rodrigues(np.asarray(det.rvec)); normal=cv_to_world(m,R[:,2]); normal/=max(1e-9,np.linalg.norm(normal)); row['marker_normal_error_deg']=float(np.degrees(np.arccos(np.clip(np.dot(normal,[0,0,1]),-1,1))))
            cv2.polylines(im,[pts.astype(np.int32).reshape(-1,1,2)],True,(0,255,0),2); cv2.imwrite(str(args.sweep/'annotated'/f"{c['id']}.png"),im)
        reports.append(row)
    reports.sort(key=lambda r:(r['detection_status']!='PRESENT',not r['safe_external_envelope'],r.get('cube_pose_eval_error_m',99),-r.get('confidence',0)))
    result={'schema':'task28.r2b.rear_wall_qualification.v2','candidates_tested':len(reports),'trace_used_for_clearance':str(args.trace),'camera_body_envelope_half_extents_m':[.15,.15,.15],'reports':reports,'best_candidates':[r['id'] for r in reports[:3]],'calibration_comparison':{'authored_intrinsics_cy360':'cube_pose_eval_error_m','render_calibrated_cy341':'cube_pose_eval_error_render_calibrated_cy341_m','note':'cy=341 is evaluation-only; no online estimator or safety threshold was changed'},'policy':'rendered pixels/depth for visibility; historical TCP only for conservative clearance, no Cube oracle in detector'}
    out=args.out or (args.sweep/'qualification.json'); out.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
