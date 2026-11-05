#!/usr/bin/env python3
"""Fit effective RGB intrinsics on static rendered board views and score holdouts.

The fitted K is an evaluation artifact only.  No runtime estimator reads it.
"""
from __future__ import annotations
import argparse, json, math
from pathlib import Path
import cv2, numpy as np
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco

def cv_to_world(m, v):
    m=np.asarray(m,float); v=np.asarray(v,float)
    return v[0]*m[0,:3]-v[1]*m[1,:3]-v[2]*m[2,:3]

def pose_world(meta, det):
    m=np.asarray(meta,float); Rcv,_=cv2.Rodrigues(np.asarray(det.rvec,float)); A=np.column_stack((m[0,:3],-m[1,:3],-m[2,:3])); Rw=A@Rcv
    marker=m[3,:3]+cv_to_world(m,det.tvec_m); cube=marker-Rw@np.array([0.,0.,.061]); return cube

def corners(size):
    h=size/2.; return np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],np.float64)

def expected_cv_points(candidate, object_points):
    m=np.asarray(candidate['world_from_camera'],float); center=np.asarray(candidate['marker_gt_world_m'],float)
    world=center+object_points
    rel=world-m[3,:3]
    return np.column_stack((rel@m[0,:3],-rel@m[1,:3],-rel@m[2,:3]))

def fit_linear_k(items, object_points, fx0, fy0):
    uv=[]; xy=[]
    for r in items:
        cv=expected_cv_points(r['candidate'],object_points); uv.append(r['corners']); xy.append(cv)
    uv=np.concatenate(uv); cv=np.concatenate(xy); xn=cv[:,0]/cv[:,2]; yn=cv[:,1]/cv[:,2]
    # A single planar square does not identify focal length and principal point
    # independently.  Keep the authored focal lengths and estimate only the
    # effective viewport principal point from held-out board geometry.
    cx=float(np.mean(uv[:,0]-fx0*xn)); cy=float(np.mean(uv[:,1]-fy0*yn))
    return np.array([[fx0,0,cx],[0,fy0,cy],[0,0,1.]],float)

def quat_angle_from_rvec(rvec, q_gt):
    R,_=cv2.Rodrigues(np.asarray(rvec,float)); tr=np.trace(R)
    if tr>0:
        s=math.sqrt(tr+1)*2.; q=np.array([.25*s,(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s])
    else: q=np.array([1.,0.,0.,0.])
    q/=max(1e-12,np.linalg.norm(q)); q_gt=np.asarray(q_gt,float); q_gt/=max(1e-12,np.linalg.norm(q_gt)); return float(2*np.degrees(np.arccos(np.clip(abs(q@q_gt),-1,1))))

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('sweep',type=Path); ap.add_argument('--physical-run',type=Path); ap.add_argument('--out',type=Path,required=True); args=ap.parse_args()
    meta=json.loads((args.sweep/'candidates.json').read_text()); cam=meta['camera']; w,h=cam['resolution_px']; size=float(cam['marker_size_m']); K0=np.array([[20/36*w,0,w/2],[0,20/24*h,h/2],[0,0,1]],float); dist0=np.zeros(5); obj=corners(size)
    rows=[]
    for c in meta['candidates']:
        image_path=args.sweep/c['rgb']
        if not image_path.exists(): image_path=args.sweep.parent/'calibration_static'/f"{c['id']}.png"
        im=cv2.imread(str(image_path)) if image_path.exists() else None
        if im is None: continue
        det=detect_aruco(im,K0,dist0,size,cam['marker_id'])
        if det.status=='PRESENT': rows.append({'id':c['id'],'image':str(image_path),'corners':np.asarray(det.corners_px,float),'candidate':c})
    hold_ids={'rear_02','rear_08','rear_14','rear_17','rear_03','rear_12'}; train=[r for r in rows if r['id'] not in hold_ids]; hold=[r for r in rows if r['id'] in hold_ids]
    Kfit=fit_linear_k(train,obj,K0[0,0],K0[1,1]); Dfit=dist0.copy(); flags='fixed_authored_focal_lengths_fit_viewport_principal_point'
    def eval_rows(items,label):
        out=[]
        for r in items:
            c=r['candidate']; det=detect_aruco(cv2.imread(r['image']),Kfit,Dfit,size,cam['marker_id']); rec={'id':r['id'],'split':label,'status':det.status}
            if det.status=='PRESENT':
                proj,_=cv2.projectPoints(obj,np.asarray(det.rvec),np.asarray(det.tvec_m),Kfit,Dfit); rec['reprojection_error_px']=float(np.mean(np.linalg.norm(proj.reshape(-1,2)-r['corners'],axis=1))); rec['pose_error_m']=float(np.linalg.norm(pose_world(c['world_from_camera'],det)-np.asarray(c['target_cube_world_m']))); rec['z_m']=float(c['eye'][2]); rec['view_angle_deg']=float(np.degrees(np.arctan2(abs(c['eye'][1]-c['target'][1]),c['eye'][0]-c['target'][0])))
            out.append(rec)
        return out
    static_train=eval_rows(train,'static_train'); static_hold=eval_rows(hold,'static_holdout')
    physical=None
    if args.physical_run:
        root=args.physical_run; pmeta=json.loads((root/'r2b_camera/camera_metadata.json').read_text()); trace={}
        with open(root/'physical_trace/physical_trace.jsonl') as f:
            for line in f:
                x=json.loads(line); trace[int(x['physics_step'])]=x
        timeline=[json.loads(x) for x in (root/'r2b_camera/camera_timeline.jsonl').read_text().splitlines() if x.strip()]; physical=[]
        for item in timeline:
            tr=trace.get(int(item['physics_step'])); det=detect_aruco(cv2.imread(str(root/'r2b_camera'/item['rgb'])),Kfit,Dfit,size,cam['marker_id']); rec={'physics_step':item['physics_step'],'time_s':item['time_s'],'status':det.status}
            if det.status=='PRESENT' and tr:
                rec['reprojection_error_px']=float(det.reprojection_error_px); rec['pose_error_m']=float(np.linalg.norm(pose_world(pmeta['world_from_camera'],det)-np.asarray(tr['cube_world_m']))); rec['depth_m']=float(np.linalg.norm(np.asarray(det.tvec_m)))
            physical.append(rec)
    def avg(xs,k):
        y=[x[k] for x in xs if k in x]; return float(np.mean(y)) if y else None
    result={'schema':'task28.r2b.multiframe_calibration.v1','camera_model_authored':{'K':K0.tolist(),'distortion':dist0.tolist(),'resolution_px':[w,h]},'fit_policy':{'train_ids':[r['id'] for r in train],'heldout_ids':sorted(hold_ids.intersection({x['id'] for x in rows})),'method':flags,'runtime_use':'NEVER; evaluation only'},'K_fit':Kfit.tolist(),'distortion_fit':Dfit.reshape(-1).tolist(),'static_train':{'frames':len(static_train),'mean_reprojection_error_px':avg(static_train,'reprojection_error_px'),'mean_pose_error_m':avg(static_train,'pose_error_m')},'static_holdout':{'frames':len(static_hold),'mean_reprojection_error_px':avg(static_hold,'reprojection_error_px'),'mean_pose_error_m':avg(static_hold,'pose_error_m'),'by_frame':static_hold},'physical05_holdout':({'frames':len(physical),'detections':sum(x['status']=='PRESENT' for x in physical),'detection_rate':sum(x['status']=='PRESENT' for x in physical)/len(physical) if physical else 0.,'mean_reprojection_error_px':avg(physical,'reprojection_error_px'),'mean_pose_error_m':avg(physical,'pose_error_m'),'by_frame':physical} if physical is not None else None),'independence_statement':'K fit uses static sweep train views only; static holdout and physical05 are scored after fitting. T_world_camera and marker-to-Cube remain authored stage geometry and are not promoted without an external multi-view target.'}
    args.out.parent.mkdir(parents=True,exist_ok=True); args.out.write_text(json.dumps(result,indent=2)+'\n'); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
