#!/usr/bin/env python3
"""Evaluate pixel observations and causal dropout predictors on a new run."""
from __future__ import annotations
import argparse, gzip, json, math, re
from pathlib import Path
import cv2, numpy as np
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco

def cv_to_world(m, v):
    m=np.asarray(m,float); v=np.asarray(v,float); return v[0]*m[0,:3]-v[1]*m[1,:3]-v[2]*m[2,:3]

def quat_from_rot(R):
    tr=float(np.trace(R))
    if tr>0:
        s=math.sqrt(tr+1)*2; return np.array([.25*s,(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s])
    i=int(np.argmax(np.diag(R))); j=(i+1)%3; k=(i+2)%3; q=np.zeros(4); q[i+1]=math.sqrt(max(0,1+R[i,i]-R[j,j]-R[k,k]))/2; q[0]=(R[k,j]-R[j,k])/(4*q[i+1]); q[j+1]=(R[j,i]+R[i,j])/(4*q[i+1]); q[k+1]=(R[k,i]+R[i,k])/(4*q[i+1]); return q

def quat_angle(a,b):
    a=np.asarray(a,float); b=np.asarray(b,float); a/=max(1e-12,np.linalg.norm(a)); b/=max(1e-12,np.linalg.norm(b)); return float(2*np.degrees(np.arccos(np.clip(abs(np.dot(a,b)),-1,1))))

def phase(row, log_text):
    p=np.asarray(row['cube_world_m']); s=row.get('suction_state') or {}; both=bool(s.get('left')) and bool(s.get('right')); one=bool(s.get('left')) ^ bool(s.get('right')); none=not bool(s.get('left')) and not bool(s.get('right')); x,y,z=p
    labels=[]
    if x<.60: labels += ['cube_supply']
    if x<.60 and none: labels += ['dual_arm_approach']
    if x<.65 and both: labels += ['dual_arm_attachment']
    if both and .60<=x<1.05: labels += ['coordinated_transport']
    if both and x>=1.05: labels += ['placement_release']
    if one and x>=.80: labels += ['single_arm_recontact','single_arm_insertion']
    if x>=1.08: labels += ['deep_wall_arrival']
    if one and x>=.95 and ('SIDE_COMPACTION' in log_text or abs(float(y))>.12): labels += ['auxiliary_side_compaction']
    if none and x>=.80: labels += ['release_retreat']
    return labels

def world_pose(meta, det):
    m=np.asarray(meta['world_from_camera'],float); Rcv,_=cv2.Rodrigues(np.asarray(det.rvec,float)); A=np.column_stack((m[0,:3],-m[1,:3],-m[2,:3]))
    Rw=A@Rcv; pm=m[3,:3]+cv_to_world(m,det.tvec_m); pc=pm-Rw@np.array([0,0,.061]); return pc,quat_from_rot(Rw)

def predictor_stats(detections, trace, variant):
    """Score causal gap predictors using one visual calibration variant."""
    status_key=f'{variant}_status'; pose_key=f'{variant}_estimated_cube_world_m'
    p0=[]; p1=[]; p2=[]; prev_valid=[]; current_anchor=None
    for r in detections:
        tr=trace.get(int(r['physics_step']))
        if r.get(status_key)=='PRESENT' and pose_key in r:
            prev_valid.append(r); current_anchor=r; continue
        if tr is None or current_anchor is None: continue
        gt=np.asarray(tr['cube_world_m']); age=float(r['time_s']-current_anchor['time_s']); base=np.asarray(current_anchor[pose_key]);
        p0.append({'duration_s':age,'error_m':float(np.linalg.norm(base-gt))})
        if len(prev_valid)>=2:
            a,b=prev_valid[-2],prev_valid[-1]; vel=(np.asarray(b[pose_key])-np.asarray(a[pose_key]))/max(1e-6,b['time_s']-a['time_s']); pred=base+vel*age; p1.append({'duration_s':age,'error_m':float(np.linalg.norm(pred-gt))})
        contact_supported=tr.get('attachment_confidence')=='SUCTION_CLOSED_AND_TOOL_CUBE_CONTACT'
        if contact_supported:
            anchor_tr=trace.get(int(current_anchor['physics_step']))
            if anchor_tr:
                tcp0=np.mean([anchor_tr['tcp_world_m']['left'],anchor_tr['tcp_world_m']['right']],axis=0); tcp=np.mean([tr['tcp_world_m']['left'],tr['tcp_world_m']['right']],axis=0); pred=base+(np.asarray(tcp)-np.asarray(tcp0)); p2.append({'duration_s':age,'error_m':float(np.linalg.norm(pred-gt)),'confidence':'ATTACHMENT_SUPPORTED'})
        else: p2.append({'duration_s':age,'error_m':None,'confidence':'UNKNOWN_ATTACHMENT'})
    def stats(xs):
        ys=[x['error_m'] for x in xs if x['error_m'] is not None]; return {'samples':len(xs),'mean_error_m':(float(np.mean(ys)) if ys else None),'p95_error_m':(float(np.percentile(ys,95)) if ys else None),'max_duration_s':(float(max(x['duration_s'] for x in xs)) if xs else None)}
    return {'P0_last_valid_visual':stats(p0),'P1_visual_velocity':stats(p1),'P2_tcp_propagation_attachment_gated':stats(p2)}

def blind_gap_stats(detections, trace, variant):
    """Score long intervals with no captured image separately from dropouts."""
    pose_key=f'{variant}_estimated_cube_world_m'; p0=[]; p1=[]; p2=[]
    valid=[r for r in detections if r.get(f'{variant}_status')=='PRESENT' and pose_key in r]
    def stats(xs):
        ys=[x['error_m'] for x in xs if x['error_m'] is not None]; return {'samples':len(xs),'mean_error_m':(float(np.mean(ys)) if ys else None),'p95_error_m':(float(np.percentile(ys,95)) if ys else None),'max_duration_s':(float(max(x['duration_s'] for x in xs)) if xs else None)}
    for i,(a,b) in enumerate(zip(valid,valid[1:])):
        age=float(b['time_s']-a['time_s'])
        if age<=.25: continue
        tr=trace.get(int(b['physics_step'])); anchor=trace.get(int(a['physics_step']))
        if not tr or not anchor: continue
        gt=np.asarray(tr['cube_world_m']); base=np.asarray(a[pose_key]); p0.append({'duration_s':age,'error_m':float(np.linalg.norm(base-gt))})
        if i>0:
            prev=valid[i-1]; vel=(np.asarray(a[pose_key])-np.asarray(prev[pose_key]))/max(1e-6,a['time_s']-prev['time_s']); p1.append({'duration_s':age,'error_m':float(np.linalg.norm(base+vel*age-gt))})
        if tr.get('attachment_confidence')=='SUCTION_CLOSED_AND_TOOL_CUBE_CONTACT':
            tcp0=np.mean([anchor['tcp_world_m']['left'],anchor['tcp_world_m']['right']],axis=0); tcp=np.mean([tr['tcp_world_m']['left'],tr['tcp_world_m']['right']],axis=0); p2.append({'duration_s':age,'error_m':float(np.linalg.norm(base+(np.asarray(tcp)-np.asarray(tcp0))-gt)),'confidence':'SUCTION_CLOSED_AND_TOOL_CUBE_CONTACT'})
        else: p2.append({'duration_s':age,'error_m':None,'confidence':'UNKNOWN'})
    return {'P0_last_valid_visual':stats(p0),'P1_visual_velocity':stats(p1),'P2_tcp_propagation_attachment_gated':stats(p2)}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('run',type=Path); ap.add_argument('--calibrated-cy',type=float,default=341.0,help='evaluation-only viewport principal point; estimator remains authored K'); args=ap.parse_args(); root=args.run; meta=json.loads((root/'r2b_camera/camera_metadata.json').read_text()); K=np.array([[20/36*1280,0,640],[0,20/24*720,360],[0,0,1]],float); K_cal=K.copy(); K_cal[1,2]=args.calibrated_cy; dist=np.zeros(5)
    trace={}
    with open(root/'physical_trace/physical_trace.jsonl') as f:
        for line in f:
            x=json.loads(line); trace[int(x['physics_step'])]=x
    contact_steps=set(); contact_path=root/'physical_trace/contact_events.jsonl'
    if contact_path.exists():
        with open(contact_path) as f:
            for line in f:
                ev=json.loads(line); colliders=' '.join(ev.get('colliders',[]))
                if 'task26_cube_1' in colliders and 'side_suction' in colliders: contact_steps.add(int(ev['physics_step']))
    for step,tr in trace.items():
        s=tr.get('suction_state') or {}; both=bool(s.get('left')) and bool(s.get('right'))
        tr['attachment_confidence']='SUCTION_CLOSED_AND_TOOL_CUBE_CONTACT' if both and step in contact_steps else 'UNKNOWN'
    timeline=[json.loads(x) for x in (root/'r2b_camera/camera_timeline.jsonl').read_text().splitlines() if x.strip()]
    detections=[]
    for item in timeline:
        tr=trace.get(int(item['physics_step']))
        rgb_path=root/'r2b_camera'/item['rgb'] if item.get('rgb') else None
        if not item.get('frame_pair_ok', True) or rgb_path is None or not rgb_path.is_file():
            rec={'physics_step':item['physics_step'],'time_s':item['time_s'],'rgb':item.get('rgb'),'depth':item.get('depth'),'depth_ok':bool(item.get('depth_ok')),'status':'UNAVAILABLE','confidence':'NONE','reprojection_error_px':None,'phase':phase(tr,'') if tr else []}
            rec.update({'authored_intrinsics_status':'UNAVAILABLE','render_calibrated_cy341_status':'UNAVAILABLE'})
            detections.append(rec)
            continue
        im=cv2.imread(str(rgb_path)); dets={'authored_intrinsics':detect_aruco(im,K,dist,.06,23),'render_calibrated_cy341':detect_aruco(im,K_cal,dist,.06,23)}; det=dets['authored_intrinsics']; rec={'physics_step':item['physics_step'],'time_s':item['time_s'],'rgb':item['rgb'],'depth':item['depth'],'depth_ok':item['depth_ok'],'status':det.status,'confidence':det.confidence,'reprojection_error_px':det.reprojection_error_px}
        if item['depth']:
            d=np.load(root/'r2b_camera'/item['depth']); rec['depth_finite_fraction']=float(np.isfinite(d).mean())
        for variant,vdet in dets.items():
            rec[f'{variant}_status']=vdet.status
            rec[f'{variant}_reprojection_error_px']=vdet.reprojection_error_px
            if vdet.status=='PRESENT' and tr:
                p,q=world_pose(meta,vdet); gt=np.asarray(tr['cube_world_m']); rec.update({f'{variant}_estimated_cube_world_m':p.tolist(),f'{variant}_estimated_quaternion_wxyz':q.tolist(),f'{variant}_position_error_m':float(np.linalg.norm(p-gt)),f'{variant}_orientation_error_deg':quat_angle(q,tr['cube_quaternion_wxyz']),f'{variant}_marker_pixels':vdet.corners_px})
        if det.status=='PRESENT' and tr:
            rec.update({'estimated_cube_world_m':rec['authored_intrinsics_estimated_cube_world_m'],'estimated_quaternion_wxyz':rec['authored_intrinsics_estimated_quaternion_wxyz'],'position_error_m':rec['authored_intrinsics_position_error_m'],'orientation_error_deg':rec['authored_intrinsics_orientation_error_deg'],'marker_pixels':det.corners_px})
        rec['phase']=phase(tr,'') if tr else []
        detections.append(rec)
    # Use the executor log only as a phase-execution receipt, never as visual data.
    log_text=(root/'task27_executor.log').read_text(errors='replace')
    for rec in detections:
        if 'auxiliary_side_compaction' in rec['phase'] and 'SIDE_COMPACTION' not in log_text: rec['phase'].remove('auxiliary_side_compaction')
    present=[r for r in detections if r['status']=='PRESENT']; times=[float(r['time_s']) for r in detections]
    dropouts=[]; start=None; last_valid=None
    for r in detections:
        if r['status']=='PRESENT':
            if start is not None: dropouts.append({'start_s':start,'end_s':r['time_s'],'duration_s':float(r['time_s']-start)})
            start=None; last_valid=r
        elif last_valid is not None and start is None: start=float(r['time_s'])
    if start is not None and times: dropouts.append({'start_s':start,'end_s':times[-1],'duration_s':float(times[-1]-start)})
    phase_rows={}
    for name in ('cube_supply','dual_arm_approach','dual_arm_attachment','coordinated_transport','placement_release','single_arm_recontact','single_arm_insertion','deep_wall_arrival','auxiliary_side_compaction','release_retreat'):
        subset=[r for r in detections if name in r.get('phase',[])]
        valid=[r for r in subset if r['status']=='PRESENT']; phase_rows[name]={'frames':len(subset),'detections':len(valid),'detection_rate':(len(valid)/len(subset) if subset else None),'mean_position_error_m':(float(np.mean([r['position_error_m'] for r in valid])) if valid else None),'mean_orientation_error_deg':(float(np.mean([r['orientation_error_deg'] for r in valid])) if valid else None),'depth_available_rate':(float(np.mean([r.get('depth_ok',False) for r in subset])) if subset else None),'status':('EXECUTED_OBSERVED' if subset else 'NOT_EXECUTED')}
    def variant_summary(variant):
        valid=[r for r in detections if r.get(f'{variant}_status')=='PRESENT' and f'{variant}_position_error_m' in r]
        return {'detections':len(valid),'detection_rate':len(valid)/len(detections) if detections else 0.0,'mean_position_error_m':float(np.mean([r[f'{variant}_position_error_m'] for r in valid])) if valid else None,'mean_orientation_error_deg':float(np.mean([r[f'{variant}_orientation_error_deg'] for r in valid])) if valid else None}
    cal_phase_rows={}
    for name in ('cube_supply','dual_arm_approach','dual_arm_attachment','coordinated_transport','placement_release','single_arm_recontact','single_arm_insertion','deep_wall_arrival','auxiliary_side_compaction','release_retreat'):
        subset=[r for r in detections if name in r.get('phase',[])]; valid=[r for r in subset if r.get('render_calibrated_cy341_status')=='PRESENT' and 'render_calibrated_cy341_position_error_m' in r]; cal_phase_rows[name]={'frames':len(subset),'detections':len(valid),'detection_rate':(len(valid)/len(subset) if subset else None),'mean_position_error_m':(float(np.mean([r['render_calibrated_cy341_position_error_m'] for r in valid])) if valid else None),'mean_orientation_error_deg':(float(np.mean([r['render_calibrated_cy341_orientation_error_deg'] for r in valid])) if valid else None),'depth_available_rate':(float(np.mean([r.get('depth_ok',False) for r in subset])) if subset else None),'status':('EXECUTED_OBSERVED' if subset else 'NOT_EXECUTED')}
    times=np.asarray([float(r['time_s']) for r in detections]); gaps=np.diff(times) if len(times)>1 else np.asarray([]); blind=[{'start_s':float(detections[i]['time_s']),'end_s':float(detections[i+1]['time_s']),'duration_s':float(gaps[i])} for i in range(len(gaps)) if gaps[i]>.25]
    result={'schema':'task28.r2b.physical_camera_qualification.v3','run_id':root.name,'frames':len(detections),'detections':len(present),'detection_rate':len(present)/len(detections) if detections else 0.0,'mean_position_error_m':float(np.mean([r['position_error_m'] for r in present])) if present else None,'mean_orientation_error_deg':float(np.mean([r['orientation_error_deg'] for r in present])) if present else None,'longest_visual_dropout_s':max([d['duration_s'] for d in dropouts],default=0.0),'dropout_intervals':dropouts,'observation_timing':{'capture_span_s':(float(times[-1]-times[0]) if len(times)>1 else 0.0),'median_intercapture_s':(float(np.median(gaps)) if len(gaps) else None),'achieved_rate_hz_excluding_gaps':(float(1/np.median(gaps)) if len(gaps) else None),'max_intercapture_gap_s':(float(max(gaps)) if len(gaps) else None),'unsampled_intervals':blind,'note':'unsampled intervals are not optical no-detection intervals'},'phase_visibility':phase_rows,'phase_visibility_source':'legacy geometry/Cube-GT labels; corrected executor receipts are in execution_phase_receipts.json','calibration_comparison':{'authored_intrinsics_cy360':variant_summary('authored_intrinsics'),'render_calibrated_cy341':variant_summary('render_calibrated_cy341'),'render_calibrated_cy341_phase_visibility':cal_phase_rows,'note':'cy=341 is an evaluation-only fitted viewport principal point; online estimator remains authored cy=360 until multi-view calibration is accepted'},'predictors':predictor_stats(detections,trace,'authored_intrinsics'),'predictors_render_calibrated_cy341':predictor_stats(detections,trace,'render_calibrated_cy341'),'blind_gap_predictors':{'authored_intrinsics':blind_gap_stats(detections,trace,'authored_intrinsics'),'render_calibrated_cy341':blind_gap_stats(detections,trace,'render_calibrated_cy341')},'prediction_policy':'uncertainty grows with gap age; P2 UNKNOWN unless both suction state and tool-Cube contact support the interval; TCP predictor never reads Cube GT','physical_trace_summary':json.loads((root/'physical_trace/physical_trace_summary.json').read_text()),'executor_phase_receipt':{'attachment':('ATTACHMENT' in log_text),'single_arm_push':('PUSH' in log_text),'side_compaction':('SIDE_COMPACTION' in log_text),'batch_pass':('batch 1 PASS' in log_text)}}
    (root/'visual_qualification.json').write_text(json.dumps(result,indent=2)+'\n'); (root/'visual_detections.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in detections)); print(json.dumps(result,indent=2))
if __name__=='__main__': main()
