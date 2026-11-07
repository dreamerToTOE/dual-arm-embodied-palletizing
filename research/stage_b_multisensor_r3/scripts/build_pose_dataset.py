#!/usr/bin/env python3
"""Extract image-corner observations and separate scoring oracle, without refitting K."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco
from research.stage_b_multisensor_r3.planar_pose_estimator import yaw_from_quaternion_wxyz


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def jsonl(path):return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]
def dump(path,rows):path.write_text(''.join(json.dumps(row,allow_nan=False)+'\n' for row in rows))

def rotation_from_quaternion(q):
 w,x,y,z=np.array(q)/np.linalg.norm(q)
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def label(key,xyz,R):
 return {'key':key,'cube_xyz_m':list(xyz),'yaw_rad':float(np.arctan2(R[1,0],R[0,0])),'tilt_deg':float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))),'on_table':bool(abs(xyz[2]-.26)<=.008 and R[2,2]>=np.cos(np.radians(2))),'source':'evaluation_only_simulator_pose'}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--name',required=True);ap.add_argument('--compact',type=Path);ap.add_argument('--raw',type=Path);ap.add_argument('--static',type=Path);ap.add_argument('--stride',type=int,default=1);ap.add_argument('--out',type=Path,required=True);args=ap.parse_args()
 config=ROOT/'research/stage_b_multisensor_r3/config/p1_frozen.json';K=np.array(json.loads(config.read_text())['K']);obs=[];oracle=[];args.out.mkdir(parents=True,exist_ok=False)
 if args.static:
  m=json.loads((args.static/'manifest.json').read_text());M=m['camera']['world_from_camera']
  for i,v in enumerate(m['views']):
   image=args.static/v['image'];det=detect_aruco(cv2.imread(str(image)),K,np.zeros(5),.06,23);cm=np.array(v['cube_world_from_local_oracle']);R=cm[:3,:3].T;R=R/np.linalg.norm(R,axis=0)
   key=f'{args.name}/{i}';obs.append({'key':key,'source_image':str(image),'source_image_sha256':sha(image),'corners_px':det.corners_px,'status':det.status,'support_context':None,'receipt_time_s':v['physics_time_s'],'static':True});oracle.append(label(key,cm[3,:3],R))
  source={'manifest_sha256':sha(args.static/'manifest.json'),'camera_sdk_K':m['camera']['K_sdk_readback']}
 else:
  M=json.loads((args.compact/'r2b_camera/camera_metadata.json').read_text())['world_from_camera']
  timeline=jsonl(args.compact/'r2b_camera/camera_timeline.jsonl');trace_path=args.raw/'physical_trace/physical_trace.jsonl';trace={r['physics_step']:r for r in jsonl(trace_path)}
  cached_path=args.compact/'visual_detections.jsonl';cached={r['physics_step']:r for r in jsonl(cached_path)} if cached_path.exists() else {}
  receipts_path=args.compact/'execution_phase_receipts.json';receipts=json.loads(receipts_path.read_text())['receipts'] if receipts_path.exists() else []
  for i,t in enumerate(timeline[::args.stride]):
   step=t['physics_step'];gt=trace.get(step);key=f'{args.name}/{step}'
   if gt is None or 'cube_world_m' not in gt:continue
   image=args.raw/'r2b_camera'/t['rgb'] if t.get('rgb') else None
   phase=next((r['skill_id'] for r in receipts if r.get('start_sim_time_s') is not None and r['start_sim_time_s']<=t['time_s']<=r['end_sim_time_s']),None)
   old=cached.get(step,{})
   if old.get('status')=='PRESENT' and old.get('marker_pixels'):
    corners=old['marker_pixels'];status='PRESENT';producer='R2_cached_pixel_corners'
   elif step in cached:
    corners=None;status=old.get('status','UNKNOWN');producer='R2_cached_pixel_detection_failure'
   elif image is not None and image.exists() and t.get('frame_pair_ok',True):
    det=detect_aruco(cv2.imread(str(image)),K,np.zeros(5),.06,23);corners=det.corners_px;status=det.status;producer='R2_detect_aruco_on_raw_RGB'
   else:corners=None;status='UNAVAILABLE';producer='capture_or_writer_unavailable'
   obs.append({'key':key,'source_image':None if image is None else str(image),'source_image_sha256':sha(image) if image is not None and image.exists() else None,'corners_px':corners,'status':status,'producer':producer,'support_context':False if phase in ('coordinated_transport','dual_arm_attachment','placement_release') else None,'executor_skill':phase,'physics_step':step,'receipt_time_s':t['time_s'],'static':False,'receipt_to_render_wait_s':None if t.get('physics_time_after_render_wait_s') is None else t['physics_time_after_render_wait_s']-t['time_s'],'writer_latency_wall_s':None if t.get('writer_complete_wall_s') is None or t.get('physics_state_receipt_wall_s') is None else t['writer_complete_wall_s']-t['physics_state_receipt_wall_s']})
   R=rotation_from_quaternion(gt['cube_quaternion_wxyz']);lab=label(key,gt['cube_world_m'],R)
   end=trace.get(t.get('physics_step_after_render_wait',-1))
   lab['cube_xyz_after_render_wait_m']=None if end is None else end.get('cube_world_m');oracle.append(lab)
  source={'timeline_sha256':sha(args.compact/'r2b_camera/camera_timeline.jsonl'),'trace_sha256':sha(trace_path),'cached_corners_sha256':sha(cached_path) if cached_path.exists() else None,'stride':args.stride,'phase_source':'executor_receipt_intervals_only; legacy GT-derived visual.phase discarded'}
 dump(args.out/'observations.jsonl',obs);dump(args.out/'oracle.jsonl',oracle)
 manifest={'schema':'r3.p1.dataset.v1','name':args.name,'K':K.tolist(),'world_from_camera':M,'config_sha256':sha(config),'source':source,'observations':len(obs),'detections':sum(o['status']=='PRESENT' for o in obs),'observation_sha256':sha(args.out/'observations.jsonl'),'oracle_sha256':sha(args.out/'oracle.jsonl'),'oracle_read_by_estimator':False}
 (args.out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2),flush=True)

if __name__=='__main__':main()
