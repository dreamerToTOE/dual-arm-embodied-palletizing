#!/usr/bin/env python3
"""Offline projection proof and cached-corner checks, never estimator calibration."""
import json
import hashlib
from pathlib import Path
import sys
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_multisensor_r3.planar_pose_estimator import project,cube_marker_world_corners
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco


def main():
 r3=ROOT/'research/stage_b_multisensor_r3';manifest=json.loads((r3/'evidence/r3_static_holdout01/manifest.json').read_text());config=json.loads((r3/'config/p1_frozen.json').read_text());K=np.array(config['K']);M=np.array(manifest['camera']['world_from_camera']);obs={r['key']:r for r in map(json.loads,(r3/'evidence/static_pose_dataset/observations.jsonl').read_text().splitlines())};rows=[]
 oldK=K.copy();oldK[1,1]=600.
 for i,v in enumerate(manifest['views'][:12]):
  corners=obs[f'r3_static_holdout01/{i}']['corners_px'];actual=np.array(v['actual_corners_world_oracle']);cm=np.array(v['cube_world_from_local_oracle']);R=cm[:3,:3].T;R=R/np.linalg.norm(R,axis=0);yaw=float(np.arctan2(R[1,0],R[0,0]));model=cube_marker_world_corners(*cm[3,:2],yaw)
  row={'name':v['name'],'geometry_max_error_m':float(np.linalg.norm(actual-model,axis=1).max())}
  if corners:
   for name,kk in [('legacy_aperture_K',oldK),('frozen_sdk_K',K)]:
    residual=np.array(corners)-project(actual,kk,M);row[name]={'corner_residual_px':residual.tolist(),'mean_px':float(np.linalg.norm(residual,axis=1).mean()),'max_px':float(np.linalg.norm(residual,axis=1).max())}
  rows.append(row)
 checks=[]
 for name in ('physical05_development','physical07_development'):
  ds=r3/'evidence'/name;observations=[json.loads(l) for l in (ds/'observations.jsonl').read_text().splitlines()];observations=[r for r in observations if r.get('corners_px')]
  for index in sorted(set([0,len(observations)//4,len(observations)//2,3*len(observations)//4,len(observations)-1])):
   row=observations[index];im=cv2.imread(row['source_image']);det=detect_aruco(im,K,np.zeros(5),.06,23);checks.append({'key':row['key'],'redetection_status':det.status,'cached_match':det.corners_px is not None and np.allclose(det.corners_px,row['corners_px']),'raw_image_sha256':row['source_image_sha256']})
 result={'schema':'r3.projection_and_pixel_proof.v1','offline_scoring_only':True,'static_views':rows,'cached_corner_redetection':checks,'readback_K_matches_frozen':bool(np.allclose(manifest['camera']['K_sdk_readback'],K,atol=1e-4))}
 (r3/'evidence/projection_audit.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps({'cached_corner_matches':sum(r['cached_match'] for r in checks),'checked':len(checks),'sdk_readback_match':result['readback_K_matches_frozen'],'legacy_projection_mean_px':float(np.mean([r['legacy_aperture_K']['mean_px'] for r in rows])),'sdk_projection_mean_px':float(np.mean([r['frozen_sdk_K']['mean_px'] for r in rows])),'geometry_max_error_m':max(r['geometry_max_error_m'] for r in rows)},indent=2))

if __name__=='__main__':main()
