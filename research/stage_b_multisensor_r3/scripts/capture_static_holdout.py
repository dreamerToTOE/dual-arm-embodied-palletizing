#!/usr/bin/env python3
"""One bounded static scene capture; same Task27 geometry and legacy ID23 tag."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO))
R3 = REPO / 'research/stage_b_multisensor_r3'
# Independent of the six diagnostic poses: changes x/y/yaw and support state.
POSES = [
 ('recontact_A', [.753,.035,.260], [0,0,-7]),
 ('recontact_B', [.779,.094,.260], [0,0,9]),
 ('entry_A', [.813,.023,.260], [0,0,-4]),
 ('entry_B', [.831,.089,.260], [0,0,6]),
 ('progress_A', [.887,.042,.260], [0,0,-9]),
 ('progress_B', [.954,.083,.260], [0,0,7]),
 ('wall_approach_A', [1.018,.034,.260], [0,0,-3]),
 ('wall_approach_B', [1.067,.083,.260], [0,0,2]),
 ('wall_arrival_A', [1.095,.025,.260], [0,0,0]),
 ('wall_arrival_B', [1.095,.090,.260], [0,0,0]),
 ('side_prepare_A', [.879,-.090,.260], [0,0,-5]),
 ('side_prepare_B', [.927,.156,.260], [0,0,5]),
 ('lift_negative', [.823,.04,.40], [0,0,6]),
 ('tilt_negative', [.878,.07,.30], [0,20,-5]),
 ('roll_negative', [.907,.02,.30], [-20,0,5]),
 ('small_lift_ambiguous', [.947,.08,.28], [0,0,0]),
]

def sha(path):
 return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
 parser=argparse.ArgumentParser(); parser.add_argument('--run-id',default='r3_static_holdout01'); args=parser.parse_args()
 out=R3/'evidence'/args.run_id; out.mkdir(parents=True,exist_ok=False); (out/'rgb').mkdir()
 from isaacsim import SimulationApp
 app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
 try:
  import omni.timeline,omni.usd,omni.kit.commands
  from pxr import Gf,Usd,UsdGeom
  from isaacsim.core.utils.viewports import set_camera_view
  from isaacsim.sensors.camera import Camera
  from omni.kit.viewport.utility import get_active_viewport,capture_viewport_to_file
  from research.stage_b_vision_contact_r2b.scripts.r2b_camera_observer_smoke import make_marker,MARKER_GRID
  scene=REPO/'isaac/scripts/task26_truck_box_scene.py'
  ns={'__name__':'__r3_task27_static__','_SIDE_SUCTION_SCENARIO':'task27'}
  exec(compile(scene.read_text(),str(scene),'exec'),ns,ns)
  stage=omni.usd.get_context().get_stage(); cube_path='/World/Task27/Supply/Cube_01'; cube=stage.GetPrimAtPath(cube_path)
  xf=UsdGeom.Xformable(cube); tr=next(op for op in xf.GetOrderedXformOps() if op.GetOpName().startswith('xformOp:translate:task26_pose'))
  rot=xf.AddRotateXYZOp(opSuffix='r3_static_yaw_tilt')
  make_marker(stage,cube_path,23,.06,MARKER_GRID)
  cam_path='/World/R2B_RearWallCamera'; cam=UsdGeom.Camera.Define(stage,cam_path)
  cam.GetFocalLengthAttr().Set(20.0); cam.GetHorizontalApertureAttr().Set(36.0); cam.GetVerticalApertureAttr().Set(24.0); cam.GetClippingRangeAttr().Set((.01,100.))
  eye=[1.7,-.35,.8];target=[.82,0,.3]; set_camera_view(eye=eye,target=target,camera_prim_path=cam_path)
  vp=get_active_viewport();vp.set_active_camera(cam_path); vp.render_mode='RaytracedLighting'
  lighting=omni.kit.commands.execute('SetLightingMenuModeCommand',lighting_mode='Default',usd_context_name='')
  assert lighting and lighting[0] is True
  sensor=Camera(prim_path=cam_path,resolution=(1280,720),render_product_path=vp.get_render_product_path());sensor.initialize()
  set_camera_view(eye=eye,target=target,camera_prim_path=cam_path);vp.set_active_camera(cam_path)
  timeline=omni.timeline.get_timeline_interface();timeline.play()
  for _ in range(90):app.update()
  warmup_time=float(timeline.get_current_time());timeline.stop()
  for _ in range(12):app.update()
  M=np.array(UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),float)
  manifest={'schema':'r3.static_scene_holdout.v1','run_id':args.run_id,'type':'static_teleported_scene_holdout_not_manipulation', 'camera':{'world_from_camera':M.tolist(),'K_sdk_readback':np.array(sensor.get_intrinsics_matrix()).tolist(),'usd_focal_length':cam.GetFocalLengthAttr().Get(),'usd_horizontal_aperture':cam.GetHorizontalApertureAttr().Get(),'usd_vertical_aperture':cam.GetVerticalApertureAttr().Get(),'sdk_vertical_aperture':sensor.get_vertical_aperture(),'resolution_px':[1280,720],'render_product':vp.get_render_product_path(),'lighting':repr(lighting)},'sources':{'scene_sha256':sha(scene),'marker_script_sha256':sha(REPO/'research/stage_b_vision_contact_r2b/scripts/r2b_camera_observer_smoke.py'),'frozen_config_sha256':sha(R3/'config/p1_frozen.json')},'physics_warmup_time_s':warmup_time,'motion_commands':0,'collision_geometry_changed':False,'views':[]}
  for i,(name,xyz,rxyz) in enumerate(POSES):
   tr.Set(Gf.Vec3d(*xyz));rot.Set(Gf.Vec3f(*rxyz))
   for _ in range(12):app.update()
   path=out/'rgb'/f'{i:02d}_{name}.png'; t0=time.monotonic();capture_viewport_to_file(vp,file_path=str(path),is_hdr=False)
   for _ in range(20):app.update()
   assert path.exists() and path.stat().st_size>1000
   cm=np.array(xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default()),float)
   corners=[]
   for row,col,local in [(0,0,[-.5,.5,.5]),(0,5,[.5,.5,.5]),(5,5,[.5,-.5,.5]),(5,0,[-.5,-.5,.5])]:
    p=stage.GetPrimAtPath(f'{cube_path}/R2B_ArUco_23/cell_{row}_{col}');tm=np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),float);corners.append((np.array(local)@tm[:3,:3]+tm[3,:3]).tolist())
   manifest['views'].append({'name':name,'image':str(path.relative_to(out)),'image_sha256':sha(path),'cube_world_from_local_oracle':cm.tolist(),'actual_corners_world_oracle':corners,'physics_time_s':float(timeline.get_current_time()),'static':True,'capture_wall_duration_s':time.monotonic()-t0})
   (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
  print('R3_STATIC_CAPTURE_COMPLETE',out,flush=True)
 finally:app.close()

if __name__=='__main__':main()
