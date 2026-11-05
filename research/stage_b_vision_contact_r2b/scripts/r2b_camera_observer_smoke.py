#!/usr/bin/env python3
"""Bounded Isaac Sim RGB/ArUco + observer-frame smoke in the Task27 scene.

This is a perception/observation run, not a palletizing success claim.  Cube_01
is placed in its authored feed slot before play so the marker can be qualified
without teleporting during a measured manipulation episode.
"""
from __future__ import annotations
import argparse, cv2, hashlib, json, math, sys, time
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
ISAAC_ROOT = Path('/home/ubuntu2004/isaacsim-4.5.0')
SCENE = REPO/'isaac/scripts/task27_five_cube_center_insert_scene.py'
OUT_ROOT = REPO/'research/stage_b_vision_contact_r2b/evidence'


def sha(path):
 h=hashlib.sha256(); h.update(Path(path).read_bytes()); return h.hexdigest()

def load_scene():
 ns={'__name__':'__r2b_task27_scene__','_SIDE_SUCTION_SCENARIO':'task27'}
 src=SCENE.read_text(encoding='utf-8'); exec(compile(src,str(SCENE),'exec'),ns,ns); return ns

def set_translate(prim, value):
 from pxr import Gf, UsdGeom
 xf=UsdGeom.Xformable(prim)
 for op in xf.GetOrderedXformOps():
  if op.GetOpName().startswith('xformOp:translate:task26_pose'):
   op.Set(Gf.Vec3d(*value)); return
 xf.AddTranslateOp(opSuffix='r2b_pose').Set(Gf.Vec3d(*value))

def make_marker(stage, cube_path, marker_id, marker_size, grid):
 from pxr import Gf, UsdGeom
 print('R2B_MARKER_IMPORT', flush=True)
 root=f'{cube_path}/R2B_ArUco_{marker_id}'
 UsdGeom.Xform.Define(stage,root)
 # Cube_01 is a 0.120 m cube whose parent scale also scales child geometry.
 # Express the board in the parent's local units so it remains 60 mm in world space.
 parent_size=0.120
 cell=marker_size/6.0/parent_size
 for r in range(6):
  for c in range(6):
   p=UsdGeom.Cube.Define(stage,f'{root}/cell_{r}_{c}')
   # OpenCV's marker raster uses 0 for black and 255 for white.
   p.CreateSizeAttr(1.0); p.CreateDisplayColorAttr([Gf.Vec3f(*((0.02,0.02,0.02) if not grid[r,c] else (0.98,0.98,0.98)))])
   xf=UsdGeom.Xformable(p.GetPrim()); xf.AddTranslateOp().Set(Gf.Vec3d((c-2.5)*cell,(2.5-r)*cell,0.5+0.0010/parent_size)); xf.AddScaleOp().Set(Gf.Vec3f(cell,cell,0.0010/parent_size))
 return root

def marker_pose_world(stage, cube_path, marker_size):
 # Marker board's local origin is top-center of the Cube; local +z is world +z.
 from pxr import Gf
 cube=stage.GetPrimAtPath(cube_path); m=np.array(cube.GetAttribute('xformOp:translate:task26_pose').Get(),dtype=float)
 return m + np.array([0.,0.,0.061])

def camera_intrinsics(camera, width, height):
 f=float(camera.GetFocalLengthAttr().Get()); ha=float(camera.GetHorizontalApertureAttr().Get()); va=float(camera.GetVerticalApertureAttr().Get())
 return np.array([[f/ha*width,0,width/2],[0,f/va*height,height/2],[0,0,1]],dtype=np.float64)

MARKER_GRID = np.load(REPO/'research/stage_b_vision_contact_r2b/config/marker23_grid.npy')

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--frames',type=int,default=90); ap.add_argument('--capture-every',type=int,default=15); ap.add_argument('--run-id',default='r2b_camera_observer_smoke01'); args=ap.parse_args()
 out=OUT_ROOT/args.run_id; out.mkdir(parents=True,exist_ok=False); image_dir=out/'rgb'; image_dir.mkdir()
 osenv={'OMNI_KIT_ACCEPT_EULA':'YES'}
 import os; os.environ.update(osenv)
 from isaacsim import SimulationApp
 marker_grid=MARKER_GRID
 app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
 print('R2B_AFTER_APP', flush=True)
 feed=None; subs=[]; rows=[]; contacts=[]; step=0; sim_s=0.0; last_image=None
 try:
  import omni.timeline, omni.usd
  from isaacsim.core.utils.viewports import set_camera_view
  from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
  from pxr import Usd, UsdGeom, Gf, PhysxSchema, PhysicsSchemaTools
  stage=omni.usd.get_context().get_stage(); print('R2B_BEFORE_SCENE', flush=True); load_scene(); print('R2B_AFTER_SCENE', flush=True)
  cube_path='/World/Task27/Supply/Cube_01'; cube=stage.GetPrimAtPath(cube_path); print('R2B_BEFORE_CUBE', flush=True); set_translate(cube,(0.500,0.0,0.260)); print('R2B_AFTER_CUBE', flush=True); marker_root=make_marker(stage,cube_path,23,0.060,marker_grid); print('R2B_AFTER_MARKER', flush=True)
  cam_path='/World/R2B_Camera'; camera=UsdGeom.Camera.Define(stage,cam_path); camera.GetFocalLengthAttr().Set(20.0); camera.GetHorizontalApertureAttr().Set(36.0); camera.GetVerticalApertureAttr().Set(24.0); camera.GetClippingRangeAttr().Set((0.01,100.0))
  print('R2B_BEFORE_CAMERA_VIEW', flush=True); set_camera_view(eye=[0.50,-0.46,0.82],target=[0.50,0.0,0.26],camera_prim_path=cam_path); print('R2B_AFTER_CAMERA_VIEW', flush=True)
  viewport=get_active_viewport(); viewport.set_active_camera(cam_path)
  import omni.kit.commands
  omni.kit.commands.execute('SetLightingMenuModeCommand', lighting_mode='Default', usd_context_name='')
  try: viewport.render_mode='RaytracedLighting'
  except Exception: pass
  camera_world=np.array(UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),dtype=float).tolist()
  depth_camera=None; depth_error=None
  try:
   from isaacsim.sensors.camera import Camera
   depth_camera=Camera(prim_path=cam_path,name='r2b_rgbd_camera',resolution=(1280,720),render_product_path=viewport.get_render_product_path())
   depth_camera.initialize(); depth_camera.add_distance_to_image_plane_to_frame()
  except Exception as exc:
   depth_error=repr(exc)
  timeline=omni.timeline.get_timeline_interface(); print('R2B_BEFORE_PLAY', flush=True); timeline.play(); print('R2B_AFTER_PLAY', flush=True)
  for _ in range(90): app.update()
  print('R2B_AFTER_WARMUP', flush=True)
  # Start the original bridge after scene creation; this supplies q/dq and suction state.
  import importlib.util
  spec=importlib.util.spec_from_file_location('r2b_torque_bridge',REPO/'isaac/scripts/task28_task27_torque_bridge.py'); mod=importlib.util.module_from_spec(spec); sys.modules[spec.name]=mod; spec.loader.exec_module(mod)
  print('R2B_BEFORE_FEED', flush=True); feed=mod.start_original_task27_feed_bridge(); print('R2B_AFTER_FEED', flush=True)
  for prim in stage.Traverse():
   p=str(prim.GetPath())
   if any(x in p for x in ('/World/Task27/Supply/Cube_01','/World/Task27/TruckBox/WallDeep','/World/Task27/TruckBox/WallMinusY','/World/Task27/TruckBox/WallPlusY','/World/left_fr3/fr3_link8/side_suction_tool','/World/right_fr3/fr3_link8/side_suction_tool','/World/Table')):
    try: PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
    except Exception: pass
  phys=omni.physx.get_physx_simulation_interface()
  def contact_cb(headers,data):
   for h in headers:
    paths=[str(PhysicsSchemaTools.intToSdfPath(p)) for p in (h.collider0,h.collider1)]
    if any(cube_path in p for p in paths): contacts.append({'physics_step':step,'time_s':sim_s,'colliders':paths,'points':int(h.num_contact_data)})
  subs.append(phys.subscribe_contact_report_events(contact_cb))
  out_rows=[]; depth_frames=[]
  from research.stage_b_vision_contact_r2b.observer.contract import StageBObserverFrame,SourceStamp,VisualPose,Health
  from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco
  for i in range(int(args.frames)):
   app.update(); step+=1; sim_s += 1/60
   q={};dq={};tcp={}
   for side in ('left','right'):
    art=feed.articulations[side]; q[side]=tuple(float(v) for v in np.asarray(art.get_joint_positions())[:7]); dq[side]=tuple(float(v) for v in np.asarray(art.get_joint_velocities())[:7])
    tprim=stage.GetPrimAtPath(f'/World/{side}_fr3/fr3_link8/side_suction_tool/side_suction_tcp'); mat=UsdGeom.Xformable(tprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()); pt=mat.ExtractTranslation(); tcp[side]=tuple(float(pt[j]) for j in range(3))
   visual=VisualPose(None,None,0.0,None,None,None)
   if i % max(1,int(args.capture_every)) == 0:
    path=image_dir/f'frame_{step:06d}.png'; capture_viewport_to_file(viewport,file_path=str(path),is_hdr=False)
    for _wait in range(30):
     app.update()
     if path.is_file() and path.stat().st_size > 0: break
    last_image=path
    if depth_camera is not None:
     try:
      depth=np.asarray(depth_camera.get_depth())
      if depth.size:
       dpath=image_dir/f'depth_{step:06d}.npy'; np.save(dpath,depth); depth_frames.append(str(dpath.relative_to(out)))
     except Exception as exc:
      depth_error=repr(exc)
    # Kit capture is asynchronous; process the saved RGB pixels after shutdown.
    out_rows.append({'image':str(path.relative_to(out)),'file_present_during_run':path.is_file()})
   suction_cmd={s:(None if getattr(feed,'_last_commanded',{}).get(s) is None else bool(feed._last_commanded[s])) for s in ('left','right')}
   suction_state={s:(bool(feed.grippers[s].is_closed()) if hasattr(feed,'grippers') else None) for s in ('left','right')}
   src={k:SourceStamp(v,float(sim_s),0.0,'isaac_runtime',Health.PRESENT) for k,v in {'physics':'physics_callback','q':'articulation_readback','tcp':'usd_fk','camera':'viewport_rgb'}.items()}
   frame=StageBObserverFrame(step,sim_s,1/60,src,q,dq,tcp,{'owner':'Task26BatchedFeedBridge','mode':'FREE_SPACE_TRAJECTORY_POSITION_DRIVE'},None,None,None,visual,suction_cmd,suction_state,tuple(contacts[-4:]),{'q':Health.PRESENT,'dq':Health.PRESENT,'tcp':Health.PRESENT,'camera':Health.PRESENT},{'camera_reprojection_px':float(visual.reprojection_error_px or 0.)})
   frame.validate(); rows.append(frame.to_dict()); rows[-1]['oracle_contact_pairs']=list(frame.oracle_contact_pairs)
  (out/'observer_frames.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in rows)); (out/'contact_oracle.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in contacts))
  summary={'schema':'task28.r2b.camera_smoke.v1','status':'COMPLETED','frames':len(rows),'captured_images':len(out_rows),'detected_images':None,'depth_images':len(depth_frames),'depth_paths':depth_frames,'depth_error':depth_error,'camera':{'prim':cam_path,'modalities':['rgb','depth'],'prim_type':'Isaac Camera','focal_length_mm':20.,'horizontal_aperture_mm':36.,'vertical_aperture_mm':24.,'marker_id':23,'marker_size_m':.060,'marker_to_cube':'top_center_z_plus_0.061m','world_from_camera':camera_world,'image_width_px':1280,'image_height_px':720},'cube_oracle_eval_pose_world_m':[0.5,0.0,0.26],'scene_sha256':sha(SCENE),'estimator_input_policy':'visual/q/dq/tcp/commands/suction only; oracle contact records separate','fresh_physical_manipulation_episode':False,'note':'perception smoke places Cube in authored slot before play; no measured manipulation'}
  (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n'); print(json.dumps(summary,indent=2))
 except BaseException as exc:
  import traceback
  failure={'schema':'task28.r2b.camera_smoke.v1','status':'FAILED','error':repr(exc),'traceback':traceback.format_exc()}
  (out/'failure.json').write_text(json.dumps(failure,indent=2)+'\n')
  print(json.dumps(failure,indent=2),file=sys.stderr)
 finally:
  for sub in subs:
   try: sub.unsubscribe()
   except Exception: pass
  try:
   if feed is not None and hasattr(feed,'close'): feed.close()
  except Exception: pass
  try: app.close()
  except Exception: pass

if __name__=='__main__': main()
