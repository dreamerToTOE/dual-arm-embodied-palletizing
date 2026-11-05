#!/usr/bin/env python3
"""Render a bounded rear-wall camera-site sweep in the unchanged Task27 scene."""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import numpy as np

REPO = Path(__file__).resolve().parents[3]
OUT_ROOT = REPO / "research/stage_b_vision_contact_r2b/evidence"
SCENE = REPO / "isaac/scripts/task27_five_cube_center_insert_scene.py"
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

def candidates():
    out=[]
    idx=0
    for x in (1.45, 1.70):
        for y in (-0.35, 0.0, 0.35):
            for z in (0.40, 0.80, 1.20):
                idx += 1
                out.append({"id": f"rear_{idx:02d}", "eye": [x,y,z], "target": [0.82,0.0,0.30]})
    return out

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--run-id',default='r2b_rear_wall_sweep01'); args=ap.parse_args()
    out=OUT_ROOT/args.run_id; out.mkdir(parents=True,exist_ok=False); (out/'rgb').mkdir()
    import os; os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
    try:
        import omni.timeline, omni.usd, omni.kit.commands
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file
        from pxr import Usd, UsdGeom
        from isaacsim.sensors.camera import Camera
        from research.stage_b_vision_contact_r2b.scripts.r2b_camera_observer_smoke import load_scene, set_translate, make_marker, MARKER_GRID
        stage=omni.usd.get_context().get_stage(); load_scene()
        cube_path='/World/Task27/Supply/Cube_01'; cube=stage.GetPrimAtPath(cube_path); set_translate(cube,(0.500,0.0,0.260)); make_marker(stage,cube_path,23,0.060,MARKER_GRID)
        cam_path='/World/R2B_RearWallCamera'; camera=UsdGeom.Camera.Define(stage,cam_path); camera.GetFocalLengthAttr().Set(20.0); camera.GetHorizontalApertureAttr().Set(36.0); camera.GetVerticalApertureAttr().Set(24.0); camera.GetClippingRangeAttr().Set((0.01,100.0))
        viewport=get_active_viewport(); omni.kit.commands.execute('SetLightingMenuModeCommand',lighting_mode='Default',usd_context_name='')
        try: viewport.render_mode='RaytracedLighting'
        except Exception: pass
        depth_camera=None; depth_error=None
        timeline=omni.timeline.get_timeline_interface(); timeline.play()
        for _ in range(90): app.update()
        cube_mat=UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(Usd.TimeCode.Default()); cube_gt=np.array(cube_mat.ExtractTranslation(),dtype=float)
        marker_centers=[]
        for rr in range(6):
            for cc in range(6):
                mp=stage.GetPrimAtPath(f'{cube_path}/R2B_ArUco_23/cell_{rr}_{cc}')
                mm=UsdGeom.Xformable(mp).ComputeLocalToWorldTransform(Usd.TimeCode.Default()); marker_centers.append(np.array(mm.ExtractTranslation(),dtype=float))
        marker_gt=np.mean(marker_centers,axis=0)
        # Camera is intentionally fixed for each static candidate; there is no manipulation motion in this sweep.
        depth_camera=Camera(prim_path=cam_path,name='r2b_rear_wall_depth',resolution=(1280,720),render_product_path=viewport.get_render_product_path()); depth_camera.initialize(); depth_camera.add_distance_to_image_plane_to_frame()
        rows=[]
        for c in candidates():
            set_camera_view(eye=c['eye'],target=c['target'],camera_prim_path=cam_path); viewport.set_active_camera(cam_path)
            for _ in range(20): app.update()
            rgb=out/'rgb'/f"{c['id']}.png"; capture_viewport_to_file(viewport,file_path=str(rgb),is_hdr=False)
            for _ in range(20): app.update()
            depth_path=out/'rgb'/f"{c['id']}_depth.npy"
            try:
                d=np.asarray(depth_camera.get_depth()); np.save(depth_path,d); depth_info={'shape':list(d.shape),'finite_fraction':float(np.isfinite(d).mean())}
            except Exception as exc:
                depth_info={'error':repr(exc)}
            world_from_camera=np.array(UsdGeom.Xformable(camera.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),dtype=float).tolist()
            rows.append({**c,'rgb':str(rgb.relative_to(out)),'depth':str(depth_path.relative_to(out)),'depth_info':depth_info,'world_from_camera':world_from_camera,'target_cube_world_m':cube_gt.tolist(),'marker_gt_world_m':marker_gt.tolist()})
        (out/'candidates.json').write_text(json.dumps({'schema':'task28.r2b.rear_wall_sweep.v1','camera':{'prim':cam_path,'resolution_px':[1280,720],'focal_length_mm':20.0,'aperture_mm':[36.0,24.0],'marker_id':23,'marker_size_m':0.060},'candidates':rows,'physical_episode':False,'note':'static rendered sweep; no walls or collision geometry disabled'},indent=2)+'\n')
    except BaseException as exc:
        (out/'failure.json').write_text(json.dumps({'status':'FAILED','error':repr(exc)},indent=2)+'\n')
        raise
    finally:
        try: app.close()
        except Exception: pass

if __name__=='__main__': main()
