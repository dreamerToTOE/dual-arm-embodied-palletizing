#!/usr/bin/env python3
"""Bounded Stage-B timing stimulus. No ROS executor or fusion/controller changes."""
import argparse, copy, hashlib, json, math, sys, time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.time_alignment import RobotHistory
from offline.task28_g1_1.task28_validator.model import RobotModel


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write_rows(p,rows):p.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))


def commanded_position(t):
    # Stops make timestamp errors distinguishable from a fixed spatial bias.
    t=t%6.0
    x=.80 if t<.5 else .80+.10*(t-.5) if t<2.5 else 1.0 if t<3.2 else 1.0-.10*(t-3.2) if t<4.8 else .84
    return [x,.06,.26]


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);ap.add_argument('--updates',type=int);args=ap.parse_args()
    cfg=json.loads((R3/'config/p2a_frozen.json').read_text());out=R3/'evidence'/args.run_id;out.mkdir(parents=True,exist_ok=False);(out/'rgb').mkdir();(out/'depth').mkdir()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
    rows=[];oracles=[];frames=[];errors=[];subs=[]
    try:
        import cv2,omni.timeline,omni.usd,omni.physx,omni.kit.commands
        from pxr import Gf,Usd,UsdGeom,UsdPhysics,PhysxSchema
        from scipy.spatial.transform import Rotation
        from isaacsim.core.prims import SingleArticulation,SingleRigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from isaacsim.sensors.camera import Camera
        from omni.kit.viewport.utility import get_active_viewport
        from research.stage_b_vision_contact_r2b.scripts.r2b_camera_observer_smoke import make_marker,MARKER_GRID
        class PairedCamera(Camera):
            def _data_acquisition_callback(self,event):
                super()._data_acquisition_callback(event)
                f=self.get_current_frame()
                if f.get('rendering_frame',0)>0:
                    ref=copy.deepcopy(self._fabric_time_annotator.get_data())
                    self.research_meta={'callback_wall_s':time.monotonic(),'reference_time':ref,
                        'sdk_frame':int(f['rendering_frame']),'sdk_time_s':float(f['rendering_time']),
                        'rgb_depth_same_render_product':self._render_product_path,
                        'rgba_shape':list(np.shape(f.get('rgba'))),'depth_shape':list(np.shape(f.get('distance_to_image_plane')))}
        scene=ROOT/'isaac/scripts/task26_truck_box_scene.py';ns={'__name__':'__r3_p2a__','_SIDE_SUCTION_SCENARIO':'task27'};exec(compile(scene.read_text(),str(scene),'exec'),ns,ns)
        stage=omni.usd.get_context().get_stage();timeline=omni.timeline.get_timeline_interface()
        cube_path='/World/Task27/Supply/Cube_01';prim=stage.GetPrimAtPath(cube_path)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
        PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr().Set(True)
        make_marker(stage,cube_path,23,.06,MARKER_GRID)
        cam_path='/World/R2B_RearWallCamera';cam=UsdGeom.Camera.Define(stage,cam_path);cam.GetFocalLengthAttr().Set(20.);cam.GetHorizontalApertureAttr().Set(36.);cam.GetVerticalApertureAttr().Set(24.)
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        vp=get_active_viewport();vp.set_active_camera(cam_path);vp.render_mode='RaytracedLighting'
        assert omni.kit.commands.execute('SetLightingMenuModeCommand',lighting_mode='Default',usd_context_name='')[0]
        sensor=PairedCamera(prim_path=cam_path,resolution=tuple(cfg['resolution']));sensor.initialize();sensor.add_distance_to_image_plane_to_frame()
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        model=RobotModel();history=RobotHistory(model.fk,cfg['robot_history_max_samples'],cfg['max_interpolation_bracket_s'])
        arms={};cube=None;tool_local={};link_index={};step=0;physics_time=0.;start_time=None;latest_command=None
        def physics_sample(dt):
            nonlocal step,physics_time
            step+=1;physics_time+=float(dt)
            if not arms:return
            try:
                q={s:np.asarray(a.get_joint_positions())[:7].astype(float).tolist() for s,a in arms.items()};dq={s:np.asarray(a.get_joint_velocities())[:7].astype(float).tolist() for s,a in arms.items()}
                sample=dict(time_s=physics_time,physics_step=step,q=q,dq=dq,
                    suction_observation={'left':None,'right':None},suction_command={'left':None,'right':None},controller_command=latest_command)
                history.append(**sample);sample['tcp_fk']={s:model.fk(s,q[s]).tolist() for s in arms};sample['dt_s']=float(dt)
                sample['core_sim_time_s']=float(sensor._core_nodes_interface.get_sim_time());sample['timeline_s']=float(timeline.get_current_time());rows.append(sample)
                xyz,quat=cube.get_world_pose();tcp={}
                for s,a in arms.items():
                    pose=np.asarray(a._articulation_view._physics_view.get_link_transforms())[0,link_index[s]]
                    link=np.eye(4);link[:3,:3]=Rotation.from_quat(pose[3:7]).as_matrix();link[:3,3]=pose[:3]
                    tcp[s]=(link@tool_local[s]).tolist()
                oracles.append({'time_s':physics_time,'physics_step':step,'cube_xyz_m':np.asarray(xyz).tolist(),'tcp_physx':tcp})
            except Exception as exc:errors.append({'step':step,'error':repr(exc)})
        subs.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(physics_sample,False,0))
        timeline.play()
        for _ in range(90):app.update()
        for side in ('left','right'):
            a=SingleArticulation(f'/World/{side}_fr3');a.initialize();assert a.handles_initialized;arms[side]=a
        cube=SingleRigidPrim(cube_path);cube.initialize()
        cube_translate=next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps() if op.GetOpName().startswith('xformOp:translate:task26_pose'))
        cube_translate.Set(Gf.Vec3d(.8,.06,.26))
        for s,a in arms.items():
            link_index[s]=a._articulation_view.get_body_index('fr3_link8')
            link=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/{s}_fr3/fr3_link8')).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            tcp=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/{s}_fr3/fr3_link8/side_suction_tool/side_suction_tcp')).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            tool_local[s]=np.linalg.inv(link)@tcp
        home={s:np.asarray(a.get_joint_positions()).copy() for s,a in arms.items()}
        start_time=physics_time;pending=None;last_frame=-1
        for update in range(args.updates or cfg['main_updates']):
            t=physics_time-start_time;cube_translate.Set(Gf.Vec3d(*commanded_position(t)))
            command={s:home[s].copy() for s in arms}
            for s,a in arms.items():
                command[s][6]+=cfg['robot_wrist_amplitude_rad']*math.sin(2*math.pi*t/3.)
                a.apply_action(ArticulationAction(joint_positions=command[s]))
            latest_command={'sequence':update,'receipt_physics_time_s':physics_time,'positions_rad':{s:c[:7].tolist() for s,c in command.items()},'source':'research_local_small_wrist_stimulus'}
            if update%cfg['capture_every_updates']==0:pending={'request_update':update,'legacy_receipt_time_s':physics_time,'request_wall_s':time.monotonic()}
            app.update()
            if pending is None or update-pending['request_update']<cfg['legacy_wait_updates']:continue
            f=sensor.get_current_frame(clone=True);meta=copy.deepcopy(getattr(sensor,'research_meta',{}));fid=int(f.get('rendering_frame',0));rgb=np.asarray(f.get('rgba'));depth=np.asarray(f.get('distance_to_image_plane'))
            if fid<=last_frame or rgb.ndim!=3 or depth.ndim!=2:errors.append({'update':update,'reason':'missing_or_duplicate_frame','frame':fid});pending=None;continue
            arrival=time.monotonic();aligned=history.at(float(f['rendering_time']));legacy=history.at(pending['legacy_receipt_time_s']);last_frame=fid
            base=f'{len(frames):04d}_{fid}';rgb_path=out/'rgb'/f'{base}.png';depth_path=out/'depth'/f'{base}.npz';writer_start=time.monotonic();cv2.imwrite(str(rgb_path),cv2.cvtColor(rgb,cv2.COLOR_RGBA2BGRA));np.savez_compressed(depth_path,distance_to_image_plane=depth);writer_done=time.monotonic()
            frames.append(dict(frame_id=fid,render_time_s=float(f['rendering_time']),receipt_physics_time_s=physics_time,
                arrival_wall_s=arrival,callback_wall_s=meta.get('callback_wall_s'),writer_start_wall_s=writer_start,writer_complete_wall_s=writer_done,
                rgb=str(rgb_path.relative_to(out)),depth=str(depth_path.relative_to(out)),rgb_sha256=sha(rgb_path),depth_sha256=sha(depth_path),
                pair_id=base,metadata=meta,aligned_robot=aligned,legacy_robot=legacy,**pending));pending=None
        write_rows(out/'observations.jsonl',frames);write_rows(out/'robot_history.jsonl',rows);write_rows(out/'oracle.jsonl',oracles)
        manifest={'schema':'r3.p2a.dynamic_capture.v1','baseline_commit':'c01e18a1e30e7118a77e8e51f0856c304a74d08f','config_sha256':sha(R3/'config/p2a_frozen.json'),
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),R3/'time_alignment.py',scene,Path(__import__('inspect').getfile(RobotModel)))},
            'installed_camera_source':str(Path(__import__('inspect').getfile(Camera))),'installed_camera_sha256':sha(Path(__import__('inspect').getfile(Camera))),
            'K':np.asarray(sensor.get_intrinsics_matrix()).tolist(),'world_from_camera':np.array(UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).tolist(),
            'render_product':sensor._render_product_path,'joint_names':{s:a.dof_names for s,a in arms.items()},'tool_local':{s:v.tolist() for s,v in tool_local.items()},'updates':args.updates or cfg['main_updates'],'physics_steps':step,'physics_start_s':start_time,'physics_end_s':physics_time,
            'frames':len(frames),'errors':errors,'suction_semantics':'No Surface_Gripper instance in this isolated probe; observations/commands explicitly UNAVAILABLE, not inferred OPEN',
            'stimulus':'Kinematic tabletop Cube pose commands and +/-15mrad wrist drive. Not a physical pushing or Task27 success experiment.',
            'oracle_policy':'Cube/PhysX TCP oracle written separately; no oracle read by RobotHistory or P1 estimator',
            'data_sha256':{n:sha(out/n) for n in ('observations.jsonl','robot_history.jsonl','oracle.jsonl')}}
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('R3_P2A_CAPTURE_COMPLETE',json.dumps({'frames':len(frames),'errors':errors,'physics_end_s':physics_time}),flush=True)
    except BaseException as exc:
        import traceback
        failure={'status':'FAILED','error':repr(exc),'traceback':traceback.format_exc(),'partial_frames':len(frames),'physics_rows':len(rows),'callback_errors':errors[:5]}
        (out/'failure.json').write_text(json.dumps(failure,indent=2)+'\n');print('R3_P2A_FAILURE',json.dumps(failure),flush=True)
        raise
    finally:
        for sub in subs:sub.unsubscribe()
        app.close()


if __name__=='__main__':main()
