#!/usr/bin/env python3
"""Bounded RGB/depth temporal-code experiment in the unchanged Task27 camera."""
import argparse,copy,hashlib,inspect,json,sys,time
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
R3=ROOT/'research/stage_b_multisensor_r3'
from research.stage_b_multisensor_r3.r1_time_code import code_bits,SOURCES
from research.stage_b_multisensor_r3.planar_pose_estimator import camera_axes_from_usd
from offline.task28_g1_1.task28_validator.model import RobotModel


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save_rows(p,rows):p.write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in rows))


def stimulus(t,cfg,split):
    x=cfg[split+'_start_x_m'];remaining=max(0.,t)
    for i,(duration,velocity) in enumerate(cfg[split]):
        elapsed=min(remaining,duration);x+=elapsed*velocity;remaining-=elapsed
        if remaining<=1e-9:return [x,.06,.26],i,velocity
    return [x,.06,.26],len(cfg[split]),0.


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);ap.add_argument('--split',choices=['calibration','holdout'],required=True);ap.add_argument('--updates',type=int);ap.add_argument('--binding',type=Path);args=ap.parse_args()
    cfg_path=R3/'config/r1_time_experiment.json';cfg=json.loads(cfg_path.read_text())
    if args.split=='holdout':assert args.binding and args.binding.exists(),'freeze calibration before collecting holdout'
    assert args.updates is None or 1<=args.updates<=cfg['updates']
    out=R3/'evidence'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    for name in ['clock_rgb','clock_depth','representatives']:(out/name).mkdir()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
    frames=[];physics=[];oracle=[];commands=[];errors=[];subscriptions=[]
    try:
        import cv2,omni.timeline,omni.usd,omni.physx,omni.graph.core as og
        import omni.kit.commands
        sys.path.insert(0,str(R3/'native/build'))
        from _r1_fabric import RationalTime
        from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Sdf,Gf
        from scipy.spatial.transform import Rotation
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.prims import SingleArticulation,RigidPrim
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from research.stage_b_vision_contact_r2b.scripts.r2b_camera_observer_smoke import make_marker,MARKER_GRID
        api_probe={'RationalTime_doc':RationalTime.__doc__,'cpp_type':'omni::fabric::RationalTime from installed header; local Python binding required','native_build':json.loads((R3/'native/build/build_manifest.json').read_text())}
        rt_probe=RationalTime(1,60)
        api_probe['constructed_type']=str(type(rt_probe));api_probe['constructed_repr']=str(rt_probe)
        (out/'api_probe.json').write_text(json.dumps(api_probe,indent=2)+'\n')
        scene=ROOT/'isaac/scripts/task26_truck_box_scene.py';ns={'__name__':'__r3_r1__','_SIDE_SUCTION_SCENARIO':'task27'};exec(compile(scene.read_text(),str(scene),'exec'),ns,ns)
        stage=omni.usd.get_context().get_stage();timeline=omni.timeline.get_timeline_interface()
        cube_path='/World/Task27/Supply/Cube_01';prim=stage.GetPrimAtPath(cube_path)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True);PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr().Set(True)
        make_marker(stage,cube_path,23,.06,MARKER_GRID)
        cube=RigidPrim(cube_path,reset_xform_properties=False)
        cube_tr=next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        cube_tr.Set(Gf.Vec3d(cfg[args.split+'_start_x_m'],.06,.26))
        cube_rot=UsdGeom.Xformable(prim).AddOrientOp(opSuffix='r1_tabletop');cube_rot.Set(Gf.Quatf(1,0,0,0))
        cam_path='/World/R2B_RearWallCamera';cam=UsdGeom.Camera.Define(stage,cam_path);cam.GetFocalLengthAttr().Set(20.);cam.GetHorizontalApertureAttr().Set(36.);cam.GetVerticalApertureAttr().Set(24.)
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        vp=get_active_viewport();vp.set_active_camera(cam_path);vp.render_mode='RaytracedLighting'
        assert omni.kit.commands.execute('SetLightingMenuModeCommand',lighting_mode='Default',usd_context_name='')[0]
        class TimeCamera(Camera):
            def _data_acquisition_callback(self,event):
                super()._data_acquisition_callback(event)
                f=self.get_current_frame()
                if f.get('rendering_frame',0)<=0 or f['rendering_frame']==getattr(self,'time_meta',{}).get('frame_id'):return
                ref=copy.deepcopy(self._fabric_time_annotator.get_data());rt=RationalTime(int(ref['referenceTimeNumerator']),int(ref['referenceTimeDenominator']))
                fabric_time=float(self._core_nodes_interface.get_sim_time_at_time(rt))
                dispatcher=og.get_node_by_path('/Render/PostProcess/SDGPipeline/PostProcessDispatcher')
                graph_ref={k:int(dispatcher.get_attribute('outputs:'+k).get()) for k in ('referenceTimeNumerator','referenceTimeDenominator')} if dispatcher else None
                self.time_meta={'frame_id':int(f['rendering_frame']),'sdk_time_s':float(f['rendering_time']),'fabric_sim_time_s':fabric_time,
                    'reference_time':ref,'dispatcher_reference':graph_ref,'callback_wall_s':time.monotonic(),'callback_physics_step':step,
                    'callback_core_time_s':float(self._core_nodes_interface.get_sim_time())}
        sensor=TimeCamera(prim_path=cam_path,resolution=tuple(cfg['resolution']));sensor.initialize();sensor.add_distance_to_image_plane_to_frame()
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        M=np.array(UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()));A=camera_axes_from_usd(M);C=M[3,:3];K=np.array(sensor.get_intrinsics_matrix())
        material=UsdShade.Material.Define(stage,'/World/R1Timing/Material');shader=UsdShade.Shader.Define(stage,'/World/R1Timing/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
        reader=UsdShade.Shader.Define(stage,'/World/R1Timing/Material/Color');reader.CreateIdAttr('UsdPrimvarReader_float3');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor');reader.CreateOutput('result',Sdf.ValueTypeNames.Float3)
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(),'result');material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        u0,v0=cfg['clock_first_pixel'];spacing=cfg['clock_cell_spacing_px'];roi=[u0-12,v0-12,u0+15*spacing+12,v0+5*spacing+12]
        cells={};layout={};last_codes={}
        q=Rotation.from_matrix(A).as_quat();orient=Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3]))
        for source_index,source in enumerate(SOURCES):
            cells[source]=[];layout[source]=[]
            for bit in range(16):
                pair=[];pixels=[]
                for inverse in range(2):
                    u=u0+bit*spacing;v=v0+(source_index*2+inverse)*spacing
                    box=UsdGeom.Cube.Define(stage,f'/World/R1Timing/{source}_{bit}_{inverse}');box.CreateSizeAttr(1.);color=box.CreateDisplayColorAttr();color.Set([Gf.Vec3f(0,0,0)])
                    xf=UsdGeom.Xformable(box);tr=xf.AddTranslateOp();xf.AddOrientOp().Set(orient);scale=xf.AddScaleOp();UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(material)
                    pair.append((color,tr,scale,np.linalg.inv(K)@np.array([u,v,1.])));pixels.append([u-roi[0],v-roi[1]])
                cells[source].append(pair);layout[source].append(pixels)
        verified_writes=0
        def encode(source,value):
            nonlocal verified_writes
            previous=last_codes.get(source);bits=code_bits(value)
            for i,pair in enumerate(cells[source]):
                for j,(color,tr,scale,ray) in enumerate(pair):
                    b=int(bits[j,i])
                    if previous is not None and b==int(code_bits(previous)[j,i]):continue
                    depth=cfg['clock_depth_one_m'] if b else cfg['clock_depth_zero_m'];position=C+A@(ray*depth)
                    color.Set([Gf.Vec3f(b,b,b)]);tr.Set(Gf.Vec3d(*position));scale.Set(Gf.Vec3d(cfg['clock_cell_size_px']*depth/K[0,0],cfg['clock_cell_size_px']*depth/K[1,1],.002))
                    assert abs(float(color.Get()[0][0])-b)<1e-6 and np.linalg.norm(np.array(tr.Get())-position)<1e-8
                    verified_writes+=1
            last_codes[source]=value
        for source in SOURCES:encode(source,0)
        step=0;physics_time=0.;mode='warmup';arms={};model=RobotModel();cube_ready=False
        def pre_step(dt):
            if mode!='pattern':encode('pre',step+1)
        def post_step(dt):
            nonlocal step,physics_time
            step+=1;physics_time+=float(dt)
            if mode!='pattern':encode('post',step)
            row={'physics_step':step,'time_s':physics_time,'core_time_s':float(sensor._core_nodes_interface.get_sim_time()),'mode':mode,'pre_code':last_codes['pre'],'post_code':last_codes['post'],'app_code':last_codes['app']}
            if arms:
                row.update(q={s:np.asarray(a.get_joint_positions())[:7].tolist() for s,a in arms.items()},dq={s:np.asarray(a.get_joint_velocities())[:7].tolist() for s,a in arms.items()},suction_observation={'left':None,'right':None},suction_command={'left':None,'right':None},controller_command=None)
                row['tcp_fk']={s:model.fk(s,row['q'][s]).tolist() for s in arms}
            physics.append(row)
            if cube_ready:
                xyz,quat=cube.get_world_poses();oracle.append({'physics_step':step,'time_s':physics_time,'cube_xyz_m':xyz[0].tolist(),'cube_quaternion_wxyz':quat[0].tolist(),'kinematic_enabled':bool(UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get())})
        subscriptions.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(pre_step,True,0))
        subscriptions.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step,False,0))
        arm_handles={s:SingleArticulation(f'/World/{s}_fr3') for s in ('left','right')}
        timeline.play()
        for _ in range(90):app.update()
        for s,a in arm_handles.items():a.initialize();assert a.handles_initialized;arms[s]=a
        cube.initialize();cube_ready=True
        def capture(label,expected_pattern=None,settled=False):
            f=sensor.get_current_frame(clone=True);meta=copy.deepcopy(getattr(sensor,'time_meta',{}))
            assert meta and meta['frame_id']==f['rendering_frame'],'no valid API/frame metadata'
            rgba=np.asarray(f['rgba']);depth=np.asarray(f['distance_to_image_plane']);x0,y0,x1,y1=roi;rgb_patch=rgba[y0:y1,x0:x1].copy();depth_patch=depth[y0:y1,x0:x1].copy()
            n=len(frames);base=f'{n:04d}_{meta["frame_id"]}';rp=out/'clock_rgb'/f'{base}.png';dp=out/'clock_depth'/f'{base}.npz';arrival=time.monotonic();writer_start=time.monotonic()
            cv2.imwrite(str(rp),cv2.cvtColor(rgb_patch,cv2.COLOR_RGBA2BGRA));np.savez_compressed(dp,distance_to_image_plane=depth_patch)
            representative=None
            if n%30==0:
                representative=f'representatives/{base}.png';cv2.imwrite(str(out/representative),cv2.cvtColor(rgba,cv2.COLOR_RGBA2BGRA))
            frames.append(dict(**meta,mode=label,expected_pattern=expected_pattern,pattern_settled=settled,
                arrival_wall_s=arrival,arrival_physics_time_s=physics_time,writer_start_wall_s=writer_start,writer_complete_wall_s=time.monotonic(),
                rgb=str(rp.relative_to(out)),depth=str(dp.relative_to(out)),rgb_sha256=sha(rp),depth_sha256=sha(dp),representative=representative,
                corners_px=None))
        mode='pattern'
        for value in cfg['pattern_check_codes']:
            for source in SOURCES:encode(source,value)
            for i in range(cfg['pattern_updates']):app.update();capture('pattern',value,i>=3)
        mode='dynamic';start=physics_time
        for update in range(args.updates or cfg['updates']):
            t=physics_time-start;xyz,phase,velocity=stimulus(t,cfg,args.split);cube_tr.Set(Gf.Vec3d(*xyz));cube_rot.Set(Gf.Quatf(1,0,0,0));encode('app',step)
            commands.append({'physics_step':step,'time_s':physics_time,'relative_time_s':t,'cube_command_xyz_m':xyz,'phase':phase,'command_velocity_m_s':velocity})
            app.update();capture('dynamic')
        save_rows(out/'observations.jsonl',frames);save_rows(out/'physics_history.jsonl',physics);save_rows(out/'oracle.jsonl',oracle);save_rows(out/'stimulus_commands.jsonl',commands)
        camera_source=Path(inspect.getfile(Camera));fingerprint_fields={'camera_source_sha256':sha(camera_source),'experiment_config_sha256':sha(cfg_path),'capture_source_sha256':sha(Path(__file__)),'K':K.tolist(),'M':M.tolist(),'render_product_kind':'Camera own RP / app.update / RaytracedLighting'}
        fingerprint=hashlib.sha256(json.dumps(fingerprint_fields,sort_keys=True).encode()).hexdigest()
        manifest={'schema':'r3.p2a_r1.time_capture.v1','split':args.split,'baseline_sha':cfg['baseline_sha'],'configuration_fingerprint':fingerprint,'fingerprint_fields':fingerprint_fields,
            'binding_sha256_at_capture':sha(args.binding) if args.binding else None,'K':K.tolist(),'world_from_camera':M.tolist(),'clock_roi':roi,'clock_layout':layout,'clock_source':'noncolliding emissive USD cells; RGB color and optical depth independently encode app/pre/post uint16 plus complements',
            'verified_usd_bit_writes':verified_writes,'frame_count':len(frames),'dynamic_frames':sum(f['mode']=='dynamic' for f in frames),'physics_end_s':physics_time,'errors':errors,
            'render_product':sensor._render_product_path,'sdg_graph':str(sensor._sdg_graph_pipeline),'api_probe':api_probe,
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),R3/'r1_time_code.py',scene]},
            'data_sha256':{name:sha(out/name) for name in ['observations.jsonl','physics_history.jsonl','oracle.jsonl','stimulus_commands.jsonl']}}
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('R3_R1_COMPLETE',json.dumps({'frames':len(frames),'dynamic_frames':manifest['dynamic_frames'],'verified_writes':verified_writes}),flush=True)
    except BaseException as exc:
        import traceback
        save_rows(out/'partial_observations.jsonl',frames);save_rows(out/'partial_physics_history.jsonl',physics);save_rows(out/'partial_oracle.jsonl',oracle)
        (out/'failure.json').write_text(json.dumps({'error':repr(exc),'traceback':traceback.format_exc(),'frames':len(frames)},indent=2)+'\n');print('R3_R1_FAILURE',repr(exc),flush=True);raise
    finally:
        for s in subscriptions:s.unsubscribe()
        app.close()


if __name__=='__main__':main()
