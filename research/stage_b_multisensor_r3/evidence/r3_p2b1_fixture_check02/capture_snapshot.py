#!/usr/bin/env python3
"""Robot-first bounded independent motion probe; two fresh camera epochs, unchanged layout."""
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
    ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);ap.add_argument('--condition',choices=['fixture_check','connected','loss','slip'],required=True);ap.add_argument('--split',choices=['holdout'],default='holdout');ap.add_argument('--updates',type=int);ap.add_argument('--binding',type=Path);args=ap.parse_args()
    cfg_path=R3/'config/p2b1_frozen.json';cfg=json.loads(cfg_path.read_text());condition=cfg['conditions'][args.condition]

    assert args.updates is None or 1<=args.updates<=cfg['updates']
    out=R3/'evidence'/args.run_id;out.mkdir(parents=True,exist_ok=False)
    for name in ['rgb','depth','representatives']:(out/name).mkdir()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'create_new_stage':True,'renderer':'RaytracedLighting','multi_gpu':False,'anti_aliasing':2,'sync_loads':True})
    frames=[];physics=[];oracle=[];commands=[];errors=[];subscriptions=[];probes=[];robot_rows=[];tcp_oracle=[];epochs=[];stream_events=[]
    try:
        import cv2,omni.timeline,omni.usd,omni.physx,omni.graph.core as og
        import omni.kit.commands
        import omni.replicator.core as rep
        from usdrt import Usd as RtUsd, Rt
        sys.path.insert(0,str(R3/'native/build'))
        from _r1_fabric import RationalTime
        from pxr import Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Sdf,Gf
        from scipy.spatial.transform import Rotation
        from isaacsim.sensors.camera import Camera
        from isaacsim.core.prims import RigidPrim,SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from isaacsim.core.utils.viewports import set_camera_view
        from omni.kit.viewport.utility import get_active_viewport
        from research.stage_b_vision_contact_r2b.scripts.r2b_camera_observer_smoke import make_marker,MARKER_GRID
        api_probe={'RationalTime_doc':RationalTime.__doc__,'cpp_type':'omni::fabric::RationalTime from installed header; local Python binding required','native_build':json.loads((R3/'native/build/build_manifest.json').read_text())}
        rt_probe=RationalTime(1,60)
        api_probe['constructed_type']=str(type(rt_probe));api_probe['constructed_repr']=str(rt_probe)
        (out/'api_probe.json').write_text(json.dumps(api_probe,indent=2)+'\n')
        scene=ROOT/'isaac/scripts/task26_truck_box_scene.py';ns={'__name__':'__r3_p2b0__','_SIDE_SUCTION_SCENARIO':'task27'};exec(compile(scene.read_text(),str(scene),'exec'),ns,ns)
        stage=omni.usd.get_context().get_stage();timeline=omni.timeline.get_timeline_interface()
        fabric_stage=RtUsd.Stage.Attach(omni.usd.get_context().get_stage_id())
        cube_path='/World/Task27/Supply/Cube_01';prim=stage.GetPrimAtPath(cube_path)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(False);PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr().Set(False)
        cube=RigidPrim(cube_path,reset_xform_properties=False)
        cube_tr=next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps() if op.GetOpType()==UsdGeom.XformOp.TypeTranslate)
        cube_tr.Set(Gf.Vec3d(*condition['cube_start_m']))
        cube_rot=UsdGeom.Xformable(prim).AddOrientOp(opSuffix='r1_tabletop');cube_rot.Set(Gf.Quatf(1,0,0,0))
        cam_path='/World/R2B_RearWallCamera';cam=UsdGeom.Camera.Define(stage,cam_path);default_clipping=list(cam.GetClippingRangeAttr().Get());cam.GetClippingRangeAttr().Set(tuple(cfg['clipping_range_m']));cam.GetFocalLengthAttr().Set(20.);cam.GetHorizontalApertureAttr().Set(36.);cam.GetVerticalApertureAttr().Set(24.)
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        vp=get_active_viewport();vp.set_active_camera(cam_path);vp.render_mode='RaytracedLighting'
        assert omni.kit.commands.execute('SetLightingMenuModeCommand',lighting_mode='Default',usd_context_name='')[0]
        class TimeCamera(Camera):
            def _data_acquisition_callback(self,event):
                entry_wall=time.monotonic();entry_step=step
                super()._data_acquisition_callback(event)
                f=self.get_current_frame()
                if f.get('rendering_frame',0)<=0 or f['rendering_frame']==getattr(self,'time_meta',{}).get('frame_id'):return
                ref=copy.deepcopy(self._fabric_time_annotator.get_data());rt=RationalTime(int(ref['referenceTimeNumerator']),int(ref['referenceTimeDenominator']))
                fabric_time=float(self._core_nodes_interface.get_sim_time_at_time(rt))
                dispatcher=og.get_node_by_path('/Render/PostProcess/SDGPipeline/PostProcessDispatcher')
                graph_ref={k:int(dispatcher.get_attribute('outputs:'+k).get()) for k in ('referenceTimeNumerator','referenceTimeDenominator')} if dispatcher else None
                self.time_meta={'camera_stream_id':self.stream_id,'camera_epoch_id':self.capture_epoch_id,'frame_id':int(f['rendering_frame']),'sdk_time_s':float(f['rendering_time']),'fabric_sim_time_s':fabric_time,
                    'reference_time':ref,'dispatcher_reference':graph_ref,'callback_wall_s':time.monotonic(),'callback_physics_step':step,
                    'callback_core_time_s':float(self._core_nodes_interface.get_sim_time()),'callback_entry_wall_s':entry_wall,'callback_entry_step':entry_step}
                if cube_ready:scene_probe('camera_after_annotators',self.time_meta['frame_id'])
        sensor=TimeCamera(prim_path=cam_path,resolution=tuple(cfg['resolution']));core=sensor._core_nodes_interface
        assert sensor._render_product is None, 'no measurement render product before measured robot history'
        set_camera_view(eye=cfg['camera_eye'],target=cfg['camera_target'],camera_prim_path=cam_path)
        M=np.array(UsdGeom.Xformable(cam.GetPrim()).ComputeLocalToWorldTransform(Usd.TimeCode.Default()));A=camera_axes_from_usd(M);C=M[3,:3];K=np.array(cfg['camera_matrix_reference'])
        material=UsdShade.Material.Define(stage,'/World/R1Timing/Material');shader=UsdShade.Shader.Define(stage,'/World/R1Timing/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
        reader=UsdShade.Shader.Define(stage,'/World/R1Timing/Material/Color');reader.CreateIdAttr('UsdPrimvarReader_float3');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor');reader.CreateOutput('result',Sdf.ValueTypeNames.Float3)
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(),'result');material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        u0,v0=cfg['clock_first_pixel'];spacing=cfg['clock_cell_spacing_px'];roi=cfg['capture_roi']
        cells={};layout={};last_codes={}
        q=Rotation.from_matrix(A).as_quat();orient=Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3]))
        for source_index,source in enumerate((*SOURCES,'birth')):
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
        for source in (*SOURCES,'birth'):encode(source,0)
        step=0;physics_time=0.;mode='warmup';cube_ready=False;arms={};tool_local={};link_index={};latest_robot_command=None;model=RobotModel()
        identity={'run_id':args.run_id,'epoch_id':0,'time_base_id':'physics_dt_since_timeline_play'}
        def scene_probe(phase,frame_id=None):
            assert cube.is_physics_handle_valid(),'Cube oracle must use a live PhysX view'
            xyz,quat=cube.get_world_poses();matrix=np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
            fabric={}
            for name,path in [('cube',cube_path),('clock_post_bit0','/World/R1Timing/post_0_0')]:
                fp=fabric_stage.GetPrimAtPath(path);xf=Rt.Xformable(fp);values={}
                for key,method in [('world_position',xf.GetWorldPositionAttr),('world_matrix',xf.GetFabricHierarchyWorldMatrixAttr),('local_matrix',xf.GetLocalMatrixAttr)]:
                    attr=method();value=attr.Get() if attr else None
                    values[key]=np.asarray(value).tolist() if value is not None else None
                attr_name=str(cube_tr.GetOpName()) if name=='cube' else 'xformOp:translate'
                attr=fp.GetAttribute(attr_name);value=attr.Get() if attr else None
                values['authored_translate_read_through_usdrt']=list(value) if value is not None else None
                values['authored_translate_attr_name']=attr_name;values['fabric_prim_valid']=bool(fp)
                fabric[name]=values
            probes.append({'phase':phase,'frame_id':frame_id,'completed_physics_step':step,'time_s':physics_time,'mode':mode,
                'cube_physx_xyz_m':xyz[0].tolist(),'cube_physx_quaternion_wxyz':quat[0].tolist(),'cube_usd_matrix':matrix.tolist(),
                'fabric_current_readback':fabric,'clock_producer_values':dict(last_codes)})
        def record_robot(dt,acquisition_stage='PhysX post-step callback, pre=False order=0'):
            try:
                q={s:np.asarray(a.get_joint_positions())[:7].astype(float).tolist() for s,a in arms.items()}
                dq={s:np.asarray(a.get_joint_velocities())[:7].astype(float).tolist() for s,a in arms.items()}
                assert all(np.isfinite(q[s]).all() and np.isfinite(dq[s]).all() for s in arms)
                robot_rows.append(dict(identity,time_s=physics_time,physics_step=step,dt_s=float(dt),
                    core_sim_time_s=float(core.get_sim_time()),timeline_s=float(timeline.get_current_time()),
                    q=q,dq=dq,tcp_fk={s:model.fk(s,q[s]).tolist() for s in arms},
                    joint_state_source='Isaac4.5 SingleArticulation -> live PhysX articulation joint tensors',
                    acquisition_stage=acquisition_stage,
                    validity={'finite_q_dq':True,'articulation_handles_initialized':all(a.handles_initialized for a in arms.values())},
                    suction_observation={'left':None,'right':None},suction_command={'left':None,'right':None},
                    controller_command=copy.deepcopy(latest_robot_command),contact_observation=None,force_observation=None))
                tcp={}
                for s,a in arms.items():
                    pose=np.asarray(a._articulation_view._physics_view.get_link_transforms())[0,link_index[s]]
                    link=np.eye(4);link[:3,:3]=Rotation.from_quat(pose[3:7]).as_matrix();link[:3,3]=pose[:3]
                    tcp[s]=(link@tool_local[s]).tolist()
                tcp_oracle.append(dict(identity,time_s=physics_time,physics_step=step,tcp_physx=tcp))
            except Exception as exc:errors.append({'step':step,'channel':'robot_history','error':repr(exc)})

        def pre_step(dt):
            if mode!='pattern':encode('pre',step+1)
            if cube_ready:scene_probe('pre_step')
        def post_step(dt):
            nonlocal step,physics_time
            step+=1;physics_time+=float(dt)
            if mode!='pattern':encode('post',step)
            row={'physics_step':step,'time_s':physics_time,'core_time_s':float(core.get_sim_time()),'mode':mode,'pre_code':last_codes['pre'],'post_code':last_codes['post'],'app_code':last_codes['app']}
            physics.append(row)
            if arms:record_robot(dt)
            if cube_ready:
                scene_probe('post_step')
                xyz,quat=cube.get_world_poses();oracle.append({'physics_step':step,'time_s':physics_time,'cube_xyz_m':xyz[0].tolist(),'cube_quaternion_wxyz':quat[0].tolist(),'kinematic_enabled':bool(UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get()),'fixture_joint_enabled':bool(joint.GetJointEnabledAttr().Get())})
        subscriptions.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(pre_step,True,0))
        subscriptions.append(omni.physx.get_physx_interface().subscribe_physics_on_step_events(post_step,False,0))
        arm_handles={s:SingleArticulation(f'/World/{s}_fr3') for s in ('left','right')}
        timeline.play()
        for _ in range(cfg['warmup_updates']):app.update()
        for s,a in arm_handles.items():
            a.initialize();assert a.handles_initialized
            assert [name.split('_')[-1] for name in a.dof_names[:7]]==[f'joint{i}' for i in range(1,8)]
            link_index[s]=a._articulation_view.get_body_index('fr3_link8')
            link=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/{s}_fr3/fr3_link8')).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            tcp=np.array(UsdGeom.Xformable(stage.GetPrimAtPath(f'/World/{s}_fr3/fr3_link8/side_suction_tool/side_suction_tcp')).ComputeLocalToWorldTransform(Usd.TimeCode.Default())).T
            tool_local[s]=np.linalg.inv(link)@tcp
        for side,a in arm_handles.items():
            q0=np.asarray(a.get_joint_positions()).copy()
            if side=='left':q0[:7]=condition['left_initial_q']
            a.set_joint_positions(q0);a.set_joint_velocities(np.zeros_like(q0));a.apply_action(ArticulationAction(joint_positions=q0))
        for _ in range(cfg['settle_robot_updates']):app.update()
        arms.update(arm_handles);home={s:np.asarray(a.get_joint_positions()).copy() for s,a in arms.items()}
        cube.initialize();cube_ready=True
        # Controlled mechanical attachment fixture, not a Surface_Gripper/contact model.
        link_pose=np.asarray(arms['left']._articulation_view._physics_view.get_link_transforms())[0,link_index['left']]
        WL=np.eye(4);WL[:3,:3]=Rotation.from_quat(link_pose[3:7]).as_matrix();WL[:3,3]=link_pose[:3]
        xyz,quat=cube.get_world_poses();WC=np.eye(4);WC[:3,:3]=Rotation.from_quat(np.roll(quat[0],-1)).as_matrix();WC[:3,3]=xyz[0]
        local=np.linalg.inv(WL)@WC;qr=Rotation.from_matrix(local[:3,:3]).as_quat()
        joint=UsdPhysics.FixedJoint.Define(stage,'/World/R3P2B1FixtureJoint')
        joint.CreateBody0Rel().SetTargets(['/World/left_fr3/fr3_link8']);joint.CreateBody1Rel().SetTargets([cube_path])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local[:3,3]));joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(qr[3]),Gf.Vec3f(*qr[:3])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1,0,0,0));joint.CreateCollisionEnabledAttr().Set(False)
        fixture_enabled=True
        def start_stream(epoch_id):
            nonlocal sensor,identity
            previous_product=None
            if epoch_id:
                previous_product=sensor._render_product_path
                sensor.pause();sensor._timer_reset_callback=None;sensor._stage_open_callback=None
                for annotator in sensor.supported_annotators:getattr(sensor,'remove_'+annotator+'_from_frame')()
                sensor._rgb_annotator.detach([previous_product]);sensor._fabric_time_annotator.detach([previous_product])
                sensor._render_product.destroy();sensor._render_product=None
                sensor=TimeCamera(prim_path=cam_path,name=f'r1_epoch_{epoch_id}',resolution=tuple(cfg['resolution']))
            identity=dict(identity,epoch_id=epoch_id)
            record_robot(0.,'genuinely measured initialized boundary, before camera RP creation')
            assert robot_rows[-1]['time_s']==physics_time and robot_rows[-1]['epoch_id']==epoch_id
            birth_time=physics_time
            marker_root=cube_path+'/R2B_ArUco_23'
            if stage.GetPrimAtPath(marker_root):stage.RemovePrim(marker_root)
            make_marker(stage,cube_path,23,.06,MARKER_GRID)
            encode('birth',cfg['birth_codes'][epoch_id])
            sensor.stream_id=f'{args.run_id}:camera:{epoch_id}';sensor.capture_epoch_id=epoch_id
            product=rep.create.render_product(cam_path,tuple(cfg['resolution']),force_new=True,name=f'R3P2B0R1_epoch{epoch_id}')
            sensor._render_product=product;sensor._render_product_path=product.path
            sensor.initialize();sensor.add_distance_to_image_plane_to_frame()
            assert np.array_equal(K,np.array(sensor.get_intrinsics_matrix()))
            assert sensor._render_product_path!=previous_product
            epoch=dict(identity,camera_stream_id=sensor.stream_id,birth_time_s=birth_time,
                birth_step=step,robot_first_time_s=robot_rows[-1]['time_s'],birth_code=cfg['birth_codes'][epoch_id],
                render_product_created_time_s=physics_time,fresh_render_product=True,
                render_product=sensor._render_product_path,previous_render_product_destroyed=previous_product,
                marker_created_after_first_robot_sample=True,robot_rows_before_render_product=len(robot_rows))
            epochs.append(epoch);stream_events.append(dict(event='fresh_stream_created',**epoch))
        start_stream(0)
        def capture(label,expected_pattern=None,settled=False):
            f=sensor.get_current_frame(clone=True);meta=copy.deepcopy(getattr(sensor,'time_meta',{}))
            if not meta or meta['frame_id']!=f['rendering_frame'] or 'distance_to_image_plane' not in f:
                stream_events.append(dict(event='frame_not_yet_initialized',**identity,time_s=physics_time));return
            assert meta['camera_epoch_id']==identity['epoch_id'] and meta['camera_stream_id']==sensor.stream_id
            rgba=np.asarray(f['rgba']);depth=np.asarray(f['distance_to_image_plane']);x0,y0,x1,y1=roi;rgb_patch=rgba[y0:y1,x0:x1].copy();depth_patch=depth[y0:y1,x0:x1].copy()
            n=len(frames);base=f'{n:04d}_{meta["frame_id"]}';rp=out/'rgb'/f'{base}.png';dp=out/'depth'/f'{base}.npz';arrival=time.monotonic();writer_start=time.monotonic()
            cv2.imwrite(str(rp),cv2.cvtColor(rgb_patch,cv2.COLOR_RGBA2BGRA));np.savez_compressed(dp,distance_to_image_plane=depth_patch)
            representative=None
            if n%30==0:
                representative=f'representatives/{base}.png';cv2.imwrite(str(out/representative),cv2.cvtColor(rgba,cv2.COLOR_RGBA2BGRA))
            frames.append(dict(**identity,**meta,mode=label,expected_pattern=expected_pattern,pattern_settled=settled,
                arrival_wall_s=arrival,arrival_physics_time_s=physics_time,writer_start_wall_s=writer_start,writer_complete_wall_s=time.monotonic(),
                rgb=str(rp.relative_to(out)),depth=str(dp.relative_to(out)),rgb_sha256=sha(rp),depth_sha256=sha(dp),representative=representative,
                corners_px=None))
        mode='stream_settle'
        for _ in range(cfg['stream_settle_updates']):app.update();capture('stream_settle')
        mode='dynamic';start=physics_time
        loss_done=False
        for update in range(args.updates or cfg['updates']):
            t=physics_time-start
            u=0. if t<cfg['initial_static_s'] else .5*(1-np.cos(2*np.pi*(t-cfg['initial_static_s'])/condition['period_s']))
            qleft=(1-u)*np.array(condition['left_initial_q'])+u*np.array(condition['left_end_q'])
            assert model.within_limits(qleft)
            target=home['left'].copy();target[:7]=qleft
            arms['left'].apply_action(ArticulationAction(joint_positions=target))
            latest_robot_command={'source':'p2b1_local_fixture_probe','sequence':update,'physics_time_at_issue_s':physics_time,
                'positions_rad':{'left':qleft.tolist(),'right':home['right'][:7].tolist()},'request':'maintain_initial_connection'}
            commands.append(dict(identity,physics_step=step,time_s=physics_time,relative_time_s=t,**latest_robot_command))
            if condition['loss_time_s'] is not None and t>=condition['loss_time_s'] and not loss_done:
                joint.CreateJointEnabledAttr().Set(False);loss_done=True
                if condition['slip_velocity_m_s']:
                    cube.set_linear_velocities(np.array([[0.,condition['slip_velocity_m_s'],0.]]))
                # This perturbation receipt is oracle-only: do not export to observable command history.
                probes.append({'phase':'oracle_fixture_release','physics_step':step,'time_s':physics_time,'slip_velocity_m_s':condition['slip_velocity_m_s']})
            encode('app',step);app.update()
            if update%cfg['capture_every_updates']==0:capture('dynamic')
        save_rows(out/'camera_epochs.jsonl',epochs);save_rows(out/'stream_events.jsonl',stream_events)
        save_rows(out/'oracle_scene_probes.jsonl',probes)
        save_rows(out/'robot_history.jsonl',robot_rows);save_rows(out/'oracle_tcp.jsonl',tcp_oracle)
        save_rows(out/'observations.jsonl',frames);save_rows(out/'physics_history.jsonl',physics);save_rows(out/'oracle.jsonl',oracle);save_rows(out/'stimulus_commands.jsonl',commands)
        camera_source=Path(inspect.getfile(Camera));fingerprint_fields={'camera_source_sha256':sha(camera_source),'experiment_config_sha256':sha(cfg_path),'capture_source_sha256':sha(Path(__file__)),'K':K.tolist(),'M':M.tolist(),'render_product_kind':'Camera own RP / app.update / RaytracedLighting'}
        fingerprint=hashlib.sha256(json.dumps(fingerprint_fields,sort_keys=True).encode()).hexdigest()
        manifest={'condition':args.condition,'experiment_source':'controlled PhysX fixed joint, not validated suction contact','schema':'r3.p2b1.time_capture.v1','split':args.split,'baseline_sha':cfg['baseline_sha'],'configuration_fingerprint':fingerprint,'fingerprint_fields':fingerprint_fields,
            'binding_sha256_at_capture':sha(args.binding) if args.binding else None,'K':K.tolist(),'world_from_camera':M.tolist(),'capture_roi':roi,'clock_layout':layout,'clock_source':'noncolliding emissive USD cells; RGB color and optical depth independently encode app/pre/post uint16 plus complements',
            'verified_usd_bit_writes':verified_writes,'frame_count':len(frames),'dynamic_frames':sum(f['mode']=='dynamic' for f in frames),'physics_end_s':physics_time,'errors':errors,
            'default_clipping_range':default_clipping,'clipping_range':list(cam.GetClippingRangeAttr().Get()),'render_product':sensor._render_product_path,'sdg_graph':str(sensor._sdg_graph_pipeline),'api_probe':api_probe,
            'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in [Path(__file__),R3/'r1_time_code.py',scene]},
            'data_sha256':{name:sha(out/name) for name in ['observations.jsonl','physics_history.jsonl','oracle.jsonl','stimulus_commands.jsonl','oracle_scene_probes.jsonl','robot_history.jsonl','oracle_tcp.jsonl','camera_epochs.jsonl','stream_events.jsonl']}}
        (out/'config_snapshot.json').write_bytes(cfg_path.read_bytes())
        (out/'capture_snapshot.py').write_bytes(Path(__file__).read_bytes())
        (out/'method_snapshot.py').write_bytes((R3/'history_contract_r1.py').read_bytes())
        manifest.update(run_id=args.run_id,time_base_id=identity['time_base_id'],camera_epochs=epochs,physics_epoch_start_s=epochs[0]['birth_time_s'],robot_history_start_s=robot_rows[0]['time_s'] if robot_rows else None,
            joint_names={s:a.dof_names[:7] for s,a in arms.items()},tcp_link='fr3_link8 @ fixed tool_local',
            tool_local={s:matrix.tolist() for s,matrix in tool_local.items()},robot_samples=len(robot_rows),
            channels={'suction':'UNAVAILABLE: no Surface_Gripper instance','tool_wall_contact':'UNAVAILABLE as observable; no contact classifier','force':'UNAVAILABLE in this probe'},
            fk_fixture_manifest_sha256=sha(model.fixture_dir/'fixture_manifest.json'),
            frozen_eval_config_sha256=sha(cfg_path),frozen_pixel_estimator_sha256=sha(R3/'r2_scene_binding.py'))
        manifest['frozen_method_sha256']=sha(R3/'history_contract_r1.py')
        manifest['r1_binding_reference_sha256']=sha(R3/'config/r1_time_binding_frozen.json')
        (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print('R3_P2B1_COMPLETE',json.dumps({'frames':len(frames),'dynamic_frames':manifest['dynamic_frames'],'verified_writes':verified_writes}),flush=True)
    except BaseException as exc:
        import traceback
        save_rows(out/'partial_observations.jsonl',frames);save_rows(out/'partial_physics_history.jsonl',physics);save_rows(out/'partial_oracle.jsonl',oracle);save_rows(out/'partial_oracle_scene_probes.jsonl',probes);save_rows(out/'partial_robot_history.jsonl',robot_rows);save_rows(out/'partial_oracle_tcp.jsonl',tcp_oracle)
        (out/'failure.json').write_text(json.dumps({'error':repr(exc),'traceback':traceback.format_exc(),'frames':len(frames)},indent=2)+'\n');print('R3_P2B1_FAILURE',repr(exc),flush=True);raise
    finally:
        for s in subscriptions:s.unsubscribe()
        app.close()


if __name__=='__main__':main()
