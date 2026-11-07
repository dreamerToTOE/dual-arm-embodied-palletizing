#!/usr/bin/env python3
"""Score capture identity/timing and unchanged P1 estimates, oracle offline only."""
import argparse,csv,hashlib,json,sys
from pathlib import Path
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from research.stage_b_multisensor_r3.time_alignment import RobotHistory
from research.stage_b_multisensor_r3.planar_pose_estimator import estimate_v1,cube_marker_world_corners,project,camera_axes_from_usd
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco
from offline.task28_g1_1.task28_validator.model import RobotModel


def rows(p):return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(values):
    return dict(zip(('median','p95','max'),map(float,np.percentile(values,[50,95,100])))) if len(values) else None


def evaluate(run,out):
    m=json.loads((run/'manifest.json').read_text());obs=rows(run/'observations.jsonl');robot=rows(run/'robot_history.jsonl')
    assert all(sha(run/p)==digest for p,digest in m['data_sha256'].items() if p!='oracle.jsonl')
    cfg=json.loads((ROOT/'research/stage_b_multisensor_r3/config/p1_frozen.json').read_text());K=np.array(m['K']);M=np.array(m['world_from_camera']);predictions=[];plane=[];pair_checks=[];interface=[]
    for f in obs:
        assert not {'cube_world_m','cube_xyz_m','tcp_usd','tcp_physx','contacts'}.intersection(f)
        assert sha(run/f['rgb'])==f['rgb_sha256'] and sha(run/f['depth'])==f['depth_sha256']
        image=cv2.imread(str(run/f['rgb']));depth=np.load(run/f['depth'],allow_pickle=False)['distance_to_image_plane'];d=detect_aruco(image,K,np.zeros(5),.06,23)
        pred=estimate_v1(d.corners_px,K,M,**{k:cfg[k] for k in ('marker_size_m','marker_offset_m','table_height_m','cube_size_m','support_tolerance_m','tilt_limit_deg','residual_limit_px')}).to_dict() if d.status=='PRESENT' else {'status':'UNKNOWN'}
        interface.append({'schema':'r3.p2b.visual_robot_interface.v1','frame_id':f['frame_id'],'pair_id':f['pair_id'],'reported_render_time_s':f['render_time_s'],'measurement_time_s':None,'timing_status':'UNCERTAIN_SDK_TIME_NOT_INDEPENDENTLY_VALIDATED','measurement_time_interval_s':None,'arrival_physics_time_s':f['receipt_physics_time_s'],'arrival_wall_s':f['arrival_wall_s'],'writer_complete_wall_s':f['writer_complete_wall_s'],'pairing_status':'COMMON_RENDER_PRODUCT_CALLBACK','robot_at_reported_time':f['aligned_robot'],'visual_pose':pred,'table_support':'ASSUMED_STAGE_B_EXPERIMENT','online_fusion_eligible':False,'rgb_sha256':f['rgb_sha256'],'depth_sha256':f['depth_sha256']})
        predictions.append({'frame_id':f['frame_id'],'corners_px':d.corners_px,'prediction':pred})
        meta=f['metadata'];pair_checks.append(f['frame_id']==meta['sdk_frame']==meta['reference_time']['referenceTimeNumerator'] and abs(f['render_time_s']-meta['sdk_time_s'])<1e-10 and list(image.shape[:2])==list(depth.shape)==[720,1280] and meta['rgb_depth_same_render_product']==m['render_product'])
        if d.corners_px:
            u,v=np.mean(d.corners_px,axis=0);u,v=round(u),round(v);ray=camera_axes_from_usd(M)@np.linalg.inv(K)@np.array([u,v,1.]);expected=(.3215-M[3,2])/ray[2]
            plane.append(abs(float(np.median(depth[v-1:v+2,u-1:u+2]))-expected))
    # No Cube oracle has been opened by the pixel prediction loop.
    out.mkdir(parents=True,exist_ok=True);(out/'visual_predictions.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in predictions))
    (out/'p2b_timestamped_interface.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in interface))
    assert sha(run/'oracle.jsonl')==m['data_sha256']['oracle.jsonl']
    oracle=rows(run/'oracle.jsonl');times=np.array([r['time_s'] for r in oracle]);xyz=np.array([r['cube_xyz_m'] for r in oracle]);scored=[]
    def truth(t):
        return np.array([np.interp(t,times,xyz[:,j]) for j in range(3)]) if times[0]-1e-9<=t<=times[-1]+1e-9 else None
    photo=[]
    for f,p in zip(obs,predictions):
        now=truth(f['render_time_s']);old=truth(f['legacy_receipt_time_s']);pred=p['prediction'];record={'frame_id':f['frame_id'],'render_time_s':f['render_time_s'],'legacy_receipt_time_s':f['legacy_receipt_time_s'],'arrival_physics_time_s':f['receipt_physics_time_s'],'aligned_status':f['aligned_robot']['status'],'legacy_status':f['legacy_robot']['status'],'visual_status':pred['status']}
        if old is not None and now is not None:
            record['oracle_xy_association_error_legacy_m']=float(np.linalg.norm((now-old)[:2]));record['render_to_nearest_physics_sample_s']=float(np.min(abs(times-f['render_time_s'])))
            if pred['status']=='PRESENT':
                pxy=np.array(pred['cube_xyz_m'])[:2];record['visual_xy_error_render_m']=float(np.linalg.norm(pxy-now[:2]));record['visual_xy_error_legacy_m']=float(np.linalg.norm(pxy-old[:2]))
        if p['corners_px'] is not None:
            candidates=[r for r in oracle if abs(r['time_s']-f['render_time_s'])<=.101]
            costs=[float(np.linalg.norm(project(cube_marker_world_corners(*r['cube_xyz_m'][:2],0),K,M)-np.array(p['corners_px']),axis=1).mean()) for r in candidates]
            # Exclude stops / nonunique scene poses: pixels cannot timestamp a static scene.
            best=int(np.argmin(costs));near=[j for j,c in enumerate(costs) if abs(c-costs[best])<.01]
            if near and min(near)>0 and max(near)<len(candidates)-1:
                lo,hi=candidates[min(near)]['time_s'],candidates[max(near)]['time_s']
                if hi-lo<=1/60+1e-8:
                    photo.append({'frame_id':f['frame_id'],'best_oracle_time_interval_s':[lo,hi],'sdk_time_s':f['render_time_s'],'abs_residual_s':max(lo-f['render_time_s'],f['render_time_s']-hi,0.),'interval_width_s':hi-lo,'mean_corner_cost_px':costs[best]})
        scored.append(record)
    stimulus_checks=None
    if all('cube_quaternion_wxyz' in r for r in oracle):
        angles=[float(2*np.arccos(np.clip(abs(r['cube_quaternion_wxyz'][0]),0,1))) for r in oracle]
        scale_errors=[float(np.max(abs(np.linalg.norm(np.array(r['cube_usd_matrix'])[:3,:3],axis=1)-.12))) for r in oracle]
        stimulus_checks={'kinematic_true_rows':sum(r['kinematic_enabled'] for r in oracle),'rows':len(oracle),'orientation_angle_max_rad':max(angles),'z_error_max_m':float(np.max(abs(xyz[:,2]-.26))),'usd_scale_error_max':max(scale_errors)}
    model=RobotModel();fk_errors=[];q_replay_errors=[]
    for r,o in zip(robot,oracle):
        assert r['physics_step']==o['physics_step']
        for s in ('left','right'):fk_errors.append(np.linalg.norm(np.array(r['tcp_fk'][s])[:3,3]-np.array(o['tcp_physx'][s])[:3,3]))
    midpoint_errors=[];h=RobotHistory(model.fk)
    for i,r in enumerate(robot):
        h.append(**{k:r[k] for k in ('time_s','physics_step','q','dq','suction_observation','suction_command','controller_command')})
        if i:
            a=h.at((r['time_s']+robot[i-1]['time_s'])/2)
            assert a['status']=='ALIGNED'
            for side in ('left','right'):
                expected=(np.array(oracle[i-1]['tcp_physx'][side])[:3,3]+np.array(oracle[i]['tcp_physx'][side])[:3,3])/2
                midpoint_errors.append(np.linalg.norm(np.array(a['tcp_fk'][side])[:3,3]-expected))
    by_time={r['time_s']:r for r in robot}
    for f in obs:
        a=f['aligned_robot']
        if a['status']!='ALIGNED':continue
        left,right=[by_time[t] for t in a['bracket_times_s']]
        for s in ('left','right'):
            expected=(1-a['alpha'])*np.array(left['q'][s])+a['alpha']*np.array(right['q'][s]);q_replay_errors.append(np.max(abs(expected-a['q'][s])))
            assert np.allclose(model.fk(s,a['q'][s]),a['tcp_fk'][s])
    result={'schema':'r3.p2a.alignment_evaluation.v1','run_manifest_sha256':sha(run/'manifest.json'),'frames':len(obs),
        'render_frame_strictly_monotonic':bool(np.all(np.diff([f['frame_id'] for f in obs])>0)),
        'render_time_strictly_monotonic':bool(np.all(np.diff([f['render_time_s'] for f in obs])>0)),
        'paired_common_callback_checks_passed':sum(pair_checks),'uncertain_measurement_time_samples':len(interface),'missing_render_timestamp':sum(f.get('render_time_s') is None for f in obs),
        'aligned_robot_frames':sum(f['aligned_robot']['status']=='ALIGNED' for f in obs),'legacy_history_unavailable':sum(f['legacy_robot']['status']!='ALIGNED' for f in obs),
        'off_grid_render_times_requiring_interpolation':sum(f['aligned_robot'].get('alpha',0)>0 for f in obs),
        'legacy_request_to_render_s':stats([f['render_time_s']-f['legacy_receipt_time_s'] for f in obs]),
        'render_to_arrival_sim_s':stats([f['receipt_physics_time_s']-f['render_time_s'] for f in obs]),
        'callback_to_arrival_wall_s':stats([f['arrival_wall_s']-f['callback_wall_s'] for f in obs]),
        'writer_delay_wall_s':stats([f['writer_complete_wall_s']-f['writer_start_wall_s'] for f in obs]),
        'render_to_nearest_physics_s':stats([r['render_to_nearest_physics_sample_s'] for r in scored if 'render_to_nearest_physics_sample_s' in r]),
        'physics_integrated_vs_core_time_abs_s':stats([abs(r['time_s']-r['core_sim_time_s']) for r in robot]),
        'legacy_xy_association_error_m':stats([r['oracle_xy_association_error_legacy_m'] for r in scored if 'oracle_xy_association_error_legacy_m' in r]),
        'visual_detected':sum(p['corners_px'] is not None for p in predictions),'matched_visual_samples':sum('visual_xy_error_legacy_m' in r for r in scored),
        'offline_midpoint_interpolation_checks':{'queries':len(midpoint_errors)//2,'fk_vs_interpolated_physx_tcp_m':stats(midpoint_errors),'scope':'CPU queries between real captured physics states; actual image times were on-grid'},
        'visual_accepted':sum(r['visual_status']=='PRESENT' for r in scored),
        'matched_visual_xy_error_legacy_m':stats([r['visual_xy_error_legacy_m'] for r in scored if 'visual_xy_error_legacy_m' in r]),
        'matched_visual_xy_error_render_m':stats([r['visual_xy_error_render_m'] for r in scored if 'visual_xy_error_render_m' in r]),
        'photometric_timestamp_check':{'moving_samples_with_at_most_one_tick_ambiguity':len(photo),'distance_to_best_time_interval_s':stats([r['abs_residual_s'] for r in photo]),'interval_width_s':stats([r['interval_width_s'] for r in photo]),'rows':photo,'limit':'offline GT projection diagnostic; includes corner quantization/calibration bias, excludes ambiguous stops; not an online correction'},
        'rgb_depth_marker_plane_residual_m':stats(plane),'independent_fk_vs_physx_tcp_translation_m':stats(fk_errors),
        'q_interpolation_replay_max_abs_rad':max(q_replay_errors,default=None),'physics_samples':len(robot),
        'tabletop_stimulus_oracle_checks':stimulus_checks,
        'cube_x_range_m':[float(xyz[:,0].min()),float(xyz[:,0].max())],
        'pairing_scope':'RGB/depth fetched in one SDK NEW_FRAME callback after graph evaluation, one render product and ReferenceTime. SDK exposes no separate per-channel frame IDs; geometric plane agreement is supporting evidence, not a hardware exposure certificate.',
        'baseline_scope':'Identical new RGB samples scored at actual pre-wait request receipts vs SDK render time. Historical physical07 exposure cannot be retroactively recovered.',
        'suction_scope':m['suction_semantics']}
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    with (out/'per_frame.csv').open('w') as stream:
        fields=list(dict.fromkeys(k for r in scored for k in r));writer=csv.DictWriter(stream,fieldnames=fields,lineterminator="\n");writer.writeheader();writer.writerows(scored)
    print(json.dumps({k:v for k,v in result.items() if k!='photometric_timestamp_check'},indent=2));return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--run',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();evaluate(a.run,a.out)
