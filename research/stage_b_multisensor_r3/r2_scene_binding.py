"""R2 pixel-only Cube pose and explicit scene-time hypotheses; no oracle input."""
import time
import cv2
import numpy as np
from scipy.optimize import least_squares
from research.stage_b_multisensor_r3.planar_pose_estimator import estimate_v1,camera_axes_from_usd,rotation_z
from research.stage_b_vision_contact_r2b.visual.aruco_pose import detect_aruco


def fit_box_points(points,seed_xyyaw,cube_size_m=.12,inlier_m=.001):
    """Fit known vertical box faces to depth points, keeping table support fixed."""
    points=np.asarray(points,dtype=float);seed=np.asarray(seed_xyyaw,dtype=float)
    if len(points)<40 or not np.isfinite(points).all():return {'status':'UNCERTAIN','reason':'insufficient_depth'}
    def residual(p,xyz):
        local=(xyz[:,:2]-p[:2])@rotation_z(p[2])[:2,:2]
        return np.max(abs(local)-cube_size_m/2,axis=1)
    bounds=(seed-np.array([.025,.025,.25]),seed+np.array([.025,.025,.25]))
    fit=least_squares(residual,seed,args=(points,),bounds=bounds,loss='huber',f_scale=.0003,max_nfev=80)
    keep=abs(residual(fit.x,points))<inlier_m
    if keep.sum()<40:return {'status':'UNCERTAIN','reason':'depth_model_mismatch'}
    fit=least_squares(residual,fit.x,args=(points[keep],),bounds=bounds,loss='huber',f_scale=.0003,max_nfev=80)
    singular=np.linalg.svd(fit.jac,compute_uv=False);errors=abs(residual(fit.x,points[keep]))
    valid=fit.success and singular[-1]>singular[0]*1e-4 and np.percentile(errors,95)<inlier_m
    return {'status':'PRESENT' if valid else 'UNCERTAIN','xyyaw':fit.x.tolist() if valid else None,
        'inlier_points':int(keep.sum()),'input_points':len(points),'surface_p95_m':float(np.percentile(errors,95)),
        'jacobian_condition':float(singular[0]/max(singular[-1],1e-15))}


def estimate_cube_pixels(image,depth,K,M,roi,p1):
    """RGB marker initializes an independent RGB-D box fit; all inputs are pixels/config."""
    start=time.perf_counter();x0,y0,_,_=roi;cropped_K=np.array(K,dtype=float).copy();cropped_K[:2,2]-=[x0,y0]
    detection=detect_aruco(image,cropped_K,np.zeros(5),p1['marker_size_m'],23)
    if detection.status!='PRESENT':return {'status':'UNKNOWN','reason':'marker_not_detected','corners_px':None}
    corners=np.array(detection.corners_px)+[x0,y0]
    kwargs={k:p1[k] for k in ('marker_size_m','marker_offset_m','table_height_m','cube_size_m','support_tolerance_m','tilt_limit_deg','residual_limit_px')}
    rgb_pose=estimate_v1(corners,K,M,**kwargs).to_dict()
    if rgb_pose['status']!='PRESENT':return {'status':'UNCERTAIN','reason':'marker_support_rejected','rgb_pose':rgb_pose,'corners_px':corners.tolist()}
    # Principal-column timing cells independently verify RTX samples at index + 0.5.
    v,u=np.mgrid[0:depth.shape[0]:2,0:depth.shape[1]:2];z=depth[::2,::2].ravel();uv=np.column_stack((u.ravel()+x0+.5,v.ravel()+y0+.5,np.ones(z.size)))
    keep=np.isfinite(z)&(z>.1)&(z<5);xyz=M[3,:3]+((uv[keep]@np.linalg.inv(K).T)*z[keep,None])@camera_axes_from_usd(M).T
    seed=np.array([*rgb_pose['cube_xyz_m'][:2],rgb_pose['yaw_rad']]);height=p1['table_height_m'];size=p1['cube_size_m']
    keep=(xyz[:,2]>height+.012)&(xyz[:,2]<height+size-.012)&(np.max(abs(xyz[:,:2]-seed[:2]),axis=1)<size*.85)
    xyz=xyz[keep];xyz=xyz[::max(1,int(np.ceil(len(xyz)/1600)))]
    fit=fit_box_points(xyz,seed,cube_size_m=size)
    return dict(fit,rgb_pose=rgb_pose,corners_px=corners.tolist(),cube_xyz_m=[*fit['xyyaw'][:2],height+size/2] if fit['status']=='PRESENT' else None,
        runtime_ms=(time.perf_counter()-start)*1000,source='RGB marker + depth vertical-face box fit; fixed table geometry')


def scene_time_interface(frame,clock,physics_times,binding):
    """Never turn an unresolved set of scene epochs into a midpoint timestamp."""
    output={'frame_id':frame['frame_id'],'measurement_time_s':None,'corresponding_physics_step':None,
        'timestamp_source':'INDEPENDENT_RGB_DEPTH_CLOCK_AND_FROZEN_SCENE_TRANSFER',
        'timing_status':'UNCERTAIN','candidates':[],'online_fusion_eligible':False}
    if binding.get('status')!='FROZEN_CALIBRATION' or binding.get('configuration_fingerprint')!=frame.get('configuration_fingerprint'):return output
    if frame.get('cube_matches_known_stimulus') is not True:return output
    if not all(clock[s]['paired'] for s in ('app','pre','post')):return output
    step=clock['post']['rgb_code']
    if step!=clock['pre']['rgb_code'] or step not in physics_times:return output
    for shift in binding['cube_minus_clock_candidate_ticks']:
        candidate=step+shift
        if candidate in physics_times:output['candidates'].append({'physics_step':candidate,'time_s':physics_times[candidate],'clock_relative_ticks':shift})
    if not output['candidates']:return output
    output['timing_status']='MULTI_HYPOTHESIS_SCENE_EPOCH'
    if len(output['candidates'])==1 and binding.get('precise_transfer_validated'):
        c=output['candidates'][0];output.update(measurement_time_s=c['time_s'],corresponding_physics_step=c['physics_step'],timing_status='INSTRUMENTED_SCENE_EPOCH')
    return output
