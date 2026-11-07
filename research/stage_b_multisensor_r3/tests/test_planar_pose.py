import json
from pathlib import Path
import math
import numpy as np
import cv2
import pytest
from research.stage_b_multisensor_r3.planar_pose_estimator import (
    camera_axes_from_usd,cube_marker_world_corners,project,rotation_z,
    estimate_v0,estimate_v1,angle_error_rad,
)

ROOT=Path(__file__).resolve().parents[3]
M=np.array(json.loads((ROOT/'research/stage_b_vision_contact_r2b/evidence/r2b_rear_wall_physical07/r2b_camera/camera_metadata.json').read_text())['world_from_camera'])
K=np.array([[711.111111,0,640],[0,711.111111,360],[0,0,1.]])

@pytest.mark.parametrize('x,y,yaw',[(.75,-.1,-.12),(.9,.07,.15),(1.08,.04,-.03)])
def test_exact_image_corners_recover_known_planar_pose(x,y,yaw):
    uv=project(cube_marker_world_corners(x,y,yaw),K,M)
    # Match the existing R2 estimatePoseSingleMarkers baseline under identical geometry.
    rv,tv,_=cv2.aruco.estimatePoseSingleMarkers([uv.reshape(1,4,2)],.06,K,np.zeros(5))
    Rcv,_=cv2.Rodrigues(rv[0,0]);A=camera_axes_from_usd(M)
    legacy_cube=M[3,:3]+A@tv[0,0]-(A@Rcv)[:,2]*.0615
    assert np.linalg.norm(np.array(estimate_v0(uv,K,M).cube_xyz_m)-legacy_cube)<1e-6
    for estimate in (estimate_v0(uv,K,M),estimate_v1(uv,K,M)):
        assert estimate.status=='PRESENT'
        assert np.linalg.norm(np.array(estimate.cube_xyz_m)-[x,y,.26])<1e-5
        assert abs(angle_error_rad(estimate.yaw_rad,yaw))<1e-5


def test_world_camera_rotation_and_translation_do_not_change_result():
    Q=rotation_z(.7);translation=np.array([.1,-.3,0.])
    moved=M.copy();moved[:3,:3]=M[:3,:3]@Q.T;moved[3,:3]=M[3,:3]@Q.T+translation
    world=cube_marker_world_corners(.84,.06,.11)@Q.T+translation
    uv=project(world,K,moved);out=estimate_v1(uv,K,moved)
    assert out.status=='PRESENT'
    assert np.linalg.norm(np.array(out.cube_xyz_m)-([.84,.06,.26]@Q.T+translation))<1e-5
    assert abs(angle_error_rad(out.yaw_rad,.81))<1e-5
    assert np.isclose(np.linalg.det(camera_axes_from_usd(moved)),1.)


def test_camera_scale_is_rejected_and_invalid_pixels_do_not_produce_a_pose():
    uv=project(cube_marker_world_corners(.8,.06,0),K,M)
    scaled=M.copy();scaled[:3,:3]*=.12
    with pytest.raises(ValueError):estimate_v1(uv,K,scaled)
    with pytest.raises(ValueError):estimate_v1(np.zeros((4,2)),K,M)
    with pytest.raises(ValueError):estimate_v1(np.full((4,2),np.nan),K,M)


def test_lift_and_tilt_contradictions_reject_without_gt_inputs():
    lifted=cube_marker_world_corners(.9,.06,0)+[0,0,.15]
    assert estimate_v1(project(lifted,K,M),K,M).status=='UNCERTAIN'
    a=math.radians(20);R=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
    world=cube_marker_world_corners(.9,.06,0)
    origin=np.array([.9,.06,.3215]);tilted=(world-origin)@R.T+origin
    assert estimate_v1(project(tilted,K,M),K,M).status=='UNCERTAIN'
    # Executor carry receipt vetoes even otherwise plane-compatible pixels.
    assert estimate_v1(project(world,K,M),K,M,support_context=False).status=='UNCERTAIN'


def test_one_pixel_rounding_improves_xy_without_altering_images_or_k():
    uv=np.round(project(cube_marker_world_corners(.86,.06,.04),K,M))
    v0,v1=estimate_v0(uv,K,M),estimate_v1(uv,K,M)
    e0=np.linalg.norm(np.array(v0.cube_xyz_m)[:2]-[.86,.06]);e1=np.linalg.norm(np.array(v1.cube_xyz_m)[:2]-[.86,.06])
    assert e1<e0
    assert v1.uncertainty_xy_m>0


def test_estimator_signature_cannot_take_cube_oracle():
    uv=project(cube_marker_world_corners(.8,.06,0),K,M)
    with pytest.raises(TypeError):estimate_v1(uv,K,M,cube_world_m=[.8,.06,.26])
