import numpy as np
from offline.task28_task27_live_qp.object_reference import RigidObjectReference


def test_common_rotation_feedback_uses_the_world_jacobian_frame():
    import importlib.util
    from pathlib import Path
    from scipy.spatial.transform import Rotation
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('rotation_test', path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    target = np.eye(4); target[:3, :3] = Rotation.from_euler('x', np.pi).as_matrix()
    actual = target.copy()
    actual[:3, :3] = Rotation.from_euler('z', .1).as_matrix() @ target[:3, :3]
    correction = mod.Task27TorqueBridge._rotation_error(actual, target)
    assert np.allclose(correction, [0., 0., -.1], atol=1e-9)


def test_bottom_gap_accounts_for_scaled_cube_and_measured_tilt():
    import importlib.util
    from pathlib import Path
    from scipy.spatial.transform import Rotation
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('gap_test', path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    bridge = mod.Task27TorqueBridge.__new__(mod.Task27TorqueBridge)
    bridge._support_cube_half_extent = np.array([.06, .06, .06])
    bridge._support_table_top_z = .2
    runtime = {'pose_world': {'position_m': [.77, .06, .273], 'orientation_xyzw': [1., 0., 0., 0.]}}
    assert np.isclose(bridge._cube_bottom_gap(runtime), .013)
    quat = Rotation.from_euler('y', .02).as_quat()
    runtime['pose_world']['orientation_xyzw'] = quat[[3, 0, 1, 2]]
    assert np.isclose(bridge._cube_bottom_gap(runtime), .073 - .06 * (np.cos(.02) + np.sin(.02)))


def test_inconsistent_sources_produce_one_fixed_grasp_without_following_feedback():
    obj=np.eye(4); obj[2,3]=.26
    a={s:obj.copy() for s in ('left','right')};a['left'][1,3]=-.061;a['right'][1,3]=.061
    ref=RigidObjectReference(obj,a)
    q={s:np.zeros(7) for s in a}
    ref.sample(a,q,.1)
    b={s:t.copy() for s,t in a.items()};b['left'][0,3]=.12;b['right'][0,3]=.04
    pose,tcps,v,acc,_,_,_=ref.sample(b,q,.1)
    assert np.isclose(pose[0,3],.08)
    assert np.allclose(np.linalg.inv(tcps['left'])@tcps['right'],np.linalg.inv(a['left'])@a['right'])
    assert np.isclose(v['left'][0],.8) and np.isclose(acc['right'][0],8.)
    # No moving object/actual TCP input exists after the attachment capture.
    assert np.isclose(ref.sample(b,q,.1)[0][0,3],.08)


def test_joint_posture_ik_tracks_the_same_frozen_object_target():
    from offline.task28_g1_1.task28_validator.model import RobotModel
    model=RobotModel()
    q={s:model.home.copy() for s in ('left','right')}
    tcp={s:model.fk(s,q[s]) for s in q}
    obj=np.eye(4);obj[:3,3]=.5*(tcp['left'][:3,3]+tcp['right'][:3,3])
    frozen={s:t.copy() for s,t in tcp.items()}
    frozen['right'][0,3] += .00005
    ref=RigidObjectReference(obj,frozen)
    _,targets,_,_,_,_,projected=ref.sample(tcp,q,1/60,model=model)
    for side in q:
        assert np.linalg.norm(model.fk(side,projected[side])[:3,3]-targets[side][:3,3]) < 1e-6


def test_placement_integral_is_bounded_and_does_not_move_the_reference():
    import importlib.util
    from pathlib import Path
    path=Path(__file__).resolve().parents[3]/'isaac/scripts/task28_task27_torque_bridge.py'
    spec=importlib.util.spec_from_file_location('placement_test',path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    bridge=mod.Task27TorqueBridge.__new__(mod.Task27TorqueBridge)
    bridge._task_mode='DUAL_ATTACHED';bridge._placement_integral_accel=np.zeros(3)
    target=np.eye(4);target[:3,3]=[.77,.06,.26]
    actual=target.copy();actual[2,3]+=.013
    for _ in range(1200):a,active=bridge._placement_correction(target,actual,np.zeros(6),1/60)
    assert active and -.25 <= a[2] < 0 and np.max(np.abs(a))<=.25
    assert target[2,3]==.26
    bridge._task_mode='RELEASE_RETREAT'
    assert not bridge._placement_correction(target,actual,np.zeros(6),1/60)[0].any()
