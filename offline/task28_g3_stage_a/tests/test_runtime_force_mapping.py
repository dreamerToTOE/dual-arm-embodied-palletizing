import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np


def controller_terms(h_bias=None, observer_bias=None):
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('g3_force_mapping_test', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    bridge = module.Task27TorqueBridge.__new__(module.Task27TorqueBridge)
    bridge._task_mode='DUAL_ATTACHED'; bridge._g3_internal_force_target_N=2.0
    bridge._g3_internal_force_kp=0.0; bridge._step=1; bridge._last_full_command={s:np.zeros(9) for s in ('left','right')}
    cube=SimpleNamespace(get_world_pose=lambda:(np.zeros(3),np.array([1.,0,0,0])))
    bridge.feed=SimpleNamespace(rigids=[cube])
    transforms={'left':np.eye(4),'right':np.eye(4)}
    transforms['left'][1,3]=-.061; transforms['right'][1,3]=.061
    J=np.c_[np.eye(6),np.zeros(6)]
    target,diag=bridge._g3_internal_force_terms(J,J,transforms,np.eye(14),np.zeros(14) if h_bias is None else h_bias,np.zeros(14),observer_bias14=observer_bias)
    return J,target,diag


def test_actuator_target_compensates_opposite_cube_contact_reaction():
    J,target,diag=controller_terms()
    f=np.asarray(diag['contact_wrench_on_cube_world_N_Nm'])
    # Static: 0 = tau_act + tau_external_on_robot = tau_act - J.T*f_cube.
    assert np.allclose(target, np.r_[J.T@f[:6],J.T@f[6:]])


def test_force_target_uses_same_per_contact_units_as_half_difference_estimate():
    _,_,diag=controller_terms()
    f=np.asarray(diag['contact_wrench_on_cube_world_N_Nm'])
    axis=np.asarray(diag['grasp_axis_world'])
    assert np.isclose(.5*(axis@f[:3]-axis@f[6:9]),2.0)


def test_observer_excludes_payload_control_compensation_from_arm_model_bias():
    augmented=np.zeros(14);augmented[1]=3;augmented[8]=-3
    _,_,diag=controller_terms(h_bias=augmented,observer_bias=np.zeros(14))
    assert abs(diag['estimated_internal_force_N']) < 1e-12
