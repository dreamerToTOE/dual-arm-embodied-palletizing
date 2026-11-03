import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def bridge_module():
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('b0_snapshot_bridge', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_poststep_snapshot_rejects_physics_fallback_and_missing_joint_state(monkeypatch):
    module = bridge_module()
    transform = SimpleNamespace(ExtractTranslation=lambda: [1.096, .06, .26])
    pxr = SimpleNamespace(Usd=SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda: 0)),
                          UsdGeom=SimpleNamespace(Xformable=lambda _: SimpleNamespace(
                              ComputeLocalToWorldTransform=lambda _: transform)))
    monkeypatch.setitem(sys.modules, 'pxr', pxr)
    valid = SimpleNamespace(is_physics_handle_valid=lambda: True)
    rigid = SimpleNamespace(_prim_view=valid, get_world_pose=lambda: ([1.097, .06, .26], [1, 0, 0, 0]))
    arm = SimpleNamespace(_articulation_view=valid, dof_names=module.ARM_DOF_NAMES + module.FINGER_DOF_NAMES,
                          get_joint_positions=lambda: np.zeros(9))
    stage = SimpleNamespace(GetPrimAtPath=lambda _: SimpleNamespace(IsValid=lambda: True))
    snapshot = module.make_b0_snapshot(rigid, {'left': arm, 'right': arm}, stage, 12, 0.2)
    assert snapshot['step'] == 12 and snapshot['frame'] == 'world'
    assert snapshot['cube_xyz_m'][0] == 1.097 and snapshot['cube_usd_xyz_m'][0] == 1.096
    assert len(snapshot['left_q_rad']) == len(snapshot['right_q_rad']) == 7
    rigid._prim_view = SimpleNamespace(is_physics_handle_valid=lambda: False)
    with pytest.raises(RuntimeError, match='RIGID_PHYSICS_HANDLE_INVALID'):
        module.make_b0_snapshot(rigid, {'left': arm, 'right': arm}, stage, 12, .2)
    checks = iter((True, False))
    rigid._prim_view = SimpleNamespace(is_physics_handle_valid=lambda: next(checks))
    with pytest.raises(RuntimeError, match='RIGID_PHYSICS_HANDLE_INVALID'):
        module.make_b0_snapshot(rigid, {'left': arm, 'right': arm}, stage, 12, .2)
    rigid._prim_view = valid
    arm.get_joint_positions = lambda: [float('nan')] * 9
    with pytest.raises(RuntimeError, match='ARM_POSITION'):
        module.make_b0_snapshot(rigid, {'left': arm, 'right': arm}, stage, 12, .2)
