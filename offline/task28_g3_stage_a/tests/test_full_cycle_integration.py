import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from offline.task28_g3_stage_a import suspended_probe


def test_release_disables_internal_target_and_payload_without_changing_config():
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('release_force_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bridge = module.Task27TorqueBridge.__new__(module.Task27TorqueBridge)
    bridge._task_mode = 'DUAL_ATTACHED'
    bridge._g3_internal_force_target_N = 1.0
    bridge._cube_mass_kg = 0.8
    bridge._step = 1
    bridge.feed = SimpleNamespace(_desired={'left': False, 'right': False},
        rigids=[SimpleNamespace(get_world_pose=lambda: ([0, 0, .3], [1, 0, 0, 0]))])
    bridge._surface_grippers_closed = lambda: False
    bridge._log = lambda _: None
    bridge._update_task_mode_from_scene_event()
    assert bridge._task_mode == 'RELEASE_RETREAT'
    target, diag = bridge._g3_internal_force_terms(None, None, None, None, None, None)
    assert target is None and not diag['enabled']
    payload, diag = bridge._payload_gravity_term(None, None)
    assert not payload.any() and not diag['active']
    assert bridge._g3_internal_force_target_N == 1.0


def test_matched_comparison_rejects_failed_physical_reset(monkeypatch):
    torque = SimpleNamespace(_time_s=0., _latched_error=None,
        _trajectory_reference=None, _assumed_internal_contact_load=False,
        _surface_grippers_closed=lambda: True, _log=lambda _: None,
        set_internal_force_target=lambda _: None)
    position = np.array([0., 0., .54])
    art = SimpleNamespace(get_joint_positions=lambda: np.zeros(9),
                          get_joint_velocities=lambda: np.zeros(9))
    torque.articulations = {'left': art, 'right': art}
    torque.feed = SimpleNamespace(rigids=[SimpleNamespace(get_world_pose=lambda: (position, None))])
    app = SimpleNamespace(update=lambda: setattr(torque, '_time_s', torque._time_s + 1))
    def probe(*args, **kwargs):
        position[0] += .002
        return {}
    monkeypatch.setattr(suspended_probe, 'run_suspended_probe', probe)
    with pytest.raises(RuntimeError, match='MATCHED_RESET_MISMATCH'):
        suspended_probe.run_matched_comparison(app, torque, None)
    assert torque._trajectory_reference is None
    assert torque._assumed_internal_contact_load is False


def test_cycle_receipt_rejects_feedforward_remaining_active_after_release(tmp_path):
    import json
    from offline.task28_g3_stage_a.analyze_full_cycle import cycle
    def frame(mode, closed):
        return {'event':'TORQUE_FRAME','time_s':1.,
            'qp':{'task_mode':mode,'status':'QP_OPTIMAL',
                  'g3_qp_contact_load_model':'assumed_commanded_wrench_not_measured'},
            'reaction_diagnostic':{'suctions_closed':{'left':closed,'right':closed}},
            'tcp_pose_world':{s:np.eye(4).tolist() for s in ('left','right')},
            'cube_runtime':{'pose_world':{'position_m':[.5,0,.3]}}}
    rows=[frame('DUAL_ATTACHED',True),frame('RELEASE_RETREAT',False)]
    (tmp_path/'task27_torque_bridge.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (tmp_path/'summary.json').write_text(json.dumps({'status':'COMPLETED','executor_exit':0}))
    (tmp_path/'task27_executor.log').write_text('STAGE_A_ONLY PASS:')
    result=cycle(tmp_path)
    assert result['ff_outside_dual_attached_frames'] == 1
    assert not result['physical_cycle_completed']
