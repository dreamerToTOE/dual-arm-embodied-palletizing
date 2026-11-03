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
    bridge._b0_dual_seen = True
    bridge._g3_internal_force_target_N = 1.0
    bridge._cube_mass_kg = 0.8
    bridge._step = 1
    bridge._object_reference = object()
    bridge._object_reference_rates = object()
    bridge._object_reference_diag = {'active': True}
    bridge._attached_hold_active = True
    bridge._attached_hold_q14 = np.ones(14)
    bridge._attached_hold_transforms = object()
    bridge._attached_transition_pending = True
    bridge._placement_integral_accel = np.ones(3)
    bridge.feed = SimpleNamespace(_desired={'left': False, 'right': False},
        rigids=[SimpleNamespace(get_world_pose=lambda: ([0, 0, .3], [1, 0, 0, 0]))])
    bridge._surface_grippers_closed = lambda: False
    bridge._log = lambda _: None
    bridge._update_task_mode_from_scene_event()
    assert bridge._task_mode == 'RELEASE_RETREAT'
    assert bridge._object_reference is None and bridge._object_reference_rates is None
    assert not bridge._object_reference_diag['active'] and not bridge._attached_hold_active
    assert not bridge._placement_integral_accel.any()
    target, diag = bridge._g3_internal_force_terms(None, None, None, None, None, None)
    assert target is None and not diag['enabled']
    payload, diag = bridge._payload_gravity_term(None, None)
    assert not payload.any() and not diag['active']
    assert bridge._g3_internal_force_target_N == 1.0
    bridge.feed._desired['left'] = True
    bridge._update_task_mode_from_scene_event()
    assert bridge._task_mode == 'SINGLE_ARM_PUSH'
    assert bridge._object_reference is None
    target, diag = bridge._g3_internal_force_terms(None, None, None, None, None, None)
    assert target is None and not diag['enabled']


def test_b0_runner_rejects_more_than_one_batch():
    import subprocess
    import sys
    runner = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_one_pass.py'
    result = subprocess.run([sys.executable, str(runner), '--mode', 'integrated', '--live-qp',
        '--stage-b0-only', '--max-batches', '2', '--consistent-object-reference',
        '--assume-internal-contact-load'], capture_output=True, text=True)
    assert result.returncode != 0
    assert 'stage-b0-only requires one integrated' in result.stderr


def test_early_single_suction_release_does_not_enable_b0():
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('early_release_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    bridge = module.Task27TorqueBridge.__new__(module.Task27TorqueBridge)
    bridge._task_mode = 'SINGLE_ARM_PUSH'
    bridge._b0_dual_seen = False
    bridge._b0_release_observed = False
    bridge._step = 1
    bridge.feed = SimpleNamespace(_desired={'left': False, 'right': False})
    bridge._surface_grippers_closed = lambda: False
    bridge._log = lambda _: None
    bridge._update_task_mode_from_scene_event()
    assert bridge._task_mode == 'RELEASE_RETREAT'
    assert not bridge._b0_release_observed


def test_b0_receipt_distinguishes_single_attachment_from_objective():
    from offline.task28_task27_integration.analyze_b0 import frame_receipt
    frame = {'time_s': 2., 'cube_runtime': {'pose_world': {'position_m': [.8, .06, .26],
        'orientation_xyzw': [1, 0, 0, 0]}, 'linear_velocity_m_s': [0, 0, 0]},
        'qp': {'trajectory_adapter': {'task_mode': 'SINGLE_ARM_PUSH'}, 'relative_objective_enabled': False,
               'object_reference': {'active': False}},
        'reaction_diagnostic': {'suctions_closed': {'left': True, 'right': False}},
        'tcp_world_m': {}, 'q_error_rad': {}, 'canonical_command_effort_Nm': {}}
    result = frame_receipt(frame)
    assert result['grippers_closed'] == {'left': True, 'right': False}
    assert result['mode'] == 'SINGLE_ARM_PUSH'
    assert not result['relative_objective_enabled'] and not result['object_reference_active']


def test_cube_publication_audit_preserves_exact_message_and_compares_sources():
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_torque_bridge.py'
    spec = importlib.util.spec_from_file_location('cube_publication_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    pose = SimpleNamespace(position=SimpleNamespace(x=1.096, y=.06, z=.26),
                           orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
    message = SimpleNamespace(poses=[pose], header=SimpleNamespace(frame_id='world',
                              stamp=SimpleNamespace(sec=2, nanosec=42)))
    cube = SimpleNamespace(get_world_pose=lambda: ([1.097, .06, .26], [1., 0., 0., 0.]),
                           get_linear_velocity=lambda: [0., 0., 0.])
    records, published = [], []
    bridge = SimpleNamespace(feed=SimpleNamespace(rigids=[cube]), _step=7, _time_s=1.5, _log=records.append)
    module.CubePoseAuditPublisher(SimpleNamespace(publish=published.append), bridge).publish(message)
    assert published == [message]
    assert records[0]['header_ns'] == 2000000042
    assert np.isclose(records[0]['rigid_prim_xyz_m'][0] - records[0]['ros_usd_xyz_m'][0], .001)


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
