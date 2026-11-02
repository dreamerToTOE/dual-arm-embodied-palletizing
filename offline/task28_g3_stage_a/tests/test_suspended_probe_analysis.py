import json

import pytest

from offline.task28_g3_stage_a.analyze_suspended_probe import analyze, paired_commands


def frame(step, x):
    return {'event': 'TORQUE_FRAME', 'physics_step': step,
            'cube_runtime': {'pose_world': {'position_m': [x, 0, 1]}},
            'tcp_pose_world': {'left': [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
                               'right': [[1,0,0,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]]},
            'qp': {'status': 'QP_OPTIMAL', 'g3_internal_force': {'estimated_internal_force_N': 0.2}},
            'reaction_diagnostic': {'suctions_closed': {'left': True, 'right': True}}}


def command(step, elapsed, target=None):
    return {'event': 'SUSPENDED_PROBE_COMMAND', 'physics_step': step, 'elapsed_s': elapsed,
            'force_world_N': [0, 0.1, 0], 'cube_target_world_m': target}


def test_pairing_uses_next_completed_step_and_discards_gaps():
    c1, c2, c3 = command(1, 1), command(3, 2), command(4, 3)
    assert list(paired_commands([frame(1, 0), c1, frame(2, 1), c2, frame(8, 0), c3, frame(5, 2)])) == [(c1, frame(2, 1)), (c3, frame(5, 2))]


def test_incomplete_trace_retains_measured_results_without_return_claim(tmp_path):
    f1, f2 = frame(101, 0.0011), frame(102, 0.0022)
    f1['time_s'], f2['time_s'] = 21, 22
    rows = [{'event': 'SUSPENDED_PROBE_START', 'time_s': 0},
            {'event': 'SUSPENDED_PROBE_MOTION_START', 'time_s': 20, 'amplitude_world_x_m': 0.005, 'cube_start_world_m': [0, 0, 1]},
            command(100, 21, [0.001, 0, 1]), f1,
            command(101, 22, [0.002, 0, 1]), f2,
            command(102, 23, [0.003, 0, 1])]
    (tmp_path / 'task27_torque_bridge.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    result = analyze(tmp_path)
    assert result['paired_frames'] == 2
    assert result['unmatched_commands'] == 1
    assert not result['complete_by_trace']
    assert result['motion']['return_error_mm'] is None
    assert result['motion']['target_error_peak_mm'] == pytest.approx(0.8104)
    assert result['gripper_closed_samples'] == 2


def test_frame_time_ideal_and_fk_targets_are_independent(tmp_path, monkeypatch):
    import numpy as np
    from offline.task28_g1_1.task28_validator import model as model_module

    class Model:
        def fk(self, side, q):
            T = np.eye(4)
            T[0, 3] = q[0]
            return T

    monkeypatch.setattr(model_module, 'RobotModel', Model)
    f = frame(101, 0.0011)
    f['time_s'] = 122.5
    f['q_ref_rad'] = {'left': [0.001] + [0] * 6, 'right': [0.001] + [0] * 6}
    f['qp']['trajectory_adapter'] = {'tcp_targets_active': True, 'active_arms': ['left', 'right'],
                                      'reference_age': {side: {'source': 'suspended_probe_quintic_joint_ik'} for side in ('left', 'right')}}
    rows = [{'event': 'SUSPENDED_PROBE_START', 'time_s': 100},
            {'event': 'SUSPENDED_PROBE_MOTION_START', 'time_s': 120, 'amplitude_world_x_m': 0.005,
             'cube_start_world_m': [0, 0, 1],
             'ik': {side: {'q_start': [0] * 7} for side in ('left', 'right')}},
            command(100, 22, [0.0049, 0, 1]), f]
    (tmp_path / 'task27_torque_bridge.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    motion = analyze(tmp_path)['motion']
    assert motion['target_error_peak_mm'] == pytest.approx(1.4)
    assert motion['fk_qref_reference']['tracking_error_peak_mm'] == pytest.approx(0.1)
    assert motion['fk_qref_reference']['fk_vs_ideal_peak_mm'] == pytest.approx(1.5)
    assert motion['fk_qref_reference']['bilateral_tcp_targets_active_samples'] == 1


def test_restored_adapter_is_excluded_from_fk_comparison(tmp_path, monkeypatch):
    import numpy as np
    from offline.task28_g1_1.task28_validator import model as model_module

    class Model:
        def fk(self, side, q):
            T = np.eye(4)
            T[0, 3] = q[0]
            return T

    monkeypatch.setattr(model_module, 'RobotModel', Model)
    f = frame(101, 0.0)
    f['time_s'] = 134
    f['q_ref_rad'] = {'left': [0.1] + [0] * 6, 'right': [0.1] + [0] * 6}
    f['qp']['trajectory_adapter'] = {'reference_age': {side: {'source': 'timestamped_task27_adapter'} for side in ('left', 'right')}}
    rows = [{'event': 'SUSPENDED_PROBE_START', 'time_s': 100},
            {'event': 'SUSPENDED_PROBE_MOTION_START', 'time_s': 120, 'amplitude_world_x_m': 0.005,
             'cube_start_world_m': [0, 0, 1],
             'ik': {side: {'q_start': [0] * 7} for side in ('left', 'right')}},
            command(100, 33.98, [0, 0, 1]), f]
    (tmp_path / 'task27_torque_bridge.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    motion = analyze(tmp_path)['motion']
    assert motion['fk_qref_reference']['samples'] == 0
    assert motion['fk_qref_reference']['excluded_samples'] == 1
    assert motion['fk_qref_reference']['fk_vs_ideal_peak_mm'] is None
