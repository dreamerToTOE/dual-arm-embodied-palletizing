from offline.task28_g3_stage_a.known_load import CASES, CASE_DURATION_S, calibration_case


def test_calibration_schedule_is_bounded_and_returns_to_zero_for_each_arm():
    assert len(CASES) == 14
    assert calibration_case(0) == (0, 'left', 0.0)
    assert calibration_case(3) == (1, 'left', 0.1)
    assert calibration_case(6 * 3) == (6, 'left', 0.0)
    assert calibration_case(7 * 3) == (7, 'right', 0.0)
    assert calibration_case(len(CASES) * CASE_DURATION_S) is None
    assert all(abs(force) <= 0.2 for _, force in CASES)


def test_analysis_pairs_injection_to_next_physics_step(tmp_path):
    import json
    import numpy as np
    import pytest
    from offline.task28_g3_stage_a.analyze_known_load import analyze

    names = [f'link{i}' for i in range(8)] + ['fr3_link8']
    meta = {'body_names_by_row': names, 'joint_rows': {'fr3_joint8': 8},
            'usd_joints': {'fr3_joint8': {'body1': ['/World/left_fr3/fr3_link8'],
                                          'local_rot1_wxyz': [1, 0, 0, 0], 'local_pos1_m': [0, 0, 0]}}}
    poses = np.tile([0, 0, 0, 0, 0, 0, 1], (1, 9, 1)).tolist()
    incoming = np.zeros((9, 6)); incoming[8, 1] = -0.1
    jacobian = np.c_[np.eye(6), np.zeros(6)].tolist()
    cmd = {'event': 'KNOWN_LOAD_COMMAND', 'physics_step': 5, 'case_index': 1,
           'body_path': '/World/left_fr3/fr3_hand_tcp', 'force_world_N': [0, 0.1, 0],
           'application_point_world_m': [0, 0, 0]}
    qdd = np.zeros(14); qdd[1] = 0.1
    frame = {'event': 'TORQUE_FRAME', 'physics_step': 6, 'time_s': 1.0,
             'reaction_diagnostic': {'sides': {'left': {'valid': True, 'get_link_transforms': poses,
                                                        'incoming_child_joint_N_Nm': incoming.tolist()}},
                                     'applied_effort_completed_interval_Nm': {'left': [0]*7},
                                     'jacobian_world_at_tcp': {'left': jacobian}},
             'qp': {'status': 'QP_OPTIMAL', 'full_mass_matrix_used': {'left': np.eye(7).tolist()},
                    'full_coriolis_Nm': {'left': [0]*7}, 'full_gravity_Nm': {'left': [0]*7},
                    'tau_box_violation': 0, 'torque_saturated_indices': [], 'torque_rate_active_indices': []},
             'tcp_pose_world': {'left': np.eye(4).tolist()}, 'q_rad': {'left': [0]*7},
             'dq_rad_s': {'left': [0]*7}, 'measured_qdd14_rad_s2': qdd.tolist()}
    path = tmp_path / 'task27_torque_bridge.jsonl'
    def write(items):
        path.write_text(''.join(json.dumps(x) + '\n' for x in items))
    write([{'event': 'REACTION_SOURCE_METADATA', 'sides': {'left': meta}}, cmd, frame])
    case = analyze(tmp_path)['cases'][0]
    assert case['model_wrench_mean'][1] == pytest.approx(0.1)
    assert case['joint8_wrench_mean'][1] == pytest.approx(-0.1)
    frame['physics_step'] = 7
    write([{'event': 'REACTION_SOURCE_METADATA', 'sides': {'left': meta}}, cmd, frame])
    with pytest.raises(ValueError, match='STEP_MISMATCH'):
        analyze(tmp_path)
