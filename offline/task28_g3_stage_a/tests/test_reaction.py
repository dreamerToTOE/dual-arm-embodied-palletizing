import numpy as np
import pytest
from offline.task28_g3_stage_a.reaction import checked_reactions


def test_reaction_base_row_and_fixed_joint_index_are_not_dof_indices():
    names = ['base', 'arm', 'fixed_tool', 'finger']
    values = np.arange(24).reshape(4, 6)
    out, rows = checked_reactions(values, names, {'arm_joint': 0, 'tool_fixed': 1, 'finger': 2})
    assert rows['tool_fixed'] == 2
    assert np.array_equal(out[rows['arm_joint']], values[1])
    for bad in (values[:3], np.full((4, 6), np.nan)):
        with pytest.raises(ValueError):
            checked_reactions(bad, names, {'arm_joint': 0})
    with pytest.raises(ValueError):
        checked_reactions(values, names, {'wrong_joint': 3})


def test_world_transmission_uses_child_joint_rotation_and_anchor():
    from scipy.spatial.transform import Rotation
    from offline.task28_g3_stage_a.analyze_reaction import joint_world_wrench
    local = Rotation.from_euler('z', 90, degrees=True).as_quat()
    meta = {'joint_rows': {'fixed': 1}, 'body_names_by_row': ['base', 'tool'],
            'usd_joints': {'fixed': {'body1': ['/World/tool'], 'local_pos1_m': [0, 2, 0],
                                     'local_rot1_wxyz': local[[3,0,1,2]].tolist()}}}
    sample = {'get_link_transforms': [[[0,0,0,0,0,0,1],[1,0,0,0,0,0,1]]],
              'incoming_child_joint_N_Nm': [[0]*6,[3,0,0,0,4,0]]}
    wrench, anchor = joint_world_wrench(meta, sample, 'fixed')
    assert np.allclose(wrench, [0,3,0,-4,0,0])
    assert np.allclose(anchor, [1,2,0])
