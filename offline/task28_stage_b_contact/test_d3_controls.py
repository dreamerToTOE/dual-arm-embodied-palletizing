import copy

import pytest

from isaac.scripts.task28_d2_view_compare import validate_material_bindings
from offline.task28_stage_b_contact.run_d3 import matched


def fixture():
    common = {'runtime_mass_kg': .8, 'gravity_world_m_s2': [0, 0, -9.81], 'warmup_app_updates': 90,
              'warmup_physics_steps': 53, 'solver': 'TGS', 'stabilization': True, 'view_order': ['table'],
              'controls': {'table': {'friction': .9}}, 'cube_contact_report': True, 'table_contact_report': True,
              'ros_graph_deactivated': True, 'initial_cube_xyz_m': [.5, 0, .26],
              'initial_cube_velocity_m_s': [0, 0, 0], 'initial_cube_quaternion_xyzw': [0, 0, 0, 1]}
    c, d = copy.deepcopy(common), copy.deepcopy(common)
    c['tool_shapes'] = []
    d['tool_shapes'] = list(range(7))
    return c, d, [{'dt_s': 1 / 60}]


def test_matched_control_and_mismatch_rejection():
    c, d, rows = fixture()
    assert matched(c, d, rows, rows) == []
    d['controls']['table']['friction'] = .75
    d['initial_cube_velocity_m_s'][0] = .1
    assert set(matched(c, d, rows, [{'dt_s': .02}])) == {'controls', 'initial_cube_velocity_m_s', 'physics_dt'}


def test_missing_or_extra_tool_shapes_rejected():
    c, d, rows = fixture()
    d['tool_shapes'].pop()
    assert matched(c, d, rows, rows) == ['tool_shape_contract']


def test_missing_material_target_is_rejected():
    class Prim:
        def GetRelationship(self, _): return self
        def GetTargets(self): return ['/material']
        def GetPath(self): return '/cube'
        def HasAPI(self, _): return False
    class Stage:
        def GetPrimAtPath(self, _): return Prim()
    with pytest.raises(RuntimeError, match='PHYSICS_MATERIAL_BINDING_INVALID'):
        validate_material_bindings(Stage(), [Prim()], object())
