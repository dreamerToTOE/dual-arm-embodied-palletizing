from pathlib import Path

import pytest

from offline.task28_stage_b_contact.d4_scene import diagnostic_scene_code
from offline.task28_stage_b_contact.run_d4 import validate
from isaac.scripts.task28_d4_tool_isolation import mass_record


def test_transform_removes_graph_and_restores_material_after_objects():
    source = '''\nclass Manager:\n def set_extension_enabled_immediate(self, x, y): raise AssertionError()\nmanager = Manager()\ntrace = []\ndef _make_material(): trace.append("material"); return 1\ndef _build_objects(): trace.append("objects")\ndef _build_ros_graph(): raise AssertionError()\nmanager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)\n_build_objects()\n_build_ros_graph()\n'''
    namespace = {}
    exec(diagnostic_scene_code(source, 'scene.py'), namespace)
    assert namespace['trace'] == ['objects', 'material']
    assert namespace['_MATERIAL'] == 1


def test_production_scene_markers_and_missing_marker_fail_closed():
    source = (Path(__file__).resolve().parents[2] / 'isaac/scripts/task26_truck_box_scene.py').read_text()
    diagnostic_scene_code(source, 'task26.py')
    with pytest.raises(ValueError, match='UNRECOGNIZED_SCENE_CONSTRUCTION'):
        diagnostic_scene_code(source.replace('\n_build_ros_graph()\n', '\n'), 'bad.py')


def test_preflight_checks_match_and_separation():
    common = dict(authored={'cube': 'original'}, runtime_mass_kg=.8, gravity_world_m_s2=[0, 0, -9.81],
                  warmup_physics_steps=52, solver='TGS', stabilization=True, view_order=['table'],
                  material_paths={'cube': ['material']}, effective_material=[[[.9, .75, 0]]],
                  cube_mass_properties={'mass': .8}, ros_graph_absent=True)
    a = dict(common, tool_shapes=[], tool_separation_m={})
    b = dict(common, tool_shapes=list(range(7)), tool_separation_m={str(i): .003 for i in range(7)})
    assert validate(a, b) == []
    b['tool_separation_m']['0'] = 0
    b['runtime_mass_kg'] = 1.
    assert set(validate(a, b)) == {'runtime_mass_kg', 'tool_geometry_or_separation'}


def test_physx_vector_mass_record_is_json_serializable():
    import json
    class Vec:
        x, y, z = .1, .2, .3
    result = mass_record({'mass': .8, 'inertia': Vec(), 'com': Vec()})
    assert json.loads(json.dumps(result))['inertia'] == [.1, .2, .3]
