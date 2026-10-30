import importlib.util
from pathlib import Path


def load_bridge_module():
    path = Path(__file__).resolve().parents[3] / "isaac/scripts/task28_t0_torque_bridge.py"
    spec = importlib.util.spec_from_file_location("task28_t0_torque_bridge_profile_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_task27_profile_uses_original_roots_and_graph():
    profile = load_bridge_module().SCENE_PROFILES["task27"]
    assert profile["scene_root"] == "/World/Task27"
    assert profile["roots"] == {
        "left": "/World/left_fr3",
        "right": "/World/right_fr3",
    }
    assert profile["graph_path"] == "/World/Task27ROSGraph"
    assert "/World/Task27/Supply/Cube_05" in profile["signature_paths"]
    assert "/World/Task27/TruckBox/WallPlusY" in profile["signature_paths"]


def test_task27_profile_does_not_use_fixture_identity():
    profile = load_bridge_module().SCENE_PROFILES["task27"]
    assert profile["label"] == "TASK27_ORIGINAL_FIVE_CUBE_CENTER_INSERT"
    assert "Task28G10SingleCube" not in profile["scene_root"]
