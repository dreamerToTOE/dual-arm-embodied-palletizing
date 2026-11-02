import importlib.util
from pathlib import Path
import pytest


def runner():
    path = Path(__file__).resolve().parents[3] / 'isaac/scripts/task28_task27_one_pass.py'
    spec = importlib.util.spec_from_file_location('sequence_runner_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gui_multi_step_update_uses_physics_time_and_stops_on_bridge_error():
    class Torque:
        _time_s = 0.0
        _step = 0
        _latched_error = None
        _g3_internal_force_target_N = 0.0
        def _log(self, value): pass
        def set_internal_force_target(self, value): self._g3_internal_force_target_N = value
    torque = Torque()
    class App:
        def update(self):
            torque._time_s += 2.0 / 60
            torque._step += 2
    result = runner().run_fixed_force_sequence(App(), torque)
    assert torque._time_s == pytest.approx(28, abs=1e-6)
    assert torque._g3_internal_force_target_N == 0
    assert len(result['segments']) == 4
    torque._latched_error = 'test failure'
    with pytest.raises(RuntimeError, match='BRIDGE_STOPPED'):
        runner().run_fixed_force_sequence(App(), torque)
