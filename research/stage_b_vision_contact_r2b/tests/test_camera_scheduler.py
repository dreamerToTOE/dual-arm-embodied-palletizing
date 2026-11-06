import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np


RUNNER = Path(__file__).resolve().parents[3] / "isaac/scripts/task28_task27_one_pass.py"
SPEC = importlib.util.spec_from_file_location("task28_task27_one_pass_camera_test", RUNNER)
RUNNER_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER_MODULE)


def _camera(tmp_path, every_steps=6):
    camera = object.__new__(RUNNER_MODULE.R2BPhysicalCamera)
    camera.every_steps = every_steps
    camera.next_capture_step = every_steps
    camera.last_step = -1
    camera.rows = []
    camera._closed = False
    camera.requested_frames = 0
    camera.successful_frames = 0
    camera.failed_frames = 0
    camera.missed_deadlines = 0
    camera.capture_errors = []
    camera.depth_errors = []
    camera.out = tmp_path
    (tmp_path / "rgb").mkdir()
    (tmp_path / "depth").mkdir()
    camera.viewport = object()
    camera.depth_camera = SimpleNamespace(get_depth=lambda: np.ones((1, 1)))
    captures = []
    def capture(viewport, file_path, is_hdr):
        captures.append(file_path)
        Path(file_path).write_bytes(b"rgb")
    camera._capture = capture
    camera.captures = captures
    return camera


def test_capture_if_due_recovers_after_skipped_modulo_boundary(tmp_path):
    camera = _camera(tmp_path)
    updates = []
    app = SimpleNamespace(update=lambda: updates.append(True))

    camera.capture_if_due(app, SimpleNamespace(step=5, time_s=5 / 60))
    assert camera.rows == []
    camera.capture_if_due(app, SimpleNamespace(step=7, time_s=7 / 60))
    camera.capture_if_due(app, SimpleNamespace(step=7, time_s=7 / 60))

    assert len(camera.captures) == 1
    row = camera.rows[0]
    assert row["physics_step"] == 7
    assert row["scheduled_physics_step"] == 6
    assert row["scheduler_lag_steps"] == 1
    assert row["scheduler_skipped_intervals"] == 0
    assert row["time_s"] == 7 / 60
    assert row["rgb"] == "rgb/frame_000007.png"
    assert row["depth"] == "depth/frame_000007.npy"
    assert row["depth_ok"] is True
    assert row["frame_pair_ok"] is True
    assert row["physics_step_after_render_wait"] == 7
    assert camera.requested_frames == 1
    assert camera.successful_frames == 1
    assert camera.failed_frames == 0
    assert camera.missed_deadlines == 0
    camera.close()
    summary = __import__("json").loads((tmp_path / "camera_acquisition_summary.json").read_text())
    assert summary["requested_frames"] == 1
    assert summary["successful_frames"] == 1
    assert summary["failed_frames"] == 0
    assert camera.next_capture_step == 12
    assert len(updates) == 4


def test_capture_if_due_records_multiple_missed_boundaries(tmp_path):
    camera = _camera(tmp_path)
    app = SimpleNamespace(update=lambda: None)

    camera.capture_if_due(app, SimpleNamespace(step=13, time_s=13 / 60))

    assert camera.rows[0]["physics_step"] == 13
    assert camera.rows[0]["scheduled_physics_step"] == 6
    assert camera.rows[0]["scheduler_lag_steps"] == 7
    assert camera.rows[0]["scheduler_skipped_intervals"] == 1
    assert camera.next_capture_step == 18
    assert camera.requested_frames == 2
    assert camera.successful_frames == 1
    assert camera.missed_deadlines == 1


def test_capture_failure_is_recorded_without_false_pair(tmp_path):
    camera = _camera(tmp_path)
    camera._capture = lambda viewport, file_path, is_hdr: (_ for _ in ()).throw(RuntimeError("render-not-ready"))
    camera.capture_if_due(SimpleNamespace(update=lambda: None), SimpleNamespace(step=6, time_s=0.1))

    row = camera.rows[0]
    assert row["frame_pair_ok"] is False
    assert row["rgb"] is None
    assert row["depth"] is None
    assert camera.successful_frames == 0
    assert camera.failed_frames == 1
    assert camera.capture_errors == ["RuntimeError('render-not-ready')"]
