#!/usr/bin/env python3
"""Non-headless Isaac Sim GUI runner for the T0 torque handoff evidence."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

REPO = Path(__file__).resolve().parents[2]
ISAAC_ROOT = Path("/home/ubuntu2004/isaacsim-4.5.0")
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("TASK28_T0_GUI", "1")
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from isaacsim import SimulationApp


def _load_scene(path):
    namespace = {"__name__": "__task28_t0_original_task24_scene__"}
    source = path.read_text(encoding="utf-8")
    exec(compile(source, str(path), "exec"), namespace, namespace)
    if "build_scene" in namespace:
        namespace["build_scene"]()
    return namespace


def main():
    output = REPO / "offline/task28_t0/evidence/gui_task24_torque_frames.jsonl"
    app = SimulationApp({
        "headless": False,
        "create_new_stage": True,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
        "anti_aliasing": 2,
        "sync_loads": True,
    })
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom

    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    # This is the already approved Task28 A/G1 fixture derived from the
    # historical Task24-H geometry.  It is loaded from the repository and is
    # never exported or modified on disk.
    scene = REPO / "fixtures/task28_g1_0_new_single_cube_v0/scene/task28_single_cube_scene.py"
    os.environ["TASK28_FR3_USD"] = "/home/ubuntu2004/Documents/Codex/2026-10-25/files-pasted-by-the-user-task/work/g1_0b_asset_audit/fr3.usd"
    os.environ["TASK28_FR3_USD_SHA256"] = "3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f"
    _load_scene(scene)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    # Let the original position drives settle at their authored HOME target
    # before runtime-only effort ownership is enabled.
    for _ in range(120):
        app.update()
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    set_camera_view(
        eye=[1.9, -2.6, 1.5], target=[0.65, 0.0, 0.55],
        camera_prim_path="/OmniverseKit_Persp",
    )
    viewport = get_active_viewport()
    # The fixture has no authored lights.  Wireframe is a viewport display
    # state, so the screenshot remains readable without adding a light prim to
    # the frozen scene.
    try:
        viewport.render_mode = "Wireframe"
    except Exception:
        pass
    try:
        viewport.fill_frame()
    except Exception:
        pass
    for _ in range(10):
        app.update()
    screenshot_dir = REPO / "offline/task28_t0/evidence"
    screenshot_dir.mkdir(parents=True, exist_ok=True)
    subprocess.run(["/usr/bin/import", "-window", "root",
                    str(screenshot_dir / "gui_viewport_scene_loaded.png")], check=False)
    bridge_module_path = REPO / "isaac/scripts/task28_t0_torque_bridge.py"
    spec = importlib.util.spec_from_file_location("task28_t0_torque_bridge", bridge_module_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    bridge = module.start_torque_bridge(scene_profile="task28_g1_0", demo=True,
                                        output=output, demo_frames=90)
    # The bridge stops itself after the hold plus small HOME-neighbourhood move.
    while getattr(bridge, "_physics_sub", None) is not None and not bridge._latched_error:
        app.update()
    subprocess.run(["/usr/bin/import", "-window", "root",
                    str(screenshot_dir / "gui_viewport_torque_final.png")], check=False)
    time.sleep(1.0)
    timeline.stop()
    app.update()
    app.close()
    return 0 if bridge._latched_error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())

