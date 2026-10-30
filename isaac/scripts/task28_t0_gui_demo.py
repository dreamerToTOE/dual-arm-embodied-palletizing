#!/usr/bin/env python3
"""Non-headless Isaac Sim GUI runner for the T0 torque handoff evidence."""
from __future__ import annotations

import importlib.util
import os
from pathlib import Path
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


def _enable_default_viewport_lighting():
    """Enable Isaac's built-in viewport light rig in the session layer.

    The frozen Task28 fixture intentionally has no authored light.  The
    viewport lighting menu mounts its ``Default`` rig in the USD session
    layer, so this changes only the GUI presentation and does not edit or
    save the scene, robot asset, tools, or physics configuration.
    """
    import omni.kit.commands

    result = omni.kit.commands.execute(
        "SetLightingMenuModeCommand",
        lighting_mode="Default",
        usd_context_name="",
    )
    # The command returns (success, (mode, source)); keep a strict diagnostic
    # so a future Kit change cannot silently recreate the black-viewport gate.
    if not result or result[0] is not True:
        raise RuntimeError(f"VIEWPORT_LIGHTING_DEFAULT_REJECTED:{result!r}")
    return result


def _capture_viewport(app, viewport, path):
    """Capture only the active Isaac viewport, independent of desktop windows."""
    from omni.kit.viewport.utility import capture_viewport_to_file

    path = Path(path)
    helper = capture_viewport_to_file(viewport, file_path=str(path), is_hdr=False)
    # Kit completes the future-like capture on subsequent render updates.  Do
    # not use a root-window screenshot: another GUI window can cover it.
    for _ in range(15):
        app.update()
    if not path.is_file() or path.stat().st_size == 0:
        raise RuntimeError(f"VIEWPORT_CAPTURE_MISSING:{path}")
    return helper


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
    lighting_result = _enable_default_viewport_lighting()
    print(f"Task28-T0 viewport lighting: {lighting_result!r}")
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
    # Keep the full GUI renderer visible.  The light rig above is a viewport
    # session-layer setting; no light prim is added to the frozen scene.
    try:
        viewport.render_mode = "RaytracedLighting"
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
    _capture_viewport(app, viewport, screenshot_dir / "gui_viewport_scene_loaded.png")
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
    _capture_viewport(app, viewport, screenshot_dir / "gui_viewport_torque_final.png")
    time.sleep(1.0)
    timeline.stop()
    app.update()
    app.close()
    return 0 if bridge._latched_error is None else 2


if __name__ == "__main__":
    raise SystemExit(main())

