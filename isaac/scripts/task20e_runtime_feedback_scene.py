# Task20-E：运行时连续码垛的反馈闭环与效率基准场景。
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行：
#
#   exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#             "task20e_runtime_feedback_scene.py").read())
#
# 复用 Task18 的自包含 Task06 基线（双 FR3、compact suction、Table、Physics、
# ROS ActionGraph），但不复用 Task11 的过大 SharedBox。这里创建一个中等尺寸的
# 共同搬运箱以及两个 tall box；三个源位姿由固定随机 seed 采样，因此不规则、可复现，
# 且不会把规则网格坐标写进 ROS 控制端。

import builtins
import json
import math
import os
import random


PROJECT_ROOT = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing"
TASK18_SCENE_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task18_randomized_scene.py"
ACCEPTANCE_SEED = 20260924
TASK18_PREFIX = "/World/Task18Box_"
TABLE_SUPPORT_Z = 0.050


def _shutdown_old_runtime_bridges():
    for name in (
        "_task20_runtime_suction_bridge",
        "_task18_ground_truth_bridge",
    ):
        bridge = getattr(builtins, name, None)
        if bridge is None:
            continue
        try:
            bridge.shutdown()
        except Exception as exc:
            print(f"Task20-E 清理旧 bridge {name} 时出现非致命异常：{exc}")


def _run_task18_baseline():
    """只复用 Task18 对 Task06 自包含基线的构造，不保留其随机物体。"""
    try:
        with open(TASK18_SCENE_SCRIPT, "r", encoding="utf-8") as source_file:
            source = source_file.read()
    except OSError as exc:
        raise RuntimeError(f"无法读取 Task18 场景脚本：{TASK18_SCENE_SCRIPT}") from exc

    old_seed = os.environ.get("TASK18_SEED")
    os.environ["TASK18_SEED"] = str(ACCEPTANCE_SEED)
    try:
        exec(compile(source, TASK18_SCENE_SCRIPT, "exec"), {
            "__name__": "__task20e_task18_baseline__",
        })
    finally:
        if old_seed is None:
            os.environ.pop("TASK18_SEED", None)
        else:
            os.environ["TASK18_SEED"] = old_seed


def _top_grasp(identifier, local_position):
    return {
        "id": identifier,
        "local_pose": {
            "position": list(local_position),
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
    }


def _sample_layout():
    """固定 seed 的非规则源位姿；各取料子区分离，避免初始碰撞和不可达混淆。"""
    rng = random.Random(ACCEPTANCE_SEED)

    shared_size = (0.160, 0.240, 0.060)
    tall_size = (0.040, 0.040, 0.080)
    return (
        {
            "id": "task18_box_01",
            "type": "medium_shared_box",
            "size": shared_size,
            "mass": 0.70,
            "payload_class": "medium_shared",
            "allowed_modes": ("tight_shared_object",),
            # 两个 20 mm cup 的边缘仍与箱体边界保留 35 mm 余量；相对间距
            # 从 Task11 基线的 200 mm 缩为 150 mm，匹配 240 mm 的 Y 向长度。
            "grasp_candidates": (
                _top_grasp("left_top", (0.0, -0.075, 0.5 * shared_size[2])),
                _top_grasp("right_top", (0.0, +0.075, 0.5 * shared_size[2])),
            ),
            "pose": (
                rng.uniform(0.535, 0.575),
                rng.uniform(-0.038, 0.038),
                TABLE_SUPPORT_Z + 0.5 * shared_size[2],
            ),
            "reachable_by": "both",
        },
        {
            "id": "task18_box_03",
            "type": "tall_box",
            "size": tall_size,
            "mass": 0.45,
            "payload_class": "tall",
            "allowed_modes": ("loose_left", "loose_right"),
            "grasp_candidates": (
                _top_grasp("center_top", (0.0, 0.0, 0.5 * tall_size[2])),
            ),
            "pose": (
                rng.uniform(0.285, 0.425),
                rng.uniform(-0.292, -0.235),
                TABLE_SUPPORT_Z + 0.5 * tall_size[2],
            ),
            "reachable_by": "left",
        },
        {
            "id": "task18_box_05",
            "type": "tall_box",
            "size": tall_size,
            "mass": 0.45,
            "payload_class": "tall",
            "allowed_modes": ("loose_left", "loose_right"),
            "grasp_candidates": (
                _top_grasp("center_top", (0.0, 0.0, 0.5 * tall_size[2])),
            ),
            "pose": (
                rng.uniform(0.285, 0.425),
                rng.uniform(+0.235, +0.292),
                TABLE_SUPPORT_Z + 0.5 * tall_size[2],
            ),
            "reachable_by": "right",
        },
    )


def _replace_runtime_boxes():
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("当前没有有效 USD Stage。")
    for prim in list(stage.Traverse()):
        if prim.GetPath().pathString.startswith(TASK18_PREFIX):
            stage.RemovePrim(prim.GetPath())

    layout = _sample_layout()
    colors = (
        (0.95, 0.35, 0.18),
        (0.22, 0.50, 0.95),
        (0.58, 0.28, 0.88),
    )
    for item, color in zip(layout, colors):
        prim_path = TASK18_PREFIX + item["id"].removeprefix("task18_box_")
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*item["pose"]))
        # 目标姿态由 Task19/MoveIt 生成；初始 yaw 固定仅避免把姿态随机性与本次
        # TCP release feedback 验收混在同一变量内。
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        xform.AddScaleOp().Set(Gf.Vec3f(*item["size"]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(item["mass"])
        cube.GetPrim().SetCustomDataByKey("task18_metadata", json.dumps({
            "id": item["id"],
            "type": item["type"],
            "size": list(item["size"]),
            "mass": item["mass"],
            "payload_class": item["payload_class"],
            "allowed_modes": list(item["allowed_modes"]),
            "grasp_candidates": list(item["grasp_candidates"]),
            "seed": ACCEPTANCE_SEED,
            "reachable_by": item["reachable_by"],
        }, sort_keys=True))
    return layout


_shutdown_old_runtime_bridges()
_run_task18_baseline()
layout = _replace_runtime_boxes()

print("")
print("====================================================")
print("Task20-E runtime feedback / efficiency scene ready")
print(f"fixed replay seed = {ACCEPTANCE_SEED}")
print("shared box size = (0.160, 0.240, 0.060) m, mass=0.70 kg")
print("shared suction grasp local y = (-0.075, +0.075) m")
for item in layout:
    x, y, z = item["pose"]
    print(
        f"{item['id']}: type={item['type']}, "
        f"size={item['size']}, source=({x:.3f}, {y:.3f}, {z:.3f})"
    )
print("Next: Timeline Play -> task18_ground_truth_bridge.py")
print("      -> task20_runtime_suction_bridge.py")
print("      -> task20e_runtime_feedback_acceptance.launch.py")
print("====================================================")
