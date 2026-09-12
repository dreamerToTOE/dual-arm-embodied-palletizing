# Task18：Randomized Scene + Ground Truth Task Input 的自包含 Isaac 场景。
#
# 用法（Isaac Sim 4.5 Script Editor，Timeline Stop）：
#   exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#             "task18_randomized_scene.py").read())
#
# 运行前可在启动 Isaac 的终端设置：
#   export TASK18_SEED=20260912  # 固定 seed，完全复现
#   export TASK18_SEED=random    # 每次生成新 episode，Console 会打印实际 seed
#
# 该脚本不依赖此前打开过 Task06/Task15：空 Stage 时自动构建双 FR3、compact
# suction、桌面、物理场景与双臂 ROS ActionGraph。它只创建随机物体，不启动
# Ground Truth bridge、不执行 MoveIt、不发送机器人/吸盘命令。

import json
import math
import os
import random
import secrets
import sys


PROJECT_ROOT = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing"
TASK06_SCENE_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task06_dual_fr3_scene.py"
TASK06_GRAPH_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task06_dual_ros_graph.py"

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
TASK06_GRAPH_PATH = "/World/Task06ROSGraph"
TASK18_PREFIX = "/World/Task18Box_"

# 与 Task06 的桌面一致。表面 z=0.050 m；下面的安全区域已经扣除 50 mm 边界。
TABLE_SUPPORT_Z = 0.050
TABLE_MIN_X = -0.050
TABLE_MAX_X = 1.150
TABLE_MIN_Y = -0.400
TABLE_MAX_Y = +0.400
TABLE_EDGE_MARGIN = 0.050
OBJECT_GAP = 0.020

# 这些 envelope 不是临时 IK 猜测，而是 Task15 已成功执行的取料区的保守子集：
# loose 物体置于左右臂各自一侧，大件置于两臂公共工作区。布局采样器先满足该
# 可行 envelope，再用 no-overlap/boundary 检查拒绝不安全候选。
LOOSE_X_RANGE = (0.250, 0.450)
LEFT_LOOSE_Y_RANGE = (-0.295, -0.140)
RIGHT_LOOSE_Y_RANGE = (+0.140, +0.295)
# 当公共区放置大件后，单臂取料区收缩到离大件更远的一侧；配合左右均衡的
# 分配，每侧至多 2 个小件，避免某些随机 seed 将狭窄取料带塞满。
LEFT_LOOSE_Y_WITH_SHARED_RANGE = (-0.295, -0.230)
RIGHT_LOOSE_Y_WITH_SHARED_RANGE = (+0.230, +0.295)
SHARED_X_RANGE = (0.530, 0.580)
SHARED_Y_RANGE = (-0.030, +0.030)


def _top_grasp(identifier, local_position):
    return {
        "id": identifier,
        "local_pose": {
            "position": list(local_position),
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
    }


# 物体类型只表达尺寸、质量、抓取点及允许的协调模式；绝不携带初始 world 坐标。
# 初始 pose 完全由本 episode 的 RNG 生成，ROS 控制端仅从 bridge 的 BoxStateArray
# 获得结果。
BOX_CATALOG = (
    {
        "type": "small_cube",
        "size": (0.030, 0.030, 0.030),
        "mass": 0.20,
        "payload_class": "small",
        "allowed_modes": ("loose_left", "loose_right"),
        "grasp_builder": lambda size: (_top_grasp("center_top", (0.0, 0.0, 0.5 * size[2])),),
    },
    {
        "type": "medium_box",
        "size": (0.050, 0.040, 0.035),
        "mass": 0.35,
        "payload_class": "medium",
        "allowed_modes": ("loose_left", "loose_right"),
        "grasp_builder": lambda size: (_top_grasp("center_top", (0.0, 0.0, 0.5 * size[2])),),
    },
    {
        "type": "slim_box",
        "size": (0.060, 0.030, 0.025),
        "mass": 0.30,
        "payload_class": "slim",
        "allowed_modes": ("loose_left", "loose_right"),
        "grasp_builder": lambda size: (_top_grasp("center_top", (0.0, 0.0, 0.5 * size[2])),),
    },
    {
        "type": "large_shared_box",
        "size": (0.220, 0.320, 0.080),
        "mass": 1.00,
        "payload_class": "large_shared",
        "allowed_modes": ("tight_shared_object",),
        "grasp_builder": lambda size: (
            _top_grasp("left_top", (0.0, -0.100, 0.5 * size[2])),
            _top_grasp("right_top", (0.0, +0.100, 0.5 * size[2])),
        ),
    },
)


def _resolve_seed(seed_text=None):
    """返回实际 uint32 seed；random 模式的实际 seed 必须记入日志和消息。"""
    raw = str(seed_text if seed_text is not None else os.environ.get("TASK18_SEED", "20260912"))
    raw = raw.strip().lower()
    if raw == "random":
        return secrets.randbelow(2**32)
    try:
        seed = int(raw, 10)
    except ValueError as exc:
        raise RuntimeError("TASK18_SEED 必须是非负整数或 'random'。") from exc
    if seed < 0 or seed >= 2**32:
        raise RuntimeError("TASK18_SEED 必须在 [0, 2^32) 内。")
    return seed


def _yaw_quaternion(yaw):
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def _footprint_radius(size):
    """旋转矩形的外接圆半径，作为保守 no-overlap 判据。"""
    return 0.5 * math.hypot(size[0], size[1])


def _within_table_boundary(candidate):
    radius = candidate["radius"] + TABLE_EDGE_MARGIN
    x, y, _ = candidate["pose"]["position"]
    return (
        TABLE_MIN_X + radius <= x <= TABLE_MAX_X - radius
        and TABLE_MIN_Y + radius <= y <= TABLE_MAX_Y - radius
    )


def _does_not_overlap(candidate, accepted):
    x, y, _ = candidate["pose"]["position"]
    for other in accepted:
        other_x, other_y, _ = other["pose"]["position"]
        required = candidate["radius"] + other["radius"] + OBJECT_GAP
        if math.hypot(x - other_x, y - other_y) < required:
            return False
    return True


def _sample_pose(rng, box_type, side, accepted):
    size = box_type["size"]
    for _ in range(250):
        if side == "shared":
            x = rng.uniform(*SHARED_X_RANGE)
            y = rng.uniform(*SHARED_Y_RANGE)
            # 长件 180 度旋转仍是等价的双臂可行抓取；避免任意 yaw 破坏双吸盘对称。
            yaw = rng.choice((0.0, math.pi))
        else:
            x = rng.uniform(*LOOSE_X_RANGE)
            has_shared_object = any(item["reachable_by"] == "both" for item in accepted)
            if side == "left":
                y_range = (
                    LEFT_LOOSE_Y_WITH_SHARED_RANGE if has_shared_object else LEFT_LOOSE_Y_RANGE
                )
            else:
                y_range = (
                    RIGHT_LOOSE_Y_WITH_SHARED_RANGE if has_shared_object else RIGHT_LOOSE_Y_RANGE
                )
            y = rng.uniform(*y_range)
            yaw = rng.uniform(-math.pi, math.pi)

        candidate = {
            "pose": {
                "position": (x, y, TABLE_SUPPORT_Z + 0.5 * size[2]),
                "orientation": _yaw_quaternion(yaw),
            },
            "radius": _footprint_radius(size),
        }
        if _within_table_boundary(candidate) and _does_not_overlap(candidate, accepted):
            return candidate
    raise RuntimeError(
        "Task18 随机布局在 250 次内未找到无重叠、可达且满足桌边安全边界的候选；"
        "请缩小 count 或调整 catalog/workspace envelope。"
    )


def generate_layout(seed_text=None):
    """纯 Python 随机布局器，可在无 Isaac 的机器上用于确定性回归测试。"""
    seed = _resolve_seed(seed_text)
    rng = random.Random(seed)
    accepted = []
    large_type = next(item for item in BOX_CATALOG if item["type"] == "large_shared_box")
    loose_types = tuple(item for item in BOX_CATALOG if item["type"] != "large_shared_box")

    # object count 与 type 均随机：每个 episode 有 3--6 件，至多 1 个双臂大件。
    total_count = rng.randint(3, 6)
    include_large = rng.random() < 0.50
    if include_large:
        # 有大件时保留每侧两个小件的空间容量；小件总数仍随机。
        total_count = rng.randint(3, 5)
        sampled = _sample_pose(rng, large_type, "shared", accepted)
        accepted.append({
            "id": "task18_box_01",
            "type": large_type["type"],
            "size": large_type["size"],
            "mass": large_type["mass"],
            "payload_class": large_type["payload_class"],
            "allowed_modes": large_type["allowed_modes"],
            "grasp_candidates": large_type["grasp_builder"](large_type["size"]),
            "pose": sampled["pose"],
            "radius": sampled["radius"],
            "reachable_by": "both",
        })

    loose_side_counts = {"left": 0, "right": 0}
    while len(accepted) < total_count:
        box_type = rng.choice(loose_types)
        # 先随机，再以容量均衡约束打破平局。这样 6 件 loose 物体为 3/3，
        # 有 shared 大件时最多为 2/2，不会偶发把一侧可达带耗尽。
        if loose_side_counts["left"] < loose_side_counts["right"]:
            side = "left"
        elif loose_side_counts["right"] < loose_side_counts["left"]:
            side = "right"
        else:
            side = rng.choice(("left", "right"))
        sampled = _sample_pose(rng, box_type, side, accepted)
        accepted.append({
            "id": f"task18_box_{len(accepted) + 1:02d}",
            "type": box_type["type"],
            "size": box_type["size"],
            "mass": box_type["mass"],
            "payload_class": box_type["payload_class"],
            "allowed_modes": box_type["allowed_modes"],
            "grasp_candidates": box_type["grasp_builder"](box_type["size"]),
            "pose": sampled["pose"],
            "radius": sampled["radius"],
            "reachable_by": side,
        })
        loose_side_counts[side] += 1
    return seed, accepted


def _layout_signature(layout):
    """回归用的可比较签名；不含运行期 USD handle。"""
    return json.dumps(layout, sort_keys=True, separators=(",", ":"))


def _validate_layout_generation():
    """不导入 omni 的确定性/安全性自检：python3 ... --validate。"""
    seed = 20260912
    actual_a, layout_a = generate_layout(seed)
    actual_b, layout_b = generate_layout(seed)
    _, layout_c = generate_layout(seed + 1)
    if actual_a != seed or actual_b != seed or _layout_signature(layout_a) != _layout_signature(layout_b):
        raise RuntimeError("T18-01 FAIL：同一 seed 没有生成完全相同的布局。")
    if _layout_signature(layout_a) == _layout_signature(layout_c):
        raise RuntimeError("T18-02 FAIL：不同 seed 生成了相同布局。")
    for candidate in layout_a:
        if not _within_table_boundary(candidate) or not _does_not_overlap(
            candidate, [item for item in layout_a if item is not candidate]
        ):
            raise RuntimeError("Task18 布局安全检查 FAIL。")
    print("Task18 layout generator PASS")
    print(f"same seed={seed}: count={len(layout_a)}, deterministic=true")
    print(f"different seed={seed + 1}: different_layout=true")
    for candidate in layout_a:
        position = candidate["pose"]["position"]
        print(
            f"  {candidate['id']} type={candidate['type']} reachable_by={candidate['reachable_by']} "
            f"pose=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
        )


def _run_project_script(path, label):
    try:
        with open(path, "r", encoding="utf-8") as script_file:
            source = script_file.read()
    except OSError as exc:
        raise RuntimeError(f"无法读取 {label} 脚本：{path}") from exc
    print(f"Task18 自包含基线：运行 {label} -> {path}")
    exec(compile(source, path, "exec"), {"__name__": "__task18_baseline__"})


def _run_isaac_scene():
    # 延迟导入令 --validate 可在没有 Isaac Sim 的 ROS/CI 主机上运行。
    import omni.kit.app
    import omni.physx
    import omni.timeline
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("当前没有有效 USD Stage。")

    def has_prim(path):
        return stage.GetPrimAtPath(path).IsValid()

    def remove_if_exists(path):
        if has_prim(path):
            stage.RemovePrim(path)

    def ensure_physics_scene():
        if has_prim("/physicsScene"):
            return
        physics_scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
        print("Task18 自包含基线：已创建 /physicsScene（gravity=9.81 m/s^2）。")

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError("请先 Stop Timeline，再运行 Task18 场景脚本。")

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    ensure_physics_scene()
    required = (
        LEFT_ROOT,
        RIGHT_ROOT,
        TABLE_PATH,
        f"{LEFT_ROOT}/fr3_hand/suction_tool/suction_tcp",
        f"{RIGHT_ROOT}/fr3_hand/suction_tool/suction_tcp",
    )
    if not all(has_prim(path) for path in required):
        _run_project_script(TASK06_SCENE_SCRIPT, "Task06-A 双 FR3 + compact suction")
    missing = [path for path in required if not has_prim(path)]
    if missing:
        raise RuntimeError("Task18 基线构建失败，仍缺少：\n  - " + "\n  - ".join(missing))
    if not has_prim(TASK06_GRAPH_PATH):
        _run_project_script(TASK06_GRAPH_SCRIPT, "Task06-B 双臂 ROS ActionGraph")
    if not has_prim(TASK06_GRAPH_PATH):
        raise RuntimeError(f"Task18 无法创建 {TASK06_GRAPH_PATH}。")

    # 删除本任务重新运行的所有随机物体，并清理旧 Task15 / 单臂遗留物体。
    for prim in list(stage.Traverse()):
        path = prim.GetPath().pathString
        if path.startswith(TASK18_PREFIX):
            stage.RemovePrim(path)
    for path in (
        "/World/Task15LargeCube",
        "/World/BoxA", "/World/BoxB", "/World/BoxC", "/World/BoxD",
        "/World/PickCube", "/World/SharedBox",
    ):
        remove_if_exists(path)
    for index in range(1, 10):
        remove_if_exists(f"/World/Cube{index}")
        remove_if_exists(f"/World/Task15SmallCube{index}")

    seed, layout = generate_layout()
    palette = (
        Gf.Vec3f(0.95, 0.35, 0.18),
        Gf.Vec3f(0.22, 0.50, 0.95),
        Gf.Vec3f(0.96, 0.72, 0.16),
        Gf.Vec3f(0.58, 0.28, 0.88),
        Gf.Vec3f(0.18, 0.72, 0.44),
        Gf.Vec3f(0.84, 0.42, 0.66),
    )
    for index, item in enumerate(layout):
        prim_path = TASK18_PREFIX + item["id"].removeprefix("task18_box_")
        cube = UsdGeom.Cube.Define(stage, prim_path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([palette[index % len(palette)]])
        xform = UsdGeom.Xformable(cube.GetPrim())
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*item["pose"]["position"]))
        yaw = math.degrees(2.0 * math.atan2(
            item["pose"]["orientation"][2], item["pose"]["orientation"][3]))
        xform.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, yaw))
        xform.AddScaleOp().Set(Gf.Vec3f(*item["size"]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(item["mass"])

        # Bridge 只依赖本 metadata 与 prim 的实时 world transform，不依赖数量、
        # 固定 index 或源码坐标。scene 重新生成后 topic 会自动反映新 episode。
        metadata = {
            "id": item["id"],
            "type": item["type"],
            "size": list(item["size"]),
            "mass": item["mass"],
            "payload_class": item["payload_class"],
            "allowed_modes": list(item["allowed_modes"]),
            "grasp_candidates": item["grasp_candidates"],
            "seed": seed,
            "reachable_by": item["reachable_by"],
        }
        cube.GetPrim().SetCustomDataByKey("task18_metadata", json.dumps(metadata, sort_keys=True))

    print("")
    print("====================================================")
    print("Task18 randomized dual-FR3 scene ready")
    print("复用：Task06 双 FR3 + compact suction + /World/Table + Task06ROSGraph")
    print(f"Task18 actual seed = {seed}")
    print(f"Task18 object count = {len(layout)}")
    for item in layout:
        px, py, pz = item["pose"]["position"]
        print(
            f"{item['id']}: type={item['type']}, class={item['payload_class']}, "
            f"size={item['size']}, mass={item['mass']:.3f}, "
            f"pose=({px:.3f}, {py:.3f}, {pz:.3f}), reachable_by={item['reachable_by']}"
        )
    print("Safety: no initial overlap, table-edge margin=50 mm, conservative reachable envelope checked.")
    print("Next: Timeline Play, then run task18_ground_truth_bridge.py.")
    print("====================================================")


if "--validate" in sys.argv:
    _validate_layout_generation()
else:
    _run_isaac_scene()
