# Task15：紧/松协调混合码垛场景
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。
# 本脚本是 Task15 的自包含场景入口：
#
#   - 若当前 Stage 已有完整 Task06 基线，直接复用；
#   - 若当前是新的空 Stage，则自动运行 Task06-A 双 FR3 场景和 Task06-B
#     ROS ActionGraph 构造，再添加 Task15 物体；
#   - 自动确保 Physics Scene 存在；
#   - 不启动 Surface Gripper ROS bridge，也不执行 MoveIt 控制。
#
# 因而每次新开 Isaac Sim 后，用户只需运行本脚本，不必手工先搭 Task06。
# 机器人、紧凑顶部吸盘和 Table 的构造仍严格复用 Task06 已验收源码：
#
#   Phase A（紧协调）
#   两臂共同搬运一个大件：Task15LargeCube initial -> target
#
#   Phase B（松协调）
#   四个小箱按两批次、每批左右各一个并行放到大件顶面：
#   lower layer: SmallCube1 / SmallCube2
#   upper layer: SmallCube3 / SmallCube4
#
# 注意：大件沿用 Task11--Task13 已验收的 0.220 x 0.320 x 0.080 m
# SharedBox 几何与 +0.100 m X 向运输位移。Prim 名称使用 LargeCube，以与
# Task15 的“一个大 cube、四个小 cube”任务语义保持一致。

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"

LARGE_CUBE_PATH = "/World/Task15LargeCube"
SMALL_CUBE_PREFIX = "/World/Task15SmallCube"

PROJECT_ROOT = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing"
TASK06_SCENE_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task06_dual_fr3_scene.py"
TASK06_GRAPH_SCRIPT = f"{PROJECT_ROOT}/isaac/scripts/task06_dual_ros_graph.py"
TASK06_GRAPH_PATH = "/World/Task06ROSGraph"

# Task11--Task13 已验收的大件几何：底面落在 Table 顶面 z=0.050 m 上。
LARGE_SIZE_X = 0.220
LARGE_SIZE_Y = 0.320
LARGE_SIZE_Z = 0.080
LARGE_MASS = 1.00
LARGE_INITIAL_CENTER = (0.550, 0.000, 0.090)
LARGE_TARGET_CENTER = (0.650, 0.000, 0.090)
LARGE_TOP_Z = LARGE_TARGET_CENTER[2] + 0.5 * LARGE_SIZE_Z

SMALL_SIZE = 0.030
SMALL_MASS = 0.20
SMALL_SOURCE_Z = 0.065
LOWER_LAYER_Z = LARGE_TOP_Z + 0.5 * SMALL_SIZE
UPPER_LAYER_Z = LOWER_LAYER_Z + SMALL_SIZE

# 左臂对应负 y 一侧，右臂对应正 y 一侧，与 Task10 的双臂工作区约定一致。
# 上层只在下层两箱已经 PhysX settle、且已回写为 MoveIt World 碰撞物之后开始。
SMALL_CUBES = (
    {
        "path": f"{SMALL_CUBE_PREFIX}1",
        "arm": "left",
        "batch": "lower",
        "pick": (0.320, -0.250, SMALL_SOURCE_Z),
        "target": (0.650, -0.080, LOWER_LAYER_Z),
        "color": Gf.Vec3f(0.95, 0.35, 0.18),
    },
    {
        "path": f"{SMALL_CUBE_PREFIX}2",
        "arm": "right",
        "batch": "lower",
        "pick": (0.320, +0.250, SMALL_SOURCE_Z),
        "target": (0.650, +0.080, LOWER_LAYER_Z),
        "color": Gf.Vec3f(0.22, 0.50, 0.95),
    },
    {
        "path": f"{SMALL_CUBE_PREFIX}3",
        "arm": "left",
        "batch": "upper",
        "pick": (0.380, -0.250, SMALL_SOURCE_Z),
        "target": (0.650, -0.080, UPPER_LAYER_Z),
        "color": Gf.Vec3f(0.96, 0.72, 0.16),
    },
    {
        "path": f"{SMALL_CUBE_PREFIX}4",
        "arm": "right",
        "batch": "upper",
        "pick": (0.380, +0.250, SMALL_SOURCE_Z),
        "target": (0.650, +0.080, UPPER_LAYER_Z),
        "color": Gf.Vec3f(0.58, 0.28, 0.88),
    },
)


def remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def create_dynamic_cube(path, center, color, size, mass):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([color])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*center))
    xform.AddScaleOp().Set(Gf.Vec3f(*size))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(mass)


def has_prim(path):
    return stage.GetPrimAtPath(path).IsValid()


def run_project_script(path, label):
    """从 Task15 入口调用已验收的 Task06 脚本，不复制其建模细节。"""
    try:
        with open(path, "r", encoding="utf-8") as script_file:
            source = script_file.read()
    except OSError as exc:
        raise RuntimeError(f"无法读取 {label} 脚本：{path}") from exc

    print(f"Task15 自包含基线：运行 {label} -> {path}")
    # 独立 globals 防止 Task06 的临时变量覆盖当前 Task15 常量；两个脚本均直接
    # 从 omni.usd context 取得同一个 Stage，因此实际 USD 修改会保留。
    exec(compile(source, path, "exec"), {"__name__": "__task15_baseline__"})


def ensure_physics_scene():
    """新开空 Stage 时补齐 Task06 原脚本未显式创建的全局物理场景。"""
    physics_scene_path = "/physicsScene"
    if has_prim(physics_scene_path):
        return

    physics_scene = UsdPhysics.Scene.Define(stage, physics_scene_path)
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr().Set(9.81)
    print("Task15 自包含基线：已创建 /physicsScene（gravity=9.81 m/s^2）。")


def ensure_task06_baseline():
    """保证双 FR3、compact suction、Table 与双臂 ROS 控制图完整存在。"""
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        raise RuntimeError(
            "请先 Stop Timeline，再运行 Task15 场景脚本；"
            "这样可以安全创建或重置双 FR3 Articulation。"
        )

    # ROS ActionGraph 创建依赖 Isaac ROS2 Bridge 扩展。显式开启可使全新 Isaac
    # 会话不依赖此前手工点选 Extension 的状态。
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    extension_manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    # 先有全局物理场景，再引用两套 FR3 Articulation；新空 Stage 不依赖 GUI
    # 默认是否自动生成 physicsScene 的版本差异。
    ensure_physics_scene()

    baseline_paths = (
        LEFT_ROOT,
        RIGHT_ROOT,
        TABLE_PATH,
        f"{LEFT_ROOT}/fr3_hand/suction_tool/suction_tcp",
        f"{RIGHT_ROOT}/fr3_hand/suction_tool/suction_tcp",
    )
    if not all(has_prim(path) for path in baseline_paths):
        run_project_script(TASK06_SCENE_SCRIPT, "Task06-A 双 FR3 + compact suction")

    # 重新核验而非静默继续，避免官方 FR3 USD 路径/加载异常时生成半完整 Task15。
    missing = [path for path in baseline_paths if not has_prim(path)]
    if missing:
        raise RuntimeError(
            "Task15 自包含 Task06 基线构建失败，仍缺少：\n  - "
            + "\n  - ".join(missing)
        )

    if not has_prim(TASK06_GRAPH_PATH):
        run_project_script(TASK06_GRAPH_SCRIPT, "Task06-B 双臂 ROS ActionGraph")
    if not has_prim(TASK06_GRAPH_PATH):
        raise RuntimeError(
            f"Task15 无法创建 {TASK06_GRAPH_PATH}；ROS2 ActionGraph 未就绪。"
        )



# ============================================================
# 0. 自包含 Task06 基线：空 Stage 自动搭建双臂、吸盘、桌面、ROS 图和物理场景
# ============================================================

ensure_task06_baseline()

# 不修改 FR3、suction_tool、Table、ActionGraph 或 Task04--Task14 源码。
# 仅清理本任务及旧单/双臂任务可能残留的可移动物体。
for path in [
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
    "/World/BoxD",
    "/World/PickCube",
    "/World/SharedBox",
    "/World/Task15Guides",
    LARGE_CUBE_PATH,
]:
    remove_if_exists(path)

for index in range(1, 10):
    remove_if_exists(f"/World/Cube{index}")

for cube in SMALL_CUBES:
    remove_if_exists(cube["path"])

create_dynamic_cube(
    LARGE_CUBE_PATH,
    LARGE_INITIAL_CENTER,
    Gf.Vec3f(0.82, 0.46, 0.12),
    (LARGE_SIZE_X, LARGE_SIZE_Y, LARGE_SIZE_Z),
    LARGE_MASS,
)

for cube in SMALL_CUBES:
    create_dynamic_cube(
        cube["path"],
        cube["pick"],
        cube["color"],
        (SMALL_SIZE, SMALL_SIZE, SMALL_SIZE),
        SMALL_MASS,
    )

print("")
print("====================================================")
print("Task15 hybrid palletizing scene ready")
print("复用：Task06 双 FR3 + compact suction + /World/Table + Task06ROSGraph")
print(f"LargeCube initial = {LARGE_INITIAL_CENTER} m")
print(f"LargeCube target  = {LARGE_TARGET_CENTER} m")
print(
    "LargeCube size    = "
    f"({LARGE_SIZE_X:.3f}, {LARGE_SIZE_Y:.3f}, {LARGE_SIZE_Z:.3f}) m, "
    f"mass={LARGE_MASS:.3f} kg"
)
print(f"LargeCube top z   = {LARGE_TOP_Z:.3f} m")
for cube in SMALL_CUBES:
    print(
        f"{cube['path']} ({cube['arm']}, {cube['batch']}): "
        f"pick={cube['pick']}, target={cube['target']}"
    )
print(f"SmallCube size/mass = {SMALL_SIZE:.3f} m / {SMALL_MASS:.3f} kg")
print("Strategy Phase A / TIGHT: Task11--13 shared-object protocol moves LargeCube.")
print("Strategy Phase B1 / LOOSE: SmallCube1 + SmallCube2 lower layer in parallel after FCL SAFE.")
print("Strategy Phase B2 / LOOSE: wait for lower-layer settle + World write-back, then SmallCube3 + SmallCube4 in parallel.")
print("Task15 基线就绪：Timeline Play 后运行 task15_hybrid_suction_bridge.py。")
print("====================================================")
