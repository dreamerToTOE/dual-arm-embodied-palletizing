# Task15：紧/松协调混合码垛场景
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。
# 本脚本只建立可重复使用的 Task15 物理场景，不启动 ROS Bridge，也不执行
# MoveIt 控制。它复用 Task06 的双 FR3、紧凑顶部吸盘和既有 Table：
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


def require(path):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"缺少 {path}。请先运行 Task06 双 FR3 场景。")


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


require(LEFT_ROOT)
require(RIGHT_ROOT)
require(TABLE_PATH)

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
print("复用：Task06 双 FR3 + compact suction + 现有 /World/Table")
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
print("This script creates the scene only; Task15 ROS bridge/controller is intentionally not started yet.")
print("====================================================")
