# Task08：松协调双臂交叉冲突场景
#
# 复用 Task06 已验收的双 FR3 + 双 compact suction + Table，
# 复用 Task07 已验证的双吸盘 bridge。
# 本脚本只改变 BoxA / BoxB 的任务几何，让两条搬运任务进入公共工作区。
#
# 目标不是在 Isaac 里真的让两臂相撞，而是给 Task08 的
# SpatioTemporalConflictDetector 提供“单臂可规划、同时执行可能冲突”的场景。
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。

import math

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"

BOX_SIZE = 0.030
BOX_Z = 0.065
BOX_MASS = 0.20

# ------------------------------------------------------------
# Task08 交叉场景参数
# ------------------------------------------------------------
# PICK 继续复用 Task07 已验证起点，避免重新引入抓取可达性变量。
BOX_A_PICK = (0.320, -0.250, BOX_Z)
BOX_B_PICK = (0.320, +0.250, BOX_Z)

# 两个目标交换到对侧公共工作区。
# y=±0.120 比直接完全交换到 ±0.250 更保守：
# - 左右臂都需要跨过 y=0；
# - 目标本身仍保持 240 mm 间距，不人为制造终点箱体碰撞；
# - 单臂可达性优先保持充足余量。
TARGET_X = 0.820
CROSS_TARGET_Y = 0.120
BOX_A_TARGET = (TARGET_X, +CROSS_TARGET_Y, BOX_Z)
BOX_B_TARGET = (TARGET_X, -CROSS_TARGET_Y, BOX_Z)


# ------------------------------------------------------------
# 工具函数
# ------------------------------------------------------------
def require(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"缺少 {path}。请先运行并保留 Task06 双臂场景。")


def remove_if_exists(path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def create_box(path, xyz, color):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(BOX_SIZE)
    box.CreateDisplayColorAttr([color])

    xform = UsdGeom.Xformable(box.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))

    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(box.GetPrim())
    mass_api.CreateMassAttr(BOX_MASS)


def nominal_xy_intersection(a0, a1, b0, b1):
    """只计算 PICK->TARGET 平面直线的名义交点，用于场景几何检查。

    实际 MoveIt LIFT->PRE_PLACE 是关节空间规划，机械臂真实轨迹不等于这条直线；
    因此该交点只是说明两项任务的空间意图确实交叉，不作为碰撞判据。
    """
    ax0, ay0 = a0[0], a0[1]
    ax1, ay1 = a1[0], a1[1]
    bx0, by0 = b0[0], b0[1]
    bx1, by1 = b1[0], b1[1]

    dax = ax1 - ax0
    day = ay1 - ay0
    dbx = bx1 - bx0
    dby = by1 - by0

    det = dax * dby - day * dbx
    if abs(det) < 1e-9:
        return None

    dx = bx0 - ax0
    dy = by0 - ay0
    ta = (dx * dby - dy * dbx) / det
    tb = (dx * day - dy * dax) / det

    if not (0.0 <= ta <= 1.0 and 0.0 <= tb <= 1.0):
        return None

    return (
        ax0 + ta * dax,
        ay0 + ta * day,
    )


# ------------------------------------------------------------
# 1. 复用 Task06 双臂基础设施
# ------------------------------------------------------------
require(LEFT_ROOT)
require(RIGHT_ROOT)
require(TABLE_PATH)

# ------------------------------------------------------------
# 2. 只重置任务物体，不碰机器人 / 吸盘 / Table / ActionGraph
# ------------------------------------------------------------
for path in [
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
    "/World/PickCube",
    "/World/Task08Guides",
]:
    remove_if_exists(path)

for i in range(1, 10):
    remove_if_exists(f"/World/Cube{i}")

# ------------------------------------------------------------
# 3. 创建两个真实可抓取箱体
# ------------------------------------------------------------
create_box("/World/BoxA", BOX_A_PICK, Gf.Vec3f(0.90, 0.30, 0.20))
create_box("/World/BoxB", BOX_B_PICK, Gf.Vec3f(0.20, 0.45, 0.95))

# ------------------------------------------------------------
# 4. 输出场景几何
# ------------------------------------------------------------
intersection = nominal_xy_intersection(
    BOX_A_PICK,
    BOX_A_TARGET,
    BOX_B_PICK,
    BOX_B_TARGET,
)

target_separation = math.hypot(
    BOX_A_TARGET[0] - BOX_B_TARGET[0],
    BOX_A_TARGET[1] - BOX_B_TARGET[1],
)

print("")
print("====================================================")
print("Task08 双臂交叉冲突场景已创建")
print("复用：Task06 双臂基础设施 + Task07 双吸盘 bridge")
print("本脚本只改变任务几何，不新增控制链。")
print("")
print(f"BoxA pick   = {BOX_A_PICK}")
print(f"BoxA target = {BOX_A_TARGET}")
print(f"BoxB pick   = {BOX_B_PICK}")
print(f"BoxB target = {BOX_B_TARGET}")
print(f"Target separation = {target_separation:.3f} m")

if intersection is not None:
    print(
        "Nominal PICK->TARGET XY paths intersect near "
        f"({intersection[0]:.3f}, {intersection[1]:.3f})"
    )
else:
    print("WARNING: nominal PICK->TARGET XY paths do not intersect.")

print("")
print("注意：名义 XY 交叉 != 已证明机械臂碰撞。")
print("Task08 后续必须满足：")
print("  1) left candidate 单独规划 PASS")
print("  2) right candidate 单独规划 PASS")
print("  3) 同一时间轴联合预测 CONFLICT")
print("")
print("下一步：Play 后继续复用 task07_dual_suction_bridge.py。")
print("危险 transfer 在 detector 完成前不要直接并发执行。")
print("====================================================")
