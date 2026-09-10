# Task10：双 FR3 松协调连续码垛场景（两批次 / 四个小箱体）
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。
# 复用 Task06 的双 FR3、紧凑顶部吸盘和桌面；只重置本 Task 的四个任务箱体。

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

# 两批次分别由左右臂搬运。第一批保留 Task08 的交叉目标，
# 用于验证 Task09 的 local wait；第二批在第一批箱体仍存在于 world
# 碰撞场景时执行，验证连续码垛不会把已放置物体忽略掉。
BOXES = (
    ("/World/BoxA", (0.320, -0.250, BOX_Z), (0.820, +0.120, BOX_Z), Gf.Vec3f(0.90, 0.30, 0.20)),
    ("/World/BoxB", (0.320, +0.250, BOX_Z), (0.820, -0.120, BOX_Z), Gf.Vec3f(0.20, 0.45, 0.95)),
    # 第二批取点与 Task07 已验收的 x=0.320 m 工作区保持接近，避开
    # x=0.420 m 附近 Isaac 物理关节限位导致的吸盘中心偏差。与 A/B 的
    # x 向中心距为 60 mm，故 30 mm 箱体之间仍保留 30 mm 净距。
    ("/World/BoxC", (0.380, -0.250, BOX_Z), (0.740, -0.120, BOX_Z), Gf.Vec3f(0.95, 0.70, 0.15)),
    ("/World/BoxD", (0.380, +0.250, BOX_Z), (0.740, +0.120, BOX_Z), Gf.Vec3f(0.55, 0.20, 0.85)),
)


def require(path):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"缺少 {path}。请先运行 Task06 双 FR3 场景。")


def remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def create_box(path, xyz, color):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(BOX_SIZE)
    box.CreateDisplayColorAttr([color])
    UsdGeom.Xformable(box.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*xyz))

    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(box.GetPrim())
    mass_api.CreateMassAttr(BOX_MASS)


require(LEFT_ROOT)
require(RIGHT_ROOT)
require(TABLE_PATH)

# 不触碰 FR3、吸盘、Table、ActionGraph。清理旧 Task07/08/10 任务箱体即可。
for path in [
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
    "/World/BoxD",
    "/World/PickCube",
    "/World/Task08Guides",
    "/World/Task10Guides",
]:
    remove_if_exists(path)

for index in range(1, 10):
    remove_if_exists(f"/World/Cube{index}")

for path, pick_pose, _, color in BOXES:
    create_box(path, pick_pose, color)

print("")
print("====================================================")
print("Task10 连续松协调码垛场景已创建")
print("复用：Task06 双 FR3 基础设施 + Task10 四箱 Bridge")
print("第一批：BoxA / BoxB；第二批：BoxC / BoxD")
for index, (path, pick_pose, target_pose, _) in enumerate(BOXES, start=1):
    print(f"Batch {(index - 1) // 2 + 1}: {path}")
    print(f"  pick   = {pick_pose}")
    print(f"  target = {target_pose}")
print("第一批交叉目标：x=0.820，y=+/-0.120 m")
print("第二批同侧目标：x=0.740，left=-0.120 / right=+0.120 m")
print("已放置箱体在下一批仍保留为 MoveIt World 碰撞物。")
print("下一步：Timeline Play 后运行 task10_quad_suction_bridge.py。")
print("====================================================")
