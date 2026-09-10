# Task11：双吸盘共同物体基线场景
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。
# 复用 Task06 已验收的双 FR3、紧凑顶部吸盘和桌面；本 Task 只建立一个
# 可被两套 Surface Gripper 同时吸附的大箱体，不做共同搬运或放置。

import omni.usd
from pxr import Gf, UsdPhysics, UsdGeom


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
SHARED_BOX_PATH = "/World/SharedBox"

# 大箱沿 y 轴布置，使两台位于桌面长边两侧的 FR3 可在箱顶的两个分离点吸附。
BOX_SIZE_X = 0.220
BOX_SIZE_Y = 0.320
BOX_SIZE_Z = 0.080
BOX_MASS = 1.00
BOX_CENTER = (0.550, 0.000, 0.090)

# 仅作为 Task11 共同吸附验收参考点。它们均在上表面上方 1 mm：
# box top z = 0.130 m，suction TCP contact z = 0.131 m。
LEFT_SUCTION_POINT = (0.550, -0.100, 0.131)
RIGHT_SUCTION_POINT = (0.550, +0.100, 0.131)


def require(path):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"缺少 {path}。请先运行 Task06 双 FR3 场景。")


def remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def create_shared_box():
    box = UsdGeom.Cube.Define(stage, SHARED_BOX_PATH)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(0.82, 0.46, 0.12)])

    xform = UsdGeom.Xformable(box.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*BOX_CENTER))
    xform.AddScaleOp().Set(Gf.Vec3f(BOX_SIZE_X, BOX_SIZE_Y, BOX_SIZE_Z))

    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(box.GetPrim())
    mass_api.CreateMassAttr(BOX_MASS)


require(LEFT_ROOT)
require(RIGHT_ROOT)
require(TABLE_PATH)

# 不修改 FR3、suction_tool、Table 或 Task06 ActionGraph；仅清理旧任务物体。
for path in [
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
    "/World/BoxD",
    "/World/PickCube",
    SHARED_BOX_PATH,
]:
    remove_if_exists(path)

for index in range(1, 10):
    remove_if_exists(f"/World/Cube{index}")

create_shared_box()

print("")
print("====================================================")
print("Task11 双吸盘共同物体场景已创建")
print("复用：Task06 双 FR3 + compact suction + Table")
print(f"SharedBox center = {BOX_CENTER} m")
print(f"SharedBox size   = ({BOX_SIZE_X:.3f}, {BOX_SIZE_Y:.3f}, {BOX_SIZE_Z:.3f}) m")
print(f"SharedBox mass   = {BOX_MASS:.3f} kg")
print(f"LEFT suction reference  = {LEFT_SUCTION_POINT} m")
print(f"RIGHT suction reference = {RIGHT_SUCTION_POINT} m")
print("两吸附点 y 向间距 = 0.200 m；共同搬运留待 Task12。")
print("下一步：Timeline Play 后运行 task11_shared_box_bridge.py。")
print("====================================================")
