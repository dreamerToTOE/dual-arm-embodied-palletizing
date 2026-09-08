# Task07：松协调双臂并行基线场景
#
# 复用 Task06 已验收双 FR3 + 双 compact suction + Table。
# 本脚本只新增 Task07 两个独立箱体，不重复创建机械臂、吸盘、ROS Graph。
#
# 在 Isaac Sim 4.5 Script Editor、Timeline Stop 时运行。

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

# Task07 第一轮故意使用两条相互分离的通道。
# 目标只证明两个 Task04 primitive 可以真正并发，不提前混入冲突处理。
BOX_A_PICK = (0.320, -0.250, BOX_Z)
BOX_B_PICK = (0.320, +0.250, BOX_Z)

BOX_A_TARGET = (0.820, -0.250, BOX_Z)
BOX_B_TARGET = (0.820, +0.250, BOX_Z)


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


require(LEFT_ROOT)
require(RIGHT_ROOT)
require(TABLE_PATH)

# 清掉旧 Task 物体，但不碰双臂本体、吸盘、Table、ActionGraph。
for path in ["/World/BoxA", "/World/BoxB", "/World/BoxC", "/World/PickCube"]:
    remove_if_exists(path)
for i in range(1, 10):
    remove_if_exists(f"/World/Cube{i}")

create_box("/World/BoxA", BOX_A_PICK, Gf.Vec3f(0.90, 0.30, 0.20))
create_box("/World/BoxB", BOX_B_PICK, Gf.Vec3f(0.20, 0.45, 0.95))

print("")
print("====================================================")
print("Task07 双臂并行场景已创建")
print("只新增 BoxA / BoxB；Task06 双臂基础设施保持不变")
print("")
print(f"BoxA pick   = {BOX_A_PICK}")
print(f"BoxA target = {BOX_A_TARGET}")
print(f"BoxB pick   = {BOX_B_PICK}")
print(f"BoxB target = {BOX_B_TARGET}")
print("")
print("两条通道刻意分离：Task07 只验证并行，不验证冲突处理。")
print("下一步：Play，然后运行 task07_dual_suction_bridge.py")
print("====================================================")
