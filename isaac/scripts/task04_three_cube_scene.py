# Task 04：顶部吸盘 + 三 Cube 随机场景
# 在 Isaac Sim 4.5 Script Editor 中、Timeline 停止时运行。

import math
import random
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# ============================================================
# 参数
# ============================================================

CUBE_SIZE = 0.030
CUBE_Z = 0.065
CUBE_MASS = 0.20

# 随机取料区：与目标码垛区分开
PICK_X_MIN = 0.34
PICK_X_MAX = 0.49
PICK_Y_MIN = 0.08
PICK_Y_MAX = 0.25
MIN_CENTER_DISTANCE = 0.060

# None = 每次真正随机；改成整数可复现实验
RANDOM_SEED = None

# 第一版三 Cube 紧密码放目标，中心距 32 mm，即箱体间 2 mm 安全间隙
STACK_TARGETS = [
    (0.620, -0.150, CUBE_Z),
    (0.652, -0.150, CUBE_Z),
    (0.684, -0.150, CUBE_Z),
]

# Franka 官方 cobot_pump 使用约 0.105 m 的工具 TCP 偏置。
# Task04 的 Isaac 吸盘接触面与该 TCP 对齐，方便 MoveIt 与 Isaac 使用同一末端定义。
SUCTION_TCP_Z = 0.1050
STEM_RADIUS = 0.006
CUP_RADIUS = 0.010
CUP_HEIGHT = 0.006

COLORS = [
    Gf.Vec3f(0.95, 0.25, 0.20),
    Gf.Vec3f(0.20, 0.55, 0.95),
    Gf.Vec3f(0.20, 0.80, 0.35),
]

if RANDOM_SEED is not None:
    random.seed(RANDOM_SEED)

# ============================================================
# 工具函数
# ============================================================

def remove_if_exists(path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def disable_visual_and_collision(root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def create_cube(path, xyz, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(CUBE_SIZE)
    cube.CreateDisplayColorAttr([color])

    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(*xyz))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(CUBE_MASS)


def sample_positions(count):
    result = []
    for _ in range(5000):
        x = random.uniform(PICK_X_MIN, PICK_X_MAX)
        y = random.uniform(PICK_Y_MIN, PICK_Y_MAX)

        if all(math.hypot(x - px, y - py) >= MIN_CENTER_DISTANCE
               for px, py, _ in result):
            result.append((x, y, CUBE_Z))
            if len(result) == count:
                return result

    raise RuntimeError("随机区域太小，无法生成三个互不干涉的 Cube")

# ============================================================
# 1. 清理 Task01/03 旧 Cube
# ============================================================

for path in [
    "/World/PickCube",
    "/World/BoxB",
    "/World/BoxC",
    "/World/Cube1",
    "/World/Cube2",
    "/World/Cube3",
]:
    remove_if_exists(path)

# ============================================================
# 2. 保留 Franka articulation/joint，但移除原 Hand/Finger 实体外观与碰撞
# ============================================================

disable_visual_and_collision("/World/fr3/fr3_hand/visuals")
disable_visual_and_collision("/World/fr3/fr3_hand/collisions")
disable_visual_and_collision("/World/fr3/fr3_leftfinger")
disable_visual_and_collision("/World/fr3/fr3_rightfinger")

# ============================================================
# 3. 重建紧凑吸盘，使吸附接触面与 cobot_pump TCP 对齐
# ============================================================

hand_path = "/World/fr3/fr3_hand"
tool_path = hand_path + "/suction_tool"
remove_if_exists(tool_path)

UsdGeom.Xform.Define(stage, tool_path)

# cup 外侧接触面位于 SUCTION_TCP_Z，因此 cup 中心向内退半个厚度
cup_center_z = SUCTION_TCP_Z - 0.5 * CUP_HEIGHT
stem_end_z = SUCTION_TCP_Z - CUP_HEIGHT
stem_height = stem_end_z

stem = UsdGeom.Cylinder.Define(stage, tool_path + "/stem")
stem.CreateAxisAttr(UsdGeom.Tokens.z)
stem.CreateRadiusAttr(STEM_RADIUS)
stem.CreateHeightAttr(stem_height)
UsdGeom.Xformable(stem.GetPrim()).AddTranslateOp().Set(
    Gf.Vec3d(0.0, 0.0, 0.5 * stem_height)
)
stem.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.25)])
UsdPhysics.CollisionAPI.Apply(stem.GetPrim())

cup = UsdGeom.Cylinder.Define(stage, tool_path + "/cup")
cup.CreateAxisAttr(UsdGeom.Tokens.z)
cup.CreateRadiusAttr(CUP_RADIUS)
cup.CreateHeightAttr(CUP_HEIGHT)
UsdGeom.Xformable(cup.GetPrim()).AddTranslateOp().Set(
    Gf.Vec3d(0.0, 0.0, cup_center_z)
)
cup.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
UsdPhysics.CollisionAPI.Apply(cup.GetPrim())

suction_tcp = UsdGeom.Xform.Define(stage, tool_path + "/suction_tcp")
UsdGeom.Xformable(suction_tcp.GetPrim()).AddTranslateOp().Set(
    Gf.Vec3d(0.0, 0.0, SUCTION_TCP_Z)
)

# ============================================================
# 4. 生成三个随机 Cube
# ============================================================

positions = sample_positions(3)
for i, pos in enumerate(positions):
    create_cube(f"/World/Cube{i + 1}", pos, COLORS[i])

# ============================================================
# 输出
# ============================================================

print("")
print("====================================================")
print("Task04 三 Cube 随机场景创建完成")
print(f"Suction TCP local Z = {SUCTION_TCP_Z:.4f} m")
for i, pos in enumerate(positions):
    print(f"Cube{i + 1} pick = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
for i, pos in enumerate(STACK_TARGETS):
    print(f"Cube{i + 1} place= ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
print("下一步：保存 Stage，点击 Play，再运行 task04_suction_ros_bridge.py")
print("====================================================")
