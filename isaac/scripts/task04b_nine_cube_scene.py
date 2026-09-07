# Task 04-B：顶部吸盘 + 九 Cube 随机取料场景
# 在 Isaac Sim 4.5 Script Editor 中、Timeline 停止时运行。
# 目标：9 个 Cube 随机取料；码垛目标在 X/Y 平面形成 3x2 底层，并在其中一排继续叠第二层。

import math
import random
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf

stage = omni.usd.get_context().get_stage()

# ============================================================
# 参数
# ============================================================

NUM_CUBES = 9
CUBE_SIZE = 0.030
CUBE_Z = 0.065
CUBE_MASS = 0.20

# 随机取料区：沿用 Task04-A 的已验证区域。
# 9 个 Cube 时把最小中心距从 60 mm 调为 45 mm，仍保留 15 mm 物体间隙。
PICK_X_MIN = 0.34
PICK_X_MAX = 0.49
PICK_Y_MIN = 0.08
PICK_Y_MAX = 0.25
MIN_CENTER_DISTANCE = 0.045

# None = 每次真正随机；改成整数可复现实验
RANDOM_SEED = None

# Task04-B 只预定义 XY，不预定义 Z。
# Z 由 ROS/MoveIt 控制器根据“目标 XY 当前位置的最高支撑表面”动态计算。
# 底层：3 x 2 = 6 个；第二层：在第一排 3 个位置继续向上叠。
# 相邻中心距 32 mm，对 30 mm Cube 保留 2 mm 间隙。
STACK_TARGET_XY = [
    (0.620, -0.134),
    (0.652, -0.134),
    (0.684, -0.134),
    (0.620, -0.166),
    (0.652, -0.166),
    (0.684, -0.166),
    (0.620, -0.134),
    (0.652, -0.134),
    (0.684, -0.134),
]

# 与 Franka 官方 cobot_pump 的工具 TCP 偏置对齐。
SUCTION_TCP_Z = 0.1050
STEM_RADIUS = 0.006
CUP_RADIUS = 0.010
CUP_HEIGHT = 0.006

COLORS = [
    Gf.Vec3f(0.95, 0.25, 0.20),
    Gf.Vec3f(0.20, 0.55, 0.95),
    Gf.Vec3f(0.20, 0.80, 0.35),
    Gf.Vec3f(0.95, 0.65, 0.15),
    Gf.Vec3f(0.65, 0.35, 0.90),
    Gf.Vec3f(0.15, 0.80, 0.80),
    Gf.Vec3f(0.90, 0.40, 0.65),
    Gf.Vec3f(0.55, 0.75, 0.20),
    Gf.Vec3f(0.35, 0.55, 0.85),
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
    for _ in range(20000):
        x = random.uniform(PICK_X_MIN, PICK_X_MAX)
        y = random.uniform(PICK_Y_MIN, PICK_Y_MAX)

        if all(
            math.hypot(x - px, y - py) >= MIN_CENTER_DISTANCE
            for px, py, _ in result
        ):
            result.append((x, y, CUBE_Z))
            if len(result) == count:
                return result

    raise RuntimeError("随机取料区无法生成 9 个满足安全距离的 Cube，请调整随机区域或最小间距")

# ============================================================
# 1. 清理旧物体
# ============================================================

for path in ["/World/PickCube", "/World/BoxB", "/World/BoxC"]:
    remove_if_exists(path)

for i in range(1, NUM_CUBES + 1):
    remove_if_exists(f"/World/Cube{i}")

# ============================================================
# 2. 保留 Franka articulation/joint，但移除原 Hand/Finger 外观与碰撞
# ============================================================

disable_visual_and_collision("/World/fr3/fr3_hand/visuals")
disable_visual_and_collision("/World/fr3/fr3_hand/collisions")
disable_visual_and_collision("/World/fr3/fr3_leftfinger")
disable_visual_and_collision("/World/fr3/fr3_rightfinger")

# ============================================================
# 3. 重建紧凑吸盘
# ============================================================

hand_path = "/World/fr3/fr3_hand"
tool_path = hand_path + "/suction_tool"
remove_if_exists(tool_path)

UsdGeom.Xform.Define(stage, tool_path)

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
# 4. 生成九个随机 Cube
# ============================================================

positions = sample_positions(NUM_CUBES)
for i, pos in enumerate(positions):
    create_cube(f"/World/Cube{i + 1}", pos, COLORS[i])

# ============================================================
# 输出
# ============================================================

print("")
print("====================================================")
print("Task04-B 九 Cube 随机取料场景创建完成")
print(f"Suction TCP local Z = {SUCTION_TCP_Z:.4f} m")
print("")
print("随机 PICK 位姿：")
for i, pos in enumerate(positions):
    print(f"Cube{i + 1} pick = ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})")
print("")
print("码垛 XY 顺序（Z 由控制器按当前最高表面动态计算）：")
for i, pos in enumerate(STACK_TARGET_XY):
    print(f"Cube{i + 1} target_xy = ({pos[0]:.4f}, {pos[1]:.4f})")
print("")
print("结构：前 6 个形成 3x2 底层；Cube7~9 在第一排对应位置形成第二层。")
print("下一步：保存 Stage，点击 Play，再运行 task04b_suction_ros_bridge.py")
print("====================================================")
