# Task05-A：B-A-C 高密度放置的顶部吸盘 Isaac 场景
#
# 在 Isaac Sim 4.5 Script Editor 中、Timeline 停止时运行。
# 本脚本复用 Task04-B 已验证的 FR3、Table 和物理设置；
# 只替换待测 B-A-C 物体与顶部吸盘可视/碰撞几何。

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics


stage = omni.usd.get_context().get_stage()

# ============================================================
# Task03 B-A-C 参数：单位为 m
# ============================================================

CUBE_SIZE = 0.030
CUBE_MASS = 0.20
CUBE_CENTER_Z = 0.065

A_INITIAL_POSE = (0.45, 0.15, CUBE_CENTER_Z)
A_TARGET_POSE = (0.65, -0.15, CUBE_CENTER_Z)
B_POSE = (0.65, -0.115, CUBE_CENTER_Z)
C_POSE = (0.65, -0.185, CUBE_CENTER_Z)

BC_INNER_GAP_MM = 40
A_WIDTH_MM = 30

# 与 Task04-B、Franka 官方 cobot_pump 对齐的工具 TCP 偏置。
SUCTION_TCP_Z = 0.1050
STEM_RADIUS = 0.006
CUP_RADIUS = 0.010
CUP_HEIGHT = 0.006

A_COLOR = Gf.Vec3f(0.95, 0.25, 0.20)
B_COLOR = Gf.Vec3f(0.20, 0.55, 0.95)
C_COLOR = Gf.Vec3f(0.20, 0.80, 0.35)


# ============================================================
# 工具函数
# ============================================================

def remove_if_exists(path):
    """重复运行时删除指定 Prim。"""
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def remove_legacy_task05_objects():
    """删除 /World 下所有旧 Task05 顶层对象，避免遗留命名影响重复运行。"""
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return

    paths = [
        str(child.GetPath())
        for child in world.GetChildren()
        if child.GetName().lower().startswith("task05")
    ]
    for path in paths:
        remove_if_exists(path)


def require_task04b_baseline():
    """确认当前打开的是含 FR3 与 Table 的 Task04-B 基线 Stage。"""
    required_paths = [
        "/World/fr3",
        "/World/fr3/fr3_hand",
        "/World/Table",
    ]

    for path in required_paths:
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(
                "未找到 Task04-B 基线 Prim："
                f"{path}。请先打开含 FR3 和 Table 的已验证 Stage。"
            )

    table_prim = stage.GetPrimAtPath("/World/Table")
    if not table_prim.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError(
            "Task04-B Table 缺少 Collider；不在 Task05-A 中重建桌面。"
        )


def disable_visual_and_collision(root_path):
    """保留 Franka articulation/joint，但隐藏原 hand/finger 的实体。"""
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def create_dynamic_cube(path, xyz, color):
    """创建待抓取 A：Dynamic Rigid Body + Collider，质量为 0.2 kg。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(CUBE_SIZE)
    cube.CreateDisplayColorAttr([color])

    UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*xyz))

    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(cube.GetPrim())
    mass_api.CreateMassAttr(CUBE_MASS)


def create_static_obstacle(path, xyz, color):
    """创建 B/C：仅 Collider，作为不会被碰撞推动的固定障碍物。"""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(CUBE_SIZE)
    cube.CreateDisplayColorAttr([color])

    UsdGeom.Xformable(cube.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*xyz))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


# ============================================================
# 1. 校验并保留 Task04-B 已验证的 FR3 / Table 基线
# ============================================================

require_task04b_baseline()

# ============================================================
# 2. 清理 Task01 / Task03 / Task04 / 旧 Task05 遗留物，保证可重复运行
# ============================================================

for path in [
    "/World/PickCube",
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
]:
    remove_if_exists(path)

for index in range(1, 10):
    remove_if_exists(f"/World/Cube{index}")

remove_legacy_task05_objects()

# ============================================================
# 3. 复用 Task04-B 的顶部 suction_tool 构造
# ============================================================

disable_visual_and_collision("/World/fr3/fr3_hand/visuals")
disable_visual_and_collision("/World/fr3/fr3_hand/collisions")
disable_visual_and_collision("/World/fr3/fr3_leftfinger")
disable_visual_and_collision("/World/fr3/fr3_rightfinger")

hand_path = "/World/fr3/fr3_hand"
tool_path = hand_path + "/suction_tool"
remove_if_exists(tool_path)

UsdGeom.Xform.Define(stage, tool_path)

# cup 外侧接触面位于 hand 局部 +Z 的 SUCTION_TCP_Z。
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
# 4. 创建 Task03 同参数 B-A-C
# ============================================================

create_dynamic_cube("/World/BoxA", A_INITIAL_POSE, A_COLOR)
create_static_obstacle("/World/BoxB", B_POSE, B_COLOR)
create_static_obstacle("/World/BoxC", C_POSE, C_COLOR)

# ============================================================
# 输出
# ============================================================

print("")
print("====================================================")
print("Task05 B-A-C suction scene ready")
print(
    "A initial pose = "
    f"({A_INITIAL_POSE[0]:.3f}, {A_INITIAL_POSE[1]:.3f}, {A_INITIAL_POSE[2]:.3f})"
)
print(
    "A target pose = "
    f"({A_TARGET_POSE[0]:.3f}, {A_TARGET_POSE[1]:.3f}, {A_TARGET_POSE[2]:.3f})"
)
print(f"B pose = ({B_POSE[0]:.3f}, {B_POSE[1]:.3f}, {B_POSE[2]:.3f})")
print(f"C pose = ({C_POSE[0]:.3f}, {C_POSE[1]:.3f}, {C_POSE[2]:.3f})")
print(f"B/C inner gap = {BC_INNER_GAP_MM} mm")
print(f"A width = {A_WIDTH_MM} mm")
print(f"Suction TCP local Z = {SUCTION_TCP_Z:.4f} m")
print("BoxA = Dynamic Rigid Body + Collider, mass = 0.2 kg")
print("BoxB / BoxC = fixed Collider obstacles")
print("====================================================")
