# Task06-A：双 FR3 + 双紧凑吸盘 Isaac 场景
# Isaac Sim 4.5 Script Editor 中、Timeline 停止时运行。
#
# 第二版基座布局（按实际场景反馈修正）：
#   - 两台 FR3 分别放在桌面两条长边外侧；
#   - 两台机械臂保持相同朝向 yaw = 0 deg；
#   - 基座中心沿桌面 X 中心线布置，降低初始本体碰撞概率；
#   - 两臂到桌面中心距离相同，保留中央公共工作区。
#
#   Table long sides: y = +/-0.40 m
#   Left  FR3: (0.55, -0.50, 0.00), yaw = 0 deg
#   Right FR3: (0.55, +0.50, 0.00), yaw = 0 deg
#   Base separation = 1.00 m
#
# 本 Task 只搭建场景：
#   - 不创建 ROS topic
#   - 不运行 MoveIt
#   - 不执行抓取
#   - 不修改 ActionGraph

import math

import omni.client
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

try:
    from isaacsim.storage.native import get_assets_root_path
except Exception as exc:
    raise RuntimeError(
        "无法导入 isaacsim.storage.native.get_assets_root_path；"
        "请确认使用 Isaac Sim 4.5。"
    ) from exc


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")

# ============================================================
# 参数
# ============================================================

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
LEGACY_ROOT = "/World/fr3"

# 桌面中心 x = 0.55 m，宽度 y = +/-0.40 m。
# 两台 FR3 分别放在两条长边外侧 0.10 m，且保持同一 yaw。
LEFT_BASE = (0.55, -0.50, 0.00)
RIGHT_BASE = (0.55, +0.50, 0.00)
LEFT_YAW_DEG = 0.0
RIGHT_YAW_DEG = 0.0

TABLE_PATH = "/World/Table"
TABLE_CENTER = (0.55, 0.00, 0.025)
TABLE_SIZE = (1.20, 0.80, 0.05)

# 与 Task04 / Task05 已验证 Isaac + MoveIt compact suction 一致。
SUCTION_TCP_Z = 0.1050
STEM_RADIUS = 0.006
STEM_HEIGHT = 0.099
CUP_RADIUS = 0.010
CUP_HEIGHT = 0.006

# Task06-A 不放箱体，只看双臂布局和工具模型。
OLD_TASK_OBJECTS = [
    "/World/PickCube",
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
]
OLD_TASK_OBJECTS += [f"/World/Cube{i}" for i in range(1, 10)]


# ============================================================
# 工具函数
# ============================================================

def remove_if_exists(path):
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)


def resolve_fr3_asset():
    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("无法获取 Isaac Assets 根目录。")

    root = assets_root.rstrip("/")
    candidates = [
        f"{root}/Robots/Franka/FR3/fr3.usd",
        f"{root}/Isaac/Robots/Franka/FR3/fr3.usd",
    ]

    for url in candidates:
        result, _ = omni.client.stat(url)
        if result == omni.client.Result.OK:
            return url

    raise RuntimeError(
        "找不到 FR3 官方 USD。尝试过：\n  - "
        + "\n  - ".join(candidates)
    )


def add_fr3_reference(root_path, asset_url, xyz, yaw_deg):
    prim = stage.DefinePrim(root_path, "Xform")
    prim.GetReferences().AddReference(asset_url)

    # 使用带 suffix 的本地 override，避免与资产内部已有 xformOp 同名。
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()

    translate = xformable.AddTranslateOp(opSuffix="task06_base")
    translate.Set(Gf.Vec3d(*xyz))

    rotate = xformable.AddRotateXYZOp(opSuffix="task06_base")
    rotate.Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))

    return prim


def disable_visual_and_collision(root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return

    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def create_compact_suction(robot_root):
    hand_path = f"{robot_root}/fr3_hand"
    hand = stage.GetPrimAtPath(hand_path)
    if not hand.IsValid():
        raise RuntimeError(
            f"{hand_path} 不存在。FR3 USD 内部层级与预期不一致。"
        )

    # 保留 hand articulation / joint 结构，只隐藏原 Hand/Finger 外观并关闭碰撞。
    disable_visual_and_collision(f"{hand_path}/visuals")
    disable_visual_and_collision(f"{hand_path}/collisions")
    disable_visual_and_collision(f"{robot_root}/fr3_leftfinger")
    disable_visual_and_collision(f"{robot_root}/fr3_rightfinger")

    tool_path = f"{hand_path}/suction_tool"
    remove_if_exists(tool_path)
    UsdGeom.Xform.Define(stage, tool_path)

    # stem：z = 0 ~ 0.099 m
    stem = UsdGeom.Cylinder.Define(stage, f"{tool_path}/stem")
    stem.CreateAxisAttr(UsdGeom.Tokens.z)
    stem.CreateRadiusAttr(STEM_RADIUS)
    stem.CreateHeightAttr(STEM_HEIGHT)
    UsdGeom.Xformable(stem.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, 0.5 * STEM_HEIGHT)
    )
    stem.CreateDisplayColorAttr([Gf.Vec3f(0.25, 0.25, 0.25)])
    UsdPhysics.CollisionAPI.Apply(stem.GetPrim())

    # cup：末端中心位于 0.102 m，底面/TCP 位于 0.105 m。
    cup_center_z = SUCTION_TCP_Z - 0.5 * CUP_HEIGHT
    cup = UsdGeom.Cylinder.Define(stage, f"{tool_path}/cup")
    cup.CreateAxisAttr(UsdGeom.Tokens.z)
    cup.CreateRadiusAttr(CUP_RADIUS)
    cup.CreateHeightAttr(CUP_HEIGHT)
    UsdGeom.Xformable(cup.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, cup_center_z)
    )
    cup.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
    UsdPhysics.CollisionAPI.Apply(cup.GetPrim())

    suction_tcp = UsdGeom.Xform.Define(stage, f"{tool_path}/suction_tcp")
    UsdGeom.Xformable(suction_tcp.GetPrim()).AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, SUCTION_TCP_Z)
    )


def world_translation(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
        Usd.TimeCode.Default()
    )
    p = tf.ExtractTranslation()
    return (float(p[0]), float(p[1]), float(p[2]))


# ============================================================
# 1. 基线检查
# ============================================================

if not stage.GetPrimAtPath(TABLE_PATH).IsValid():
    raise RuntimeError(
        f"缺少 {TABLE_PATH}。请先打开当前已验证基础 Stage。"
    )

fr3_asset = resolve_fr3_asset()

# ============================================================
# 2. 清理旧单臂 / Task 物体
# ============================================================

for path in OLD_TASK_OBJECTS:
    remove_if_exists(path)

# Task06-A 使用独立 top-level Prim；旧 /World/fr3 不继续保留。
remove_if_exists(LEFT_ROOT)
remove_if_exists(RIGHT_ROOT)
remove_if_exists(LEGACY_ROOT)

# ============================================================
# 3. 创建“长边两侧、同朝向”双 FR3
# ============================================================

add_fr3_reference(LEFT_ROOT, fr3_asset, LEFT_BASE, LEFT_YAW_DEG)
add_fr3_reference(RIGHT_ROOT, fr3_asset, RIGHT_BASE, RIGHT_YAW_DEG)

# ============================================================
# 4. 两端都挂 Task04/05 已验证紧凑吸盘
# ============================================================

create_compact_suction(LEFT_ROOT)
create_compact_suction(RIGHT_ROOT)

# ============================================================
# 5. 简单几何验收输出
# ============================================================

base_separation = math.dist(LEFT_BASE, RIGHT_BASE)
table_center_xy = (TABLE_CENTER[0], TABLE_CENTER[1])
left_center_distance = math.hypot(
    table_center_xy[0] - LEFT_BASE[0],
    table_center_xy[1] - LEFT_BASE[1],
)
right_center_distance = math.hypot(
    table_center_xy[0] - RIGHT_BASE[0],
    table_center_xy[1] - RIGHT_BASE[1],
)

table_half_y = 0.5 * TABLE_SIZE[1]
left_side_clearance = abs(LEFT_BASE[1]) - table_half_y
right_side_clearance = abs(RIGHT_BASE[1]) - table_half_y

left_world = world_translation(LEFT_ROOT)
right_world = world_translation(RIGHT_ROOT)

print("")
print("====================================================")
print("Task06-A 双 FR3 + 双紧凑吸盘场景创建完成")
print("布局：桌面两条长边各一台 FR3，两臂同朝向")
print(f"FR3 asset = {fr3_asset}")
print("")
print(
    "Left  FR3 base = "
    f"({LEFT_BASE[0]:.3f}, {LEFT_BASE[1]:.3f}, {LEFT_BASE[2]:.3f}), "
    f"yaw={LEFT_YAW_DEG:.1f} deg"
)
print(
    "Right FR3 base = "
    f"({RIGHT_BASE[0]:.3f}, {RIGHT_BASE[1]:.3f}, {RIGHT_BASE[2]:.3f}), "
    f"yaw={RIGHT_YAW_DEG:.1f} deg"
)
print(f"Base separation = {base_separation:.3f} m")
print("")
print(
    f"Table center = ({TABLE_CENTER[0]:.3f}, {TABLE_CENTER[1]:.3f}, "
    f"{TABLE_CENTER[2]:.3f})"
)
print(
    f"Table size   = ({TABLE_SIZE[0]:.3f}, {TABLE_SIZE[1]:.3f}, "
    f"{TABLE_SIZE[2]:.3f})"
)
print(f"Table long sides at y = +/-{table_half_y:.3f} m")
print(f"Left  base outside long side by = {left_side_clearance:.3f} m")
print(f"Right base outside long side by = {right_side_clearance:.3f} m")
print(f"Left  base -> table center XY distance = {left_center_distance:.3f} m")
print(f"Right base -> table center XY distance = {right_center_distance:.3f} m")
print("")
print(f"Left  root world = {left_world}")
print(f"Right root world = {right_world}")
print(f"Compact suction TCP local Z = {SUCTION_TCP_Z:.3f} m")
print("Compact suction max diameter = 20 mm")
print("")
print("Task06-A 当前只验收布局，不点击 Play 做控制。")
print("重点观察：")
print("1) 两台 FR3 是否分别位于桌面两条长边外侧；")
print("2) 两台机械臂是否保持同朝向；")
print("3) HOME 外观是否明显降低了本体碰撞风险；")
print("4) 桌面中央是否仍存在足够的双臂公共工作区；")
print("5) 左右吸盘外观是否均为细杆 + 20 mm cup。")
print("====================================================")
