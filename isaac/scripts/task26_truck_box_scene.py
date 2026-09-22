# Task26：车厢三面围墙 + 预推入位（第一层 4 件）的侧面吸盘场景。
#
# Isaac Sim 4.5 Script Editor 中，Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task26_truck_box_scene.py").read())
#
# 与 Task24/Task25 的区别有两处：
#   1. 车厢（TruckBox）：三面围墙 + 顶上无墙，装料口按用户指认开在 **-X** 面；
#   2. 导轨（rail）：两个 FR3 各自一条 **X 向**导轨，由 bridge 通过
#      /task26/{side}/rail_command 平移基座（本脚本只建导轨的视觉件与静止位）。
# 供料方式沿用 Task25：4 件 Cube 一次性创建为真实 Dynamic Rigid Body，初始停在
# 工作区外的休眠位并关闭重力，由 task26_truck_box_bridge.py 按 ROS 命令唤醒。
# 吸附几何、工具、桌面与已验收的基座高度基线都不改动。

import json
import math
import os

import omni.client
import omni.kit.app
import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    from isaacsim.storage.native import get_assets_root_path
except Exception as exc:
    raise RuntimeError("请在 Isaac Sim 4.5 中运行 Task26 场景脚本。") from exc


LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
GRAPH_PATH = "/World/Task26ROSGraph"
PHYSICS_PATH = "/physicsScene"
TASK_ROOT = "/World/Task26"
SUPPLY_ROOT = f"{TASK_ROOT}/Supply"
MARKER_ROOT = f"{TASK_ROOT}/TargetMarkers"
MATERIAL_PATH = f"{TASK_ROOT}/HighFrictionMaterial"

# 基座静止位必须与 MoveIt URDF（fr3_dual_side_suction_description，Task24/25/26 共用）
# 的 world_to_*_fr3 固定关节一致：x=0.650、y=±0.60。执行器按 URDF 的 0.65 规划，
# 场景若把基座放成别的 x 会让所有 TCP 目标整体错位。
# 导轨只在此静止位两侧 ±0.20 内平移；执行器目前不移动它（规划按固定基座 0.65）。
LEFT_BASE = (0.65, -0.60, 0.00)
RIGHT_BASE = (0.65, +0.60, 0.00)

# Task24-H 的官方工作高度基线：桌面顶面 0.200 m，底层侧吸接触中心 z=0.261 m。
TABLE_CENTER = (0.55, 0.00, 0.100)
TABLE_SIZE = (1.50, 0.80, 0.200)
TABLE_TOP_Z = 0.200

FR3_OFFICIAL_START_Q_RAD = (
    0.0,
    -math.pi / 4.0,
    0.0,
    -3.0 * math.pi / 4.0,
    0.0,
    math.pi / 2.0,
    math.pi / 4.0,
)

CUBE_SIZE = 0.120
CUBE_HALF = 0.060
CUBE_MASS = 0.800
BOTTOM_Z = TABLE_TOP_Z + CUBE_HALF
UPPER_Z = BOTTOM_Z + CUBE_SIZE

# Task26：车厢第一层 4 件，分 2 批到料、每批 2 件（同一列）。
BATCH_SIZE = 2
BATCH_COUNT = 2
ACTIVE_CUBE_COUNT = BATCH_SIZE * BATCH_COUNT

# 供料槽：放在**中轴线 y=0** 上、靠近基座（x=0.50 / 0.35），两臂对称可达。
# 为什么不放在 ±Y 或更西：双臂共同夹取要求两臂都能到同一件。y=0 时两臂 TCP
# 各偏 0.539 m（对称）；实测放到 (0.28, ±0.10) 时右臂要到 0.74 m 斜向长臂，
# RRT 8 候选全部超时。x=0.50/0.35 把最大伸展压到约 0.60 m。
SLOT_A = (0.500, 0.000, BOTTOM_Z)
SLOT_B = (0.350, 0.000, BOTTOM_Z)

# 休眠位：桌面下方 5 m，且已关闭重力。被瞬移唤醒前不可能挡住任何通道。
PARK_Z = -5.000
PARK_X0 = -0.600
PARK_X_PITCH = 0.200
PARK_X_COLUMNS = 4
PARK_Y = (-0.300, +0.300)

# ------------------------------------------------------------------ 车厢
#
# Task26 车厢：三面围墙 + 顶上无墙。装料口按用户 2026-09-21 的指认开在 **-X** 面，
# 机械臂从 -X 侧推入、从上方进出。底板直接沿用桌面顶面（Cube 中心保持已验收的
# z=0.260，不引入新的高度变量）。
#
# 开口放在 -X 的直接好处：推入方向变成 **+X**，预推位落在车厢**西侧空地**上，
# 不再出现「预推位紧贴车厢口、另一臂前臂从车厢上方扫过」的冲突，因此墙高可以
# 恢复到 0.250 m（两层 Cube 共 0.240 m，墙必须高过它才能约束第二层）。
#
# 内净空取 0.250 = 2 x 0.120 货物外廓 + 两侧各 5 mm。零净空（内腔正好 0.240）会
# 让 Cube 与墙、Cube 与 Cube 直接贴合，落位时必然擦碰。车厢在 Y 上**居中**（对称轴
# y=0，两臂基座 ±0.60 到两排等距），X 上放在桌面东侧（+X）：
#   * +X 墙内表面 0.910，深格 Cube 外表面 0.900 -> 10 mm；
#   * -Y 墙内表面 -0.125、+Y 墙内表面 +0.125，两排外表面各留 5 mm。
#
# 墙高 0.150（顶面 0.35）：**必须让开推入臂的前臂**，这是硬约束。实测推入时前臂
# link5 的下沿会扫到 z≈0.40（link 原点 0.447），所以墙顶必须低于 0.40：
#   * 0.250 高（顶面 0.45）-> 顶到墙顶沿 0.420 mm，被 FCL 拒绝；
#   * 0.200 高（顶面 0.40）-> 仍擦 0.014 mm，被 FCL 拒绝；
#   * 0.150 高（顶面 0.35）-> 前臂净空约 50 mm，通过。
# 代价：墙顶只比第一层 Cube 顶面（0.32）高 30 mm，是个"矮挡边"。第二层需要更高的
# 墙，与本条前臂净空直接冲突——做第二层时必须换推入方式（例如让 X 导轨跟随推入，
# 使前臂不再扫过侧墙上方），届时单独设计。
# 车厢整体 +X 让位：为"推入臂改走滑轨 +X"腾出净空。滑轨静止位 0.650、行程上限已
# 提到 1.050，推入时基座要走到 0.950，原先车厢 -X 装料口 (0.790) 会挡住前臂。
# 车厢、格位、预推位**同步**平移，推入行程保持 0.300 m 不变。
TRUCK_SHIFT_X = 0.150
BOX_INTERIOR_X = (0.810 + TRUCK_SHIFT_X, 1.060 + TRUCK_SHIFT_X)   # 深 0.250：2 格
BOX_INTERIOR_Y = (-0.125, 0.125)  # 宽 0.250：2 排，排心 y = +0.065 / -0.065（居中）
BOX_WALL_THICKNESS = 0.020
BOX_WALL_HEIGHT = 0.150
CELL_SHALLOW_X = 0.870 + TRUCK_SHIFT_X
CELL_DEEP_X = 0.990 + TRUCK_SHIFT_X
ROW_Y = (+0.065, -0.065)
PRE_PUSH_X = 0.690 + TRUCK_SHIFT_X   # 装料口外 120 mm 的预推位（纯 +X 推入）

# 装填顺序：每排**先推深格、再推浅格**——浅格先落会挡住通往深格的直推通道。
TASKS = (
    {"cube": 1, "row": 0, "depth": "deep", "cell": (CELL_DEEP_X, ROW_Y[0], BOTTOM_Z)},
    {"cube": 2, "row": 0, "depth": "shallow", "cell": (CELL_SHALLOW_X, ROW_Y[0], BOTTOM_Z)},
    {"cube": 3, "row": 1, "depth": "deep", "cell": (CELL_DEEP_X, ROW_Y[1], BOTTOM_Z)},
    {"cube": 4, "row": 1, "depth": "shallow", "cell": (CELL_SHALLOW_X, ROW_Y[1], BOTTOM_Z)},
)
PRE_PUSH = tuple((PRE_PUSH_X, item["cell"][1], BOTTOM_Z) for item in TASKS)

# ---------------------------------------------------------------- 导轨（第七轴）
#
# 用户要求：**导轨沿 ±X 轴**，两个机械臂各一条，基座沿 X 平移；车厢放到 +X、Y 向居中。
# 导轨沿 X 的正好处：推入方向也是 +X，第二层或更深的格位不再需要极限拉伸。
# 左/右臂各一条、行程相同（[0.45, 0.85]，即静止位 0.65 两侧 ±0.20）。
# 静止位 0.65 必须保持不变：那是 URDF 里 world_to_*_fr3 的固定关节值，也是执行器
# 规划所依据的基座位置。
#
# 实现方式（不改 FR3 资产、不动任何已验收的关节序）：
#   * 每条导轨由一组**纯视觉**件组成（两条导轨条 + 两端挡块 + 一块滑台），不加
#     CollisionAPI、不参与物理，与格位/预推位标记同级；
#   * 机械臂仍是 /World 的直接子节点（路径不变），由 bridge 通过
#     /task26/{side}/rail_command 整体平移：必须**同时**写 /World/{side}_fr3 的
#     USD 平移和 articulation 的世界位姿——Isaac 4.5 里这两者分别是渲染/物理两套
#     表示，只写一套会出现「物理动了但渲染不动 / 读数互相矛盾」（已实测）；
#   * 移动门槛：两臂吸盘都松开、且最近 1 s 内没有关节命令才接受移动；
#     到位判定与「分批到料」同一套语义（连续 RAIL_HOLD_SEC 落在容差内才算到位）。
#
# 滑台顶面正好在 z=0（机械臂基座底面），所以导轨**不改变基座高度**。
# 导轨沿 X 时整条导轨都在桌面两侧（y=±0.60，桌面 y∈[-0.40,0.40] 之外），不会像 Y 向
# 导轨那样伸到桌面下方；肩部外壳在 y 向突出、与 X 行程正交，所以也不受桌面内沿约束。
RAIL_ROOT = {side: f"/World/{side}_rail" for side in ("left", "right")}
RAIL_BASE_Y = {"left": LEFT_BASE[1], "right": RIGHT_BASE[1]}   # 固定 ±0.60
RAIL_REST_X = LEFT_BASE[0]                                     # 静止位 x = 0.45
RAIL_TRAVEL = {"left": (0.450, 1.050), "right": (0.450, 1.050)}  # X 行程 0.60 m
# 行程上限 0.850 -> 1.050：推入 0.300 m 改由滑轨承担（静止位 0.650 -> 0.950），
# 关节在推入段几乎不动，避免长臂姿态把推力需求顶到力矩上限。
RAIL_TRACK_LENGTH = 0.900      # 导轨条总长 = 行程 0.60 + 滑台 0.26 + 余量
# 导轨条以**行程中点**居中（原先借用静止位，行程放长后不再重合）。
RAIL_TRACK_CENTER_X = 0.5 * (RAIL_TRAVEL["left"][0] + RAIL_TRAVEL["left"][1])
RAIL_TRACK_WIDTH = 0.028
RAIL_TRACK_Y_OFFSET = 0.145    # 两条导轨条相对基座中心的 ±Y 偏移（沿 X 走轨）
RAIL_TRACK_HEIGHT = 0.030
RAIL_CARRIAGE_SIZE = (0.260, 0.260, 0.036)
RAIL_STOP_SIZE = (0.030, 0.318, 0.030)   # 挡块：X 向薄、跨两条导轨条的 Y 宽
RAIL_CARRIAGE_Z = -0.5 * RAIL_CARRIAGE_SIZE[2]   # 滑台中心：顶面 = 0.000
RAIL_TRACK_Z = -0.5 * RAIL_TRACK_HEIGHT
RAIL_COLOR_TRACK = (0.30, 0.32, 0.36)
RAIL_COLOR_CARRIAGE = (0.62, 0.66, 0.72)


# 已确认的固定 L 型阵列式侧面吸盘（与 Task24-E/H 同尺寸，不改动）。
VERTICAL_DROP_Z = 0.080
LATERAL_STANDOFF_Y = 0.130
VERTICAL_SUPPORT_WIDTH = 0.028
LATERAL_SUPPORT_WIDTH = 0.018
MANIFOLD_Y = 0.138
MANIFOLD_X_SIZE = 0.070
MANIFOLD_Y_SIZE = 0.014
MANIFOLD_Z_SIZE = 0.070
ARRAY_X_OFFSET = 0.024
ARRAY_Z_OFFSET = 0.024
CUP_RADIUS = 0.010
CUP_LENGTH = 0.010
CUP_CENTER_Y = 0.150
TCP_Y = 0.155


def _slot_pose(index):
    """Cube_01/02 为批 1；奇数排在槽 A（先取），偶数排在槽 B（后取）。"""
    return SLOT_A if index % 2 == 1 else SLOT_B


def _park_pose(index):
    zero_based = index - 1
    column = zero_based % PARK_X_COLUMNS
    row = zero_based // PARK_X_COLUMNS
    row = min(row, len(PARK_Y) - 1)
    return (
        PARK_X0 + PARK_X_PITCH * column,
        PARK_Y[row],
        PARK_Z,
    )


def _batch_of(index):
    return (index - 1) // BATCH_SIZE + 1


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")


def _remove(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def _require_stopped_timeline():
    import omni.timeline
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("请先停止 Timeline，再重建 Task26 场景。")


def _asset_url():
    root = get_assets_root_path()
    if not root:
        raise RuntimeError("无法获取 Isaac Assets 根目录。")
    candidates = (
        f"{root.rstrip('/')}/Robots/Franka/FR3/fr3.usd",
        f"{root.rstrip('/')}/Isaac/Robots/Franka/FR3/fr3.usd",
    )
    for candidate in candidates:
        result, _ = omni.client.stat(candidate)
        if result == omni.client.Result.OK:
            return candidate
    raise RuntimeError("找不到官方 FR3 USD。")


def _disable_visual_and_collision(root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Imageable):
            UsdGeom.Imageable(prim).MakeInvisible()
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)


def _add_fr3(root_path, base):
    robot = stage.DefinePrim(root_path, "Xform")
    robot.GetReferences().AddReference(_asset_url())
    xform = UsdGeom.Xformable(robot)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(opSuffix="task26_base").Set(Gf.Vec3d(*base))


# 本仓库声明的机器人是 franka_description 的 FR3（effort 87/12 N·m 与 FR3 规格一致），
# 但 _add_fr3() 引用的是 **Isaac 自带的 NVIDIA 转换资产** fr3.usd，它的关节量程比官方
# 每侧窄 0~9°。规划器按官方 URDF 规划，仿真却在更窄的位置硬挡——实测直接造成
# J6 追不上（残差 5.21°、速度 0.000，同时 J7 距上限仅 0.08°）。
# 这里把 7 个关节量程对齐本仓库 franka_description/robots/fr3/joint_limits.yaml，
# 让"规划的世界"和"物理的世界"用同一套量程。数值就是该文件里的 rad 值。
FR3_OFFICIAL_JOINT_LIMITS_RAD = {
    1: (-2.9007, 2.9007),
    2: (-1.8361, 1.8361),
    3: (-2.9007, 2.9007),
    4: (-3.0770, -0.1169),
    5: (-2.8763, 2.8763),
    6: (0.4398, 4.6216),
    7: (-3.0508, 3.0508),
}


def _apply_official_joint_limits():
    """把仿真侧关节量程对齐本仓库 franka_description 的官方 FR3 值。

    NVIDIA 的 fr3.usd 量程更窄（J1 每侧少 9.0°、J6 少 6.0°、J2/J4/J5/J7 少 2~4°），
    会造成"规划器能规划、物理到不了"的假失败。planning_only 查不出这一类问题
    （规划器用的是更宽的那套），必须读仿真侧属性来核对。
    """
    for root in (LEFT_ROOT, RIGHT_ROOT):
        for joint_index, (lower, upper) in FR3_OFFICIAL_JOINT_LIMITS_RAD.items():
            prim = stage.GetPrimAtPath(
                f"{root}/fr3_link{joint_index - 1}/fr3_joint{joint_index}")
            if not prim.IsValid():
                continue
            low_attr = prim.GetAttribute("physics:lowerLimit")
            high_attr = prim.GetAttribute("physics:upperLimit")
            if low_attr.IsValid():
                low_attr.Set(math.degrees(lower))
            if high_attr.IsValid():
                high_attr.Set(math.degrees(upper))
    print("[Task26] 关节量程已对齐本仓库官方 FR3（joint_limits.yaml）；"
          "NVIDIA fr3.usd 的窄量程是规划/物理不一致的来源，见 _apply_official_joint_limits 注释")


def _apply_diag_overrides():
    """诊断用：从 task26_diag.json 读取临时覆盖项（默认不存在=完全按官方参数）。

    支持（任一项为空 / <=0 表示不改该项）：
        {"wrist_max_force": <N·m>, "wrist_stiffness": <>, "wrist_damping": <>}
    只作用于 J5/J6/J7，用于把"腕关节 12 N·m 上限 vs 驱动刚度"这件事分离出来。
    **这不是验收配置，正式运行不得启用。**
    """
    path = "/home/ubuntu2004/WorkBuddy/2026-09-21-10-05-14/task26_diag.json"
    if not os.path.exists(path):
        return None
    with open(path) as handle:
        data = json.load(handle)
    overrides = {}
    for key, attribute in (
        ("wrist_max_force", "drive:angular:physics:maxForce"),
        ("wrist_stiffness", "drive:angular:physics:stiffness"),
        ("wrist_damping", "drive:angular:physics:damping"),
    ):
        try:
            value = float(data.get(key, 0.0))
        except (TypeError, ValueError):
            value = 0.0
        if value > 0.0:
            overrides[attribute] = value
    if not overrides:
        return None
    for root in (LEFT_ROOT, RIGHT_ROOT):
        for joint_index in (5, 6, 7):
            prim = stage.GetPrimAtPath(
                f"{root}/fr3_link{joint_index - 1}/fr3_joint{joint_index}")
            if not prim.IsValid():
                continue
            for attribute, value in overrides.items():
                attr = prim.GetAttribute(attribute)
                if attr.IsValid():
                    attr.Set(value)
    print("[诊断] 腕关节 J5/J6/J7 临时覆盖 %s（正式运行不得启用）"
          % {k.split(":")[-1]: v for k, v in overrides.items()})
    return overrides


def _set_fr3_official_start_target(root_path):
    """写入官方起始构型的 PhysX drive target（USD 角度单位）。"""
    for joint_index, position_rad in enumerate(FR3_OFFICIAL_START_Q_RAD, start=1):
        joint_path = (
            f"{root_path}/fr3_link{joint_index - 1}/fr3_joint{joint_index}"
        )
        joint_prim = stage.GetPrimAtPath(joint_path)
        if not joint_prim.IsValid():
            raise RuntimeError(f"FR3 缺少关节 Prim：{joint_path}")
        target = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        if not target.IsValid():
            raise RuntimeError(f"FR3 缺少角度 drive target：{joint_path}")
        target.Set(float(math.degrees(position_rad)))


def _make_material():
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(0.90)
    api.CreateDynamicFrictionAttr().Set(0.75)
    api.CreateRestitutionAttr().Set(0.0)
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _box(path, center, size, color, dynamic=False, metadata=None, gravity=True):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.Xformable(box.GetPrim())
    transform.AddTranslateOp(opSuffix="task26_pose").Set(Gf.Vec3d(*center))
    transform.AddScaleOp(opSuffix="task26_size").Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
        UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr().Set(CUBE_MASS)
        # 休眠件关闭重力，使其稳定停在桌下休眠位；到料时由 bridge 重新打开。
        body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(box.GetPrim())
        body_api.CreateDisableGravityAttr().Set(not gravity)
    if metadata:
        box.GetPrim().SetCustomDataByKey("task26_metadata", json.dumps(metadata))
    _bind(box.GetPrim(), _MATERIAL)
    return box


def _visual_box(path, center, size, color):
    """纯视觉方块：不加 CollisionAPI、不参与物理（与格位/预推位标记同级）。"""
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.Xformable(box.GetPrim())
    transform.AddTranslateOp(opSuffix="task26_pose").Set(Gf.Vec3d(*center))
    transform.AddScaleOp(opSuffix="task26_size").Set(Gf.Vec3f(*size))
    return box


def _build_tool(robot_root, branch_sign):
    hand_path = f"{robot_root}/fr3_hand"
    # 与 Task24 相同的安装 frame：工具固定在 fr3_link8，而不是带额外固定旋转的
    # fr3_hand。否则 MoveIt TCP 与物理 Cup 会产生数厘米偏差。
    mount_path = f"{robot_root}/fr3_link8"
    if not stage.GetPrimAtPath(hand_path).IsValid():
        raise RuntimeError(f"缺少 {hand_path}。")
    if not stage.GetPrimAtPath(mount_path).IsValid():
        raise RuntimeError(f"缺少 {mount_path}。")
    _disable_visual_and_collision(f"{hand_path}/visuals")
    _disable_visual_and_collision(f"{hand_path}/collisions")
    _disable_visual_and_collision(f"{robot_root}/fr3_leftfinger")
    _disable_visual_and_collision(f"{robot_root}/fr3_rightfinger")

    _remove(f"{hand_path}/side_suction_tool")
    _remove(f"{mount_path}/side_suction_tool")
    tool = f"{mount_path}/side_suction_tool"
    _remove(tool)
    UsdGeom.Xform.Define(stage, tool)

    vertical_support = UsdGeom.Cube.Define(stage, f"{tool}/vertical_support")
    vertical_support.CreateSizeAttr(1.0)
    vertical_xform = UsdGeom.Xformable(vertical_support.GetPrim())
    vertical_xform.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, 0.5 * VERTICAL_DROP_Z)
    )
    vertical_xform.AddScaleOp().Set(
        Gf.Vec3f(
            VERTICAL_SUPPORT_WIDTH,
            VERTICAL_SUPPORT_WIDTH,
            VERTICAL_DROP_Z,
        )
    )
    vertical_support.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.25, 0.30)])
    UsdPhysics.CollisionAPI.Apply(vertical_support.GetPrim())

    lateral_support = UsdGeom.Cube.Define(stage, f"{tool}/lateral_support")
    lateral_support.CreateSizeAttr(1.0)
    lateral_xform = UsdGeom.Xformable(lateral_support.GetPrim())
    lateral_xform.AddTranslateOp().Set(
        Gf.Vec3d(
            0.0,
            branch_sign * 0.5 * LATERAL_STANDOFF_Y,
            VERTICAL_DROP_Z,
        )
    )
    lateral_xform.AddScaleOp().Set(
        Gf.Vec3f(
            LATERAL_SUPPORT_WIDTH,
            LATERAL_STANDOFF_Y,
            LATERAL_SUPPORT_WIDTH,
        )
    )
    lateral_support.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.25, 0.30)])
    UsdPhysics.CollisionAPI.Apply(lateral_support.GetPrim())

    _box(
        f"{tool}/vacuum_manifold",
        (0.0, branch_sign * MANIFOLD_Y, VERTICAL_DROP_Z),
        (MANIFOLD_X_SIZE, MANIFOLD_Y_SIZE, MANIFOLD_Z_SIZE),
        (0.18, 0.22, 0.28),
    )
    for row, z_offset in enumerate((-ARRAY_Z_OFFSET, ARRAY_Z_OFFSET), start=1):
        for column, x_offset in enumerate((-ARRAY_X_OFFSET, ARRAY_X_OFFSET), start=1):
            cup = UsdGeom.Cylinder.Define(stage, f"{tool}/cup_r{row}_c{column}")
            cup.CreateAxisAttr(UsdGeom.Tokens.y)
            cup.CreateRadiusAttr(CUP_RADIUS)
            cup.CreateHeightAttr(CUP_LENGTH)
            UsdGeom.Xformable(cup.GetPrim()).AddTranslateOp().Set(
                Gf.Vec3d(
                    x_offset,
                    branch_sign * CUP_CENTER_Y,
                    VERTICAL_DROP_Z + z_offset,
                )
            )
            cup.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
            UsdPhysics.CollisionAPI.Apply(cup.GetPrim())

    tcp = UsdGeom.Xform.Define(stage, f"{tool}/side_suction_tcp")
    tcp_xform = UsdGeom.Xformable(tcp.GetPrim())
    tcp_xform.AddTranslateOp().Set(
        Gf.Vec3d(0.0, branch_sign * TCP_Y, VERTICAL_DROP_Z)
    )
    tcp_xform.AddRotateZOp().Set(branch_sign * 90.0)


def _build_objects():
    # 同一 GUI Session 若曾运行其它 Task，遗留刚体会参与 PhysX，遗留的 ROS
    # ActionGraph 还会重复发布 /joint_states 并重复写 joint command。这里只清理
    # 旧任务根与旧 ROS 图，不触碰 FR3 / Table。
    _remove("/World/Task23")
    _remove("/World/Task24")
    _remove(TASK_ROOT)
    for graph in (
        "/World/ActionGraph",
        "/ActionGraph",
        "/World/Task06ROSGraph",
        "/World/Task11SharedBoxROSGraph",
        "/World/Task12SharedBoxROSGraph",
        "/World/Task20RuntimeROSGraph",
        "/World/Task23GripperROSGraph",
        "/World/Task24SideSuctionROSGraph",
        "/World/Task26ROSGraph",
    ):
        _remove(graph)

    UsdGeom.Xform.Define(stage, TASK_ROOT)
    task_root = stage.GetPrimAtPath(TASK_ROOT)
    task_root.SetCustomDataByKey("task26_active_cube_count", ACTIVE_CUBE_COUNT)
    task_root.SetCustomDataByKey("task26_batch_size", BATCH_SIZE)
    task_root.SetCustomDataByKey("task26_batch_count", BATCH_COUNT)
    task_root.SetCustomDataByKey("task26_slot_a", json.dumps(list(SLOT_A)))
    task_root.SetCustomDataByKey("task26_slot_b", json.dumps(list(SLOT_B)))
    task_root.SetCustomDataByKey(
        "task26_feed_settle_offset_z", 0.002
    )
    task_root.SetCustomDataByKey("task26_box_interior_x", json.dumps(list(BOX_INTERIOR_X)))
    task_root.SetCustomDataByKey("task26_box_interior_y", json.dumps(list(BOX_INTERIOR_Y)))
    task_root.SetCustomDataByKey("task26_box_wall_thickness", BOX_WALL_THICKNESS)
    task_root.SetCustomDataByKey("task26_box_wall_height", BOX_WALL_HEIGHT)
    task_root.SetCustomDataByKey("task26_box_opening", "-X")
    task_root.SetCustomDataByKey("task26_push_axis", "+X")
    task_root.SetCustomDataByKey("task26_rail_axis", "X")
    task_root.SetCustomDataByKey("task26_rail_base_y", json.dumps(RAIL_BASE_Y))
    task_root.SetCustomDataByKey("task26_rail_rest_x", RAIL_REST_X)
    task_root.SetCustomDataByKey("task26_rail_travel", json.dumps(RAIL_TRAVEL))
    task_root.SetCustomDataByKey("task26_rail_carriage_z", RAIL_CARRIAGE_Z)
    task_root.SetCustomDataByKey(
        "task26_cells", json.dumps([list(item["cell"]) for item in TASKS]))
    task_root.SetCustomDataByKey(
        "task26_pre_push", json.dumps([list(pose) for pose in PRE_PUSH]))
    UsdGeom.Xform.Define(stage, SUPPLY_ROOT)
    UsdGeom.Xform.Define(stage, MARKER_ROOT)

    # 车厢：三面静态围墙（+X、+Y、-Y），装料口开在 -X，顶部无墙；底板就是桌面。
    x0, x1 = BOX_INTERIOR_X
    y0, y1 = BOX_INTERIOR_Y
    thickness = BOX_WALL_THICKNESS
    height = BOX_WALL_HEIGHT
    wall_z = TABLE_TOP_Z + height * 0.5
    box_root = f"{TASK_ROOT}/TruckBox"
    UsdGeom.Xform.Define(stage, box_root)
    # +X 墙（深端）
    _box(f"{box_root}/WallDeep",
         (x1 + thickness * 0.5, (y0 + y1) * 0.5, wall_z),
         (thickness, (y1 - y0) + 2.0 * thickness, height), (0.32, 0.40, 0.55), False,
         {"role": "truck_box_wall"})
    # -Y 墙
    _box(f"{box_root}/WallMinusY",
         ((x0 + x1) * 0.5, y0 - thickness * 0.5, wall_z),
         ((x1 - x0) + 2.0 * thickness, thickness, height), (0.32, 0.40, 0.55), False,
         {"role": "truck_box_wall"})
    # +Y 墙
    _box(f"{box_root}/WallPlusY",
         ((x0 + x1) * 0.5, y1 + thickness * 0.5, wall_z),
         ((x1 - x0) + 2.0 * thickness, thickness, height), (0.32, 0.40, 0.55), False,
         {"role": "truck_box_wall"})

    # 4 件全部是真实 Dynamic Rigid Body：到料后立即进入 PhysX，并由执行器按
    # Ground Truth 加入 MoveIt Planning Scene。休眠件不进入任何规划场景。
    for index in range(1, ACTIVE_CUBE_COUNT + 1):
        path = f"{SUPPLY_ROOT}/Cube_{index:02d}"
        _box(
            path, _park_pose(index),
            (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), (0.92, 0.42, 0.18), True,
            {
                "id": f"task26_cube_{index:02d}",
                "role": "truck_box_cube",
                "batch": _batch_of(index),
                "slot": "A" if index % 2 == 1 else "B",
                "slot_pose": list(_slot_pose(index)),
                "park_pose": list(_park_pose(index)),
                "row": TASKS[index - 1]["row"],
                "depth": TASKS[index - 1]["depth"],
                "push_axis": "+X",
                "pre_push_pose": list(PRE_PUSH[index - 1]),
                "cell_pose": list(TASKS[index - 1]["cell"]),
                "mass": CUBE_MASS,
            },
            gravity=False,
        )
    # 4 个格位 + 4 个预推位均为可视标记，不创建 Collider 或影响物理。
    for index, pose in enumerate([item["cell"] for item in TASKS], start=1):
        marker = UsdGeom.Cube.Define(stage, f"{MARKER_ROOT}/Cell_{index:02d}")
        marker.CreateSizeAttr(1.0)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.80, 0.25)])
        xform = UsdGeom.Xformable(marker.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*pose))
        xform.AddScaleOp().Set(Gf.Vec3f(0.026, 0.026, 0.002))
    for index, pose in enumerate(PRE_PUSH, start=1):
        marker = UsdGeom.Cube.Define(stage, f"{MARKER_ROOT}/PrePush_{index:02d}")
        marker.CreateSizeAttr(1.0)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.90, 0.75, 0.15)])
        xform = UsdGeom.Xformable(marker.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*pose))
        xform.AddScaleOp().Set(Gf.Vec3f(0.026, 0.026, 0.002))


def _build_ros_graph():
    import omni.graph.core as og
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("LeftPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("RightPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("Context.outputs:context", "Clock.inputs:context"),
                ("Tick.outputs:tick", "LeftPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "LeftPub.inputs:timeStamp"),
                ("Context.outputs:context", "LeftPub.inputs:context"),
                ("Tick.outputs:tick", "RightPub.inputs:execIn"),
                ("Time.outputs:simulationTime", "RightPub.inputs:timeStamp"),
                ("Context.outputs:context", "RightPub.inputs:context"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Time.inputs:resetOnStop", True),
                ("Clock.inputs:topicName", "/clock"),
                ("LeftPub.inputs:targetPrim", LEFT_ROOT),
                ("LeftPub.inputs:topicName", "/left/joint_states"),
                ("RightPub.inputs:targetPrim", RIGHT_ROOT),
                ("RightPub.inputs:topicName", "/right/joint_states"),
            ],
        },
    )


_require_stopped_timeline()
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
UsdGeom.Xform.Define(stage, "/World")
physics = stage.GetPrimAtPath(PHYSICS_PATH)
if not physics.IsValid():
    physics = UsdPhysics.Scene.Define(stage, PHYSICS_PATH).GetPrim()
scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics)
scene_api.CreateEnableCCDAttr().Set(True)
scene_api.CreateEnableStabilizationAttr().Set(True)
scene_api.CreateSolverTypeAttr().Set("TGS")
_MATERIAL = _make_material()

for root in (LEFT_ROOT, RIGHT_ROOT):
    _remove(root)
_remove(TABLE_PATH)
_box(TABLE_PATH, TABLE_CENTER, TABLE_SIZE, (0.0, 0.65, 0.85), False, {"role": "table"})
# 导轨：每个机械臂一条 Y 向导轨。导轨件是纯视觉的；机械臂本身仍是 /World 的
# 直接子节点（/World/{left,right}_fr3，路径不变），由 bridge 平移它的基座变换
# 来"走轨"，滑台（carriage）随之同步移动。
for side, root in RAIL_ROOT.items():
    _remove(root)
    rail = UsdGeom.Xform.Define(stage, root)
    UsdGeom.Xformable(rail.GetPrim()).ClearXformOpOrder()
    base_y = RAIL_BASE_Y[side]
    for index, sign in enumerate((-1.0, +1.0), start=1):
        _visual_box(
            f"{root}/track_{index}",
            (RAIL_TRACK_CENTER_X, base_y + sign * RAIL_TRACK_Y_OFFSET, RAIL_TRACK_Z),
            (RAIL_TRACK_LENGTH, RAIL_TRACK_WIDTH, RAIL_TRACK_HEIGHT),
            RAIL_COLOR_TRACK,
        )
    for index, sign in enumerate((-1.0, +1.0), start=1):
        _visual_box(
            f"{root}/stop_{index}",
            (RAIL_TRACK_CENTER_X + sign * (0.5 * RAIL_TRACK_LENGTH + 0.015), base_y, RAIL_TRACK_Z),
            RAIL_STOP_SIZE,
            RAIL_COLOR_TRACK,
        )
    _visual_box(
        f"{root}/carriage",
        (RAIL_REST_X, base_y, RAIL_CARRIAGE_Z),
        RAIL_CARRIAGE_SIZE,
        RAIL_COLOR_CARRIAGE,
    )
_add_fr3(LEFT_ROOT, LEFT_BASE)
_add_fr3(RIGHT_ROOT, RIGHT_BASE)
_apply_official_joint_limits()
_set_fr3_official_start_target(LEFT_ROOT)
_set_fr3_official_start_target(RIGHT_ROOT)
_apply_diag_overrides()
_build_tool(LEFT_ROOT, -1.0)
_build_tool(RIGHT_ROOT, +1.0)
_build_objects()
_build_ros_graph()

print("\n====================================================")
print("Task26 truck-box push-in scene ready")
print(
    "tools: fixed 2x2 side-suction array, short L support "
    "(80 mm down + 130 mm side), TCP offset=0.155 m"
)
print("truck box: interior x=%s y=%s, walls %.3f m thick x %.3f m high" %
      (BOX_INTERIOR_X, BOX_INTERIOR_Y, BOX_WALL_THICKNESS, BOX_WALL_HEIGHT))
print("opening: -X side + top; push direction: +X")
print("table top z=%.3f m; FR3 official move_to_start targets are configured" % TABLE_TOP_Z)
print("first layer cells: %s" % (tuple(item["cell"] for item in TASKS),))
print("pre-push positions: %s" % (PRE_PUSH,))
print("rails (X axis, visual only): rest x=%.3f travel=%s; base y fixed %s" %
      (RAIL_REST_X, RAIL_TRAVEL["left"], (RAIL_BASE_Y["left"], RAIL_BASE_Y["right"])))
print("TRUCK SHIFT: 车厢/格位/预推位 X 平移 %+.3f m；滑轨行程 %s；静止位 %.3f"
      % (TRUCK_SHIFT_X, RAIL_TRAVEL["left"], RAIL_REST_X))
print("rails are moved by the bridge via /task26/{left,right}/rail_command")
print(
    "feed: %d batches x %d cubes; slot A=%s (picked first), slot B=%s; all cubes start parked at z=%.3f" %
    (BATCH_COUNT, BATCH_SIZE, SLOT_A, SLOT_B, PARK_Z)
)
print("PUB: /clock, /left/joint_states, /right/joint_states")
print("Next: Play, then run task26_truck_box_bridge.py")
print("====================================================")
