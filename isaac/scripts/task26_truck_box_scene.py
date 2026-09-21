# Task24-A：分批到料的侧面吸盘紧协调码垛场景（休眠区瞬移版）。
#
# Isaac Sim 4.5 Script Editor 中，Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task26_batched_feed_scene.py").read())
#
# 与 Task24 的区别只有供料方式：8 件 Cube 在场景中一次性创建为真实 Dynamic
# Rigid Body，但初始全部停放在工作区外的休眠位并关闭重力；哪一批出现、什么
# 时候出现由 task26_batched_feed_bridge.py 按 ROS 命令决定。吸附几何、工具、
# 桌面、基座与目标垛型与 Task24 完全一致，不重新标定。

import json
import math

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

# 与 Task24-G 完全相同的固定基座：x=0.650 前移，沿 +/-Y 外扩到 1.20 m 间距。
LEFT_BASE = (0.65, -0.60, 0.00)
RIGHT_BASE = (0.65, +0.60, 0.00)

# Task24-H 的官方工作高度基线：桌面顶面 0.200 m，底层侧吸接触中心 z=0.261 m。
TABLE_CENTER = (0.55, 0.00, 0.100)
TABLE_SIZE = (1.20, 0.80, 0.200)
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

# 供料槽沿用 Task25 已验收的两个位置。
SLOT_A = (0.520, -0.070, BOTTOM_Z)
SLOT_B = (0.300, -0.070, BOTTOM_Z)

# 休眠位：桌面下方 5 m，且已关闭重力。被瞬移唤醒前不可能挡住任何通道。
PARK_Z = -5.000
PARK_X0 = -0.600
PARK_X_PITCH = 0.200
PARK_X_COLUMNS = 4
PARK_Y = (-0.300, +0.300)

# ------------------------------------------------------------------ 车厢
#
# Task26 车厢：三面围墙 + 顶上无墙。装料口开在 -Y，机械臂从上方进出。
# 底板直接沿用桌面顶面（Cube 中心保持已验收的 z=0.260，不引入新的高度变量）。
#
# 墙高 0.250 m 是按“至少两层”定的：两层 Cube 共 0.240 m，墙必须高过它才能约束第二层。
# 尺寸依据（L 型支架 TCP 相对法兰侧向偏置 155 mm）：
#   * 双臂紧协调放置要求 Cube 中心两侧各留 60+1+155 = 216 mm 通道，所以预推位
#     (y=-0.150) 到最近一格中心至少要 216+60 = 276 mm；
#   * 单臂在 Cube 中心高度推进不要求运输高度净空，因此格位可以比放置位更靠里。
# 车厢内净空按「机械臂前臂要能越过车厢口」定，而不是仅按货物外廓定。
# 实测：右臂在预推位做低位姿态时，前臂 link5/link6 扫到
# x≈0.61-0.73、y≈0.13-0.32、z≈0.45；因此侧墙内表面必须离它 50 mm 以上，
# 后墙也必须比它北 50 mm 以上。真实货车车厢本来就比货物宽。
BOX_INTERIOR_X = (0.560, 0.960)   # 2 列，列心 x = 0.700 / 0.820（货物 x∈[0.640,0.880]）
BOX_INTERIOR_Y = (0.090, 0.380)   # 2 排，排心 y = +0.150 / +0.270（货物 y∈[0.090,0.330]）
BOX_WALL_THICKNESS = 0.020
# 第一层墙高 0.150（顶面 z=0.350）。这不是随意取值：右臂在预推位做低位姿态时，
# 前臂 link5/link6 的 z 在 0.42-0.50 之间扫过车厢口，实测 0.250 高（顶面 0.450）
# 的任何宽度都会挡住它（纯 Z 下降的笛卡尔 fraction 掉到 0.61）。
# 0.150 高给出 70 mm 以上净空；第二层的推入同样净空（推入臂前臂在 z≈0.57）。
BOX_WALL_HEIGHT = 0.150
COLUMN_X = (0.700, 0.820)
ROW_Y_NEAR = +0.150               # 近排（后做）
ROW_Y_FAR = +0.270                # 远排（先做，见下方顺序说明）
PRE_PUSH_Y = -0.120               # 装料口外 210 mm
# 预推位的 y 由右臂可达性定：右臂要跨过桌面中线去够 Cube 的 +Y 面，
# 在运输高度 z=0.540 上，它的腕部位置 = cube_y + 216 mm。取 -0.150 时腕部落在
# -0.034，肩距 0.72 m 已贴到 0.70 m 的臂链边界，实测四种步长的笛卡尔抬升轨迹
# 全部失真（偏差 1.2 m 量级）而被拒绝；取 -0.120 后腕部回到 +0.096、肩距 0.58 m。
# 同时满足通道约束：预推位 + 216 mm 通道不得压到车厢最近一排。

# 每件 Cube：先到预推位，再被左臂沿 +Y 推入自己的格位。
# 顺序由几何强制：同一列内必须先远排后近排——左臂支架在 TCP 后方 155 mm，
# 推远排时会扫过 y ∈ [cube_y-0.216, cube_y-0.061]，若近排已落稳就会真实碰撞。
TASKS = (
    {"cube": 1, "column": 0, "row": "far",  "cell": (COLUMN_X[0], ROW_Y_FAR, BOTTOM_Z)},
    {"cube": 2, "column": 0, "row": "near", "cell": (COLUMN_X[0], ROW_Y_NEAR, BOTTOM_Z)},
    {"cube": 3, "column": 1, "row": "far",  "cell": (COLUMN_X[1], ROW_Y_FAR, BOTTOM_Z)},
    {"cube": 4, "column": 1, "row": "near", "cell": (COLUMN_X[1], ROW_Y_NEAR, BOTTOM_Z)},
)
PRE_PUSH = tuple((COLUMN_X[item["column"]], PRE_PUSH_Y, BOTTOM_Z) for item in TASKS)

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
    task_root.SetCustomDataByKey(
        "task26_cells", json.dumps([list(item["cell"]) for item in TASKS]))
    task_root.SetCustomDataByKey(
        "task26_pre_push", json.dumps([list(pose) for pose in PRE_PUSH]))
    UsdGeom.Xform.Define(stage, SUPPLY_ROOT)
    UsdGeom.Xform.Define(stage, MARKER_ROOT)

    # 车厢：三面静态围墙（左/右/后），装料口开在 -Y，顶部无墙；底板就是桌面。
    x0, x1 = BOX_INTERIOR_X
    y0, y1 = BOX_INTERIOR_Y
    thickness = BOX_WALL_THICKNESS
    height = BOX_WALL_HEIGHT
    wall_z = TABLE_TOP_Z + height * 0.5
    box_root = f"{TASK_ROOT}/TruckBox"
    UsdGeom.Xform.Define(stage, box_root)
    _box(f"{box_root}/WallLeft",
         (x0 - thickness * 0.5, (y0 + y1 + thickness) * 0.5, wall_z),
         (thickness, (y1 - y0) + thickness, height), (0.32, 0.40, 0.55), False,
         {"role": "truck_box_wall"})
    _box(f"{box_root}/WallRight",
         (x1 + thickness * 0.5, (y0 + y1 + thickness) * 0.5, wall_z),
         (thickness, (y1 - y0) + thickness, height), (0.32, 0.40, 0.55), False,
         {"role": "truck_box_wall"})
    _box(f"{box_root}/WallBack",
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
                "column": TASKS[index - 1]["column"],
                "row": TASKS[index - 1]["row"],
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
_add_fr3(LEFT_ROOT, LEFT_BASE)
_add_fr3(RIGHT_ROOT, RIGHT_BASE)
_set_fr3_official_start_target(LEFT_ROOT)
_set_fr3_official_start_target(RIGHT_ROOT)
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
print("truck box: interior x=%s y=%s, walls %.3f m thick x %.3f m high, open at -Y and top" %
      (BOX_INTERIOR_X, BOX_INTERIOR_Y, BOX_WALL_THICKNESS, BOX_WALL_HEIGHT))
print("first layer cells: %s" % (tuple(item["cell"] for item in TASKS),))
print("pre-push positions: %s" % (PRE_PUSH,))
print(
    "feed: %d batches x %d cubes; slot A=%s (picked first), slot B=%s; all cubes start parked at z=%.3f" %
    (BATCH_COUNT, BATCH_SIZE, SLOT_A, SLOT_B, PARK_Z)
)
print("table top z=%.3f m; FR3 official move_to_start targets are configured" % TABLE_TOP_Z)
print("PUB: /clock, /left/joint_states, /right/joint_states")
print("Next: Play, then run task26_truck_box_bridge.py")
print("====================================================")
