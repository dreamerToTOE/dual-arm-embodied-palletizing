# Task25-A：分批到料的侧面吸盘紧协调码垛场景（休眠区瞬移版）。
#
# Isaac Sim 4.5 Script Editor 中，Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task25_batched_feed_scene.py").read())
#
# 与 Task24 的区别只有供料方式：8 件 Cube 在场景中一次性创建为真实 Dynamic
# Rigid Body，但初始全部停放在工作区外的休眠位并关闭重力；哪一批出现、什么
# 时候出现由 task25_batched_feed_bridge.py 按 ROS 命令决定。吸附几何、工具、
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
    raise RuntimeError("请在 Isaac Sim 4.5 中运行 Task25 场景脚本。") from exc


LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
GRAPH_PATH = "/World/Task25ROSGraph"
PHYSICS_PATH = "/physicsScene"
TASK_ROOT = "/World/Task25"
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

# Task25：固定 4 批、每批 2 件，共 8 件。
BATCH_SIZE = 2
BATCH_COUNT = 4
ACTIVE_CUBE_COUNT = BATCH_SIZE * BATCH_COUNT

# 每批两件放在同一 y 行、沿 X 分隔的两个槽位。两件在 X 上完全错开 220 mm，
# 远大于 L 型阵列面板 70 mm 的 X 向包络，因此任何一件都不会占据另一臂进入
# “侧面中心吸附位”的 X 走廊。
#
# 槽 A 批内先取：负载段 COMMON_X_TRAVEL 在“与供料相同的 y”高度上沿 X 飞行，
# 先取靠目标侧的槽 A，后取槽 B 时其飞行通道上不再有未取供料件。
SLOT_A = (0.520, -0.070, BOTTOM_Z)
SLOT_B = (0.300, -0.070, BOTTOM_Z)

# 休眠位：桌面下方 5 m，且已关闭重力。它们不进入 MoveIt Planning Scene，
# 也不参与任何 FCL 采样；被瞬移唤醒前在物理上不可能挡住任何通道。
PARK_Z = -5.000
PARK_X0 = -0.600
PARK_X_PITCH = 0.200
PARK_X_COLUMNS = 4
PARK_Y = (-0.300, +0.300)

# 目标垛型：批 1/2 打远墙 x=0.820，批 3/4 打近墙 x=0.640。
#
# Task25-B：YZ 墙的 Y 向中心距由 240 mm 改为 300 mm。
# 判据（左臂把 Cube 放到 y=+pitch/2 的一格，邻件已在 y=-pitch/2）：
#   支架尾端（法兰侧）y = pitch/2 - 60 - 1 - 155 = pitch/2 - 216
#   邻件朝向落点的那一面 y = -pitch/2 + 60
#   要求支架尾端仍在邻件之外：pitch/2 - 216 >= -pitch/2 + 60  =>  pitch >= 276 mm
# 240 mm 时支架尾端在 -0.096 m，邻件面在 -0.060 m，会侵入 36 mm（实测在
# COMMON_DESCENT_TO_ENTRY 报 left_fr3_side_suction <-> 邻件碰撞）。
# 取 300 mm：支架尾端 -0.066 m 对邻件面 -0.090 m，余量 24 mm。
TARGETS = (
    (0.820, -0.150, BOTTOM_Z),
    (0.820, +0.150, BOTTOM_Z),
    (0.820, -0.150, UPPER_Z),
    (0.820, +0.150, UPPER_Z),
    (0.640, -0.150, BOTTOM_Z),
    (0.640, +0.150, BOTTOM_Z),
    (0.640, -0.150, UPPER_Z),
    (0.640, +0.150, UPPER_Z),
)

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
        raise RuntimeError("请先停止 Timeline，再重建 Task25 场景。")


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
    xform.AddTranslateOp(opSuffix="task25_base").Set(Gf.Vec3d(*base))


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
    transform.AddTranslateOp(opSuffix="task25_pose").Set(Gf.Vec3d(*center))
    transform.AddScaleOp(opSuffix="task25_size").Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
        UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr().Set(CUBE_MASS)
        # 休眠件关闭重力，使其稳定停在桌下休眠位；到料时由 bridge 重新打开。
        body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(box.GetPrim())
        body_api.CreateDisableGravityAttr().Set(not gravity)
    if metadata:
        box.GetPrim().SetCustomDataByKey("task25_metadata", json.dumps(metadata))
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
        "/World/Task25ROSGraph",
    ):
        _remove(graph)

    UsdGeom.Xform.Define(stage, TASK_ROOT)
    task_root = stage.GetPrimAtPath(TASK_ROOT)
    task_root.SetCustomDataByKey("task25_active_cube_count", ACTIVE_CUBE_COUNT)
    task_root.SetCustomDataByKey("task25_batch_size", BATCH_SIZE)
    task_root.SetCustomDataByKey("task25_batch_count", BATCH_COUNT)
    task_root.SetCustomDataByKey("task25_slot_a", json.dumps(list(SLOT_A)))
    task_root.SetCustomDataByKey("task25_slot_b", json.dumps(list(SLOT_B)))
    task_root.SetCustomDataByKey(
        "task25_feed_settle_offset_z", 0.002
    )
    UsdGeom.Xform.Define(stage, SUPPLY_ROOT)
    UsdGeom.Xform.Define(stage, MARKER_ROOT)

    # 8 件全部是真实 Dynamic Rigid Body：到料后立即进入 PhysX，并由执行器按
    # Ground Truth 加入 MoveIt Planning Scene。休眠件不进入任何规划场景。
    for index in range(1, ACTIVE_CUBE_COUNT + 1):
        path = f"{SUPPLY_ROOT}/Cube_{index:02d}"
        _box(
            path, _park_pose(index),
            (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), (0.92, 0.42, 0.18), True,
            {
                "id": f"task25_cube_{index:02d}",
                "role": "batched_feed",
                "batch": _batch_of(index),
                "slot": "A" if index % 2 == 1 else "B",
                "slot_pose": list(_slot_pose(index)),
                "park_pose": list(_park_pose(index)),
                "mass": CUBE_MASS,
            },
            gravity=False,
        )
    # 八个目标均仅为可视标记，不创建 Collider 或影响物理。
    for index, pose in enumerate(TARGETS[:ACTIVE_CUBE_COUNT], start=1):
        marker = UsdGeom.Cube.Define(stage, f"{MARKER_ROOT}/Target_{index:02d}")
        marker.CreateSizeAttr(1.0)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.80, 0.25)])
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
print("Task25 batched-feed side-suction scene ready")
print(
    "tools: fixed 2x2 side-suction array, short L support "
    "(80 mm down + 130 mm side), TCP offset=0.155 m"
)
print(
    "batched feed: %d batches x %d cubes; slot A=%s (picked first), slot B=%s" %
    (BATCH_COUNT, BATCH_SIZE, SLOT_A, SLOT_B)
)
print(
    "park: z=%.3f m with gravity disabled; ALL Cube_01 ... Cube_%02d start parked" %
    (PARK_Z, ACTIVE_CUBE_COUNT)
)
print("table top z=%.3f m; FR3 official move_to_start targets are configured" % TABLE_TOP_Z)
print("target: far YZ wall x=0.820 then near YZ wall x=0.640, 2x2 each")
print("PUB: /clock, /left/joint_states, /right/joint_states")
print("Next: Play, then run task25_batched_feed_bridge.py")
print("====================================================")
