# Task24-A：固定阵列式侧面吸盘的离线紧协调码垛场景。
#
# Isaac Sim 4.5 Script Editor 中，Timeline 停止时运行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task24_side_suction_tight_scene.py").read())
#
# 该脚本不依赖 Task06 已打开的 USD：它重新建立两台官方 FR3、桌面、ROS 图、
# 8 个固定已知供料 Cube 与墙优先的离线目标标记。它不运行 MoveIt 或执行吸附。

import json
import math

import omni.client
import omni.kit.app
import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

try:
    from isaacsim.storage.native import get_assets_root_path
except Exception as exc:
    raise RuntimeError("请在 Isaac Sim 4.5 中运行 Task24 场景脚本。") from exc


LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
GRAPH_PATH = "/World/Task24SideSuctionROSGraph"
PHYSICS_PATH = "/physicsScene"
TASK_ROOT = "/World/Task24"
SUPPLY_ROOT = f"{TASK_ROOT}/Supply"
MARKER_ROOT = f"{TASK_ROOT}/TargetMarkers"
MATERIAL_PATH = f"{TASK_ROOT}/HighFrictionMaterial"

# Task24-D：加长侧向 standoff 后，保持目标墙 x=0.820 不变，但将独立 Task24
# 的双臂基座前移到 x=0.650。此前 x=0.550 时，共同横移末端让右臂 joint2/joint6
# 到达真实驱动力上限；前移 100 mm 令主要运输行程从 250 mm 降为 150 mm。
LEFT_BASE = (0.65, -0.50, 0.00)
RIGHT_BASE = (0.65, +0.50, 0.00)
TABLE_CENTER = (0.55, 0.00, 0.025)
TABLE_SIZE = (1.20, 0.80, 0.050)
TABLE_TOP_Z = 0.050

# Task24-B：所有物体由双侧吸盘紧协调搬运，Cube 线性尺寸按用户要求放大为
# 原先 30 mm 基线的 4 倍。质量使用 0.8 kg 的共同搬运基线，而不是按体积
# 64 倍放大到 6.4 kg；后者超出本轮“物理可行性”验证的 FR3 安全范围。
CUBE_SIZE = 0.120
CUBE_HALF = 0.060
CUBE_MASS = 0.800
BOTTOM_Z = TABLE_TOP_Z + CUBE_HALF
UPPER_Z = BOTTOM_Z + CUBE_SIZE

# Task24-C 单件隔离开关：首件已用于定位侧吸附接触误差。交付场景默认启用全部
# 8 个真实 Dynamic Cube；如需复现首件隔离对照，可临时设为 True，并同时以
# isolate_first_cube:=true 运行执行器。
ISOLATE_FIRST_CUBE = False

# Task24-E 外观样机的工具变体。默认保持已确认的 L 型侧吸阵列；
# task24_vertical_array_suction_scene.py 会注入 vertical_down_array，用于和
# 老师并列评审纯竖直下探、阵列向下的顶吸结构。
TASK24_TOOL_VARIANT = globals().get("TASK24_TOOL_VARIANT", "side_l_array")
if TASK24_TOOL_VARIANT not in ("side_l_array", "vertical_down_array"):
    raise RuntimeError(
        "TASK24_TOOL_VARIANT 只能为 side_l_array 或 vertical_down_array。"
    )

# Task24-E-A：固定 L 型阵列式侧面吸盘。
#
# 从 link8 先竖直向下，再水平向侧方伸出；这是固定支架，不含可动关节。
# 横向偏置把 FR3 腕部留在 Cube 外侧，避免纯竖杆使腕部贴近箱体和垛墙。
# 阵列面位于 local X-Z 平面，杯面法向沿 local Y；左右工具完全相同，仅以
# branch_sign 镜像安装。码垛时由末端姿态确保两块阵列面水平、相向朝内。
#
# 本次仅供 Isaac 外观评审；确认样式后再将对应 manifold/collision 与多杯
# 物理约束同步到 MoveIt/bridge。
VERTICAL_DROP_Z = 0.080
LATERAL_STANDOFF_Y = 0.130
# 竖直段承担主支撑，视觉与结构上加粗；横向段保持紧凑，保留垛墙附近的通道。
VERTICAL_SUPPORT_WIDTH = 0.028
LATERAL_SUPPORT_WIDTH = 0.018
MANIFOLD_Y = 0.138
MANIFOLD_X_SIZE = 0.090
MANIFOLD_Y_SIZE = 0.014
MANIFOLD_Z_SIZE = 0.090
ARRAY_X_OFFSET = 0.024
ARRAY_Z_OFFSET = 0.024
CUP_RADIUS = 0.010
CUP_LENGTH = 0.010
CUP_CENTER_Y = 0.150
TCP_Y = 0.155

# Task24-E-B：纯竖直下探的顶吸阵列。阵列板位于 local X-Y 平面，四个 Cup
# 沿 local +Z 指向下方；没有横向支臂，也不使用左右镜像的偏置。默认保留
# 145 mm 视觉对照杆；80 mm 对照入口会覆写该值。
VERTICAL_ARRAY_SUPPORT_LENGTH = float(
    globals().get("TASK24_VERTICAL_ARRAY_SUPPORT_LENGTH", 0.145)
)
if VERTICAL_ARRAY_SUPPORT_LENGTH <= 0.0:
    raise RuntimeError("TASK24_VERTICAL_ARRAY_SUPPORT_LENGTH 必须为正数。")
VERTICAL_ARRAY_PANEL_Z = VERTICAL_ARRAY_SUPPORT_LENGTH
# 14 mm 面板之后接 10 mm Cup；TCP 取 Cup 的接触端中心。
VERTICAL_ARRAY_CUP_CENTER_Z = VERTICAL_ARRAY_PANEL_Z + 0.012
VERTICAL_ARRAY_TCP_Z = VERTICAL_ARRAY_PANEL_Z + 0.017
VERTICAL_ARRAY_XY_OFFSET = 0.024

# 墙优先的离线固定垛型：先完整 YZ 墙 x=0.820，再完整 YZ 墙 x=0.640。
# 每一面墙按 y/z 的 2x2 顺序完成；Y/Z 单元中心间距 150 mm，使 120 mm Cube
# 间留有 30 mm 物理净空；两面墙的 X 间隔为 180 mm。
TARGETS = (
    (0.820, -0.075, BOTTOM_Z),
    (0.820, +0.075, BOTTOM_Z),
    (0.820, -0.075, UPPER_Z),
    (0.820, +0.075, UPPER_Z),
    (0.640, -0.075, BOTTOM_Z),
    (0.640, +0.075, BOTTOM_Z),
    (0.640, -0.075, UPPER_Z),
    (0.640, +0.075, UPPER_Z),
)

# 第一版无需选择器，供料起点和执行顺序固定且开放，给侧面吸盘预接触留出空间。
# 供料区与 x=0.650/0.700 的两面目标墙分离；它不是等待/灰色中转台，Cube
# 被共同搬到垛型上的 entry 位姿后只再作 +X 20 mm 的短推。不能把尚未抓取的
# Cube 放在目标墙旁，否则它会作为真实静态障碍物阻断共同下降。
SUPPLY_POSES = (
    # 首件位于两臂基座中间的 y=0 中线。Task24-D 将 source 与双臂基座统一到
    # x=0.650，减少到远侧墙的前伸；仍在同一张桌面上，未添加中转/等待区域。
    (0.650, 0.000, BOTTOM_Z),
    # 其余供料固定在桌面最左侧。共同侧吸时右臂工具的真实 sweep 会经过
    # x=0.25--0.45 的外侧空间，所以该区域不能放“暂未抓取”的动态 Cube。
    # 采用左边缘 x=0.010 的五格列和 x=0.170 的两格列，任意两 Cube 仍留有
    # 至少 40 mm 间隔。这里不是中转/等待区；每件仍从已知供料位直接共同搬至
    # 垛型 entry，并只在垛型上作短推与释放。
    (0.010, +0.320, BOTTOM_Z),
    (0.010, +0.160, BOTTOM_Z),
    (0.010, +0.000, BOTTOM_Z),
    (0.010, -0.160, BOTTOM_Z),
    (0.010, -0.320, BOTTOM_Z),
    (0.170, +0.240, BOTTOM_Z),
    (0.170, -0.240, BOTTOM_Z),
)

stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")


def _remove(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def _require_stopped_timeline():
    import omni.timeline
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("请先停止 Timeline，再重建 Task24 场景。")


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
    xform.AddTranslateOp(opSuffix="task24_base").Set(Gf.Vec3d(*base))


def _make_material():
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr().Set(0.90)
    api.CreateDynamicFrictionAttr().Set(0.75)
    api.CreateRestitutionAttr().Set(0.0)
    return material


def _bind(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _box(path, center, size, color, dynamic=False, metadata=None):
    box = UsdGeom.Cube.Define(stage, path)
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    transform = UsdGeom.Xformable(box.GetPrim())
    transform.AddTranslateOp(opSuffix="task24_pose").Set(Gf.Vec3d(*center))
    transform.AddScaleOp(opSuffix="task24_size").Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
        UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr().Set(CUBE_MASS)
    if metadata:
        box.GetPrim().SetCustomDataByKey("task24_metadata", json.dumps(metadata))
    _bind(box.GetPrim(), _MATERIAL)
    return box


def _build_tool(robot_root, branch_sign):
    hand_path = f"{robot_root}/fr3_hand"
    # MoveIt 的 Task24 URDF 将当前 L 型工具固定在 fr3_link8；竖直顶吸版仅
    # 用于本轮 Isaac 外观评审，尚未绑定 MoveIt/bridge。
    # 必须在 Isaac 中使用同一个安装 frame：fr3_hand 在官方资产里带有
    # 额外的固定旋转，若把工具挂在它下面，MoveIt TCP 与物理 Cup 会产生
    # 数厘米的 X 向偏差，导致侧面 Surface Gripper 无法接触 Cube。
    mount_path = f"{robot_root}/fr3_link8"
    if not stage.GetPrimAtPath(hand_path).IsValid():
        raise RuntimeError(f"缺少 {hand_path}。")
    if not stage.GetPrimAtPath(mount_path).IsValid():
        raise RuntimeError(f"缺少 {mount_path}。")
    _disable_visual_and_collision(f"{hand_path}/visuals")
    _disable_visual_and_collision(f"{hand_path}/collisions")
    _disable_visual_and_collision(f"{robot_root}/fr3_leftfinger")
    _disable_visual_and_collision(f"{robot_root}/fr3_rightfinger")

    # 清理早期错误挂在 fr3_hand 下的工具，保证重复运行时不会留下旧 Prim。
    _remove(f"{hand_path}/side_suction_tool")
    _remove(f"{mount_path}/side_suction_tool")
    _remove(f"{mount_path}/vertical_array_tool")
    tool_leaf = (
        "side_suction_tool"
        if TASK24_TOOL_VARIANT == "side_l_array"
        else "vertical_array_tool"
    )
    tool = f"{mount_path}/{tool_leaf}"
    _remove(tool)
    UsdGeom.Xform.Define(stage, tool)

    if TASK24_TOOL_VARIANT == "side_l_array":
        # 固定短 L：先沿局部 +Z 竖直下探，再沿局部 +/-Y 横向伸出。
        # 这使腕部相对 Cup 保持在箱体外侧，留下垛墙与法兰的碰撞余量。
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

        # 刚性吸盘面板：其 X-Z 面承载四个完全相同的吸盘。
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
        # 与 Task24 URDF 中 side_suction_tcp_joint 的 rpy 完全一致。位置和杯面
        # 已经一致时，Ground Truth 的姿态也必须是同一个 TCP frame，不能只发布
        # link8 的姿态。
        tcp_xform.AddRotateZOp().Set(branch_sign * 90.0)
    else:
        # 顶吸对照版：仅一根竖直杆，面板平行桌面，2 x 2 Cup 垂直向下。
        vertical_support = UsdGeom.Cube.Define(stage, f"{tool}/vertical_support")
        vertical_support.CreateSizeAttr(1.0)
        vertical_xform = UsdGeom.Xformable(vertical_support.GetPrim())
        vertical_xform.AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, 0.5 * VERTICAL_ARRAY_PANEL_Z)
        )
        vertical_xform.AddScaleOp().Set(
            Gf.Vec3f(
                VERTICAL_SUPPORT_WIDTH,
                VERTICAL_SUPPORT_WIDTH,
                VERTICAL_ARRAY_PANEL_Z,
            )
        )
        vertical_support.CreateDisplayColorAttr([Gf.Vec3f(0.22, 0.25, 0.30)])
        UsdPhysics.CollisionAPI.Apply(vertical_support.GetPrim())

        _box(
            f"{tool}/vacuum_manifold",
            (0.0, 0.0, VERTICAL_ARRAY_PANEL_Z),
            (MANIFOLD_X_SIZE, MANIFOLD_Z_SIZE, MANIFOLD_Y_SIZE),
            (0.18, 0.22, 0.28),
        )
        for row, y_offset in enumerate((-VERTICAL_ARRAY_XY_OFFSET, VERTICAL_ARRAY_XY_OFFSET), start=1):
            for column, x_offset in enumerate((-VERTICAL_ARRAY_XY_OFFSET, VERTICAL_ARRAY_XY_OFFSET), start=1):
                cup = UsdGeom.Cylinder.Define(stage, f"{tool}/cup_r{row}_c{column}")
                cup.CreateAxisAttr(UsdGeom.Tokens.z)
                cup.CreateRadiusAttr(CUP_RADIUS)
                cup.CreateHeightAttr(CUP_LENGTH)
                UsdGeom.Xformable(cup.GetPrim()).AddTranslateOp().Set(
                    Gf.Vec3d(x_offset, y_offset, VERTICAL_ARRAY_CUP_CENTER_Z)
                )
                cup.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.05, 0.05)])
                UsdPhysics.CollisionAPI.Apply(cup.GetPrim())

        tcp = UsdGeom.Xform.Define(stage, f"{tool}/vertical_array_tcp")
        UsdGeom.Xformable(tcp.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, VERTICAL_ARRAY_TCP_Z)
        )


def _build_objects():
    # Task24 是可从空 Isaac Stage 直接重建的场景；同一 GUI Session 若曾运行
    # Task23，遗留的刚体会参与 PhysX，遗留的 ROS ActionGraph 还会对同一机器人
    # 重复发布 /joint_states、重复写 joint command。两者都会破坏 MoveIt--Isaac
    # 一一对应关系，因此只清理旧任务根，不触碰 FR3 / Table。
    _remove("/World/Task23")
    _remove(TASK_ROOT)
    UsdGeom.Xform.Define(stage, TASK_ROOT)
    UsdGeom.Xform.Define(stage, SUPPLY_ROOT)
    UsdGeom.Xform.Define(stage, MARKER_ROOT)
    for index, pose in enumerate(SUPPLY_POSES, start=1):
        path = f"{SUPPLY_ROOT}/Cube_{index:02d}"
        if not ISOLATE_FIRST_CUBE or index == 1:
            _box(
                path, pose,
                (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), (0.92, 0.42, 0.18), True,
                {"id": f"task24_cube_{index:02d}", "role": "offline_supply", "mass": CUBE_MASS},
            )
        else:
            # Bridge 仍需发布固定索引的 Ground Truth；空 Xform 不含 visual、
            # Collider 或 RigidBody，因而物理上等同于从场景删除。
            placeholder = UsdGeom.Xform.Define(stage, path)
            UsdGeom.Xformable(placeholder.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*pose))
    for index, pose in enumerate(TARGETS, start=1):
        marker = UsdGeom.Cube.Define(stage, f"{MARKER_ROOT}/Target_{index:02d}")
        marker.CreateSizeAttr(1.0)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.80, 0.25)])
        xform = UsdGeom.Xformable(marker.GetPrim())
        xform.AddTranslateOp().Set(Gf.Vec3d(*pose))
        # 目标仅为可视标记，绝不创建 Collider 或影响物理。
        xform.AddScaleOp().Set(Gf.Vec3f(0.026, 0.026, 0.002))


def _build_ros_graph():
    import omni.graph.core as og
    for path in (
        "/World/ActionGraph",
        "/ActionGraph",
        "/World/Task06ROSGraph",
        "/World/Task11SharedBoxROSGraph",
        "/World/Task12SharedBoxROSGraph",
        "/World/Task20RuntimeROSGraph",
        "/World/Task23GripperROSGraph",
        GRAPH_PATH,
    ):
        _remove(path)
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("LeftPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("LeftSub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("LeftCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
                ("RightPub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("RightSub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("RightCtrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("Tick.outputs:tick", "Clock.inputs:execIn"),
                ("Time.outputs:simulationTime", "Clock.inputs:timeStamp"),
                ("Context.outputs:context", "Clock.inputs:context"),
                ("Tick.outputs:tick", "LeftPub.inputs:execIn"),
                ("Tick.outputs:tick", "LeftSub.inputs:execIn"),
                ("Tick.outputs:tick", "LeftCtrl.inputs:execIn"),
                ("Time.outputs:simulationTime", "LeftPub.inputs:timeStamp"),
                ("Context.outputs:context", "LeftPub.inputs:context"),
                ("Context.outputs:context", "LeftSub.inputs:context"),
                ("LeftSub.outputs:jointNames", "LeftCtrl.inputs:jointNames"),
                ("LeftSub.outputs:positionCommand", "LeftCtrl.inputs:positionCommand"),
                ("LeftSub.outputs:velocityCommand", "LeftCtrl.inputs:velocityCommand"),
                ("LeftSub.outputs:effortCommand", "LeftCtrl.inputs:effortCommand"),
                ("Tick.outputs:tick", "RightPub.inputs:execIn"),
                ("Tick.outputs:tick", "RightSub.inputs:execIn"),
                ("Tick.outputs:tick", "RightCtrl.inputs:execIn"),
                ("Time.outputs:simulationTime", "RightPub.inputs:timeStamp"),
                ("Context.outputs:context", "RightPub.inputs:context"),
                ("Context.outputs:context", "RightSub.inputs:context"),
                ("RightSub.outputs:jointNames", "RightCtrl.inputs:jointNames"),
                ("RightSub.outputs:positionCommand", "RightCtrl.inputs:positionCommand"),
                ("RightSub.outputs:velocityCommand", "RightCtrl.inputs:velocityCommand"),
                ("RightSub.outputs:effortCommand", "RightCtrl.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("Time.inputs:resetOnStop", True),
                ("Clock.inputs:topicName", "/clock"),
                ("LeftPub.inputs:targetPrim", LEFT_ROOT),
                ("LeftPub.inputs:topicName", "/left/joint_states"),
                ("LeftSub.inputs:topicName", "/left/joint_command"),
                ("LeftCtrl.inputs:robotPath", LEFT_ROOT),
                ("RightPub.inputs:targetPrim", RIGHT_ROOT),
                ("RightPub.inputs:topicName", "/right/joint_states"),
                ("RightSub.inputs:topicName", "/right/joint_command"),
                ("RightCtrl.inputs:robotPath", RIGHT_ROOT),
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
_build_tool(LEFT_ROOT, -1.0)
_build_tool(RIGHT_ROOT, +1.0)
_build_objects()
_build_ros_graph()

print("\n====================================================")
if TASK24_TOOL_VARIANT == "side_l_array":
    print("Task24 side-suction tight offline scene ready")
    print(
        "tools: fixed 2x2 side-suction array, short L support "
        "(80 mm down + 130 mm side), TCP offset=0.155 m"
    )
else:
    print("Task24 vertical-array top-suction visual comparison scene ready")
    print(
        "tools: fixed 2x2 top-suction array, vertical-only support "
        "(%.0f mm down), TCP offset=%.3f m"
        % (VERTICAL_ARRAY_SUPPORT_LENGTH * 1000.0, VERTICAL_ARRAY_TCP_Z)
    )
print("supply layout: 8 fixed-known Cube slots, active Cube mass=%.3f kg" % CUBE_MASS)
if ISOLATE_FIRST_CUBE:
    print("ISOLATION: only Cube_01 is physical; Cube_02..Cube_08 have no visual/rigid/collision")
print("wall schedule: YZ face x=0.820 (4 cubes) -> YZ face x=0.640 (4 cubes)")
print("cube: 0.120 m; Y/Z pitch: 0.150 m; pre-push offset: -X 0.020 m; short push: +X 0.020 m")
print("PUB: /clock, /left/joint_states, /right/joint_states")
print("SUB: /left/joint_command, /right/joint_command")
if TASK24_TOOL_VARIANT == "side_l_array":
    print("Next: Play, then run task24_side_suction_tight_bridge.py")
else:
    print("Visual comparison only: do not run the side-suction bridge or Task24 executor.")
print("====================================================")
