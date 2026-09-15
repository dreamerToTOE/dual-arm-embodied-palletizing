# Task23-A：二指夹爪“暂存后平推”二层码垛验证场景。
#
# 用法：Isaac Sim 4.5 Script Editor，Timeline Stop 时执行：
# exec(open("/home/ubuntu2004/lmy/dual-arm-embodied-palletizing/isaac/scripts/"
#           "task23_gripper_push_stack_scene.py").read())
#
# 本脚本完全不复用冻结的 suction_tool：两台 FR3 保留官方 hand / finger。
# 第一层 4x4 Cube 是静态碰撞体，用于先验证“二层支撑平面上的平推”这一最小
# 物理问题；第二层的 16 个待搬运 Cube 为 Dynamic Rigid Body。第一层整体稳定性
# 与多层支撑传播不在本 Task23-A 的结论范围内。

import json
import math

import omni.client
import omni.kit.app
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

try:
    from isaacsim.storage.native import get_assets_root_path
except Exception as exc:
    raise RuntimeError("请在 Isaac Sim 4.5 中运行 Task23 场景脚本。") from exc


PROJECT_ROOT = "/home/ubuntu2004/lmy/dual-arm-embodied-palletizing"

LEFT_ROOT = "/World/left_fr3"
RIGHT_ROOT = "/World/right_fr3"
TABLE_PATH = "/World/Table"
GRAPH_PATH = "/World/Task23GripperROSGraph"
PHYSICS_PATH = "/physicsScene"
TASK23_ROOT = "/World/Task23"
BASE_ROOT = f"{TASK23_ROOT}/BaseLayer"
SUPPLY_ROOT = f"{TASK23_ROOT}/Supply"
STAGE_PATH = f"{TASK23_ROOT}/LoadingStage"
MATERIAL_PATH = f"{TASK23_ROOT}/HighFrictionMaterial"

LEFT_BASE = (0.55, -0.50, 0.00)
RIGHT_BASE = (0.55, +0.50, 0.00)
TABLE_CENTER = (0.55, 0.00, 0.025)
TABLE_SIZE = (1.20, 0.80, 0.050)
TABLE_TOP_Z = 0.050

CUBE_SIZE = 0.030
CUBE_HALF = 0.015
BASE_CENTER_Z = TABLE_TOP_Z + CUBE_HALF
LAYER2_CENTER_Z = TABLE_TOP_Z + 3.0 * CUBE_HALF
LAYER2_SUPPORT_Z = TABLE_TOP_Z + CUBE_SIZE

# 推送方向固定为 +X，因此 selector 采用每一行从最远 X 到最近 X 的顺序。
GRID_X = (0.615, 0.645, 0.675, 0.705)
GRID_Y = (-0.045, -0.015, 0.015, 0.045)
STAGING_X = 0.565
STAGE_CENTER = (0.565, 0.000, 0.065)
STAGE_SIZE = (0.070, 0.150, 0.030)

SUPPLY_X = (0.240, 0.300, 0.360, 0.420)
SUPPLY_Y = (-0.300, -0.255, -0.210, -0.165)


stage = omni.usd.get_context().get_stage()
if stage is None:
    raise RuntimeError("当前没有有效 USD Stage。")


def _require_stopped_timeline():
    import omni.timeline

    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("请先停止 Timeline，再重建 Task23 场景。")


def _remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)


def _ensure_world_and_physics():
    UsdGeom.Xform.Define(stage, "/World")
    physics = stage.GetPrimAtPath(PHYSICS_PATH)
    if not physics.IsValid():
        physics = UsdPhysics.Scene.Define(stage, PHYSICS_PATH).GetPrim()
    scene = UsdPhysics.Scene(physics)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    api = PhysxSchema.PhysxSceneAPI.Apply(physics)
    api.CreateEnableCCDAttr().Set(True)
    api.CreateEnableStabilizationAttr().Set(True)
    api.CreateSolverTypeAttr().Set("TGS")


def _resolve_fr3_asset():
    root = get_assets_root_path()
    if not root:
        raise RuntimeError("无法获取 Isaac Assets 根目录。")
    root = root.rstrip("/")
    candidates = (
        f"{root}/Robots/Franka/FR3/fr3.usd",
        f"{root}/Isaac/Robots/Franka/FR3/fr3.usd",
    )
    for asset in candidates:
        result, _ = omni.client.stat(asset)
        if result == omni.client.Result.OK:
            return asset
    raise RuntimeError("找不到官方 FR3 USD：\n  - " + "\n  - ".join(candidates))


def _add_fr3(root_path, position):
    prim = stage.DefinePrim(root_path, "Xform")
    prim.GetReferences().AddReference(_resolve_fr3_asset())
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(opSuffix="task23_base").Set(Gf.Vec3d(*position))


def _make_material():
    material = UsdShade.Material.Define(stage, MATERIAL_PATH)
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(0.90)
    physics_material.CreateDynamicFrictionAttr().Set(0.75)
    physics_material.CreateRestitutionAttr().Set(0.0)
    return material


def _bind_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _make_box(path, center, size, color, dynamic, metadata):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(cube.GetPrim())
    xform.AddTranslateOp(opSuffix="task23_pose").Set(Gf.Vec3d(*center))
    xform.AddScaleOp(opSuffix="task23_size").Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        mass = UsdPhysics.MassAPI.Apply(cube.GetPrim())
        mass.CreateMassAttr().Set(0.050)
    cube.GetPrim().SetCustomDataByKey("task23_metadata", json.dumps(metadata))
    _bind_material(cube.GetPrim(), _MATERIAL)
    return cube


def _ensure_table():
    _remove_if_exists(TABLE_PATH)
    table = _make_box(
        TABLE_PATH,
        TABLE_CENTER,
        TABLE_SIZE,
        (0.0, 0.65, 0.85),
        False,
        {"role": "table"},
    )
    return table


def _build_task23_objects():
    _remove_if_exists(TASK23_ROOT)
    UsdGeom.Xform.Define(stage, TASK23_ROOT)
    UsdGeom.Xform.Define(stage, BASE_ROOT)
    UsdGeom.Xform.Define(stage, SUPPLY_ROOT)

    _make_box(
        STAGE_PATH,
        STAGE_CENTER,
        STAGE_SIZE,
        (0.18, 0.18, 0.18),
        False,
        {"role": "loading_stage", "top_z": LAYER2_SUPPORT_Z},
    )
    for row, y in enumerate(GRID_Y):
        for column, x in enumerate(GRID_X):
            _make_box(
                f"{BASE_ROOT}/Base_{row + 1}_{column + 1}",
                (x, y, BASE_CENTER_Z),
                (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
                (0.78, 0.62, 0.25),
                False,
                {
                    "id": f"task23_base_{row}_{column}",
                    "role": "fixed_base_layer",
                    "size": [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
                },
            )
    index = 1
    for row, y in enumerate(SUPPLY_Y):
        for column, x in enumerate(SUPPLY_X):
            identifier = f"task23_supply_{index:02d}"
            _make_box(
                f"{SUPPLY_ROOT}/Supply_{index:02d}",
                (x, y, BASE_CENTER_Z),
                (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
                (0.92, 0.42, 0.18),
                True,
                {
                    "id": identifier,
                    "role": "supply_dynamic",
                    "size": [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
                    "mass": 0.050,
                    "payload_class": "small_cube",
                    "allowed_modes": ["GRIPPER_STAGE_AND_PUSH"],
                },
            )
            index += 1


def _build_ros_graph():
    import omni.graph.core as og

    # 清理的是当前 USD Stage 的旧双臂通信图；不会改动任何历史 Task 源码。
    for path in ("/World/ActionGraph", "/ActionGraph", "/World/Task06ROSGraph", GRAPH_PATH):
        _remove_if_exists(path)
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("LeftPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("LeftSubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("LeftController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("RightPublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("RightSubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("RightController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("OnPlaybackTick.outputs:tick", "LeftPublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "LeftSubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "LeftController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "LeftPublishJointState.inputs:timeStamp"),
                ("Context.outputs:context", "LeftPublishJointState.inputs:context"),
                ("Context.outputs:context", "LeftSubscribeJointState.inputs:context"),
                ("LeftSubscribeJointState.outputs:jointNames", "LeftController.inputs:jointNames"),
                ("LeftSubscribeJointState.outputs:positionCommand", "LeftController.inputs:positionCommand"),
                ("LeftSubscribeJointState.outputs:velocityCommand", "LeftController.inputs:velocityCommand"),
                ("LeftSubscribeJointState.outputs:effortCommand", "LeftController.inputs:effortCommand"),
                ("OnPlaybackTick.outputs:tick", "RightPublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RightSubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "RightController.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "RightPublishJointState.inputs:timeStamp"),
                ("Context.outputs:context", "RightPublishJointState.inputs:context"),
                ("Context.outputs:context", "RightSubscribeJointState.inputs:context"),
                ("RightSubscribeJointState.outputs:jointNames", "RightController.inputs:jointNames"),
                ("RightSubscribeJointState.outputs:positionCommand", "RightController.inputs:positionCommand"),
                ("RightSubscribeJointState.outputs:velocityCommand", "RightController.inputs:velocityCommand"),
                ("RightSubscribeJointState.outputs:effortCommand", "RightController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", True),
                ("PublishClock.inputs:topicName", "/clock"),
                ("LeftPublishJointState.inputs:targetPrim", LEFT_ROOT),
                ("LeftPublishJointState.inputs:topicName", "/left/joint_states"),
                ("LeftSubscribeJointState.inputs:topicName", "/left/joint_command"),
                ("LeftController.inputs:robotPath", LEFT_ROOT),
                ("RightPublishJointState.inputs:targetPrim", RIGHT_ROOT),
                ("RightPublishJointState.inputs:topicName", "/right/joint_states"),
                ("RightSubscribeJointState.inputs:topicName", "/right/joint_command"),
                ("RightController.inputs:robotPath", RIGHT_ROOT),
            ],
        },
    )


_require_stopped_timeline()
manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
_ensure_world_and_physics()
_MATERIAL = _make_material()

# Task23 是自包含场景：不继承和不修改冻结吸盘场景的机器人或工具。
for root in (LEFT_ROOT, RIGHT_ROOT):
    _remove_if_exists(root)
_ensure_table()
_add_fr3(LEFT_ROOT, LEFT_BASE)
_add_fr3(RIGHT_ROOT, RIGHT_BASE)
_build_task23_objects()
_build_ros_graph()

print("")
print("====================================================")
print("Task23 gripper stage-and-push scene ready")
print("robots: dual FR3 with native franka_hand / finger collision enabled")
print("base layer: 4 x 4 static cubes, top z = %.3f m" % LAYER2_SUPPORT_Z)
print("supply: 16 dynamic cubes, mass = 0.050 kg each")
print("target layer: 4 x 4 empty cells, center z = %.3f m" % LAYER2_CENTER_Z)
print("loading stage: x=%.3f, push direction=+X" % STAGING_X)
print("PUB: /clock, /left/joint_states, /right/joint_states")
print("SUB: /left/joint_command, /right/joint_command")
print("Next: click Play, then run task23_gripper_ground_truth_bridge.py")
print("====================================================")
