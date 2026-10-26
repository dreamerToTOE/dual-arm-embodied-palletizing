#!/usr/bin/env python3
"""Isolated NEW Task28 fixture, derived from Task24 source at 2ef2edaa.

NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED. Importing this file is inert.
Future invocation requires a stopped, otherwise empty physics stage and an
explicit local TASK28_FR3_USD plus TASK28_FR3_USD_SHA256. No model fallback.
No simulator execution was performed when establishing this fixture.
"""
import hashlib
import json
import math
import os
from pathlib import Path

FIXTURE_ID = "task28_g1_0_new_single_cube_v0"
STATUS = "NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED"
EXPECTED_FR3_USD_SHA256 = "3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f"
FR3_USD_IDENTITY = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Franka/FR3/fr3.usd"
TASK_ROOT = "/World/Task28G10SingleCube"
LEFT_ROOT = TASK_ROOT + "/left_fr3"
RIGHT_ROOT = TASK_ROOT + "/right_fr3"
TABLE_PATH = TASK_ROOT + "/Table"
GRAPH_PATH = TASK_ROOT + "/ROSGraph"
PHYSICS_PATH = TASK_ROOT + "/physicsScene"
SUPPLY_ROOT = TASK_ROOT + "/Supply"
MARKER_ROOT = TASK_ROOT + "/TargetMarkers"
MATERIAL_PATH = TASK_ROOT + "/HighFrictionMaterial"
ROS_NAMESPACE = "/task28_g1_0"
LEFT_BASE = (0.65, -0.60, 0.0)
RIGHT_BASE = (0.65, 0.60, 0.0)
TABLE_CENTER = (0.55, 0.0, 0.10)
TABLE_SIZE = (1.20, 0.80, 0.20)
TABLE_TOP_Z = 0.20
FR3_OFFICIAL_START_Q_RAD = (0.0, -0.7853981634, 0.0, -2.3561944902, 0.0, 1.5707963268, 0.7853981634)
CUBE_SIZE = 0.120
CUBE_HALF = 0.060
CUBE_MASS = 0.800
ACTIVE_CUBE_COUNT = 1
CUBE_ID = "Cube_01"
CUBE_INDEX = 0
SUPPLY_POSES = ((0.650, 0.0, 0.260),)
TARGETS = ((0.820, -0.075, 0.260),)
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
STAGE_METERS_PER_UNIT = 1.0
STAGE_UP_AXIS = "Z"
GRAVITY_DIRECTION = (0.0, 0.0, -1.0)
GRAVITY_MAGNITUDE_M_S2 = 9.81
PHYSICS_STEPS_PER_SECOND = 60


def verified_asset_path():
    """Resolve only the explicitly identified local FR3 asset; fail closed."""
    value = os.environ.get("TASK28_FR3_USD", "")
    expected = os.environ.get("TASK28_FR3_USD_SHA256", "").lower()
    if not value or len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
        raise RuntimeError("UNKNOWN FR3 asset: require TASK28_FR3_USD and its 64-digit SHA256")
    if expected != EXPECTED_FR3_USD_SHA256:
        raise RuntimeError("TASK28_FR3_USD_SHA256 must equal the frozen FR3 asset hash")
    asset = Path(value).expanduser()
    if not asset.is_absolute() or not asset.is_file():
        raise RuntimeError("TASK28_FR3_USD must identify an existing absolute local USD file")
    digest = hashlib.sha256()
    with asset.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    if digest.hexdigest() != expected:
        raise RuntimeError("TASK28_FR3_USD_SHA256 mismatch; model substitution is forbidden")
    return str(asset.resolve()), expected


def build_scene():
    """Future opt-in entry; creates only prims under TASK_ROOT."""
    asset_path, asset_sha256 = verified_asset_path()
    import omni.kit.app
    import omni.usd
    import omni.timeline
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath("/World").IsValid():
        raise RuntimeError("Require a valid stage with an existing /World root")
    if omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Stop Timeline before constructing the isolated Task28 scene")
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or UsdGeom.GetStageMetersPerUnit(stage) != STAGE_METERS_PER_UNIT:
        raise RuntimeError("Task28 requires a metre/Z-up stage; stage metadata is never rewritten")
    # A foreign task/physics/graph is rejected, never removed or reconfigured.
    foreign = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path == TASK_ROOT or path.startswith(TASK_ROOT + "/"):
            continue
        name = prim.GetName()
        if (name.startswith("Task") or "ActionGraph" in name or
                prim.HasAPI(UsdPhysics.RigidBodyAPI) or
                prim.HasAPI(UsdPhysics.CollisionAPI) or
                prim.HasAPI(UsdPhysics.ArticulationRootAPI) or
                prim.IsA(UsdPhysics.Scene) or prim.GetTypeName() == "OmniGraph"):
            foreign.append(path)
    if foreign:
        raise RuntimeError("Foreign task/physics roots present; use a fresh stage: " + ", ".join(foreign))

    def _remove(path):
        if path != TASK_ROOT and not path.startswith(TASK_ROOT + "/"):
            raise RuntimeError("Refusing mutation outside Task28 root: " + path)
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)

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
        robot.GetReferences().AddReference(asset_path)
        xform = UsdGeom.Xformable(robot)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp(opSuffix="task28_g1_0_base").Set(Gf.Vec3d(*base))


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


    def _box(path, center, size, color, dynamic=False, metadata=None):
        box = UsdGeom.Cube.Define(stage, path)
        box.CreateSizeAttr(1.0)
        box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        transform = UsdGeom.Xformable(box.GetPrim())
        transform.AddTranslateOp(opSuffix="task28_g1_0_pose").Set(Gf.Vec3d(*center))
        transform.AddScaleOp(opSuffix="task28_g1_0_size").Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        if dynamic:
            UsdPhysics.RigidBodyAPI.Apply(box.GetPrim())
            UsdPhysics.MassAPI.Apply(box.GetPrim()).CreateMassAttr().Set(CUBE_MASS)
        if metadata:
            box.GetPrim().SetCustomDataByKey("task28_g1_0_metadata", json.dumps(metadata))
        _bind(box.GetPrim(), _MATERIAL)
        return box


    def _build_tool(robot_root, branch_sign):
        hand_path = f"{robot_root}/fr3_hand"
        # MoveIt 的 Task28 URDF 将 L 型工具固定在 fr3_link8。必须在 Isaac 中使用
        # 同一个安装 frame：fr3_hand 在官方资产里带有
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
        tool = f"{mount_path}/side_suction_tool"
        _remove(tool)
        UsdGeom.Xform.Define(stage, tool)

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
        # 与 Task28 URDF 中 side_suction_tcp_joint 的 rpy 完全一致。位置和杯面
        # 已经一致时，Ground Truth 的姿态也必须是同一个 TCP frame，不能只发布
        # link8 的姿态。
        tcp_xform.AddRotateZOp().Set(branch_sign * 90.0)


    def _build_objects():
        UsdGeom.Xform.Define(stage, SUPPLY_ROOT)
        UsdGeom.Xform.Define(stage, MARKER_ROOT)
        _box(f"{SUPPLY_ROOT}/{CUBE_ID}", SUPPLY_POSES[0],
             (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), (0.92, 0.42, 0.18), True,
             {"id": CUBE_ID, "index": CUBE_INDEX, "role": "single_cube_supply", "mass": CUBE_MASS})
        marker = UsdGeom.Cube.Define(stage, f"{MARKER_ROOT}/Target_01")
        marker.CreateSizeAttr(1.0)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.80, 0.25)])
        transform = UsdGeom.Xformable(marker.GetPrim())
        transform.AddTranslateOp().Set(Gf.Vec3d(*TARGETS[0]))
        transform.AddScaleOp().Set(Gf.Vec3f(0.026, 0.026, 0.002))
        # Marker is deliberately nonphysical: no CollisionAPI or RigidBodyAPI.


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
                    ("Clock.inputs:topicName", ROS_NAMESPACE + "/clock"),
                    ("LeftPub.inputs:targetPrim", LEFT_ROOT),
                    ("LeftPub.inputs:topicName", ROS_NAMESPACE + "/left/joint_states"),
                    ("RightPub.inputs:targetPrim", RIGHT_ROOT),
                    ("RightPub.inputs:topicName", ROS_NAMESPACE + "/right/joint_states"),
                ],
            },
        )

    _remove(TASK_ROOT)
    root = UsdGeom.Xform.Define(stage, TASK_ROOT).GetPrim()
    root.SetCustomDataByKey("task28_g1_0_fixture_id", FIXTURE_ID)
    root.SetCustomDataByKey("task28_g1_0_status", STATUS)
    root.SetCustomDataByKey("task28_g1_0_active_cube_count", ACTIVE_CUBE_COUNT)
    root.SetCustomDataByKey("task28_g1_0_fr3_usd", asset_path)
    root.SetCustomDataByKey("task28_g1_0_fr3_usd_sha256", asset_sha256)
    # Explicit NEW fixture construction defaults; no historical execution claim.
    physics_scene = UsdPhysics.Scene.Define(stage, PHYSICS_PATH)
    physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(*GRAVITY_DIRECTION))
    physics_scene.CreateGravityMagnitudeAttr().Set(GRAVITY_MAGNITUDE_M_S2)
    physics = physics_scene.GetPrim()
    scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics)
    scene_api.CreateTimeStepsPerSecondAttr().Set(PHYSICS_STEPS_PER_SECOND)
    scene_api.CreateEnableCCDAttr().Set(True)
    scene_api.CreateEnableStabilizationAttr().Set(True)
    scene_api.CreateSolverTypeAttr().Set("TGS")
    _MATERIAL = _make_material()
    _box(TABLE_PATH, TABLE_CENTER, TABLE_SIZE, (0.0, 0.65, 0.85), False, {"id": "task28_table", "role": "table"})
    _add_fr3(LEFT_ROOT, LEFT_BASE)
    _add_fr3(RIGHT_ROOT, RIGHT_BASE)
    _set_fr3_official_start_target(LEFT_ROOT)
    _set_fr3_official_start_target(RIGHT_ROOT)
    _build_tool(LEFT_ROOT, -1.0)
    _build_tool(RIGHT_ROOT, +1.0)
    _build_objects()
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    _build_ros_graph()
    print("Task28 fixture constructed (not physically validated): Cube_01 / index 0")
    return TASK_ROOT


if __name__ == "__main__":
    build_scene()
