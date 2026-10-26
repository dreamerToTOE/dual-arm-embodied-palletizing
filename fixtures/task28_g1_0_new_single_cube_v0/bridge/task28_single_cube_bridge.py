#!/usr/bin/env python3
"""Isolated Task28 ROS/Isaac adapter, derived from Task24 at 2ef2edaa.

NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED. Module import is inert. Future
start_bridge() defaults to telemetry only: no command subscriptions, gripper
construction, articulation command writes, or suction actions. Explicit
start_bridge(enable_robot_commands=True) requires separate execution approval.
No simulator/ROS execution was performed when establishing this fixture.
"""
import builtins
import hashlib
import math
import os
import threading
from pathlib import Path

FIXTURE_ID = "task28_g1_0_new_single_cube_v0"
STATUS = "NEW_BASELINE_NOT_YET_PHYSICALLY_VALIDATED"
EXPECTED_FR3_USD_SHA256 = "3feceb47747b49f908f9719299132deb9759f2d5edb390d14f0f3ed62c63255f"
FR3_USD_IDENTITY = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Franka/FR3/fr3.usd"
TASK_ROOT = "/World/Task28G10SingleCube"
ROS_NAMESPACE = "/task28_g1_0"
REGISTRY_NAME = "_task28_g1_0_single_cube_bridge"
SIDES = ("left", "right")
BRANCH_SIGN = {"left": -1.0, "right": 1.0}
ACTIVE_CUBE_COUNT = 1
CUBE_ID = "Cube_01"
CUBE_INDEX = 0
CUBE_PATHS = (TASK_ROOT + "/Supply/Cube_01",)
DUAL_SYNC_FRAME_ID = "task28_g1_0_dual_sync"
RIGID_GRASP_FORCE_LIMIT = 1.0e6
RIGID_GRASP_TORQUE_LIMIT = 1.0e6
GRASP_STIFFNESS = {"left": 1.0e4, "right": 1.0e4}
GRASP_DAMPING = {"left": 1.0e3, "right": 1.0e3}
TCP_LOCAL_TRANSLATIONS = {"left": (0.0, -0.155, 0.080), "right": (0.0, 0.155, 0.080)}
TCP_LOCAL_RZ_DEGREES = {"left": -90.0, "right": 90.0}
# These desired world grasp orientations are distinct from link8->TCP local Rz.
DESIRED_WORLD_GRASP_XYZW = {"left": (0.7071067812, 0.7071067812, 0.0, 0.0), "right": (0.7071067812, -0.7071067812, 0.0, 0.0)}


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



def _scene_cube_paths(stage):
    """Require the exact one-Cube identity and ordered PoseArray mapping."""
    root = stage.GetPrimAtPath(TASK_ROOT)
    if not root.IsValid() or root.GetCustomDataByKey("task28_g1_0_fixture_id") != FIXTURE_ID:
        raise RuntimeError("Missing/mismatched isolated Task28 scene")
    active = root.GetCustomDataByKey("task28_g1_0_active_cube_count")
    if isinstance(active, bool) or not isinstance(active, int) or active != ACTIVE_CUBE_COUNT:
        raise RuntimeError("Task28 requires active_cube_count == 1")
    supply = stage.GetPrimAtPath(TASK_ROOT + "/Supply")
    children = tuple(str(child.GetPath()) for child in supply.GetChildren()) if supply.IsValid() else ()
    if children != CUBE_PATHS:
        raise RuntimeError("Task28 Supply must contain exactly Cube_01 at index 0")
    return CUBE_PATHS


def start_bridge(enable_robot_commands=False):
    """Future explicit entry. Default is telemetry-only; no robot commands."""
    if type(enable_robot_commands) is not bool:
        raise TypeError("enable_robot_commands requires an explicit Boolean")
    if getattr(builtins, REGISTRY_NAME, None) is not None:
        raise RuntimeError("Task28 bridge already registered; no existing bridge will be stopped")
    verified_asset_path()
    # Runtime dependencies remain below the explicit entry point.
    import numpy as np
    import omni.kit.app
    import omni.physx
    import omni.physics.tensors
    import omni.usd
    from pxr import Usd, UsdGeom
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.robot.surface_gripper", True)
    from isaacsim.robot.surface_gripper._surface_gripper import Surface_Gripper, Surface_Gripper_Properties
    import rclpy
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    class Task28SingleCubeBridge:
        def __init__(self, enable_robot_commands=False):
            self.enable_robot_commands = bool(enable_robot_commands)
            self.stage = omni.usd.get_context().get_stage()
            if self.stage is None:
                raise RuntimeError("当前没有有效 USD Stage。")
            asset_path, asset_sha256 = verified_asset_path()
            root = self.stage.GetPrimAtPath(TASK_ROOT)
            if (root.GetCustomDataByKey("task28_g1_0_fr3_usd") != asset_path or
                    root.GetCustomDataByKey("task28_g1_0_fr3_usd_sha256") != asset_sha256):
                raise RuntimeError("Scene/bridge FR3 asset identity mismatch")
            self.cube_paths = _scene_cube_paths(self.stage)
            missing = [path for path in self.cube_paths if not self.stage.GetPrimAtPath(path).IsValid()]
            if missing:
                raise RuntimeError("缺少 Task28 Supply Cube：" + ", ".join(missing))

            self._lock = threading.Lock()
            self._desired = {side: False for side in SIDES}
            self._last_commanded = {side: None for side in SIDES}
            self._retry_accum = {side: 0.0 for side in SIDES}
            self._publish_accum = 0.0
            self._retry_interval = 0.05
            self.grippers = ({side: self._build_gripper(side) for side in SIDES}
                             if self.enable_robot_commands else {})

            # Scene ActionGraph publishes only /task28_g1_0/{left,right}/joint_states.
            # 所有关节目标都
            # 由本 Bridge 写入，以免 Graph controller 与这里的控制器在同一帧争夺目标。
            self.articulations = {}
            for side in (SIDES if self.enable_robot_commands else ()):
                articulation = SingleArticulation(f"{TASK_ROOT}/{side}_fr3")
                articulation.initialize()
                if not articulation.handles_initialized:
                    raise RuntimeError(f"{side} FR3 Articulation 初始化失败；请确认 Timeline 已 Play。")
                self.articulations[side] = articulation
            self._single_commands = {side: None for side in SIDES}
            self._dual_commands = {side: None for side in SIDES}

            self.context = Context()
            rclpy.init(context=self.context)
            self.node = rclpy.create_node(
                "single_cube_bridge", namespace=ROS_NAMESPACE, context=self.context,
                cli_args=["--ros-args", "-r", "/clock:=" + ROS_NAMESPACE + "/clock",
                          "-r", "/rosout:=" + ROS_NAMESPACE + "/rosout",
                          "-r", "/parameter_events:=" + ROS_NAMESPACE + "/parameter_events"],
                parameter_overrides=[Parameter("use_sim_time", value=True)],
            )
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.command_subs = {}
            self.joint_command_subs = {}
            self.state_pubs = {}
            self.tcp_pubs = {}
            for side in SIDES:
                if self.enable_robot_commands:
                    self.command_subs[side] = self.node.create_subscription(
                        Bool, f"/task28_g1_0/{side}/suction_command",
                        lambda message, current_side=side: self._command(current_side, message), 10,
                    )
                    self.joint_command_subs[side] = self.node.create_subscription(
                        JointState, f"{ROS_NAMESPACE}/{side}/joint_command",
                        lambda message, current_side=side: self._joint_command(current_side, message), 20,
                    )
                self.state_pubs[side] = self.node.create_publisher(
                    Bool, f"/task28_g1_0/{side}/suction_state", 10
                )
                self.tcp_pubs[side] = self.node.create_publisher(
                    PoseStamped, f"/task28_g1_0/{side}/side_suction_tcp_pose", 10
                )
            self.cube_pub = self.node.create_publisher(PoseArray, "/task28_g1_0/cube_poses", 10)

            self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.spin_thread.start()
            self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._physics_step
            )

            print("Task28 bridge active; Cube_01/index 0; commands_enabled=" + str(self.enable_robot_commands))
            print("All topics are scoped under " + ROS_NAMESPACE)

        def _build_gripper(self, side):
            # 与 Task28 MoveIt URDF 完全一致：L 型工具的安装 frame 是 fr3_link8，
            # 而非 Isaac 官方资产中带额外固定旋转的 fr3_hand。
            mount_path = f"{TASK_ROOT}/{side}_fr3/fr3_link8"
            if not self.stage.GetPrimAtPath(mount_path).IsValid():
                raise RuntimeError(f"缺少 {mount_path}。")
            joint_path = f"{mount_path}/task28_g1_0_side_surface_gripper_joint"
            if self.stage.GetPrimAtPath(joint_path).IsValid():
                raise RuntimeError("Existing Task28 gripper joint; use a fresh scene: " + joint_path)

            sign = BRANCH_SIGN[side]
            props = Surface_Gripper_Properties()
            props.d6JointPath = f"{mount_path}/task28_g1_0_side_surface_gripper_joint"
            props.parentPath = mount_path
            # 默认搜索方向为 offset 局部 +X。Rz(sign*90°) 将其旋到 L 型横臂方向；
            # left 的杯面指向 +Y、right 的杯面指向 -Y（世界方向由对应 TCP pose 保证）。
            offset = omni.physics.tensors.Transform()
            offset.p.x = 0.0
            # 与 Task28-E L 型阵列、MoveIt URDF 的 side_suction_tcp 完全一致：
            # 80 mm 竖直段 + 130 mm 横向段，Cup 接触 TCP 为 side=155 mm、z=80 mm。
            offset.p.y = sign * 0.155
            offset.p.z = 0.080
            offset.r.x = 0.0
            offset.r.y = 0.0
            offset.r.z = sign * 0.70710678
            offset.r.w = 0.70710678
            props.offset = offset
            # 接触 TCP 已由执行器送到 Cup 面的名义侧面。Surface Gripper close()
            # 固定当前相对几何，不会自动消除大间隙；3 mm 阈值只保留捕获容差，
            # 不允许 10 mm 浮空吸附。
            props.gripThreshold = 0.003
            props.forceLimit = RIGID_GRASP_FORCE_LIMIT
            props.torqueLimit = RIGID_GRASP_TORQUE_LIMIT
            # Task28 的共同搬运不允许 Cup--Cube 相对转动。0 使 Surface Gripper
            # 创建 Fixed joint，而不是允许 15 deg 弯折的 Spherical joint。
            props.bendAngle = 0.0
            # 双侧同时吸附形成闭环。两侧采用一致的有限阻抗，避免把两个 D6
            # 变成互相争夺同一 Cube 六维位姿的数值刚体；该设置不改变吸附门限。
            props.stiffness = GRASP_STIFFNESS[side]
            props.damping = GRASP_DAMPING[side]
            props.retryClose = True
            props.disableGravity = False
            gripper = Surface_Gripper()
            if not gripper.initialize(props):
                raise RuntimeError(f"{side} 侧面 Surface Gripper 初始化失败；请确认 Timeline 已 Play。")
            return gripper

        def _command(self, side, message):
            if not self.enable_robot_commands:
                return
            with self._lock:
                self._desired[side] = bool(message.data)

        @staticmethod
        def _stamp_key(message):
            stamp = message.header.stamp
            return int(stamp.sec), int(stamp.nanosec)

        def _joint_command(self, side, message):
            if not self.enable_robot_commands:
                return
            if (not message.name or len(message.name) != len(message.position) or
                    len(set(message.name)) != len(message.name) or
                    not all(math.isfinite(float(value)) for value in message.position)):
                print(f"[Task28 Isaac][{side}] ignored malformed joint command")
                return
            command = (
                self._stamp_key(message),
                tuple(message.name),
                np.asarray(message.position, dtype=np.float64),
            )
            with self._lock:
                if message.header.frame_id == DUAL_SYNC_FRAME_ID:
                    self._dual_commands[side] = command
                else:
                    self._single_commands[side] = command

        def _apply_joint_command(self, side, command):
            if not self.enable_robot_commands:
                raise RuntimeError("Robot commands are disabled for this bridge")
            _, names, positions = command
            articulation = self.articulations[side]
            indices = []
            for name in names:
                index = articulation.get_dof_index(name)
                if index is None or index < 0:
                    raise RuntimeError(f"{side} FR3 不存在关节 {name}。")
                indices.append(index)
            action = ArticulationAction(
                joint_positions=positions,
                joint_indices=np.asarray(indices, dtype=np.int64),
            )
            articulation.apply_action(action)

        def _apply_pending_joint_commands(self):
            """单臂命令可立即写入；共同命令必须按相同 stamp 成对原子提交。"""
            with self._lock:
                single = self._single_commands
                self._single_commands = {side: None for side in SIDES}
                left_dual = self._dual_commands["left"]
                right_dual = self._dual_commands["right"]
                dual = None
                if left_dual is not None and right_dual is not None:
                    if left_dual[0] == right_dual[0]:
                        dual = (left_dual, right_dual)
                        self._dual_commands = {side: None for side in SIDES}
                    elif left_dual[0] < right_dual[0]:
                        # ROS 本地可靠通信不会常态丢包；若旧的一侧迟到，丢弃旧帧而
                        # 保留新帧，绝不把不同 phase 的左右目标拼成一对。
                        self._dual_commands["left"] = None
                    else:
                        self._dual_commands["right"] = None

            for side in SIDES:
                if single[side] is not None:
                    self._apply_joint_command(side, single[side])
            if dual is not None:
                # 两个 apply_action 均在这个 PhysX callback 内完成，下一 physics step
                # 才会同时消费左右目标；不存在独立 Action Graph 的帧级先后差。
                self._apply_joint_command("left", dual[0])
                self._apply_joint_command("right", dual[1])

        def _physics_step(self, dt):
            if self.enable_robot_commands:
                self._apply_pending_joint_commands()
                with self._lock:
                    desired = dict(self._desired)
                for side in SIDES:
                    gripper = self.grippers[side]
                    if desired[side] != self._last_commanded[side]:
                        ok = gripper.close() if desired[side] else gripper.open()
                        print(f"[Task28 Isaac][{side}] SUCTION {'ON' if desired[side] else 'OFF'}, result={ok}")
                        self._last_commanded[side] = desired[side]
                        self._retry_accum[side] = 0.0
                    if desired[side]:
                        gripper.update()
                        if not gripper.is_closed():
                            self._retry_accum[side] += float(dt)
                            if self._retry_accum[side] >= self._retry_interval:
                                self._retry_accum[side] = 0.0
                                if gripper.close():
                                    print(f"[Task28 Isaac][{side}] side suction retry -> CLOSED")
                    else:
                        self._retry_accum[side] = 0.0
            self._publish_accum += float(dt)
            if self._publish_accum >= 0.05:
                self._publish_accum = 0.0
                self._publish()

        def _pose(self, path):
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Ground Truth Prim 不存在：{path}")
            transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            position = transform.ExtractTranslation()
            rotation = transform.ExtractRotationQuat()
            imaginary = rotation.GetImaginary()
            norm = math.sqrt(sum(float(value) ** 2 for value in imaginary) + float(rotation.GetReal()) ** 2)
            result = Pose()
            result.position.x, result.position.y, result.position.z = map(float, position)
            result.orientation.x = float(imaginary[0]) / norm
            result.orientation.y = float(imaginary[1]) / norm
            result.orientation.z = float(imaginary[2]) / norm
            result.orientation.w = float(rotation.GetReal()) / norm
            return result

        def _publish(self):
            stamp = self.node.get_clock().now().to_msg()
            array = PoseArray()
            array.header.stamp = stamp
            array.header.frame_id = "world"
            array.poses = [self._pose(path) for path in self.cube_paths]
            self.cube_pub.publish(array)
            for side in SIDES:
                state = Bool()
                state.data = bool(self.grippers[side].is_closed()) if self.enable_robot_commands else False
                self.state_pubs[side].publish(state)
                tcp = PoseStamped()
                tcp.header.stamp = stamp
                tcp.header.frame_id = "world"
                tcp.pose = self._pose(f"{TASK_ROOT}/{side}_fr3/fr3_link8/side_suction_tool/side_suction_tcp")
                self.tcp_pubs[side].publish(tcp)

        def shutdown(self):
            for side in self.grippers:
                try:
                    self.grippers[side].open()
                except Exception:
                    pass
            if getattr(self, "physics_sub", None) is not None:
                try:
                    self.physics_sub.unsubscribe()
                except Exception:
                    pass
            try:
                self.executor.shutdown(timeout_sec=0.5)
                self.node.destroy_node()
                self.context.shutdown()
            except Exception:
                pass
            if getattr(builtins, REGISTRY_NAME, None) is self:
                delattr(builtins, REGISTRY_NAME)

    bridge = Task28SingleCubeBridge(enable_robot_commands=enable_robot_commands)
    setattr(builtins, REGISTRY_NAME, bridge)
    return bridge


if __name__ == "__main__":
    # Even direct script execution defaults to telemetry only.
    start_bridge()
