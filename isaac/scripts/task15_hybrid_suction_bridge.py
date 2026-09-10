# Task15：紧/松协调混合码垛的双 Surface Gripper ROS 2 bridge
#
# 在 Task15 场景已加载且 Timeline Play 后，从 Isaac Script Editor 运行。
# Bridge 只维护 PhysX 吸附和 Ground Truth；紧协调 / 松协调决策均在 ROS 节点中。
# 物理参数严格复用已验收的 Task11--Task13 / Task10 基线。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.physics.tensors
import omni.usd
from pxr import Usd, UsdGeom

extension_manager = omni.kit.app.get_app().get_extension_manager()
extension_manager.set_extension_enabled_immediate(
    "isaacsim.robot.surface_gripper", True
)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool


SIDES = ("left", "right")
LARGE_CUBE_PATH = "/World/Task15LargeCube"
SMALL_CUBE_PATHS = (
    "/World/Task15SmallCube1",
    "/World/Task15SmallCube2",
    "/World/Task15SmallCube3",
    "/World/Task15SmallCube4",
)


class Task15HybridSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")

        for prim_path in (LARGE_CUBE_PATH, *SMALL_CUBE_PATHS):
            if not self.stage.GetPrimAtPath(prim_path).IsValid():
                raise RuntimeError(
                    f"缺少 {prim_path}。请先运行 task15_hybrid_palletizing_scene.py。"
                )

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._retry_interval = 0.05
        self._publish_accum = 0.0
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task15_hybrid_suction_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.command_subs = {}
        self.state_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool,
                f"/task15/{side}/suction_command",
                lambda message, s=side: self._on_suction_command(s, message),
                10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool,
                f"/task15/{side}/suction_state",
                10,
            )

        self.large_pose_pub = self.node.create_publisher(
            PoseStamped,
            "/task15/large_cube_pose",
            10,
        )
        self.small_poses_pub = self.node.create_publisher(
            PoseArray,
            "/task15/small_cube_poses",
            10,
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.physics_sub = (
            omni.physx.get_physx_interface().subscribe_physics_step_events(
                self._on_physics_step
            )
        )

        print("")
        print("====================================================")
        print("Task15 hybrid dual-suction ROS bridge started")
        print("LEFT : /task15/left/suction_command  <-> suction_state")
        print("RIGHT: /task15/right/suction_command <-> suction_state")
        print("PUB  : /task15/large_cube_pose geometry_msgs/PoseStamped")
        print("PUB  : /task15/small_cube_poses [SmallCube1, 2, 3, 4]")
        print("Surface Gripper 参数复用 Task11--13 / Task10 已验收基线")
        print("====================================================")

    def _build_gripper(self, side):
        hand_path = f"/World/{side}_fr3/fr3_hand"
        if not self.stage.GetPrimAtPath(hand_path).IsValid():
            raise RuntimeError(f"缺少 {hand_path}。请先保留 Task06 双 FR3 场景。")

        # 一个末端只能有一套 D6 Surface Gripper Joint。Task15 切换前清理
        # 历史任务残留；不触碰机器人、Table 或 ActionGraph。
        for joint_name in (
            "task07_surface_gripper_joint",
            "task10_surface_gripper_joint",
            "task11_surface_gripper_joint",
            "task15_surface_gripper_joint",
        ):
            old_joint = self.stage.GetPrimAtPath(f"{hand_path}/{joint_name}")
            if old_joint.IsValid():
                self.stage.RemovePrim(old_joint.GetPath())

        props = Surface_Gripper_Properties()
        props.d6JointPath = f"{hand_path}/task15_surface_gripper_joint"
        props.parentPath = hand_path

        offset = omni.physics.tensors.Transform()
        offset.p.x = 0.0
        offset.p.y = 0.0
        offset.p.z = 0.1050
        offset.r.x = 0.0
        offset.r.y = -0.70710678
        offset.r.z = 0.0
        offset.r.w = 0.70710678
        props.offset = offset

        # 保持既有物理基线：contact 后才 SUCTION ON，3 mm 阈值、周期 retry。
        props.gripThreshold = 0.003
        props.forceLimit = 1.0e6
        props.torqueLimit = 1.0e6
        props.bendAngle = math.radians(15.0)
        props.stiffness = 1.0e4
        props.damping = 1.0e3
        props.retryClose = True
        props.disableGravity = False

        gripper = Surface_Gripper()
        if not gripper.initialize(props):
            raise RuntimeError(
                f"{side} Surface Gripper 初始化失败。请确认 Timeline 已 Play。"
            )
        return gripper

    def _on_suction_command(self, side, message):
        with self._lock:
            self._desired[side] = bool(message.data)

    def _on_physics_step(self, dt):
        with self._lock:
            desired = dict(self._desired)

        for side in SIDES:
            gripper = self.grippers[side]
            want_closed = desired[side]
            if want_closed != self._last_commanded[side]:
                ok = gripper.close() if want_closed else gripper.open()
                print(
                    f"[Task15 Isaac][{side}] SUCTION "
                    f"{'ON' if want_closed else 'OFF'} request, result={ok}"
                )
                self._last_commanded[side] = want_closed
                self._retry_accum[side] = 0.0

            # desired=true 仅由 ROS primitive 在 CONTACT 后发出。保留 retry，
            # 用于吸盘与物体有极小 PhysX 时间差时可靠闭合，而不产生提前吸附。
            if want_closed:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        if gripper.close():
                            print(f"[Task15 Isaac][{side}] retry -> CLOSED")
                else:
                    self._retry_accum[side] = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_states_and_poses()

    def _publish_states_and_poses(self):
        for side in SIDES:
            state = Bool()
            state.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(state)

        self.large_pose_pub.publish(self._ground_truth_pose(LARGE_CUBE_PATH))

        small_array = PoseArray()
        small_array.header.stamp = self.node.get_clock().now().to_msg()
        small_array.header.frame_id = "world"
        for path in SMALL_CUBE_PATHS:
            small_array.poses.append(self._ground_truth_pose(path).pose)
        self.small_poses_pub.publish(small_array)

    def _ground_truth_pose(self, prim_path):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Ground Truth Prim 不存在：{prim_path}")
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = transform.ExtractTranslation()
        quaternion = transform.ExtractRotationQuat()
        imaginary = quaternion.GetImaginary()

        # Task15 的 Cube 经 USD Scale 设置尺寸。完整矩阵直接提取的 quaternion
        # 可能长度非 1；ROS Pose/MoveIt Planning Scene 必须使用单位四元数。
        norm = math.sqrt(
            float(imaginary[0]) ** 2
            + float(imaginary[1]) ** 2
            + float(imaginary[2]) ** 2
            + float(quaternion.GetReal()) ** 2
        )
        if norm <= 1.0e-9:
            raise RuntimeError(f"Ground Truth quaternion 无效：{prim_path}")

        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(imaginary[0]) / norm
        message.pose.orientation.y = float(imaginary[1]) / norm
        message.pose.orientation.z = float(imaginary[2]) / norm
        message.pose.orientation.w = float(quaternion.GetReal()) / norm
        return message

    def shutdown(self):
        for side in SIDES:
            try:
                self.grippers[side].open()
            except Exception:
                pass
        try:
            if self.physics_sub is not None:
                self.physics_sub.unsubscribe()
                self.physics_sub = None
        except Exception:
            pass
        try:
            self.executor.shutdown(timeout_sec=0.5)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.context.shutdown()
        except Exception:
            pass


for old_name in (
    "_task07_dual_suction_bridge",
    "_task10_quad_suction_bridge",
    "_task11_shared_box_bridge",
    "_task15_hybrid_suction_bridge",
):
    old_bridge = getattr(builtins, old_name, None)
    if old_bridge is not None:
        try:
            old_bridge.shutdown()
        except Exception as exc:
            print(f"清理旧 bridge {old_name} 时出现非致命异常：", exc)

builtins._task15_hybrid_suction_bridge = Task15HybridSuctionBridge()
