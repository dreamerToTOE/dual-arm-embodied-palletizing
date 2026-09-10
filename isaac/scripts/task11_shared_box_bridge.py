# Task11：共同大箱体的双 Surface Gripper ROS 2 bridge
#
# Timeline Play 后运行。左右吸盘独立接收 ON/OFF 命令，但目标为同一个
# /World/SharedBox。该脚本只提供物理双吸附与 Ground Truth，不实现搬运。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.physics.tensors
import omni.usd
from pxr import Usd, UsdGeom

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.robot.surface_gripper", True)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool


SIDES = ("left", "right")
SHARED_BOX_PATH = "/World/SharedBox"


class Task11SharedBoxBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        if not self.stage.GetPrimAtPath(SHARED_BOX_PATH).IsValid():
            raise RuntimeError("缺少 /World/SharedBox。请先运行 Task11 场景脚本。")

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._retry_interval = 0.05
        self._publish_accum = 0.0
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            "task11_shared_box_bridge", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.command_subs = {}
        self.state_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool,
                f"/task11/{side}/suction_command",
                lambda message, s=side: self._on_suction_command(s, message),
                10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool,
                f"/task11/{side}/suction_state",
                10,
            )

        self.box_pose_pub = self.node.create_publisher(
            PoseStamped,
            "/task11/shared_box_pose",
            10,
        )
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task11 双吸盘共同物体 ROS bridge 已启动")
        print("LEFT : /task11/left/suction_command  <-> suction_state")
        print("RIGHT: /task11/right/suction_command <-> suction_state")
        print("PUB  : /task11/shared_box_pose geometry_msgs/PoseStamped")
        print("两个 Surface Gripper 可同时连接同一个 /World/SharedBox")
        print("====================================================")

    def _build_gripper(self, side):
        hand_path = f"/World/{side}_fr3/fr3_hand"
        if not self.stage.GetPrimAtPath(hand_path).IsValid():
            raise RuntimeError(f"缺少 {hand_path}。请先保留 Task06 双 FR3 场景。")

        # 一个末端只能保留一套 D6 Surface Gripper Joint；切换 Task 时清理旧桥接残留。
        for joint_name in (
            "task07_surface_gripper_joint",
            "task10_surface_gripper_joint",
            "task11_surface_gripper_joint",
        ):
            old_joint = self.stage.GetPrimAtPath(f"{hand_path}/{joint_name}")
            if old_joint.IsValid():
                self.stage.RemovePrim(old_joint.GetPath())

        props = Surface_Gripper_Properties()
        props.d6JointPath = f"{hand_path}/task11_surface_gripper_joint"
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

        # 复用 Task09/10 已验证参数；Task11 不以扩大阈值掩盖吸附误差。
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
                    f"[Task11 Isaac][{side}] SUCTION "
                    f"{'ON' if want_closed else 'OFF'} request, result={ok}"
                )
                self._last_commanded[side] = want_closed
                self._retry_accum[side] = 0.0

            if want_closed:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        if gripper.close():
                            print(f"[Task11 Isaac][{side}] retry -> CLOSED")
                else:
                    self._retry_accum[side] = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_states_and_pose()

    def _publish_states_and_pose(self):
        for side in SIDES:
            state = Bool()
            state.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(state)

        box_prim = self.stage.GetPrimAtPath(SHARED_BOX_PATH)
        if not box_prim.IsValid():
            return
        transform = UsdGeom.Xformable(box_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        position = transform.ExtractTranslation()
        quaternion = transform.ExtractRotationQuat()
        imaginary = quaternion.GetImaginary()

        # SharedBox 是通过 Xform Scale 设置尺寸的。USD 从含 Scale 的完整
        # 矩阵提取 rotation quaternion 时，其长度可能不为 1；ROS / MoveIt
        # 的 Pose 必须发布单位四元数，否则会污染 Planning Scene 碰撞位姿。
        quaternion_norm = math.sqrt(
            float(imaginary[0]) ** 2
            + float(imaginary[1]) ** 2
            + float(imaginary[2]) ** 2
            + float(quaternion.GetReal()) ** 2
        )
        if quaternion_norm <= 1.0e-9:
            raise RuntimeError("SharedBox Ground Truth quaternion 无效。")

        message = PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(imaginary[0]) / quaternion_norm
        message.pose.orientation.y = float(imaginary[1]) / quaternion_norm
        message.pose.orientation.z = float(imaginary[2]) / quaternion_norm
        message.pose.orientation.w = float(quaternion.GetReal()) / quaternion_norm
        self.box_pose_pub.publish(message)

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
):
    old_bridge = getattr(builtins, old_name, None)
    if old_bridge is not None:
        try:
            old_bridge.shutdown()
        except Exception as exc:
            print(f"清理旧 bridge {old_name} 时出现非致命异常：", exc)

builtins._task11_shared_box_bridge = Task11SharedBoxBridge()
