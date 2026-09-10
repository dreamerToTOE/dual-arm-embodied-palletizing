# Task10：四箱连续码垛的双紧凑吸盘 ROS 2 bridge
#
# Timeline Play 后运行。吸盘参数严格复用 Task07 / Task09 已验证值，
# 仅把 Box Ground Truth 扩展为 [BoxA, BoxB, BoxC, BoxD]。

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
from geometry_msgs.msg import Pose, PoseArray
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool


SIDES = ("left", "right")
BOX_PATHS = (
    "/World/BoxA",
    "/World/BoxB",
    "/World/BoxC",
    "/World/BoxD",
)


class Task10QuadSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")

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
            "task10_quad_suction_bridge", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.command_subs = {}
        self.state_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool,
                f"/task10/{side}/suction_command",
                lambda msg, s=side: self._on_suction_command(s, msg),
                10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool,
                f"/task10/{side}/suction_state",
                10,
            )

        self.pose_pub = self.node.create_publisher(
            PoseArray,
            "/task10/box_poses",
            10,
        )
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task10 四箱双吸盘 ROS bridge 已启动")
        print("LEFT : /task10/left/suction_command  <-> suction_state")
        print("RIGHT: /task10/right/suction_command <-> suction_state")
        print("PUB  : /task10/box_poses [BoxA, BoxB, BoxC, BoxD]")
        print("Surface Gripper 参数复用 Task09 已验证基线")
        print("====================================================")

    def _build_gripper(self, side):
        hand_path = f"/World/{side}_fr3/fr3_hand"
        if not self.stage.GetPrimAtPath(hand_path).IsValid():
            raise RuntimeError(f"缺少 {hand_path}。请先保留 Task06 双臂场景。")

        joint_path = f"{hand_path}/task10_surface_gripper_joint"
        old_joint = self.stage.GetPrimAtPath(joint_path)
        if old_joint.IsValid():
            self.stage.RemovePrim(joint_path)

        props = Surface_Gripper_Properties()
        props.d6JointPath = joint_path
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

        # Task09 已验证的稳定吸盘参数；不要为 Task10 重设物理变量。
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

    def _on_suction_command(self, side, msg):
        with self._lock:
            self._desired[side] = bool(msg.data)

    def _on_physics_step(self, dt):
        with self._lock:
            desired = dict(self._desired)

        for side in SIDES:
            gripper = self.grippers[side]
            want_closed = desired[side]

            if want_closed != self._last_commanded[side]:
                if want_closed:
                    ok = gripper.close()
                    print(
                        f"[Task10 Isaac][{side}] SUCTION ON request, "
                        f"immediate_result={ok}"
                    )
                else:
                    ok = gripper.open()
                    print(
                        f"[Task10 Isaac][{side}] SUCTION OFF request, result={ok}"
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
                            print(f"[Task10 Isaac][{side}] retry -> CLOSED")
                else:
                    self._retry_accum[side] = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_states_and_poses()

    def _publish_states_and_poses(self):
        for side in SIDES:
            message = Bool()
            message.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(message)

        pose_array = PoseArray()
        pose_array.header.stamp = self.node.get_clock().now().to_msg()
        pose_array.header.frame_id = "world"

        for path in BOX_PATHS:
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                return

            world_tf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            translation = world_tf.ExtractTranslation()
            quaternion = world_tf.ExtractRotationQuat()
            imaginary = quaternion.GetImaginary()

            pose = Pose()
            pose.position.x = float(translation[0])
            pose.position.y = float(translation[1])
            pose.position.z = float(translation[2])
            pose.orientation.x = float(imaginary[0])
            pose.orientation.y = float(imaginary[1])
            pose.orientation.z = float(imaginary[2])
            pose.orientation.w = float(quaternion.GetReal())
            pose_array.poses.append(pose)

        self.pose_pub.publish(pose_array)

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


old_bridge = getattr(builtins, "_task10_quad_suction_bridge", None)
if old_bridge is not None:
    try:
        old_bridge.shutdown()
    except Exception as exc:
        print("清理旧 Task10 bridge 时出现非致命异常：", exc)

builtins._task10_quad_suction_bridge = Task10QuadSuctionBridge()
