# Task07：双 compact suction <-> ROS 2 bridge
#
# 复用 Task04 已验证 Surface Gripper 参数；只把单臂 bridge 参数化成左右两套。
# Timeline Play 后运行。

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
BOX_PATHS = ("/World/BoxA", "/World/BoxB")


class Task07DualSuctionBridge:
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

        self.grippers = {
            side: self._build_gripper(side)
            for side in SIDES
        }

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            "task07_dual_suction_bridge", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.command_subs = {}
        self.state_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool,
                f"/task07/{side}/suction_command",
                lambda msg, s=side: self._on_suction_command(s, msg),
                10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool,
                f"/task07/{side}/suction_state",
                10,
            )

        self.pose_pub = self.node.create_publisher(
            PoseArray,
            "/task07/box_poses",
            10,
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task07 双吸盘 ROS bridge 已启动")
        print("LEFT : /task07/left/suction_command  <-> suction_state")
        print("RIGHT: /task07/right/suction_command <-> suction_state")
        print("PUB  : /task07/box_poses  [BoxA, BoxB]")
        print("Surface Gripper 参数严格复用 Task04 已验证基线")
        print("====================================================")

    def _build_gripper(self, side):
        hand_path = f"/World/{side}_fr3/fr3_hand"
        hand = self.stage.GetPrimAtPath(hand_path)
        if not hand.IsValid():
            raise RuntimeError(f"缺少 {hand_path}。请先保留 Task06 双臂场景。")

        joint_path = f"{hand_path}/task07_surface_gripper_joint"
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
                f"{side} Surface Gripper 初始化失败。"
                "请确认 Timeline 已 Play。"
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
                        f"[Task07 Isaac][{side}] SUCTION ON request, "
                        f"immediate_result={ok}"
                    )
                else:
                    ok = gripper.open()
                    print(
                        f"[Task07 Isaac][{side}] SUCTION OFF request, result={ok}"
                    )
                self._last_commanded[side] = want_closed
                self._retry_accum[side] = 0.0

            if want_closed:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        ok = gripper.close()
                        if ok:
                            print(f"[Task07 Isaac][{side}] retry -> CLOSED")
                else:
                    self._retry_accum[side] = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_states_and_poses()

    def _publish_states_and_poses(self):
        for side in SIDES:
            msg = Bool()
            msg.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(msg)

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
            p = world_tf.ExtractTranslation()
            q = world_tf.ExtractRotationQuat()
            qi = q.GetImaginary()

            pose = Pose()
            pose.position.x = float(p[0])
            pose.position.y = float(p[1])
            pose.position.z = float(p[2])
            pose.orientation.x = float(qi[0])
            pose.orientation.y = float(qi[1])
            pose.orientation.z = float(qi[2])
            pose.orientation.w = float(q.GetReal())
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


old_bridge = getattr(builtins, "_task07_dual_suction_bridge", None)
if old_bridge is not None:
    try:
        old_bridge.shutdown()
    except Exception as exc:
        print("清理旧 Task07 bridge 时出现非致命异常：", exc)

builtins._task07_dual_suction_bridge = Task07DualSuctionBridge()
