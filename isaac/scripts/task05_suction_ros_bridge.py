# Task 05：Isaac Surface Gripper <-> ROS 2 桥接（B-A-C）
# 使用方式：
# 1) 先运行 task05_bac_suction_scene.py；
# 2) 保存场景并点击 Isaac Sim Play；
# 3) 再在 Script Editor 运行本脚本；
# 4) ROS 侧通过 /task05/* 与吸盘和 BoxA/BoxB/BoxC Ground Truth 通信。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.usd
import omni.physics.tensors

from pxr import Usd, UsdGeom

ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("isaacsim.robot.surface_gripper", True)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseArray


BOX_PATHS = ["/World/BoxA", "/World/BoxB", "/World/BoxC"]


class Task05BACSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self._lock = threading.Lock()
        self._desired_closed = False
        self._last_commanded = None
        self._publish_accum = 0.0

        # 沿用 Task04-B 已验证机制：CONTACT 后才发 SUCTION ON；若第一次未闭合，
        # desired=True 时每 50 ms 主动重试 close()。
        self._retry_accum = 0.0
        self._retry_interval = 0.05

        # ----------------------------------------------------
        # Surface Gripper
        # ----------------------------------------------------
        props = Surface_Gripper_Properties()
        props.d6JointPath = "/World/fr3/fr3_hand/task05_surface_gripper_joint"
        props.parentPath = "/World/fr3/fr3_hand"

        # Surface Gripper 默认沿 offset pose 局部 +X 搜索。
        # 将 +X 旋转到 fr3_hand 局部 +Z，并与 cobot_pump TCP 的 0.105 m 偏置对齐。
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
        # Task04-B 稳定性实验：
        # 使用极高断裂阈值，将 Surface Gripper 近似视为不可断吸盘。
        props.forceLimit = 1.0e6
        props.torqueLimit = 1.0e6
        props.bendAngle = math.radians(15.0)
        props.stiffness = 1.0e4
        props.damping = 1.0e3
        props.retryClose = True
        props.disableGravity = False

        self.gripper = Surface_Gripper()
        if not self.gripper.initialize(props):
            raise RuntimeError(
                "Surface Gripper 初始化失败。请确认 Isaac 已点击 Play，"
                "且 /World/fr3/fr3_hand 是有效刚体。"
            )

        # ----------------------------------------------------
        # 独立 ROS 2 Context
        # ----------------------------------------------------
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(
            "task05_isaac_suction_bridge", context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.suction_sub = self.node.create_subscription(
            Bool,
            "/task05/suction_command",
            self._on_suction_command,
            10,
        )

        self.suction_state_pub = self.node.create_publisher(
            Bool,
            "/task05/suction_state",
            10,
        )

        self.pose_pub = self.node.create_publisher(
            PoseArray,
            "/task05/box_poses",
            10,
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task05 B-A-C Surface Gripper ROS bridge 已启动")
        print("SUB : /task05/suction_command  std_msgs/Bool")
        print("PUB : /task05/suction_state    std_msgs/Bool")
        print("PUB : /task05/box_poses        geometry_msgs/PoseArray")
        print("PoseArray 顺序固定为 BoxA, BoxB, BoxC")
        print("====================================================")

    def _on_suction_command(self, msg):
        with self._lock:
            self._desired_closed = bool(msg.data)

    def _on_physics_step(self, dt):
        with self._lock:
            desired = self._desired_closed

        # 命令切换时立即尝试一次。
        if desired != self._last_commanded:
            if desired:
                ok = self.gripper.close()
                print(
                    f"[Task05 Isaac] SUCTION ON request, "
                    f"immediate_result={ok}"
                )
                self._retry_accum = 0.0
            else:
                ok = self.gripper.open()
                print(
                    f"[Task05 Isaac] SUCTION OFF request, result={ok}"
                )
                self._retry_accum = 0.0

            self._last_commanded = desired

        # SUCTION ON 后每个 physics step 更新；未吸住时周期性主动重试 close()。
        if desired:
            self.gripper.update()

            if not self.gripper.is_closed():
                self._retry_accum += float(dt)

                if self._retry_accum >= self._retry_interval:
                    self._retry_accum = 0.0
                    ok = self.gripper.close()
                    if ok:
                        print(
                            "[Task05 Isaac] "
                            "SUCTION retry succeeded -> CLOSED"
                        )
            else:
                self._retry_accum = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_state_and_box_poses()

    def _publish_state_and_box_poses(self):
        state_msg = Bool()
        state_msg.data = bool(self.gripper.is_closed())
        self.suction_state_pub.publish(state_msg)

        pose_array = PoseArray()
        pose_array.header.stamp = self.node.get_clock().now().to_msg()
        pose_array.header.frame_id = "base"

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


old_bridge = getattr(builtins, "_task05_suction_bridge", None)
if old_bridge is not None:
    try:
        old_bridge.shutdown()
    except Exception as exc:
        print("清理旧 Task05 bridge 时出现非致命异常：", exc)

builtins._task05_suction_bridge = Task05BACSuctionBridge()
