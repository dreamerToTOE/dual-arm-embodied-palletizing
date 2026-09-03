# Task 04：Isaac Surface Gripper <-> ROS 2 桥接
# 使用方式：
# 1) 先运行 task04_three_cube_scene.py；
# 2) 保存场景并点击 Isaac Sim Play；
# 3) 再在 Script Editor 运行本脚本；
# 4) ROS 侧发布 /task04/suction_command (std_msgs/Bool) 控制吸附。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.usd
import omni.physics.tensors

from pxr import Usd, UsdGeom

# 确保 Surface Gripper 扩展开启
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


class Task04SuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self._lock = threading.Lock()
        self._desired_closed = False
        self._last_commanded = None
        self._publish_accum = 0.0

        # ----------------------------------------------------
        # Surface Gripper
        # ----------------------------------------------------
        props = Surface_Gripper_Properties()
        props.d6JointPath = "/World/fr3/fr3_hand/task04_surface_gripper_joint"
        props.parentPath = "/World/fr3/fr3_hand"

        # Surface Gripper 默认沿 offset pose 的局部 +X 方向寻找物体。
        # 当前 suction_tool 沿 fr3_hand 局部 +Z 伸出，因此将 +X 旋转到 +Z。
        # 0.105 m 与 Franka cobot_pump 的工具 TCP 偏置对齐。
        offset = omni.physics.tensors.Transform()
        offset.p.x = 0.0
        offset.p.y = 0.0
        offset.p.z = 0.1050
        offset.r.x = 0.0
        offset.r.y = -0.70710678
        offset.r.z = 0.0
        offset.r.w = 0.70710678
        props.offset = offset

        props.gripThreshold = 0.012
        props.forceLimit = 200.0
        props.torqueLimit = 20.0
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
        # 独立 rclpy Context，避免污染 Isaac ROS2 Bridge 自身上下文
        # ----------------------------------------------------
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task04_isaac_suction_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.suction_sub = self.node.create_subscription(
            Bool,
            "/task04/suction_command",
            self._on_suction_command,
            10,
        )

        self.suction_state_pub = self.node.create_publisher(
            Bool,
            "/task04/suction_state",
            10,
        )

        self.pose_pub = self.node.create_publisher(
            PoseArray,
            "/task04/cube_poses",
            10,
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        # Surface_Gripper.update() 必须在物理步中持续调用。
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task04 Surface Gripper ROS bridge 已启动")
        print("SUB : /task04/suction_command  std_msgs/Bool")
        print("PUB : /task04/suction_state    std_msgs/Bool")
        print("PUB : /task04/cube_poses       geometry_msgs/PoseArray")
        print("Cube PoseArray 顺序固定为 Cube1, Cube2, Cube3")
        print("====================================================")

    def _on_suction_command(self, msg):
        with self._lock:
            self._desired_closed = bool(msg.data)

    def _on_physics_step(self, dt):
        with self._lock:
            desired = self._desired_closed

        # 只在命令发生切换时触发 open/close。
        if desired != self._last_commanded:
            if desired:
                ok = self.gripper.close()
                print(f"[Task04 Isaac] SUCTION ON request, immediate_result={ok}")
            else:
                ok = self.gripper.open()
                print(f"[Task04 Isaac] SUCTION OFF request, result={ok}")
            self._last_commanded = desired

        # 官方 Surface Gripper 要求闭合/尝试闭合期间每物理步 update。
        if desired:
            self.gripper.update()

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_state_and_cube_poses()

    def _publish_state_and_cube_poses(self):
        state_msg = Bool()
        state_msg.data = bool(self.gripper.is_closed())
        self.suction_state_pub.publish(state_msg)

        pose_array = PoseArray()
        pose_array.header.stamp = self.node.get_clock().now().to_msg()
        pose_array.header.frame_id = "base"

        for path in ["/World/Cube1", "/World/Cube2", "/World/Cube3"]:
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


# Script Editor 重复运行时，先清理旧实例。
old_bridge = getattr(builtins, "_task04_suction_bridge", None)
if old_bridge is not None:
    try:
        old_bridge.shutdown()
    except Exception as exc:
        print("清理旧 Task04 bridge 时出现非致命异常：", exc)

builtins._task04_suction_bridge = Task04SuctionBridge()
