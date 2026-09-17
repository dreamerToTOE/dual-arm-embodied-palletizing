# Task24-B：短 L 型侧面双吸盘的 Isaac ROS 2 物理桥。
#
# 仅在 Task24 场景 Timeline Play 后运行。当前单 Cube 基线只发布 Cube_01；
# Bridge 不选择任务、不规划轨迹，只负责真实侧面 Surface Gripper 与 Ground Truth。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.physics.tensors
import omni.usd
from pxr import Usd, UsdGeom

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.robot.surface_gripper", True)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool


SIDES = ("left", "right")
BRANCH_SIGN = {"left": -1.0, "right": +1.0}
# Task24-F：必须和 Scene 的 ACTIVE_CUBE_COUNT=1、执行器的 active_cube_count:=1
# 一致。多 Cube 阶段恢复前不发布任何不存在或未同步的 Cube。
CUBE_PATHS = ("/World/Task24/Supply/Cube_01",)


class Task24SideSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        missing = [path for path in CUBE_PATHS if not self.stage.GetPrimAtPath(path).IsValid()]
        if missing:
            raise RuntimeError("缺少 Task24 Supply Cube：" + ", ".join(missing))

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._publish_accum = 0.0
        self._retry_interval = 0.05
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task24_side_suction_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.command_subs = {}
        self.state_pubs = {}
        self.tcp_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool, f"/task24/{side}/suction_command",
                lambda message, current_side=side: self._command(current_side, message), 10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool, f"/task24/{side}/suction_state", 10
            )
            self.tcp_pubs[side] = self.node.create_publisher(
                PoseStamped, f"/task24/{side}/side_suction_tcp_pose", 10
            )
        self.cube_pub = self.node.create_publisher(PoseArray, "/task24/cube_poses", 10)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._physics_step
        )

        print("\n====================================================")
        print("Task24 side-suction tight ROS bridge started")
        print("SUB: /task24/{left,right}/suction_command std_msgs/Bool")
        print("PUB: /task24/{left,right}/suction_state std_msgs/Bool")
        print("PUB: /task24/cube_poses geometry_msgs/PoseArray (Cube_01 only)")
        print("PUB: /task24/{left,right}/side_suction_tcp_pose geometry_msgs/PoseStamped")
        print("Surface Gripper normal: left +Y, right -Y; retry close only after command ON")
        print("====================================================")

    def _build_gripper(self, side):
        # 与 Task24 MoveIt URDF 完全一致：L 型工具的安装 frame 是 fr3_link8，
        # 而非 Isaac 官方资产中带额外固定旋转的 fr3_hand。
        mount_path = f"/World/{side}_fr3/fr3_link8"
        if not self.stage.GetPrimAtPath(mount_path).IsValid():
            raise RuntimeError(f"缺少 {mount_path}。")
        for name in ("task11_surface_gripper_joint", "task20_surface_gripper_joint", "task24_side_surface_gripper_joint"):
            old = self.stage.GetPrimAtPath(f"{mount_path}/{name}")
            if old.IsValid():
                self.stage.RemovePrim(old.GetPath())

        sign = BRANCH_SIGN[side]
        props = Surface_Gripper_Properties()
        props.d6JointPath = f"{mount_path}/task24_side_surface_gripper_joint"
        props.parentPath = mount_path
        # 默认搜索方向为 offset 局部 +X。Rz(sign*90°) 将其旋到 L 型横臂方向；
        # left 的杯面指向 +Y、right 的杯面指向 -Y（世界方向由对应 TCP pose 保证）。
        offset = omni.physics.tensors.Transform()
        offset.p.x = 0.0
        # 与 Task24-E L 型阵列、MoveIt URDF 的 side_suction_tcp 完全一致：
        # 80 mm 竖直段 + 130 mm 横向段，Cup 接触 TCP 为 side=155 mm、z=80 mm。
        offset.p.y = sign * 0.155
        offset.p.z = 0.080
        offset.r.x = 0.0
        offset.r.y = 0.0
        offset.r.z = sign * 0.70710678
        offset.r.w = 0.70710678
        props.offset = offset
        # 与执行器的 10 mm 侧向真空近接触间隙对应。阈值略大于间隙，避免
        # 大 Cube 在两侧实体 Cup 挤压后才建立约束。
        props.gripThreshold = 0.012
        props.forceLimit = 1.0e6
        props.torqueLimit = 1.0e6
        props.bendAngle = math.radians(15.0)
        props.stiffness = 1.0e4
        props.damping = 1.0e3
        props.retryClose = True
        props.disableGravity = False
        gripper = Surface_Gripper()
        if not gripper.initialize(props):
            raise RuntimeError(f"{side} 侧面 Surface Gripper 初始化失败；请确认 Timeline 已 Play。")
        return gripper

    def _command(self, side, message):
        with self._lock:
            self._desired[side] = bool(message.data)

    def _physics_step(self, dt):
        with self._lock:
            desired = dict(self._desired)
        for side in SIDES:
            gripper = self.grippers[side]
            if desired[side] != self._last_commanded[side]:
                ok = gripper.close() if desired[side] else gripper.open()
                print(f"[Task24 Isaac][{side}] SUCTION {'ON' if desired[side] else 'OFF'}, result={ok}")
                self._last_commanded[side] = desired[side]
                self._retry_accum[side] = 0.0
            if desired[side]:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        if gripper.close():
                            print(f"[Task24 Isaac][{side}] side suction retry -> CLOSED")
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
        array.poses = [self._pose(path) for path in CUBE_PATHS]
        self.cube_pub.publish(array)
        for side in SIDES:
            state = Bool()
            state.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(state)
            tcp = PoseStamped()
            tcp.header.stamp = stamp
            tcp.header.frame_id = "world"
            tcp.pose = self._pose(f"/World/{side}_fr3/fr3_link8/side_suction_tool/side_suction_tcp")
            self.tcp_pubs[side].publish(tcp)

    def shutdown(self):
        for side in SIDES:
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


for name in (
    "_task11_shared_box_bridge",
    "_task20_runtime_suction_bridge",
    "_task23_gripper_ground_truth_bridge",
    "_task24_side_suction_bridge",
):
    old = getattr(builtins, name, None)
    if old is not None:
        try:
            old.shutdown()
        except Exception as exc:
            print("旧 bridge 清理的非致命异常：", exc)

builtins._task24_side_suction_bridge = Task24SideSuctionBridge()
