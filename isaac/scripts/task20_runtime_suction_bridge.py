# Task20-C：运行时松/紧协调执行的双 Surface Gripper bridge。
#
# 前置条件：
#   1. Timeline Stop 时运行 task18_randomized_scene.py；
#   2. Timeline Play；
#   3. 运行 task18_ground_truth_bridge.py 发布 /task18/box_states；
#   4. 再运行本文件。
#
# 本 bridge 不包含任何路由或轨迹规划。它只提供已验收的 Task11--13 / Task15
# Surface Gripper 物理参数、/task20 吸盘状态、suction TCP Ground Truth，以及
# 已落稳物体的可选 kinematic 支撑锁。所有 Box id 均从 Task18 metadata 动态发现。

import builtins
import math
import threading

import omni.kit.app
import omni.physx
import omni.physics.tensors
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

extension_manager = omni.kit.app.get_app().get_extension_manager()
extension_manager.set_extension_enabled_immediate(
    "isaacsim.robot.surface_gripper", True
)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool, String


SIDES = ("left", "right")
TASK18_PREFIX = "/World/Task18Box_"


class Task20RuntimeSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        self.box_paths = self._discover_box_paths()
        if not self.box_paths:
            raise RuntimeError(
                "没有发现 /World/Task18Box_*。请先运行 task18_randomized_scene.py。"
            )

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._retry_interval = 0.05
        self._publish_accum = 0.0
        self._requested_locks = set()
        self._locked_objects = set()
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task20_runtime_suction_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)

        self.command_subs = {}
        self.state_pubs = {}
        self.tcp_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool,
                f"/task20/{side}/suction_command",
                lambda message, s=side: self._on_suction_command(s, message),
                10,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool,
                f"/task20/{side}/suction_state",
                10,
            )
            self.tcp_pubs[side] = self.node.create_publisher(
                PoseStamped,
                f"/task20/{side}/suction_tcp_pose",
                10,
            )
        self.lock_sub = self.node.create_subscription(
            String,
            "/task20/lock_placed_object",
            self._on_lock_request,
            10,
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        print("")
        print("====================================================")
        print("Task20-C runtime dual-suction bridge started")
        print("LEFT : /task20/left/suction_command  <-> suction_state")
        print("RIGHT: /task20/right/suction_command <-> suction_state")
        print("PUB  : /task20/{left,right}/suction_tcp_pose PoseStamped")
        print("SUB  : /task20/lock_placed_object std_msgs/String")
        print("Box Ground Truth remains on /task18/box_states via task18_ground_truth_bridge.py")
        print(f"Runtime Box ids = {sorted(self.box_paths)}")
        print("Surface Gripper: threshold=3 mm, force/torque=1e6, retry=50 ms")
        print("====================================================")

    def _discover_box_paths(self):
        found = {}
        for prim in self.stage.Traverse():
            path = prim.GetPath().pathString
            if not path.startswith(TASK18_PREFIX):
                continue
            metadata = prim.GetCustomDataByKey("task18_metadata")
            if not metadata:
                continue
            import json

            try:
                object_id = str(json.loads(str(metadata))["id"])
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"Task18 metadata 无法读取 object id：{path}") from exc
            if not object_id or object_id in found:
                raise RuntimeError(f"Task18 Box id 无效或重复：{object_id}")
            found[object_id] = path
        return found

    def _build_gripper(self, side):
        hand_path = f"/World/{side}_fr3/fr3_hand"
        if not self.stage.GetPrimAtPath(hand_path).IsValid():
            raise RuntimeError(f"缺少 {hand_path}。请先保留 Task06 双 FR3 基线。")

        # 一个末端只能有一套 D6 Surface Gripper Joint。只清理本项目历史 bridge
        # 的 joint，不修改机器人、Table、ActionGraph 或 Task18 Box。
        for joint_name in (
            "task07_surface_gripper_joint",
            "task10_surface_gripper_joint",
            "task11_surface_gripper_joint",
            "task15_surface_gripper_joint",
            "task20_surface_gripper_joint",
        ):
            old_joint = self.stage.GetPrimAtPath(f"{hand_path}/{joint_name}")
            if old_joint.IsValid():
                self.stage.RemovePrim(old_joint.GetPath())

        props = Surface_Gripper_Properties()
        props.d6JointPath = f"{hand_path}/task20_surface_gripper_joint"
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

        # 严格复用 Task11--13 / Task15 已验收的物理参数，不通过扩大 threshold
        # 或关闭重力掩盖共同搬运误差。
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
                f"{side} Surface Gripper 初始化失败。请先点击 Timeline Play。"
            )
        return gripper

    def _on_suction_command(self, side, message):
        with self._lock:
            self._desired[side] = bool(message.data)

    def _on_lock_request(self, message):
        object_id = str(message.data)
        if object_id not in self.box_paths:
            print(f"[Task20 Isaac] Ignore lock for unknown object: {object_id}")
            return
        with self._lock:
            self._requested_locks.add(object_id)

    def _apply_requested_locks(self):
        with self._lock:
            requested = set(self._requested_locks)
        for object_id in requested:
            if object_id in self._locked_objects:
                continue
            prim = self.stage.GetPrimAtPath(self.box_paths[object_id])
            if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                print(f"[Task20 Isaac] Cannot lock non-rigid object: {object_id}")
                continue
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(True)
            self._locked_objects.add(object_id)
            pose = self._ground_truth_pose(self.box_paths[object_id]).pose
            print(
                "[Task20 Isaac] STABLE SUPPORT LOCKED: "
                f"{object_id} at ({pose.position.x:.4f}, {pose.position.y:.4f}, "
                f"{pose.position.z:.4f})"
            )

    def _on_physics_step(self, dt):
        with self._lock:
            desired = dict(self._desired)
        self._apply_requested_locks()

        for side in SIDES:
            gripper = self.grippers[side]
            want_closed = desired[side]
            if want_closed != self._last_commanded[side]:
                result = gripper.close() if want_closed else gripper.open()
                print(
                    f"[Task20 Isaac][{side}] SUCTION "
                    f"{'ON' if want_closed else 'OFF'} request, result={result}"
                )
                self._last_commanded[side] = want_closed
                self._retry_accum[side] = 0.0

            # ROS 端只会在 CONTACT 后发出 ON；保留重试处理 PhysX 接触与 DDS
            # 调度间的细小时间差，不会在下降途中提前吸附。
            if want_closed:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        if gripper.close():
                            print(f"[Task20 Isaac][{side}] retry -> CLOSED")
                else:
                    self._retry_accum[side] = 0.0

        self._publish_accum += float(dt)
        if self._publish_accum >= 0.10:
            self._publish_accum = 0.0
            self._publish_states_and_tcp()

    def _publish_states_and_tcp(self):
        for side in SIDES:
            state = Bool()
            state.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(state)
            tcp_path = f"/World/{side}_fr3/fr3_hand/suction_tool/suction_tcp"
            self.tcp_pubs[side].publish(self._ground_truth_pose(tcp_path))

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
        for gripper in self.grippers.values():
            try:
                gripper.open()
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
    "_task20_runtime_suction_bridge",
):
    old_bridge = getattr(builtins, old_name, None)
    if old_bridge is not None:
        try:
            old_bridge.shutdown()
        except Exception as exc:
            print(f"清理旧 bridge {old_name} 时出现非致命异常：", exc)

builtins._task20_runtime_suction_bridge = Task20RuntimeSuctionBridge()
