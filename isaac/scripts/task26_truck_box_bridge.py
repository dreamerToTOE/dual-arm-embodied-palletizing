# Task24-A：分批到料的 Isaac ROS 2 物理桥（侧吸 + 休眠区瞬移）。
#
# 仅在 Task26 场景 Timeline Play 后运行。Bridge 不选择任务、不规划轨迹，只负责：
#   1. 真实侧面 Surface Gripper；
#   2. 八件 Cube 的 Ground Truth PoseArray；
#   3. 原子双臂关节下发（共同命令必须同 stamp 成对写入）；
#   4. 分批到料：按 /task26/feed_command 把对应批次从休眠位瞬移到供料槽，
#      落稳后通过 /task26/feed_state 报告“已到位且静止”。
#
# 未到货的 Cube 停在桌下休眠位（z=-5 m、已关重力），不进入 MoveIt Planning
# Scene，也不参与任何 FCL 采样。

import builtins
import json
import math
import threading

import numpy as np
import omni.kit.app
import omni.physx
import omni.physics.tensors
import omni.usd
from pxr import PhysxSchema, Usd, UsdGeom

manager = omni.kit.app.get_app().get_extension_manager()
manager.set_extension_enabled_immediate("isaacsim.robot.surface_gripper", True)

from isaacsim.robot.surface_gripper._surface_gripper import (
    Surface_Gripper,
    Surface_Gripper_Properties,
)

import rclpy
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
from isaacsim.core.utils.types import ArticulationAction
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, Int32MultiArray


SIDES = ("left", "right")
BRANCH_SIGN = {"left": -1.0, "right": +1.0}

# 与 Task24 相同的有限阻抗 Side Grasp：两个 D6 同时约束同一个刚体构成闭环，
# 不能用无限刚度伪装刚体。3 mm 捕获阈值、Fixed grasp（bendAngle=0）与高断裂
# 阈值均沿用 Task24 的已验证设置。
RIGID_GRASP_FORCE_LIMIT = 1.0e6
RIGID_GRASP_TORQUE_LIMIT = 1.0e6
GRASP_STIFFNESS = {"left": 1.0e4, "right": 1.0e4}
GRASP_DAMPING = {"left": 1.0e3, "right": 1.0e3}

# 共同负载阶段由同一个 physics callback 原子写入左右目标。Task26 使用独立
# frame_id，避免与 Task24 执行器的消息混入同一对（两个执行器不得同时运行）。
DUAL_SYNC_FRAME_ID = "task26_dual_sync"

TASK_ROOT = "/World/Task26"
TABLE_TOP_Z = 0.200
CUBE_HALF = 0.060
BOTTOM_Z = TABLE_TOP_Z + CUBE_HALF

# 到料判定：位置必须落在槽位附近，且线速度与角速度都接近零并保持一小段时间。
# 这是“到位且静止”的证据，不是等待区；执行器只在本标志为 1 时读取该件 pose。
ARRIVAL_TOLERANCE_XY = 0.002
ARRIVAL_TOLERANCE_Z = 0.001
ARRIVAL_MAX_LINEAR_SPEED = 0.002
ARRIVAL_MAX_ANGULAR_SPEED = 0.05
ARRIVAL_HOLD_SEC = 0.20
# 到料下落高度由场景标记 task26_feed_settle_offset_z 提供，不在这里重复硬编码。

STATE_PARKED = 0
STATE_ARRIVING = 1
STATE_ARRIVED = 2
STATE_LABEL = {STATE_PARKED: "PARKED", STATE_ARRIVING: "ARRIVING", STATE_ARRIVED: "ARRIVED"}


def _task_custom(key):
    task_root = _stage().GetPrimAtPath(TASK_ROOT)
    if not task_root.IsValid():
        raise RuntimeError(f"缺少 {TASK_ROOT}。请先运行 Task26 场景脚本。")
    value = task_root.GetCustomDataByKey(key)
    if value is None:
        raise RuntimeError(f"Task26 场景缺少标记 {key}。")
    return value


def _stage():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("当前没有有效 USD Stage。")
    return stage


def _json_tuple(value):
    return tuple(float(item) for item in json.loads(value))


class Task26BatchedFeedBridge:
    def __init__(self):
        self.stage = _stage()
        active_count = int(_task_custom("task26_active_cube_count"))
        self.batch_size = int(_task_custom("task26_batch_size"))
        self.batch_count = int(_task_custom("task26_batch_count"))
        if active_count != self.batch_size * self.batch_count:
            raise RuntimeError(
                "Task26 场景的件数与批次定义不一致："
                f"{active_count} != {self.batch_size} x {self.batch_count}。"
            )
        self.slot_poses = {
            "A": _json_tuple(_task_custom("task26_slot_a")),
            "B": _json_tuple(_task_custom("task26_slot_b")),
        }
        self.cube_paths = tuple(
            f"{TASK_ROOT}/Supply/Cube_{index:02d}"
            for index in range(1, active_count + 1)
        )
        missing = [
            path for path in self.cube_paths
            if not self.stage.GetPrimAtPath(path).IsValid()
        ]
        if missing:
            raise RuntimeError("缺少 Task26 Supply Cube：" + ", ".join(missing))

        metadata = []
        for path in self.cube_paths:
            raw = self.stage.GetPrimAtPath(path).GetCustomDataByKey("task26_metadata")
            if raw is None:
                raise RuntimeError(f"{path} 缺少 task26_metadata 标记。")
            metadata.append(json.loads(raw))
        self.batch_of = [int(item["batch"]) for item in metadata]
        self.slot_name = [str(item["slot"]) for item in metadata]
        # 奇数排在槽 A（批内先取），偶数排在槽 B（批内后取）。
        self.slot_pose = [self.slot_poses[name] for name in self.slot_name]
        for index, pose in enumerate(self.slot_pose, start=1):
            if abs(pose[2] - BOTTOM_Z) > 1.0e-6:
                raise RuntimeError(
                    f"Cube_{index:02d} 的槽位高度 {pose[2]:.4f} 与本 bridge 的"
                    f"基线 {BOTTOM_Z:.4f} 不一致；场景与 bridge 不得漂移。"
                )
        self.feed_settle_offset_z = float(_task_custom("task26_feed_settle_offset_z"))

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._publish_accum = 0.0
        self._retry_interval = 0.05
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        # 关节目标全部由本 Bridge 写入，避免 Action Graph controller 争抢同一目标。
        self.articulations = {}
        for side in SIDES:
            articulation = SingleArticulation(f"/World/{side}_fr3")
            articulation.initialize()
            if not articulation.handles_initialized:
                raise RuntimeError(
                    f"{side} FR3 Articulation 初始化失败；请确认 Timeline 已 Play。"
                )
            self.articulations[side] = articulation
        self._single_commands = {side: None for side in SIDES}
        self._dual_commands = {side: None for side in SIDES}

        # 刚体句柄：只用于“瞬移到槽位 + 清零速度 + 读取是否静止”。
        # Ground Truth pose 仍由 USD 变换发布，与 Task24 同源。
        self.rigids = []
        for index, path in enumerate(self.cube_paths, start=1):
            rigid = SingleRigidPrim(path, name=f"task26_cube_{index:02d}")
            rigid.initialize()
            self.rigids.append(rigid)
        self.cube_state = [STATE_PARKED] * len(self.cube_paths)
        self.stable_accum = [0.0] * len(self.cube_paths)
        self.released_batches = set()

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task26_batched_feed_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.command_subs = {}
        self.joint_command_subs = {}
        self.state_pubs = {}
        self.tcp_pubs = {}
        self.force_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool, f"/task26/{side}/suction_command",
                lambda message, current_side=side: self._command(current_side, message), 10,
            )
            self.joint_command_subs[side] = self.node.create_subscription(
                JointState, f"/{side}/joint_command",
                lambda message, current_side=side: self._joint_command(current_side, message), 20,
            )
            self.state_pubs[side] = self.node.create_publisher(
                Bool, f"/task26/{side}/suction_state", 10
            )
            self.tcp_pubs[side] = self.node.create_publisher(
                PoseStamped, f"/task26/{side}/side_suction_tcp_pose", 10
            )
            # 关节实测力矩：给执行器的推进力监督提供真实数据（不是估算）。
            self.force_pubs[side] = self.node.create_publisher(
                JointState, f"/task26/{side}/measured_joint_forces", 10
            )
        self.cube_pub = self.node.create_publisher(PoseArray, "/task26/cube_poses", 10)
        self.feed_state_pub = self.node.create_publisher(
            Int32MultiArray, "/task26/feed_state", 10
        )
        self.feed_command_sub = self.node.create_subscription(
            Int32, "/task26/feed_command", self._feed_command, 10
        )

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()
        self._pending_batch = None
        self._ever_commanded = False
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._physics_step
        )

        print("\n====================================================")
        print("Task26 batched-feed side-suction ROS bridge started")
        print("SUB: /task26/feed_command std_msgs/Int32 (batch index 1..%d)" % self.batch_count)
        print("SUB: /task26/{left,right}/suction_command std_msgs/Bool")
        print("SUB: /{left,right}/joint_command sensor_msgs/JointState")
        print("PUB: /task26/feed_state std_msgs/Int32MultiArray (0=parked, 1=arriving, 2=arrived and settled)")
        print(
            "PUB: /task26/cube_poses geometry_msgs/PoseArray "
            f"(Cube_01 ... Cube_{len(self.cube_paths):02d})"
        )
        print("PUB: /task26/{left,right}/suction_state std_msgs/Bool")
        print("PUB: /task26/{left,right}/side_suction_tcp_pose geometry_msgs/PoseStamped")
        print("PUB: /task26/{left,right}/measured_joint_forces sensor_msgs/JointState (effort)")
        print(
            "Surface Gripper normal: left +Y, right -Y; rigid bilateral grasp; "
            "feed batches are teleported from the below-table park pose"
        )
        print("====================================================")
        # 批 1 在 bridge 启动后立即到料，使机制无需额外命令即可观察。
        # 真正的瞬移在下一个 physics callback 内执行。
        self._pending_batch = 1

    def _build_gripper(self, side):
        # 与 Task26 MoveIt URDF 一致：L 型工具挂在 fr3_link8。
        mount_path = f"/World/{side}_fr3/fr3_link8"
        if not self.stage.GetPrimAtPath(mount_path).IsValid():
            raise RuntimeError(f"缺少 {mount_path}。")
        for name in (
            "task11_surface_gripper_joint",
            "task20_surface_gripper_joint",
            "task24_side_surface_gripper_joint",
            "task26_side_surface_gripper_joint",
        ):
            old = self.stage.GetPrimAtPath(f"{mount_path}/{name}")
            if old.IsValid():
                self.stage.RemovePrim(old.GetPath())

        sign = BRANCH_SIGN[side]
        props = Surface_Gripper_Properties()
        props.d6JointPath = f"{mount_path}/task26_side_surface_gripper_joint"
        props.parentPath = mount_path
        offset = omni.physics.tensors.Transform()
        offset.p.x = 0.0
        offset.p.y = sign * 0.155
        offset.p.z = 0.080
        offset.r.x = 0.0
        offset.r.y = 0.0
        offset.r.z = sign * 0.70710678
        offset.r.w = 0.70710678
        props.offset = offset
        props.gripThreshold = 0.003
        props.forceLimit = RIGID_GRASP_FORCE_LIMIT
        props.torqueLimit = RIGID_GRASP_TORQUE_LIMIT
        props.bendAngle = 0.0
        props.stiffness = GRASP_STIFFNESS[side]
        props.damping = GRASP_DAMPING[side]
        props.retryClose = True
        props.disableGravity = False
        gripper = Surface_Gripper()
        if not gripper.initialize(props):
            raise RuntimeError(
                f"{side} 侧面 Surface Gripper 初始化失败；请确认 Timeline 已 Play。"
            )
        return gripper

    # ------------------------------------------------------------------ 到料

    def _set_gravity(self, index, enabled):
        prim = self.stage.GetPrimAtPath(self.cube_paths[index])
        body_api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        body_api.CreateDisableGravityAttr().Set(not enabled)

    def _activate_batch(self, batch, automatic=False):
        with self._lock:
            if not 1 <= batch <= self.batch_count:
                print(f"[Task26 Isaac] feed_command 批次越界：{batch}")
                return False
            if batch in self.released_batches:
                print(f"[Task26 Isaac] 批 {batch} 已释放过，忽略重复命令。")
                return False
            earlier = [
                index for index, value in enumerate(self.batch_of) if value < batch
            ]
            not_arrived = [
                index + 1 for index in earlier
                if self.cube_state[index] != STATE_ARRIVED
            ]
            if not_arrived:
                print(
                    f"[Task26 Isaac] 拒绝释放批 {batch}：更早的 Cube {not_arrived} 尚未"
                    "到位；执行器必须先等上一批完成并退出共同工作区。"
                )
                return False
            indices = [
                index for index, value in enumerate(self.batch_of) if value == batch
            ]
            self.released_batches.add(batch)

        for index in indices:
            target = self.slot_pose[index]
            settle = (
                float(target[0]),
                float(target[1]),
                float(target[2]) + self.feed_settle_offset_z,
            )
            self._set_gravity(index, True)
            self.rigids[index].set_world_pose(
                position=np.asarray(settle, dtype=np.float64),
                orientation=np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            )
            self.rigids[index].set_linear_velocity(np.zeros(3, dtype=np.float64))
            self.rigids[index].set_angular_velocity(np.zeros(3, dtype=np.float64))
            self.cube_state[index] = STATE_ARRIVING
            self.stable_accum[index] = 0.0
            print(
                "[Task26 Isaac] 批 %d%s：Cube_%02d -> 槽 %s %s，"
                "底面高于桌面 %.1f mm，等待落稳" %
                (
                    batch,
                    "（自动）" if automatic else "",
                    index + 1,
                    self.slot_name[index],
                    tuple(round(value, 3) for value in target),
                    self.feed_settle_offset_z * 1000.0,
                )
            )
        return True

    def _feed_command(self, message):
        # ROS 回调只登记请求；真正的瞬移必须在 physics callback 内完成，
        # 不允许从订阅线程直接改动物理刚体。
        with self._lock:
            self._pending_batch = int(message.data)
            self._ever_commanded = True

    def _check_arrival(self, index, dt):
        if self.cube_state[index] != STATE_ARRIVING:
            return
        position, _ = self.rigids[index].get_world_pose()
        linear = np.asarray(self.rigids[index].get_linear_velocity(), dtype=np.float64)
        angular = np.asarray(self.rigids[index].get_angular_velocity(), dtype=np.float64)
        target = self.slot_pose[index]
        settled = (
            abs(float(position[0]) - target[0]) <= ARRIVAL_TOLERANCE_XY
            and abs(float(position[1]) - target[1]) <= ARRIVAL_TOLERANCE_XY
            and abs(float(position[2]) - BOTTOM_Z) <= ARRIVAL_TOLERANCE_Z
            and float(np.linalg.norm(linear)) <= ARRIVAL_MAX_LINEAR_SPEED
            and float(np.linalg.norm(angular)) <= ARRIVAL_MAX_ANGULAR_SPEED
        )
        if not settled:
            self.stable_accum[index] = 0.0
            return
        self.stable_accum[index] += float(dt)
        if self.stable_accum[index] >= ARRIVAL_HOLD_SEC:
            self.cube_state[index] = STATE_ARRIVED
            print(
                "[Task26 Isaac] Cube_%02d 到位且静止：pos=(%.3f, %.3f, %.3f)，"
                "|v|=%.4f m/s，|w|=%.4f rad/s" %
                (
                    index + 1,
                    float(position[0]), float(position[1]), float(position[2]),
                    float(np.linalg.norm(linear)), float(np.linalg.norm(angular)),
                )
            )

    # ------------------------------------------------------------------ 关节

    def _command(self, side, message):
        with self._lock:
            self._desired[side] = bool(message.data)

    @staticmethod
    def _stamp_key(message):
        stamp = message.header.stamp
        return int(stamp.sec), int(stamp.nanosec)

    def _joint_command(self, side, message):
        if not message.name or len(message.name) != len(message.position):
            print(f"[Task26 Isaac][{side}] ignored malformed joint command")
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
                    self._dual_commands["left"] = None
                else:
                    self._dual_commands["right"] = None

        for side in SIDES:
            if single[side] is not None:
                self._apply_joint_command(side, single[side])
        if dual is not None:
            self._apply_joint_command("left", dual[0])
            self._apply_joint_command("right", dual[1])

    # ------------------------------------------------------------- physics

    def _physics_step(self, dt):
        self._apply_pending_joint_commands()
        with self._lock:
            desired = dict(self._desired)
            pending_batch = self._pending_batch
            self._pending_batch = None
            ever_commanded = self._ever_commanded
        if pending_batch is not None:
            self._activate_batch(pending_batch, automatic=not ever_commanded)
        for side in SIDES:
            gripper = self.grippers[side]
            if desired[side] != self._last_commanded[side]:
                ok = gripper.close() if desired[side] else gripper.open()
                print(
                    f"[Task26 Isaac][{side}] SUCTION "
                    f"{'ON' if desired[side] else 'OFF'}, result={ok}"
                )
                self._last_commanded[side] = desired[side]
                self._retry_accum[side] = 0.0
            if desired[side]:
                gripper.update()
                if not gripper.is_closed():
                    self._retry_accum[side] += float(dt)
                    if self._retry_accum[side] >= self._retry_interval:
                        self._retry_accum[side] = 0.0
                        if gripper.close():
                            print(f"[Task26 Isaac][{side}] side suction retry -> CLOSED")
            else:
                self._retry_accum[side] = 0.0
        for index in range(len(self.cube_paths)):
            self._check_arrival(index, dt)
        self._publish_accum += float(dt)
        if self._publish_accum >= 0.05:
            self._publish_accum = 0.0
            self._publish()

    def _measured_joint_efforts(self, side):
        """读被测关节力/力矩。不同 Isaac 版本 API 名不同，两种都试。"""
        articulation = self.articulations[side]
        values = None
        try:
            values = articulation.get_measured_joint_efforts()
        except Exception:
            try:
                values = articulation.get_measured_joint_forces()
            except Exception:
                return None
        if values is None:
            return None
        array = np.asarray(values, dtype=np.float64)
        if array.ndim == 2:
            array = array[:, 0]
        return array.reshape(-1)

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
        feed_state = Int32MultiArray()
        feed_state.data = [
            STATE_ARRIVED if state == STATE_ARRIVED else 0
            for state in self.cube_state
        ]
        self.feed_state_pub.publish(feed_state)
        for side in SIDES:
            state = Bool()
            state.data = bool(self.grippers[side].is_closed())
            self.state_pubs[side].publish(state)
            tcp = PoseStamped()
            tcp.header.stamp = stamp
            tcp.header.frame_id = "world"
            tcp.pose = self._pose(f"/World/{side}_fr3/fr3_link8/side_suction_tool/side_suction_tcp")
            self.tcp_pubs[side].publish(tcp)
            efforts = self._measured_joint_efforts(side)
            if efforts is not None and efforts.size > 0:
                forces = JointState()
                forces.header.stamp = stamp
                forces.header.frame_id = "world"
                forces.name = ["fr3_joint%d" % index
                               for index in range(1, int(efforts.size) + 1)]
                forces.effort = [float(value) for value in efforts]
                self.force_pubs[side].publish(forces)

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
    "_task26_batched_feed_bridge",
):
    old = getattr(builtins, name, None)
    if old is not None:
        try:
            old.shutdown()
        except Exception as exc:
            print("旧 bridge 清理的非致命异常：", exc)

builtins._task26_batched_feed_bridge = Task26BatchedFeedBridge()
