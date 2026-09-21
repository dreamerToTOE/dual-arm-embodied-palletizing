# Task24-B：短 L 型侧面双吸盘的 Isaac ROS 2 物理桥。
#
# 仅在 Task24 场景 Timeline Play 后运行。Bridge 不选择任务、不规划轨迹，只负责
# 真实侧面 Surface Gripper、八个 Cube Ground Truth 与原子双臂关节下发。

import builtins
import math
import threading

import numpy as np
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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


SIDES = ("left", "right")
BRANCH_SIGN = {"left": -1.0, "right": +1.0}
# Task24 的两个 Surface Gripper 同时约束同一个刚体，构成闭环机构。此前将
# stiffness/damping 都设为 1e10 虽能阻止滑移，却把两个 D6 变成数值上的“无限
# 刚体”；在共同下降中段已实测产生约 3.3 deg 的瞬态 Cube 倾角。
#
# 两个 Surface Gripper 同时约束同一个刚体，形成闭环。这里两侧采用相同的
# 1e4 / 1e3 稳定阻抗量级（Task11 顶吸基线）：不把 D6 伪装成无限刚体，避免
# 微小的机器人跟踪误差在共同下降时被转成瞬态 Cube 扭转。侧面紧协调采用
# Fixed grasp（bendAngle=0）并使用高断裂阈值：共同抬升时单杯受到的约束反力
# 会超过 20 Nm，故不能沿用低阈值。3 mm 捕获阈值和碰撞规则均不改变。
RIGID_GRASP_FORCE_LIMIT = 1.0e6
RIGID_GRASP_TORQUE_LIMIT = 1.0e6
GRASP_STIFFNESS = {"left": 1.0e4, "right": 1.0e4}
GRASP_DAMPING = {"left": 1.0e3, "right": 1.0e3}
# 共同负载阶段不能再由两个独立 Action Graph 各自取消息、各自写控制器。
# 双侧指令带有同一个 task24_dual_sync stamp，Bridge 只在收齐这对消息后，在
# 同一个 physics callback 中写入两个 ArticulationAction；从而将“同一时间参数”
# 真正落实到 PhysX 的下一物理步，而非仅停留在 ROS publish 的先后顺序。
DUAL_SYNC_FRAME_ID = "task24_dual_sync"
TASK_ROOT = "/World/Task24"


def _scene_cube_paths(stage):
    """从场景标记生成 PoseArray 顺序，禁止 Bridge 与场景件数漂移。"""
    task_root = stage.GetPrimAtPath(TASK_ROOT)
    if not task_root.IsValid():
        raise RuntimeError(f"缺少 {TASK_ROOT}。请先运行 Task24 场景脚本。")
    active_count = task_root.GetCustomDataByKey("task24_active_cube_count")
    try:
        active_count = int(active_count)
    except (TypeError, ValueError) as error:
        raise RuntimeError("Task24 场景缺少有效 task24_active_cube_count 标记。") from error
    if not 1 <= active_count <= 8:
        raise RuntimeError(f"Task24 active cube 数量越界：{active_count}（应为 1..8）。")
    return tuple(
        f"{TASK_ROOT}/Supply/Cube_{index:02d}"
        for index in range(1, active_count + 1)
    )


class Task24SideSuctionBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        self.cube_paths = _scene_cube_paths(self.stage)
        missing = [path for path in self.cube_paths if not self.stage.GetPrimAtPath(path).IsValid()]
        if missing:
            raise RuntimeError("缺少 Task24 Supply Cube：" + ", ".join(missing))

        self._lock = threading.Lock()
        self._desired = {side: False for side in SIDES}
        self._last_commanded = {side: None for side in SIDES}
        self._retry_accum = {side: 0.0 for side in SIDES}
        self._publish_accum = 0.0
        self._retry_interval = 0.05
        self.grippers = {side: self._build_gripper(side) for side in SIDES}

        # Scene 中的 Action Graph 仅发布 /left|right/joint_states。所有关节目标都
        # 由本 Bridge 写入，以免 Graph controller 与这里的控制器在同一帧争夺目标。
        self.articulations = {}
        for side in SIDES:
            articulation = SingleArticulation(f"/World/{side}_fr3")
            articulation.initialize()
            if not articulation.handles_initialized:
                raise RuntimeError(f"{side} FR3 Articulation 初始化失败；请确认 Timeline 已 Play。")
            self.articulations[side] = articulation
        self._single_commands = {side: None for side in SIDES}
        self._dual_commands = {side: None for side in SIDES}

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task24_side_suction_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.command_subs = {}
        self.joint_command_subs = {}
        self.state_pubs = {}
        self.tcp_pubs = {}
        for side in SIDES:
            self.command_subs[side] = self.node.create_subscription(
                Bool, f"/task24/{side}/suction_command",
                lambda message, current_side=side: self._command(current_side, message), 10,
            )
            self.joint_command_subs[side] = self.node.create_subscription(
                JointState, f"/{side}/joint_command",
                lambda message, current_side=side: self._joint_command(current_side, message), 20,
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
        print("SUB: /{left,right}/joint_command sensor_msgs/JointState")
        print("PUB: /task24/{left,right}/suction_state std_msgs/Bool")
        print(
            "PUB: /task24/cube_poses geometry_msgs/PoseArray "
            f"(Cube_01 ... Cube_{len(self.cube_paths):02d})"
        )
        print("PUB: /task24/{left,right}/side_suction_tcp_pose geometry_msgs/PoseStamped")
        print(
            "Surface Gripper normal: left +Y, right -Y; "
            "rigid bilateral grasp, retry close only after command ON"
        )
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
        # 接触 TCP 已由执行器送到 Cup 面的名义侧面。Surface Gripper close()
        # 固定当前相对几何，不会自动消除大间隙；3 mm 阈值只保留捕获容差，
        # 不允许 10 mm 浮空吸附。
        props.gripThreshold = 0.003
        props.forceLimit = RIGID_GRASP_FORCE_LIMIT
        props.torqueLimit = RIGID_GRASP_TORQUE_LIMIT
        # Task24 的共同搬运不允许 Cup--Cube 相对转动。0 使 Surface Gripper
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
        with self._lock:
            self._desired[side] = bool(message.data)

    @staticmethod
    def _stamp_key(message):
        stamp = message.header.stamp
        return int(stamp.sec), int(stamp.nanosec)

    def _joint_command(self, side, message):
        if not message.name or len(message.name) != len(message.position):
            print(f"[Task24 Isaac][{side}] ignored malformed joint command")
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
        self._apply_pending_joint_commands()
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
        array.poses = [self._pose(path) for path in self.cube_paths]
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
