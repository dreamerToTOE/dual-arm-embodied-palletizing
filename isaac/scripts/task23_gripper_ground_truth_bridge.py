# Task23：仅发布 16 个供料 Cube 与双手 TCP 的 Isaac Ground Truth。
#
# 必须先运行 task23_gripper_push_stack_scene.py 并点击 Play。该 bridge 不控制
# 机器人、不执行夹爪命令；它是 Python selector 与 C++ 执行器共享的只读状态源。

import builtins
import json
import math

import omni.physx
import omni.usd
from pxr import Usd, UsdGeom

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

try:
    from fr3_dual_palletize.msg import BoxState, BoxStateArray
except ImportError as exc:
    raise RuntimeError(
        "无法导入 fr3_dual_palletize.msg。请从已编译并 source 的 ros_ws/install 环境启动 Isaac Sim。"
    ) from exc


TASK23_ROOT = "/World/Task23/Supply/"
TCP_PATHS = {
    "left": "/World/left_fr3/fr3_hand_tcp",
    "right": "/World/right_fr3/fr3_hand_tcp",
}
PUBLISH_PERIOD_SEC = 0.05
SEED = 20260923


def pose_from_transform(transform):
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
        raise RuntimeError("Task23 USD transform 的 quaternion 无效。")
    pose = BoxState().pose
    pose.position.x = float(position[0])
    pose.position.y = float(position[1])
    pose.position.z = float(position[2])
    pose.orientation.x = float(imaginary[0]) / norm
    pose.orientation.y = float(imaginary[1]) / norm
    pose.orientation.z = float(imaginary[2]) / norm
    pose.orientation.w = float(quaternion.GetReal()) / norm
    return pose


class Task23GroundTruthBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        self.boxes = self._discover_supply_or_raise()
        for path in TCP_PATHS.values():
            if not self.stage.GetPrimAtPath(path).IsValid():
                raise RuntimeError(f"缺少 {path}；Task23 FR3 hand TCP 未创建。")

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task23_gripper_ground_truth_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.box_publisher = self.node.create_publisher(BoxStateArray, "/task23/box_states", 20)
        self.tcp_publishers = {
            side: self.node.create_publisher(PoseStamped, f"/task23/{side}/hand_tcp_pose", 20)
            for side in TCP_PATHS
        }
        self.publish_accum = 0.0
        self.physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )
        self._publish()
        print("")
        print("====================================================")
        print("Task23 gripper Ground Truth bridge started")
        print("PUB: /task23/box_states fr3_dual_palletize/msg/BoxStateArray")
        print("PUB: /task23/left/hand_tcp_pose geometry_msgs/PoseStamped")
        print("PUB: /task23/right/hand_tcp_pose geometry_msgs/PoseStamped")
        print("This bridge is read-only: no joint or gripper command is published.")
        print("====================================================")

    def _discover_supply_or_raise(self):
        discovered = []
        for prim in self.stage.Traverse():
            path = prim.GetPath().pathString
            if not path.startswith(TASK23_ROOT):
                continue
            raw = prim.GetCustomDataByKey("task23_metadata")
            if not raw:
                continue
            try:
                metadata = json.loads(str(raw))
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"Task23 metadata 无法解析：{path}") from exc
            if metadata.get("role") != "supply_dynamic":
                continue
            required = ("id", "size", "mass", "allowed_modes")
            missing = [key for key in required if key not in metadata]
            if missing:
                raise RuntimeError(f"Task23 supply metadata 不完整：{path} 缺少 {missing}")
            discovered.append((path, metadata))
        if len(discovered) != 16:
            raise RuntimeError(
                "Task23 必须发现恰好 16 个 Dynamic supply Cube；实际为 %d。" % len(discovered)
            )
        return sorted(discovered, key=lambda item: item[1]["id"])

    def _world_pose(self, path):
        prim = self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            raise RuntimeError(f"Task23 Prim 已丢失：{path}")
        return pose_from_transform(
            UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        )

    def _publish(self):
        now = self.node.get_clock().now().to_msg()
        boxes = BoxStateArray()
        boxes.header.stamp = now
        boxes.header.frame_id = "world"
        boxes.seed = SEED
        for path, metadata in self.boxes:
            state = BoxState()
            state.stamp = now
            state.id = str(metadata["id"])
            state.pose = self._world_pose(path)
            state.size = [float(value) for value in metadata["size"]]
            state.mass = float(metadata["mass"])
            state.payload_class = str(metadata.get("payload_class", "small_cube"))
            state.allowed_modes = [str(value) for value in metadata["allowed_modes"]]
            state.grasp_candidate_ids = ["center_top"]
            local_grasp = BoxState().pose
            local_grasp.position.z = 0.5 * state.size[2]
            local_grasp.orientation.w = 1.0
            state.grasp_candidates = [local_grasp]
            boxes.boxes.append(state)
        self.box_publisher.publish(boxes)

        for side, path in TCP_PATHS.items():
            message = PoseStamped()
            message.header.stamp = now
            message.header.frame_id = "world"
            message.pose = self._world_pose(path)
            self.tcp_publishers[side].publish(message)

    def _on_physics_step(self, dt):
        self.publish_accum += float(dt)
        if self.publish_accum >= PUBLISH_PERIOD_SEC:
            self.publish_accum = 0.0
            self._publish()

    def shutdown(self):
        try:
            if self.physics_subscription is not None:
                self.physics_subscription.unsubscribe()
                self.physics_subscription = None
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


old = getattr(builtins, "_task23_gripper_ground_truth_bridge", None)
if old is not None:
    try:
        old.shutdown()
    except Exception as exc:
        print("清理旧 Task23 bridge 时出现非致命异常：", exc)

builtins._task23_gripper_ground_truth_bridge = Task23GroundTruthBridge()
