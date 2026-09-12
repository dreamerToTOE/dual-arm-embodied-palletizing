# Task18：随机场景 Ground Truth 输入 bridge。
#
# 在 task18_randomized_scene.py 已运行且 Timeline Play 后，从 Isaac Script Editor
# 运行。本 bridge 不发布机器人命令、不创建 Surface Gripper；它只扫描
# /World/Task18Box_*，读取 scene metadata 与实时 USD transform，并以统一的
# fr3_dual_palletize/msg/BoxStateArray 发布给 ROS 控制器。

import builtins
import json
import math

import omni.physx
import omni.usd
from pxr import Usd, UsdGeom

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

try:
    from fr3_dual_palletize.msg import BoxState, BoxStateArray
except ImportError as exc:
    raise RuntimeError(
        "无法导入 fr3_dual_palletize.msg.BoxStateArray。请从已编译并 source 的 "
        "ros_ws/install 环境启动 Isaac Sim，然后重新运行本 bridge。"
    ) from exc


TASK18_PREFIX = "/World/Task18Box_"
PUBLISH_PERIOD_SEC = 0.10


class Task18GroundTruthBridge:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError("当前没有有效 USD Stage。")
        self._discover_boxes_or_raise()

        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("task18_ground_truth_bridge", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publisher = self.node.create_publisher(BoxStateArray, "/task18/box_states", 10)
        self._publish_accum = 0.0
        self.physics_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self._on_physics_step
        )

        self._publish_states()
        print("")
        print("====================================================")
        print("Task18 Ground Truth bridge started")
        print("PUB : /task18/box_states fr3_dual_palletize/msg/BoxStateArray")
        print(f"seed={self.seed}, objects={len(self.boxes)}")
        print("No robot command / no suction command is published by this bridge.")
        print("====================================================")

    def _discover_boxes_or_raise(self):
        discovered = []
        seeds = set()
        for prim in self.stage.Traverse():
            path = prim.GetPath().pathString
            if not path.startswith(TASK18_PREFIX):
                continue
            raw_metadata = prim.GetCustomDataByKey("task18_metadata")
            if not raw_metadata:
                continue
            try:
                metadata = json.loads(str(raw_metadata))
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"Task18 metadata 无法解析：{path}") from exc
            required = ("id", "size", "mass", "payload_class", "allowed_modes", "grasp_candidates", "seed")
            missing = [name for name in required if name not in metadata]
            if missing:
                raise RuntimeError(f"Task18 metadata 不完整：{path} 缺少 {missing}")
            if len(metadata["size"]) != 3 or not metadata["grasp_candidates"]:
                raise RuntimeError(f"Task18 metadata 几何/抓取候选无效：{path}")
            discovered.append((path, metadata))
            seeds.add(int(metadata["seed"]))
        if not discovered:
            raise RuntimeError(
                "未发现 /World/Task18Box_*。请先在 Timeline Stop 状态运行 "
                "task18_randomized_scene.py，再点击 Play。"
            )
        if len(seeds) != 1:
            raise RuntimeError("Task18 场景包含多个 seed 的对象，拒绝发布混合 episode。")
        self.boxes = sorted(discovered, key=lambda item: item[1]["id"])
        self.seed = seeds.pop()

    def _pose_from_prim(self, prim_path):
        prim = self.stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Task18 Ground Truth Prim 不存在：{prim_path}")
        transform = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        position = transform.ExtractTranslation()
        quaternion = transform.ExtractRotationQuat()
        imaginary = quaternion.GetImaginary()
        norm = math.sqrt(
            float(imaginary[0]) ** 2 + float(imaginary[1]) ** 2 +
            float(imaginary[2]) ** 2 + float(quaternion.GetReal()) ** 2
        )
        if norm <= 1.0e-9:
            raise RuntimeError(f"Task18 Ground Truth quaternion 无效：{prim_path}")
        pose = BoxState().pose
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        pose.orientation.x = float(imaginary[0]) / norm
        pose.orientation.y = float(imaginary[1]) / norm
        pose.orientation.z = float(imaginary[2]) / norm
        pose.orientation.w = float(quaternion.GetReal()) / norm
        return pose

    @staticmethod
    def _pose_from_metadata(raw_pose):
        position = raw_pose.get("position", [])
        orientation = raw_pose.get("orientation", [0.0, 0.0, 0.0, 1.0])
        if len(position) != 3 or len(orientation) != 4:
            raise RuntimeError("Task18 grasp candidate pose 格式无效。")
        pose = BoxState().pose
        pose.position.x, pose.position.y, pose.position.z = (float(value) for value in position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            float(value) for value in orientation
        )
        return pose

    def _publish_states(self):
        message = BoxStateArray()
        now = self.node.get_clock().now().to_msg()
        message.header.stamp = now
        message.header.frame_id = "world"
        message.seed = int(self.seed)
        for prim_path, metadata in self.boxes:
            state = BoxState()
            state.stamp = now
            state.id = str(metadata["id"])
            state.pose = self._pose_from_prim(prim_path)
            state.size = [float(value) for value in metadata["size"]]
            state.mass = float(metadata["mass"])
            state.payload_class = str(metadata["payload_class"])
            state.allowed_modes = [str(value) for value in metadata["allowed_modes"]]
            for grasp in metadata["grasp_candidates"]:
                state.grasp_candidate_ids.append(str(grasp["id"]))
                state.grasp_candidates.append(self._pose_from_metadata(grasp["local_pose"]))
            message.boxes.append(state)
        self.publisher.publish(message)

    def _on_physics_step(self, dt):
        self._publish_accum += float(dt)
        if self._publish_accum >= PUBLISH_PERIOD_SEC:
            self._publish_accum = 0.0
            self._publish_states()

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


old_bridge = getattr(builtins, "_task18_ground_truth_bridge", None)
if old_bridge is not None:
    try:
        old_bridge.shutdown()
    except Exception as exc:
        print("清理旧 Task18 bridge 时出现非致命异常：", exc)

builtins._task18_ground_truth_bridge = Task18GroundTruthBridge()
