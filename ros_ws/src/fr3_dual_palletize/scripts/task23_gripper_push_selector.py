#!/usr/bin/env python3
"""Task23：从 Isaac Ground Truth 选择二层 4x4 的供料、暂存和推送目标。

选择器不导入 MoveIt，也不发布关节或夹爪命令。它只确保每一行按 +X 推送方向
从最远格到最近格排布，避免已放 Cube 堵住后续的直线推送通道。
"""

from __future__ import annotations

from typing import Dict, Optional, Set

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fr3_dual_palletize.msg import BoxState, BoxStateArray, PushStackTask, TaskWorldCommit


CUBE_SIZE = 0.030
LAYER2_CENTER_Z = 0.095
STAGING_X = 0.565
GRID_X_FAR_TO_NEAR = (0.705, 0.675, 0.645, 0.615)
GRID_Y = (-0.045, -0.015, 0.015, 0.045)


def pose(x: float, y: float, z: float) -> Pose:
    result = Pose()
    result.position.x = x
    result.position.y = y
    result.position.z = z
    result.orientation.w = 1.0
    return result


class Task23PushSelector(Node):
    def __init__(self):
        super().__init__("task23_gripper_push_selector")
        self.boxes: Dict[str, BoxState] = {}
        self.completed: Set[str] = set()
        self.active_object: Optional[str] = None
        self.sequence_index = 0
        self.task_publisher = self.create_publisher(
            PushStackTask,
            "/task23/push_task",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(BoxStateArray, "/task23/box_states", self._boxes, 20)
        self.create_subscription(TaskWorldCommit, "/task23/world_commit", self._commit, 20)
        self.get_logger().info(
            "Task23 selector ready: Ground Truth -> PushStackTask; "
            "no MoveIt, joint or gripper command is published."
        )

    @staticmethod
    def _target_for(index: int) -> Pose:
        row, column = divmod(index, 4)
        return pose(GRID_X_FAR_TO_NEAR[column], GRID_Y[row], LAYER2_CENTER_Z)

    def _boxes(self, message: BoxStateArray):
        self.boxes = {
            box.id: box
            for box in message.boxes
            if box.id.startswith("task23_supply_")
        }
        if len(self.boxes) != 16:
            self.get_logger().warning(
                "Task23 selector waits for 16 supply boxes; received %d." % len(self.boxes),
                throttle_duration_sec=2.0,
            )
            return
        self._publish_next_if_ready()

    def _commit(self, message: TaskWorldCommit):
        if message.coordination_mode != "GRIPPER_STAGE_AND_PUSH":
            return
        if self.active_object != message.object_id:
            self.get_logger().warning(
                "Task23 ignored unexpected commit object=%s active=%s"
                % (message.object_id, self.active_object)
            )
            return
        self.completed.add(message.object_id)
        self.active_object = None
        self.sequence_index += 1
        self.get_logger().info(
            "Task23 commit accepted: object=%s completed=%d/16"
            % (message.object_id, len(self.completed))
        )
        self._publish_next_if_ready()

    def _publish_next_if_ready(self):
        if self.active_object is not None or len(self.boxes) != 16:
            return
        pending = sorted(set(self.boxes).difference(self.completed))
        if not pending:
            self.get_logger().info("Task23 selector COMPLETE: 16/16 push tasks committed.")
            return
        object_id = pending[0]
        source = self.boxes[object_id].pose
        target = self._target_for(self.sequence_index)
        staging = pose(STAGING_X, target.position.y, LAYER2_CENTER_Z)
        task = PushStackTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = "world"
        task.scene_version = self.sequence_index + 1
        task.sequence_index = self.sequence_index
        task.task_id = "task23_stage_push_%02d" % (self.sequence_index + 1)
        task.object_id = object_id
        task.source_pose = source
        task.staging_pose = staging
        task.target_pose = target
        task.supply_arm = "left"
        task.push_arm = "right"
        task.push_distance = target.position.x - staging.position.x
        self.task_publisher.publish(task)
        self.active_object = object_id
        self.get_logger().info(
            "Task23 SELECT: index=%d object=%s source=(%.3f, %.3f, %.3f) "
            "stage=(%.3f, %.3f, %.3f) target=(%.3f, %.3f, %.3f) push=+X %.3f m"
            % (
                task.sequence_index + 1,
                task.object_id,
                source.position.x,
                source.position.y,
                source.position.z,
                staging.position.x,
                staging.position.y,
                staging.position.z,
                target.position.x,
                target.position.y,
                target.position.z,
                task.push_distance,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = Task23PushSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
