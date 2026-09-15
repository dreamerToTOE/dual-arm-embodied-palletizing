#!/usr/bin/env python3
"""Task23：将桌面、固定第一层和二层暂存台写入 MoveIt Planning Scene。"""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


CUBE_SIZE = 0.030
TABLE = ("task23_table", (0.55, 0.00, 0.025), (1.20, 0.80, 0.050))
STAGE = ("task23_loading_stage", (0.565, 0.00, 0.065), (0.070, 0.150, 0.030))
GRID_X = (0.615, 0.645, 0.675, 0.705)
GRID_Y = (-0.045, -0.015, 0.015, 0.045)


def make_box(identifier, center, size):
    object_message = CollisionObject()
    object_message.header.frame_id = "world"
    object_message.id = identifier
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = center
    pose.orientation.w = 1.0
    object_message.primitives = [primitive]
    object_message.primitive_poses = [pose]
    object_message.operation = CollisionObject.ADD
    return object_message


class Task23EnvironmentPublisher(Node):
    def __init__(self):
        super().__init__("task23_environment_publisher")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def apply(self):
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Task23 waiting for /apply_planning_scene...")
        if not rclpy.ok():
            return False

        objects = [
            make_box(*TABLE),
            make_box(*STAGE),
        ]
        for row, y in enumerate(GRID_Y):
            for column, x in enumerate(GRID_X):
                objects.append(
                    make_box(
                        f"task23_base_{row}_{column}",
                        (x, y, 0.065),
                        (CUBE_SIZE, CUBE_SIZE, CUBE_SIZE),
                    )
                )
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = objects
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None or not future.result().success:
            self.get_logger().error("Task23 failed to load static planning scene.")
            return False
        self.get_logger().info(
            "Task23 planning scene ready: table + 4x4 fixed base layer + loading stage."
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Task23EnvironmentPublisher()
    success = False
    try:
        success = node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
