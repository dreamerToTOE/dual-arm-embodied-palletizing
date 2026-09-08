#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class Task06EnvironmentPublisher(Node):
    """把 Task06 Isaac 基准桌面写入 MoveIt Planning Scene。"""

    def __init__(self):
        super().__init__("task06_environment_publisher")

        self.declare_parameter("table_id", "task06_table")
        self.declare_parameter("table_frame", "world")
        self.declare_parameter("table_center", [0.55, 0.00, 0.025])
        self.declare_parameter("table_size", [1.20, 0.80, 0.05])

        self.table_id = self.get_parameter("table_id").value
        self.table_frame = self.get_parameter("table_frame").value
        self.table_center = tuple(self.get_parameter("table_center").value)
        self.table_size = tuple(self.get_parameter("table_size").value)

        self.apply_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

    def build_table(self) -> CollisionObject:
        table = CollisionObject()
        table.header.frame_id = self.table_frame
        table.id = self.table_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(self.table_size)

        pose = Pose()
        pose.position.x = self.table_center[0]
        pose.position.y = self.table_center[1]
        pose.position.z = self.table_center[2]
        pose.orientation.w = 1.0

        table.primitives = [primitive]
        table.primitive_poses = [pose]
        table.operation = CollisionObject.ADD
        return table

    def apply_environment(self) -> bool:
        self.get_logger().info(
            "等待 MoveIt /apply_planning_scene 服务，用于加载 Task06 环境。"
        )

        while rclpy.ok() and not self.apply_scene_client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info("/apply_planning_scene 尚未就绪，继续等待...")

        if not rclpy.ok():
            return False

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [self.build_table()]

        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.get_logger().error("调用 /apply_planning_scene 超时。")
            return False

        result = future.result()
        if result is None or not result.success:
            self.get_logger().error("MoveIt 拒绝加载 Task06 环境桌面。")
            return False

        self.get_logger().info(
            "Task06 MoveIt 环境加载完成: "
            f"id={self.table_id}, frame={self.table_frame}, "
            f"center={self.table_center}, size={self.table_size}"
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = Task06EnvironmentPublisher()
    success = False

    try:
        success = node.apply_environment()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not success:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
