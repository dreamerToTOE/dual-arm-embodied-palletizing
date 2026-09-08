#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


class DualJointStateBridge(Node):
    """把 Isaac 左右两路 JointState 合并为 MoveIt 使用的唯一前缀 /joint_states。"""

    def __init__(self):
        super().__init__("dual_joint_state_bridge")

        self.left_msg = None
        self.right_msg = None

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(
            JointState,
            "/left/joint_states",
            self.left_callback,
            10,
        )
        self.create_subscription(
            JointState,
            "/right/joint_states",
            self.right_callback,
            10,
        )

        self.get_logger().info(
            "Task06-C joint-state bridge started: "
            "/left/joint_states + /right/joint_states -> /joint_states"
        )

    @staticmethod
    def remap(msg: JointState, prefix: str):
        lookup = {name: idx for idx, name in enumerate(msg.name)}
        names = []
        positions = []
        velocities = []
        efforts = []

        has_velocity = len(msg.velocity) == len(msg.name)
        has_effort = len(msg.effort) == len(msg.name)

        for joint in ARM_JOINTS:
            if joint not in lookup:
                continue
            idx = lookup[joint]
            names.append(f"{prefix}_{joint}")
            positions.append(msg.position[idx])
            if has_velocity:
                velocities.append(msg.velocity[idx])
            if has_effort:
                efforts.append(msg.effort[idx])

        return names, positions, velocities, efforts

    def left_callback(self, msg: JointState):
        self.left_msg = msg
        self.publish_combined()

    def right_callback(self, msg: JointState):
        self.right_msg = msg
        self.publish_combined()

    def publish_combined(self):
        if self.left_msg is None or self.right_msg is None:
            return

        left = self.remap(self.left_msg, "left")
        right = self.remap(self.right_msg, "right")

        if len(left[0]) != 7 or len(right[0]) != 7:
            self.get_logger().warning(
                "左右 JointState 未同时包含 fr3_joint1..7，暂不发布完整 /joint_states。",
                throttle_duration_sec=2.0,
            )
            return

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = left[0] + right[0]
        out.position = left[1] + right[1]

        if len(left[2]) == 7 and len(right[2]) == 7:
            out.velocity = left[2] + right[2]
        if len(left[3]) == 7 and len(right[3]) == 7:
            out.effort = left[3] + right[3]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DualJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
