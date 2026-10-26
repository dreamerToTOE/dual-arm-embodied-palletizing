#!/usr/bin/env python3
"""Isolated Task28 derivative of the legacy prefix merger; publishes state only."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]


class Task28JointStateBridge(Node):
    def __init__(self):
        super().__init__("task28_joint_state_bridge")
        self.messages = {"left": None, "right": None}
        self.publisher = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "left/joint_states", self.left_callback, 10)
        self.create_subscription(JointState, "right/joint_states", self.right_callback, 10)

    @staticmethod
    def remap(message, side):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return None
        lookup = {name: index for index, name in enumerate(message.name)}
        if not all(joint in lookup for joint in ARM_JOINTS):
            return None
        indices = [lookup[joint] for joint in ARM_JOINTS]
        positions = [message.position[index] for index in indices]
        if not all(math.isfinite(value) for value in positions):
            return None
        velocities = [message.velocity[index] for index in indices] if len(message.velocity) == len(message.name) else []
        efforts = [message.effort[index] for index in indices] if len(message.effort) == len(message.name) else []
        return [f"{side}_{joint}" for joint in ARM_JOINTS], positions, velocities, efforts

    def left_callback(self, message):
        self.receive("left", message)

    def right_callback(self, message):
        self.receive("right", message)

    def receive(self, side, message):
        remapped = self.remap(message, side)
        if remapped is None:
            self.get_logger().error("Rejecting malformed Task28 JointState; no state published.")
            return
        self.messages[side] = remapped
        if any(value is None for value in self.messages.values()):
            return
        left, right = self.messages["left"], self.messages["right"]
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = left[0] + right[0]
        output.position = left[1] + right[1]
        if len(left[2]) == len(right[2]) == 7:
            output.velocity = left[2] + right[2]
        if len(left[3]) == len(right[3]) == 7:
            output.effort = left[3] + right[3]
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Task28JointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
