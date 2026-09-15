#!/usr/bin/env python3
"""Task23：合并双 FR3 的 arm + finger JointState，供 MoveIt 使用。"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINTS = [
    *(f"fr3_joint{i}" for i in range(1, 8)),
    "fr3_finger_joint1",
    "fr3_finger_joint2",
]


class DualGripperJointStateBridge(Node):
    """保留真实 finger state，禁止用 arm-only RobotState 掩盖夹爪碰撞。"""

    def __init__(self):
        super().__init__("dual_gripper_joint_state_bridge")
        self.left_message = None
        self.right_message = None
        self.publisher = self.create_publisher(JointState, "/joint_states", 20)
        self.create_subscription(JointState, "/left/joint_states", self._left, 20)
        self.create_subscription(JointState, "/right/joint_states", self._right, 20)
        self.get_logger().info(
            "Task23 joint-state bridge: /left|right/joint_states -> /joint_states "
            "(including both finger joints)."
        )

    @staticmethod
    def _remap(message: JointState, side: str):
        indices = {name: index for index, name in enumerate(message.name)}
        missing = [name for name in JOINTS if name not in indices]
        if missing:
            return None, missing
        result = JointState()
        result.name = [f"{side}_{name}" for name in JOINTS]
        result.position = [message.position[indices[name]] for name in JOINTS]
        if len(message.velocity) == len(message.name):
            result.velocity = [message.velocity[indices[name]] for name in JOINTS]
        if len(message.effort) == len(message.name):
            result.effort = [message.effort[indices[name]] for name in JOINTS]
        return result, []

    def _left(self, message: JointState):
        self.left_message = message
        self._publish()

    def _right(self, message: JointState):
        self.right_message = message
        self._publish()

    def _publish(self):
        if self.left_message is None or self.right_message is None:
            return
        left, left_missing = self._remap(self.left_message, "left")
        right, right_missing = self._remap(self.right_message, "right")
        if left is None or right is None:
            self.get_logger().warning(
                "Task23 waiting for finger states; left missing=%s, right missing=%s"
                % (left_missing, right_missing),
                throttle_duration_sec=2.0,
            )
            return
        output = JointState()
        output.header.stamp = self.get_clock().now().to_msg()
        output.name = left.name + right.name
        output.position = left.position + right.position
        if len(left.velocity) == len(JOINTS) and len(right.velocity) == len(JOINTS):
            output.velocity = left.velocity + right.velocity
        if len(left.effort) == len(JOINTS) and len(right.effort) == len(JOINTS):
            output.effort = left.effort + right.effort
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = DualGripperJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
