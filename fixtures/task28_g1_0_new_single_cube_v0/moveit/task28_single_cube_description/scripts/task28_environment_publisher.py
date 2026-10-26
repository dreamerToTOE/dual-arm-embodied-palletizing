#!/usr/bin/env python3
"""Future-use Task28 world geometry publisher; never emits robot commands.

The Cube stays a world collision object and follows PoseArray index 0. This is
payload-preserving geometry, not an AttachedCollisionObject implementation or a
closed-chain collision validator. No AllowedCollisionMatrix field is written.
"""

import json
import math
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class Task28EnvironmentPublisher(Node):
    def __init__(self):
        super().__init__("task28_environment_publisher")
        self.declare_parameter("fixture_geometry_path", "")
        path = self.get_parameter("fixture_geometry_path").value
        with open(path, "r", encoding="utf-8") as stream:
            self.geometry = json.load(stream)
        if (self.geometry["cube_id"], self.geometry["cube_index"], self.geometry["cube_count"]) != ("Cube_01", 0, 1):
            raise ValueError("Task28 requires exactly Cube_01/index 0/count 1")
        if self.geometry["frame_id"] != "world" or self.geometry["table_id"] != "task28_table":
            raise ValueError("Unexpected Task28 frame or table identity")
        self.cube_pose = self.make_pose(self.geometry["cube_initial_pose_world_m"], self.geometry["cube_orientation_xyzw"])
        self.client = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.create_subscription(PoseArray, "cube_poses", self.receive_cube_poses, 10)
        self.in_flight = False
        self.initialized = False
        self.dirty = True
        self.revision = 0
        self.deadline = time.monotonic() + 30.0
        self.timer = self.create_timer(0.1, self.publish_pending)

    @staticmethod
    def make_pose(position, quaternion):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
        return pose

    def receive_cube_poses(self, message):
        if message.header.frame_id != "world" or len(message.poses) != 1:
            self.get_logger().error("Rejecting Cube PoseArray: expected world/[Cube_01].")
            return
        pose = message.poses[0]
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        values = [pose.position.x, pose.position.y, pose.position.z] + quaternion
        if not all(math.isfinite(value) for value in values) or abs(sum(value * value for value in quaternion) - 1.0) > 1e-5:
            self.get_logger().error("Rejecting invalid Cube_01 pose.")
            return
        self.cube_pose = pose
        self.revision += 1
        self.dirty = True

    def box(self, object_id, size, pose):
        result = CollisionObject()
        result.header.frame_id = "world"
        result.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        result.primitives = [primitive]
        result.primitive_poses = [pose]
        result.operation = CollisionObject.ADD
        return result

    def publish_pending(self):
        if self.in_flight or not self.dirty:
            return
        if not self.client.service_is_ready():
            if time.monotonic() > self.deadline:
                self.get_logger().error("Task28 apply_planning_scene unavailable after 30 seconds; stopped.")
                self.timer.cancel()
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        cube = self.box("Cube_01", self.geometry["cube_size_m"], self.cube_pose)
        scene.world.collision_objects = [cube]
        if not self.initialized:
            table = self.box("task28_table", self.geometry["table_size_m"], self.make_pose(
                self.geometry["table_center_world_m"], [0.0, 0.0, 0.0, 1.0]))
            scene.world.collision_objects = [table, cube]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self.in_flight = True
        revision = self.revision
        future = self.client.call_async(request)
        future.add_done_callback(lambda completed: self.applied(completed, revision))

    def applied(self, future, revision):
        self.in_flight = False
        try:
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError("MoveIt rejected Task28 world geometry")
        except Exception as error:
            self.get_logger().error(str(error))
            self.timer.cancel()
            return
        self.initialized = True
        self.dirty = revision != self.revision


def main(args=None):
    rclpy.init(args=args)
    node = Task28EnvironmentPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
