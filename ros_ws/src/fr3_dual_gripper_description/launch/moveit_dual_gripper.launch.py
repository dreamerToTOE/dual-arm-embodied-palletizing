import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    package_name = "fr3_dual_gripper_description"
    package_path = get_package_share_directory(package_name)
    robot_xacro = os.path.join(package_path, "urdf", "dual_fr3_gripper.urdf.xacro")
    srdf_xacro = os.path.join(package_path, "srdf", "dual_fr3_gripper.srdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_xacro]), value_type=str
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", srdf_xacro]), value_type=str
        )
    }
    kinematics = load_yaml(package_name, "config/kinematics.yaml")
    ompl = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    planners = ["RRTConnectkConfigDefault"]
    ompl["left_arm"] = {"planner_configs": planners}
    ompl["right_arm"] = {"planner_configs": planners}
    ompl["dual_arm"] = {"planner_configs": planners}
    pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/ResolveConstraintFrames "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    pipeline["move_group"].update(ompl)
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        Node(
            package=package_name,
            executable="dual_gripper_joint_state_bridge.py",
            name="dual_gripper_joint_state_bridge",
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="dual_gripper_robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[robot_description, robot_description_semantic, kinematics, pipeline, scene_monitor],
        ),
        Node(
            package=package_name,
            executable="task23_environment_publisher.py",
            name="task23_environment_publisher",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="task23_moveit_rviz",
            output="screen",
            condition=IfCondition(use_rviz),
            parameters=[robot_description, robot_description_semantic, kinematics],
        ),
    ])
