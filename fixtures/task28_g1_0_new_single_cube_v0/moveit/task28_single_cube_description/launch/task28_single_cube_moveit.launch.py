"""Future launch only: NEW Task28 fixture, with trajectory execution disabled.

G1-0B statically inspects this source. It does not execute this launch file.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = "task28_single_cube_description"
NAMESPACE = "/task28_g1_0"
# Namespace the standard absolute endpoints used by MoveIt/RSP as well as our
# relative endpoints. No global Task24/26/27 topic or service is consumed.
REMAPPINGS = [
    ("/joint_states", "joint_states"),
    ("/tf", "tf"),
    ("/tf_static", "tf_static"),
    ("/clock", "clock"),
    ("/rosout", "rosout"),
    ("/parameter_events", "parameter_events"),
    ("/robot_description", "robot_description"),
    ("/robot_description_semantic", "robot_description_semantic"),
    ("/planning_scene", "planning_scene"),
    ("/monitored_planning_scene", "monitored_planning_scene"),
    ("/apply_planning_scene", "apply_planning_scene"),
    ("/get_planning_scene", "get_planning_scene"),
    ("/check_state_validity", "check_state_validity"),
]


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _node(**kwargs):
    return Node(namespace=NAMESPACE, remappings=REMAPPINGS, **kwargs)


def generate_launch_description():
    package_path = get_package_share_directory(PACKAGE_NAME)
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    time_parameters = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
    frozen_urdf = os.path.join(package_path, "urdf", "task28_single_cube_frozen.urdf")
    srdf_xacro = os.path.join(package_path, "srdf", "dual_fr3_side_suction.srdf.xacro")
    with open(frozen_urdf, "r", encoding="utf-8") as stream:
        robot_description = {"robot_description": stream.read()}
    semantic_description = {"robot_description_semantic": ParameterValue(
        Command([FindExecutable(name="xacro"), " ", srdf_xacro]), value_type=str)}
    kinematics = {"robot_description_kinematics": _load_yaml(PACKAGE_NAME, "config/kinematics.yaml")}
    ompl = _load_yaml(PACKAGE_NAME, "config/ompl_planning.yaml")
    for group in ("left_arm", "right_arm", "dual_arm"):
        ompl[group] = {"planner_configs": ["RRTConnectkConfigDefault"]}
    pipeline = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/ResolveConstraintFrames "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"),
        "start_state_max_bounds_error": 0.1,
    }}
    pipeline["move_group"].update(ompl)
    monitoring = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "allow_trajectory_execution": False,
        "moveit_manage_controllers": False,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        _node(package=PACKAGE_NAME, executable="task28_joint_state_bridge.py",
              name="task28_joint_state_bridge", output="screen", parameters=[time_parameters]),
        _node(package="robot_state_publisher", executable="robot_state_publisher",
              name="task28_robot_state_publisher", output="screen",
              parameters=[robot_description, time_parameters]),
        _node(package="moveit_ros_move_group", executable="move_group", name="move_group",
              output="screen", parameters=[robot_description, semantic_description,
                                           kinematics, pipeline, monitoring, time_parameters]),
        _node(package=PACKAGE_NAME, executable="task28_environment_publisher.py",
              name="task28_environment_publisher", output="screen",
              parameters=[time_parameters, {"fixture_geometry_path": os.path.join(
                  package_path, "config", "fixture_geometry.json")}]),
        _node(package="rviz2", executable="rviz2", name="rviz2", output="log",
              parameters=[robot_description, semantic_description, kinematics, time_parameters],
              condition=IfCondition(use_rviz)),
    ])
