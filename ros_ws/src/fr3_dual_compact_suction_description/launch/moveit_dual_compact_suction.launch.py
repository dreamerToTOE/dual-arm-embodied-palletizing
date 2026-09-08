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
    path = os.path.join(package_path, relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")

    package_name = "fr3_dual_compact_suction_description"
    package_path = get_package_share_directory(package_name)

    robot_xacro = os.path.join(
        package_path, "urdf", "dual_fr3_compact_suction.urdf.xacro"
    )
    srdf_xacro = os.path.join(
        package_path, "srdf", "dual_fr3_compact_suction.srdf.xacro"
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_xacro]),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", srdf_xacro]),
            value_type=str,
        )
    }

    kinematics_yaml = load_yaml(package_name, "config/kinematics.yaml")

    # 复用当前 FR3 MoveIt 已验证的 OMPL planner 列表，但把规划组改成 Task06 的左右臂。
    ompl_yaml = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    default_planners = ["RRTConnectkConfigDefault"]
    ompl_yaml["left_arm"] = {"planner_configs": default_planners}
    ompl_yaml["right_arm"] = {"planner_configs": default_planners}
    ompl_yaml["dual_arm"] = {"planner_configs": default_planners}

    ompl_planning_pipeline_config = {
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
    ompl_planning_pipeline_config["move_group"].update(ompl_yaml)

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Isaac Task06-B 仍发布 /left/joint_states 与 /right/joint_states。
    # 该桥接节点只做名字前缀和合并，不改变 Isaac 侧控制链。
    joint_state_bridge = Node(
        package=package_name,
        executable="dual_joint_state_bridge.py",
        name="dual_joint_state_bridge",
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="dual_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            planning_scene_monitor_parameters,
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("franka_fr3_moveit_config"),
        "rviz",
        "moveit.rviz",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="是否启动 RViz2",
        ),
        joint_state_bridge,
        robot_state_publisher,
        move_group,
        rviz,
    ])
