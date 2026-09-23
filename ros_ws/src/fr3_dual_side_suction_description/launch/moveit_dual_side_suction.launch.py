import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    package_name = "fr3_dual_side_suction_description"
    package_path = get_package_share_directory(package_name)
    robot_xacro = os.path.join(package_path, "urdf", "dual_fr3_side_suction.urdf.xacro")
    srdf_xacro = os.path.join(package_path, "srdf", "dual_fr3_side_suction.srdf.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", robot_xacro,
            ]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", srdf_xacro]), value_type=str
        )
    }
    kinematics_yaml = _load_yaml(package_name, "config/kinematics.yaml")
    ompl_yaml = _load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    planners = ["RRTConnectkConfigDefault"]
    for group in ("left_arm", "right_arm", "dual_arm"):
        ompl_yaml[group] = {"planner_configs": planners}
    planning_pipeline = {
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
    planning_pipeline["move_group"].update(ompl_yaml)
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # 复用已验证的名称前缀合并桥与桌面 CollisionObject 发布器；它们不绑定旧吸盘几何。
    joint_state_bridge = Node(
        package="fr3_dual_compact_suction_description",
        executable="dual_joint_state_bridge.py",
        name="dual_joint_state_bridge",
        output="screen",
    )
    environment = Node(
        package="fr3_dual_compact_suction_description",
        executable="task06_environment_publisher.py",
        name="task24_environment_publisher",
        output="screen",
        parameters=[{
            "table_id": "task24_table",
            "table_frame": "world",
            # 与 Task24 Isaac 场景及执行器的 CollisionObject 严格同构。
            # 桌面顶面 z=0.200 m：按官方 FR3 准备姿态使侧吸工作区靠近 shoulder 高度。
            "table_center": [0.55, 0.0, 0.100],
            "table_size": [1.20, 0.80, 0.200],
        }],
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
            planning_pipeline,
            scene_monitor,
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="dual_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(
            get_package_share_directory("franka_fr3_moveit_config"), "rviz", "moveit.rviz"
        )],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, planning_pipeline],
        condition=IfCondition(use_rviz),
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        joint_state_bridge,
        robot_state_publisher,
        move_group,
        environment,
        rviz,
    ])
