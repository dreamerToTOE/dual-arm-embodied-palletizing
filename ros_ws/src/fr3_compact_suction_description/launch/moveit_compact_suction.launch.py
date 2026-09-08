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

    description_pkg = get_package_share_directory(
        "fr3_compact_suction_description"
    )

    robot_xacro = os.path.join(
        description_pkg, "urdf", "fr3_compact_suction.urdf.xacro"
    )
    srdf_xacro = os.path.join(
        description_pkg, "srdf", "fr3_compact_suction.srdf.xacro"
    )

    # 不加载官方 franka_hand / cobot_pump。
    # 当前仅需要 MoveIt 规划，机器人实际运动仍由 Task04 基线代码通过
    # /joint_command 驱动 Isaac，因此这里使用 fake hardware 描述即可。
    robot_description_config = Command([
        FindExecutable(name="xacro"), " ", robot_xacro,
        " robot_type:=fr3",
        " hand:=false",
        " ee_id:=none",
        " robot_ip:=dont-care",
        " use_fake_hardware:=true",
        " fake_sensor_commands:=false",
    ])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_config, value_type=str
        )
    }

    robot_description_semantic_config = Command([
        FindExecutable(name="xacro"), " ", srdf_xacro,
    ])
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_config, value_type=str
        )
    }

    kinematics_yaml = load_yaml(
        "franka_fr3_moveit_config", "config/kinematics.yaml"
    )

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
    ompl_planning_pipeline_config["move_group"].update(
        load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    )

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

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

    # Isaac ActionGraph 已发布 /joint_states；该节点只负责按照同一份
    # robot_description 发布 TF，不再启动一套额外 fake controller 状态源。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
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
        robot_state_publisher,
        move_group,
        rviz,
    ])
