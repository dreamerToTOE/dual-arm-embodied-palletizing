"""Task22-C2：与执行 MoveIt 隔离的只读 predictive planning sandbox。"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    sandbox_namespace = LaunchConfiguration("sandbox_namespace")
    description_package = "fr3_dual_compact_suction_description"
    description_share = get_package_share_directory(description_package)

    robot_xacro = os.path.join(
        description_share, "urdf", "dual_fr3_compact_suction.urdf.xacro"
    )
    srdf_xacro = os.path.join(
        description_share, "srdf", "dual_fr3_compact_suction.srdf.xacro"
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

    kinematics_yaml = load_yaml(description_package, "config/kinematics.yaml")
    ompl_yaml = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml")
    planners = ["RRTConnectkConfigDefault"]
    ompl_yaml["left_arm"] = {"planner_configs": planners}
    ompl_yaml["right_arm"] = {"planner_configs": planners}
    ompl_yaml["dual_arm"] = {"planner_configs": planners}
    ompl_pipeline = {
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
    ompl_pipeline["move_group"].update(ompl_yaml)
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # 不启动 robot_state_publisher、RViz、环境发布器或执行器。sandbox 与真实执行器
    # 共用 Isaac 的状态/TF 输入，但 action、service、planning scene 均在此 namespace。
    # allow_trajectory_execution=false 是额外硬门：即使错误调用 execute，也不能触发控制器。
    sandbox_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # 与执行器 /move_group 使用不同的 node name。action/service 仍位于
        # /task22_sandbox namespace；参数服务路径因此明确，探针不会误从执行器
        # 复制 URDF/SRDF。
        name="sandbox_move_group",
        namespace=sandbox_namespace,
        output="screen",
        remappings=[
            ("joint_states", "/joint_states"),
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_pipeline,
            scene_monitor,
            {"allow_trajectory_execution": False},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sandbox_namespace",
            default_value="task22_sandbox",
            description="Task22-C2 predictive MoveIt sandbox namespace（不含前导 /）",
        ),
        sandbox_move_group,
    ])
