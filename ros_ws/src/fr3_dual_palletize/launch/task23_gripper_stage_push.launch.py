"""Task23：自包含双夹爪 MoveIt + selector + stage-and-push 执行启动器。

Isaac 的 Task23 场景和 Ground Truth bridge 必须已经处于 Play；本 launch 不创建 USD。
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gripper_share = get_package_share_directory("fr3_dual_gripper_description")
    return LaunchDescription([
        DeclareLaunchArgument("max_tasks", default_value="16"),
        DeclareLaunchArgument("execution_time_scale", default_value="1.5"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{gripper_share}/launch/moveit_dual_gripper.launch.py"
            ),
            launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        ),
        # selector 不依赖 MoveIt，可以立即接收 Isaac Ground Truth 并保留最新任务。
        Node(
            package="fr3_dual_palletize",
            executable="task23_gripper_push_selector",
            name="task23_gripper_push_selector",
            output="screen",
        ),
        # 等待 move_group、robot_state_publisher 和 planning scene service 建立。
        TimerAction(
            period=5.0,
            actions=[Node(
                package="fr3_dual_palletize",
                executable="task23_gripper_stage_push",
                name="task23_gripper_stage_push",
                output="screen",
                parameters=[{
                    "max_tasks": ParameterValue(
                        LaunchConfiguration("max_tasks"), value_type=int
                    ),
                    "execution_time_scale": ParameterValue(
                        LaunchConfiguration("execution_time_scale"), value_type=float
                    ),
                }],
            )],
        ),
    ])
