"""Task20-E：反馈闭环与效率剖析的三对象 Isaac 验收入口。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    timeout_sec = LaunchConfiguration("timeout_sec")
    execution_time_scale = LaunchConfiguration("execution_time_scale")

    parameters = {
        "execution_label": "Task20-E",
        "expected_seed": 20260924,
        "max_tasks": 3,
        "timeout_sec": ParameterValue(timeout_sec, value_type=float),
        "execution_time_scale": ParameterValue(execution_time_scale, value_type=float),
        "task_plan_attempts": 3,
        "max_placement_candidates_to_try": 8,
        # 非对称 pallet region 让几何中心不是旧 Task20-D 的 (0.650, 0.000)，
        # 而共享箱仍有满足双臂共同搬运验证的中央可达空间。
        "pallet_min_x": 0.555,
        "pallet_max_x": 0.785,
        "pallet_min_y": -0.155,
        "pallet_max_y": 0.145,
        "pallet_support_height": 0.050,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "timeout_sec", default_value="60.0",
            description="等待 /task18/box_states 的最长时间（秒）。",
        ),
        DeclareLaunchArgument(
            "execution_time_scale", default_value="2.0",
            description="Isaac 实际执行时间缩放，必须不小于 1。",
        ),
        LogInfo(
            msg=(
                "Task20-E physical acceptance: medium shared box + two tall boxes; "
                "release is gated by real suction TCP Ground Truth and timing is profiled."
            )
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task20_runtime_execute",
            name="task20_runtime_execute",
            output="screen",
            parameters=[parameters],
        ),
    ])
