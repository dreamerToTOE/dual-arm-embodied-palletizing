"""Task20-C 单运行时大件的真实 Isaac 执行入口。

这个 launch 不替用户启动 MoveIt 或 Isaac：两者必须先就绪，且 Isaac 端必须已启动
Task18 Ground Truth bridge 与 Task20 dual-suction bridge。这样无 bridge / 无最新
Ground Truth 时执行器会安全拒绝，而不会悄悄使用虚拟关节状态。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    timeout_sec = LaunchConfiguration("timeout_sec")
    execution_time_scale = LaunchConfiguration("execution_time_scale")

    # 此窗口的中心 (0.650, 0.000) 来自 Task15 已完成物理验收的共同搬运目标；
    # 仍由 Task19 在窗口内生成 PlacementSpec，并非给单个 Box 写死目标坐标。
    parameters = {
        "expected_seed": 20260922,
        "max_tasks": 1,
        "timeout_sec": ParameterValue(timeout_sec, value_type=float),
        "execution_time_scale": ParameterValue(execution_time_scale, value_type=float),
        "pallet_min_x": 0.535,
        "pallet_max_x": 0.765,
        "pallet_min_y": -0.165,
        "pallet_max_y": 0.165,
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
                "Task20-C physical acceptance: seed=20260922, max_tasks=1; "
                "will reject before command publication unless Task18/Task20 bridges are ready."
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
