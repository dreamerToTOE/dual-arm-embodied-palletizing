"""Task20-D：多对象连续运行时 Isaac 物理验收入口。

本 launch 不会启动 MoveIt 或 Isaac。操作者必须先启动 Task20 acceptance scene、
Task18 Ground Truth bridge 和 Task20 dual-suction bridge；节点会在任何 bridge 或
Ground Truth 未就绪时安全退出，而不会发布不完整的机器人命令。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    timeout_sec = LaunchConfiguration("timeout_sec")
    execution_time_scale = LaunchConfiguration("execution_time_scale")
    task_plan_attempts = LaunchConfiguration("task_plan_attempts")
    max_placement_candidates_to_try = LaunchConfiguration(
        "max_placement_candidates_to_try"
    )

    # 与 Task20-C 保持同一固定 replay、桌面和窄托盘窗口。大件先被紧协调放到
    # 唯一的 table 基层；后续两件 tall box 必须由 Task19 自动选到大件顶面的完整
    # 支撑候选。每次实际释放后，执行器都读回 Ground Truth 并从更新后的世界重规划。
    parameters = {
        "expected_seed": 20260922,
        "max_tasks": 3,
        "timeout_sec": ParameterValue(timeout_sec, value_type=float),
        "execution_time_scale": ParameterValue(execution_time_scale, value_type=float),
        "task_plan_attempts": ParameterValue(task_plan_attempts, value_type=int),
        "max_placement_candidates_to_try": ParameterValue(
            max_placement_candidates_to_try, value_type=int
        ),
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
        DeclareLaunchArgument(
            "task_plan_attempts", default_value="3",
            description="同一 placement candidate 的完整 MoveIt/Router/FCL 规划重试次数。",
        ),
        DeclareLaunchArgument(
            "max_placement_candidates_to_try", default_value="8",
            description="当前物体可逐一尝试的已通过几何门禁的 placement candidates 上限。",
        ),
        LogInfo(
            msg=(
                "Task20-D continuous physical acceptance: seed=20260922, max_tasks=3; "
                "large tight route then two runtime loose routes, with real settle/replan between tasks."
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
