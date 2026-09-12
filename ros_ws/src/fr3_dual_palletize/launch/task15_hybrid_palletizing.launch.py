"""Task15 单入口：先紧协调搬运大件，再松协调码垛四个小件。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    execute = LaunchConfiguration("execute")
    wait_step_sec = LaunchConfiguration("wait_step_sec")
    max_wait_sec = LaunchConfiguration("max_wait_sec")
    planner_candidate_count = LaunchConfiguration("planner_candidate_count")
    redundancy_mode = LaunchConfiguration("redundancy_mode")
    preferred_redundant_joint = LaunchConfiguration("preferred_redundant_joint")

    common_parameters = {
        "execute": ParameterValue(execute, value_type=bool),
        "wait_step_sec": ParameterValue(wait_step_sec, value_type=float),
        "max_wait_sec": ParameterValue(max_wait_sec, value_type=float),
        "planner_candidate_count": ParameterValue(planner_candidate_count, value_type=int),
        "redundancy_mode": redundancy_mode,
        "preferred_redundant_joint": ParameterValue(preferred_redundant_joint, value_type=float),
    }

    # Phase A 复用 Task11--13 协议。Task15 的水平运输保持在桌面上方 100 mm：
    # 这为双臂前臂提供额外桌面净空，且所有同步段仍需通过 10 ms FCL。
    tight = Node(
        package="fr3_dual_palletize",
        executable="task15_tight_large_cube",
        name="task15_tight_large_cube",
        output="screen",
        parameters=[{
            "execute": ParameterValue(execute, value_type=bool),
            "lift_height_m": 0.10,
        }],
    )

    # Phase B 只能在 Phase A 正常退出后创建，避免大件尚未落稳便开始小件任务。
    loose = Node(
        package="fr3_dual_palletize",
        executable="task15_loose_layer_stack",
        name="task15_loose_layer_stack",
        output="screen",
        parameters=[common_parameters],
    )

    def start_loose_after_tight(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(
                    msg=(
                        "Task15 Phase A PASS：大 Cube 紧协调已完成；"
                        "自动进入 Phase B 四个小 Cube 松协调。"
                    )
                ),
                loose,
            ]
        return [
            LogInfo(
                msg=(
                    "ERROR: Task15 Phase A FAIL（exit code={}）；"
                    "不会启动 Phase B。"
                ).format(event.returncode)
            ),
            EmitEvent(
                event=Shutdown(
                    reason="Task15 Phase A failed; loose phase was not started."
                )
            ),
        ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "execute",
            default_value="true",
            choices=["true", "false"],
            description="true: 执行完整紧/松协调；false: 仅顺序完成规划、FCL 与调度预检。",
        ),
        DeclareLaunchArgument(
            "wait_step_sec",
            default_value="0.20",
            description="松协调 LocalWait 的离散等待步长（秒）。",
        ),
        DeclareLaunchArgument(
            "max_wait_sec",
            default_value="30.0",
            description="松协调单臂最大允许等待时间（秒）。",
        ),
        DeclareLaunchArgument(
            "planner_candidate_count",
            default_value="3",
            description="Task16 自由空间阶段生成并评分的 RRTConnect 候选数。",
        ),
        DeclareLaunchArgument(
            "redundancy_mode",
            default_value="soft_preference",
            choices=["free_7dof", "hard_lock_joint", "soft_preference"],
            description="Task16 冗余管理模式；默认 soft_preference 由 Task16 benchmark 确定。",
        ),
        DeclareLaunchArgument(
            "preferred_redundant_joint",
            default_value="0.0",
            description="Task16 joint7 的软偏好/硬锁定参考值（弧度）；默认与 benchmark 一致。",
        ),
        LogInfo(
            msg=(
                "Task15 单入口启动：Phase A 紧协调大 Cube -> "
                "Phase B 两层四小 Cube 松协调。"
            )
        ),
        tight,
        RegisterEventHandler(
            OnProcessExit(target_action=tight, on_exit=start_loose_after_tight)
        ),
    ])
