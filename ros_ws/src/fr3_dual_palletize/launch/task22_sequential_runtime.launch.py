"""Task22-D：Python 快速分派 + 单件最终 MoveIt/FCL + 真实 Isaac 串行执行。

不启动 MoveIt 或 Isaac。操作者必须先运行 Task20-E compatible scene、Task18
Ground Truth bridge 与 Task20 dual-suction bridge；本 launch 在任何物理命令前由
executor 进行 source pose、MoveIt/OMPL 与 FCL 门禁。Task22-C sandbox 明确不参与。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    expected_seed = LaunchConfiguration("expected_seed")
    max_tasks = LaunchConfiguration("max_tasks")
    execution_time_scale = LaunchConfiguration("execution_time_scale")

    provider_parameters = {
        "input_topic": "/task18/box_states",
        "output_topic": "/task22/dispatch_candidates",
        "world_commit_topic": "/task22/world_commit",
        "max_candidates_per_box": 3,
        # 与 Task06 的 1.20 x 0.80 m /World/Table 对齐的保守中央子区。它在
        # 共同大件落稳后仍为单臂小件保留 table-level 候选，而非强制继续堆叠。
        "pallet_min_x": 0.455,
        "pallet_max_x": 0.855,
        "pallet_min_y": -0.245,
        "pallet_max_y": 0.245,
        "pallet_support_height": 0.050,
    }
    selector_parameters = {
        "candidate_topic": "/task22/dispatch_candidates",
        "output_topic": "/task22/dispatch",
        "left_tcp_topic": "/task20/left/suction_tcp_pose",
        "right_tcp_topic": "/task20/right/suction_tcp_pose",
        "nominal_tcp_speed_mps": 0.25,
        "placement_cost_weight_sec": 0.10,
        "opposite_workspace_penalty_sec": 0.15,
        "selector_id": "task22_distance_time_v1",
    }
    executor_parameters = {
        "input_topic": "/task18/box_states",
        "candidate_topic": "/task22/dispatch_candidates",
        "dispatch_topic": "/task22/dispatch",
        "world_commit_topic": "/task22/world_commit",
        "expected_seed": ParameterValue(expected_seed, value_type=int),
        "max_tasks": ParameterValue(max_tasks, value_type=int),
        "plan_attempts": 3,
        "fast_candidate_count": 1,
        "execution_time_scale": ParameterValue(execution_time_scale, value_type=float),
        "source_stale_tolerance_m": 0.003,
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "expected_seed", default_value="-1",
            description="可选 Task18 seed 断言；-1 表示接受当前合法 episode。",
        ),
        DeclareLaunchArgument(
            "max_tasks", default_value="0",
            description="0 表示执行当前 episode 的全部对象；正数为验收上限。",
        ),
        DeclareLaunchArgument(
            "execution_time_scale", default_value="2.0",
            description="物理执行时间缩放，必须不小于 1。",
        ),
        LogInfo(msg=(
            "Task22-D stable serial runtime: GT -> Python dispatch -> final MoveIt/FCL -> "
            "execute -> settle/World Commit -> next. Task22-C sandbox remains frozen."
        )),
        Node(
            package="fr3_dual_palletize",
            executable="task22_gt_candidate_provider",
            name="task22_gt_candidate_provider",
            output="screen",
            parameters=[provider_parameters],
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task22_gt_dispatch_selector",
            name="task22_gt_dispatch_selector",
            output="screen",
            parameters=[selector_parameters],
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task22_sequential_runtime_execute",
            name="task22_sequential_runtime_execute",
            output="screen",
            parameters=[executor_parameters],
        ),
    ])
