"""Task22-A：Ground Truth -> candidates -> Python dispatch 的只读预览。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_moveit_preflight = LaunchConfiguration("enable_moveit_preflight")
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_moveit_preflight",
            default_value="false",
            description="true 时启动 Task22-B primary-arm-first MoveIt/FCL preflight；仍不会执行机器人。",
        ),
        LogInfo(
            msg=(
                "Task22 read-only dispatch preview: no joint command and no suction command; "
                "MoveIt/FCL starts only when enable_moveit_preflight:=true."
            )
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task22_gt_candidate_provider",
            name="task22_gt_candidate_provider",
            output="screen",
            parameters=[{
                "input_topic": "/task18/box_states",
                "output_topic": "/task22/dispatch_candidates",
                "max_candidates_per_box": 3,
                "pallet_min_x": 0.555,
                "pallet_max_x": 0.785,
                "pallet_min_y": -0.155,
                "pallet_max_y": 0.145,
                "pallet_support_height": 0.050,
            }],
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task22_gt_dispatch_selector",
            name="task22_gt_dispatch_selector",
            output="screen",
            parameters=[{
                "candidate_topic": "/task22/dispatch_candidates",
                "output_topic": "/task22/dispatch",
                "left_tcp_topic": "/task20/left/suction_tcp_pose",
                "right_tcp_topic": "/task20/right/suction_tcp_pose",
                "nominal_tcp_speed_mps": 0.25,
                "placement_cost_weight_sec": 0.10,
                "opposite_workspace_penalty_sec": 0.15,
                "selector_id": "task22_distance_time_v1",
            }],
        ),
        Node(
            package="fr3_dual_palletize",
            executable="task22_moveit_dispatch_gateway",
            name="task22_moveit_dispatch_gateway",
            output="screen",
            condition=IfCondition(enable_moveit_preflight),
            parameters=[{
                "candidate_topic": "/task22/dispatch_candidates",
                "dispatch_topic": "/task22/dispatch",
                "fast_candidate_count": 1,
            }],
        ),
    ])
