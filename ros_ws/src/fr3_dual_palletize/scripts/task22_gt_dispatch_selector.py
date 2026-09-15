#!/usr/bin/env python3
"""Task22-A：只读 Ground Truth 分派器。

该节点的唯一职责是以常数时间特征为 TaskDispatchCandidate 排序，输出一个 typed
TaskDispatch。它不导入 MoveIt、不调用规划器、不发布 joint/suction command；Task22-B
Gateway 必须在执行前重新验证 scene_version、IK、FCL 和 Task08/09 时空冲突。
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from fr3_dual_palletize.msg import TaskDispatch
from fr3_dual_palletize.msg import TaskDispatchCandidate
from fr3_dual_palletize.msg import TaskDispatchCandidateArray


LOOSE_LEFT = "loose_left"
LOOSE_RIGHT = "loose_right"
TIGHT_SHARED = "tight_shared_object"
MODE_LOOSE = "LOOSE"
MODE_TIGHT = "TIGHT_SHARED_OBJECT"


def distance(first: Pose, second: Pose) -> float:
    dx = first.position.x - second.position.x
    dy = first.position.y - second.position.y
    dz = first.position.z - second.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class GroundTruthDispatchSelector(Node):
    """以毫秒级以下的 cheap features 决定 primary/fallback arm。"""

    def __init__(self) -> None:
        super().__init__("task22_gt_dispatch_selector")
        self.candidate_topic = self.declare_parameter(
            "candidate_topic", "/task22/dispatch_candidates"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/task22/dispatch"
        ).value
        self.left_tcp_topic = self.declare_parameter(
            "left_tcp_topic", "/task20/left/suction_tcp_pose"
        ).value
        self.right_tcp_topic = self.declare_parameter(
            "right_tcp_topic", "/task20/right/suction_tcp_pose"
        ).value
        self.nominal_tcp_speed = float(
            self.declare_parameter("nominal_tcp_speed_mps", 0.25).value
        )
        self.placement_cost_weight = float(
            self.declare_parameter("placement_cost_weight_sec", 0.10).value
        )
        self.opposite_workspace_penalty = float(
            self.declare_parameter("opposite_workspace_penalty_sec", 0.15).value
        )
        self.selector_id = self.declare_parameter(
            "selector_id", "task22_distance_time_v1"
        ).value

        if self.nominal_tcp_speed <= 0.0:
            raise ValueError("nominal_tcp_speed_mps 必须大于零")

        self.tcp_poses: Dict[str, Pose] = {}
        self.last_scene_version: Optional[int] = None
        self.latest_candidates: Optional[TaskDispatchCandidateArray] = None
        # 决策和候选同样是 snapshot 合同。transient-local 让稍后启动的
        # Task22-B Gateway 能先读取最后一个 scene_version，再决定接受或拒绝，
        # 而不是因启动顺序丢失唯一的高层分派。
        self.publisher = self.create_publisher(
            TaskDispatch,
            self.output_topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.candidate_subscription = self.create_subscription(
            TaskDispatchCandidateArray,
            self.candidate_topic,
            self._on_candidates,
            10,
        )
        self.left_tcp_subscription = self.create_subscription(
            PoseStamped,
            self.left_tcp_topic,
            lambda message: self._update_tcp("left", message),
            10,
        )
        self.right_tcp_subscription = self.create_subscription(
            PoseStamped,
            self.right_tcp_topic,
            lambda message: self._update_tcp("right", message),
            10,
        )
        self.get_logger().info(
            "Task22-A Python selector ready: %s -> %s; "
            "no MoveIt/OMPL/FCL/joint/suction command will be used."
            % (self.candidate_topic, self.output_topic)
        )

    def _update_tcp(self, arm: str, message: PoseStamped) -> None:
        self.tcp_poses[arm] = message.pose
        if (
            self.latest_candidates is not None
            and self.latest_candidates.scene_version != self.last_scene_version
        ):
            self._try_dispatch(self.latest_candidates)

    @staticmethod
    def _has_mode(candidate: TaskDispatchCandidate, mode: str) -> bool:
        return mode in candidate.allowed_modes

    def _workspace_penalty(self, candidate: TaskDispatchCandidate, arm: str) -> float:
        # 只是一项廉价先验，不替代 MoveIt 的真实可达性。Task18 的左/右取料区以
        # world Y 负/正分离，若跨区分派则给予时间等价惩罚；公共区没有偏置。
        y = candidate.pick_pose.position.y
        if y < -0.05 and arm == "right":
            return self.opposite_workspace_penalty
        if y > 0.05 and arm == "left":
            return self.opposite_workspace_penalty
        return 0.0

    def _arm_score(self, candidate: TaskDispatchCandidate, arm: str) -> float:
        tcp = self.tcp_poses[arm]
        pick_time = distance(tcp, candidate.pick_pose) / self.nominal_tcp_speed
        transport_time = (
            distance(candidate.pick_pose, candidate.target_pose)
            / self.nominal_tcp_speed
        )
        return (
            pick_time
            + transport_time
            + self.placement_cost_weight * candidate.placement_cost
            + self._workspace_penalty(candidate, arm)
        )

    def _loose_choice(
        self, candidate: TaskDispatchCandidate
    ) -> Optional[Tuple[str, str, float]]:
        feasible_arms = []
        if self._has_mode(candidate, LOOSE_LEFT) and "left" in self.tcp_poses:
            feasible_arms.append(("left", self._arm_score(candidate, "left")))
        if self._has_mode(candidate, LOOSE_RIGHT) and "right" in self.tcp_poses:
            feasible_arms.append(("right", self._arm_score(candidate, "right")))
        if not feasible_arms:
            return None
        feasible_arms.sort(key=lambda item: (item[1], item[0]))
        primary, score = feasible_arms[0]
        fallback = feasible_arms[1][0] if len(feasible_arms) > 1 else "none"
        return primary, fallback, score

    def _on_candidates(self, message: TaskDispatchCandidateArray) -> None:
        self.latest_candidates = message
        if self.last_scene_version == message.scene_version:
            return
        self._try_dispatch(message)

    def _try_dispatch(self, message: TaskDispatchCandidateArray) -> None:
        if self.last_scene_version == message.scene_version:
            return
        began = time.perf_counter()

        # 共同物体保持既有 Task11--13 / Task21 紧协调，不用“最近手臂”错误分派。
        tight = [
            item for item in message.candidates
            if self._has_mode(item, TIGHT_SHARED)
        ]
        if tight:
            selected = min(
                tight,
                key=lambda item: (item.placement_cost, item.placement_rank, item.candidate_id),
            )
            self._publish(
                message, selected, MODE_TIGHT, "none", "none", selected.placement_cost, began
            )
            return

        choices = []
        for candidate in message.candidates:
            choice = self._loose_choice(candidate)
            if choice is None:
                continue
            primary, fallback, score = choice
            choices.append((score, candidate.candidate_id, candidate, primary, fallback))
        if not choices:
            self.get_logger().warning(
                "Task22-A no loose decision: waiting for allowed modes and both/one TCP Ground Truth."
            )
            return
        choices.sort(key=lambda item: (item[0], item[1]))
        score, _, selected, primary, fallback = choices[0]
        self._publish(message, selected, MODE_LOOSE, primary, fallback, score, began)

    def _publish(
        self,
        source: TaskDispatchCandidateArray,
        candidate: TaskDispatchCandidate,
        coordination_mode: str,
        primary: str,
        fallback: str,
        score: float,
        began: float,
    ) -> None:
        decision = TaskDispatch()
        decision.header = source.header
        decision.scene_version = source.scene_version
        decision.candidate_id = candidate.candidate_id
        decision.object_id = candidate.object_id
        decision.target_pose = candidate.target_pose
        decision.support_surface_id = candidate.support_surface_id
        decision.coordination_mode = coordination_mode
        decision.preferred_arm = primary
        decision.fallback_arm = fallback
        decision.selector_score = float(score)
        decision.selector_id = self.selector_id
        self.publisher.publish(decision)
        self.last_scene_version = source.scene_version
        elapsed_us = (time.perf_counter() - began) * 1.0e6
        self.get_logger().info(
            "Task22-A DISPATCH READY: scene_version=%d object=%s target=(%.3f, %.3f, %.3f) "
            "mode=%s primary=%s fallback=%s score=%.4f selector_wall=%.1f us"
            % (
                decision.scene_version,
                decision.object_id,
                decision.target_pose.position.x,
                decision.target_pose.position.y,
                decision.target_pose.position.z,
                decision.coordination_mode,
                decision.preferred_arm,
                decision.fallback_arm,
                decision.selector_score,
                elapsed_us,
            )
        )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = GroundTruthDispatchSelector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                # ros2 launch 会向每个子进程发送 SIGINT；若第二个 SIGINT 恰好落在
                # destroy_node()，也应视作正常停止，而不是 preview 测试失败。
                pass
        # launch 在 SIGINT 时可能已经替本进程关闭默认 context；二次 shutdown 会把
        # 正常的用户停止误报为 selector 进程失败。
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
