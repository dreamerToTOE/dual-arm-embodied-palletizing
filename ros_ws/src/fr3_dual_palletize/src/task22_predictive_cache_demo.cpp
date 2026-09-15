// Task22-C1：预测缓存的无执行回归。
//
// 本节点不连接 Isaac、MoveIt action、joint command 或 suction command。它验证 cache
// hit 的最低安全条件：释放后的 GT、下一件 source GT、左右臂终端关节与 scene_version
// 必须同时匹配。真正的后台 MoveIt sandbox planner 将在这些条件成立后才可接入。

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/predictive_plan_cache.hpp"

namespace
{

geometry_msgs::msg::Pose makePose(double x, double y, double z, double yaw = 0.0)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

fr3_dual_palletize::TaskTrajectoryCandidate makeCandidate()
{
  fr3_dual_palletize::TaskTrajectoryCandidate candidate;
  candidate.label = "Task22-C cached lookahead";
  candidate.arm_name = "left_arm";
  candidate.planning_group = "left_arm";
  candidate.object_id = "loose_box";
  candidate.eef_link = "left_fr3_link8";
  candidate.trajectory.joint_names = {
    "left_fr3_joint1", "left_fr3_joint2", "left_fr3_joint3", "left_fr3_joint4",
    "left_fr3_joint5", "left_fr3_joint6", "left_fr3_joint7"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {0.0, 0.1, -0.2, -0.3, 0.0, 0.5, 0.1};
  candidate.trajectory.points.push_back(point);
  candidate.start_q = point.positions;
  candidate.goal_q = point.positions;
  return candidate;
}

fr3_dual_palletize::PredictiveRuntimeSnapshot makeSnapshot()
{
  fr3_dual_palletize::PredictiveRuntimeSnapshot snapshot;
  snapshot.scene_version = 43;
  snapshot.object_ids = {"tight_box", "loose_box"};
  snapshot.object_poses = {
    makePose(0.650, 0.000, 0.090),
    makePose(0.360, -0.120, 0.065)};
  snapshot.left_joint_positions = {0.0, 0.1, -0.2, -0.3, 0.0, 0.5, 0.1};
  snapshot.right_joint_positions = {0.0, -0.1, 0.2, -0.3, 0.0, 0.5, -0.1};
  return snapshot;
}

bool expect(
  const rclcpp::Logger& logger,
  const fr3_dual_palletize::PredictivePlanCache& cache,
  const fr3_dual_palletize::PredictiveRuntimeSnapshot& snapshot,
  fr3_dual_palletize::PredictiveCacheDecision expected,
  const std::string& label)
{
  const auto result = cache.validate(snapshot);
  RCLCPP_INFO(
    logger, "Task22-C %s: decision=%s, diagnostics=%s",
    label.c_str(), fr3_dual_palletize::predictiveCacheDecisionName(result.decision),
    result.diagnostics.c_str());
  return result.decision == expected;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task22_predictive_cache_demo");
  bool success = false;
  do
  {
    fr3_dual_palletize::PredictivePlanCache cache;
    fr3_dual_palletize::PredictivePlanCacheEntry entry;
    entry.fingerprint.source_scene_version = 42;
    entry.fingerprint.completed_object_id = "tight_box";
    entry.fingerprint.expected_completed_release_pose = makePose(0.650, 0.000, 0.090);
    entry.fingerprint.lookahead_object_id = "loose_box";
    entry.fingerprint.expected_lookahead_source_pose = makePose(0.360, -0.120, 0.065);
    entry.fingerprint.selected_arm = "left";
    entry.fingerprint.expected_left_terminal_q = {0.0, 0.1, -0.2, -0.3, 0.0, 0.5, 0.1};
    entry.fingerprint.expected_right_terminal_q = {0.0, -0.1, 0.2, -0.3, 0.0, 0.5, -0.1};
    entry.lookahead_candidate = makeCandidate();
    std::string error;
    if (!cache.store(std::move(entry), error))
    {
      RCLCPP_ERROR(node->get_logger(), "Task22-C 无法建立预测缓存：%s", error.c_str());
      break;
    }

    const auto valid = makeSnapshot();
    auto stale_scene = valid;
    stale_scene.scene_version = 42;
    auto completed_pose_drift = valid;
    completed_pose_drift.object_poses[0].position.x += 0.020;
    auto source_pose_drift = valid;
    source_pose_drift.object_poses[1].position.y -= 0.020;
    auto terminal_joint_drift = valid;
    terminal_joint_drift.left_joint_positions[6] += 0.050;

    success =
      expect(node->get_logger(), cache, valid,
        fr3_dual_palletize::PredictiveCacheDecision::HIT, "CACHE_HIT") &&
      expect(node->get_logger(), cache, stale_scene,
        fr3_dual_palletize::PredictiveCacheDecision::SCENE_VERSION_NOT_ADVANCED,
        "STALE_SCENE") &&
      expect(node->get_logger(), cache, completed_pose_drift,
        fr3_dual_palletize::PredictiveCacheDecision::COMPLETED_OBJECT_POSE_MISMATCH,
        "RELEASE_DRIFT") &&
      expect(node->get_logger(), cache, source_pose_drift,
        fr3_dual_palletize::PredictiveCacheDecision::LOOKAHEAD_OBJECT_POSE_MISMATCH,
        "SOURCE_DRIFT") &&
      expect(node->get_logger(), cache, terminal_joint_drift,
        fr3_dual_palletize::PredictiveCacheDecision::LEFT_TERMINAL_STATE_MISMATCH,
        "JOINT_DRIFT");
  } while (false);

  if (success)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "Task22-C1 PASS：预测缓存仅在 GT release/source、双臂 terminal q 与递增 scene "
      "同时匹配时可复用；任何 drift 均被拒绝。未连接 MoveIt/Isaac 执行器。");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Task22-C1 FAIL：预测缓存安全判定回归失败。");
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
