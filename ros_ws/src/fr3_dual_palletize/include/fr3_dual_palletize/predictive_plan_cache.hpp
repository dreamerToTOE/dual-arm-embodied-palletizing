#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "fr3_dual_palletize/task_trajectory_candidate.hpp"

namespace fr3_dual_palletize
{

// Task22-C：后台预测不能只按“下一件 box id 相同”复用轨迹。该指纹绑定紧协调
// 释放后的真实几何、下一件待取物的 source pose，以及两台机械臂的预测终端关节状态。
// 任何一项不满足，就必须放弃缓存并从当前 Ground Truth 重新规划。
struct PredictivePlanFingerprint
{
  std::uint64_t source_scene_version{0};

  std::string completed_object_id;
  geometry_msgs::msg::Pose expected_completed_release_pose;

  std::string lookahead_object_id;
  geometry_msgs::msg::Pose expected_lookahead_source_pose;
  std::string selected_arm;

  std::vector<double> expected_left_terminal_q;
  std::vector<double> expected_right_terminal_q;
};

// 用于 cache hit 前的真实运行时快照。scene_version 允许比 source_scene_version
// 更大：紧协调 release 后 GT 必然发生一次 World Commit；只拒绝未推进或回退的版本。
struct PredictiveRuntimeSnapshot
{
  std::uint64_t scene_version{0};
  std::vector<std::string> object_ids;
  std::vector<geometry_msgs::msg::Pose> object_poses;
  std::vector<double> left_joint_positions;
  std::vector<double> right_joint_positions;
};

struct PredictiveCacheValidationConfig
{
  double position_tolerance_m{0.005};
  double orientation_tolerance_rad{0.050};
  double joint_tolerance_rad{0.010};
};

enum class PredictiveCacheDecision
{
  EMPTY,
  HIT,
  INVALID_SNAPSHOT,
  SCENE_VERSION_NOT_ADVANCED,
  COMPLETED_OBJECT_MISSING,
  COMPLETED_OBJECT_POSE_MISMATCH,
  LOOKAHEAD_OBJECT_MISSING,
  LOOKAHEAD_OBJECT_POSE_MISMATCH,
  LEFT_TERMINAL_STATE_MISMATCH,
  RIGHT_TERMINAL_STATE_MISMATCH,
};

struct PredictiveCacheValidationResult
{
  PredictiveCacheDecision decision{PredictiveCacheDecision::EMPTY};
  std::string diagnostics;

  bool reusable() const
  {
    return decision == PredictiveCacheDecision::HIT;
  }
};

struct PredictivePlanCacheEntry
{
  PredictivePlanFingerprint fingerprint;
  // 该轨迹必须来自独立 planning sandbox；缓存层本身绝不发布、执行或改写
  // /move_group 的共享 collision world。
  TaskTrajectoryCandidate lookahead_candidate;
};

// Task22-C 的线程安全 cache 合同。它仅决定“缓存是否仍可进入最终 FCL 复核”，
// 不把 cache hit 误当成执行授权。
class PredictivePlanCache
{
public:
  bool store(PredictivePlanCacheEntry entry, std::string& error);
  void clear();
  bool hasEntry() const;

  PredictiveCacheValidationResult validate(
    const PredictiveRuntimeSnapshot& runtime,
    const PredictiveCacheValidationConfig& config = PredictiveCacheValidationConfig{}) const;

  bool entry(PredictivePlanCacheEntry& output) const;

private:
  mutable std::mutex mutex_;
  PredictivePlanCacheEntry entry_;
  bool has_entry_{false};
};

const char* predictiveCacheDecisionName(PredictiveCacheDecision decision);

}  // namespace fr3_dual_palletize
