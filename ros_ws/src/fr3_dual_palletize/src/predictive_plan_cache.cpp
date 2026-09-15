#include "fr3_dual_palletize/predictive_plan_cache.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace fr3_dual_palletize
{
namespace
{

double positionDistance(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double orientationDistance(
  const geometry_msgs::msg::Quaternion& first,
  const geometry_msgs::msg::Quaternion& second)
{
  const double first_norm = std::sqrt(
    first.x * first.x + first.y * first.y + first.z * first.z + first.w * first.w);
  const double second_norm = std::sqrt(
    second.x * second.x + second.y * second.y + second.z * second.z + second.w * second.w);
  if (first_norm <= 1.0e-12 || second_norm <= 1.0e-12)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::clamp(
    std::abs(
      (first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w) /
      (first_norm * second_norm)),
    0.0, 1.0);
  return 2.0 * std::acos(dot);
}

const geometry_msgs::msg::Pose* poseFor(
  const PredictiveRuntimeSnapshot& snapshot,
  const std::string& id)
{
  const auto iterator = std::find(snapshot.object_ids.begin(), snapshot.object_ids.end(), id);
  if (iterator == snapshot.object_ids.end())
  {
    return nullptr;
  }
  const auto index = static_cast<std::size_t>(std::distance(snapshot.object_ids.begin(), iterator));
  return index < snapshot.object_poses.size() ? &snapshot.object_poses[index] : nullptr;
}

bool withinJointTolerance(
  const std::vector<double>& expected,
  const std::vector<double>& actual,
  double tolerance)
{
  if (expected.size() != 7 || actual.size() != expected.size())
  {
    return false;
  }
  return std::all_of(
    expected.begin(), expected.end(),
    [&actual, tolerance, index = std::size_t{0}](double value) mutable
    {
      return std::abs(value - actual[index++]) <= tolerance;
    });
}

bool validPose(const geometry_msgs::msg::Pose& pose)
{
  const auto finite = [](double value) { return std::isfinite(value); };
  return finite(pose.position.x) && finite(pose.position.y) && finite(pose.position.z) &&
         finite(pose.orientation.x) && finite(pose.orientation.y) &&
         finite(pose.orientation.z) && finite(pose.orientation.w);
}

bool validJointVector(const std::vector<double>& values)
{
  return values.size() == 7 && std::all_of(
    values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

}  // namespace

const char* predictiveCacheDecisionName(PredictiveCacheDecision decision)
{
  switch (decision)
  {
    case PredictiveCacheDecision::EMPTY:
      return "EMPTY";
    case PredictiveCacheDecision::HIT:
      return "HIT";
    case PredictiveCacheDecision::INVALID_SNAPSHOT:
      return "INVALID_SNAPSHOT";
    case PredictiveCacheDecision::SCENE_VERSION_NOT_ADVANCED:
      return "SCENE_VERSION_NOT_ADVANCED";
    case PredictiveCacheDecision::COMPLETED_OBJECT_MISSING:
      return "COMPLETED_OBJECT_MISSING";
    case PredictiveCacheDecision::COMPLETED_OBJECT_POSE_MISMATCH:
      return "COMPLETED_OBJECT_POSE_MISMATCH";
    case PredictiveCacheDecision::LOOKAHEAD_OBJECT_MISSING:
      return "LOOKAHEAD_OBJECT_MISSING";
    case PredictiveCacheDecision::LOOKAHEAD_OBJECT_POSE_MISMATCH:
      return "LOOKAHEAD_OBJECT_POSE_MISMATCH";
    case PredictiveCacheDecision::LEFT_TERMINAL_STATE_MISMATCH:
      return "LEFT_TERMINAL_STATE_MISMATCH";
    case PredictiveCacheDecision::RIGHT_TERMINAL_STATE_MISMATCH:
      return "RIGHT_TERMINAL_STATE_MISMATCH";
  }
  return "UNKNOWN";
}

bool PredictivePlanCache::store(PredictivePlanCacheEntry entry, std::string& error)
{
  const auto& fingerprint = entry.fingerprint;
  if (fingerprint.source_scene_version == 0 || fingerprint.completed_object_id.empty() ||
      fingerprint.lookahead_object_id.empty() || fingerprint.completed_object_id ==
      fingerprint.lookahead_object_id || fingerprint.selected_arm.empty() ||
      !validPose(fingerprint.expected_completed_release_pose) ||
      !validPose(fingerprint.expected_lookahead_source_pose) ||
      !validJointVector(fingerprint.expected_left_terminal_q) ||
      !validJointVector(fingerprint.expected_right_terminal_q) ||
      entry.lookahead_candidate.object_id != fingerprint.lookahead_object_id ||
      entry.lookahead_candidate.trajectory.points.empty())
  {
    error = "预测缓存条目不完整：需要递增 scene、对象匹配、双臂 7 关节终端状态和完整候选。";
    return false;
  }
  const auto& expected_start = fingerprint.selected_arm == "left" ?
    fingerprint.expected_left_terminal_q : fingerprint.expected_right_terminal_q;
  if ((fingerprint.selected_arm != "left" && fingerprint.selected_arm != "right") ||
      entry.lookahead_candidate.start_q.size() != expected_start.size() ||
      !std::equal(
        entry.lookahead_candidate.start_q.begin(), entry.lookahead_candidate.start_q.end(),
        expected_start.begin(), [](double first, double second) { return std::abs(first - second) <= 1.0e-9; }))
  {
    error = "缓存候选的起始关节必须等于所选臂的预测 terminal state。";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  entry_ = std::move(entry);
  has_entry_ = true;
  error.clear();
  return true;
}

void PredictivePlanCache::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  PredictivePlanCacheEntry empty_entry;
  entry_ = std::move(empty_entry);
  has_entry_ = false;
}

bool PredictivePlanCache::hasEntry() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_entry_;
}

bool PredictivePlanCache::entry(PredictivePlanCacheEntry& output) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_entry_)
  {
    return false;
  }
  output = entry_;
  return true;
}

PredictiveCacheValidationResult PredictivePlanCache::validate(
  const PredictiveRuntimeSnapshot& runtime,
  const PredictiveCacheValidationConfig& config) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_entry_)
  {
    return {PredictiveCacheDecision::EMPTY, "没有可复用的预测候选。"};
  }
  if (config.position_tolerance_m <= 0.0 || config.orientation_tolerance_rad <= 0.0 ||
      config.joint_tolerance_rad <= 0.0 || runtime.object_ids.size() != runtime.object_poses.size())
  {
    return {PredictiveCacheDecision::INVALID_SNAPSHOT, "运行时快照或 cache 阈值无效。"};
  }
  const auto& fingerprint = entry_.fingerprint;
  if (runtime.scene_version <= fingerprint.source_scene_version)
  {
    return {PredictiveCacheDecision::SCENE_VERSION_NOT_ADVANCED,
      "release 后尚未收到递增的 Ground Truth scene_version。"};
  }
  const auto* completed = poseFor(runtime, fingerprint.completed_object_id);
  if (!completed)
  {
    return {PredictiveCacheDecision::COMPLETED_OBJECT_MISSING,
      "已完成对象不在最新 Ground Truth 中。"};
  }
  const double completed_position_error = positionDistance(
    completed->position, fingerprint.expected_completed_release_pose.position);
  const double completed_orientation_error = orientationDistance(
    completed->orientation, fingerprint.expected_completed_release_pose.orientation);
  if (completed_position_error > config.position_tolerance_m ||
      completed_orientation_error > config.orientation_tolerance_rad)
  {
    std::ostringstream stream;
    stream << "已完成对象 release pose 偏差 position=" << completed_position_error << " m, orientation="
           << completed_orientation_error << " rad。";
    return {PredictiveCacheDecision::COMPLETED_OBJECT_POSE_MISMATCH, stream.str()};
  }
  const auto* lookahead = poseFor(runtime, fingerprint.lookahead_object_id);
  if (!lookahead)
  {
    return {PredictiveCacheDecision::LOOKAHEAD_OBJECT_MISSING,
      "下一件待取对象不在最新 Ground Truth 中。"};
  }
  const double lookahead_position_error = positionDistance(
    lookahead->position, fingerprint.expected_lookahead_source_pose.position);
  const double lookahead_orientation_error = orientationDistance(
    lookahead->orientation, fingerprint.expected_lookahead_source_pose.orientation);
  if (lookahead_position_error > config.position_tolerance_m ||
      lookahead_orientation_error > config.orientation_tolerance_rad)
  {
    std::ostringstream stream;
    stream << "下一件 source pose 偏差 position=" << lookahead_position_error << " m, orientation="
           << lookahead_orientation_error << " rad。";
    return {PredictiveCacheDecision::LOOKAHEAD_OBJECT_POSE_MISMATCH, stream.str()};
  }
  if (!withinJointTolerance(
        fingerprint.expected_left_terminal_q, runtime.left_joint_positions,
        config.joint_tolerance_rad))
  {
    return {PredictiveCacheDecision::LEFT_TERMINAL_STATE_MISMATCH,
      "left_arm 未到达预测终端状态。"};
  }
  if (!withinJointTolerance(
        fingerprint.expected_right_terminal_q, runtime.right_joint_positions,
        config.joint_tolerance_rad))
  {
    return {PredictiveCacheDecision::RIGHT_TERMINAL_STATE_MISMATCH,
      "right_arm 未到达预测终端状态。"};
  }
  return {PredictiveCacheDecision::HIT,
    "预测缓存与 Ground Truth / 双臂终端状态一致；仍须做最终 FCL 复核后才能执行。"};
}

}  // namespace fr3_dual_palletize
