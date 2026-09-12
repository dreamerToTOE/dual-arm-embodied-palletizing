#include "fr3_dual_palletize/robust_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

namespace fr3_dual_palletize
{
namespace
{
constexpr double EPSILON = 1.0e-9;
constexpr double LIMIT_COST_EPSILON = 0.02;

double boundedRange(const moveit::core::VariableBounds& bounds)
{
  if (!bounds.position_bounded_ ||
      bounds.max_position_ <= bounds.min_position_ + EPSILON)
  {
    return 1.0;
  }
  return bounds.max_position_ - bounds.min_position_;
}

bool robotModelHasVariable(
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& variable_name)
{
  return robot_model && std::find(
    robot_model->getVariableNames().begin(),
    robot_model->getVariableNames().end(),
    variable_name) != robot_model->getVariableNames().end();
}

}  // namespace

const char* redundancyModeName(RedundancyMode mode)
{
  switch (mode)
  {
    case RedundancyMode::FREE_7DOF:
      return "free_7dof";
    case RedundancyMode::HARD_LOCK_JOINT:
      return "hard_lock_joint";
    case RedundancyMode::SOFT_PREFERENCE:
      return "soft_preference";
  }
  return "unknown";
}

bool parseRedundancyMode(const std::string& value, RedundancyMode& mode)
{
  if (value == "free_7dof")
  {
    mode = RedundancyMode::FREE_7DOF;
    return true;
  }
  if (value == "hard_lock_joint")
  {
    mode = RedundancyMode::HARD_LOCK_JOINT;
    return true;
  }
  if (value == "soft_preference")
  {
    mode = RedundancyMode::SOFT_PREFERENCE;
    return true;
  }
  return false;
}

RobustPlanner::RobustPlanner(rclcpp::Logger logger, RobustPlannerConfig config)
  : logger_(std::move(logger)), config_(std::move(config))
{
}

bool RobustPlanner::validateConfig(std::string& error) const
{
  if (config_.planner_id.empty() || config_.candidate_count == 0 ||
      config_.planning_time_sec <= 0.0 ||
      config_.velocity_scaling <= 0.0 || config_.velocity_scaling > 1.0 ||
      config_.acceleration_scaling <= 0.0 || config_.acceleration_scaling > 1.0 ||
      config_.hard_lock_tolerance < 0.0 ||
      config_.weight_path_length < 0.0 || config_.weight_joint_limit < 0.0 ||
      config_.weight_redundancy < 0.0)
  {
    error = "Task16 RobustPlanner 参数无效。";
    return false;
  }
  return true;
}

void RobustPlanner::configureMoveGroup(
  moveit::planning_interface::MoveGroupInterface& move_group) const
{
  move_group.setPlannerId(config_.planner_id);
  move_group.setPlanningTime(config_.planning_time_sec);
  // 候选数量由本类显式管理，不能让一次 MoveIt 请求隐藏多次内部尝试。
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(config_.velocity_scaling);
  move_group.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
}

std::string RobustPlanner::resolveRedundantJointName(
  const RobustPlannerConfig& config,
  const moveit::core::JointModelGroup* joint_model_group)
{
  if (!config.redundant_joint_name.empty())
  {
    return config.redundant_joint_name;
  }
  if (joint_model_group == nullptr || joint_model_group->getVariableNames().empty())
  {
    return "";
  }
  return joint_model_group->getVariableNames().back();
}

bool RobustPlanner::applyRedundancyConstraint(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& start_state,
  const moveit::core::JointModelGroup* joint_model_group,
  std::string& redundant_joint_name,
  double& preferred_position,
  std::string& error) const
{
  redundant_joint_name.clear();
  preferred_position = std::numeric_limits<double>::quiet_NaN();
  move_group.clearPathConstraints();

  if (config_.redundancy_mode == RedundancyMode::FREE_7DOF)
  {
    return true;
  }

  redundant_joint_name = resolveRedundantJointName(config_, joint_model_group);
  if (redundant_joint_name.empty() ||
      !robotModelHasVariable(start_state.getRobotModel(), redundant_joint_name))
  {
    error = "无法解析 Task16 冗余关节。";
    return false;
  }

  preferred_position = std::isfinite(config_.preferred_redundant_joint) ?
    config_.preferred_redundant_joint : start_state.getVariablePosition(redundant_joint_name);

  const auto& bounds = start_state.getRobotModel()->getVariableBounds(redundant_joint_name);
  if (bounds.position_bounded_ &&
      (preferred_position < bounds.min_position_ - EPSILON ||
       preferred_position > bounds.max_position_ + EPSILON))
  {
    error = "Task16 冗余关节偏好值超出 joint limit。";
    return false;
  }

  if (config_.redundancy_mode != RedundancyMode::HARD_LOCK_JOINT)
  {
    return true;
  }

  moveit_msgs::msg::Constraints constraints;
  constraints.name = "task16_hard_redundancy_lock";
  moveit_msgs::msg::JointConstraint joint_constraint;
  joint_constraint.joint_name = redundant_joint_name;
  joint_constraint.position = preferred_position;
  joint_constraint.tolerance_above = config_.hard_lock_tolerance;
  joint_constraint.tolerance_below = config_.hard_lock_tolerance;
  joint_constraint.weight = 1.0;
  constraints.joint_constraints.push_back(joint_constraint);
  move_group.setPathConstraints(constraints);
  return true;
}

RobustCandidateMetrics RobustPlanner::scoreCandidate(
  std::size_t attempt_index,
  double planning_time_sec,
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& redundant_joint_name,
  double preferred_position) const
{
  RobustCandidateMetrics metrics;
  metrics.attempt_index = attempt_index;
  metrics.plan_success = true;
  metrics.planning_time_sec = planning_time_sec;

  if (!robot_model || trajectory.points.empty() || trajectory.joint_names.empty())
  {
    metrics.plan_success = false;
    metrics.failure_reason = "RRTConnect 返回的 JointTrajectory 不完整。";
    return metrics;
  }

  double minimum_margin = 0.5;
  bool found_bounded_joint = false;
  double redundancy_sum = 0.0;
  std::size_t redundancy_samples = 0;

  for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index)
  {
    const auto& point = trajectory.points[point_index];
    if (point.positions.size() != trajectory.joint_names.size())
    {
      metrics.plan_success = false;
      metrics.failure_reason = "JointTrajectory position 数量不匹配。";
      return metrics;
    }

    double segment_squared = 0.0;
    for (std::size_t joint = 0; joint < trajectory.joint_names.size(); ++joint)
    {
      const auto& name = trajectory.joint_names[joint];
      if (!robotModelHasVariable(robot_model, name))
      {
        continue;
      }
      const auto& bounds = robot_model->getVariableBounds(name);
      const double range = boundedRange(bounds);
      const double position = point.positions[joint];

      if (bounds.position_bounded_)
      {
        const double margin = std::min(
          (position - bounds.min_position_) / range,
          (bounds.max_position_ - position) / range);
        minimum_margin = std::min(minimum_margin, margin);
        found_bounded_joint = true;
      }

      if (point_index > 0)
      {
        const double delta = position - trajectory.points[point_index - 1].positions[joint];
        segment_squared += (delta / range) * (delta / range);
      }

      if (config_.redundancy_mode == RedundancyMode::SOFT_PREFERENCE &&
          name == redundant_joint_name && std::isfinite(preferred_position))
      {
        const double normalized_error = (position - preferred_position) / range;
        redundancy_sum += normalized_error * normalized_error;
        ++redundancy_samples;
      }
    }
    if (point_index > 0)
    {
      metrics.path_length += std::sqrt(segment_squared);
    }
  }

  metrics.min_joint_limit_margin = found_bounded_joint ? minimum_margin : 0.5;
  metrics.joint_limit_cost = 1.0 /
    std::max(EPSILON, metrics.min_joint_limit_margin + LIMIT_COST_EPSILON);
  metrics.redundancy_cost = redundancy_samples == 0 ? 0.0 :
    redundancy_sum / static_cast<double>(redundancy_samples);
  metrics.total_cost = config_.weight_path_length * metrics.path_length +
    config_.weight_joint_limit * metrics.joint_limit_cost +
    config_.weight_redundancy * metrics.redundancy_cost;
  return metrics;
}

RobustPlanResult RobustPlanner::planPose(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& start_state,
  const moveit::core::JointModelGroup* joint_model_group,
  const geometry_msgs::msg::Pose& target_pose,
  const std::string& eef_link,
  const std::string& stage_name) const
{
  RobustPlanResult result;
  result.requested_candidate_count = config_.candidate_count;
  if (!validateConfig(result.error) || joint_model_group == nullptr)
  {
    if (joint_model_group == nullptr)
    {
      result.error = "Task16 RobustPlanner 缺少 JointModelGroup。";
    }
    return result;
  }

  configureMoveGroup(move_group);
  move_group.clearPoseTargets();
  if (!move_group.setPoseTarget(target_pose, eef_link))
  {
    result.error = "Task16 无法设置 Pose target。";
    return result;
  }

  std::string redundant_joint_name;
  double preferred_position = std::numeric_limits<double>::quiet_NaN();
  if (!applyRedundancyConstraint(
        move_group, start_state, joint_model_group, redundant_joint_name, preferred_position,
        result.error))
  {
    move_group.clearPoseTargets();
    return result;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t attempt = 1; attempt <= config_.candidate_count; ++attempt)
  {
    move_group.setStartState(start_state);
    const auto begin = std::chrono::steady_clock::now();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      !plan.trajectory_.joint_trajectory.points.empty();
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin).count();
    result.total_planning_time_sec += elapsed;

    if (!success)
    {
      RobustCandidateMetrics metrics;
      metrics.attempt_index = attempt;
      metrics.planning_time_sec = elapsed;
      metrics.failure_reason = "RRTConnect 未返回可执行轨迹。";
      result.candidates.push_back(metrics);
      RCLCPP_WARN(
        logger_, "[Task16] %s candidate=%zu/%zu FAIL, time=%.3f s",
        stage_name.c_str(), attempt, config_.candidate_count, elapsed);
      continue;
    }

    auto metrics = scoreCandidate(
      attempt, elapsed, plan.trajectory_.joint_trajectory, move_group.getRobotModel(),
      redundant_joint_name, preferred_position);
    result.candidates.push_back(metrics);
    if (!metrics.plan_success)
    {
      RCLCPP_WARN(
        logger_, "[Task16] %s candidate=%zu/%zu INVALID: %s",
        stage_name.c_str(), attempt, config_.candidate_count, metrics.failure_reason.c_str());
      continue;
    }

    ++result.successful_candidate_count;
    RCLCPP_INFO(
      logger_, "[Task16] %s candidate=%zu/%zu OK: J=%.4f, length=%.4f, margin=%.4f, time=%.3f s",
      stage_name.c_str(), attempt, config_.candidate_count, metrics.total_cost,
      metrics.path_length, metrics.min_joint_limit_margin, elapsed);
    if (metrics.total_cost < best_cost)
    {
      best_cost = metrics.total_cost;
      result.selected_attempt_index = attempt;
      result.selected_trajectory = plan.trajectory_.joint_trajectory;
    }
  }

  move_group.clearPathConstraints();
  move_group.clearPoseTargets();
  if (result.successful_candidate_count == 0)
  {
    result.error = "Task16 多候选 RRTConnect 未找到可行 Pose 轨迹。";
    return result;
  }
  result.valid = true;
  RCLCPP_INFO(
    logger_, "[Task16] %s SELECT candidate=%zu/%zu, success=%zu, total_plan_time=%.3f s",
    stage_name.c_str(), result.selected_attempt_index, config_.candidate_count,
    result.successful_candidate_count, result.total_planning_time_sec);
  return result;
}

RobustPlanResult RobustPlanner::planJointTarget(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::RobotState& start_state,
  const moveit::core::JointModelGroup* joint_model_group,
  const std::vector<double>& target_q,
  const std::string& stage_name) const
{
  RobustPlanResult result;
  result.requested_candidate_count = config_.candidate_count;
  if (!validateConfig(result.error) || joint_model_group == nullptr ||
      target_q.size() != joint_model_group->getVariableCount())
  {
    if (result.error.empty())
    {
      result.error = "Task16 Joint target 与 JointModelGroup 不匹配。";
    }
    return result;
  }

  configureMoveGroup(move_group);
  move_group.clearPoseTargets();
  if (!move_group.setJointValueTarget(target_q))
  {
    result.error = "Task16 无法设置 Joint target。";
    return result;
  }

  std::string redundant_joint_name;
  double preferred_position = std::numeric_limits<double>::quiet_NaN();
  if (!applyRedundancyConstraint(
        move_group, start_state, joint_model_group, redundant_joint_name, preferred_position,
        result.error))
  {
    return result;
  }

  double best_cost = std::numeric_limits<double>::infinity();
  for (std::size_t attempt = 1; attempt <= config_.candidate_count; ++attempt)
  {
    move_group.setStartState(start_state);
    const auto begin = std::chrono::steady_clock::now();
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool success = move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
      !plan.trajectory_.joint_trajectory.points.empty();
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - begin).count();
    result.total_planning_time_sec += elapsed;

    if (!success)
    {
      RobustCandidateMetrics metrics;
      metrics.attempt_index = attempt;
      metrics.planning_time_sec = elapsed;
      metrics.failure_reason = "RRTConnect 未返回可执行轨迹。";
      result.candidates.push_back(metrics);
      RCLCPP_WARN(
        logger_, "[Task16] %s candidate=%zu/%zu FAIL, time=%.3f s",
        stage_name.c_str(), attempt, config_.candidate_count, elapsed);
      continue;
    }

    auto metrics = scoreCandidate(
      attempt, elapsed, plan.trajectory_.joint_trajectory, move_group.getRobotModel(),
      redundant_joint_name, preferred_position);
    result.candidates.push_back(metrics);
    if (!metrics.plan_success)
    {
      continue;
    }
    ++result.successful_candidate_count;
    RCLCPP_INFO(
      logger_, "[Task16] %s candidate=%zu/%zu OK: J=%.4f, length=%.4f, margin=%.4f, time=%.3f s",
      stage_name.c_str(), attempt, config_.candidate_count, metrics.total_cost,
      metrics.path_length, metrics.min_joint_limit_margin, elapsed);
    if (metrics.total_cost < best_cost)
    {
      best_cost = metrics.total_cost;
      result.selected_attempt_index = attempt;
      result.selected_trajectory = plan.trajectory_.joint_trajectory;
    }
  }

  move_group.clearPathConstraints();
  if (result.successful_candidate_count == 0)
  {
    result.error = "Task16 多候选 RRTConnect 未找到可行 Joint 轨迹。";
    return result;
  }
  result.valid = true;
  RCLCPP_INFO(
    logger_, "[Task16] %s SELECT candidate=%zu/%zu, success=%zu, total_plan_time=%.3f s",
    stage_name.c_str(), result.selected_attempt_index, config_.candidate_count,
    result.successful_candidate_count, result.total_planning_time_sec);
  return result;
}

}  // namespace fr3_dual_palletize
