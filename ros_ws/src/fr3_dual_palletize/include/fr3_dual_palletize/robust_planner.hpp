#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/logger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace fr3_dual_palletize
{

// Task16 的三组 FR3 冗余实验。FREE_7DOF 与硬锁定仅作为 benchmark 对照；
// Task15 固定场景的正式数据表明软偏好保留可达性且具有更低的候选代价，
// 因而作为当前默认。后续场景扩展时仍须重新 benchmark。
enum class RedundancyMode
{
  FREE_7DOF,
  HARD_LOCK_JOINT,
  SOFT_PREFERENCE,
};

const char* redundancyModeName(RedundancyMode mode);
bool parseRedundancyMode(const std::string& value, RedundancyMode& mode);

struct RobustPlannerConfig
{
  std::string planner_id{"RRTConnectkConfigDefault"};
  std::size_t candidate_count{3};
  double planning_time_sec{5.0};
  double velocity_scaling{0.20};
  double acceleration_scaling{0.20};

  RedundancyMode redundancy_mode{RedundancyMode::SOFT_PREFERENCE};
  // 空字符串表示当前 planning group 的最后一个变量。FR3 arm group 中即 joint7。
  std::string redundant_joint_name;
  // NaN 表示使用当前 stage start state 中该关节的位置作为偏好/锁定值。
  double preferred_redundant_joint{std::numeric_limits<double>::quiet_NaN()};
  double hard_lock_tolerance{0.005};

  // 第一版使用可解释的质量项。所有路径均已由 MoveIt 碰撞检查；本 Task
  // 暂不把近碰距离和奇异性作为硬门槛，保留接口供后续 benchmark 扩展。
  double weight_path_length{1.0};
  double weight_joint_limit{0.20};
  double weight_redundancy{0.20};
};

struct RobustCandidateMetrics
{
  std::size_t attempt_index{0};
  bool plan_success{false};
  double planning_time_sec{0.0};
  double path_length{0.0};
  // 所有有界关节、所有 trajectory point 中最小的归一化关节余量，范围 [0, 0.5]。
  double min_joint_limit_margin{0.0};
  // 越靠近 joint limit 值越高；用于排名，不作为碰撞替代品。
  double joint_limit_cost{0.0};
  double redundancy_cost{0.0};
  double total_cost{std::numeric_limits<double>::infinity()};
  std::string failure_reason;
};

struct RobustPlanResult
{
  bool valid{false};
  std::string error;
  std::size_t requested_candidate_count{0};
  std::size_t successful_candidate_count{0};
  std::size_t selected_attempt_index{0};
  double total_planning_time_sec{0.0};
  trajectory_msgs::msg::JointTrajectory selected_trajectory;
  std::vector<RobustCandidateMetrics> candidates;
};

// 对同一起点/目标重复调用 RRTConnect，随后在明确定义的代价函数下选择候选。
// 它不改变 MoveIt 的碰撞语义：每条候选仍须先由 MoveIt 全 Planning Scene 验证。
class RobustPlanner
{
public:
  RobustPlanner(rclcpp::Logger logger, RobustPlannerConfig config);

  RobustPlanResult planPose(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::RobotState& start_state,
    const moveit::core::JointModelGroup* joint_model_group,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& eef_link,
    const std::string& stage_name) const;

  RobustPlanResult planJointTarget(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::RobotState& start_state,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& target_q,
    const std::string& stage_name) const;

private:
  bool validateConfig(std::string& error) const;
  void configureMoveGroup(
    moveit::planning_interface::MoveGroupInterface& move_group) const;
  bool applyRedundancyConstraint(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::RobotState& start_state,
    const moveit::core::JointModelGroup* joint_model_group,
    std::string& redundant_joint_name,
    double& preferred_position,
    std::string& error) const;
  RobustCandidateMetrics scoreCandidate(
    std::size_t attempt_index,
    double planning_time_sec,
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& redundant_joint_name,
    double preferred_position) const;
  static std::string resolveRedundantJointName(
    const RobustPlannerConfig& config,
    const moveit::core::JointModelGroup* joint_model_group);

  rclcpp::Logger logger_;
  RobustPlannerConfig config_;
};

}  // namespace fr3_dual_palletize
