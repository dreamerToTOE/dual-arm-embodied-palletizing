#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace fr3_dual_palletize
{

// Task07 只增加“双臂并行”能力。
// 单臂内部时序严格沿用 Task04 已验证 primitive，不在这里重新发明抓放流程。
struct PrimitiveConfig
{
  std::string label;
  std::string planning_group;
  std::string eef_link;
  std::string tool_link;
  std::string moveit_joint_prefix;
  std::string joint_command_topic;
  std::string suction_command_topic;
  std::string suction_state_topic;
  std::string object_id;
  std::size_t pose_index{0};
  double target_x{0.0};
  double target_y{0.0};
};

// 两个 primitive 在第一段实际运动前汇合，确保 Task07 真正同时启动。
class StartGate
{
public:
  explicit StartGate(std::size_t participants);

  bool arriveAndWait();
  void cancel();

private:
  const std::size_t participants_;
  std::size_t arrived_{0};
  bool released_{false};
  bool cancelled_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
};

class PalletizePrimitive
{
public:
  using PoseProvider = std::function<geometry_msgs::msg::Pose(std::size_t)>;

  PalletizePrimitive(
    rclcpp::Node::SharedPtr node,
    PrimitiveConfig config,
    PoseProvider pose_provider,
    std::shared_ptr<std::mutex> planning_scene_mutex);

  // 完整执行一次 Task04 canonical primitive：
  // PRE_PICK -> CONTACT -> SUCTION ON -> ATTACH -> LIFT -> PRE_PLACE
  // -> PLACE -> 先规划 RETREAT -> SUCTION OFF -> settle -> GT 回写 -> RETREAT。
  bool runOnce(StartGate& start_gate);

private:
  bool waitForIsaacBridge();

  geometry_msgs::msg::Pose makeTopDownPose(
    double x, double y, double suction_tcp_world_z, double yaw = 0.0) const;

  bool planPoseStage(
    class moveit::planning_interface::MoveGroupInterface& move_group,
    const class moveit::core::JointModelGroup* joint_model_group,
    const std::string& eef_link,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& stage_name,
    trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool planCartesianStage(
    class moveit::planning_interface::MoveGroupInterface& move_group,
    const class moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& stage_name,
    trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool setStartStateForGroup(
    class moveit::planning_interface::MoveGroupInterface& move_group,
    const class moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& group_q);

  bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& input);
  std::vector<std::string> isaacJointNames(const std::vector<std::string>& moveit_names) const;

  void commandSuction(bool on);
  bool waitForSuctionClosed(bool expected, double timeout_sec);

  void removeWorldObject(const std::string& object_id);
  bool addWorldObject(const std::string& object_id, const geometry_msgs::msg::Pose& pose);
  bool attachObject(const std::string& object_id, const std::string& eef_link);
  bool detachObject(const std::string& object_id, const std::string& eef_link);

  static std::vector<double> currentFrom(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

private:
  rclcpp::Node::SharedPtr node_;
  PrimitiveConfig config_;
  PoseProvider pose_provider_;
  std::shared_ptr<std::mutex> planning_scene_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_state_sub_;

  std::atomic_bool suction_closed_{false};
  std::atomic_bool have_suction_state_{false};
};

}  // namespace fr3_dual_palletize
