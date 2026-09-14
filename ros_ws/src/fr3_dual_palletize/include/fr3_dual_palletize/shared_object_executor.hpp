#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "fr3_dual_palletize/palletizing_job.hpp"
#include "fr3_dual_palletize/shared_object_planner.hpp"

namespace fr3_dual_palletize
{

// Task20-C tight：只消费已经由 SharedObjectPlanner 完整预检的共同物体候选。
// 物理时间可以放慢，但不会改变预检的 joint path、共同阶段顺序或抓放语义。
struct SharedObjectExecutionConfig
{
  double command_period_sec{0.010};
  double execution_time_scale{2.0};
  double grasp_timeout_sec{3.0};
  double release_timeout_sec{3.0};
  double settle_sec{1.0};
  double final_hold_sec{0.50};
  double stage_position_tolerance_m{0.010};
  double placement_position_tolerance_m{0.010};
  double placement_orientation_tolerance_rad{0.052};
  double relative_tcp_tolerance_m{0.005};
};

struct SharedObjectExecutionResult
{
  bool completed{false};
  std::size_t stages_executed{0};
  double wall_duration_sec{0.0};
  double max_stage_position_error_m{0.0};
  double max_relative_tcp_error_m{0.0};
  double placement_position_error_m{0.0};
  double placement_orientation_error_rad{0.0};
  std::string error;
};

// 对真实 Isaac Surface Gripper 的共同物体执行器。MoveIt 无法把同一个
// CollisionObject 同时 attach 给两个 link，所以共同搬运阶段不伪造双 attach：
// 它只执行 SharedObjectPlanner 已通过的 private-scene FCL/TCP 轨迹；物理释放
// 后再以 Ground Truth 将物体写回外部 MoveIt World，随后执行预规划退出。
class SharedObjectExecutor
{
public:
  using PoseProvider = std::function<geometry_msgs::msg::Pose()>;

  SharedObjectExecutor(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<std::mutex> planning_scene_mutex,
    SharedObjectExecutionConfig config = SharedObjectExecutionConfig{});

  bool execute(
    const BoxSpec& box,
    const PlacementSpec& placement,
    const SharedObjectPlan& plan,
    const PoseProvider& box_pose_provider,
    SharedObjectExecutionResult& result);

private:
  bool waitForBridge() const;
  bool executeOne(
    bool left,
    const trajectory_msgs::msg::JointTrajectory& trajectory) const;
  bool executeTogether(const SharedObjectStage& stage) const;
  bool commandSuction(bool left, bool on) const;
  bool waitForSuction(bool left, bool expected, double timeout_sec) const;
  bool getTcpPose(bool left, geometry_msgs::msg::Pose& pose) const;
  bool writeWorldObject(const BoxSpec& box, const geometry_msgs::msg::Pose& pose) const;
  void emergencySuctionOff() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::mutex> planning_scene_mutex_;
  SharedObjectExecutionConfig config_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_suction_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_suction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lock_object_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_suction_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_suction_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_tcp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_tcp_sub_;

  std::atomic_bool left_suction_closed_{false};
  std::atomic_bool right_suction_closed_{false};
  std::atomic_bool have_left_suction_state_{false};
  std::atomic_bool have_right_suction_state_{false};
  mutable std::mutex tcp_mutex_;
  geometry_msgs::msg::Pose left_tcp_pose_;
  geometry_msgs::msg::Pose right_tcp_pose_;
  bool have_left_tcp_pose_{false};
  bool have_right_tcp_pose_{false};
};

}  // namespace fr3_dual_palletize
