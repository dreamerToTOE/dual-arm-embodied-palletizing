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
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "fr3_dual_palletize/robust_planner.hpp"
#include "fr3_dual_palletize/task_trajectory_candidate.hpp"

namespace fr3_dual_palletize
{

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
  // 目标物体在释放后应落到的支撑面高度。默认桌面顶面 z=0.050 m，
  // 因而完全保持 Task04--Task10 的地面码垛几何；Task15 上层码垛只需
  // 按配置提供已落稳下层的顶面高度，而不复制 primitive 状态机。
  double target_support_surface_z{0.050};
  // Task09-B 安全退出扩展：放置完成后追加 MoveIt 规划的安全退出段，终点回到本次候选的 HOME。
  // 默认关闭，保持 Task08-A/B 原有 HOME -> ... -> RETREAT 候选定义不变。
  bool include_safe_egress{false};
  // Task15 的上层码垛中，已通过真实落稳误差验收的小件会请求 Isaac 将其设为
  // kinematic 支撑块；Collider 保留，后续 Cube 仍必须与它发生真实碰撞。
  bool freeze_after_settle{false};
  // Task16：自由空间阶段从多条 RRTConnect 可行候选中选择代价最低者。
  // Task15 固定场景的 benchmark 当前选择 soft_preference 为默认；三种模式仍可
  // 配置切换，并要求在新场景中重新做对照验证。
  RobustPlannerConfig robust_planner;
};

// Task08 协调层读取的候选搬运轨迹。
// primitive 负责“怎么搬”，Coordinator 负责“这条轨迹何时允许执行”。
struct TransferCandidate
{
  std::string label;
  std::string planning_group;
  std::string object_id;
  std::string eef_link;

  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<double> start_q;
  std::vector<double> goal_q;

  double duration_sec{0.0};
  double target_x{0.0};
  double target_y{0.0};
  bool carrying_object{false};
};

// 通用同步闸门。
// Task07 用于 PRE_PICK 同时放行；Task08 额外在 LIFT 后建立 coordination point。
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
  using RobustPlanObserver = std::function<void(
    const std::string& stage_name,
    const RobustPlanResult& result)>;

  PalletizePrimitive(
    rclcpp::Node::SharedPtr node,
    PrimitiveConfig config,
    PoseProvider pose_provider,
    std::shared_ptr<std::mutex> planning_scene_mutex,
    RobustPlanObserver robust_plan_observer = {});

  // Task07 兼容入口：完整执行一次 Task04 canonical primitive。
  bool runOnce(StartGate& start_gate);

  // Task08-A：只生成完整 HOME -> ... -> RETREAT 候选，不执行机器人或吸盘。
  // 为获得正确的携带物碰撞模型，函数会临时修改 MoveIt Planning Scene，
  // 返回前无论成功或失败都会把当前 object 恢复到其初始 world pose。
  bool planTaskTrajectoryCandidate(TaskTrajectoryCandidate& candidate);

  // Task09 执行层使用的细粒度接口。CoordinatedTaskExecutor 以一条共享时钟
  // 采样两条已经通过 Task08-B/FCL 复检的候选；它在离散事件处让两臂共同保持，
  // 因而不会因等待 Surface Gripper 状态而破坏协调后的相对时间关系。
  bool waitForTaskExecutionBridge();
  bool publishTaskTrajectorySample(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    double time_sec);
  bool applyTaskEvent(
    const TaskEvent& event,
    double grasp_timeout_sec,
    double release_timeout_sec);
  void emergencySuctionOff();

  // Task08 新接口：
  // 1) 执行 PRE_PICK -> CONTACT -> SUCTION ON -> ATTACH -> LIFT；
  // 2) 在 coordination_gate 等待另一臂也到 LIFT；
  // 3) 只规划 LIFT -> PRE_PLACE candidate，不执行；
  // 4) 通过 candidate 输出给双臂 Coordinator。
  bool prepareTransferCandidate(
    StartGate& start_gate,
    StartGate& coordination_gate,
    TransferCandidate& candidate);

  // Task09 及以后复用：Coordinator 审批 candidate 后再调用。
  // 执行 candidate，并继续 PLACE -> release -> GT 回写 -> RETREAT。
  bool executePreparedTransferAndFinish(const TransferCandidate& candidate);

private:
  bool waitForIsaacBridge();

  bool configureMoveGroup(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup*& joint_model_group);

  bool prepareTransferCandidateImpl(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    StartGate& start_gate,
    StartGate& coordination_gate,
    TransferCandidate& candidate);

  bool planTaskTrajectoryCandidateImpl(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    TaskTrajectoryCandidate& candidate);

  bool finishPreparedTransferImpl(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const TransferCandidate& candidate);

  geometry_msgs::msg::Pose makeTopDownPose(
    double x, double y, double suction_tcp_world_z, double yaw = 0.0) const;

  bool planPoseStage(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::string& eef_link,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& stage_name,
    trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool planCartesianStage(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& stage_name,
    trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool planJointStage(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& start_q,
    const std::vector<double>& target_q,
    const std::string& stage_name,
    trajectory_msgs::msg::JointTrajectory& trajectory_out);

  bool setStartStateForGroup(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const moveit::core::JointModelGroup* joint_model_group,
    const std::vector<double>& group_q);

  bool executeTrajectory(const trajectory_msgs::msg::JointTrajectory& input);
  std::vector<std::string> isaacJointNames(
    const std::vector<std::string>& moveit_names) const;

  void commandSuction(bool on);
  bool waitForSuctionClosed(bool expected, double timeout_sec);

  void removeWorldObject(const std::string& object_id);
  bool addWorldObject(
    const std::string& object_id,
    const geometry_msgs::msg::Pose& pose);
  bool attachObject(const std::string& object_id, const std::string& eef_link);
  bool detachObject(const std::string& object_id, const std::string& eef_link);

  static std::vector<double> currentFrom(
    const trajectory_msgs::msg::JointTrajectory& trajectory);
  static bool appendTrajectory(
    trajectory_msgs::msg::JointTrajectory& destination,
    const trajectory_msgs::msg::JointTrajectory& stage);
  static bool appendHold(
    trajectory_msgs::msg::JointTrajectory& trajectory,
    double hold_sec);
  static double trajectoryDuration(
    const trajectory_msgs::msg::JointTrajectory& trajectory);

private:
  rclcpp::Node::SharedPtr node_;
  PrimitiveConfig config_;
  PoseProvider pose_provider_;
  std::shared_ptr<std::mutex> planning_scene_mutex_;
  RobustPlanObserver robust_plan_observer_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lock_object_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr suction_state_sub_;

  std::atomic_bool suction_closed_{false};
  std::atomic_bool have_suction_state_{false};
};

}  // namespace fr3_dual_palletize
