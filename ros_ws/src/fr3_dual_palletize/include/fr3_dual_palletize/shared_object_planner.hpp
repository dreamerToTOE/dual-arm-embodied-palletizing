#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "fr3_dual_palletize/palletizing_job.hpp"

namespace fr3_dual_palletize
{

// 通用紧协调 planner 的输入只描述任务事实：一个运行时 Box、一个自动生成的
// PlacementSpec 和两个局部顶面抓取点。它不绑定 Task11--Task15 的 object id、
// 绝对坐标或固定抓取偏置。
struct SharedObjectPlanConfig
{
  double suction_tcp_offset_z{0.105};
  double contact_clearance_z{0.001};
  double pre_contact_clearance_z{0.065};
  double lift_height{0.050};
  double cartesian_eef_step{0.002};
  double cartesian_min_fraction{0.999};
  double collision_sample_period_sec{0.010};
  // 这是离线双 IK candidate 的相对误差，而不是相对某个按时间线性插值的
  // Cartesian 参考点。4 mm 仅吸收两条独立 MoveIt IK 离散路径的数值差异；
  // 真正 PhysX 执行仍由 Task12/Task14 的 3 mm 实测几何验收把关。
  double tcp_position_tolerance_m{0.004};
  double tcp_orientation_tolerance_rad{0.050};
  // 共享物体由两个实际 TCP 反推后，仍须处于本阶段规划 Box 线段附近。检查点到
  // 线段的横向距离，而不要求与 joint-space 时间参数化具有相同瞬时进度。
  double box_path_tracking_tolerance_m{0.005};
  int max_planning_retries{3};
};

struct SharedObjectStage
{
  std::string name;
  trajectory_msgs::msg::JointTrajectory left;
  trajectory_msgs::msg::JointTrajectory right;
  geometry_msgs::msg::Pose box_start;
  geometry_msgs::msg::Pose box_end;
  // CONTACT 是尚未吸附的接近段；共同 lift/transport/descent/retreat 才要求
  // 两个 TCP 严格跟随 Box 上的对应局部 grasp 点。
  bool enforce_grasp_constraint{true};
  double duration_sec{0.0};
  std::size_t fcl_samples{0};
};

struct SharedObjectPlan
{
  bool valid{false};
  std::string diagnostics;
  GraspCandidate left_grasp;
  GraspCandidate right_grasp;
  std::vector<SharedObjectStage> stages;
  double duration_sec{0.0};
  double planning_cost{0.0};
  std::size_t fcl_samples{0};
  double max_left_tcp_error_m{0.0};
  double max_right_tcp_error_m{0.0};
  double max_tcp_orientation_error_rad{0.0};
  double max_box_path_tracking_error_m{0.0};
};

// Task21 tight route 的只读几何门禁。它复用 Task11--13 的共同阶段：
// PRE_CONTACT -> CONTACT -> LIFT -> TRANSPORT -> DESCENT -> RETREAT。
// 在每个共同阶段以 10 ms 采样完整双臂 RobotState，并把共享 Box 作为真实
// 碰撞体加入私有 PlanningScene；仅允许 Box 与两只 compact_suction 的预期接触。
// 此类不发布机器人/吸盘命令，完成后将输入 Box 恢复到 MoveIt World 初始 pose。
class SharedObjectPlanner
{
public:
  SharedObjectPlanner(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<std::mutex> planning_scene_mutex,
    SharedObjectPlanConfig config = SharedObjectPlanConfig{});

  bool plan(
    const BoxSpec& box,
    const PlacementSpec& placement,
    SharedObjectPlan& output);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<std::mutex> planning_scene_mutex_;
  SharedObjectPlanConfig config_;
};

}  // namespace fr3_dual_palletize
