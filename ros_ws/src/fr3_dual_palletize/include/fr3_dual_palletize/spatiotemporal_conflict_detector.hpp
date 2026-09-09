#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/collision_object.hpp>

#include "fr3_dual_palletize/task_trajectory_candidate.hpp"

namespace fr3_dual_palletize
{

// 一次真实几何碰撞的可读记录。body_a/body_b 保留 MoveIt/FCL 返回的原始名称，
// 方便后续 Task09 决策层判断是双臂、机械臂-携带物还是携带物之间的冲突。
struct ConflictEvent
{
  double time_sec{0.0};
  std::string type;
  std::string body_a;
  std::string body_b;
};

// 同类的连续碰撞采样区间。边界精度由 sample_period_sec 决定。
struct ConflictWindow
{
  double start_time_sec{0.0};
  double end_time_sec{0.0};
  std::string type;
  std::string body_a;
  std::string body_b;
  std::size_t samples{0};
};

struct ConflictReport
{
  bool valid{false};
  bool conflict{false};
  double first_conflict_time_sec{-1.0};
  double horizon_sec{0.0};
  double sample_period_sec{0.01};
  double left_start_delay_sec{0.0};
  double right_start_delay_sec{0.0};
  std::size_t samples_checked{0};
  double wall_time_sec{0.0};
  std::string error;
  // 首个冲突采样点的 contact，兼容 Task08-A 既有日志。
  std::vector<ConflictEvent> events;
  // Task09-B 消费连续冲突窗口，而非只看首个冲突时刻。
  std::vector<ConflictWindow> conflict_windows;
};

// Task08-B：在同一个 14-DoF RobotState 上检查两个完整任务候选。
//
// 该类不连接 Isaac、不发布 ROS command、不修改 move_group 的 Planning Scene。
// 它从传入的静态 CollisionObject 快照构建私有 PlanningScene，并在每个采样点
// 根据 ATTACH/DETACH 事件重建两个 Box 的 World / Attached 状态。
class SpatioTemporalConflictDetector
{
public:
  SpatioTemporalConflictDetector(
    moveit::core::RobotModelConstPtr robot_model,
    std::vector<moveit_msgs::msg::CollisionObject> static_world_objects);

  ConflictReport check(
    const TaskTrajectoryCandidate& left,
    const TaskTrajectoryCandidate& right,
    double sample_period_sec = 0.01,
    double left_start_delay_sec = 0.0,
    double right_start_delay_sec = 0.0) const;

private:
  moveit::core::RobotModelConstPtr robot_model_;
  std::vector<moveit_msgs::msg::CollisionObject> static_world_objects_;
};

}  // namespace fr3_dual_palletize
