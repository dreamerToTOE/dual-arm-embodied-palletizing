#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "fr3_dual_palletize/task_event.hpp"

namespace fr3_dual_palletize
{

// 一台机械臂从当前 HOME/起始状态到 RETREAT 的完整预测任务。
// 它只描述计划，不下发 Isaac joint command 或 suction command。
struct TaskTrajectoryCandidate
{
  // 保留初版接口字段，便于轻量级 demo 和后续日志复用。
  std::string arm_name;
  std::vector<trajectory_msgs::msg::JointTrajectory> trajectory_segments;

  std::string label;
  std::string planning_group;
  std::string object_id;
  std::string eef_link;

  // 供 Task08-B 以统一时间轴采样的完整拼接轨迹。
  trajectory_msgs::msg::JointTrajectory trajectory;
  std::vector<double> start_q;
  std::vector<double> goal_q;
  double duration_sec{0.0};

  geometry_msgs::msg::Pose initial_object_pose;
  geometry_msgs::msg::Pose planned_release_pose;
  std::vector<TaskEvent> events;

  void printSummary() const
  {
    std::cout << "===== Task Trajectory Candidate =====\n";
    std::cout << "arm: " << arm_name << "\n";
    std::cout << "duration: " << duration_sec << " s\n";
    std::cout << "events:" << std::endl;

    for (const auto& event : events)
    {
      std::cout << "  t=" << event.time_sec << " "
                << taskEventTypeName(event.type) << " "
                << event.object_name << std::endl;
    }
  }
};

}  // namespace fr3_dual_palletize
