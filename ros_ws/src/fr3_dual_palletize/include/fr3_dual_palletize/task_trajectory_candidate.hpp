#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "task_event.hpp"

namespace fr3_dual_palletize
{

struct TaskTrajectoryCandidate
{
  std::string arm_name;

  // Complete task trajectory, separated by execution stages.
  std::vector<trajectory_msgs::msg::JointTrajectory> trajectory_segments;

  double duration_sec{0.0};

  // Object state changes during the task.
  std::vector<TaskEvent> events;

  void printSummary() const
  {
    std::cout << "===== Task Trajectory Candidate =====\n";
    std::cout << "arm: " << arm_name << "\n";
    std::cout << "duration: " << duration_sec << " s\n";
    std::cout << "events:" << std::endl;

    for (const auto &event : events)
    {
      std::cout << "  t=" << event.time_sec << " ";
      if (event.type == TaskEventType::ATTACH)
      {
        std::cout << "ATTACH ";
      }
      else
      {
        std::cout << "DETACH ";
      }
      std::cout << event.object_name << std::endl;
    }
  }
};

}  // namespace fr3_dual_palletize
