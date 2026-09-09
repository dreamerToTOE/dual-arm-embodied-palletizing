#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "fr3_dual_palletize/task_trajectory_candidate.hpp"

namespace fr3_dual_palletize
{

class Task08CandidateDemo : public rclcpp::Node
{
public:
  Task08CandidateDemo()
  : Node("task08_full_task_candidate_demo")
  {
    TaskTrajectoryCandidate candidate;
    candidate.arm_name = "fr3_left_arm";
    candidate.duration_sec = 10.0;

    candidate.events.push_back({2.0, TaskEventType::ATTACH, "cube_A"});
    candidate.events.push_back({8.0, TaskEventType::DETACH, "cube_A"});

    candidate.printSummary();
  }
};

}  // namespace fr3_dual_palletize

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fr3_dual_palletize::Task08CandidateDemo>());
  rclcpp::shutdown();
  return 0;
}
