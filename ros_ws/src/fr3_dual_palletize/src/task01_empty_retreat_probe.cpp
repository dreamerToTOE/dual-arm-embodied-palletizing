// [ENGINEERING] 只读 FK / 退路单测，直接调用运行节点的实现；不发布任何机器人命令。
#define TASK27_FIVE_CUBE 1
#define TASK01_CUBE04_PRECISION_INSERT 1
#define main task01_unexecuted_controller_main
#include "task26_truck_box_push_in.cpp"
#undef main

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("task01_empty_retreat_probe");
  if (!copyRobotDescriptions(node))
  {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&]() { executor.spin(); });
  bool passed = false;
  try
  {
    moveit::planning_interface::MoveGroupInterface group(node, "left_arm");
    const auto model = group.getRobotModel();
    const auto* left = model->getJointModelGroup("left_arm");
    const auto* right = model->getJointModelGroup("right_arm");
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    const std::vector<double> home(kHomeQ.begin(), kHomeQ.end());
    state.setJointGroupPositions(left, home);
    state.setJointGroupPositions(right, home);
    state.update();
    const auto target = [&](const std::string& eef, double z)
    {
      const auto& transform = state.getGlobalLinkTransform(eef);
      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.translation().x();
      pose.position.y = transform.translation().y();
      pose.position.z = transform.translation().z() + z;
      const Eigen::Quaterniond q(transform.rotation());
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();
      pose.orientation.w = q.w();
      return pose;
    };
    trajectory_msgs::msg::JointTrajectory left_path, right_path, rejected;
    const std::string left_eef = "left_fr3_side_suction_tcp";
    const std::string right_eef = "right_fr3_side_suction_tcp";
    passed = planSeededEmptyCartesian(node, state, left, left_eef,
      target(left_eef, 0.02), "UNIT left 20mm", &left_path) &&
      planSeededEmptyCartesian(node, state, right, right_eef,
      target(right_eef, 0.02), "UNIT right 20mm", &right_path) &&
      synchronize(&left_path, &right_path) &&
      validateSync(node, model, {}, left_path, right_path, "UNIT unobstructed") &&
      !planSeededEmptyCartesian(node, state, left, left_eef,
        target(left_eef, 0.8), "UNIT oversized reject", &rejected);
    auto invalid = target(left_eef, 0.02);
    invalid.orientation = geometry_msgs::msg::Quaternion{};
    passed = passed && !planSeededEmptyCartesian(
      node, state, left, left_eef, invalid, "UNIT quaternion reject", &rejected);
    const auto obstructing_cube = cubeObject("unit_released_cube", target(left_eef, 0.0));
    passed = passed && !validateSync(node, model, {obstructing_cube},
      left_path, right_path, "UNIT released Cube collision must reject");
    trajectory_msgs::msg::JointTrajectory zero;
    passed = passed && planSeededEmptyCartesian(node, state, left, left_eef,
      target(left_eef, 0.0), "UNIT zero displacement", &zero) &&
      zero.points.size() >= 2 && pointTime(zero.points.back()) > 0.0;
    RCLCPP_INFO(node->get_logger(), "EMPTY_RETREAT_READ_ONLY_TEST %s; no joint/suction publisher constructed.",
      passed ? "PASS" : "FAIL");
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "Empty retreat probe exception: %s", error.what());
  }
  executor.cancel();
  spin.join();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
