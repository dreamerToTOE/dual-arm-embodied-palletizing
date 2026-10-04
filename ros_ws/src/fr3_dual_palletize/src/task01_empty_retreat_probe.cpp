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
    // ROS geometry_msgs 的默认 w=1，不是无效的全零四元数。
    invalid.orientation.x = invalid.orientation.y = invalid.orientation.z = invalid.orientation.w = 0.0;
    passed = passed && !planSeededEmptyCartesian(
      node, state, left, left_eef, invalid, "UNIT quaternion reject", &rejected);
    const auto obstructing_cube = cubeObject("unit_released_cube", target(left_eef, 0.0));
    passed = passed && !validateSync(node, model, {obstructing_cube},
      left_path, right_path, "UNIT released Cube collision must reject");
    trajectory_msgs::msg::JointTrajectory zero;
    passed = passed && planSeededEmptyCartesian(node, state, left, left_eef,
      target(left_eef, 0.0), "UNIT zero displacement", &zero) &&
      zero.points.size() >= 2 && pointTime(zero.points.back()) > 0.0;
    // [ENGINEERING] 同时覆盖新 XYZ 微调在真实 RobotModel 上的细 IK / 联合 FCL。
    // 这里不构造 Arm，也不创建机器人/吸盘 publisher。
    const auto xyz_target = [&](const std::string& eef, double direction)
    {
      auto pose = target(eef, 0.0);
      const auto delta = fr3_dual_palletize::precloseAlignmentDeltaXYZ(
        {direction * .0003, direction * .0004, .0005}, {0, 0, 0});
      pose.position.x += delta[0]; pose.position.y += delta[1]; pose.position.z += delta[2];
      return pose;
    };
    const auto xyz_left_goal = xyz_target(left_eef, 1);
    const auto xyz_right_goal = xyz_target(right_eef, -1);
    trajectory_msgs::msg::JointTrajectory xyz_left, xyz_right;
    const bool xyz_safe = planFinePreclose(node, state, left, left_eef,
      xyz_left_goal, "UNIT left XYZ", &xyz_left) &&
      planFinePreclose(node, state, right, right_eef,
      xyz_right_goal, "UNIT right XYZ", &xyz_right) &&
      synchronize(&xyz_left, &xyz_right) &&
      cartesianLineDeviation(model, left_eef, xyz_left, xyz_left_goal) <= kMaxCartesianLineDeviation &&
      cartesianLineDeviation(model, right_eef, xyz_right, xyz_right_goal) <= kMaxCartesianLineDeviation &&
      validateSync(node, model, {}, xyz_left, xyz_right, "UNIT combined XYZ") &&
      !validateSync(node, model, {obstructing_cube}, xyz_left, xyz_right,
        "UNIT XYZ obstruction must reject");
    passed = passed && xyz_safe;
    RCLCPP_INFO(node->get_logger(), "PRE_CLOSE_XYZ_READ_ONLY_TEST %s; no robot commands.",
      xyz_safe ? "PASS" : "FAIL");
    // [EXPERIMENTAL] 已执行中心件落桌日志的四位小数 seed 重放。
    // 主运行可能普通 IK 就通过；这个无命令测试必须直接覆盖长距离备用求解器。
    moveit::core::RobotState replay(model);
    replay.setToDefaultValues();
    replay.setJointGroupPositions(left,
      std::vector<double>{0.9743, -0.3174, 0.3022, -2.6567, 0.1309, 2.3462, 1.1693});
    replay.setJointGroupPositions(right,
      std::vector<double>{-1.2243, -0.3044, -0.0443, -2.6584, -0.0178, 2.3519, -1.2533});
    replay.update();
    trajectory_msgs::msg::JointTrajectory replay_left, replay_right;
    const double exit_z = kTableTopZ + kCubeHalf + kReleaseGapZ + kLiftHeight;
    const bool replay_safe = planSeededEmptyCartesian(node, replay, left, left_eef,
      sidePose(kPrePushX - 0.10, -0.32, exit_z, true), "REPLAY rounded left empty exit", &replay_left) &&
      planSeededEmptyCartesian(node, replay, right, right_eef,
      sidePose(kPrePushX, kCubeHalf + kSideContactCommandGap, exit_z, false),
      "REPLAY rounded right empty exit", &replay_right) &&
      synchronize(&replay_left, &replay_right) &&
      validateSync(node, model, {cubeObject("replay_released_cube", worldPose(0.7712, 0.0001, 0.2600))},
        replay_left, replay_right, "REPLAY released Cube included");
    passed = passed && replay_safe;
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
