// [EXPERIMENTAL] 名义 rear+side 接触链无命令筛选，不等价于物理可行证明。
// 不构造 Arm、不发布 joint/suction/feed/rail，不改远程 Planning Scene/ACM。
#define TASK27_FIVE_CUBE 1
#define TASK01_CUBE04_PRECISION_INSERT 1
#define TASK01_DUAL_SUCTION_FIXTURE 1
#define main task01_unexecuted_controller_main
#include "task26_truck_box_push_in.cpp"
#undef main

namespace
{
Eigen::Isometry3d fixturePoseTransform(const geometry_msgs::msg::Pose& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z).normalized().toRotationMatrix();
  return transform;
}

bool copyProbeKinematics(const rclcpp::Node::SharedPtr& node)
{
  // 探针需要本地 IK；原执行器 computeCartesianPath 是远端 MoveIt 服务。
  // 读取当前运行 MoveIt 的参数，不硬编码插件，也不修改 MoveIt 参数。
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  const auto listed = client->list_parameters({}, 10);
  if (rclcpp::spin_until_future_complete(node, listed, 10s) != rclcpp::FutureReturnCode::SUCCESS)
    return false;
  std::vector<std::string> names;
  for (const auto& name : listed.get().names)
    if (name.find("kinematics_solver") != std::string::npos) names.push_back(name);
  if (names.empty()) return false;
  const auto values = client->get_parameters(names);
  if (rclcpp::spin_until_future_complete(node, values, 10s) != rclcpp::FutureReturnCode::SUCCESS)
    return false;
  for (const auto& value : values.get())
  {
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return false;
    node->declare_parameter(value.get_name(), value.get_parameter_value());
  }
  return true;
}
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task01_dual_fixture_probe");
  if (!copyRobotDescriptions(node) || !copyProbeKinematics(node))
    { rclcpp::shutdown(); return 1; }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] { executor.spin(); });
  bool passed = true;
  try
  {
    moveit::planning_interface::MoveGroupInterface lg(node, "left_arm"), rg(node, "right_arm");
    const auto model = lg.getRobotModel();
    const auto* lj = model->getJointModelGroup("left_arm");
    const auto* rj = model->getJointModelGroup("right_arm");
    const std::string le = "left_fr3_side_suction_tcp", re = "right_fr3_side_suction_tcp";
    lg.setEndEffectorLink(le); rg.setEndEffectorLink(re);
    lg.setPoseReferenceFrame("world"); rg.setPoseReferenceFrame("world");
    g_world_shift_x = .100;  // 原推入阶段的双轨站位，不移动任何物理基座。
    for (std::size_t index = 0; index < 3; ++index)
    {
      const auto& task = kTasks.at(index);
      const bool side_left = index != 1;
      const double direction = side_left ? 1.0 : -1.0;
      const auto initial = task.pre_push;
      auto rear_pose = pushPose(initial.position.x - kPushCupOffsetX,
        initial.position.y, initial.position.z);
      auto side_pose = sidePose(initial.position.x,
        initial.position.y - direction * (kCubeHalf + kSideContactCommandGap),
        initial.position.z, side_left);
      auto world = boxWallObjects(); world.push_back(tableObject());
      for (std::size_t previous = 0; previous < index; ++previous)
        world.push_back(cubeObject("task26_cube_" + std::to_string(previous + 1), kTasks[previous].cell));
      auto contact_world = world;
      contact_world.push_back(cubeObject("task26_cube_" + std::to_string(index + 1), initial));
      planning_scene::PlanningScene collision_scene(model);
      for (const auto& object : contact_world) collision_scene.processCollisionObjectMsg(object);
      bool found = false;
      for (int attempt = 0; attempt < 40 && rclcpp::ok(); ++attempt)
      {
        moveit::core::RobotState state(model);
        state.setToDefaultValues();
        // 显式不同 seed（不修改机器人实测状态）；不能只给两个对置机械臂
        // 同一个负 joint1/home wrist seed，造成 nominal IK 分支覆盖不足。
        std::vector<double> lseed{1.1, .1, .2, -2.3, .1, 2.1, -.8};
        std::vector<double> rseed{-1.1, .1, -.2, -2.3, -.1, 2.1, -.8};
        lseed[0] += .3 * (attempt % 5 - 2);
        rseed[0] -= .3 * (attempt % 5 - 2);
        lseed[2] += .6 * ((attempt / 5) % 4 - 2);
        rseed[2] -= .6 * ((attempt / 5) % 4 - 2);
        lseed[6] += 1.0 * (attempt / 20); rseed[6] += 1.0 * (attempt / 20);
        state.setJointGroupPositions(lj, lseed); state.setJointGroupPositions(rj, rseed);
        state.update();
        if (!state.setFromIK(side_left ? rj : lj, fixturePoseTransform(rear_pose),
              side_left ? re : le, .05) ||
            !state.setFromIK(side_left ? lj : rj, fixturePoseTransform(side_pose),
              side_left ? le : re, .05)) continue;
        state.update();
        if (!state.satisfiesBounds() || collision_scene.isStateColliding(state)) continue;
        std::vector<double> lq, rq;
        state.copyJointGroupPositions(lj, lq); state.copyJointGroupPositions(rj, rq);
        trajectory_msgs::msg::JointTrajectory ls, rs, lx, rx, ly, ry, lo, ro;
        ls.joint_names = lj->getVariableNames(); rs.joint_names = rj->getVariableNames();
        ls = holdTrajectory(ls, lq, .1); rs = holdTrajectory(rs, rq, .1);
        const auto label = std::string(task.id) + " NO_COMMAND";
        if (!validateSync(node, model, contact_world, ls, rs, label + " CONTACT_WITH_CUBE") ||
            !planDualFixtureTranslation(node, lg, rg, ls, rs,
              task.cell.position.x - initial.position.x, 0.0, world, label + " X", &lx, &rx) ||
            !planDualFixtureTranslation(node, lg, rg, lx, rx, 0.0,
              dualFixturePressY(task, initial.position.y, direction) - initial.position.y,
              world, label + " Y", &ly, &ry) ||
            !planDualFixtureReleaseExit(node, model, ly, ry, side_left, world,
              label + " RELEASE_EXIT", &lo, &ro)) continue;
        RCLCPP_INFO(node->get_logger(),
          "%s NO_COMMAND_CONTACT_X_Y PASS: seed_attempt=%d; current Cube included at contact, intentional moving Cube excluded during wall push. NOT_EXECUTED.",
          task.id, attempt + 1);
        found = true; break;
      }
      if (!found)
      {
        RCLCPP_ERROR(node->get_logger(), "%s NO_COMMAND_CONTACT_X_Y FAIL within 40 seeds; not an infeasibility proof.", task.id);
        passed = false;
      }
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "NO_COMMAND probe exception: %s", error.what());
    passed = false;
  }
  RCLCPP_INFO(node->get_logger(), "NO_COMMAND three nominal contact chains %s; no physical benchmark PASS implied.", passed ? "PASS" : "FAIL");
  executor.cancel(); spin.join(); rclcpp::shutdown();
  return passed ? 0 : 1;
}
