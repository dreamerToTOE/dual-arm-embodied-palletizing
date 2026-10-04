// [ENGINEERING] 默认只读 FK / 退路单测，直接调用运行节点的实现；不发布机器人命令。
// 可选 Cube02 RRT 重放会临时同步并恢复少量命名 Planning Scene 对象，不能称为场景只读。
#define TASK27_FIVE_CUBE 1
#define TASK01_CUBE04_PRECISION_INSERT 1
#define main task01_unexecuted_controller_main
#include "task26_truck_box_push_in.cpp"
#undef main

namespace
{
// [EXPERIMENTAL] 四位小数日志 seed 的零机器人命令重放，不代表精确物理重放或五件验收。
// 仅应在没有执行器正在运行时启用；保留桌面、三面墙、已完成 Cube01 和当前 Cube02。
bool runFailedCube02RrtReplay(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& left_group)
{
  moveit::planning_interface::MoveGroupInterface right_group(node, "right_arm");
  const auto model = left_group.getRobotModel();
  const auto* left = model->getJointModelGroup("left_arm");
  const auto* right = model->getJointModelGroup("right_arm");
  if (!left || !right)
  {
    RCLCPP_ERROR(node->get_logger(), "CUBE02_RRT_REPLAY invalid groups.");
    return false;
  }
  // [EXPERIMENTAL] Cube01 推入后双滑轨仍前移 100 mm，Cube02 空载退路才会复位。
  // FK/MoveIt 坐标以基座静止位为参考，物理 Cube GT 必须走同一个已有世界偏移。
  struct WorldShiftGuard
  {
    double previous{g_world_shift_x};
    WorldShiftGuard() { g_world_shift_x = .100; }
    ~WorldShiftGuard() { g_world_shift_x = previous; }
  } shift_guard;
  const std::string left_eef = "left_fr3_side_suction_tcp";
  const std::string right_eef = "right_fr3_side_suction_tcp";
  moveit::core::RobotState replay(model);
  replay.setToDefaultValues();
  replay.setJointGroupPositions(left,
    std::vector<double>{-1.9806, .6149, -2.7546, -2.8750, .2901, 2.2732, 1.2913});
  replay.setJointGroupPositions(right,
    std::vector<double>{-1.0633, -.1968, -.4661, -2.5267, -.1247, 2.3452, -1.4328});
  replay.update();
  const double exit_z = kBottomZ + kReleaseGapZ + kLiftHeight;
  const auto left_target = sidePose(
    kPrePushX, kRowYMinus - kCubeHalf - kSideContactCommandGap, exit_z, true);
  // 与执行器 push_left=true 的原 helper_park 完全同一目标，不移动退出目标。
  const auto right_target = sidePose(kPrePushX - .10, .32, exit_z, false);
  auto world = boxWallObjects();
  world.push_back(tableObject());
  world.push_back(cubeObject("task26_cube_1", worldPose(1.09933, .243, .260)));
  world.push_back(cubeObject("task26_cube_2", worldPose(.7679252, -.06004855, .26000002)));
  const auto& left_start_tcp = replay.getGlobalLinkTransform(left_eef).translation();
  const auto& right_start_tcp = replay.getGlobalLinkTransform(right_eef).translation();
  const auto& model_cube = world.back().primitive_poses.front().position;
  RCLCPP_INFO(node->get_logger(),
    "CUBE02_RRT_REPLAY rail_shift=%.3f m; model Cube=(%.6f,%.6f,%.6f), left TCP=(%.6f,%.6f,%.6f), right TCP=(%.6f,%.6f,%.6f); goals x=(%.3f,%.3f).",
    g_world_shift_x, model_cube.x, model_cube.y, model_cube.z,
    left_start_tcp.x(), left_start_tcp.y(), left_start_tcp.z(),
    right_start_tcp.x(), right_start_tcp.y(), right_start_tcp.z(),
    left_target.position.x, right_target.position.x);

  trajectory_msgs::msg::JointTrajectory left_path, right_path;
  // [ENGINEERING] CLOSED 状态必须在任何规划/场景写入之前拒绝。
  const bool closed_rejected = !planReleasedEmptyRrt(node, left_group, right_group, replay,
    true, false, left_eef, right_eef, left_target, right_target, world,
    "CUBE02_RRT_REPLAY reject LEFT CLOSED", &left_path, &right_path) &&
    !planReleasedEmptyRrt(node, left_group, right_group, replay,
    false, true, left_eef, right_eef, left_target, right_target, world,
    "CUBE02_RRT_REPLAY reject RIGHT CLOSED", &left_path, &right_path) &&
    left_path.points.empty() && right_path.points.empty();
  auto obstruction_pose = worldPose(0, 0, 0);
  const auto& tcp = replay.getGlobalLinkTransform(left_eef).translation();
  // cubeObject 本身会减去轨道偏移，先转回物理 GT 的坐标，避免二次减偏移。
  obstruction_pose.position.x = tcp.x() + g_world_shift_x;
  obstruction_pose.position.y = tcp.y();
  obstruction_pose.position.z = tcp.z();
  auto obstructed_world = world;
  obstructed_world.push_back(cubeObject("task01_empty_rrt_start_obstruction", obstruction_pose));
  // 起点显式障碍物只送入同一个本地 FCL，必须在发送 RRT 请求之前拒绝。
  const bool obstruction_rejected = !planReleasedEmptyRrt(node, left_group, right_group, replay,
    false, false, left_eef, right_eef, left_target, right_target, obstructed_world,
    "CUBE02_RRT_REPLAY reject obstructed measured start", &left_path, &right_path) &&
    left_path.points.empty() && right_path.points.empty();
  if (!closed_rejected || !obstruction_rejected)
  {
    RCLCPP_ERROR(node->get_logger(), "CUBE02_RRT_REPLAY safety-negative tests FAIL.");
    return false;
  }

  moveit::planning_interface::PlanningSceneInterface scene;
  std::vector<std::string> ids;
  for (const auto& object : world)
  {
    ids.push_back(object.id);
  }
  if (!scene.getAttachedObjects(ids).empty())
  {
    RCLCPP_ERROR(node->get_logger(), "CUBE02_RRT_REPLAY named object is attached; refusing scene mutation.");
    return false;
  }
  const auto original = scene.getObjects(ids);
  RCLCPP_WARN(node->get_logger(),
    "CUBE02_RRT_REPLAY no robot command: temporarily sync/restore %zu named Planning Scene objects; not scene-read-only.",
    ids.size());
  bool safe = false;
  try
  {
    if (scene.applyCollisionObjects(world) && scene.getObjects(ids).size() == ids.size())
    {
      safe = planReleasedEmptyRrt(node, left_group, right_group, replay,
        false, false, left_eef, right_eef, left_target, right_target, world,
        "CUBE02_RRT_REPLAY rounded failure seed", &left_path, &right_path);
    }
  }
  catch (const std::exception& error)
  {
    RCLCPP_ERROR(node->get_logger(), "CUBE02_RRT_REPLAY planning exception: %s", error.what());
  }

  // [ENGINEERING] 不用异步 removeCollisionObjects。对每个被触碰 ID 同步恢复原值；
  // 原本不存在的 ID 用同步 REMOVE 删除，绝不清理无关对象或修改 ACM。
  moveit_msgs::msg::PlanningScene restore;
  restore.is_diff = true;
  restore.robot_state.is_diff = true;
  for (const auto& id : ids)
  {
    const auto found = original.find(id);
    if (found != original.end())
    {
      auto object = found->second;
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      restore.world.collision_objects.push_back(std::move(object));
    }
    else
    {
      moveit_msgs::msg::CollisionObject remove;
      remove.id = id;
      remove.header.frame_id = "world";
      remove.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      restore.world.collision_objects.push_back(std::move(remove));
    }
  }
  const bool restore_applied = scene.applyPlanningScene(restore);
  const auto after = scene.getObjects(ids);
  bool restored = restore_applied && after.size() == original.size();
  RCLCPP_INFO(node->get_logger(),
    "CUBE02_RRT_REPLAY restore diagnostics: apply=%s touched_ids=%zu before_count=%zu after_count=%zu.",
    restore_applied ? "true" : "false", ids.size(), original.size(), after.size());
  for (const auto& id : ids)
  {
    RCLCPP_INFO(node->get_logger(), "CUBE02_RRT_REPLAY restore id=%s before_present=%s after_present=%s.",
      id.c_str(), original.count(id) ? "true" : "false", after.count(id) ? "true" : "false");
  }
  const auto dump_pose = [&](const std::string& name, const geometry_msgs::msg::Pose& pose)
  {
    RCLCPP_INFO(node->get_logger(),
      "CUBE02_RRT_REPLAY restore %s p=(%.17g,%.17g,%.17g) q=(%.17g,%.17g,%.17g,%.17g).",
      name.c_str(), pose.position.x, pose.position.y, pose.position.z,
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  };
  const auto dump_object = [&](const std::string& name, const moveit_msgs::msg::CollisionObject& object)
  {
    RCLCPP_INFO(node->get_logger(),
      "CUBE02_RRT_REPLAY restore %s frame=%s primitives=%zu primitive_poses=%zu meshes=%zu planes=%zu subframes=%zu.",
      name.c_str(), object.header.frame_id.c_str(), object.primitives.size(), object.primitive_poses.size(),
      object.meshes.size(), object.planes.size(), object.subframe_names.size());
    dump_pose(name + " object_pose", object.pose);
    for (std::size_t index = 0; index < object.primitives.size(); ++index)
    {
      const auto& primitive = object.primitives[index];
      std::ostringstream dimensions;
      dimensions << std::setprecision(17);
      for (const auto dimension : primitive.dimensions)
      {
        dimensions << dimension << ',';
      }
      RCLCPP_INFO(node->get_logger(), "CUBE02_RRT_REPLAY restore %s primitive[%zu] type=%u dimensions=[%s].",
        name.c_str(), index, static_cast<unsigned int>(primitive.type), dimensions.str().c_str());
    }
    for (std::size_t index = 0; index < object.primitive_poses.size(); ++index)
    {
      dump_pose(name + " primitive_pose[" + std::to_string(index) + "]", object.primitive_poses[index]);
    }
  };
  for (const auto& [id, before] : original)
  {
    const auto found = after.find(id);
    if (found == after.end())
    {
      RCLCPP_ERROR(node->get_logger(), "CUBE02_RRT_REPLAY restore missing original id=%s.", id.c_str());
      dump_object(id + " BEFORE", before);
      restored = false;
      continue;
    }
    const auto& object = found->second;
    // getter 的同一表示逐字段验证，包含几何、姿态和子坐标系；不比较时间戳。
    const bool frame_equal = before.header.frame_id == object.header.frame_id;
    const bool pose_equal = before.pose == object.pose;
    const bool primitives_equal = before.primitives == object.primitives;
    const bool primitive_poses_equal = before.primitive_poses == object.primitive_poses;
    const bool meshes_equal = before.meshes == object.meshes;
    const bool mesh_poses_equal = before.mesh_poses == object.mesh_poses;
    const bool planes_equal = before.planes == object.planes;
    const bool plane_poses_equal = before.plane_poses == object.plane_poses;
    const bool subframe_names_equal = before.subframe_names == object.subframe_names;
    const bool subframe_poses_equal = before.subframe_poses == object.subframe_poses;
    const bool geometry_equal = frame_equal && pose_equal && primitives_equal && primitive_poses_equal &&
      meshes_equal && mesh_poses_equal && planes_equal && plane_poses_equal &&
      subframe_names_equal && subframe_poses_equal;
    RCLCPP_INFO(node->get_logger(),
      "CUBE02_RRT_REPLAY restore equality id=%s frame=%d pose=%d primitives=%d primitive_poses=%d meshes=%d mesh_poses=%d planes=%d plane_poses=%d subframe_names=%d subframe_poses=%d.",
      id.c_str(), frame_equal, pose_equal, primitives_equal, primitive_poses_equal, meshes_equal,
      mesh_poses_equal, planes_equal, plane_poses_equal, subframe_names_equal, subframe_poses_equal);
    if (!geometry_equal)
    {
      dump_object(id + " BEFORE", before);
      dump_object(id + " AFTER", object);
    }
    restored = restored && geometry_equal;
  }
  RCLCPP_INFO(node->get_logger(),
    "CUBE02_RRT_REPLAY %s; CLOSED_rejected=%s obstructed_start_rejected=%s scene_restored=%s; no joint/suction command.",
    safe && restored ? "PASS" : "FAIL", closed_rejected ? "true" : "false",
    obstruction_rejected ? "true" : "false", restored ? "true" : "false");
  return safe && restored;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("task01_empty_retreat_probe");
  const bool run_cube02_replay = node->declare_parameter<bool>("run_failed_cube02_rrt_replay", false);
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
    if (run_cube02_replay)
    {
      const bool cube02_replay_safe = runFailedCube02RrtReplay(node, group);
      passed = passed && cube02_replay_safe;
    }
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
