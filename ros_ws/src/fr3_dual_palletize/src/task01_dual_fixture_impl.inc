// [ADAPTATION]/[ENGINEERING] 仅新独立目标包含；复用同一规划/同步/FCL实现。
namespace
{
struct DualFixtureMotion
{
  trajectory_msgs::msg::JointTrajectory left_end, right_end, left_exit, right_exit;
};

bool fixtureRelativePathSafe(const rclcpp::Node::SharedPtr& node,
  const moveit::core::RobotModelConstPtr& model,
  const std::string& left_eef, const std::string& right_eef,
  const trajectory_msgs::msg::JointTrajectory& lp,
  const trajectory_msgs::msg::JointTrajectory& rp, const std::string& label)
{
  if (lp.points.empty() || rp.points.empty()) return false;
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  const auto relative = [&](double time)
  {
    state.setVariablePositions(lp.joint_names, interpolate(lp, time));
    state.setVariablePositions(rp.joint_names, interpolate(rp, time));
    state.update();
    return Eigen::Isometry3d(state.getGlobalLinkTransform(left_eef).inverse() *
      state.getGlobalLinkTransform(right_eef));
  };
  const Eigen::Isometry3d reference = relative(0.0);
  const double duration = pointTime(lp.points.back());
  if (!std::isfinite(duration) || duration < 0.0 ||
      std::abs(duration - pointTime(rp.points.back())) > 1e-6 ||
      !reference.matrix().allFinite()) return false;
  double max_position = 0.0, max_angle = 0.0;
  for (double time = 0.0; ; time = std::min(duration, time + kFclSamplePeriod))
  {
    const auto actual = relative(time);
    if (!actual.matrix().allFinite()) return false;
    max_position = std::max(max_position,
      (actual.translation() - reference.translation()).cwiseAbs().maxCoeff());
    max_angle = std::max(max_angle,
      Eigen::AngleAxisd(reference.rotation().transpose() * actual.rotation()).angle() * 180.0 / kPi);
    if (!std::isfinite(max_position) || !std::isfinite(max_angle) ||
        max_position > kAttachmentAlignmentTolerance || max_angle > kAttachmentOrientationToleranceDeg)
    {
      RCLCPP_ERROR(node->get_logger(), "%s relative TCP rejected: %.3f mm / %.3f deg at %.3f s.",
        label.c_str(), max_position * 1000.0, max_angle, time);
      return false;
    }
    if (time >= duration) break;
  }
  RCLCPP_INFO(node->get_logger(), "%s relative TCP PASS: max_axis=%.3f mm angle=%.3f deg.",
    label.c_str(), max_position * 1000.0, max_angle);
  return true;
}

// [ENGINEERING] 无命令探针与执行器共用同一规划/FCL/相对 TCP 校验链。
bool planDualFixtureTranslation(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  const trajectory_msgs::msg::JointTrajectory& lstart,
  const trajectory_msgs::msg::JointTrajectory& rstart, double dx, double dy,
  const std::vector<moveit_msgs::msg::CollisionObject>& world,
  const std::string& label, trajectory_msgs::msg::JointTrajectory* l,
  trajectory_msgs::msg::JointTrajectory* r)
{
  const auto model = lg.getRobotModel();
  const std::string le = "left_fr3_side_suction_tcp", re = "right_fr3_side_suction_tcp";
  (void)rg;
  const auto* lj = model->getJointModelGroup("left_arm");
  const auto* rj = model->getJointModelGroup("right_arm");
  if (!l || !r || !lj || !rj || !std::isfinite(dx) || !std::isfinite(dy) ||
      std::hypot(dx, dy) > .350 || (std::abs(dx) > 1e-9 && std::abs(dy) > 1e-9)) return false;
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setJointGroupPositions(lj, finalPositions(lstart));
  state.setJointGroupPositions(rj, finalPositions(rstart));
  state.update();
  if (!state.satisfiesBounds()) return false;
  auto lgoal = linkPose(model, le, lstart), rgoal = linkPose(model, re, rstart);
  const auto segments = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(std::hypot(dx, dy) / .002)));
  trajectory_msgs::msg::JointTrajectory lp, rp;
  lp.joint_names = lj->getVariableNames(); rp.joint_names = rj->getVariableNames();
  trajectory_msgs::msg::JointTrajectoryPoint first_l, first_r;
  state.copyJointGroupPositions(lj, first_l.positions); state.copyJointGroupPositions(rj, first_r.positions);
  lp.points.push_back(first_l); rp.points.push_back(first_r);
  for (std::size_t index = 1; index <= segments; ++index)
  {
    const double phase = static_cast<double>(index) / segments;
    auto lt = lgoal, rt = rgoal;
    lt.position.x += phase * dx; rt.position.x += phase * dx;
    lt.position.y += phase * dy; rt.position.y += phase * dy;
    trajectory_msgs::msg::JointTrajectory micro_l, micro_r;
    // [ENGINEERING] 复用已验证的世界 FK 数值 Jacobian 内核（每段 <=2 mm），
    // 将其扩展为已批准新接触协议的共同位移网格，不改变旧节点负载路径。
    // 每段保留近 seed 构型、bounds/5um 解算门限；整条路径另做联合 FCL。
    if (!planFinePreclose(node, state, lj, le, lt, label + " paired left " + std::to_string(index), &micro_l, false)) return false;
    state.setJointGroupPositions(lj, finalPositions(micro_l)); state.update();
    if (!planFinePreclose(node, state, rj, re, rt, label + " paired right " + std::to_string(index), &micro_r, false)) return false;
    state.setJointGroupPositions(rj, finalPositions(micro_r)); state.update();
    trajectory_msgs::msg::JointTrajectoryPoint point_l, point_r;
    state.copyJointGroupPositions(lj, point_l.positions); state.copyJointGroupPositions(rj, point_r.positions);
    lp.points.push_back(point_l); rp.points.push_back(point_r);
  }
  ensureTiming(lp); ensureTiming(rp);
  if (!validateSync(node, model, world, lp, rp, label) ||
      !fixtureRelativePathSafe(node, model, le, re, lp, rp, label)) return false;
  RCLCPP_INFO(node->get_logger(), "%s paired common-progress PASS: points=%zu, delta=(%.3f,%.3f) m.",
    label.c_str(), lp.points.size(), dx, dy);
  *l = std::move(lp); *r = std::move(rp);
  return true;
}

bool planDualFixtureReleaseExit(const rclcpp::Node::SharedPtr& node,
  const moveit::core::RobotModelConstPtr& model,
  const trajectory_msgs::msg::JointTrajectory& ls,
  const trajectory_msgs::msg::JointTrajectory& rs, bool side_left,
  const std::vector<moveit_msgs::msg::CollisionObject>& world,
  const std::string& label, trajectory_msgs::msg::JointTrajectory* l,
  trajectory_msgs::msg::JointTrajectory* r)
{
  const std::string le = "left_fr3_side_suction_tcp", re = "right_fr3_side_suction_tcp";
  auto lt = linkPose(model, le, ls), rt = linkPose(model, re, rs);
  (side_left ? rt : lt).position.x -= .030;
  (side_left ? lt : rt).position.y += side_left ? -.030 : .030;
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariablePositions(ls.joint_names, finalPositions(ls));
  state.setVariablePositions(rs.joint_names, finalPositions(rs)); state.update();
  // [ENGINEERING] 原短清障策略的30mm局部包络；不要求第三件helper穿过整个中央槽
  // 退出120mm。两杯OPEN之后才执行；之后由原RRT/FCL转场或HOME接管。
  return planSeededEmptyCartesian(node, state, model->getJointModelGroup("left_arm"), le,
      lt, label + " left", l) &&
    planSeededEmptyCartesian(node, state, model->getJointModelGroup("right_arm"), re,
      rt, label + " right", r) && synchronize(l, r) && validateSync(node, model, world, *l, *r, label);
}

double dualFixturePressY(const OfflineTask& task, double actual_y, double direction)
{
  // [ADAPTATION] 双杯闭环持件时以既有贴墙格位为终点，不沿用旧未吸附侧压的
  // 4 mm 穿墙命令；场景、格位、验收门限和旧节点的参数/行为全部保持不变。
  return task.cube_index < 2 ? task.cell.position.y : actual_y + direction * kInnerLateralTrim;
}

bool runDualFixture(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  moveit::planning_interface::PlanningSceneInterface& scene,
  const Arm& left, const Arm& right, const Arm& rear, const Arm& side,
  const CubeBuffer& cubes, const TcpBuffer& lt, const TcpBuffer& rt,
  const ForceBuffer& lf, const ForceBuffer& rf, const OfflineTask& task,
  DualFixtureMotion* output)
{
  if (!output || !fr3_dual_palletize::dualFixtureRequired(task.cube_index) ||
      !rear.isClosed() || side.isClosed()) return false;
  const auto model = lg.getRobotModel();
  const bool side_left = side.isLeft();
  const double direction = side_left ? 1.0 : -1.0;
  auto& sg = side_left ? lg : rg;
  const std::string id = "task26_cube_" + std::to_string(task.cube_index + 1);
  const auto seed = [&](trajectory_msgs::msg::JointTrajectory* l,
                        trajectory_msgs::msg::JointTrajectory* r)
  {
    const auto current = lg.getCurrentState(2.0);
    const auto* lj = model->getJointModelGroup(left.groupName());
    const auto* rj = model->getJointModelGroup(right.groupName());
    if (!current || !lj || !rj) return false;
    std::vector<double> lq, rq;
    current->copyJointGroupPositions(lj, lq);
    current->copyJointGroupPositions(rj, rq);
    if (!fr3_dual_palletize::validMeasuredJointSeed(lq, lj->getVariableCount()) ||
        !fr3_dual_palletize::validMeasuredJointSeed(rq, rj->getVariableCount()) ||
        !current->satisfiesBounds(lj) || !current->satisfiesBounds(rj)) return false;
    l->joint_names = lj->getVariableNames(); r->joint_names = rj->getVariableNames();
    *l = holdTrajectory(*l, lq, .1); *r = holdTrajectory(*r, rq, .1);
    return true;
  };
  const auto stage = [&](const trajectory_msgs::msg::JointTrajectory& lstart,
    const trajectory_msgs::msg::JointTrajectory& rstart, double dx, double dy,
    const std::vector<moveit_msgs::msg::CollisionObject>& world,
    const std::string& label, trajectory_msgs::msg::JointTrajectory* l,
    trajectory_msgs::msg::JointTrajectory* r)
  {
    // 不重新强迫名义 1 mm 点：保持实际已建立的两个 grasp 相对变换，同一物体平移。
    // linkPose 已是模型系；planCommonCartesian 不会再次减去 rail shift。
    return planDualFixtureTranslation(node, lg, rg, lstart, rstart, dx, dy, world, label, l, r);
  };
  const auto geometry = [&](bool require_closed, const std::string& label)
  {
    const auto [cube, revision] = cubes.get(task.cube_index); (void)revision;
    const auto rear_tcp = rear.isLeft() ? lt.get() : rt.get();
    const auto side_tcp = side_left ? lt.get() : rt.get();
    const double rear_gap = cube.position.x - rear_tcp.position.x - kCubeHalf;
    const double side_gap = direction * (cube.position.y - side_tcp.position.y) - kCubeHalf;
    const double alignment = std::max({std::abs(rear_tcp.position.y - cube.position.y),
      std::abs(rear_tcp.position.z - cube.position.z),
      std::abs(side_tcp.position.x - cube.position.x),
      std::abs(side_tcp.position.z - cube.position.z)});
    const double tilt = orientationAngleDeg(cube.orientation);
    const bool closed = fr3_dual_palletize::validDualFixtureContactState(
      task.cube_index, rear.isClosed(), side.isClosed());
    RCLCPP_INFO(node->get_logger(),
      "%s %s rear=%s side=%s rear_gap=%.3f mm side_gap=%.3f mm alignment=%.3f mm tilt=%.3f deg.",
      task.id, label.c_str(), rear.isClosed() ? "CLOSED" : "OPEN", side.isClosed() ? "CLOSED" : "OPEN",
      rear_gap * 1000.0, side_gap * 1000.0, alignment * 1000.0, tilt);
    return std::isfinite(alignment) && std::isfinite(rear_gap) && std::isfinite(side_gap) &&
      std::isfinite(tilt) && rear_gap >= kMinAttachmentGap && side_gap >= kMinAttachmentGap &&
      rear_gap <= kMaxAttachmentGap && side_gap <= kMaxAttachmentGap &&
      alignment <= kAttachmentAlignmentTolerance && tilt <= kAttachmentOrientationToleranceDeg &&
      (require_closed ? closed : (rear.isClosed() && !side.isClosed()));
  };
  trajectory_msgs::msg::JointTrajectory ls, rs;
  if (!seed(&ls, &rs)) return false;
  const auto [initial, initial_revision] = cubes.get(task.cube_index);
  if (!scene.applyCollisionObject(cubeObject(id, initial))) return false;
  const auto world_contact = staticWorld(scene);
  const double contact_y = initial.position.y - direction * (kCubeHalf + kSideContactCommandGap);
  const auto high = sidePose(initial.position.x, contact_y,
    initial.position.z + kRegraspLiftHeight, side_left);
  const auto contact = sidePose(initial.position.x, contact_y, initial.position.z, side_left);
  std::vector<trajectory_msgs::msg::JointTrajectory> candidates;
  if (!side.planPoseCandidatesFrom(sg, side_left ? finalPositions(ls) : finalPositions(rs),
        high, std::string(task.id) + " DUAL_SIDE_HIGH", &candidates)) return false;
  trajectory_msgs::msg::JointTrajectory la, ra, lc, rc;
  bool found = false;
  for (const auto& candidate : candidates)
  {
    la = side_left ? candidate : holdTrajectory(ls, finalPositions(ls), pointTime(candidate.points.back()));
    ra = side_left ? holdTrajectory(rs, finalPositions(rs), pointTime(candidate.points.back())) : candidate;
    if (!validateSync(node, model, world_contact, la, ra, std::string(task.id) + " DUAL_SIDE_HIGH")) continue;
    trajectory_msgs::msg::JointTrajectory approach;
    if (!planCommonCartesian(node, sg, side.groupName(), rear.groupName(),
          side_left ? finalPositions(la) : finalPositions(ra),
          side_left ? finalPositions(ra) : finalPositions(la), contact,
          std::string(task.id) + " DUAL_SIDE_CONTACT", &approach, side.eefLink(), true)) continue;
    lc = side_left ? approach : holdTrajectory(la, finalPositions(la), pointTime(approach.points.back()));
    rc = side_left ? holdTrajectory(ra, finalPositions(ra), pointTime(approach.points.back())) : approach;
    if (!validateSync(node, model, world_contact, lc, rc, std::string(task.id) + " DUAL_SIDE_CONTACT")) continue;
    // 新流程不借用旧 helper-park / mono-trim 的预检结论。先验双吸推入与换角色横移。
    if (!scene.applyPlanningScene([&] {
      moveit_msgs::msg::PlanningScene remove; remove.is_diff = true; remove.robot_state.is_diff = true;
      moveit_msgs::msg::CollisionObject object; object.id = id;
      object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      remove.world.collision_objects.push_back(object); return remove; }())) return false;
    const auto world_push = staticWorld(scene);
    trajectory_msgs::msg::JointTrajectory lx, rx, ly, ry;
    const double press_y = dualFixturePressY(task, initial.position.y, direction);
    const bool preview = stage(lc, rc, task.cell.position.x - initial.position.x, 0,
        world_push, std::string(task.id) + " DUAL_PREFLIGHT_X", &lx, &rx) &&
      stage(lx, rx, 0, press_y - initial.position.y, world_push,
        std::string(task.id) + " DUAL_PREFLIGHT_Y", &ly, &ry) &&
      planDualFixtureReleaseExit(node, model, ly, ry, side_left, world_push,
        std::string(task.id) + " DUAL_PREFLIGHT_RELEASE_EXIT", &output->left_exit, &output->right_exit);
    if (!scene.applyCollisionObject(cubeObject(id, initial))) return false;
    if (preview) { found = true; break; }
  }
  const auto approaching_guard = [&] {
    return rclcpp::ok() && rear.isClosed() && !side.isClosed() &&
      std::max(lf.peak(), rf.peak()) <= kPushTorqueLimit;
  };
  if (!found || !executeSync(left, la, right, ra, approaching_guard) ||
      !executeSync(left, lc, right, rc, approaching_guard)) return false;
  std::this_thread::sleep_for(250ms);
  if (!geometry(false, "DUAL_PRE_CLOSE")) return false;
  side.suction(true);
  if (!side.waitSuction(true, 3.0)) return false;
  std::this_thread::sleep_for(250ms);
  if (!geometry(true, "DUAL_CONTACT_READY") || !seed(&ls, &rs)) return false;
  geometry_msgs::msg::Pose start_cube;
  if (!cubes.waitNew(task.cube_index, initial_revision, 2.0, &start_cube)) return false;
  // 与旧接触阶段相同，仅当前被推 Cube 不作静态障碍；所有墙、桌、已放 Cube 保留。
  moveit_msgs::msg::PlanningScene remove; remove.is_diff = true; remove.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject object; object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  remove.world.collision_objects.push_back(object);
  if (!scene.applyPlanningScene(remove)) return false;
  const auto world_push = staticWorld(scene);
  trajectory_msgs::msg::JointTrajectory lx, rx, ly, ry;
  const double press_y = dualFixturePressY(task, start_cube.position.y, direction);
  if (!stage(ls, rs, task.cell.position.x - start_cube.position.x, 0,
        world_push, std::string(task.id) + " DUAL_REAR_PRIMARY_X", &lx, &rx) ||
      !stage(lx, rx, 0, press_y - start_cube.position.y, world_push,
        std::string(task.id) + " DUAL_SIDE_PRIMARY_Y", &ly, &ry) ||
      !planDualFixtureReleaseExit(node, model, ly, ry, side_left, world_push,
        std::string(task.id) + " DUAL_RELEASE_EXIT", &output->left_exit, &output->right_exit)) return false;
  const auto run = [&](const trajectory_msgs::msg::JointTrajectory& l,
    const trajectory_msgs::msg::JointTrajectory& r, const std::string& label, bool pushing_x)
  {
    const double duration = pointTime(l.points.back());
    for (int slice = 1; slice <= kPushSlices; ++slice)
    {
      trajectory_msgs::msg::JointTrajectory lpart, rpart;
      if (!sliceTrajectory(l, duration * (slice-1) / kPushSlices, duration * slice / kPushSlices, &lpart) ||
          !sliceTrajectory(r, duration * (slice-1) / kPushSlices, duration * slice / kPushSlices, &rpart) ||
          !executeSync(left, lpart, right, rpart, [&] {
            return rclcpp::ok() && rear.isClosed() && side.isClosed() &&
              std::max(lf.peak(), rf.peak()) <= kPushTorqueLimit;
          }) || !geometry(true, label + " slice " + std::to_string(slice))) return false;
      const auto [actual, revision] = cubes.get(task.cube_index); (void)revision;
      if (pushing_x)
      {
        const auto& part = rear.isLeft() ? lpart : rpart;
        const auto& initial_path = rear.isLeft() ? ls : rs;
        const double planned_x = start_cube.position.x +
          linkPosition(model, rear.eefLink(), part.joint_names, finalPositions(part)).x() -
          linkPosition(model, rear.eefLink(), initial_path.joint_names, finalPositions(initial_path)).x();
        if (!std::isfinite(planned_x) || std::abs(actual.position.x-planned_x) > kPushJamTolerance) return false;
      }
      else if (!validDeepWallSeat(node, task, actual)) return false;
    }
    return true;
  };
  RCLCPP_INFO(node->get_logger(), "%s DUAL_REAR_PRIMARY_X: rear=%s side=%s; both suction CLOSED.",
    task.id, rear.isLeft() ? "left" : "right", side_left ? "left" : "right");
  if (!run(lx, rx, "DUAL_REAR_PRIMARY_X", true)) return false;
  const auto [seated, revision] = cubes.get(task.cube_index); (void)revision;
  if (!validDeepWallSeat(node, task, seated)) return false;
  RCLCPP_INFO(node->get_logger(), "%s DUAL_ROLE_SWAP: side PRIMARY, rear deep-wall HOLD; no suction release.", task.id);
  if (!run(ly, ry, "DUAL_SIDE_PRIMARY_Y", false)) return false;
  const auto [pressed, pressed_revision] = cubes.get(task.cube_index); (void)pressed_revision;
  if (!validCellPlacement(node, task, pressed, cubes)) return false;
  output->left_end = ly; output->right_end = ry;
  return true;  // 清障包络已在负载推入之前预检；真正执行仍等待双OPEN。
}
}  // namespace
