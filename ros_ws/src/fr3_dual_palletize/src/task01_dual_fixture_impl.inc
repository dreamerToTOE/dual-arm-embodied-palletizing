// [ADAPTATION]/[ENGINEERING] 仅新独立目标包含；复用同一规划/同步/FCL实现。
namespace
{
// [ENGINEERING] 单消息内的Cube/双TCP来自一个live PhysX步，不拼三个latest话题。
// 旧Task26/27及精确单rear流程保持原样；新3+2节点缺消息时拒绝启动/继续。
class FixtureGeometryBuffer
{
public:
  FixtureGeometryBuffer(const rclcpp::Node::SharedPtr& node, std::size_t cube_count)
    : count_(cube_count), node_(node)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task01/physics/fixture_geometry", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::int64_t stamp = static_cast<std::int64_t>(message->header.stamp.sec)*1000000000LL +
          message->header.stamp.nanosec;
        incoming_stamp_ = stamp;
        bool valid = message->header.frame_id == "world" && message->poses.size() == count_+2 &&
          fr3_dual_palletize::validFixtureStamp(stamp, stamp_);
        for (const auto& p : message->poses)
          valid = valid && fr3_dual_palletize::validFixturePose({p.position.x, p.position.y,
            p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w});
        ready_ = valid;
        if (valid) {
          poses_ = message->poses; stamp_ = stamp;
          received_ = std::chrono::steady_clock::now();
        }
        condition_.notify_all();
      });
  }
  bool wait(double timeout_sec) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
      [this] { return freshLocked(); });
  }
  bool fresh() const
  { std::lock_guard<std::mutex> lock(mutex_); return freshLocked(); }
  bool playbackStamp(std::int64_t* stamp) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!stamp || !freshLocked()) {
      RCLCPP_ERROR(node_->get_logger(),
        "PhysX playback feedback rejection: ready=%s age_ms=%.3f accepted_stamp_ns=%lld "
        "last_incoming_stamp_ns=%lld; original250ms gate unchanged.",
        ready_ ? "true" : "false", 1000.0*std::chrono::duration<double>(
          std::chrono::steady_clock::now()-received_).count(),
        static_cast<long long>(stamp_), static_cast<long long>(incoming_stamp_));
      return false;
    }
    *stamp = stamp_;
    return true;
  }
  bool get(std::size_t index, geometry_msgs::msg::Pose* cube,
    geometry_msgs::msg::Pose* left, geometry_msgs::msg::Pose* right, std::int64_t* stamp) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!freshLocked() || index >= count_ || !cube || !left || !right || !stamp) return false;
    *cube = poses_.at(index); *left = poses_.at(count_); *right = poses_.at(count_+1);
    *stamp = stamp_; return true;
  }
private:
  bool freshLocked() const
  { return ready_ && fr3_dual_palletize::freshFixtureReceipt(std::chrono::duration<double>(
      std::chrono::steady_clock::now()-received_).count()); }
  std::size_t count_;
  rclcpp::Node::SharedPtr node_;
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  std::int64_t stamp_{0};
  std::int64_t incoming_stamp_{0};
  std::chrono::steady_clock::time_point received_{};
  std::vector<geometry_msgs::msg::Pose> poses_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

geometry_msgs::msg::Pose dualSideFixturePose(double x, double y, double z, bool left,
  double world_y_roll_deg)
{
  if (!fr3_dual_palletize::validSideFixtureRoll(world_y_roll_deg))
    throw std::runtime_error("Invalid cup-normal wrist roll");
  auto pose = sidePose(x, y, z, left);
  if (world_y_roll_deg == 0.0) return pose;  // 原始姿态精确保留为诊断参数。
  const Eigen::Quaterniond original(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  const Eigen::Quaterniond rotated = Eigen::Quaterniond(Eigen::AngleAxisd(
    world_y_roll_deg * kPi / 180.0, Eigen::Vector3d::UnitY())) * original;
  // [ENGINEERING] TCP+X仍为世界+/-Y；杯面中心和法向不变，仅在面内转动阵列。
  // 姿态在OPEN重抓时建立；双杯CLOSED后的X/Y推压不执行任何腕部旋转。
  if ((rotated * Eigen::Vector3d::UnitX() - original * Eigen::Vector3d::UnitX()).norm() > 1e-10)
    throw std::runtime_error("Wrist roll changed cup normal");
  pose.orientation.w = rotated.w(); pose.orientation.x = rotated.x();
  pose.orientation.y = rotated.y(); pose.orientation.z = rotated.z();
  return pose;
}

struct DualFixtureMotion
{
  trajectory_msgs::msg::JointTrajectory left_end, right_end, left_exit, right_exit;
};

bool copyFixtureKinematics(const rclcpp::Node::SharedPtr& node)
{
  // [ENGINEERING] 新节点的本地候选IK读取正在使用的MoveIt插件；旧节点不启用。
  // 在加入工作executor前、有界异步读取；缺配置就拒绝，不改远端参数。
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  const auto listed = client->list_parameters({}, 10);
  if (rclcpp::spin_until_future_complete(node, listed, 10s) != rclcpp::FutureReturnCode::SUCCESS)
    return false;
  std::vector<std::string> names;
  for (const auto& name : listed.get().names)
    if ((name.find("robot_description_kinematics.") == 0 ||
         name.find("left_arm.") == 0 || name.find("right_arm.") == 0) &&
        name.find("kinematics_solver") != std::string::npos) names.push_back(name);
  if (names.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "No current MoveIt arm kinematics parameters found.");
    return false;
  }
  const auto values = client->get_parameters(names);
  if (rclcpp::spin_until_future_complete(node, values, 10s) != rclcpp::FutureReturnCode::SUCCESS)
    return false;
  for (const auto& value : values.get())
  {
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return false;
    if (!node->has_parameter(value.get_name()))
      node->declare_parameter(value.get_name(), value.get_parameter_value());
  }
  return true;
}

// 只读复制本机OMPL配置；仅本节点副本用于OPEN转场，不写/move_group参数。
bool copyFixtureEmptyPlannerConfig(const rclcpp::Node::SharedPtr& node)
{
  auto client=std::make_shared<rclcpp::AsyncParametersClient>(node,"/move_group");
  if (!client->wait_for_service(3s)) return false;
  auto listed=client->list_parameters({"move_group"},10);
  if (rclcpp::spin_until_future_complete(node,listed,10s)!=rclcpp::FutureReturnCode::SUCCESS) return false;
  const auto names=listed.get().names;
  if (names.empty()) return false;
  auto fetched=client->get_parameters(names);
  if (rclcpp::spin_until_future_complete(node,fetched,10s)!=rclcpp::FutureReturnCode::SUCCESS) return false;
  for (const auto& p:fetched.get())
  {
    if (p.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET) return false;
    if (!node->has_parameter(p.get_name())) node->declare_parameter(p.get_name(),p.get_parameter_value());
  }
  return true;
}

// [ENGINEERING] 复用既有XYZ残差累加/1mm限步/FK微解算，只在后杯OPEN时精调。
// RRT允许的毫米级到位残差不能直接带入双D6闭环；不放宽2.5mm原验收门限。
bool refineOpenRearFixture(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  moveit::planning_interface::PlanningSceneInterface& scene,
  const Arm& left, const Arm& right, const Arm& rear,
  const FixtureGeometryBuffer& feedback, const OfflineTask& task,
  trajectory_msgs::msg::JointTrajectory* rear_command)
{
  if (!rear_command || rear_command->points.empty() || left.isClosed() || right.isClosed()) return false;
  const auto model = lg.getRobotModel();
  const auto* own = model->getJointModelGroup(rear.groupName());
  const auto* other = model->getJointModelGroup(rear.isLeft() ? rg.getName() : lg.getName());
  if (!own || !other) return false;
  for (int correction = 0; correction <= kRetries; ++correction)
  {
    std::this_thread::sleep_for(250ms);
    geometry_msgs::msg::Pose cube, lt, rt;
    std::int64_t stamp;
    if (left.isClosed() || right.isClosed() ||
        !feedback.get(task.cube_index, &cube, &lt, &rt, &stamp)) return false;
    const auto& actual = rear.isLeft() ? lt : rt;
    const fr3_dual_palletize::PrecloseVector3 desired{
      cube.position.x-kPushCupOffsetX, cube.position.y, cube.position.z};
    const fr3_dual_palletize::PrecloseVector3 measured{
      actual.position.x, actual.position.y, actual.position.z};
    const double error = std::hypot(std::hypot(desired[0]-measured[0], desired[1]-measured[1]),
                                   desired[2]-measured[2]);
    RCLCPP_INFO(node->get_logger(),
      "%s REAR_OPEN_XYZ check=%d/%d residual_mm=(%+.3f,%+.3f,%+.3f) norm=%.3f mm physics_stamp_ns=%lld.",
      task.id, correction+1, kRetries+1, (desired[0]-measured[0])*1000.,
      (desired[1]-measured[1])*1000., (desired[2]-measured[2])*1000., error*1000.,
      static_cast<long long>(stamp));
    // 0.3mm是吸附前目标精度，不替代原2.5mm加载验收；最多3次1mm修正。
    if (!std::isfinite(error)) return false;
    if (error <= .0003) return true;
    if (correction == kRetries) return false;
    const auto delta = fr3_dual_palletize::precloseAlignmentDeltaXYZ(desired, measured);
    auto target = linkPose(model, rear.eefLink(), *rear_command);
    target.position.x += delta[0]; target.position.y += delta[1]; target.position.z += delta[2];
    auto state = lg.getCurrentState(2.0);
    if (!state) return false;
    state->update();
    trajectory_msgs::msg::JointTrajectory move, hold;
    if (!planFinePreclose(node, *state, own, rear.eefLink(), target,
          std::string(task.id)+" REAR_OPEN_PRE_CLOSE_REACQUIRE_XYZ", &move)) return false;
    std::vector<double> other_q; state->copyJointGroupPositions(other, other_q);
    hold.joint_names=other->getVariableNames();
    hold=holdTrajectory(hold, other_q, pointTime(move.points.back()));
    auto world=staticWorld(scene);
    const auto id="task26_cube_"+std::to_string(task.cube_index+1);
    world.erase(std::remove_if(world.begin(), world.end(), [&](const auto& o) { return o.id==id; }), world.end());
    world.push_back(cubeObject(id, cube));  // 自由Cube仍严格参与FCL，不能删掉规避。
    const auto& lp=rear.isLeft() ? move : hold;
    const auto& rp=rear.isLeft() ? hold : move;
    if (!validateSync(node, model, world, lp, rp,
          std::string(task.id)+" REAR_OPEN_XYZ_CURRENT_CUBE_INCLUDED") ||
        left.isClosed() || right.isClosed() ||
        !executeSync(left, lp, right, rp, [&] {
          return rclcpp::ok() && feedback.fresh() && !left.isClosed() && !right.isClosed();
        })) return false;
    *rear_command=move;
  }
  return false;
}

// [EXPERIMENTAL] 探针与独立3+2执行器共用：后杯面TCP+X始终指世界+X。
// 仅OPEN重抓绕该法向翻腕；+Y件向中央(-Y)让腕，-Y件镜像(+Y)。
geometry_msgs::msg::Pose dualRearFixturePose(double x, double y, double z,
  std::size_t cube_index, double roll_magnitude_deg)
{
  if (!fr3_dual_palletize::validRearFixtureRoll(roll_magnitude_deg) || cube_index >= 3)
    throw std::runtime_error("Invalid rear cup-normal diagnostic roll");
  auto pose = pushPose(x, y, z);
  if (roll_magnitude_deg == 0.0) return pose;
  const Eigen::Quaterniond original(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  const double sign = cube_index == 1 ? -1.0 : 1.0;
  const Eigen::Quaterniond rotated = Eigen::Quaterniond(Eigen::AngleAxisd(
    sign * roll_magnitude_deg * kPi / 180., Eigen::Vector3d::UnitX())) * original;
  if ((rotated * Eigen::Vector3d::UnitX()-original * Eigen::Vector3d::UnitX()).norm() > 1e-10)
    throw std::runtime_error("Rear diagnostic roll changed cup normal");
  pose.orientation.w=rotated.w(); pose.orientation.x=rotated.x();
  pose.orientation.y=rotated.y(); pose.orientation.z=rotated.z();
  return pose;
}

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

// [ENGINEERING] rear候选不能只通过旧solo推入；必须给新rear+side链留下空间。
// 全部为本地假设/无命令预检：HIGH终点、连续下降、同进度X/Y及短退出。
// 不声称HIGH的RRT连接已验证；现场runDualFixture仍从实测状态完整检查再执行。
bool preflightDualFixtureRearCandidate(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  const trajectory_msgs::msg::JointTrajectory& ls,
  const trajectory_msgs::msg::JointTrajectory& rs,
  const geometry_msgs::msg::Pose& cube, const OfflineTask& task,
  const std::vector<moveit_msgs::msg::CollisionObject>& contact_world,
  const std::string& label, trajectory_msgs::msg::JointTrajectory* xleft,
  trajectory_msgs::msg::JointTrajectory* xright,
  trajectory_msgs::msg::JointTrajectory* exit_left,
  trajectory_msgs::msg::JointTrajectory* exit_right)
{
  if (!fr3_dual_palletize::dualFixtureRequired(task.cube_index) || ls.points.empty() ||
      rs.points.empty() || !xleft || !xright || !exit_left || !exit_right) return false;
  const auto model=lg.getRobotModel();
  const auto* lj=model->getJointModelGroup("left_arm");
  const auto* rj=model->getJointModelGroup("right_arm");
  const bool side_left=task.cube_index!=1;
  const auto* side=side_left ? lj:rj;
  const std::string eef=side_left ? "left_fr3_side_suction_tcp":"right_fr3_side_suction_tcp";
  if (!lj || !rj || !model->hasLinkModel(eef) || !side->getSolverInstance()) return false;
  moveit::core::RobotState original(model);
  original.setToDefaultValues();
  original.setVariablePositions(ls.joint_names, finalPositions(ls));
  original.setVariablePositions(rs.joint_names, finalPositions(rs)); original.update();
  if (!original.satisfiesBounds()) return false;
  planning_scene::PlanningScene collision_scene(model);
  for (const auto& object:contact_world)
    if (!collision_scene.processCollisionObjectMsg(object)) return false;
  const auto roll=node->get_parameter("dual_fixture_side_roll_world_y_deg").as_double();
  const double direction=side_left ? 1.:-1.;
  const auto high=dualSideFixturePose(cube.position.x,
    cube.position.y-direction*(kCubeHalf+kSideContactCommandGap),
    cube.position.z+kRegraspLiftHeight,side_left,roll);
  auto contact=high; contact.position.z=cube.position.z;
  Eigen::Isometry3d high_tf=Eigen::Isometry3d::Identity();
  high_tf.translation()=Eigen::Vector3d(high.position.x,high.position.y,high.position.z);
  high_tf.linear()=Eigen::Quaterniond(high.orientation.w,high.orientation.x,
    high.orientation.y,high.orientation.z).normalized().toRotationMatrix();
  auto moving_world=contact_world;
  const auto id="task26_cube_"+std::to_string(task.cube_index+1);
  // 与原合法接触推入一致，仅运动中的当前Cube不作为静态障碍；不改远端Scene/ACM。
  moving_world.erase(std::remove_if(moving_world.begin(),moving_world.end(),
    [&](const auto& object) { return object.id==id; }),moving_world.end());
  int solved=0,high_free=0;
  for (int sample=0;sample<60 && rclcpp::ok();++sample)
  {
    auto state=original;
    if (sample)
    {
      std::vector<double> q; original.copyJointGroupPositions(side,q);
      const auto& names=side->getVariableNames();
      for (std::size_t joint=0;joint<q.size();++joint)
      {
        const auto& bound=model->getVariableBounds(names[joint]);
        const double fraction=.01+.98*((sample*17+static_cast<int>(joint)*7)%101)/100.;
        q[joint]=bound.min_position_+fraction*(bound.max_position_-bound.min_position_);
      }
      state.setJointGroupPositions(side,q); state.update();
    }
    if (!state.setFromIK(side,high_tf,eef,.05)) continue;
    state.update();
    if (!state.satisfiesBounds()) continue;
    ++solved;
    if (collision_scene.isStateColliding(state)) continue;
    ++high_free;
    trajectory_msgs::msg::JointTrajectory approach;
    if (!planSeededEmptyCartesian(node,state,side,eef,contact,
          label+" SIDE_DESCENT seed="+std::to_string(sample),&approach)) continue;
    std::vector<double> lq,rq;
    state.copyJointGroupPositions(lj,lq); state.copyJointGroupPositions(rj,rq);
    trajectory_msgs::msg::JointTrajectory lc,rc;
    lc.joint_names=lj->getVariableNames(); rc.joint_names=rj->getVariableNames();
    const double duration=pointTime(approach.points.back());
    lc=side_left ? approach:holdTrajectory(lc,lq,duration);
    rc=side_left ? holdTrajectory(rc,rq,duration):approach;
    if (!validateSync(node,model,contact_world,lc,rc,label+" SIDE_DESCENT_WITH_FREE_CUBE")) continue;
    trajectory_msgs::msg::JointTrajectory lx,rx,ly,ry,le,re;
    const double press_y=dualFixturePressY(task,cube.position.y,direction);
    if (!planDualFixtureTranslation(node,lg,rg,lc,rc,task.cell.position.x-cube.position.x,0.,
          moving_world,label+" DUAL_X",&lx,&rx) ||
        !planDualFixtureTranslation(node,lg,rg,lx,rx,0.,press_y-cube.position.y,
          moving_world,label+" DUAL_Y",&ly,&ry) ||
        !planDualFixtureReleaseExit(node,model,ly,ry,side_left,moving_world,
          label+" DUAL_SHORT_EXIT",&le,&re)) continue;
    *xleft=std::move(lx); *xright=std::move(rx);
    *exit_left=std::move(le); *exit_right=std::move(re);
    RCLCPP_INFO(node->get_logger(),
      "%s COUPLED_REAR_PREFLIGHT PASS seed=%d: HIGH/continuous CONTACT/X/Y/short exit; no commands, HIGH RRT connection still required.",
      label.c_str(),sample);
    return true;
  }
  RCLCPP_WARN(node->get_logger(),
    "%s COUPLED_REAR_PREFLIGHT rejected: IK_solved=%d HIGH_free=%d /60; reject before rear CLOSE, not global infeasibility proof.",
    label.c_str(),solved,high_free);
  return false;
}

bool runDualFixture(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  moveit::planning_interface::PlanningSceneInterface& scene,
  const Arm& left, const Arm& right, const Arm& rear, const Arm& side,
  const CubeBuffer& cubes,
  const FixtureGeometryBuffer& fixture_geometry,
  const ForceBuffer& lf, const ForceBuffer& rf, const OfflineTask& task,
  DualFixtureMotion* output)
{
  if (!output || !fr3_dual_palletize::dualFixtureRequired(task.cube_index) ||
      !rear.isClosed() || side.isClosed()) return false;
  const auto model = lg.getRobotModel();
  const bool side_left = side.isLeft();
  // [ENGINEERING] 专用异步阶段标签，仅供只读post-step记录；不改变控制律。
  const auto phase_pub = node->create_publisher<std_msgs::msg::String>(
    "/task01/fixture_phase", rclcpp::QoS(10).transient_local());
  const auto mark_phase = [&](const std::string& phase) {
    std_msgs::msg::String message; message.data = std::string(task.id) + ":" + phase;
    phase_pub->publish(message);
  };
  struct PhaseExit
  {
    std::function<void(const std::string&)> mark;
    bool success{false};
    ~PhaseExit() { mark(success ? "COMPLETE" : "ABORT"); }
  } phase_exit{mark_phase};
  mark_phase("CONTACT");
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
    geometry_msgs::msg::Pose cube, left_tcp, right_tcp;
    std::int64_t stamp;
    if (!fixture_geometry.get(task.cube_index, &cube, &left_tcp, &right_tcp, &stamp)) {
      RCLCPP_ERROR(node->get_logger(), "%s %s atomic physics feedback missing/invalid/stale; no legacy fallback.",
        task.id, label.c_str());
      return false;
    }
    const auto& rear_tcp = rear.isLeft() ? left_tcp : right_tcp;
    const auto& side_tcp = side_left ? left_tcp : right_tcp;
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
      "%s %s rear=%s side=%s rear_gap=%.3f mm side_gap=%.3f mm alignment=%.3f mm tilt=%.3f deg. physics_stamp_ns=%lld",
      task.id, label.c_str(), rear.isClosed() ? "CLOSED" : "OPEN", side.isClosed() ? "CLOSED" : "OPEN",
      rear_gap * 1000.0, side_gap * 1000.0, alignment * 1000.0, tilt, static_cast<long long>(stamp));
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
  const double side_roll = node->get_parameter("dual_fixture_side_roll_world_y_deg").as_double();
  RCLCPP_INFO(node->get_logger(), "%s DUAL_SIDE_WRIST_ROLL worldY=%.3f deg; original cup center/normal, OPEN regrasp only.",
    task.id, side_roll);
  const auto high = dualSideFixturePose(initial.position.x, contact_y,
    initial.position.z + kRegraspLiftHeight, side_left, side_roll);
  const auto contact = dualSideFixturePose(initial.position.x, contact_y, initial.position.z,
    side_left, side_roll);
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
    const double lp = lf.peak(), rp = rf.peak();
    const bool safe = rclcpp::ok() && rear.isClosed() && !side.isClosed() &&
      std::max(lp, rp) <= kPushTorqueLimit;
    if (!safe) RCLCPP_ERROR(node->get_logger(),
      "%s DUAL_SIDE_APPROACH active interlock: rear=%s side=%s left_peak=%.3f Nm "
      "right_peak=%.3f Nm limit=%.3f Nm; no later command.", task.id,
      rear.isClosed() ? "CLOSED" : "OPEN", side.isClosed() ? "CLOSED" : "OPEN",
      lp, rp, kPushTorqueLimit);
    return safe;
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
      mark_phase(std::string(pushing_x ? "X_" : "Y_") + std::to_string(slice));
      trajectory_msgs::msg::JointTrajectory lpart, rpart;
      if (!sliceTrajectory(l, duration * (slice-1) / kPushSlices, duration * slice / kPushSlices, &lpart) ||
          !sliceTrajectory(r, duration * (slice-1) / kPushSlices, duration * slice / kPushSlices, &rpart) ||
          !executeSync(left, lpart, right, rpart, [&] {
            const double left_torque = lf.peak(), right_torque = rf.peak();
            const bool fresh_feedback = fixture_geometry.fresh();
            const bool safe = rclcpp::ok() && fresh_feedback && rear.isClosed() && side.isClosed() &&
              std::isfinite(left_torque) && std::isfinite(right_torque) &&
              std::max(left_torque, right_torque) <= kPushTorqueLimit;
            if (!safe)
              RCLCPP_ERROR(node->get_logger(),
                "%s %s slice=%d active interlock: rear=%s side=%s left_peak=%.3f Nm right_peak=%.3f Nm (limit=%.3f) atomic_feedback=%s; no later command.",
                task.id, label.c_str(), slice, rear.isClosed() ? "CLOSED" : "OPEN",
                side.isClosed() ? "CLOSED" : "OPEN", left_torque, right_torque, kPushTorqueLimit,
                fresh_feedback ? "FRESH" : "STALE_OR_INVALID");
            return safe;
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
  mark_phase("ROLE_SWAP");
  // [ENGINEERING] 推到实体深墙后不能从命令终点继续：微小伺服残差在两个D6
  // 闭环下会变成额外顶墙命令。重新读取一份双臂实测状态，不改几何/力矩门限。
  trajectory_msgs::msg::JointTrajectory measured_l, measured_r;
  geometry_msgs::msg::Pose measured_cube;
  if (!seed(&measured_l, &measured_r) ||
      !cubes.waitNew(task.cube_index, revision, 2.0, &measured_cube) ||
      !rear.isClosed() || !side.isClosed() || !geometry(true, "DUAL_ROLE_SWAP_MEASURED") ||
      !validateSync(node, model, world_push, measured_l, measured_r,
        std::string(task.id) + " DUAL_ROLE_SWAP_MEASURED_START")) return false;
  const auto log_delta = [&](const char* arm,
    const trajectory_msgs::msg::JointTrajectory& command,
    const trajectory_msgs::msg::JointTrajectory& measured)
  {
    double delta = 0.0;
    const auto q = finalPositions(command), actual = finalPositions(measured);
    for (std::size_t joint = 0; joint < q.size(); ++joint)
      delta = std::max(delta, std::abs(q[joint] - actual.at(joint)));
    RCLCPP_INFO(node->get_logger(), "%s DUAL_ROLE_SWAP measured %s max_command_delta=%.6f rad.",
      task.id, arm, delta);
  };
  log_delta("left", lx, measured_l); log_delta("right", rx, measured_r);
  if (!stage(measured_l, measured_r, 0,
        dualFixturePressY(task, measured_cube.position.y, direction) - measured_cube.position.y,
        world_push, std::string(task.id) + " DUAL_SIDE_PRIMARY_Y_MEASURED", &ly, &ry) ||
      !planDualFixtureReleaseExit(node, model, ly, ry, side_left, world_push,
        std::string(task.id) + " DUAL_RELEASE_EXIT_MEASURED", &output->left_exit, &output->right_exit)) return false;
  RCLCPP_INFO(node->get_logger(), "%s DUAL_ROLE_SWAP: side PRIMARY, rear deep-wall HOLD; no suction release.", task.id);
  if (!run(ly, ry, "DUAL_SIDE_PRIMARY_Y", false)) return false;
  const auto [pressed, pressed_revision] = cubes.get(task.cube_index); (void)pressed_revision;
  if (!validCellPlacement(node, task, pressed, cubes)) return false;
  output->left_end = ly; output->right_end = ry;
  phase_exit.success = true;
  return true;  // 清障包络已在负载推入之前预检；真正执行仍等待双OPEN。
}
// [ENGINEERING] 仅OPEN空载转场，直接复用本机MoveIt OMPL插件的局部场景。
// 不启动额外move_group/后台沙箱，也不写远程Scene/参数。加密碰撞检查，
// 使用不改变关节路点的IPTP计时；持件链/全局MoveIt默认计时保持不变。
// 第一臂最多3候选；每个已通过联合FCL的第一臂终点，第二臂最多3候选。
// 不再先选最短后丢弃整池，不改变RRTConnect、目标、ACM或清障距离。
bool planFixtureEmptyHandoff(const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& lg,
  moveit::planning_interface::MoveGroupInterface& rg,
  const moveit::core::RobotState& start, bool left_closed, bool right_closed,
  const geometry_msgs::msg::Pose& left_target, const geometry_msgs::msg::Pose& right_target,
  const std::vector<moveit_msgs::msg::CollisionObject>& world, const std::string& label,
  trajectory_msgs::msg::JointTrajectory* left_out,
  trajectory_msgs::msg::JointTrajectory* right_out)
{
  const auto model = start.getRobotModel();
  const auto* lj = model->getJointModelGroup(lg.getName());
  const auto* rj = model->getJointModelGroup(rg.getName());
  const auto valid_pose = [](const geometry_msgs::msg::Pose& p) {
    return fr3_dual_palletize::validFixturePose({p.position.x,p.position.y,p.position.z,
      p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w});
  };
  std::vector<const moveit::core::AttachedBody*> attached;
  start.getAttachedBodies(attached);
  if (!left_out || !right_out || !lj || !rj || left_closed || right_closed ||
      !attached.empty() || !valid_pose(left_target) || !valid_pose(right_target)) return false;
  std::vector<double> lq,rq;
  start.copyJointGroupPositions(lj,lq); start.copyJointGroupPositions(rj,rq);
  if (!fr3_dual_palletize::validEmptyRetreatSeed(false,false,lq,rq,
        lj->getVariableCount(),rj->getVariableCount()) || !start.satisfiesBounds()) return false;
  trajectory_msgs::msg::JointTrajectory lp,rp;
  lp.joint_names=lj->getVariableNames(); rp.joint_names=rj->getVariableNames();
  if (!validateSync(node,model,world,holdTrajectory(lp,lq,.1),holdTrajectory(rp,rq,.1),
      label+" SAME_MEASURED_START_CUBE_INCLUDED")) return false;
  planning_pipeline::PlanningPipeline pipeline(model,node,"move_group",
    "ompl_interface/OMPLPlanner",std::vector<std::string>{});
  pipeline.displayComputedMotionPlans(false); pipeline.publishReceivedRequests(false);
  auto configs=pipeline.getPlannerManager()->getPlannerConfigurations();
  for (auto& [name,config]:configs)
  {
    (void)name;
    config.config["longest_valid_segment_fraction"]="0.0005";
  }
  pipeline.getPlannerManager()->setPlannerConfigurations(configs);
  const auto candidates = [&](moveit::planning_interface::MoveGroupInterface& group,
      const moveit::core::RobotState& state, const geometry_msgs::msg::Pose& target)
  {
    std::vector<std::pair<double,trajectory_msgs::msg::JointTrajectory>> ranked;
    const auto* own=model->getJointModelGroup(group.getName());
    std::vector<double> seed; state.copyJointGroupPositions(own,seed);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(3.0); group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(.12); group.setMaxAccelerationScalingFactor(.12);
    group.setPoseReferenceFrame("world"); group.setStartState(state);
    const std::string eef=group.getName()==lg.getName() ? "left_fr3_side_suction_tcp" : "right_fr3_side_suction_tcp";
    group.setEndEffectorLink(eef); group.clearPoseTargets();
    if (!group.setPoseTarget(target,eef)) return ranked;
    moveit_msgs::msg::MotionPlanRequest request;
    group.constructMotionPlanRequest(request);
    moveit::core::robotStateToRobotStateMsg(state,request.start_state);
    request.start_state.is_diff=false;
    auto local=std::make_shared<planning_scene::PlanningScene>(model);
    local->setCurrentState(state);
    for (const auto& object:world)
      if (!local->processCollisionObjectMsg(object)) throw std::runtime_error("Invalid handoff scene object");
    for (int attempt=1;attempt<=3;++attempt)
    {
      planning_interface::MotionPlanResponse response;
      if (!pipeline.generatePlan(local,request,response) || !response.trajectory_ ||
          response.error_code_.val!=moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
      {
        RCLCPP_WARN(node->get_logger(),"%s %s LOCAL_SCENE RRT failed attempt=%d/3.",
          label.c_str(),group.getName().c_str(),attempt); continue;
      }
      moveit_msgs::msg::RobotTrajectory raw;
      response.trajectory_->getRobotTrajectoryMsg(raw);
      bool geometry_preserved=true;
      trajectory_processing::IterativeParabolicTimeParameterization timing;
      if (!timing.computeTimeStamps(*response.trajectory_,.12,.12)) continue;
      moveit_msgs::msg::RobotTrajectory timed;
      response.trajectory_->getRobotTrajectoryMsg(timed);
      auto path=timed.joint_trajectory;
      geometry_preserved=path.points.size()==raw.joint_trajectory.points.size();
      for (std::size_t i=0;geometry_preserved && i<path.points.size();++i)
        geometry_preserved=path.points[i].positions==raw.joint_trajectory.points[i].positions;
      bool audit=false; node->get_parameter_or("empty_handoff_audit_totg",audit,false);
      if (audit)
      {
        auto raw_path=raw.joint_trajectory; ensureTiming(raw_path);
        trajectory_msgs::msg::JointTrajectory held; held.joint_names=(group.getName()==lg.getName() ? rj:lj)->getVariableNames();
        std::vector<double> hq; state.copyJointGroupPositions(group.getName()==lg.getName() ? rj:lj,hq);
        const auto check=[&](const auto& motion,const std::string& tag) {
          return group.getName()==lg.getName() ?
            validateSync(node,model,world,motion,holdTrajectory(held,hq,pointTime(motion.points.back())),tag) :
            validateSync(node,model,world,holdTrajectory(held,hq,pointTime(motion.points.back())),motion,tag);
        };
        const bool raw_safe=check(raw_path,label+" RAW_GEOMETRIC_AB");
        robot_trajectory::RobotTrajectory rounded(*response.trajectory_,true);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool rounded_safe=false;
        if (totg.computeTimeStamps(rounded,.12,.12))
        {
          moveit_msgs::msg::RobotTrajectory output; rounded.getRobotTrajectoryMsg(output);
          rounded_safe=check(output.joint_trajectory,label+" TOTG_AB_NOT_EXECUTED");
        }
        RCLCPP_INFO(node->get_logger(),"%s %s SAME_RAW_PATH_AB raw_safe=%s default_TOTG_safe=%s IPTP_geometry_exact=%s; NO_PHYSICS.",
          label.c_str(),group.getName().c_str(),raw_safe ? "PASS":"FAIL",rounded_safe ? "PASS":"FAIL",geometry_preserved ? "PASS":"FAIL");
      }
      bool valid=geometry_preserved && path.joint_names==own->getVariableNames() && path.points.size()>=2;
      double travel=0.,previous=-1.;
      auto checked=state;
      for (std::size_t i=0;valid && i<path.points.size();++i)
      {
        const auto& q=path.points[i].positions; const auto time=pointTime(path.points[i]);
        valid=fr3_dual_palletize::validMeasuredJointSeed(q,seed.size()) &&
          std::isfinite(time) && time>=0. && time>previous;
        if (!valid) break;
        for (std::size_t j=0;j<q.size();++j)
        {
          if (i==0 && std::abs(q[j]-seed[j])>1e-6) valid=false;
          if (i) travel+=std::abs(q[j]-path.points[i-1].positions[j]);
        }
        checked.setJointGroupPositions(own,q); checked.update();
        valid=valid && checked.satisfiesBounds(); previous=time;
      }
      bool goal_met=false;
      if (valid) for (const auto& constraint:request.goal_constraints)
      {
        kinematic_constraints::KinematicConstraintSet set(model);
        if (set.add(constraint,local->getTransforms()) && set.decide(checked).satisfied) goal_met=true;
      }
      valid=valid && goal_met;
      RCLCPP_INFO(node->get_logger(),"%s %s LOCAL_SCENE RRT attempt=%d/3 numeric_start_goal=%s travel=%.3f.",
        label.c_str(),group.getName().c_str(),attempt,valid ? "PASS":"FAIL",travel);
      if (valid) { ensureTiming(path); ranked.emplace_back(travel,std::move(path)); }
    }
    group.clearPoseTargets(); group.setStartStateToCurrentState();
    std::sort(ranked.begin(),ranked.end(),[](const auto& a,const auto& b) { return a.first<b.first; });
    return ranked;
  };
  const auto left_paths=candidates(lg,start,left_target);
  for (const auto& [left_cost,left_path]:left_paths)
  {
    (void)left_cost;
    if (!validateSync(node,model,world,left_path,
        holdTrajectory(rp,rq,pointTime(left_path.points.back())),label+" LEFT_POOL_FCL")) continue;
    auto second_start=start;
    second_start.setJointGroupPositions(lj,finalPositions(left_path)); second_start.update();
    const auto right_paths=candidates(rg,second_start,right_target);
    for (const auto& [right_cost,right_path]:right_paths)
    {
      (void)right_cost;
      if (!validateSync(node,model,world,
          holdTrajectory(lp,finalPositions(left_path),pointTime(right_path.points.back())),
          right_path,label+" RIGHT_PREDICTED_PARTNER_FCL")) continue;
      *left_out=left_path; *right_out=right_path;
      RCLCPP_INFO(node->get_logger(),"%s SAFE_EMPTY_HANDOFF_PAIR_READY: sequential left/right, both OPEN, current Cube included; NOT_YET_EXECUTED.",label.c_str());
      return true;
    }
  }
  return false;
}
}  // namespace
