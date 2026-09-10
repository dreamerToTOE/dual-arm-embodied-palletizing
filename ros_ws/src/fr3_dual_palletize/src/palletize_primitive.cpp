#include "fr3_dual_palletize/palletize_primitive.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>
#include <utility>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

namespace fr3_dual_palletize
{
namespace
{
constexpr double TABLE_TOP_Z = 0.050;
constexpr double BOX_SIZE = 0.030;
constexpr double BOX_HALF = 0.5 * BOX_SIZE;
constexpr double SUCTION_TCP_OFFSET_Z = 0.105;
constexpr double CONTACT_TCP_CLEARANCE = 0.001;
constexpr double PLACEMENT_RELEASE_GAP = 0.001;
constexpr double PRE_PICK_CLEARANCE = 0.064;
constexpr double LIFT_CLEARANCE = 0.099;
constexpr double MIN_PRE_PLACE_TCP_Z = 0.180;
constexpr double PRE_PLACE_CLEARANCE = 0.070;
constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

// Task08-A 以 Task04 的既有控制时序构造名义时间轴。
// 轨迹候选不接入 Isaac，因此 SUCTION state 的实际异步等待不在此处估计。
constexpr double NOMINAL_SUCTION_COMMAND_SEC = 0.20;
constexpr double NOMINAL_RELEASE_SETTLE_SEC = 0.30;


double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  int32_t sec = static_cast<int32_t>(std::floor(seconds));
  int64_t nsec = static_cast<int64_t>(
    std::llround((seconds - static_cast<double>(sec)) * 1e9));

  if (nsec >= 1000000000LL)
  {
    ++sec;
    nsec -= 1000000000LL;
  }

  point.time_from_start.sec = sec;
  point.time_from_start.nanosec =
    static_cast<uint32_t>(std::max<int64_t>(0, nsec));
}

void ensureTrajectoryTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1e-6)
  {
    return;
  }

  constexpr double FALLBACK_DT = 0.03;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i)
  {
    setPointTime(trajectory.points[i], static_cast<double>(i) * FALLBACK_DT);
  }
}

moveit_msgs::msg::CollisionObject makeBoxObject(
  const std::string& id,
  const geometry_msgs::msg::Pose& pose,
  int operation)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;

  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = operation;
  return object;
}

}  // namespace

StartGate::StartGate(std::size_t participants)
  : participants_(participants)
{
}

bool StartGate::arriveAndWait()
{
  std::unique_lock<std::mutex> lock(mutex_);

  if (cancelled_)
  {
    return false;
  }

  ++arrived_;
  if (arrived_ >= participants_)
  {
    released_ = true;
    cv_.notify_all();
    return true;
  }

  cv_.wait(lock, [this]() { return released_ || cancelled_; });
  return released_ && !cancelled_;
}

void StartGate::cancel()
{
  std::lock_guard<std::mutex> lock(mutex_);
  cancelled_ = true;
  cv_.notify_all();
}

PalletizePrimitive::PalletizePrimitive(
  rclcpp::Node::SharedPtr node,
  PrimitiveConfig config,
  PoseProvider pose_provider,
  std::shared_ptr<std::mutex> planning_scene_mutex)
  : node_(std::move(node)),
    config_(std::move(config)),
    pose_provider_(std::move(pose_provider)),
    planning_scene_mutex_(std::move(planning_scene_mutex))
{
  command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    config_.joint_command_topic, 10);
  suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    config_.suction_command_topic, 10);

  suction_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    config_.suction_state_topic,
    10,
    [this](const std_msgs::msg::Bool::SharedPtr msg)
    {
      suction_closed_.store(msg->data);
      have_suction_state_.store(true);
    });
}

bool PalletizePrimitive::waitForIsaacBridge()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] 等待 Isaac 控制链与吸盘 bridge...",
    config_.label.c_str());

  for (int i = 0; i < 120; ++i)
  {
    if (command_pub_->get_subscription_count() > 0 &&
        suction_pub_->get_subscription_count() > 0 &&
        have_suction_state_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "[%s] bridge READY", config_.label.c_str());
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }

  RCLCPP_ERROR(
    node_->get_logger(),
    "[%s] bridge 超时：检查 %s / %s / %s",
    config_.label.c_str(),
    config_.joint_command_topic.c_str(),
    config_.suction_command_topic.c_str(),
    config_.suction_state_topic.c_str());
  return false;
}

bool PalletizePrimitive::waitForTaskExecutionBridge()
{
  return waitForIsaacBridge();
}

bool PalletizePrimitive::configureMoveGroup(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup*& joint_model_group)
{
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);
  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink(config_.eef_link);

  joint_model_group =
    move_group.getRobotModel()->getJointModelGroup(config_.planning_group);
  if (joint_model_group == nullptr)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] 找不到 planning group=%s",
      config_.label.c_str(), config_.planning_group.c_str());
    return false;
  }
  return true;
}

geometry_msgs::msg::Pose PalletizePrimitive::makeTopDownPose(
  double x, double y, double suction_tcp_world_z, double yaw) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = suction_tcp_world_z + SUCTION_TCP_OFFSET_Z;

  const double half_yaw = 0.5 * yaw;
  pose.orientation.x = std::cos(half_yaw);
  pose.orientation.y = std::sin(half_yaw);
  pose.orientation.z = 0.0;
  pose.orientation.w = 0.0;
  return pose;
}

bool PalletizePrimitive::setStartStateForGroup(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  const std::vector<double>& group_q)
{
  auto full_state = move_group.getCurrentState(2.0);
  if (!full_state)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] 无法读取双臂当前 RobotState。",
      config_.label.c_str());
    return false;
  }

  full_state->setJointGroupPositions(joint_model_group, group_q);
  full_state->update();
  move_group.setStartState(*full_state);
  return true;
}

bool PalletizePrimitive::planPoseStage(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  const std::string& eef_link,
  const std::vector<double>& start_q,
  const geometry_msgs::msg::Pose& target_pose,
  const std::string& stage_name,
  trajectory_msgs::msg::JointTrajectory& trajectory_out)
{
  if (!setStartStateForGroup(move_group, joint_model_group, start_q))
  {
    return false;
  }

  move_group.clearPoseTargets();
  if (!move_group.setPoseTarget(target_pose, eef_link))
  {
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group.plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] %s：RRTConnect 规划失败。",
      config_.label.c_str(), stage_name.c_str());
    return false;
  }

  trajectory_out = plan.trajectory_.joint_trajectory;
  if (trajectory_out.points.empty())
  {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %s：plan OK, points=%zu, duration=%.3f s",
    config_.label.c_str(), stage_name.c_str(),
    trajectory_out.points.size(), pointTime(trajectory_out.points.back()));
  return true;
}

bool PalletizePrimitive::planCartesianStage(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  const std::vector<double>& start_q,
  const geometry_msgs::msg::Pose& target_pose,
  const std::string& stage_name,
  trajectory_msgs::msg::JointTrajectory& trajectory_out)
{
  if (!setStartStateForGroup(move_group, joint_model_group, start_q))
  {
    return false;
  }

  std::vector<geometry_msgs::msg::Pose> waypoints{target_pose};
  moveit_msgs::msg::RobotTrajectory robot_trajectory;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  const double fraction = move_group.computeCartesianPath(
    waypoints,
    CARTESIAN_EEF_STEP,
    0.0,
    robot_trajectory,
    true,
    &error_code);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %s：Cartesian fraction=%.4f, error=%d",
    config_.label.c_str(), stage_name.c_str(), fraction, error_code.val);

  if (fraction < CARTESIAN_MIN_FRACTION)
  {
    return false;
  }

  trajectory_out = robot_trajectory.joint_trajectory;
  if (trajectory_out.points.empty())
  {
    return false;
  }

  ensureTrajectoryTiming(trajectory_out);
  return true;
}

bool PalletizePrimitive::planJointStage(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  const std::vector<double>& start_q,
  const std::vector<double>& target_q,
  const std::string& stage_name,
  trajectory_msgs::msg::JointTrajectory& trajectory_out)
{
  if (!setStartStateForGroup(move_group, joint_model_group, start_q))
  {
    return false;
  }

  move_group.clearPoseTargets();
  if (!move_group.setJointValueTarget(target_q))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] %s：无法设置关节目标。",
      config_.label.c_str(), stage_name.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] %s：RRTConnect 规划失败。",
      config_.label.c_str(), stage_name.c_str());
    return false;
  }

  trajectory_out = plan.trajectory_.joint_trajectory;
  if (trajectory_out.points.empty())
  {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] %s：plan OK, points=%zu, duration=%.3f s",
    config_.label.c_str(), stage_name.c_str(),
    trajectory_out.points.size(), pointTime(trajectory_out.points.back()));
  return true;
}

std::vector<std::string> PalletizePrimitive::isaacJointNames(
  const std::vector<std::string>& moveit_names) const
{
  std::vector<std::string> result;
  result.reserve(moveit_names.size());

  for (const auto& name : moveit_names)
  {
    if (name.rfind(config_.moveit_joint_prefix, 0) == 0)
    {
      result.push_back(name.substr(config_.moveit_joint_prefix.size()));
    }
    else
    {
      result.push_back(name);
    }
  }
  return result;
}

bool PalletizePrimitive::executeTrajectory(
  const trajectory_msgs::msg::JointTrajectory& input)
{
  if (input.points.empty())
  {
    return false;
  }

  auto trajectory = input;
  ensureTrajectoryTiming(trajectory);
  const auto isaac_names = isaacJointNames(trajectory.joint_names);

  const double total_time = pointTime(trajectory.points.back());
  std::size_t segment = 0;
  const auto start_time = std::chrono::steady_clock::now();

  auto publish = [&](const std::vector<double>& q)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = node_->now();
    msg.name = isaac_names;
    msg.position = q;
    command_pub_->publish(msg);
  };

  while (true)
  {
    const double t = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();

    if (t > total_time)
    {
      break;
    }

    while (segment + 1 < trajectory.points.size() &&
           pointTime(trajectory.points[segment + 1]) < t)
    {
      ++segment;
    }

    std::vector<double> q_command;
    if (segment + 1 >= trajectory.points.size())
    {
      q_command = trajectory.points.back().positions;
    }
    else
    {
      const auto& p0 = trajectory.points[segment];
      const auto& p1 = trajectory.points[segment + 1];
      const double t0 = pointTime(p0);
      const double t1 = pointTime(p1);
      double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
      alpha = std::clamp(alpha, 0.0, 1.0);

      q_command.resize(p0.positions.size());
      for (std::size_t i = 0; i < q_command.size(); ++i)
      {
        q_command[i] = p0.positions[i] +
                       alpha * (p1.positions[i] - p0.positions[i]);
      }
    }

    publish(q_command);
    std::this_thread::sleep_for(10ms);
  }

  const auto& final_q = trajectory.points.back().positions;
  for (int i = 0; i < 50; ++i)
  {
    publish(final_q);
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

bool PalletizePrimitive::publishTaskTrajectorySample(
  const trajectory_msgs::msg::JointTrajectory& input,
  double time_sec)
{
  if (input.joint_names.empty() || input.points.empty())
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] 无法发布空的 TaskTrajectoryCandidate。",
      config_.label.c_str());
    return false;
  }

  auto trajectory = input;
  ensureTrajectoryTiming(trajectory);
  const auto isaac_names = isaacJointNames(trajectory.joint_names);
  const double clamped_time = std::clamp(
    time_sec, 0.0, pointTime(trajectory.points.back()));

  std::size_t segment = 0;
  while (segment + 1 < trajectory.points.size() &&
         pointTime(trajectory.points[segment + 1]) < clamped_time)
  {
    ++segment;
  }

  std::vector<double> positions;
  if (segment + 1 >= trajectory.points.size())
  {
    positions = trajectory.points.back().positions;
  }
  else
  {
    const auto& first = trajectory.points[segment];
    const auto& second = trajectory.points[segment + 1];
    const double first_time = pointTime(first);
    const double second_time = pointTime(second);
    const double alpha = second_time > first_time ? std::clamp(
      (clamped_time - first_time) / (second_time - first_time), 0.0, 1.0) : 0.0;

    positions.resize(first.positions.size());
    for (std::size_t index = 0; index < positions.size(); ++index)
    {
      positions[index] = first.positions[index] +
        alpha * (second.positions[index] - first.positions[index]);
    }
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = isaac_names;
  msg.position = std::move(positions);
  command_pub_->publish(msg);
  return true;
}

void PalletizePrimitive::commandSuction(bool on)
{
  std_msgs::msg::Bool msg;
  msg.data = on;
  for (int i = 0; i < 20; ++i)
  {
    suction_pub_->publish(msg);
    std::this_thread::sleep_for(10ms);
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] SUCTION %s",
    config_.label.c_str(), on ? "ON" : "OFF");
}

bool PalletizePrimitive::waitForSuctionClosed(bool expected, double timeout_sec)
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(
           std::chrono::steady_clock::now() - start).count() < timeout_sec)
  {
    if (have_suction_state_.load() && suction_closed_.load() == expected)
    {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

void PalletizePrimitive::removeWorldObject(const std::string& object_id)
{
  {
    std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    psi.removeCollisionObjects({object_id});
  }
  std::this_thread::sleep_for(250ms);
}

bool PalletizePrimitive::addWorldObject(
  const std::string& object_id,
  const geometry_msgs::msg::Pose& pose)
{
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    ok = psi.applyCollisionObject(makeBoxObject(
      object_id, pose, moveit_msgs::msg::CollisionObject::ADD));
  }
  std::this_thread::sleep_for(250ms);
  return ok;
}

bool PalletizePrimitive::attachObject(
  const std::string& object_id,
  const std::string& eef_link)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = eef_link;
  attached.touch_links = {eef_link, config_.tool_link};
  attached.object.header.frame_id = eef_link;
  attached.object.id = object_id;

  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};

  geometry_msgs::msg::Pose relative_pose;
  relative_pose.position.z =
    SUCTION_TCP_OFFSET_Z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  relative_pose.orientation.x = 1.0;
  relative_pose.orientation.w = 0.0;

  attached.object.primitives.push_back(shape);
  attached.object.primitive_poses.push_back(relative_pose);
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    ok = psi.applyAttachedCollisionObject(attached);
  }
  std::this_thread::sleep_for(350ms);
  return ok;
}

bool PalletizePrimitive::detachObject(
  const std::string& object_id,
  const std::string& eef_link)
{
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = eef_link;
  attached.object.id = object_id;
  attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface psi;
    ok = psi.applyAttachedCollisionObject(attached);
  }
  std::this_thread::sleep_for(300ms);
  return ok;
}

bool PalletizePrimitive::applyTaskEvent(
  const TaskEvent& event,
  double grasp_timeout_sec,
  double release_timeout_sec)
{
  if (event.object_name != config_.object_id)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] TaskEvent object=%s 与 primitive object=%s 不一致。",
      config_.label.c_str(), event.object_name.c_str(), config_.object_id.c_str());
    return false;
  }

  switch (event.type)
  {
    case TaskEventType::SUCTION_ON:
      // CONTACT 前目标 Box 仍是 World object；进入吸附状态前才删除它，
      // 保持 Task08-B 的几何状态和 Task04 的真实抓取顺序一致。
      removeWorldObject(config_.object_id);
      commandSuction(true);
      if (!waitForSuctionClosed(true, grasp_timeout_sec))
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[%s] SUCTION_ON 后等待 CLOSED 超时。", config_.label.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "[%s] SUCTION CLOSED confirmed", config_.label.c_str());
      return true;

    case TaskEventType::ATTACH:
      if (!suction_closed_.load() ||
          !attachObject(config_.object_id, config_.eef_link))
      {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MoveIt ATTACH 失败。", config_.label.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "[%s] MoveIt ATTACH completed", config_.label.c_str());
      return true;

    case TaskEventType::SUCTION_OFF:
      commandSuction(false);
      if (!waitForSuctionClosed(false, release_timeout_sec))
      {
        RCLCPP_ERROR(
          node_->get_logger(),
          "[%s] SUCTION_OFF 后等待 OPEN 超时。", config_.label.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "[%s] SUCTION OPEN confirmed", config_.label.c_str());
      return true;

    case TaskEventType::DETACH:
    {
      // 必须等 Isaac 已经物理释放后才从 MoveIt AttachedCollisionObject 脱离；
      // 然后将最新 Ground Truth 回写为 World object，供安全退出段使用。
      if (!detachObject(config_.object_id, config_.eef_link))
      {
        RCLCPP_ERROR(node_->get_logger(), "[%s] MoveIt DETACH 失败。", config_.label.c_str());
        return false;
      }

      const auto settled_pose = pose_provider_(config_.pose_index);
      removeWorldObject(config_.object_id);
      if (!addWorldObject(config_.object_id, settled_pose))
      {
        RCLCPP_ERROR(node_->get_logger(), "[%s] 释放后 Box 回写 MoveIt World 失败。", config_.label.c_str());
        return false;
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] Isaac settle pose -> MoveIt World: (%.4f, %.4f, %.4f)",
        config_.label.c_str(),
        settled_pose.position.x, settled_pose.position.y, settled_pose.position.z);
      return true;
    }
  }

  return false;
}

void PalletizePrimitive::emergencySuctionOff()
{
  commandSuction(false);
}

std::vector<double> PalletizePrimitive::currentFrom(
  const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    return {};
  }
  return trajectory.points.back().positions;
}

double PalletizePrimitive::trajectoryDuration(
  const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    return 0.0;
  }
  return pointTime(trajectory.points.back());
}

bool PalletizePrimitive::appendTrajectory(
  trajectory_msgs::msg::JointTrajectory& destination,
  const trajectory_msgs::msg::JointTrajectory& stage_input)
{
  if (stage_input.points.empty())
  {
    return false;
  }

  auto stage = stage_input;
  ensureTrajectoryTiming(stage);

  if (destination.joint_names.empty())
  {
    destination.joint_names = stage.joint_names;
  }
  else if (destination.joint_names != stage.joint_names)
  {
    return false;
  }

  const double offset = trajectoryDuration(destination);
  for (std::size_t index = 0; index < stage.points.size(); ++index)
  {
    // 相邻规划段的首点就是上段终点，跳过它以保持时间严格递增。
    if (!destination.points.empty() && index == 0 &&
        pointTime(stage.points[index]) <= 1e-6)
    {
      continue;
    }

    auto point = stage.points[index];
    setPointTime(point, offset + pointTime(point));
    destination.points.push_back(std::move(point));
  }

  return !destination.points.empty();
}

bool PalletizePrimitive::appendHold(
  trajectory_msgs::msg::JointTrajectory& trajectory,
  double hold_sec)
{
  if (trajectory.points.empty() || hold_sec < 0.0)
  {
    return false;
  }

  auto hold = trajectory.points.back();
  hold.velocities.assign(hold.positions.size(), 0.0);
  hold.accelerations.assign(hold.positions.size(), 0.0);
  setPointTime(hold, trajectoryDuration(trajectory) + hold_sec);
  trajectory.points.push_back(std::move(hold));
  return true;
}

bool PalletizePrimitive::planTaskTrajectoryCandidate(
  TaskTrajectoryCandidate& candidate)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "========== %s Task08-A full candidate planning START ==========" ,
    config_.label.c_str());

  // Task08-A 不能发布 command 或 suction；这里只依赖 MoveIt 与最新 Box Ground Truth。
  moveit::planning_interface::MoveGroupInterface move_group(
    node_, config_.planning_group);
  const moveit::core::JointModelGroup* joint_model_group = nullptr;
  if (!configureMoveGroup(move_group, joint_model_group))
  {
    return false;
  }

  return planTaskTrajectoryCandidateImpl(
    move_group, joint_model_group, candidate);
}

bool PalletizePrimitive::planTaskTrajectoryCandidateImpl(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  TaskTrajectoryCandidate& candidate)
{
  auto current_state = move_group.getCurrentState(3.0);
  if (!current_state)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] 无法读取当前 RobotState。", config_.label.c_str());
    return false;
  }

  std::vector<double> current_q;
  current_state->copyJointGroupPositions(joint_model_group, current_q);
  if (current_q.size() != 7)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] 当前关节数量=%zu，预期 7。",
      config_.label.c_str(), current_q.size());
    return false;
  }

  const geometry_msgs::msg::Pose pick_pose = pose_provider_(config_.pose_index);
  const double pick_contact_tcp_z =
    pick_pose.position.z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  const double pre_pick_tcp_z = pick_contact_tcp_z + PRE_PICK_CLEARANCE;
  const double lift_tcp_z = pick_contact_tcp_z + LIFT_CLEARANCE;

  const double planned_release_center_z =
    TABLE_TOP_Z + BOX_HALF + PLACEMENT_RELEASE_GAP;
  const double place_tcp_z =
    planned_release_center_z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  const double pre_place_tcp_z =
    std::max(MIN_PRE_PLACE_TCP_Z, place_tcp_z + PRE_PLACE_CLEARANCE);

  candidate = TaskTrajectoryCandidate();
  candidate.arm_name = config_.planning_group;
  candidate.label = config_.label;
  candidate.planning_group = config_.planning_group;
  candidate.object_id = config_.object_id;
  candidate.eef_link = config_.eef_link;
  candidate.object_touch_links = {config_.eef_link, config_.tool_link};
  candidate.start_q = current_q;
  candidate.initial_object_pose = pick_pose;
  candidate.planned_release_pose = pick_pose;
  candidate.planned_release_pose.position.x = config_.target_x;
  candidate.planned_release_pose.position.y = config_.target_y;
  candidate.planned_release_pose.position.z = planned_release_center_z;

  // 所有阶段均记录在同一条完整轨迹的时间轴上。Task09-B 据此只在
  // LIFT 完成、进入公共工作区前插入局部等待，不改变 Task04 抓放时序。
  auto appendStage =
    [&](const std::string& marker_name,
        const trajectory_msgs::msg::JointTrajectory& stage)
    {
      const double start_time = trajectoryDuration(candidate.trajectory);
      if (!appendTrajectory(candidate.trajectory, stage))
      {
        return false;
      }
      candidate.trajectory_segments.push_back(stage);
      candidate.stages.push_back(TaskStageMarker{
        marker_name,
        start_time,
        trajectoryDuration(candidate.trajectory)
      });
      return true;
    };

  auto pre_pick = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, pre_pick_tcp_z);
  trajectory_msgs::msg::JointTrajectory pre_pick_traj;
  if (!planPoseStage(
        move_group, joint_model_group, config_.eef_link,
        current_q, pre_pick, "TASK08-A HOME -> PRE_PICK", pre_pick_traj) ||
      !appendStage("HOME_TO_PRE_PICK", pre_pick_traj))
  {
    return false;
  }

  bool attached = false;
  bool restored = false;
  auto restoreInitialWorldObject = [&]()
  {
    if (restored)
    {
      return true;
    }

    bool ok = true;
    if (attached)
    {
      ok = detachObject(config_.object_id, config_.eef_link) && ok;
      attached = false;
    }

    // 无论 detach 是否将物体自动放回 world，都先删除后按原始 pose 回写。
    removeWorldObject(config_.object_id);
    ok = addWorldObject(config_.object_id, pick_pose) && ok;
    restored = true;
    return ok;
  };

  // CONTACT 阶段允许本臂与目标物产生有意接触；候选规划结束后会恢复该物体。
  removeWorldObject(config_.object_id);

  auto contact = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, pick_contact_tcp_z);
  trajectory_msgs::msg::JointTrajectory contact_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(pre_pick_traj),
        contact, "TASK08-A PRE_PICK -> CONTACT", contact_traj) ||
      !appendStage("PRE_PICK_TO_CONTACT", contact_traj))
  {
    restoreInitialWorldObject();
    return false;
  }

  // 对齐 Task04 的 nominal suction command 时间。物理吸盘命令与 MoveIt
  // AttachedCollisionObject 分别记录：Task08-B 只在 ATTACH 时切换几何状态。
  candidate.events.push_back(TaskEvent{
    trajectoryDuration(candidate.trajectory),
    TaskEventType::SUCTION_ON,
    config_.object_id,
    config_.eef_link
  });
  if (!appendHold(candidate.trajectory, NOMINAL_SUCTION_COMMAND_SEC) ||
      !attachObject(config_.object_id, config_.eef_link))
  {
    restoreInitialWorldObject();
    return false;
  }
  attached = true;
  candidate.events.push_back(TaskEvent{
    trajectoryDuration(candidate.trajectory),
    TaskEventType::ATTACH,
    config_.object_id,
    config_.eef_link
  });

  auto lift = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, lift_tcp_z);
  trajectory_msgs::msg::JointTrajectory lift_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(contact_traj),
        lift, "TASK08-A CONTACT -> LIFT", lift_traj) ||
      !appendStage("CONTACT_TO_LIFT", lift_traj))
  {
    restoreInitialWorldObject();
    return false;
  }

  auto pre_place = makeTopDownPose(
    config_.target_x, config_.target_y, pre_place_tcp_z);
  trajectory_msgs::msg::JointTrajectory pre_place_traj;
  if (!planPoseStage(
        move_group, joint_model_group, config_.eef_link,
        currentFrom(lift_traj), pre_place,
        "TASK08-A LIFT -> PRE_PLACE", pre_place_traj) ||
      !appendStage("LIFT_TO_PRE_PLACE", pre_place_traj))
  {
    restoreInitialWorldObject();
    return false;
  }

  auto place = makeTopDownPose(
    config_.target_x, config_.target_y, place_tcp_z);
  trajectory_msgs::msg::JointTrajectory place_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(pre_place_traj),
        place, "TASK08-A PRE_PLACE -> PLACE", place_traj) ||
      !appendStage("PRE_PLACE_TO_PLACE", place_traj))
  {
    restoreInitialWorldObject();
    return false;
  }

  // 保持 Attached + SUCTION ON 预先规划上退，与已验证 Task04 release/retreat 时序一致。
  auto retreat = makeTopDownPose(
    config_.target_x, config_.target_y, pre_place_tcp_z);
  trajectory_msgs::msg::JointTrajectory retreat_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(place_traj),
        retreat, "TASK08-A PLACE -> RETREAT", retreat_traj))
  {
    restoreInitialWorldObject();
    return false;
  }
  // 对齐 Task04：SUCTION OFF 命令窗口和 PhysX settle 后，才发生 MoveIt detach。
  // 与 SUCTION_ON 相同，物理命令不直接改变 Task08-B 的碰撞对象状态。
  candidate.events.push_back(TaskEvent{
    trajectoryDuration(candidate.trajectory),
    TaskEventType::SUCTION_OFF,
    config_.object_id,
    config_.eef_link
  });
  if (!appendHold(
        candidate.trajectory,
        NOMINAL_SUCTION_COMMAND_SEC + NOMINAL_RELEASE_SETTLE_SEC))
  {
    restoreInitialWorldObject();
    return false;
  }
  candidate.events.push_back(TaskEvent{
    trajectoryDuration(candidate.trajectory),
    TaskEventType::DETACH,
    config_.object_id,
    config_.eef_link
  });

  if (!appendStage("PLACE_TO_RETREAT", retreat_traj))
  {
    restoreInitialWorldObject();
    return false;
  }

  // 当前交叉场景中，两臂若都永久停在 RETREAT，会形成静态终态碰撞。
  // Task09-B 安全退出扩展仅在协调候选模式下，在真实释放后以 MoveIt 规划一段独立安全退出：
  // RETREAT -> HOME。放置后的 Box 保持为 world object，绝不忽略其碰撞几何。
  if (config_.include_safe_egress)
  {
    if (!detachObject(config_.object_id, config_.eef_link))
    {
      restoreInitialWorldObject();
      return false;
    }
    attached = false;
    removeWorldObject(config_.object_id);
    if (!addWorldObject(config_.object_id, candidate.planned_release_pose))
    {
      restoreInitialWorldObject();
      return false;
    }

    trajectory_msgs::msg::JointTrajectory egress_traj;
    if (!planJointStage(
          move_group,
          joint_model_group,
          currentFrom(retreat_traj),
          candidate.start_q,
          "TASK09-B RETREAT -> HOME SAFE_EGRESS",
          egress_traj) ||
        !appendStage("RETREAT_TO_HOME_SAFE_EGRESS", egress_traj))
    {
      restoreInitialWorldObject();
      return false;
    }
  }

  candidate.goal_q = currentFrom(candidate.trajectory);
  candidate.duration_sec = trajectoryDuration(candidate.trajectory);

  const bool restore_ok = restoreInitialWorldObject();
  if (!restore_ok)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] Task08-A 规划后恢复 Planning Scene 失败。",
      config_.label.c_str());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] FULL CANDIDATE READY: points=%zu, duration=%.3f s, events=%zu",
    config_.label.c_str(),
    candidate.trajectory.points.size(),
    candidate.duration_sec,
    candidate.events.size());
  return true;
}

bool PalletizePrimitive::prepareTransferCandidate(
  StartGate& start_gate,
  StartGate& coordination_gate,
  TransferCandidate& candidate)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "========== %s prepare candidate START ==========" ,
    config_.label.c_str());

  if (!waitForIsaacBridge())
  {
    start_gate.cancel();
    coordination_gate.cancel();
    return false;
  }

  moveit::planning_interface::MoveGroupInterface move_group(
    node_, config_.planning_group);
  const moveit::core::JointModelGroup* joint_model_group = nullptr;
  if (!configureMoveGroup(move_group, joint_model_group))
  {
    start_gate.cancel();
    coordination_gate.cancel();
    return false;
  }

  const bool ok = prepareTransferCandidateImpl(
    move_group,
    joint_model_group,
    start_gate,
    coordination_gate,
    candidate);

  if (!ok)
  {
    start_gate.cancel();
    coordination_gate.cancel();
  }
  return ok;
}

bool PalletizePrimitive::prepareTransferCandidateImpl(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  StartGate& start_gate,
  StartGate& coordination_gate,
  TransferCandidate& candidate)
{
  auto current_state = move_group.getCurrentState(3.0);
  if (!current_state)
  {
    return false;
  }

  std::vector<double> current_q;
  current_state->copyJointGroupPositions(joint_model_group, current_q);
  if (current_q.size() != 7)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] 当前关节数量=%zu，预期 7。",
      config_.label.c_str(), current_q.size());
    return false;
  }

  const geometry_msgs::msg::Pose pick_pose = pose_provider_(config_.pose_index);
  const double pick_contact_tcp_z =
    pick_pose.position.z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  const double pre_pick_tcp_z = pick_contact_tcp_z + PRE_PICK_CLEARANCE;
  const double lift_tcp_z = pick_contact_tcp_z + LIFT_CLEARANCE;

  auto pre_pick = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, pre_pick_tcp_z);

  trajectory_msgs::msg::JointTrajectory pre_pick_traj;
  if (!planPoseStage(
        move_group, joint_model_group, config_.eef_link,
        current_q, pre_pick, "PRE_PICK", pre_pick_traj))
  {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] PRE_PICK 已规划，等待另一臂同时启动。",
    config_.label.c_str());

  if (!start_gate.arriveAndWait())
  {
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] PARALLEL GO", config_.label.c_str());
  if (!executeTrajectory(pre_pick_traj))
  {
    return false;
  }

  // Task04 canonical primitive：接触前暂时从 MoveIt world 移除目标物体。
  removeWorldObject(config_.object_id);

  auto contact = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, pick_contact_tcp_z);
  trajectory_msgs::msg::JointTrajectory contact_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(pre_pick_traj),
        contact, "PRE_PICK -> CONTACT", contact_traj) ||
      !executeTrajectory(contact_traj))
  {
    return false;
  }

  commandSuction(true);
  if (!waitForSuctionClosed(true, 2.0))
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] CONTACT 后吸附失败。",
      config_.label.c_str());
    return false;
  }

  if (!attachObject(config_.object_id, config_.eef_link))
  {
    return false;
  }

  auto lift = makeTopDownPose(
    pick_pose.position.x, pick_pose.position.y, lift_tcp_z);
  trajectory_msgs::msg::JointTrajectory lift_traj;
  if (!planCartesianStage(
        move_group, joint_model_group, currentFrom(contact_traj),
        lift, "CONTACT -> LIFT", lift_traj) ||
      !executeTrajectory(lift_traj))
  {
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] 已到 LIFT coordination point；保持 Attached + SUCTION ON。",
    config_.label.c_str());

  // Task08 的关键协调接口：两臂都到达 LIFT 后，才开始生成候选 transfer。
  if (!coordination_gate.arriveAndWait())
  {
    return false;
  }

  const double planned_release_center_z =
    TABLE_TOP_Z + BOX_HALF + PLACEMENT_RELEASE_GAP;
  const double place_tcp_z =
    planned_release_center_z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  const double pre_place_tcp_z =
    std::max(MIN_PRE_PLACE_TCP_Z, place_tcp_z + PRE_PLACE_CLEARANCE);

  auto pre_place = makeTopDownPose(
    config_.target_x, config_.target_y, pre_place_tcp_z);

  trajectory_msgs::msg::JointTrajectory transfer_traj;
  const auto start_q = currentFrom(lift_traj);
  if (!planPoseStage(
        move_group,
        joint_model_group,
        config_.eef_link,
        start_q,
        pre_place,
        "LIFT -> PRE_PLACE CANDIDATE",
        transfer_traj))
  {
    return false;
  }

  candidate.label = config_.label;
  candidate.planning_group = config_.planning_group;
  candidate.object_id = config_.object_id;
  candidate.eef_link = config_.eef_link;
  candidate.trajectory = transfer_traj;
  candidate.start_q = start_q;
  candidate.goal_q = currentFrom(transfer_traj);
  candidate.duration_sec = pointTime(transfer_traj.points.back());
  candidate.target_x = config_.target_x;
  candidate.target_y = config_.target_y;
  candidate.carrying_object = true;

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] CANDIDATE READY: joints=%zu, points=%zu, duration=%.3f s, target=(%.3f, %.3f)",
    config_.label.c_str(),
    candidate.trajectory.joint_names.size(),
    candidate.trajectory.points.size(),
    candidate.duration_sec,
    candidate.target_x,
    candidate.target_y);

  return true;
}

bool PalletizePrimitive::executePreparedTransferAndFinish(
  const TransferCandidate& candidate)
{
  if (candidate.trajectory.points.empty() || !candidate.carrying_object)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] candidate 无效。", config_.label.c_str());
    return false;
  }

  if (candidate.planning_group != config_.planning_group ||
      candidate.object_id != config_.object_id ||
      candidate.eef_link != config_.eef_link)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] candidate 元数据与 primitive 配置不匹配。",
      config_.label.c_str());
    return false;
  }

  moveit::planning_interface::MoveGroupInterface move_group(
    node_, config_.planning_group);
  const moveit::core::JointModelGroup* joint_model_group = nullptr;
  if (!configureMoveGroup(move_group, joint_model_group))
  {
    return false;
  }

  return finishPreparedTransferImpl(move_group, joint_model_group, candidate);
}

bool PalletizePrimitive::finishPreparedTransferImpl(
  moveit::planning_interface::MoveGroupInterface& move_group,
  const moveit::core::JointModelGroup* joint_model_group,
  const TransferCandidate& candidate)
{
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] Coordinator 已批准 candidate，开始执行 LIFT -> PRE_PLACE。",
    config_.label.c_str());

  if (!executeTrajectory(candidate.trajectory))
  {
    return false;
  }

  const double planned_release_center_z =
    TABLE_TOP_Z + BOX_HALF + PLACEMENT_RELEASE_GAP;
  const double place_tcp_z =
    planned_release_center_z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  const double pre_place_tcp_z =
    std::max(MIN_PRE_PLACE_TCP_Z, place_tcp_z + PRE_PLACE_CLEARANCE);

  auto place = makeTopDownPose(
    candidate.target_x, candidate.target_y, place_tcp_z);
  trajectory_msgs::msg::JointTrajectory place_traj;
  if (!planCartesianStage(
        move_group,
        joint_model_group,
        candidate.goal_q,
        place,
        "PRE_PLACE -> PLACE",
        place_traj) ||
      !executeTrajectory(place_traj))
  {
    return false;
  }

  // Task04 canonical primitive：仍 Attached + SUCTION ON 时先规划 RETREAT。
  auto retreat = makeTopDownPose(
    candidate.target_x, candidate.target_y, pre_place_tcp_z);
  trajectory_msgs::msg::JointTrajectory retreat_traj;
  if (!planCartesianStage(
        move_group,
        joint_model_group,
        currentFrom(place_traj),
        retreat,
        "PLACE -> RETREAT",
        retreat_traj))
  {
    return false;
  }

  commandSuction(false);
  if (!waitForSuctionClosed(false, 2.0))
  {
    return false;
  }

  std::this_thread::sleep_for(300ms);
  const auto settled_pose = pose_provider_(config_.pose_index);

  if (!detachObject(config_.object_id, config_.eef_link))
  {
    return false;
  }

  removeWorldObject(config_.object_id);
  if (!addWorldObject(config_.object_id, settled_pose))
  {
    return false;
  }

  if (!executeTrajectory(retreat_traj))
  {
    return false;
  }

  const double e_xy = std::hypot(
    settled_pose.position.x - candidate.target_x,
    settled_pose.position.y - candidate.target_y);

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] SUCCESS: settled=(%.4f, %.4f, %.4f), target=(%.4f, %.4f), e_xy=%.2f mm",
    config_.label.c_str(),
    settled_pose.position.x, settled_pose.position.y, settled_pose.position.z,
    candidate.target_x, candidate.target_y, e_xy * 1000.0);
  return true;
}

bool PalletizePrimitive::runOnce(StartGate& start_gate)
{
  // Task07 继续走完整 primitive，但内部也通过同一 candidate 接口，
  // coordination_gate=1 表示无需等待另一臂的 LIFT 审批点。
  StartGate coordination_gate(1);
  TransferCandidate candidate;

  if (!prepareTransferCandidate(start_gate, coordination_gate, candidate))
  {
    return false;
  }

  return executePreparedTransferAndFinish(candidate);
}

}  // namespace fr3_dual_palletize
