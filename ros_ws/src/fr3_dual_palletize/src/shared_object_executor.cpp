#include "fr3_dual_palletize/shared_object_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <thread>
#include <utility>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std::chrono_literals;

namespace fr3_dual_palletize
{
namespace
{

constexpr double SCENE_SYNC_WAIT_SEC = 0.30;
constexpr double TIME_EPSILON = 1.0e-6;
constexpr double PI = 3.14159265358979323846;

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

void ensureTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty() || pointTime(trajectory.points.back()) > TIME_EPSILON)
  {
    return;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index)
  {
    const double seconds = 0.03 * static_cast<double>(index);
    trajectory.points[index].time_from_start.sec = static_cast<std::int32_t>(seconds);
    trajectory.points[index].time_from_start.nanosec = static_cast<std::uint32_t>(
      std::llround((seconds - std::floor(seconds)) * 1.0e9));
  }
}

std::vector<double> interpolatePositions(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  double time_sec)
{
  if (trajectory.points.empty())
  {
    return {};
  }
  if (time_sec <= pointTime(trajectory.points.front()))
  {
    return trajectory.points.front().positions;
  }
  if (time_sec >= pointTime(trajectory.points.back()))
  {
    return trajectory.points.back().positions;
  }
  for (std::size_t index = 1; index < trajectory.points.size(); ++index)
  {
    const double after = pointTime(trajectory.points[index]);
    if (time_sec > after)
    {
      continue;
    }
    const auto& before_point = trajectory.points[index - 1];
    const auto& after_point = trajectory.points[index];
    const double before = pointTime(before_point);
    const double alpha = after > before ? std::clamp(
      (time_sec - before) / (after - before), 0.0, 1.0) : 0.0;
    std::vector<double> positions(before_point.positions.size());
    for (std::size_t joint = 0; joint < positions.size(); ++joint)
    {
      positions[joint] = before_point.positions[joint] + alpha *
        (after_point.positions[joint] - before_point.positions[joint]);
    }
    return positions;
  }
  return trajectory.points.back().positions;
}

std::vector<std::string> isaacJointNames(
  const std::vector<std::string>& moveit_names,
  bool left)
{
  const std::string prefix = left ? "left_" : "right_";
  std::vector<std::string> names;
  names.reserve(moveit_names.size());
  for (const auto& name : moveit_names)
  {
    names.push_back(name.rfind(prefix, 0) == 0 ? name.substr(prefix.size()) : name);
  }
  return names;
}

double positionDistance(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  const double dz = first.z - second.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double quaternionAngularDistance(
  const geometry_msgs::msg::Quaternion& first,
  const geometry_msgs::msg::Quaternion& second)
{
  const double first_norm = std::sqrt(
    first.x * first.x + first.y * first.y + first.z * first.z + first.w * first.w);
  const double second_norm = std::sqrt(
    second.x * second.x + second.y * second.y + second.z * second.z + second.w * second.w);
  if (first_norm <= TIME_EPSILON || second_norm <= TIME_EPSILON)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = std::abs(
    first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w) /
    (first_norm * second_norm);
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

geometry_msgs::msg::Point subtractPoints(
  const geometry_msgs::msg::Point& first,
  const geometry_msgs::msg::Point& second)
{
  geometry_msgs::msg::Point result;
  result.x = first.x - second.x;
  result.y = first.y - second.y;
  result.z = first.z - second.z;
  return result;
}

bool expectedStages(const SharedObjectPlan& plan, std::string& error)
{
  static const std::vector<std::string> names{
    "SEQUENTIAL_PRE_CONTACT",
    "DUAL_CONTACT",
    "COMMON_LIFT",
    "COMMON_TRANSPORT",
    "COMMON_DESCENT",
    "PREPLANNED_COMMON_RETREAT",
  };
  if (!plan.valid || plan.stages.size() != names.size())
  {
    error = "SharedObjectPlan 无效或共同阶段数量不完整。";
    return false;
  }
  for (std::size_t index = 0; index < names.size(); ++index)
  {
    const auto& stage = plan.stages[index];
    if (stage.name != names[index] || stage.left.points.empty() || stage.right.points.empty() ||
        stage.left.joint_names.empty() || stage.right.joint_names.empty())
    {
      error = "SharedObjectPlan 阶段顺序或 JointTrajectory 无效：" + names[index];
      return false;
    }
  }
  return true;
}

}  // namespace

SharedObjectExecutor::SharedObjectExecutor(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<std::mutex> planning_scene_mutex,
  SharedObjectExecutionConfig config)
  : node_(std::move(node)),
    planning_scene_mutex_(std::move(planning_scene_mutex)),
    config_(std::move(config))
{
  left_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/left/joint_command", 10);
  right_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/right/joint_command", 10);
  left_suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/task20/left/suction_command", 10);
  right_suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
    "/task20/right/suction_command", 10);
  lock_object_pub_ = node_->create_publisher<std_msgs::msg::String>(
    "/task20/lock_placed_object", 10);
  left_suction_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/task20/left/suction_state", 10,
    [this](const std_msgs::msg::Bool::SharedPtr message)
    {
      left_suction_closed_.store(message->data);
      have_left_suction_state_.store(true);
    });
  right_suction_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/task20/right/suction_state", 10,
    [this](const std_msgs::msg::Bool::SharedPtr message)
    {
      right_suction_closed_.store(message->data);
      have_right_suction_state_.store(true);
    });
  left_tcp_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/task20/left/suction_tcp_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      left_tcp_pose_ = message->pose;
      have_left_tcp_pose_ = true;
    });
  right_tcp_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/task20/right/suction_tcp_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      right_tcp_pose_ = message->pose;
      have_right_tcp_pose_ = true;
    });
}

bool SharedObjectExecutor::waitForBridge() const
{
  for (int attempt = 0; attempt < 120; ++attempt)
  {
    bool have_tcp = false;
    {
      std::lock_guard<std::mutex> lock(tcp_mutex_);
      have_tcp = have_left_tcp_pose_ && have_right_tcp_pose_;
    }
    if (left_command_pub_->get_subscription_count() > 0 &&
        right_command_pub_->get_subscription_count() > 0 &&
        left_suction_pub_->get_subscription_count() > 0 &&
        right_suction_pub_->get_subscription_count() > 0 &&
        have_left_suction_state_.load() && have_right_suction_state_.load() && have_tcp)
    {
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  return false;
}

bool SharedObjectExecutor::executeOne(
  bool left,
  const trajectory_msgs::msg::JointTrajectory& input) const
{
  if (input.joint_names.empty() || input.points.empty())
  {
    return false;
  }
  auto trajectory = input;
  ensureTiming(trajectory);
  const double duration = pointTime(trajectory.points.back());
  if (duration <= TIME_EPSILON)
  {
    return false;
  }
  const auto names = isaacJointNames(trajectory.joint_names, left);
  const auto& publisher = left ? left_command_pub_ : right_command_pub_;
  const auto start = std::chrono::steady_clock::now();
  const auto publish = [&]()
  {
    const double logical_time = std::min(
      duration,
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() /
      config_.execution_time_scale);
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_->now();
    message.name = names;
    message.position = interpolatePositions(trajectory, logical_time);
    publisher->publish(message);
  };
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() /
           config_.execution_time_scale < duration)
  {
    publish();
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.command_period_sec));
  }
  for (int repeat = 0; repeat < 50; ++repeat)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = node_->now();
    message.name = names;
    message.position = trajectory.points.back().positions;
    publisher->publish(message);
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.command_period_sec));
  }
  return true;
}

bool SharedObjectExecutor::executeTogether(const SharedObjectStage& input) const
{
  auto left = input.left;
  auto right = input.right;
  ensureTiming(left);
  ensureTiming(right);
  if (left.points.empty() || right.points.empty())
  {
    return false;
  }
  const double duration = std::max(pointTime(left.points.back()), pointTime(right.points.back()));
  if (duration <= TIME_EPSILON)
  {
    return false;
  }
  const auto left_names = isaacJointNames(left.joint_names, true);
  const auto right_names = isaacJointNames(right.joint_names, false);
  const auto start = std::chrono::steady_clock::now() + 100ms;
  std::this_thread::sleep_until(start);
  const auto publish = [&](double logical_time)
  {
    sensor_msgs::msg::JointState left_message;
    left_message.header.stamp = node_->now();
    left_message.name = left_names;
    left_message.position = interpolatePositions(left, logical_time);
    left_command_pub_->publish(left_message);

    sensor_msgs::msg::JointState right_message;
    right_message.header.stamp = node_->now();
    right_message.name = right_names;
    right_message.position = interpolatePositions(right, logical_time);
    right_command_pub_->publish(right_message);
  };
  while (true)
  {
    const double logical_time = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start).count() / config_.execution_time_scale;
    if (logical_time > duration)
    {
      break;
    }
    publish(logical_time);
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.command_period_sec));
  }
  for (int repeat = 0; repeat < 50; ++repeat)
  {
    publish(duration);
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.command_period_sec));
  }
  return true;
}

bool SharedObjectExecutor::commandSuction(bool left, bool on) const
{
  const auto& publisher = left ? left_suction_pub_ : right_suction_pub_;
  if (!publisher || publisher->get_subscription_count() == 0)
  {
    return false;
  }
  std_msgs::msg::Bool message;
  message.data = on;
  for (int repeat = 0; repeat < 20; ++repeat)
  {
    publisher->publish(message);
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

bool SharedObjectExecutor::waitForSuction(bool left, bool expected, double timeout_sec) const
{
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() <
         timeout_sec)
  {
    const bool have = left ? have_left_suction_state_.load() : have_right_suction_state_.load();
    const bool closed = left ? left_suction_closed_.load() : right_suction_closed_.load();
    if (have && closed == expected)
    {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return false;
}

bool SharedObjectExecutor::getTcpPose(bool left, geometry_msgs::msg::Pose& pose) const
{
  std::lock_guard<std::mutex> lock(tcp_mutex_);
  const bool available = left ? have_left_tcp_pose_ : have_right_tcp_pose_;
  if (!available)
  {
    return false;
  }
  pose = left ? left_tcp_pose_ : right_tcp_pose_;
  return true;
}

bool SharedObjectExecutor::writeWorldObject(
  const BoxSpec& box,
  const geometry_msgs::msg::Pose& pose) const
{
  bool applied = false;
  {
    std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
    moveit::planning_interface::PlanningSceneInterface scene;
    const auto existing = scene.getObjects({box.id});
    if (!existing.empty())
    {
      scene.removeCollisionObjects({box.id});
      std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
    }
    applied = scene.applyCollisionObject(makeCollisionObject(box, pose));
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  return applied;
}

void SharedObjectExecutor::emergencySuctionOff() const
{
  std::thread left([this]() { commandSuction(true, false); });
  std::thread right([this]() { commandSuction(false, false); });
  left.join();
  right.join();
}

bool SharedObjectExecutor::execute(
  const BoxSpec& box,
  const PlacementSpec& placement,
  const SharedObjectPlan& plan,
  const PoseProvider& box_pose_provider,
  SharedObjectExecutionResult& result)
{
  result = SharedObjectExecutionResult();
  if (!node_ || !planning_scene_mutex_ || !box_pose_provider || box.id.empty() ||
      placement.object_id != box.id || config_.command_period_sec <= 0.0 ||
      config_.execution_time_scale < 1.0 || config_.grasp_timeout_sec <= 0.0 ||
      config_.release_timeout_sec <= 0.0 || config_.settle_sec < 0.0 ||
      config_.stage_position_tolerance_m <= 0.0 ||
      config_.placement_position_tolerance_m <= 0.0 ||
      config_.placement_orientation_tolerance_rad <= 0.0 ||
      config_.relative_tcp_tolerance_m <= 0.0 ||
      !expectedStages(plan, result.error))
  {
    return false;
  }
  if (!waitForBridge())
  {
    result.error = "Task20 Isaac dual-suction bridge 未就绪。";
    return false;
  }

  const auto execution_start = std::chrono::steady_clock::now();
  geometry_msgs::msg::Point contact_relative_tcp;
  bool have_contact_relative_tcp = false;
  const auto validate_stage = [&](const SharedObjectStage& stage, bool check_relative)
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.final_hold_sec));
    const auto actual_box = box_pose_provider();
    const double position_error = positionDistance(actual_box.position, stage.box_end.position);
    result.max_stage_position_error_m = std::max(result.max_stage_position_error_m, position_error);
    if (position_error > config_.stage_position_tolerance_m)
    {
      result.error = stage.name + " SharedBox Ground Truth position error=" +
        std::to_string(position_error * 1000.0) + " mm exceeds limit";
      return false;
    }
    if (check_relative)
    {
      geometry_msgs::msg::Pose left_tcp;
      geometry_msgs::msg::Pose right_tcp;
      if (!getTcpPose(true, left_tcp) || !getTcpPose(false, right_tcp))
      {
        result.error = stage.name + " 缺少 suction TCP Ground Truth";
        return false;
      }
      const auto relative = subtractPoints(right_tcp.position, left_tcp.position);
      if (!have_contact_relative_tcp)
      {
        contact_relative_tcp = relative;
        have_contact_relative_tcp = true;
      }
      const double relative_error = positionDistance(relative, contact_relative_tcp);
      result.max_relative_tcp_error_m = std::max(result.max_relative_tcp_error_m, relative_error);
      if (relative_error > config_.relative_tcp_tolerance_m)
      {
        result.error = stage.name + " relative TCP error=" +
          std::to_string(relative_error * 1000.0) + " mm exceeds limit";
        return false;
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "Task20-C tight %s METRIC: box_error=%.3f mm%s",
      stage.name.c_str(), position_error * 1000.0,
      check_relative ? ", relative_tcp checked" : "");
    return true;
  };

  RCLCPP_INFO(
    node_->get_logger(),
    "========== Task20-C TIGHT ISAAC EXECUTION: object=%s, stages=%zu, time_scale=%.2f ==========" ,
    box.id.c_str(), plan.stages.size(), config_.execution_time_scale);

  bool success = false;
  do
  {
    const auto& pre_contact = plan.stages[0];
    if (!executeOne(true, pre_contact.left) || !executeOne(false, pre_contact.right))
    {
      result.error = "SEQUENTIAL_PRE_CONTACT joint command 失败。";
      break;
    }
    ++result.stages_executed;

    // DUAL_CONTACT 前只从 MoveIt World 移除当前 SharedBox；物理 Scene 和私有
    // FCL 均保留它。绝不修改全局 ACM，也不把 Box 永久忽略。
    {
      std::lock_guard<std::mutex> lock(*planning_scene_mutex_);
      moveit::planning_interface::PlanningSceneInterface scene;
      scene.removeCollisionObjects({box.id});
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));

    const auto& contact = plan.stages[1];
    if (!executeTogether(contact))
    {
      result.error = "DUAL_CONTACT joint command 失败。";
      break;
    }
    ++result.stages_executed;
    std::thread left_on([this]() { commandSuction(true, true); });
    std::thread right_on([this]() { commandSuction(false, true); });
    left_on.join();
    right_on.join();
    if (!waitForSuction(true, true, config_.grasp_timeout_sec) ||
        !waitForSuction(false, true, config_.grasp_timeout_sec))
    {
      result.error = "DUAL_CONTACT 后双 Surface Gripper 未同时 CLOSED。";
      break;
    }
    if (!validate_stage(contact, true))
    {
      break;
    }

    for (std::size_t index = 2; index <= 4; ++index)
    {
      const auto& stage = plan.stages[index];
      if (!executeTogether(stage))
      {
        result.error = stage.name + " joint command 失败。";
        break;
      }
      ++result.stages_executed;
      if (!validate_stage(stage, true))
      {
        break;
      }
    }
    if (!result.error.empty())
    {
      break;
    }

    std::thread left_off([this]() { commandSuction(true, false); });
    std::thread right_off([this]() { commandSuction(false, false); });
    left_off.join();
    right_off.join();
    if (!waitForSuction(true, false, config_.release_timeout_sec) ||
        !waitForSuction(false, false, config_.release_timeout_sec))
    {
      result.error = "COMMON_DESCENT 后双 Surface Gripper 未同时 OPEN。";
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(config_.settle_sec));
    const auto settled_box = box_pose_provider();
    result.placement_position_error_m = positionDistance(
      settled_box.position, placement.target_pose.position);
    result.placement_orientation_error_rad = quaternionAngularDistance(
      settled_box.orientation, placement.target_pose.orientation);
    if (result.placement_position_error_m > config_.placement_position_tolerance_m ||
        result.placement_orientation_error_rad > config_.placement_orientation_tolerance_rad)
    {
      result.error = "SharedBox release Ground Truth exceeds placement tolerance.";
      break;
    }
    if (!writeWorldObject(box, settled_box))
    {
      result.error = "无法将 release 后 SharedBox Ground Truth 写回 MoveIt World。";
      break;
    }
    if (lock_object_pub_->get_subscription_count() > 0)
    {
      std_msgs::msg::String lock;
      lock.data = box.id;
      for (int repeat = 0; repeat < 3; ++repeat)
      {
        lock_object_pub_->publish(lock);
        std::this_thread::sleep_for(50ms);
      }
    }

    const auto& retreat = plan.stages[5];
    if (!executeTogether(retreat))
    {
      result.error = "PREPLANNED_COMMON_RETREAT joint command 失败。";
      break;
    }
    ++result.stages_executed;
    success = true;
  } while (false);

  if (!success)
  {
    emergencySuctionOff();
    result.wall_duration_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - execution_start).count();
    RCLCPP_ERROR(node_->get_logger(), "Task20-C tight EXECUTION FAIL: %s", result.error.c_str());
    return false;
  }

  result.completed = true;
  result.wall_duration_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - execution_start).count();
  RCLCPP_INFO(
    node_->get_logger(),
    "Task20-C tight EXECUTION PASS: stages=%zu, place=%.3f mm, orientation=%.3f deg, "
    "max_stage=%.3f mm, max_relative_tcp=%.3f mm, wall=%.3f s",
    result.stages_executed, result.placement_position_error_m * 1000.0,
    result.placement_orientation_error_rad * 180.0 / PI,
    result.max_stage_position_error_m * 1000.0, result.max_relative_tcp_error_m * 1000.0,
    result.wall_duration_sec);
  return true;
}

}  // namespace fr3_dual_palletize
