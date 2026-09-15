// Task23-B：二指夹爪“先暂存、后平推”的最小物理执行器。
//
// 左臂复用 Task01 已验证的真实 finger 接触抓取语义：OPEN -> 下探 -> CLOSE
// -> MoveIt ATTACH -> LIFT。右臂不夹持物体；它以闭合后的两块 finger rubber tip
// 组成推送面，在当前二层支撑平面上沿 +X 做受控 Cartesian 直推。
//
// 关键边界：这是固定第一层上的二层推送验证，不能据此宣称动态多层垛整体稳定。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "fr3_dual_palletize/msg/box_state_array.hpp"
#include "fr3_dual_palletize/msg/push_stack_task.hpp"
#include "fr3_dual_palletize/msg/task_world_commit.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kCubeSize = 0.030;
constexpr double kCubeHalf = 0.015;
constexpr double kOpenFinger = 0.040;
constexpr double kGraspFinger = 0.014;
// 推送时两只 finger 均闭合。其 rubber tip 在 local-Y 方向拼成近似 30 mm 推送面。
constexpr double kPushFinger = 0.000;
constexpr double kAttachedCubeTcpZ = 0.009;
constexpr double kFingerTipHalfX = 0.00875;
constexpr double kPrePickTcpZ = 0.160;
constexpr double kGraspTcpOffsetZ = 0.010;
constexpr double kLiftTcpZ = 0.180;
constexpr double kStagePlaceTcpClearance = 0.011;
constexpr double kPushApproachTcpZ = 0.175;
constexpr double kCartesianStep = 0.001;
constexpr double kMinCartesianFraction = 0.999;
constexpr int kMaxPlanAttempts = 3;
constexpr double kStageXyLimit = 0.010;
constexpr double kStageZLimit = 0.010;
constexpr double kFinalXyLimit = 0.010;
constexpr double kFinalZLimit = 0.010;
constexpr double kFinalYawLimitDeg = 5.0;

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  auto seconds_int = static_cast<int32_t>(std::floor(seconds));
  auto nanoseconds = static_cast<int64_t>(std::llround((seconds - seconds_int) * 1e9));
  if (nanoseconds >= 1000000000LL)
  {
    ++seconds_int;
    nanoseconds -= 1000000000LL;
  }
  point.time_from_start.sec = seconds_int;
  point.time_from_start.nanosec = static_cast<uint32_t>(std::max<int64_t>(0, nanoseconds));
}

void ensureTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1e-6)
  {
    return;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index)
  {
    setPointTime(trajectory.points[index], static_cast<double>(index) * 0.035);
  }
}

geometry_msgs::msg::Pose topDownPose(double x, double y, double z)
{
  // R = Rx(pi)：hand TCP +Z 指向世界 -Z，finger local-Y 映射到世界 -Y。
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = 1.0;
  pose.orientation.w = 0.0;
  return pose;
}

double xyError(const geometry_msgs::msg::Pose& actual, const geometry_msgs::msg::Pose& expected)
{
  return std::hypot(actual.position.x - expected.position.x, actual.position.y - expected.position.y);
}

double yawRadians(const geometry_msgs::msg::Pose& pose)
{
  const double siny = 2.0 * (pose.orientation.w * pose.orientation.z +
                             pose.orientation.x * pose.orientation.y);
  const double cosy = 1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
                                   pose.orientation.z * pose.orientation.z);
  return std::atan2(siny, cosy);
}

double shortestAngle(double first, double second)
{
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task23 cannot reach /move_group parameter service.");
    return false;
  }
  const auto parameters = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (parameters.size() != 2 ||
      parameters[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[0].as_string().empty() || parameters[1].as_string().empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Task23 received invalid robot_description / semantic from MoveIt.");
    return false;
  }
  node->declare_parameter<std::string>("robot_description", parameters[0].as_string());
  node->declare_parameter<std::string>("robot_description_semantic", parameters[1].as_string());
  return true;
}

moveit_msgs::msg::CollisionObject makeBox(
  const std::string& id, const geometry_msgs::msg::Pose& pose,
  double size_x = kCubeSize, double size_y = kCubeSize, double size_z = kCubeSize)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size_x, size_y, size_z};
  object.primitives.push_back(box);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

class GroundTruthBuffer
{
public:
  explicit GroundTruthBuffer(const rclcpp::Node::SharedPtr& node)
  {
    subscription_ = node->create_subscription<fr3_dual_palletize::msg::BoxStateArray>(
      "/task23/box_states", 20,
      [this](const fr3_dual_palletize::msg::BoxStateArray::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto& box : message->boxes)
          {
            poses_[box.id] = box.pose;
            ++revisions_[box.id];
          }
        }
        condition_.notify_all();
      });
  }

  bool waitFor(const std::string& object_id, double timeout_sec, geometry_msgs::msg::Pose* result) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received = condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec),
      [this, &object_id]() { return poses_.count(object_id) > 0; });
    if (!received)
    {
      return false;
    }
    *result = poses_.at(object_id);
    return true;
  }

  bool waitForAllSupply(double timeout_sec, std::map<std::string, geometry_msgs::msg::Pose>* result) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received = condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec),
      [this]()
      {
        std::size_t count = 0;
        for (const auto& item : poses_)
        {
          if (item.first.rfind("task23_supply_", 0) == 0)
          {
            ++count;
          }
        }
        return count == 16;
      });
    if (!received)
    {
      return false;
    }
    result->clear();
    for (const auto& item : poses_)
    {
      if (item.first.rfind("task23_supply_", 0) == 0)
      {
        result->insert(item);
      }
    }
    return true;
  }

  uint64_t revision(const std::string& object_id) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = revisions_.find(object_id);
    return found == revisions_.end() ? 0U : found->second;
  }

  // 释放或推送之后必须等待一帧新的 Isaac Ground Truth。不能复用缓存中的抓取前 pose，
  // 否则会把“收到过该对象”误当成“对象已经在当前阶段稳定”。
  bool waitForNew(const std::string& object_id, uint64_t previous_revision,
                  double timeout_sec, geometry_msgs::msg::Pose* result) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received = condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec),
      [this, &object_id, previous_revision]()
      {
        const auto revision = revisions_.find(object_id);
        return revision != revisions_.end() && revision->second > previous_revision &&
               poses_.count(object_id) > 0;
      });
    if (!received)
    {
      return false;
    }
    *result = poses_.at(object_id);
    return true;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  std::map<std::string, geometry_msgs::msg::Pose> poses_;
  std::map<std::string, uint64_t> revisions_;
  rclcpp::Subscription<fr3_dual_palletize::msg::BoxStateArray>::SharedPtr subscription_;
};

class PushTaskBuffer
{
public:
  explicit PushTaskBuffer(const rclcpp::Node::SharedPtr& node)
  {
    rclcpp::QoS qos(1);
    qos.reliable().transient_local();
    subscription_ = node->create_subscription<fr3_dual_palletize::msg::PushStackTask>(
      "/task23/push_task", qos,
      [this](const fr3_dual_palletize::msg::PushStackTask::SharedPtr message)
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = *message;
        }
        condition_.notify_all();
      });
  }

  bool waitFor(std::size_t sequence_index, double timeout_sec,
               fr3_dual_palletize::msg::PushStackTask* result) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool received = condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec),
      [this, sequence_index]()
      {
        return latest_.has_value() && latest_->sequence_index == sequence_index;
      });
    if (!received)
    {
      return false;
    }
    *result = *latest_;
    return true;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  std::optional<fr3_dual_palletize::msg::PushStackTask> latest_;
  rclcpp::Subscription<fr3_dual_palletize::msg::PushStackTask>::SharedPtr subscription_;
};

struct ArmContext
{
  std::string side;
  std::string group_name;
  std::string eef_link;
  std::string finger_joint1;
  std::string finger_joint2;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  const moveit::core::JointModelGroup* joint_group{nullptr};
};

class Task23Executor
{
public:
  explicit Task23Executor(const rclcpp::Node::SharedPtr& node)
  : node_(node), ground_truth_(node), tasks_(node)
  {
    left_.side = "left";
    left_.group_name = "left_arm";
    left_.eef_link = "left_fr3_hand_tcp";
    left_.finger_joint1 = "left_fr3_finger_joint1";
    left_.finger_joint2 = "left_fr3_finger_joint2";
    left_.command_publisher = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/left/joint_command", 20);
    right_.side = "right";
    right_.group_name = "right_arm";
    right_.eef_link = "right_fr3_hand_tcp";
    right_.finger_joint1 = "right_fr3_finger_joint1";
    right_.finger_joint2 = "right_fr3_finger_joint2";
    right_.command_publisher = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/right/joint_command", 20);
    commit_publisher_ = node_->create_publisher<fr3_dual_palletize::msg::TaskWorldCommit>(
      "/task23/world_commit", 20);
  }

  bool run(std::size_t max_tasks, double time_scale)
  {
    time_scale_ = std::max(1.0, time_scale);
    if (!createMoveGroups())
    {
      return false;
    }
    if (!waitForIsaacSubscribers())
    {
      return false;
    }
    std::map<std::string, geometry_msgs::msg::Pose> supply;
    if (!ground_truth_.waitForAllSupply(15.0, &supply))
    {
      RCLCPP_ERROR(node_->get_logger(), "Task23 did not receive 16 supply Cube Ground Truth states.");
      return false;
    }
    if (!initializeWorld(supply))
    {
      return false;
    }

    RCLCPP_INFO(node_->get_logger(),
      "========== Task23 GRIPPER STAGE-AND-PUSH START: tasks=%zu time_scale=%.2f ==========" ,
      max_tasks, time_scale_);
    for (std::size_t index = 0; index < max_tasks; ++index)
    {
      fr3_dual_palletize::msg::PushStackTask task;
      if (!tasks_.waitFor(index, 30.0, &task))
      {
        RCLCPP_ERROR(node_->get_logger(), "Task23 timeout waiting for selector task index=%zu.", index);
        return false;
      }
      if (task.supply_arm != "left" || task.push_arm != "right" || task.push_distance <= 0.0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Task23 selector task %s violates fixed prototype contract.",
          task.task_id.c_str());
        return false;
      }
      if (!executeOne(task))
      {
        RCLCPP_ERROR(node_->get_logger(),
          "Task23 STOP: task=%s object=%s failed; no later Cube will be commanded.",
          task.task_id.c_str(), task.object_id.c_str());
        return false;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
      "Task23 PASS: %zu/%zu selected Cube completed stage-and-push with Ground Truth commit.",
      max_tasks, max_tasks);
    return true;
  }

private:
  bool createMoveGroups()
  {
    left_.move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, left_.group_name);
    right_.move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, right_.group_name);
    for (ArmContext* arm : {&left_, &right_})
    {
      arm->move_group->setPlannerId("RRTConnectkConfigDefault");
      arm->move_group->setPlanningTime(5.0);
      arm->move_group->setNumPlanningAttempts(3);
      arm->move_group->setMaxVelocityScalingFactor(0.15);
      arm->move_group->setMaxAccelerationScalingFactor(0.15);
      arm->move_group->setPoseReferenceFrame("world");
      arm->move_group->setEndEffectorLink(arm->eef_link);
      arm->joint_group = arm->move_group->getRobotModel()->getJointModelGroup(arm->group_name);
      if (arm->joint_group == nullptr)
      {
        RCLCPP_ERROR(node_->get_logger(), "Task23 missing MoveIt group %s.", arm->group_name.c_str());
        return false;
      }
    }
    return true;
  }

  bool waitForIsaacSubscribers() const
  {
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (left_.command_publisher->get_subscription_count() > 0 &&
          right_.command_publisher->get_subscription_count() > 0)
      {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_ERROR(node_->get_logger(),
      "Task23 no Isaac subscriber on /left/joint_command or /right/joint_command; click Play first.");
    return false;
  }

  bool initializeWorld(const std::map<std::string, geometry_msgs::msg::Pose>& supply)
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.reserve(18 + supply.size());
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.55;
    table_pose.position.z = 0.025;
    table_pose.orientation.w = 1.0;
    objects.push_back(makeBox("task23_table", table_pose, 1.20, 0.80, 0.050));
    geometry_msgs::msg::Pose stage_pose;
    stage_pose.position.x = 0.565;
    stage_pose.position.z = 0.065;
    stage_pose.orientation.w = 1.0;
    objects.push_back(makeBox("task23_loading_stage", stage_pose, 0.070, 0.150, 0.030));
    const std::vector<double> grid_x{0.615, 0.645, 0.675, 0.705};
    const std::vector<double> grid_y{-0.045, -0.015, 0.015, 0.045};
    for (std::size_t row = 0; row < grid_y.size(); ++row)
    {
      for (std::size_t column = 0; column < grid_x.size(); ++column)
      {
        auto pose = topDownPose(grid_x[column], grid_y[row], 0.065);
        pose.orientation.x = 0.0;
        pose.orientation.w = 1.0;
        objects.push_back(makeBox("task23_base_" + std::to_string(row) + "_" +
          std::to_string(column), pose));
      }
    }
    for (const auto& item : supply)
    {
      objects.push_back(makeBox(item.first, item.second));
    }
    if (!scene.applyCollisionObjects(objects))
    {
      RCLCPP_ERROR(node_->get_logger(), "Task23 failed to initialize MoveIt planning world.");
      return false;
    }
    std::this_thread::sleep_for(500ms);
    return true;
  }

  moveit::core::RobotState stateFor(
    ArmContext& arm, const std::vector<double>& joints, double finger) const
  {
    moveit::core::RobotState state(arm.move_group->getRobotModel());
    state.setToDefaultValues();
    state.setJointGroupPositions(arm.joint_group, joints);
    state.setVariablePosition(arm.finger_joint1, finger);
    state.setVariablePosition(arm.finger_joint2, finger);
    state.update();
    return state;
  }

  bool currentJoints(ArmContext& arm, std::vector<double>* joints) const
  {
    const auto state = arm.move_group->getCurrentState(3.0);
    if (!state)
    {
      RCLCPP_ERROR(node_->get_logger(), "Task23 %s did not receive current joint state.", arm.side.c_str());
      return false;
    }
    state->copyJointGroupPositions(arm.joint_group, *joints);
    return joints->size() == arm.joint_group->getVariableCount();
  }

  bool planPose(ArmContext& arm, const std::vector<double>& start, double finger,
                const geometry_msgs::msg::Pose& target, const std::string& label,
                trajectory_msgs::msg::JointTrajectory* output)
  {
    for (int attempt = 1; attempt <= kMaxPlanAttempts; ++attempt)
    {
      const auto start_state = stateFor(arm, start, finger);
      if (!start_state.satisfiesBounds())
      {
        RCLCPP_ERROR(node_->get_logger(), "%s start state violates bounds.", label.c_str());
        return false;
      }
      arm.move_group->setStartState(start_state);
      arm.move_group->clearPoseTargets();
      if (!arm.move_group->setPoseTarget(target, arm.eef_link))
      {
        RCLCPP_ERROR(node_->get_logger(), "%s pose target rejected.", label.c_str());
        return false;
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm.move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
      {
        *output = plan.trajectory_.joint_trajectory;
        RCLCPP_INFO(node_->get_logger(), "%s RRTConnect plan OK attempt=%d/%d points=%zu.",
          label.c_str(), attempt, kMaxPlanAttempts, output->points.size());
        return true;
      }
      RCLCPP_WARN(node_->get_logger(), "%s RRTConnect failed attempt=%d/%d.",
        label.c_str(), attempt, kMaxPlanAttempts);
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool planCartesian(ArmContext& arm, const std::vector<double>& start, double finger,
                     const geometry_msgs::msg::Pose& target, const std::string& label,
                     trajectory_msgs::msg::JointTrajectory* output)
  {
    for (int attempt = 1; attempt <= kMaxPlanAttempts; ++attempt)
    {
      arm.move_group->setStartState(stateFor(arm, start, finger));
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      moveit_msgs::msg::MoveItErrorCodes error;
      const double fraction = arm.move_group->computeCartesianPath(
        std::vector<geometry_msgs::msg::Pose>{target}, kCartesianStep, 0.0,
        robot_trajectory, true, &error);
      RCLCPP_INFO(node_->get_logger(), "%s Cartesian fraction=%.4f error=%d attempt=%d/%d.",
        label.c_str(), fraction, error.val, attempt, kMaxPlanAttempts);
      if (fraction >= kMinCartesianFraction && !robot_trajectory.joint_trajectory.points.empty())
      {
        *output = robot_trajectory.joint_trajectory;
        ensureTiming(*output);
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  void commandFingers(ArmContext& arm, double position, double duration_sec) const
  {
    const int cycles = std::max(1, static_cast<int>(duration_sec / 0.01));
    for (int index = 0; index < cycles; ++index)
    {
      sensor_msgs::msg::JointState command;
      command.header.stamp = node_->now();
      command.name = {arm.finger_joint1, arm.finger_joint2};
      command.position = {position, position};
      arm.command_publisher->publish(command);
      std::this_thread::sleep_for(10ms);
    }
  }

  void publishArm(ArmContext& arm, const std::vector<std::string>& names,
                  const std::vector<double>& positions, double finger) const
  {
    sensor_msgs::msg::JointState command;
    command.header.stamp = node_->now();
    command.name = names;
    command.position = positions;
    command.name.push_back(arm.finger_joint1);
    command.name.push_back(arm.finger_joint2);
    command.position.push_back(finger);
    command.position.push_back(finger);
    arm.command_publisher->publish(command);
  }

  void execute(ArmContext& arm, const trajectory_msgs::msg::JointTrajectory& trajectory,
               double finger) const
  {
    if (trajectory.points.empty())
    {
      return;
    }
    const double total = pointTime(trajectory.points.back());
    std::size_t segment = 0;
    const auto begin = std::chrono::steady_clock::now();
    while (true)
    {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      const double source_time = elapsed / time_scale_;
      if (source_time > total)
      {
        break;
      }
      while (segment + 1 < trajectory.points.size() &&
             pointTime(trajectory.points[segment + 1]) < source_time)
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
        const double alpha = second_time > first_time ?
          std::clamp((source_time - first_time) / (second_time - first_time), 0.0, 1.0) : 0.0;
        positions.resize(first.positions.size());
        for (std::size_t joint = 0; joint < positions.size(); ++joint)
        {
          positions[joint] = first.positions[joint] + alpha * (second.positions[joint] - first.positions[joint]);
        }
      }
      publishArm(arm, trajectory.joint_names, positions, finger);
      std::this_thread::sleep_for(10ms);
    }
    for (int index = 0; index < 50; ++index)
    {
      publishArm(arm, trajectory.joint_names, trajectory.points.back().positions, finger);
      std::this_thread::sleep_for(10ms);
    }
  }

  bool removeWorldObject(const std::string& id) const
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    scene.removeCollisionObjects({id});
    std::this_thread::sleep_for(250ms);
    return true;
  }

  bool addWorldObject(const std::string& id, const geometry_msgs::msg::Pose& pose) const
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    if (!scene.applyCollisionObject(makeBox(id, pose)))
    {
      return false;
    }
    std::this_thread::sleep_for(250ms);
    return true;
  }

  bool attach(ArmContext& arm, const std::string& id) const
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = arm.eef_link;
    attached.touch_links = {arm.eef_link,
      arm.side + "_fr3_hand", arm.side + "_fr3_leftfinger", arm.side + "_fr3_rightfinger"};
    attached.object.header.frame_id = arm.eef_link;
    attached.object.id = id;
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {kCubeSize, kCubeSize, kCubeSize};
    geometry_msgs::msg::Pose relative;
    relative.position.z = kAttachedCubeTcpZ;
    relative.orientation.x = 1.0;
    attached.object.primitives.push_back(box);
    attached.object.primitive_poses.push_back(relative);
    attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene.applyAttachedCollisionObject(attached))
    {
      return false;
    }
    std::this_thread::sleep_for(300ms);
    return true;
  }

  bool detach(ArmContext& arm, const std::string& id) const
  {
    moveit::planning_interface::PlanningSceneInterface scene;
    moveit_msgs::msg::AttachedCollisionObject attached;
    attached.link_name = arm.eef_link;
    attached.object.id = id;
    attached.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    if (!scene.applyAttachedCollisionObject(attached))
    {
      return false;
    }
    std::this_thread::sleep_for(300ms);
    return true;
  }

  bool finalStagePose(const std::string& label, const geometry_msgs::msg::Pose& actual,
                      const geometry_msgs::msg::Pose& expected) const
  {
    const double xy = xyError(actual, expected);
    const double z = std::abs(actual.position.z - expected.position.z);
    if (xy > kStageXyLimit || z > kStageZLimit)
    {
      RCLCPP_ERROR(node_->get_logger(),
        "%s stage Ground Truth exceeded: actual=(%.4f, %.4f, %.4f), xy=%.3f mm z=%.3f mm.",
        label.c_str(), actual.position.x, actual.position.y, actual.position.z, xy * 1000.0, z * 1000.0);
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "%s stage Ground Truth PASS: xy=%.3f mm z=%.3f mm.",
      label.c_str(), xy * 1000.0, z * 1000.0);
    return true;
  }

  bool finalTargetPose(const std::string& label, const geometry_msgs::msg::Pose& actual,
                       const geometry_msgs::msg::Pose& expected) const
  {
    const double xy = xyError(actual, expected);
    const double z = std::abs(actual.position.z - expected.position.z);
    const double yaw_deg = std::abs(shortestAngle(yawRadians(actual), yawRadians(expected))) * 180.0 / M_PI;
    if (xy > kFinalXyLimit || z > kFinalZLimit || yaw_deg > kFinalYawLimitDeg)
    {
      RCLCPP_ERROR(node_->get_logger(),
        "%s final Ground Truth exceeded: actual=(%.4f, %.4f, %.4f), xy=%.3f mm z=%.3f mm yaw=%.3f deg.",
        label.c_str(), actual.position.x, actual.position.y, actual.position.z,
        xy * 1000.0, z * 1000.0, yaw_deg);
      return false;
    }
    RCLCPP_INFO(node_->get_logger(),
      "%s final Ground Truth PASS: xy=%.3f mm z=%.3f mm yaw=%.3f deg.",
      label.c_str(), xy * 1000.0, z * 1000.0, yaw_deg);
    return true;
  }

  bool executeOne(const fr3_dual_palletize::msg::PushStackTask& task)
  {
    RCLCPP_INFO(node_->get_logger(), "========== %s / %s ==========" ,
      task.task_id.c_str(), task.object_id.c_str());
    geometry_msgs::msg::Pose source;
    if (!ground_truth_.waitFor(task.object_id, 3.0, &source))
    {
      RCLCPP_ERROR(node_->get_logger(), "No Ground Truth pose for %s.", task.object_id.c_str());
      return false;
    }

    // ----- 左臂：Task01 同语义的侧夹抓取、抬升与二层暂存 -----
    std::vector<double> left_start;
    if (!currentJoints(left_, &left_start))
    {
      return false;
    }
    trajectory_msgs::msg::JointTrajectory left_pre_pick;
    if (!planPose(left_, left_start, kOpenFinger,
                  topDownPose(source.position.x, source.position.y, kPrePickTcpZ),
                  task.task_id + " LEFT PRE_PICK", &left_pre_pick))
    {
      return false;
    }
    execute(left_, left_pre_pick, kOpenFinger);
    commandFingers(left_, kOpenFinger, 0.4);
    removeWorldObject(task.object_id);

    trajectory_msgs::msg::JointTrajectory left_contact;
    if (!planCartesian(left_, left_pre_pick.points.back().positions, kOpenFinger,
                       topDownPose(source.position.x, source.position.y,
                                   source.position.z + kGraspTcpOffsetZ),
                       task.task_id + " LEFT GRASP", &left_contact))
    {
      return false;
    }
    execute(left_, left_contact, kOpenFinger);
    commandFingers(left_, kGraspFinger, 0.8);
    std::this_thread::sleep_for(500ms);
    if (!attach(left_, task.object_id))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s MoveIt attach failed.", task.task_id.c_str());
      return false;
    }

    trajectory_msgs::msg::JointTrajectory left_lift;
    if (!planCartesian(left_, left_contact.points.back().positions, kGraspFinger,
                       topDownPose(source.position.x, source.position.y, kLiftTcpZ),
                       task.task_id + " LEFT LIFT", &left_lift))
    {
      return false;
    }
    execute(left_, left_lift, kGraspFinger);

    trajectory_msgs::msg::JointTrajectory left_transfer;
    if (!planPose(left_, left_lift.points.back().positions, kGraspFinger,
                  topDownPose(task.staging_pose.position.x, task.staging_pose.position.y, kLiftTcpZ),
                  task.task_id + " LEFT TRANSFER_TO_STAGE", &left_transfer))
    {
      return false;
    }
    execute(left_, left_transfer, kGraspFinger);

    trajectory_msgs::msg::JointTrajectory left_place;
    if (!planCartesian(left_, left_transfer.points.back().positions, kGraspFinger,
                       topDownPose(task.staging_pose.position.x, task.staging_pose.position.y,
                                   task.staging_pose.position.z + kStagePlaceTcpClearance),
                       task.task_id + " LEFT STAGE_DESCENT", &left_place))
    {
      return false;
    }
    execute(left_, left_place, kGraspFinger);

    // 在物理释放前预规划上撤：此时 Cube 仍是 AttachedCollisionObject，避免接触起点
    // 被误判碰撞。计划完成后才 OPEN、detach 与读取真实暂存 pose。
    trajectory_msgs::msg::JointTrajectory left_stage_retreat;
    if (!planCartesian(left_, left_place.points.back().positions, kGraspFinger,
                       topDownPose(task.staging_pose.position.x, task.staging_pose.position.y, kLiftTcpZ),
                       task.task_id + " LEFT PREPLANNED_STAGE_RETREAT", &left_stage_retreat))
    {
      return false;
    }
    const uint64_t stage_release_revision = ground_truth_.revision(task.object_id);
    commandFingers(left_, kOpenFinger, 0.8);
    std::this_thread::sleep_for(900ms);
    if (!detach(left_, task.object_id))
    {
      RCLCPP_ERROR(node_->get_logger(), "%s MoveIt detach after stage failed.", task.task_id.c_str());
      return false;
    }
    geometry_msgs::msg::Pose staged;
    if (!ground_truth_.waitForNew(task.object_id, stage_release_revision, 2.0, &staged) ||
        !finalStagePose(task.task_id, staged, task.staging_pose))
    {
      return false;
    }
    execute(left_, left_stage_retreat, kOpenFinger);
    if (!addWorldObject(task.object_id, staged))
    {
      return false;
    }

    // ----- 右臂：闭合双 finger tip 形成后侧推送面；不抓取、不附着当前 Cube -----
    commandFingers(right_, kPushFinger, 0.5);
    std::vector<double> right_start;
    if (!currentJoints(right_, &right_start))
    {
      return false;
    }
    const double push_start_x = staged.position.x - kCubeHalf - kFingerTipHalfX - 0.003;
    const double push_end_x = task.target_pose.position.x - kCubeHalf - kFingerTipHalfX;
    trajectory_msgs::msg::JointTrajectory push_high;
    if (!planPose(right_, right_start, kPushFinger,
                  topDownPose(push_start_x, staged.position.y, kPushApproachTcpZ),
                  task.task_id + " RIGHT PUSH_APPROACH", &push_high))
    {
      return false;
    }
    execute(right_, push_high, kPushFinger);
    trajectory_msgs::msg::JointTrajectory push_contact;
    if (!planCartesian(right_, push_high.points.back().positions, kPushFinger,
                       // 与已验证的单臂夹爪抓取一致：手 TCP 必须比 Cube 中心高 10 mm。
                       // 若直接使用 staged.position.z，finger 会下探到暂存台内，
                       // 既不代表有效侧向推送接触，也会造成虚假的台面碰撞。
                       topDownPose(push_start_x, staged.position.y,
                                   staged.position.z + kGraspTcpOffsetZ),
                       task.task_id + " RIGHT PUSH_CONTACT", &push_contact))
    {
      return false;
    }
    execute(right_, push_contact, kPushFinger);

    // 只临时移除“当前被推 Cube”；所有固定底层及已提交二层 Cube 仍参与碰撞检查。
    // 不扩大 ACM，也不永久忽略任何 Cube-robot 碰撞。
    removeWorldObject(task.object_id);
    trajectory_msgs::msg::JointTrajectory push;
    if (!planCartesian(right_, push_contact.points.back().positions, kPushFinger,
                       topDownPose(push_end_x, staged.position.y,
                                   staged.position.z + kGraspTcpOffsetZ),
                       task.task_id + " RIGHT CONTROLLED_PUSH", &push))
    {
      return false;
    }
    trajectory_msgs::msg::JointTrajectory push_retreat;
    if (!planCartesian(right_, push.points.back().positions, kPushFinger,
                       topDownPose(push_start_x - 0.020, staged.position.y,
                                   staged.position.z + kGraspTcpOffsetZ),
                       task.task_id + " RIGHT PREPLANNED_PUSH_RETREAT", &push_retreat))
    {
      return false;
    }
    const uint64_t push_revision = ground_truth_.revision(task.object_id);
    execute(right_, push, kPushFinger);
    std::this_thread::sleep_for(900ms);
    geometry_msgs::msg::Pose settled;
    if (!ground_truth_.waitForNew(task.object_id, push_revision, 2.0, &settled) ||
        !finalTargetPose(task.task_id, settled, task.target_pose))
    {
      // 当前 Cube 仍不在 MoveIt World，避免以错误 pose 继续后续规划；立即终止。
      return false;
    }
    execute(right_, push_retreat, kPushFinger);
    commandFingers(right_, kOpenFinger, 0.3);
    if (!addWorldObject(task.object_id, settled))
    {
      return false;
    }

    fr3_dual_palletize::msg::TaskWorldCommit commit;
    commit.header.stamp = node_->now();
    commit.header.frame_id = "world";
    commit.source_seed = 20260923;
    commit.completed_scene_version = task.scene_version;
    commit.object_id = task.object_id;
    commit.settled_pose = settled;
    commit.coordination_mode = "GRIPPER_STAGE_AND_PUSH";
    commit.executed_arm = "left_supply_right_push";
    commit_publisher_->publish(commit);
    RCLCPP_INFO(node_->get_logger(),
      "%s PASS: gripper pickup -> stage -> single-arm +X push -> Ground Truth commit.",
      task.task_id.c_str());
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  GroundTruthBuffer ground_truth_;
  PushTaskBuffer tasks_;
  ArmContext left_;
  ArmContext right_;
  rclcpp::Publisher<fr3_dual_palletize::msg::TaskWorldCommit>::SharedPtr commit_publisher_;
  double time_scale_{1.0};
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task23_gripper_stage_push");
  const int requested_tasks = node->declare_parameter<int>("max_tasks", 16);
  const double time_scale = node->declare_parameter<double>("execution_time_scale", 1.5);
  if (requested_tasks < 1 || requested_tasks > 16)
  {
    RCLCPP_ERROR(node->get_logger(), "max_tasks must be within [1, 16].");
    rclcpp::shutdown();
    return 1;
  }
  if (!copyRobotModelParameters(node))
  {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  bool success = false;
  try
  {
    Task23Executor task_executor(node);
    success = task_executor.run(static_cast<std::size_t>(requested_tasks), time_scale);
  }
  catch (const std::exception& exception)
  {
    RCLCPP_FATAL(node->get_logger(), "Task23 unhandled exception: %s", exception.what());
  }
  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
