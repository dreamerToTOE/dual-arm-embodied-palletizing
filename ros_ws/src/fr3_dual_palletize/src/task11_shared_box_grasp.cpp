#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double BOX_X = 0.220;
constexpr double BOX_Y = 0.320;
constexpr double BOX_Z = 0.080;
constexpr double SUCTION_TCP_OFFSET_Z = 0.105;
constexpr double CONTACT_CLEARANCE_Z = 0.001;
constexpr double PRE_CONTACT_CLEARANCE_Z = 0.065;
constexpr double CARTESIAN_EEF_STEP = 0.002;
constexpr double CARTESIAN_MIN_FRACTION = 0.999;

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(
  trajectory_msgs::msg::JointTrajectoryPoint& point,
  double seconds)
{
  auto sec = static_cast<std::int32_t>(std::floor(seconds));
  auto nanosec = static_cast<std::int64_t>(std::llround(
    (seconds - static_cast<double>(sec)) * 1e9));
  if (nanosec >= 1000000000LL)
  {
    ++sec;
    nanosec -= 1000000000LL;
  }
  point.time_from_start.sec = sec;
  point.time_from_start.nanosec = static_cast<std::uint32_t>(nanosec);
}

void ensureTiming(trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1e-6)
  {
    return;
  }
  for (std::size_t index = 0; index < trajectory.points.size(); ++index)
  {
    setPointTime(trajectory.points[index], 0.03 * static_cast<double>(index));
  }
}

std::vector<double> finalPositions(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  return trajectory.points.empty() ? std::vector<double>{} : trajectory.points.back().positions;
}

geometry_msgs::msg::Pose topDownPose(
  double x,
  double y,
  double suction_tcp_z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = suction_tcp_z + SUCTION_TCP_OFFSET_Z;
  pose.orientation.x = 1.0;
  pose.orientation.w = 0.0;
  return pose;
}

moveit_msgs::msg::CollisionObject sharedBoxObject(
  const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "task11_shared_box";
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {BOX_X, BOX_Y, BOX_Z};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

bool copyRobotModelParameters(const rclcpp::Node::SharedPtr& node)
{
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "无法连接 /move_group。请先启动 Task06 双臂 MoveIt2。");
    return false;
  }
  const auto values = client->get_parameters({
    "robot_description", "robot_description_semantic"});
  if (values.size() != 2 ||
      values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      values[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "无法从 /move_group 读取 RobotModel 参数。");
    return false;
  }
  node->declare_parameter<std::string>("robot_description", values[0].as_string());
  node->declare_parameter<std::string>(
    "robot_description_semantic", values[1].as_string());
  return true;
}

class SharedBoxPoseBuffer
{
public:
  explicit SharedBoxPoseBuffer(const rclcpp::Node::SharedPtr& node)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/task11/shared_box_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = message->pose;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(
      lock, std::chrono::duration<double>(timeout_sec), [this]() { return ready_; });
  }

  geometry_msgs::msg::Pose get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool ready_{false};
  geometry_msgs::msg::Pose pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

class ArmControl
{
public:
  ArmControl(const rclcpp::Node::SharedPtr& node, bool left)
    : node_(node),
      side_(left ? "left" : "right"),
      group_name_(left ? "left_arm" : "right_arm"),
      eef_link_(left ? "left_fr3_link8" : "right_fr3_link8"),
      joint_prefix_(left ? "left_" : "right_")
  {
    command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/" + side_ + "/joint_command", 10);
    suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/task11/" + side_ + "/suction_command", 10);
    state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task11/" + side_ + "/suction_state", 10,
      [this](const std_msgs::msg::Bool::SharedPtr message)
      {
        have_suction_state_.store(true);
        suction_closed_.store(message->data);
      });
  }

  bool waitForBridge() const
  {
    for (int attempt = 0; attempt < 100; ++attempt)
    {
      if (command_pub_->get_subscription_count() > 0 &&
          suction_pub_->get_subscription_count() > 0 && have_suction_state_.load())
      {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool planPose(
    moveit::planning_interface::MoveGroupInterface& group,
    const geometry_msgs::msg::Pose& target,
    trajectory_msgs::msg::JointTrajectory& output) const
  {
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(5);
    group.setMaxVelocityScalingFactor(0.15);
    group.setMaxAccelerationScalingFactor(0.15);
    group.setPoseReferenceFrame("world");
    group.setEndEffectorLink(eef_link_);
    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    if (!group.setPoseTarget(target, eef_link_))
    {
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS ||
        plan.trajectory_.joint_trajectory.points.empty())
    {
      return false;
    }
    output = plan.trajectory_.joint_trajectory;
    ensureTiming(output);
    return true;
  }

  bool planCartesianContact(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target,
    trajectory_msgs::msg::JointTrajectory& output) const
  {
    const auto* jmg = group.getRobotModel()->getJointModelGroup(group_name_);
    auto state = group.getCurrentState(2.0);
    if (jmg == nullptr || !state || start_q.empty())
    {
      return false;
    }
    state->setJointGroupPositions(jmg, start_q);
    state->update();
    group.setStartState(*state);
    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction = group.computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target}, CARTESIAN_EEF_STEP, 0.0,
      robot_trajectory, true, &error);
    RCLCPP_INFO(
      node_->get_logger(),
      "Task11 %s CONTACT Cartesian fraction=%.4f, error=%d",
      side_.c_str(), fraction, error.val);
    if (fraction < CARTESIAN_MIN_FRACTION || robot_trajectory.joint_trajectory.points.empty())
    {
      return false;
    }
    output = robot_trajectory.joint_trajectory;
    ensureTiming(output);
    return true;
  }

  bool execute(const trajectory_msgs::msg::JointTrajectory& input) const
  {
    if (input.joint_names.empty() || input.points.empty())
    {
      return false;
    }
    auto trajectory = input;
    ensureTiming(trajectory);
    std::vector<std::string> names;
    for (const auto& name : trajectory.joint_names)
    {
      names.push_back(name.rfind(joint_prefix_, 0) == 0 ?
        name.substr(joint_prefix_.size()) : name);
    }
    const auto start = std::chrono::steady_clock::now();
    std::size_t segment = 0;
    const auto publish = [&](const std::vector<double>& positions)
    {
      sensor_msgs::msg::JointState message;
      message.header.stamp = node_->now();
      message.name = names;
      message.position = positions;
      command_pub_->publish(message);
    };
    const double duration = pointTime(trajectory.points.back());
    while (true)
    {
      const double time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      if (time > duration)
      {
        break;
      }
      while (segment + 1 < trajectory.points.size() &&
             pointTime(trajectory.points[segment + 1]) < time)
      {
        ++segment;
      }
      const auto& first = trajectory.points[segment];
      const auto& second = trajectory.points[
        std::min(segment + 1, trajectory.points.size() - 1)];
      const double start_time = pointTime(first);
      const double end_time = pointTime(second);
      const double alpha = end_time > start_time ? std::clamp(
        (time - start_time) / (end_time - start_time), 0.0, 1.0) : 0.0;
      std::vector<double> positions(first.positions.size());
      for (std::size_t index = 0; index < positions.size(); ++index)
      {
        positions[index] = first.positions[index] +
          alpha * (second.positions[index] - first.positions[index]);
      }
      publish(positions);
      std::this_thread::sleep_for(10ms);
    }
    for (int repeat = 0; repeat < 50; ++repeat)
    {
      publish(trajectory.points.back().positions);
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void commandSuction(bool on) const
  {
    std_msgs::msg::Bool message;
    message.data = on;
    for (int repeat = 0; repeat < 20; ++repeat)
    {
      suction_pub_->publish(message);
      std::this_thread::sleep_for(10ms);
    }
  }

  bool waitForSuction(bool expected, double timeout_sec) const
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

  const std::string& groupName() const { return group_name_; }
  const std::string& side() const { return side_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string side_;
  std::string group_name_;
  std::string eef_link_;
  std::string joint_prefix_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  std::atomic_bool have_suction_state_{false};
  std::atomic_bool suction_closed_{false};
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task11_shared_box_grasp");
  const bool execute = node->declare_parameter<bool>("execute", false);
  const double grasp_timeout = node->declare_parameter<double>("grasp_timeout_sec", 3.0);

  if (!copyRobotModelParameters(node) || grasp_timeout <= 0.0)
  {
    rclcpp::shutdown();
    return 1;
  }
  auto pose_buffer = std::make_shared<SharedBoxPoseBuffer>(node);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  bool success = false;
  do
  {
    RCLCPP_INFO(node->get_logger(), "========== Task11 SHARED-BOX DUAL-SUCTION ==========");
    if (!pose_buffer->wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "等待 /task11/shared_box_pose 超时。");
      break;
    }
    ArmControl left(node, true);
    ArmControl right(node, false);
    if (!left.waitForBridge() || !right.waitForBridge())
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 Isaac bridge 未就绪。");
      break;
    }

    const auto box_pose = pose_buffer->get();
    moveit::planning_interface::PlanningSceneInterface scene;
    scene.removeCollisionObjects({"task11_shared_box"});
    std::this_thread::sleep_for(250ms);
    if (!scene.applyCollisionObject(sharedBoxObject(box_pose)))
    {
      RCLCPP_ERROR(node->get_logger(), "无法将 SharedBox 加入 MoveIt World。");
      break;
    }
    std::this_thread::sleep_for(300ms);

    const double contact_z = box_pose.position.z + 0.5 * BOX_Z + CONTACT_CLEARANCE_Z;
    const auto left_contact = topDownPose(
      box_pose.position.x, box_pose.position.y - 0.100, contact_z);
    const auto right_contact = topDownPose(
      box_pose.position.x, box_pose.position.y + 0.100, contact_z);
    const auto left_pre = topDownPose(
      box_pose.position.x, box_pose.position.y - 0.100,
      contact_z + PRE_CONTACT_CLEARANCE_Z);
    const auto right_pre = topDownPose(
      box_pose.position.x, box_pose.position.y + 0.100,
      contact_z + PRE_CONTACT_CLEARANCE_Z);

    moveit::planning_interface::MoveGroupInterface left_group(node, left.groupName());
    moveit::planning_interface::MoveGroupInterface right_group(node, right.groupName());
    trajectory_msgs::msg::JointTrajectory left_pre_traj;
    trajectory_msgs::msg::JointTrajectory right_pre_traj;
    if (!left.planPose(left_group, left_pre, left_pre_traj) ||
        !right.planPose(right_group, right_pre, right_pre_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 PRE_CONTACT 规划失败。");
      break;
    }

    if (execute)
    {
      // 右臂的 pre-contact 规划必须在左臂到位后重新生成，使 MoveIt 使用
      // 左臂真实静止状态，而非预检时的 HOME 状态。
      if (!left.execute(left_pre_traj) ||
          !right.planPose(right_group, right_pre, right_pre_traj) ||
          !right.execute(right_pre_traj))
      {
        RCLCPP_ERROR(node->get_logger(), "Task11 PRE_CONTACT 执行失败。");
        break;
      }
      RCLCPP_INFO(node->get_logger(), "Task11 两臂均到达 PRE_CONTACT。");
    }

    // CONTACT 是有意接触，故仅在两条已完成的预接触轨迹之间临时移除 SharedBox。
    // 本 Task 不进行后续 MoveIt 规划；Task12 将以共同 AttachedBody 模型恢复完整碰撞语义。
    scene.removeCollisionObjects({"task11_shared_box"});
    std::this_thread::sleep_for(250ms);
    trajectory_msgs::msg::JointTrajectory left_contact_traj;
    trajectory_msgs::msg::JointTrajectory right_contact_traj;
    if (!left.planCartesianContact(
          left_group, finalPositions(left_pre_traj), left_contact, left_contact_traj) ||
        !right.planCartesianContact(
          right_group, finalPositions(right_pre_traj), right_contact, right_contact_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 CONTACT Cartesian 规划失败。");
      break;
    }

    if (!execute)
    {
      RCLCPP_INFO(node->get_logger(), "Task11 PRECHECK PASS：未发布任何 joint / suction 命令。");
      success = true;
      break;
    }

    std::thread left_contact_thread([&left, &left_contact_traj]() {
      left.execute(left_contact_traj);
    });
    std::thread right_contact_thread([&right, &right_contact_traj]() {
      right.execute(right_contact_traj);
    });
    left_contact_thread.join();
    right_contact_thread.join();

    std::thread left_suction_thread([&left]() { left.commandSuction(true); });
    std::thread right_suction_thread([&right]() { right.commandSuction(true); });
    left_suction_thread.join();
    right_suction_thread.join();

    const bool left_closed = left.waitForSuction(true, grasp_timeout);
    const bool right_closed = right.waitForSuction(true, grasp_timeout);
    if (!left_closed || !right_closed)
    {
      RCLCPP_ERROR(
        node->get_logger(), "Task11 FAIL：left_closed=%s, right_closed=%s",
        left_closed ? "true" : "false", right_closed ? "true" : "false");
      left.commandSuction(false);
      right.commandSuction(false);
      break;
    }

    const auto settled = pose_buffer->get();
    RCLCPP_INFO(
      node->get_logger(),
      "Task11 PASS：双 Surface Gripper 同时 CLOSED；SharedBox=(%.4f, %.4f, %.4f)。",
      settled.position.x, settled.position.y, settled.position.z);
    success = true;
  } while (false);

  executor.cancel();
  if (spin_thread.joinable())
  {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
