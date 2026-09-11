#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
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
constexpr double SYNCHRONIZED_COLLISION_SAMPLE_PERIOD_SEC = 0.010;
// 对随机采样规划及瞬时 Planning Scene 更新做显式外层重试。
constexpr int MAX_PLANNING_RETRIES = 3;

// 大件需要双吸盘落在顶面内侧。Task15 使用 105 mm 对称偏置：它比
// Task11--13 已验收的 100 mm 稍向外避开相邻前臂，同时不会让双吸盘约束
// 在大件边缘附近产生不必要的动态扭矩。完整同步轨迹仍须通过 10 ms FCL。
#ifdef TASK15_TIGHT_PHASE
constexpr double DEFAULT_SHARED_GRIP_Y_OFFSET_M = 0.105;
// Isaac 的 Articulation position controller 不消费 MoveIt 的点间速度；紧协调
// 搬运必须给 PhysX 与双吸盘约束足够的跟随时间，不能把轨迹点按墙钟过快推送。
constexpr double DEFAULT_EXECUTION_TIME_SCALE = 2.0;
constexpr int FINAL_COMMAND_HOLD_REPEATS = 200;
// Task15-A 在 105 mm 对称吸附偏置、100 mm 共同抬升下离线 FCL 验收得到的
// PRE_CONTACT 关节姿态。固定该冗余解可消除 RRT 姿态采样使后续同步 Cartesian
// 候选偶发切换肘部构型的问题；到该姿态的 RRT 路径仍每次做碰撞检测与三次重试。
constexpr std::array<double, 7> TASK15_LEFT_PRE_CONTACT_Q{
  0.37910, -0.65746, 1.25502, -2.65743, 0.82499, 2.23021, 0.97346};
constexpr std::array<double, 7> TASK15_RIGHT_PRE_CONTACT_Q{
  -1.38519, -0.24313, -0.19564, -2.71551, -0.07425, 2.47476, -1.51658};
#else
constexpr double DEFAULT_SHARED_GRIP_Y_OFFSET_M = 0.100;
constexpr double DEFAULT_EXECUTION_TIME_SCALE = 1.0;
constexpr int FINAL_COMMAND_HOLD_REPEATS = 50;
#endif

// Task15 不复制 Task11--Task13 已验收的紧协调状态机，而是通过独立 target
// 复用同一源文件。仅切换任务命名、ROS 话题与 Planning Scene object id。
#ifdef TASK15_TIGHT_PHASE
constexpr const char* TASK_NAMESPACE = "/task15";
constexpr const char* SHARED_OBJECT_ID = "task15_large_cube";
constexpr const char* SHARED_POSE_TOPIC = "/task15/large_cube_pose";
constexpr const char* TASK_NODE_NAME = "task15_tight_large_cube";
#else
constexpr const char* TASK_NAMESPACE = "/task11";
constexpr const char* SHARED_OBJECT_ID = "task11_shared_box";
constexpr const char* SHARED_POSE_TOPIC = "/task11/shared_box_pose";
constexpr const char* TASK_NODE_NAME = "task11_shared_box_grasp";
#endif

#ifdef TASK12_SHARED_LIFT
constexpr double DEFAULT_LIFT_HEIGHT_M = 0.050;
constexpr double DEFAULT_SETTLE_SEC = 1.0;
constexpr double DEFAULT_BOX_LIFT_TOLERANCE_M = 0.010;
constexpr double DEFAULT_RELATIVE_TCP_TOLERANCE_M = 0.003;
constexpr double DEFAULT_BOX_ORIENTATION_TOLERANCE_RAD = 0.035;
constexpr double PI = 3.14159265358979323846;
#ifdef TASK12_SHARED_TRANSPORT
constexpr double DEFAULT_TRANSPORT_DELTA_X_M = 0.100;
constexpr double DEFAULT_TRANSPORT_DELTA_Y_M = 0.000;
constexpr double DEFAULT_BOX_TRANSPORT_TOLERANCE_M = 0.010;
#ifdef TASK13_SHARED_PLACE
constexpr double DEFAULT_RELEASE_TIMEOUT_SEC = 3.0;
constexpr double DEFAULT_PLACEMENT_TOLERANCE_M = 0.010;
#endif
#endif
#endif

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

#ifdef TASK12_SHARED_LIFT
bool synchronizeDurations(
  trajectory_msgs::msg::JointTrajectory& first,
  trajectory_msgs::msg::JointTrajectory& second)
{
  ensureTiming(first);
  ensureTiming(second);
  if (first.points.empty() || second.points.empty())
  {
    return false;
  }
  const double first_duration = pointTime(first.points.back());
  const double second_duration = pointTime(second.points.back());
  if (first_duration <= 1e-6 || second_duration <= 1e-6)
  {
    return false;
  }
  const double synchronized_duration = std::max(first_duration, second_duration);
  const auto rescale = [synchronized_duration](
    trajectory_msgs::msg::JointTrajectory& trajectory,
    double original_duration)
  {
    const double scale = synchronized_duration / original_duration;
    for (auto& point : trajectory.points)
    {
      setPointTime(point, pointTime(point) * scale);
    }
  };
  rescale(first, first_duration);
  rescale(second, second_duration);
  return true;
}

std::vector<double> interpolatePositions(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  double time_sec)
{
  const auto& points = trajectory.points;
  if (points.empty())
  {
    return {};
  }
  if (time_sec <= pointTime(points.front()))
  {
    return points.front().positions;
  }
  if (time_sec >= pointTime(points.back()))
  {
    return points.back().positions;
  }

  for (std::size_t index = 1; index < points.size(); ++index)
  {
    const double before_time = pointTime(points[index - 1]);
    const double after_time = pointTime(points[index]);
    if (time_sec > after_time)
    {
      continue;
    }
    const double duration = after_time - before_time;
    if (duration <= 1.0e-9)
    {
      return points[index].positions;
    }
    const double alpha = (time_sec - before_time) / duration;
    std::vector<double> positions(points[index].positions.size());
    for (std::size_t joint = 0; joint < positions.size(); ++joint)
    {
      positions[joint] = points[index - 1].positions[joint] +
        alpha * (points[index].positions[joint] - points[index - 1].positions[joint]);
    }
    return positions;
  }
  return points.back().positions;
}

// 紧协调不是“左臂先完成、右臂再完成”。两个末端必须在同一时间轴上同步运动。
// 因此单臂 Cartesian 仅以双方的共同阶段起点作为碰撞上下文；随后在完整双臂
// RobotState 上逐 10 ms FCL 采样，验证真正将要执行的同步轨迹。
bool validateSynchronizedStage(
  const rclcpp::Node::SharedPtr& node,
  const moveit::core::RobotModelConstPtr& model,
  const std::vector<moveit_msgs::msg::CollisionObject>& static_world_objects,
  const trajectory_msgs::msg::JointTrajectory& left_trajectory,
  const trajectory_msgs::msg::JointTrajectory& right_trajectory,
  const std::string& stage)
{
  if (!model || left_trajectory.joint_names.empty() || right_trajectory.joint_names.empty() ||
      left_trajectory.points.empty() || right_trajectory.points.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "Task12 %s：同步 FCL 输入不完整。", stage.c_str());
    return false;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  for (const auto& object : static_world_objects)
  {
    if (!scene->processCollisionObjectMsg(object))
    {
      RCLCPP_ERROR(
        node->get_logger(), "Task12 %s：无法载入静态碰撞物 %s。",
        stage.c_str(), object.id.c_str());
      return false;
    }
  }

  const double duration = std::max(
    pointTime(left_trajectory.points.back()), pointTime(right_trajectory.points.back()));
  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(duration / SYNCHRONIZED_COLLISION_SAMPLE_PERIOD_SEC));
  for (std::size_t index = 0; index <= sample_count; ++index)
  {
    const double sample_time = std::min(
      static_cast<double>(index) * SYNCHRONIZED_COLLISION_SAMPLE_PERIOD_SEC, duration);
    const auto left_positions = interpolatePositions(left_trajectory, sample_time);
    const auto right_positions = interpolatePositions(right_trajectory, sample_time);
    if (left_positions.size() != left_trajectory.joint_names.size() ||
        right_positions.size() != right_trajectory.joint_names.size())
    {
      RCLCPP_ERROR(node->get_logger(), "Task12 %s：同步 FCL 轨迹点无效。", stage.c_str());
      return false;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(left_trajectory.joint_names, left_positions);
    state.setVariablePositions(right_trajectory.joint_names, right_positions);
    state.update();

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    if (!result.collision)
    {
      continue;
    }

    std::string pair = "unknown";
    if (!result.contacts.empty())
    {
      const auto& first_contact = *result.contacts.begin();
      pair = first_contact.first.first + " <-> " + first_contact.first.second;
    }
    RCLCPP_ERROR(
      node->get_logger(),
      "Task12 %s：同步 FCL 冲突，t=%.3f s，pair=%s。",
      stage.c_str(), sample_time, pair.c_str());
    return false;
  }

  RCLCPP_INFO(
    node->get_logger(), "Task12 %s：同步 FCL PASS，samples=%zu。",
    stage.c_str(), sample_count + 1);
  return true;
}

double distance3d(
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
  if (first_norm <= 1e-9 || second_norm <= 1e-9)
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

bool planCombinedCartesianStage(
  const rclcpp::Node::SharedPtr& node,
  moveit::planning_interface::MoveGroupInterface& group,
  const std::string& group_name,
  const std::string& partner_group_name,
  const std::vector<double>& own_start_q,
  const std::vector<double>& partner_start_q,
  const geometry_msgs::msg::Pose& target,
  const std::string& stage,
  const std::string& side,
  trajectory_msgs::msg::JointTrajectory& output)
{
  const auto model = group.getRobotModel();
  const auto* own_jmg = model->getJointModelGroup(group_name);
  const auto* partner_jmg = model->getJointModelGroup(partner_group_name);
  if (!own_jmg || !partner_jmg || own_start_q.empty() || partner_start_q.empty())
  {
    return false;
  }

  for (int attempt = 1; attempt <= MAX_PLANNING_RETRIES; ++attempt)
  {
    auto state = group.getCurrentState(2.0);
    if (!state)
    {
      return false;
    }
    state->setJointGroupPositions(own_jmg, own_start_q);
    state->setJointGroupPositions(partner_jmg, partner_start_q);
    state->update();
    group.setStartState(*state);

    moveit_msgs::msg::RobotTrajectory robot_trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    // 两臂的真实轨迹将严格同步。若此处开启单臂 avoid_collisions，MoveIt 只能把
    // 另一臂冻结在阶段起点，便会把“另一臂随后同步离开”的合法相对运动误判为
    // 碰撞，导致 Cartesian fraction 从 0 开始。
    //
    // 因而这里只生成单臂几何候选；两条候选同步后必须通过
    // validateSynchronizedStage() 的完整双臂 FCL 检查，未通过绝不执行。
    const double fraction = group.computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target}, CARTESIAN_EEF_STEP, 0.0,
      robot_trajectory, false, &error);
    RCLCPP_INFO(
      node->get_logger(),
      "Task12 %s %s Cartesian fraction=%.4f, error=%d, attempt=%d/%d",
      side.c_str(), stage.c_str(), fraction, error.val, attempt, MAX_PLANNING_RETRIES);
    if (fraction >= CARTESIAN_MIN_FRACTION && !robot_trajectory.joint_trajectory.points.empty())
    {
      output = robot_trajectory.joint_trajectory;
      ensureTiming(output);
      return true;
    }
  }

  RCLCPP_ERROR(
    node->get_logger(),
    "Task12 %s %s：三次 Cartesian 规划均失败，停止任务。",
    side.c_str(), stage.c_str());
  return false;
}
#endif

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
  object.id = SHARED_OBJECT_ID;
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
      SHARED_POSE_TOPIC, 10,
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

#ifdef TASK15_TIGHT_PHASE
bool requestTask15StableSupportLock(
  const rclcpp::Node::SharedPtr& node,
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
  const std::string& object_id)
{
  for (int attempt = 0; attempt < 30; ++attempt)
  {
    if (publisher && publisher->get_subscription_count() > 0)
    {
      std_msgs::msg::String request;
      request.data = object_id;
      // Isaac 端对 object id 幂等；短间隔重发仅避免 DDS 刚发现时的首包丢失。
      for (int repeat = 0; repeat < 3; ++repeat)
      {
        publisher->publish(request);
        std::this_thread::sleep_for(50ms);
      }
      RCLCPP_INFO(
        node->get_logger(), "Task15 stable-support lock requested: %s", object_id.c_str());
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  RCLCPP_ERROR(node->get_logger(), "Task15 lock bridge 未就绪：%s", object_id.c_str());
  return false;
}
#endif

class ArmControl
{
public:
  ArmControl(
    const rclcpp::Node::SharedPtr& node,
    bool left,
    double execution_time_scale)
    : node_(node),
      side_(left ? "left" : "right"),
      group_name_(left ? "left_arm" : "right_arm"),
      eef_link_(left ? "left_fr3_link8" : "right_fr3_link8"),
      joint_prefix_(left ? "left_" : "right_"),
      execution_time_scale_(execution_time_scale)
  {
    command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/" + side_ + "/joint_command", 10);
    suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      std::string(TASK_NAMESPACE) + "/" + side_ + "/suction_command", 10);
    state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      std::string(TASK_NAMESPACE) + "/" + side_ + "/suction_state", 10,
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
    // 每次 MoveIt 调用只做一次；外层以固定三次重规划提供确定的失败语义。
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.15);
    group.setMaxAccelerationScalingFactor(0.15);
    group.setPoseReferenceFrame("world");
    group.setEndEffectorLink(eef_link_);
    group.clearPoseTargets();
    if (!group.setPoseTarget(target, eef_link_))
    {
      return false;
    }

    for (int attempt = 1; attempt <= MAX_PLANNING_RETRIES; ++attempt)
    {
      group.setStartStateToCurrentState();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
      {
        output = plan.trajectory_.joint_trajectory;
        ensureTiming(output);
        RCLCPP_INFO(
          node_->get_logger(),
          "Task11 %s PRE_CONTACT plan OK, attempt=%d/%d",
          side_.c_str(), attempt, MAX_PLANNING_RETRIES);
        return true;
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "Task11 %s PRE_CONTACT RRTConnect 失败，attempt=%d/%d。",
        side_.c_str(), attempt, MAX_PLANNING_RETRIES);
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Task11 %s PRE_CONTACT：三次 RRTConnect 规划均失败，停止任务。",
      side_.c_str());
    return false;
  }

  bool planJointTarget(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::vector<double>& target_q,
    trajectory_msgs::msg::JointTrajectory& output) const
  {
    const auto* joint_model_group = group.getRobotModel()->getJointModelGroup(group_name_);
    if (joint_model_group == nullptr ||
        target_q.size() != joint_model_group->getVariableCount())
    {
      return false;
    }

    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.15);
    group.setMaxAccelerationScalingFactor(0.15);
    group.clearPoseTargets();
    group.setJointValueTarget(target_q);

    for (int attempt = 1; attempt <= MAX_PLANNING_RETRIES; ++attempt)
    {
      group.setStartStateToCurrentState();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
      {
        output = plan.trajectory_.joint_trajectory;
        ensureTiming(output);
        RCLCPP_INFO(
          node_->get_logger(),
          "Task15 %s FIXED PRE_CONTACT joint plan OK, attempt=%d/%d",
          side_.c_str(), attempt, MAX_PLANNING_RETRIES);
        return true;
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "Task15 %s FIXED PRE_CONTACT RRTConnect 失败，attempt=%d/%d。",
        side_.c_str(), attempt, MAX_PLANNING_RETRIES);
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Task15 %s FIXED PRE_CONTACT：三次 RRTConnect 规划均失败，停止任务。",
      side_.c_str());
    return false;
  }

  bool planCartesianContact(
    moveit::planning_interface::MoveGroupInterface& group,
    const std::vector<double>& start_q,
    const geometry_msgs::msg::Pose& target,
    trajectory_msgs::msg::JointTrajectory& output) const
  {
    const auto* jmg = group.getRobotModel()->getJointModelGroup(group_name_);
    if (jmg == nullptr || start_q.empty())
    {
      return false;
    }

    for (int attempt = 1; attempt <= MAX_PLANNING_RETRIES; ++attempt)
    {
      auto state = group.getCurrentState(2.0);
      if (!state)
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
        "Task11 %s CONTACT Cartesian fraction=%.4f, error=%d, attempt=%d/%d",
        side_.c_str(), fraction, error.val, attempt, MAX_PLANNING_RETRIES);
      if (fraction >= CARTESIAN_MIN_FRACTION && !robot_trajectory.joint_trajectory.points.empty())
      {
        output = robot_trajectory.joint_trajectory;
        ensureTiming(output);
        return true;
      }
    }

    RCLCPP_ERROR(
      node_->get_logger(),
      "Task11 %s CONTACT：三次 Cartesian 规划均失败，停止任务。",
      side_.c_str());
    return false;
  }

  bool execute(const trajectory_msgs::msg::JointTrajectory& input) const
  {
    return executeAt(input, std::chrono::steady_clock::now());
  }

  bool executeAt(
    const trajectory_msgs::msg::JointTrajectory& input,
    const std::chrono::steady_clock::time_point& start) const
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
    std::this_thread::sleep_until(start);
    std::size_t segment = 0;
    const auto publish = [&](const std::vector<double>& positions)
    {
      sensor_msgs::msg::JointState message;
      message.header.stamp = node_->now();
      message.name = names;
      message.position = positions;
      command_pub_->publish(message);
    };
    const double logical_duration = pointTime(trajectory.points.back());
    while (true)
    {
      const double wall_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
      const double time = wall_time / execution_time_scale_;
      if (time > logical_duration)
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
    for (int repeat = 0; repeat < FINAL_COMMAND_HOLD_REPEATS; ++repeat)
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
  const std::string& eefLink() const { return eef_link_; }

private:
  rclcpp::Node::SharedPtr node_;
  std::string side_;
  std::string group_name_;
  std::string eef_link_;
  std::string joint_prefix_;
  double execution_time_scale_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  std::atomic_bool have_suction_state_{false};
  std::atomic_bool suction_closed_{false};
};

// 两条共同搬运轨迹共享同一个未来起始时刻。此前两个线程分别在各自的
// execute() 内读取 now()，会产生毫秒级起跑偏差并放大为动态相对位姿误差。
bool executeSynchronously(
  const ArmControl& left,
  const trajectory_msgs::msg::JointTrajectory& left_trajectory,
  const ArmControl& right,
  const trajectory_msgs::msg::JointTrajectory& right_trajectory)
{
  const auto start = std::chrono::steady_clock::now() + 100ms;
  bool left_ok = false;
  bool right_ok = false;
  std::thread left_thread([&left, &left_trajectory, &left_ok, &start]()
  {
    left_ok = left.executeAt(left_trajectory, start);
  });
  std::thread right_thread([&right, &right_trajectory, &right_ok, &start]()
  {
    right_ok = right.executeAt(right_trajectory, start);
  });
  left_thread.join();
  right_thread.join();
  return left_ok && right_ok;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
#ifdef TASK15_TIGHT_PHASE
  auto node = std::make_shared<rclcpp::Node>(TASK_NODE_NAME);
#elif defined(TASK13_SHARED_PLACE)
  auto node = std::make_shared<rclcpp::Node>("task13_shared_box_place");
#elif defined(TASK12_SHARED_TRANSPORT)
  auto node = std::make_shared<rclcpp::Node>("task12_shared_box_transport");
#elif defined(TASK12_SHARED_LIFT)
  auto node = std::make_shared<rclcpp::Node>("task12_shared_box_lift");
#else
  auto node = std::make_shared<rclcpp::Node>("task11_shared_box_grasp");
#endif
  const bool execute = node->declare_parameter<bool>("execute", false);
  const double grasp_timeout = node->declare_parameter<double>("grasp_timeout_sec", 3.0);
  const double shared_grip_y_offset = node->declare_parameter<double>(
    "shared_grip_y_offset_m", DEFAULT_SHARED_GRIP_Y_OFFSET_M);
  const double execution_time_scale = node->declare_parameter<double>(
    "execution_time_scale", DEFAULT_EXECUTION_TIME_SCALE);
#ifdef TASK12_SHARED_LIFT
  const double lift_height = node->declare_parameter<double>(
    "lift_height_m", DEFAULT_LIFT_HEIGHT_M);
  const double settle_sec = node->declare_parameter<double>("settle_sec", DEFAULT_SETTLE_SEC);
  const double box_lift_tolerance = node->declare_parameter<double>(
    "box_lift_tolerance_m", DEFAULT_BOX_LIFT_TOLERANCE_M);
  const double relative_tcp_tolerance = node->declare_parameter<double>(
    "relative_tcp_tolerance_m", DEFAULT_RELATIVE_TCP_TOLERANCE_M);
  const double box_orientation_tolerance = node->declare_parameter<double>(
    "box_orientation_tolerance_rad", DEFAULT_BOX_ORIENTATION_TOLERANCE_RAD);
#ifdef TASK12_SHARED_TRANSPORT
  const double transport_delta_x = node->declare_parameter<double>(
    "transport_delta_x_m", DEFAULT_TRANSPORT_DELTA_X_M);
  const double transport_delta_y = node->declare_parameter<double>(
    "transport_delta_y_m", DEFAULT_TRANSPORT_DELTA_Y_M);
  const double box_transport_tolerance = node->declare_parameter<double>(
    "box_transport_tolerance_m", DEFAULT_BOX_TRANSPORT_TOLERANCE_M);
#ifdef TASK13_SHARED_PLACE
  const double release_timeout = node->declare_parameter<double>(
    "release_timeout_sec", DEFAULT_RELEASE_TIMEOUT_SEC);
  const double placement_tolerance = node->declare_parameter<double>(
    "placement_tolerance_m", DEFAULT_PLACEMENT_TOLERANCE_M);
#endif
#endif
#endif

  if (!copyRobotModelParameters(node) || grasp_timeout <= 0.0 ||
      shared_grip_y_offset <= 0.0 || shared_grip_y_offset >= 0.5 * BOX_Y ||
      execution_time_scale < 1.0
#ifdef TASK12_SHARED_LIFT
      || lift_height <= 0.0 || settle_sec < 0.0 || box_lift_tolerance <= 0.0 ||
      relative_tcp_tolerance <= 0.0 || box_orientation_tolerance <= 0.0
#ifdef TASK12_SHARED_TRANSPORT
      || std::hypot(transport_delta_x, transport_delta_y) <= 0.0 ||
      box_transport_tolerance <= 0.0
#ifdef TASK13_SHARED_PLACE
      || release_timeout <= 0.0 || placement_tolerance <= 0.0
#endif
#endif
#endif
  )
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
#ifdef TASK15_TIGHT_PHASE
    RCLCPP_INFO(
      node->get_logger(),
      "========== Task15 Phase A / TIGHT: reuse Task11--13 shared-object protocol (lift=%.3f m, delta=(%.3f, %.3f) m) ==========" ,
      lift_height, transport_delta_x, transport_delta_y);
#elif defined(TASK13_SHARED_PLACE)
    RCLCPP_INFO(
      node->get_logger(),
      "========== Task13 SHARED-BOX PLACE (lift=%.3f m, delta=(%.3f, %.3f) m) ==========",
      lift_height, transport_delta_x, transport_delta_y);
#elif defined(TASK12_SHARED_TRANSPORT)
    RCLCPP_INFO(
      node->get_logger(),
      "========== Task12 SHARED-BOX LIFT + TRANSPORT (lift=%.3f m, delta=(%.3f, %.3f) m) ==========",
      lift_height, transport_delta_x, transport_delta_y);
#elif defined(TASK12_SHARED_LIFT)
    RCLCPP_INFO(
      node->get_logger(),
      "========== Task12 SHARED-BOX COMMON LIFT (height=%.3f m) ==========",
      lift_height);
#else
    RCLCPP_INFO(node->get_logger(), "========== Task11 SHARED-BOX DUAL-SUCTION ==========");
#endif
    if (!pose_buffer->wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "等待 %s 超时。", SHARED_POSE_TOPIC);
      break;
    }
    ArmControl left(node, true, execution_time_scale);
    ArmControl right(node, false, execution_time_scale);
    if (!left.waitForBridge() || !right.waitForBridge())
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 Isaac bridge 未就绪。");
      break;
    }
#ifdef TASK15_TIGHT_PHASE
    const auto task15_lock_pub = node->create_publisher<std_msgs::msg::String>(
      "/task15/lock_placed_object", 10);
#endif

    moveit::planning_interface::PlanningSceneInterface scene;
    // Task15 是单入口可重复执行任务。Phase B 的预检或中途失败可能把四个小件
    // 留在 MoveIt World；它们若残留在目标层高，会确定性阻挡本轮共同运输。
    // 这里只清理 Task15 自己的 object id，不触碰桌面或其他任务的碰撞物。
    std::vector<std::string> stale_task_objects{SHARED_OBJECT_ID};
#ifdef TASK15_TIGHT_PHASE
    stale_task_objects.insert(
      stale_task_objects.end(),
      {
        "task15_small_cube_1",
        "task15_small_cube_2",
        "task15_small_cube_3",
        "task15_small_cube_4",
      });
#endif
    scene.removeCollisionObjects(stale_task_objects);
    std::this_thread::sleep_for(250ms);

    const auto box_pose = pose_buffer->get();
    if (!scene.applyCollisionObject(sharedBoxObject(box_pose)))
    {
      RCLCPP_ERROR(node->get_logger(), "无法将 SharedBox 加入 MoveIt World。");
      break;
    }
    std::this_thread::sleep_for(300ms);

    const double contact_z = box_pose.position.z + 0.5 * BOX_Z + CONTACT_CLEARANCE_Z;
    const auto left_contact = topDownPose(
      box_pose.position.x, box_pose.position.y - shared_grip_y_offset, contact_z);
    const auto right_contact = topDownPose(
      box_pose.position.x, box_pose.position.y + shared_grip_y_offset, contact_z);
#ifndef TASK15_TIGHT_PHASE
    const auto left_pre = topDownPose(
      box_pose.position.x, box_pose.position.y - shared_grip_y_offset,
      contact_z + PRE_CONTACT_CLEARANCE_Z);
    const auto right_pre = topDownPose(
      box_pose.position.x, box_pose.position.y + shared_grip_y_offset,
      contact_z + PRE_CONTACT_CLEARANCE_Z);
#endif

    moveit::planning_interface::MoveGroupInterface left_group(node, left.groupName());
    moveit::planning_interface::MoveGroupInterface right_group(node, right.groupName());
    trajectory_msgs::msg::JointTrajectory left_pre_traj;
    trajectory_msgs::msg::JointTrajectory right_pre_traj;
#ifdef TASK15_TIGHT_PHASE
    if (!left.planJointTarget(
          left_group,
          std::vector<double>(
            TASK15_LEFT_PRE_CONTACT_Q.begin(), TASK15_LEFT_PRE_CONTACT_Q.end()),
          left_pre_traj) ||
        !right.planJointTarget(
          right_group,
          std::vector<double>(
            TASK15_RIGHT_PRE_CONTACT_Q.begin(), TASK15_RIGHT_PRE_CONTACT_Q.end()),
          right_pre_traj))
#else
    if (!left.planPose(left_group, left_pre, left_pre_traj) ||
        !right.planPose(right_group, right_pre, right_pre_traj))
#endif
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 PRE_CONTACT 规划失败。");
      break;
    }

    if (execute)
    {
      // Task15 固定已验收的冗余关节目标，避免右臂在左臂到位后再次采样出
      // 不同肘部构型。其 RRT 路径仍通过 MoveIt 的全机器人碰撞检查。
      if (!left.execute(left_pre_traj)
#ifdef TASK15_TIGHT_PHASE
          || !right.planJointTarget(
            right_group,
            std::vector<double>(
              TASK15_RIGHT_PRE_CONTACT_Q.begin(), TASK15_RIGHT_PRE_CONTACT_Q.end()),
            right_pre_traj)
#else
          // 旧 Task11--13 保持原有行为：右臂到左臂静止后的当前状态重新规划。
          || !right.planPose(right_group, right_pre, right_pre_traj)
#endif
          || !right.execute(right_pre_traj))
      {
        RCLCPP_ERROR(node->get_logger(), "Task11 PRE_CONTACT 执行失败。");
        break;
      }
      RCLCPP_INFO(node->get_logger(), "Task11 两臂均到达 PRE_CONTACT。");
    }

    // CONTACT 是有意接触，故仅在两条已完成的预接触轨迹之间临时移除 SharedBox。
    // Task12 共同抬升阶段会以完整双臂 RobotState 检查臂-臂碰撞；由于同一物体
    // 不能同时作为两个末端的 MoveIt AttachedBody，SharedBox 的实际保持由 Isaac
    // 两个 Surface Gripper 物理约束负责。
    scene.removeCollisionObjects({SHARED_OBJECT_ID});
    std::this_thread::sleep_for(250ms);
    // SharedBox 在双吸盘保持期间由 Isaac PhysX 约束表示，不能同时 attach 到两条
    // MoveIt link。同步 FCL 因此只载入仍存在于 World 的静态物体（桌面等）。
    std::vector<moveit_msgs::msg::CollisionObject> static_world_objects;
    for (const auto& [object_id, object] : scene.getObjects())
    {
      if (object_id != SHARED_OBJECT_ID)
      {
        static_world_objects.push_back(object);
      }
    }
    trajectory_msgs::msg::JointTrajectory left_contact_traj;
    trajectory_msgs::msg::JointTrajectory right_contact_traj;
    if (!left.planCartesianContact(
          left_group, finalPositions(left_pre_traj), left_contact, left_contact_traj) ||
        !right.planCartesianContact(
          right_group, finalPositions(right_pre_traj), right_contact, right_contact_traj)
#ifdef TASK12_SHARED_LIFT
        || !synchronizeDurations(left_contact_traj, right_contact_traj)
        || !validateSynchronizedStage(
          node, left_group.getRobotModel(), static_world_objects,
          left_contact_traj, right_contact_traj, "CONTACT")
#endif
        )
    {
      RCLCPP_ERROR(node->get_logger(), "Task11 CONTACT Cartesian 规划失败。");
      break;
    }

#ifdef TASK12_SHARED_LIFT
    // SharedBox 由两个物理 Surface Gripper 共同保持，MoveIt 当前没有可同时
    // attach 到两个末端的 AttachedBody 表示。因此共同抬升采用完整双臂 RobotState
    // 做臂-臂碰撞检查；箱体只在有意接触与吸附期间临时移出 MoveIt World。
    const auto left_lift_target = topDownPose(
      box_pose.position.x, box_pose.position.y - shared_grip_y_offset, contact_z + lift_height);
    const auto right_lift_target = topDownPose(
      box_pose.position.x, box_pose.position.y + shared_grip_y_offset, contact_z + lift_height);
    trajectory_msgs::msg::JointTrajectory left_lift_traj;
    trajectory_msgs::msg::JointTrajectory right_lift_traj;
    if (!planCombinedCartesianStage(
          node, left_group, left.groupName(), right.groupName(),
          finalPositions(left_contact_traj), finalPositions(right_contact_traj),
          left_lift_target, "COMMON_LIFT", left.side(), left_lift_traj) ||
        !planCombinedCartesianStage(
          node, right_group, right.groupName(), left.groupName(),
          finalPositions(right_contact_traj), finalPositions(left_contact_traj),
          right_lift_target, "COMMON_LIFT", right.side(), right_lift_traj) ||
        !synchronizeDurations(left_lift_traj, right_lift_traj) ||
        !validateSynchronizedStage(
          node, left_group.getRobotModel(), static_world_objects,
          left_lift_traj, right_lift_traj, "COMMON_LIFT"))
    {
      RCLCPP_ERROR(node->get_logger(), "Task12 COMMON_LIFT 规划或同步失败。");
      break;
    }

#ifdef TASK12_SHARED_TRANSPORT
    // 水平运输从共同抬升的终点开始；该姿态同时写入左右 group 的 start state，
    // 使每次 Cartesian 计算都以另一台机械臂真实的共同搬运姿态为碰撞上下文。
    const auto left_transport_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y - shared_grip_y_offset + transport_delta_y,
      contact_z + lift_height);
    const auto right_transport_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y + shared_grip_y_offset + transport_delta_y,
      contact_z + lift_height);
    trajectory_msgs::msg::JointTrajectory left_transport_traj;
    trajectory_msgs::msg::JointTrajectory right_transport_traj;
    if (!planCombinedCartesianStage(
          node, left_group, left.groupName(), right.groupName(),
          finalPositions(left_lift_traj), finalPositions(right_lift_traj),
          left_transport_target, "COMMON_TRANSPORT", left.side(), left_transport_traj) ||
        !planCombinedCartesianStage(
          node, right_group, right.groupName(), left.groupName(),
          finalPositions(right_lift_traj), finalPositions(left_lift_traj),
          right_transport_target, "COMMON_TRANSPORT", right.side(), right_transport_traj) ||
        !synchronizeDurations(left_transport_traj, right_transport_traj) ||
        !validateSynchronizedStage(
          node, left_group.getRobotModel(), static_world_objects,
          left_transport_traj, right_transport_traj, "COMMON_TRANSPORT"))
    {
      RCLCPP_ERROR(node->get_logger(), "Task12 COMMON_TRANSPORT 规划或同步失败。");
      break;
    }

#ifdef TASK13_SHARED_PLACE
    // 共同下降时仍由双吸盘保持 SharedBox。目标 TCP 保持 1 mm release gap，
    // 使箱体底面在释放前恰好悬于桌面顶面上方，随后由 PhysX 自然落稳。
    const auto left_place_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y - shared_grip_y_offset + transport_delta_y,
      contact_z);
    const auto right_place_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y + shared_grip_y_offset + transport_delta_y,
      contact_z);
    trajectory_msgs::msg::JointTrajectory left_descent_traj;
    trajectory_msgs::msg::JointTrajectory right_descent_traj;
    if (!planCombinedCartesianStage(
          node, left_group, left.groupName(), right.groupName(),
          finalPositions(left_transport_traj), finalPositions(right_transport_traj),
          left_place_target, "COMMON_DESCENT", left.side(), left_descent_traj) ||
        !planCombinedCartesianStage(
          node, right_group, right.groupName(), left.groupName(),
          finalPositions(right_transport_traj), finalPositions(left_transport_traj),
          right_place_target, "COMMON_DESCENT", right.side(), right_descent_traj) ||
        !synchronizeDurations(left_descent_traj, right_descent_traj) ||
        !validateSynchronizedStage(
          node, left_group.getRobotModel(), static_world_objects,
          left_descent_traj, right_descent_traj, "COMMON_DESCENT"))
    {
      RCLCPP_ERROR(node->get_logger(), "Task13 COMMON_DESCENT 规划或同步失败。");
      break;
    }

    // RETREAT 必须在物理释放前规划：此时 SharedBox 仍由双端吸附保持，不会成为
    // 夹在吸盘下方的 World CollisionObject。释放、落稳、回写 MoveIt World 后只执行。
    const auto left_retreat_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y - shared_grip_y_offset + transport_delta_y,
      contact_z + PRE_CONTACT_CLEARANCE_Z);
    const auto right_retreat_target = topDownPose(
      box_pose.position.x + transport_delta_x,
      box_pose.position.y + shared_grip_y_offset + transport_delta_y,
      contact_z + PRE_CONTACT_CLEARANCE_Z);
    trajectory_msgs::msg::JointTrajectory left_retreat_traj;
    trajectory_msgs::msg::JointTrajectory right_retreat_traj;
    if (!planCombinedCartesianStage(
          node, left_group, left.groupName(), right.groupName(),
          finalPositions(left_descent_traj), finalPositions(right_descent_traj),
          left_retreat_target, "COMMON_RETREAT", left.side(), left_retreat_traj) ||
        !planCombinedCartesianStage(
          node, right_group, right.groupName(), left.groupName(),
          finalPositions(right_descent_traj), finalPositions(left_descent_traj),
          right_retreat_target, "COMMON_RETREAT", right.side(), right_retreat_traj) ||
        !synchronizeDurations(left_retreat_traj, right_retreat_traj) ||
        !validateSynchronizedStage(
          node, left_group.getRobotModel(), static_world_objects,
          left_retreat_traj, right_retreat_traj, "COMMON_RETREAT"))
    {
      RCLCPP_ERROR(node->get_logger(), "Task13 COMMON_RETREAT 规划或同步失败。");
      break;
    }
#endif
#endif
#endif

    if (!execute)
    {
#ifdef TASK15_TIGHT_PHASE
      RCLCPP_INFO(
        node->get_logger(),
        "Task15 Phase A PRECHECK PASS：复用的 CONTACT / LIFT / TRANSPORT / PLACE / RETREAT 均已规划；未发布任何 joint / suction 命令。");
#elif defined(TASK13_SHARED_PLACE)
      RCLCPP_INFO(
        node->get_logger(),
        "Task13 PRECHECK PASS：CONTACT / LIFT / TRANSPORT / DESCENT / RETREAT 均已规划；未发布任何 joint / suction 命令。");
#elif defined(TASK12_SHARED_TRANSPORT)
      RCLCPP_INFO(
        node->get_logger(),
        "Task12 PRECHECK PASS：双 CONTACT、%.3f m COMMON_LIFT 与 delta=(%.3f, %.3f) m COMMON_TRANSPORT 已规划；未发布任何 joint / suction 命令。",
        lift_height, transport_delta_x, transport_delta_y);
#elif defined(TASK12_SHARED_LIFT)
      RCLCPP_INFO(
        node->get_logger(),
        "Task12 PRECHECK PASS：双 CONTACT 与 %.3f m COMMON_LIFT 已规划；未发布任何 joint / suction 命令。",
        lift_height);
#else
      RCLCPP_INFO(node->get_logger(), "Task11 PRECHECK PASS：未发布任何 joint / suction 命令。");
#endif
      success = true;
      break;
    }

    if (!executeSynchronously(left, left_contact_traj, right, right_contact_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "CONTACT 同步执行失败。");
      break;
    }

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

#ifdef TASK12_SHARED_LIFT
    const auto box_before_lift = pose_buffer->get();
    const auto left_before_lift = left_group.getCurrentPose(left.eefLink()).pose;
    const auto right_before_lift = right_group.getCurrentPose(right.eefLink()).pose;
    const auto relative_before = subtractPoints(
      right_before_lift.position, left_before_lift.position);

    if (!executeSynchronously(left, left_lift_traj, right, right_lift_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "COMMON_LIFT 同步执行失败。");
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_sec));

    const auto box_after_lift = pose_buffer->get();
    const auto left_after_lift = left_group.getCurrentPose(left.eefLink()).pose;
    const auto right_after_lift = right_group.getCurrentPose(right.eefLink()).pose;
    const auto relative_after = subtractPoints(
      right_after_lift.position, left_after_lift.position);
    geometry_msgs::msg::Point expected_box_position = box_before_lift.position;
    expected_box_position.z += lift_height;
    const double box_lift_error = distance3d(
      box_after_lift.position, expected_box_position);
    const double relative_tcp_error = distance3d(relative_after, relative_before);
    const double box_orientation_error = quaternionAngularDistance(
      box_after_lift.orientation, box_before_lift.orientation);
    const bool lift_ok =
      box_lift_error <= box_lift_tolerance &&
      relative_tcp_error <= relative_tcp_tolerance &&
      box_orientation_error <= box_orientation_tolerance;
    RCLCPP_INFO(
      node->get_logger(),
      "Task12 LIFT METRICS: expected_box_z=%.4f, actual_box=(%.4f, %.4f, %.4f), "
      "box_error=%.3f mm, relative_link8_error=%.3f mm, orientation_error=%.3f deg",
      expected_box_position.z,
      box_after_lift.position.x, box_after_lift.position.y, box_after_lift.position.z,
      box_lift_error * 1000.0, relative_tcp_error * 1000.0,
      box_orientation_error * 180.0 / PI);
    if (!lift_ok)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task12 FAIL：共同抬升误差超限（box<=%.3f mm, relative_link8<=%.3f mm, orientation<=%.3f deg）。",
        box_lift_tolerance * 1000.0, relative_tcp_tolerance * 1000.0,
        box_orientation_tolerance * 180.0 / PI);
      break;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Task12 PASS：双 Surface Gripper 共同抬升 %.3f m；SharedBox 与双臂相对几何均在阈值内。",
      lift_height);

#ifdef TASK12_SHARED_TRANSPORT
    const auto box_before_transport = box_after_lift;
    const auto left_before_transport = left_after_lift;
    const auto right_before_transport = right_after_lift;
    const auto relative_before_transport = subtractPoints(
      right_before_transport.position, left_before_transport.position);
    if (!executeSynchronously(left, left_transport_traj, right, right_transport_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "COMMON_TRANSPORT 同步执行失败。");
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_sec));

    const auto box_after_transport = pose_buffer->get();
    const auto left_after_transport = left_group.getCurrentPose(left.eefLink()).pose;
    const auto right_after_transport = right_group.getCurrentPose(right.eefLink()).pose;
    const auto relative_after_transport = subtractPoints(
      right_after_transport.position, left_after_transport.position);
    geometry_msgs::msg::Point expected_transport_position = box_before_transport.position;
    expected_transport_position.x += transport_delta_x;
    expected_transport_position.y += transport_delta_y;
    const double box_transport_error = distance3d(
      box_after_transport.position, expected_transport_position);
    const double relative_transport_error = distance3d(
      relative_after_transport, relative_before_transport);
    const double transport_orientation_error = quaternionAngularDistance(
      box_after_transport.orientation, box_before_transport.orientation);
    const bool transport_ok =
      box_transport_error <= box_transport_tolerance &&
      relative_transport_error <= relative_tcp_tolerance &&
      transport_orientation_error <= box_orientation_tolerance;
    RCLCPP_INFO(
      node->get_logger(),
      "Task12 TRANSPORT METRICS: expected_box=(%.4f, %.4f, %.4f), actual_box=(%.4f, %.4f, %.4f), "
      "box_error=%.3f mm, relative_link8_error=%.3f mm, orientation_error=%.3f deg",
      expected_transport_position.x, expected_transport_position.y, expected_transport_position.z,
      box_after_transport.position.x, box_after_transport.position.y, box_after_transport.position.z,
      box_transport_error * 1000.0, relative_transport_error * 1000.0,
      transport_orientation_error * 180.0 / PI);
    if (!transport_ok)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task12 TRANSPORT FAIL：误差超限（box<=%.3f mm, relative_link8<=%.3f mm, orientation<=%.3f deg）。",
        box_transport_tolerance * 1000.0, relative_tcp_tolerance * 1000.0,
        box_orientation_tolerance * 180.0 / PI);
      break;
    }
    RCLCPP_INFO(
      node->get_logger(),
      "Task12-B PASS：双 Surface Gripper 共同运输 delta=(%.3f, %.3f) m；SharedBox 与双臂相对几何均在阈值内。",
      transport_delta_x, transport_delta_y);

#ifdef TASK13_SHARED_PLACE
    if (!executeSynchronously(left, left_descent_traj, right, right_descent_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "COMMON_DESCENT 同步执行失败。");
      break;
    }

    std::thread left_release_thread([&left]() { left.commandSuction(false); });
    std::thread right_release_thread([&right]() { right.commandSuction(false); });
    left_release_thread.join();
    right_release_thread.join();
    const bool left_open = left.waitForSuction(false, release_timeout);
    const bool right_open = right.waitForSuction(false, release_timeout);
    if (!left_open || !right_open)
    {
      RCLCPP_ERROR(
        node->get_logger(), "Task13 RELEASE FAIL：left_open=%s, right_open=%s",
        left_open ? "true" : "false", right_open ? "true" : "false");
      break;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(settle_sec));
    const auto box_after_release = pose_buffer->get();
    geometry_msgs::msg::Point expected_placement = box_pose.position;
    expected_placement.x += transport_delta_x;
    expected_placement.y += transport_delta_y;
    const double placement_error = distance3d(
      box_after_release.position, expected_placement);
    const double placement_orientation_error = quaternionAngularDistance(
      box_after_release.orientation, box_pose.orientation);

    // 已释放的真实 pose 写回 MoveIt World；随后执行的是释放前已经规划好的上退，
    // 不在接触起点重新调用 Cartesian 规划。
    scene.removeCollisionObjects({SHARED_OBJECT_ID});
    std::this_thread::sleep_for(250ms);
    if (!scene.applyCollisionObject(sharedBoxObject(box_after_release)))
    {
      RCLCPP_ERROR(node->get_logger(), "Task13 无法将释放后的 SharedBox 回写 MoveIt World。");
      break;
    }
    std::this_thread::sleep_for(300ms);
#ifdef TASK15_TIGHT_PHASE
    // Phase A 的大件已通过真实落稳测量。将其锁定为保留 Collider 的稳定支撑面，
    // 避免后续小件搬运时被末端轻微接触推离目标；绝不从 MoveIt/PhysX 中忽略它。
    if (!requestTask15StableSupportLock(node, task15_lock_pub, SHARED_OBJECT_ID))
    {
      break;
    }
#endif
    if (!executeSynchronously(left, left_retreat_traj, right, right_retreat_traj))
    {
      RCLCPP_ERROR(node->get_logger(), "COMMON_RETREAT 同步执行失败。");
      break;
    }
    const bool placement_ok =
      placement_error <= placement_tolerance &&
      placement_orientation_error <= box_orientation_tolerance;
    RCLCPP_INFO(
      node->get_logger(),
      "Task13 PLACE METRICS: expected_box=(%.4f, %.4f, %.4f), actual_box=(%.4f, %.4f, %.4f), "
      "placement_error=%.3f mm, orientation_error=%.3f deg",
      expected_placement.x, expected_placement.y, expected_placement.z,
      box_after_release.position.x, box_after_release.position.y, box_after_release.position.z,
      placement_error * 1000.0, placement_orientation_error * 180.0 / PI);
    if (!placement_ok)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "Task13 PLACE FAIL：释放后位置/姿态误差超限（position<=%.3f mm, orientation<=%.3f deg）。",
        placement_tolerance * 1000.0, box_orientation_tolerance * 180.0 / PI);
      break;
    }
#ifdef TASK15_TIGHT_PHASE
    RCLCPP_INFO(
      node->get_logger(),
      "Task15 Phase A PASS：LargeCube 已完成紧协调共同抓取、运输、放置、Ground Truth 回写与共同安全退出。");
#else
    RCLCPP_INFO(
      node->get_logger(),
      "Task13 PASS：共同下降、同步释放、SharedBox Ground Truth 回写与共同安全退出均完成。");
#endif
#endif
#endif
#else
    const auto settled = pose_buffer->get();
    RCLCPP_INFO(
      node->get_logger(),
      "Task11 PASS：双 Surface Gripper 同时 CLOSED；SharedBox=(%.4f, %.4f, %.4f)。",
      settled.position.x, settled.position.y, settled.position.z);
#endif
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
