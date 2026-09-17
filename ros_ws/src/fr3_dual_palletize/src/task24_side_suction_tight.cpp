// Task24：短 L 型侧面吸盘的离线紧协调码垛执行器。
//
// 设计边界：不读取选择器，不做在线任务排序。完整离线任务表预留 8 件 Cube，但
// 当前只允许执行一个真实 Cube；单件通过前不启动多件调度。每件仍由 Isaac Ground
// Truth 门禁、MoveIt 规划、同步 FCL 与双 Surface Gripper 物理闭环共同验证。该文件
// 复用 Task11--13 的共同 lift / transport / descent / release 原则，但侧吸几何和
// “墙优先 + 短推”是独立 Task24。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
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
// Task24-B：线性尺寸为旧 30 mm Cube 的 4 倍；所有 Cube 都由双臂紧协调。
constexpr double kCubeSize = 0.120;
constexpr double kCubeHalf = 0.060;
constexpr double kTableTopZ = 0.050;
constexpr double kBottomZ = kTableTopZ + kCubeHalf;
constexpr double kUpperZ = kBottomZ + kCubeSize;
// Cup 面与 Cube 侧面的静态间隙固定为 1 mm。Surface Gripper 在 close() 时
// 固定当前相对位姿而不会主动把 Cube 拉过这段距离；10 mm 会形成明显浮空，
// 因此不能再作为“真空近接触”使用。1 mm 保留 PhysX 数值余量，同时视觉和
// 约束几何上均等价于贴合吸附，而不是挤压 Cube。
constexpr double kSideContactGap = 0.001;
constexpr double kPreContactOffsetY = 0.100;
// L 型工具的竖直段在侧向姿态下会向桌面方向占据额外空间。160 mm 抬升在
// COMMON_X_TRAVEL 的中间采样中仍会擦到桌面；提高到 280 mm 后，工具最低点
// 仍保有明确净空。该值是载荷共同运输高度，不改变最终放置高度。
constexpr double kLiftHeight = 0.280;
// 只为释放/退出留出余量：20 mm 仅为 Cube 边长的 1/6，不是长距离推送。
constexpr double kPrePushOffsetX = 0.020;
constexpr double kReleaseGapZ = 0.001;
constexpr double kCartesianStep = 0.002;
constexpr double kMinCartesianFraction = 0.999;
constexpr double kFclSamplePeriod = 0.010;
constexpr double kPlacementTolerance = 0.010;
// 吸附接触的 x/z、左右跨距与间隙都必须在 2 mm 内；否则 CLOSED 也不能说明
// 阵列 Cup 已对应贴合 Cube，禁止进入共同运输。
constexpr double kAttachmentAlignmentTolerance = 0.002;
constexpr double kAttachmentOrientationToleranceDeg = 3.0;
constexpr double kPi = 3.14159265358979323846;
constexpr int kRetries = 3;
// 0.8 kg 共同搬运物在每段末端需要额外收敛时间，避免刚到短推终点就 release。
constexpr int kFinalCommandHold = 200;

struct OfflineTask
{
  const char* id;
  std::size_t cube_index;
  geometry_msgs::msg::Pose target;
};

geometry_msgs::msg::Pose worldPose(double x, double y, double z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.w = 1.0;
  return pose;
}

// 一个 TCP frame 的 +X 始终是吸盘朝向 Cube 的法向；+Z 指向下。
// 左臂位于 -Y，从 -Y 侧吸附且 +X 指向世界 +Y；右臂相反。
geometry_msgs::msg::Pose sidePose(double x, double y, double z, bool left)
{
  auto pose = worldPose(x, y, z);
  constexpr double half_root = 0.7071067811865476;
  if (left)
  {
    pose.orientation.x = half_root;
    pose.orientation.y = half_root;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
  }
  else
  {
    // 右侧同样只用杯面法向定义姿态：局部 +X 指向世界 -Y。Task24-D 通过
    // 加长轻量 L 型 standoff 让 wrist/flange 留在 Cube 外侧，不再依赖此前
    // 90 度绕法向翻转而产生的极端腕部姿态。
    pose.orientation.x = half_root;
    pose.orientation.y = -half_root;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
  }
  return pose;
}

const std::array<OfflineTask, 8> kTasks{{
  {"task24_yz_wall_far_bottom_left", 0, worldPose(0.820, -0.075, kBottomZ)},
  {"task24_yz_wall_far_bottom_right", 1, worldPose(0.820, +0.075, kBottomZ)},
  {"task24_yz_wall_far_upper_left", 2, worldPose(0.820, -0.075, kUpperZ)},
  {"task24_yz_wall_far_upper_right", 3, worldPose(0.820, +0.075, kUpperZ)},
  {"task24_yz_wall_near_bottom_left", 4, worldPose(0.640, -0.075, kBottomZ)},
  {"task24_yz_wall_near_bottom_right", 5, worldPose(0.640, +0.075, kBottomZ)},
  {"task24_yz_wall_near_upper_left", 6, worldPose(0.640, -0.075, kUpperZ)},
  {"task24_yz_wall_near_upper_right", 7, worldPose(0.640, +0.075, kUpperZ)},
}};

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  point.time_from_start.sec = static_cast<std::int32_t>(std::floor(seconds));
  point.time_from_start.nanosec = static_cast<std::uint32_t>(std::llround(
    (seconds - std::floor(seconds)) * 1e9));
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

std::string formatJointPositions(const std::vector<double>& positions)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << "[";
  for (std::size_t index = 0; index < positions.size(); ++index)
  {
    if (index > 0)
    {
      stream << ", ";
    }
    stream << positions[index];
  }
  stream << "]";
  return stream.str();
}

// 侧面双吸盘的最终 CONTACT 必须两臂都到位，但两条从 PRE_CONTACT 到 CONTACT
// 的短 Cartesian 进给若严格同一时刻开始，局部肘部扫掠会偶发相交。这里使用
// 可验证的微时序：左臂先进入，右臂保持 PRE_CONTACT；随后右臂进入，左臂保持
// CONTACT。两杯均接触后仍同步 ON，并从 COMMON_LIFT 开始执行真正的紧协调。
// 这不是等待区或长距离串行搬运，只是接触建立阶段的安全时序。
trajectory_msgs::msg::JointTrajectory holdTrajectory(
  const trajectory_msgs::msg::JointTrajectory& reference,
  const std::vector<double>& positions, double duration)
{
  trajectory_msgs::msg::JointTrajectory hold;
  hold.joint_names = reference.joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint begin;
  begin.positions = positions;
  setPointTime(begin, 0.0);
  trajectory_msgs::msg::JointTrajectoryPoint end = begin;
  setPointTime(end, std::max(0.05, duration));
  hold.points = {begin, end};
  return hold;
}

std::vector<double> interpolate(
  const trajectory_msgs::msg::JointTrajectory& trajectory, double time_sec)
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
    const auto& first = trajectory.points[index - 1];
    const auto& second = trajectory.points[index];
    if (time_sec > pointTime(second))
    {
      continue;
    }
    const double denominator = pointTime(second) - pointTime(first);
    const double alpha = denominator > 1e-9 ?
      std::clamp((time_sec - pointTime(first)) / denominator, 0.0, 1.0) : 0.0;
    std::vector<double> positions(first.positions.size());
    for (std::size_t joint = 0; joint < positions.size(); ++joint)
    {
      positions[joint] = first.positions[joint] +
        alpha * (second.positions[joint] - first.positions[joint]);
    }
    return positions;
  }
  return trajectory.points.back().positions;
}

double distance3d(const geometry_msgs::msg::Point& first, const geometry_msgs::msg::Point& second)
{
  return std::hypot(std::hypot(first.x - second.x, first.y - second.y), first.z - second.z);
}

moveit_msgs::msg::CollisionObject cubeObject(
  const std::string& id, const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {kCubeSize, kCubeSize, kCubeSize};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

moveit_msgs::msg::CollisionObject tableObject()
{
  auto pose = worldPose(0.55, 0.0, 0.025);
  auto object = cubeObject("task24_table", pose);
  object.primitives.front().dimensions = {1.20, 0.80, 0.050};
  return object;
}

bool copyRobotDescriptions(const rclcpp::Node::SharedPtr& node)
{
  // 在本节点加入工作 executor 前完成参数复制。SyncParametersClient 会自行
  // 驱动这个短请求；这也是 Task11--23 已验证的启动形式。
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task24 无法连接 /move_group。");
    return false;
  }
  const auto values = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (values.size() != 2 ||
      values[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      values[1].get_type() != rclcpp::ParameterType::PARAMETER_STRING)
  {
    RCLCPP_ERROR(node->get_logger(), "Task24 未从 /move_group 读取到有效 RobotModel。");
    return false;
  }
  node->declare_parameter<std::string>("robot_description", values[0].as_string());
  node->declare_parameter<std::string>("robot_description_semantic", values[1].as_string());
  return true;
}

class CubeBuffer
{
public:
  CubeBuffer(const rclcpp::Node::SharedPtr& node, std::size_t expected_count)
    : expected_count_(expected_count)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseArray>(
      "/task24/cube_poses", 10,
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message)
      {
        if (message->poses.size() != expected_count_)
        {
          return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        poses_ = message->poses;
        ++revision_;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec), [this]() { return ready_; });
  }

  std::pair<geometry_msgs::msg::Pose, std::uint64_t> get(std::size_t index) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {poses_.at(index), revision_};
  }

  bool waitNew(std::size_t index, std::uint64_t previous, double timeout_sec,
               geometry_msgs::msg::Pose* output) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
                             [this, previous]() { return revision_ > previous; }))
    {
      return false;
    }
    *output = poses_.at(index);
    return true;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  std::uint64_t revision_{0};
  std::size_t expected_count_{0};
  std::vector<geometry_msgs::msg::Pose> poses_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

// Isaac Bridge 在同一个 physics tick 发布 Cube 与两个 side_suction_tcp Pose。
// Task24 不将 CLOSED 当成“抓正了”：必须用这三个 Ground Truth 显式验证。
class TcpBuffer
{
public:
  TcpBuffer(const rclcpp::Node::SharedPtr& node, const std::string& topic)
  {
    subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic, 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pose_ = message->pose;
        ready_ = true;
        condition_.notify_all();
      });
  }

  bool wait(double timeout_sec) const
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, std::chrono::duration<double>(timeout_sec),
      [this]() { return ready_; });
  }

  geometry_msgs::msg::Pose get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool ready_{false};
  geometry_msgs::msg::Pose pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

double orientationAngleDeg(const geometry_msgs::msg::Quaternion& orientation)
{
  const double norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (norm <= 1e-9)
  {
    return 180.0;
  }
  return 2.0 * std::acos(std::clamp(std::abs(orientation.w) / norm, 0.0, 1.0)) * 180.0 / kPi;
}

bool validateDualSideAttachment(
  const rclcpp::Node::SharedPtr& node, const OfflineTask& task,
  const geometry_msgs::msg::Pose& cube, const geometry_msgs::msg::Pose& left_tcp,
  const geometry_msgs::msg::Pose& right_tcp, const std::string& phase)
{
  const double expected_span = kCubeSize + 2.0 * kSideContactGap;
  const double left_gap = cube.position.y - left_tcp.position.y - kCubeHalf;
  const double right_gap = right_tcp.position.y - cube.position.y - kCubeHalf;
  const double span_error = std::abs((right_tcp.position.y - left_tcp.position.y) - expected_span);
  const double x_mismatch = std::max({
    std::abs(cube.position.x - left_tcp.position.x),
    std::abs(cube.position.x - right_tcp.position.x),
    std::abs(left_tcp.position.x - right_tcp.position.x)});
  const double z_mismatch = std::max({
    std::abs(cube.position.z - left_tcp.position.z),
    std::abs(cube.position.z - right_tcp.position.z),
    std::abs(left_tcp.position.z - right_tcp.position.z)});
  const double gap_error = std::max(
    std::abs(left_gap - kSideContactGap), std::abs(right_gap - kSideContactGap));
  const double tilt_deg = orientationAngleDeg(cube.orientation);

  RCLCPP_INFO(node->get_logger(),
    "%s %s ATTACHMENT: x=%.3f mm z=%.3f mm span=%.3f mm gap=%.3f mm tilt=%.3f deg.",
    task.id, phase.c_str(), x_mismatch * 1000.0, z_mismatch * 1000.0,
    span_error * 1000.0, gap_error * 1000.0, tilt_deg);
  if (x_mismatch > kAttachmentAlignmentTolerance ||
      z_mismatch > kAttachmentAlignmentTolerance ||
      span_error > kAttachmentAlignmentTolerance ||
      gap_error > kAttachmentAlignmentTolerance ||
      tilt_deg > kAttachmentOrientationToleranceDeg)
  {
    RCLCPP_ERROR(node->get_logger(),
      "%s %s: dual side attachment is not corresponding; stop before a skewed transport.",
      task.id, phase.c_str());
    return false;
  }
  return true;
}

class Arm
{
public:
  Arm(const rclcpp::Node::SharedPtr& node, bool left, double time_scale)
    : node_(node), left_(left), side_(left ? "left" : "right"),
      group_(left ? "left_arm" : "right_arm"),
      eef_(left ? "left_fr3_side_suction_tcp" : "right_fr3_side_suction_tcp"),
      prefix_(left ? "left_" : "right_"), time_scale_(time_scale)
  {
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/" + side_ + "/joint_command", 10);
    suction_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/task24/" + side_ + "/suction_command", 10);
    state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/task24/" + side_ + "/suction_state", 10,
      [this](const std_msgs::msg::Bool::SharedPtr state)
      {
        seen_state_.store(true);
        closed_.store(state->data);
      });
  }

  const std::string& groupName() const { return group_; }
  const std::string& eefLink() const { return eef_; }
  bool isLeft() const { return left_; }
  bool isClosed() const { return closed_.load(); }

  bool waitBridge() const
  {
    for (int attempt = 0; attempt < 100; ++attempt)
    {
      if (joint_pub_->get_subscription_count() > 0 && suction_pub_->get_subscription_count() > 0 &&
          seen_state_.load())
      {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  bool planPoseCandidates(moveit::planning_interface::MoveGroupInterface& group,
                          const geometry_msgs::msg::Pose& target, const std::string& label,
                          std::vector<trajectory_msgs::msg::JointTrajectory>* outputs) const
  {
    configure(group);
    group.clearPoseTargets();
    if (!group.setPoseTarget(target, eef_))
    {
      return false;
    }
    // RRTConnect 对同一个 TCP pose 可能给出不同的冗余腕部构型。空载阶段
    // 主动采样三个候选并选择关节累计位移最小者，避免偶发选择绕过关节极限的
    // 大回环解；这不是放宽碰撞约束，MoveIt 仍逐候选做碰撞检查。
    std::vector<std::pair<double, trajectory_msgs::msg::JointTrajectory>> candidates;
    for (int attempt = 1; attempt <= kRetries; ++attempt)
    {
      group.setStartStateToCurrentState();
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
          !plan.trajectory_.joint_trajectory.points.empty())
      {
        const auto& candidate = plan.trajectory_.joint_trajectory;
        double cost = 0.0;
        for (std::size_t point = 1; point < candidate.points.size(); ++point)
        {
          for (std::size_t joint = 0; joint < candidate.points[point].positions.size(); ++joint)
          {
            cost += std::abs(candidate.points[point].positions[joint] -
              candidate.points[point - 1].positions[joint]);
          }
        }
        RCLCPP_INFO(node_->get_logger(),
          "%s %s RRTConnect candidate attempt=%d/%d points=%zu joint_travel=%.3f.",
          side_.c_str(), label.c_str(), attempt, kRetries, candidate.points.size(), cost);
        candidates.emplace_back(cost, candidate);
        continue;
      }
      RCLCPP_WARN(node_->get_logger(), "%s %s RRTConnect failed attempt=%d/%d.",
        side_.c_str(), label.c_str(), attempt, kRetries);
    }
    if (candidates.empty())
    {
      return false;
    }
    std::sort(candidates.begin(), candidates.end(),
      [](const auto& first, const auto& second) { return first.first < second.first; });
    outputs->clear();
    for (auto& [cost, candidate] : candidates)
    {
      (void)cost;
      ensureTiming(candidate);
      outputs->push_back(std::move(candidate));
    }
    RCLCPP_INFO(node_->get_logger(), "%s %s RRTConnect candidates ready=%zu shortest_joint_travel=%.3f.",
      side_.c_str(), label.c_str(), outputs->size(), candidates.front().first);
    return true;
  }

  bool planPose(moveit::planning_interface::MoveGroupInterface& group,
                const geometry_msgs::msg::Pose& target, const std::string& label,
                trajectory_msgs::msg::JointTrajectory* output) const
  {
    std::vector<trajectory_msgs::msg::JointTrajectory> candidates;
    if (!planPoseCandidates(group, target, label, &candidates))
    {
      return false;
    }
    *output = std::move(candidates.front());
    RCLCPP_INFO(node_->get_logger(), "%s %s RRTConnect selected points=%zu.",
      side_.c_str(), label.c_str(), output->points.size());
    return true;
  }

  bool executeAt(const trajectory_msgs::msg::JointTrajectory& input,
                 const std::chrono::steady_clock::time_point& start) const
  {
    if (input.points.empty() || input.joint_names.empty())
    {
      return false;
    }
    auto trajectory = input;
    ensureTiming(trajectory);
    std::vector<std::string> names;
    for (const auto& name : trajectory.joint_names)
    {
      names.push_back(name.rfind(prefix_, 0) == 0 ? name.substr(prefix_.size()) : name);
    }
    std::this_thread::sleep_until(start);
    const auto publish = [&](const std::vector<double>& positions)
    {
      sensor_msgs::msg::JointState command;
      command.header.stamp = node_->now();
      command.name = names;
      command.position = positions;
      joint_pub_->publish(command);
    };
    std::size_t segment = 0;
    const double duration = pointTime(trajectory.points.back());
    while (true)
    {
      const double logical_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count() / time_scale_;
      if (logical_time > duration)
      {
        break;
      }
      while (segment + 1 < trajectory.points.size() &&
             pointTime(trajectory.points[segment + 1]) < logical_time)
      {
        ++segment;
      }
      publish(interpolate(trajectory, logical_time));
      std::this_thread::sleep_for(10ms);
    }
    for (int repeat = 0; repeat < kFinalCommandHold; ++repeat)
    {
      publish(trajectory.points.back().positions);
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void suction(bool enabled) const
  {
    std_msgs::msg::Bool message;
    message.data = enabled;
    for (int repeat = 0; repeat < 20; ++repeat)
    {
      suction_pub_->publish(message);
      std::this_thread::sleep_for(10ms);
    }
  }

  bool waitSuction(bool expected, double timeout_sec) const
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline)
    {
      if (seen_state_.load() && closed_.load() == expected)
      {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

private:
  void configure(moveit::planning_interface::MoveGroupInterface& group) const
  {
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(5.0);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(0.12);
    group.setMaxAccelerationScalingFactor(0.12);
    group.setPoseReferenceFrame("world");
    group.setEndEffectorLink(eef_);
  }

  rclcpp::Node::SharedPtr node_;
  bool left_;
  std::string side_;
  std::string group_;
  std::string eef_;
  std::string prefix_;
  double time_scale_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr suction_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  std::atomic_bool seen_state_{false};
  std::atomic_bool closed_{false};
};

bool executeSync(const Arm& left, const trajectory_msgs::msg::JointTrajectory& left_trajectory,
                 const Arm& right, const trajectory_msgs::msg::JointTrajectory& right_trajectory)
{
  const auto start = std::chrono::steady_clock::now() + 100ms;
  bool left_ok = false;
  bool right_ok = false;
  std::thread left_thread([&]() { left_ok = left.executeAt(left_trajectory, start); });
  std::thread right_thread([&]() { right_ok = right.executeAt(right_trajectory, start); });
  left_thread.join();
  right_thread.join();
  return left_ok && right_ok;
}

bool synchronize(trajectory_msgs::msg::JointTrajectory* first, trajectory_msgs::msg::JointTrajectory* second)
{
  ensureTiming(*first);
  ensureTiming(*second);
  if (first->points.empty() || second->points.empty())
  {
    return false;
  }
  const double duration = std::max(pointTime(first->points.back()), pointTime(second->points.back()));
  for (auto* trajectory : {first, second})
  {
    const double original = pointTime(trajectory->points.back());
    if (original <= 1e-9)
    {
      return false;
    }
    for (auto& point : trajectory->points)
    {
      setPointTime(point, pointTime(point) * duration / original);
    }
  }
  return true;
}

bool validateSync(
  const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& model,
  const std::vector<moveit_msgs::msg::CollisionObject>& world,
  const trajectory_msgs::msg::JointTrajectory& left, const trajectory_msgs::msg::JointTrajectory& right,
  const std::string& label)
{
  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  for (const auto& object : world)
  {
    if (!scene->processCollisionObjectMsg(object))
    {
      RCLCPP_ERROR(node->get_logger(), "%s cannot load world object %s into FCL.", label.c_str(), object.id.c_str());
      return false;
    }
  }
  const double duration = std::max(pointTime(left.points.back()), pointTime(right.points.back()));
  const std::size_t samples = static_cast<std::size_t>(std::ceil(duration / kFclSamplePeriod));
  for (std::size_t index = 0; index <= samples; ++index)
  {
    const auto left_q = interpolate(left, std::min(duration, index * kFclSamplePeriod));
    const auto right_q = interpolate(right, std::min(duration, index * kFclSamplePeriod));
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(left.joint_names, left_q);
    state.setVariablePositions(right.joint_names, right_q);
    state.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, state);
    if (result.collision)
    {
      const double collision_time = std::min(duration, index * kFclSamplePeriod);
      RCLCPP_ERROR(node->get_logger(), "%s FCL collision at t=%.3f s.",
        label.c_str(), collision_time);
      for (const auto& [pair, contacts] : result.contacts)
      {
        (void)contacts;
        RCLCPP_ERROR(node->get_logger(), "%s FCL pair: %s <-> %s.",
          label.c_str(), pair.first.c_str(), pair.second.c_str());
        // 记录碰撞采样点的 link 原点，区分“目标姿态本身过低”和“Cartesian
        // IK 分支在中途下探”。这只是诊断，不放宽任何碰撞规则。
        for (const auto& name : {pair.first, pair.second})
        {
          if (model->hasLinkModel(name))
          {
            const auto& p = state.getGlobalLinkTransform(name).translation();
            RCLCPP_ERROR(node->get_logger(), "%s FCL link %s origin=(%.3f, %.3f, %.3f).",
              label.c_str(), name.c_str(), p.x(), p.y(), p.z());
          }
        }
      }
      return false;
    }
  }
  RCLCPP_INFO(node->get_logger(), "%s synchronized FCL PASS, samples=%zu.", label.c_str(), samples + 1);
  return true;
}

bool planCommonCartesian(
  const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& group,
  const std::string& own_group, const std::string& partner_group,
  const std::vector<double>& own_start, const std::vector<double>& partner_start,
  const geometry_msgs::msg::Pose& target, const std::string& label,
  trajectory_msgs::msg::JointTrajectory* output,
  bool avoid_collisions = false)
{
  const auto model = group.getRobotModel();
  const auto* own = model->getJointModelGroup(own_group);
  const auto* partner = model->getJointModelGroup(partner_group);
  if (!own || !partner || own_start.empty() || partner_start.empty())
  {
    return false;
  }
  for (int attempt = 1; attempt <= kRetries; ++attempt)
  {
    auto state = group.getCurrentState(2.0);
    if (!state)
    {
      return false;
    }
    state->setJointGroupPositions(own, own_start);
    state->setJointGroupPositions(partner, partner_start);
    state->update();
    group.setStartState(*state);
    moveit_msgs::msg::RobotTrajectory candidate;
    moveit_msgs::msg::MoveItErrorCodes error;
    // 共同搬运阶段的完整双臂 FCL 在候选同步后进行；不能把搭档臂冻结在阶段
    // 起点而误判。但空载接触阶段必须同时避开桌面，故由调用点显式开启。
    const double fraction = group.computeCartesianPath(
      {target}, kCartesianStep, 0.0, candidate, avoid_collisions, &error);
    RCLCPP_INFO(node->get_logger(), "%s Cartesian fraction=%.4f error=%d attempt=%d/%d.",
      label.c_str(), fraction, error.val, attempt, kRetries);
    if (fraction >= kMinCartesianFraction && !candidate.joint_trajectory.points.empty())
    {
      *output = candidate.joint_trajectory;
      ensureTiming(*output);
      return true;
    }
  }
  return false;
}

std::vector<moveit_msgs::msg::CollisionObject> staticWorld(
  moveit::planning_interface::PlanningSceneInterface& scene)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& [id, object] : scene.getObjects())
  {
    (void)id;
    objects.push_back(object);
  }
  return objects;
}

bool planAndCheckCommon(
  const rclcpp::Node::SharedPtr& node, moveit::planning_interface::MoveGroupInterface& left_group,
  moveit::planning_interface::MoveGroupInterface& right_group, const Arm& left, const Arm& right,
  const std::vector<double>& left_start, const std::vector<double>& right_start,
  const geometry_msgs::msg::Pose& left_target, const geometry_msgs::msg::Pose& right_target,
  const std::vector<moveit_msgs::msg::CollisionObject>& world, const std::string& stage,
  trajectory_msgs::msg::JointTrajectory* left_output, trajectory_msgs::msg::JointTrajectory* right_output)
{
  if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(), left_start, right_start,
                           left_target, stage + " left", left_output, true) ||
      !planCommonCartesian(node, right_group, right.groupName(), left.groupName(), right_start, left_start,
                           right_target, stage + " right", right_output, true) ||
      !synchronize(left_output, right_output) ||
      !validateSync(node, left_group.getRobotModel(), world, *left_output, *right_output, stage))
  {
    return false;
  }
  return true;
}

bool validPlacement(const rclcpp::Node::SharedPtr& node, const OfflineTask& task,
                    const geometry_msgs::msg::Pose& actual)
{
  const double error = distance3d(task.target.position, actual.position);
  RCLCPP_INFO(node->get_logger(), "%s final Ground Truth: expected=(%.3f, %.3f, %.3f), "
    "actual=(%.3f, %.3f, %.3f), error=%.3f mm.", task.id,
    task.target.position.x, task.target.position.y, task.target.position.z,
    actual.position.x, actual.position.y, actual.position.z, error * 1000.0);
  return error <= kPlacementTolerance;
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("task24_side_suction_tight");
  const int requested = node->declare_parameter<int>("max_cubes", 1);
  // Task24-F 默认严格为单件；恢复多件时必须同时恢复 Isaac Scene、bridge 与
  // MoveIt 中所有 Cube CollisionObject，不能仅提高这个参数。
  const int active_cube_count = node->declare_parameter<int>("active_cube_count", 1);
  const double time_scale = node->declare_parameter<double>("execution_time_scale", 3.0);
  if (requested < 1 || requested > active_cube_count ||
      active_cube_count < 1 || active_cube_count > static_cast<int>(kTasks.size()) ||
      time_scale < 1.0 ||
      !copyRobotDescriptions(node))
  {
    rclcpp::shutdown();
    return 1;
  }

  CubeBuffer cubes(node, static_cast<std::size_t>(active_cube_count));
  TcpBuffer left_tcp(node, "/task24/left/side_suction_tcp_pose");
  TcpBuffer right_tcp(node, "/task24/right/side_suction_tcp_pose");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin([&executor]() { executor.spin(); });
  bool success = false;
  do
  {
    RCLCPP_INFO(node->get_logger(),
      "========== Task24 OFFLINE SIDE-SUCTION TIGHT: requested=%d, active_scene_cubes=%d, wall-first YZ->X, time_scale=%.2f ==========" ,
      requested, active_cube_count, time_scale);
    if (!cubes.wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "Task24 等待 /task24/cube_poses 超时。");
      break;
    }
    if (!left_tcp.wait(10.0) || !right_tcp.wait(10.0))
    {
      RCLCPP_ERROR(node->get_logger(), "Task24 等待侧吸盘 TCP Ground Truth 超时。");
      break;
    }
    Arm left(node, true, time_scale);
    Arm right(node, false, time_scale);
    if (!left.waitBridge() || !right.waitBridge())
    {
      RCLCPP_ERROR(node->get_logger(), "Task24 Isaac side-suction bridge 未就绪。");
      break;
    }
    moveit::planning_interface::MoveGroupInterface left_group(node, left.groupName());
    moveit::planning_interface::MoveGroupInterface right_group(node, right.groupName());
    left_group.setEndEffectorLink(left.eefLink());
    right_group.setEndEffectorLink(right.eefLink());
    moveit::planning_interface::PlanningSceneInterface scene;
    std::vector<moveit_msgs::msg::CollisionObject> initial{tableObject()};
    const std::size_t scene_cube_count = static_cast<std::size_t>(active_cube_count);
    for (std::size_t index = 0; index < scene_cube_count; ++index)
    {
      const auto [pose, revision] = cubes.get(index);
      (void)revision;
      initial.push_back(cubeObject("task24_cube_" + std::to_string(index + 1), pose));
    }
    scene.removeCollisionObjects({
      "task24_table", "task24_cube_1", "task24_cube_2", "task24_cube_3", "task24_cube_4",
      "task24_cube_5", "task24_cube_6", "task24_cube_7", "task24_cube_8"});
    std::this_thread::sleep_for(300ms);
    if (!scene.applyCollisionObjects(initial))
    {
      RCLCPP_ERROR(node->get_logger(), "Task24 无法初始化 MoveIt Planning Scene。");
      break;
    }
    std::this_thread::sleep_for(500ms);

    bool all_complete = true;
    for (int task_index = 0; task_index < requested; ++task_index)
    {
      const auto& task = kTasks.at(static_cast<std::size_t>(task_index));
      const std::string object_id = "task24_cube_" + std::to_string(task.cube_index + 1);
      const auto [source, source_revision] = cubes.get(task.cube_index);
      RCLCPP_INFO(node->get_logger(), "---------- %s: source=(%.3f, %.3f, %.3f), target=(%.3f, %.3f, %.3f) ----------",
        task.id, source.position.x, source.position.y, source.position.z,
        task.target.position.x, task.target.position.y, task.target.position.z);

      const double initial_left_contact_y = source.position.y - kCubeHalf - kSideContactGap;
      const double initial_right_contact_y = source.position.y + kCubeHalf + kSideContactGap;
      // 大 Cube 不能让两臂同时在低位 PRE_CONTACT：先到位的手会占据另一只手的
      // OMPL 接近通道。两臂先在共同 lift 高度的两侧待命，再以受 FCL 门禁的
      // 左->右微时序下降至 CONTACT；真正负载阶段仍从 COMMON_LIFT 起严格同步。
      const double pre_contact_z = source.position.z + kLiftHeight;
      trajectory_msgs::msg::JointTrajectory left_pre;
      if (!left.planPose(left_group,
            sidePose(source.position.x, initial_left_contact_y - kPreContactOffsetY, pre_contact_z, true),
            std::string(task.id) + " LEFT HIGH_PRE_CONTACT", &left_pre) || !left.executeAt(left_pre, std::chrono::steady_clock::now()))
      {
        all_complete = false;
        break;
      }
      trajectory_msgs::msg::JointTrajectory right_pre;
      if (!right.planPose(right_group,
            sidePose(source.position.x, initial_right_contact_y + kPreContactOffsetY, pre_contact_z, false),
            std::string(task.id) + " RIGHT HIGH_PRE_CONTACT", &right_pre) || !right.executeAt(right_pre, std::chrono::steady_clock::now()))
      {
        all_complete = false;
        break;
      }

      scene.removeCollisionObjects({object_id});
      std::this_thread::sleep_for(250ms);
      const auto world_after_remove = staticWorld(scene);
      trajectory_msgs::msg::JointTrajectory left_contact;
      trajectory_msgs::msg::JointTrajectory right_contact;
      // Contact 建立使用确定的左->右微时序。不能从 HIGH_PRE_CONTACT 斜向
      // 直接插入：长 L 型支臂会使 IK 在下降过程中绕到桌面下方。先在供料
      // 外侧作纯 Z 下降，再作纯 Y 靠近；两段均启用桌面碰撞检查。
      const auto left_low_pre = sidePose(
        source.position.x, initial_left_contact_y - kPreContactOffsetY, source.position.z, true);
      trajectory_msgs::msg::JointTrajectory left_outer_descent;
      trajectory_msgs::msg::JointTrajectory right_outer_descent;
      if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
            finalPositions(left_pre), finalPositions(right_pre),
            left_low_pre, std::string(task.id) + " LEFT_OUTER_DESCENT", &left_outer_descent, true))
      {
        all_complete = false;
        break;
      }
      auto right_hold_pre = holdTrajectory(
        right_pre, finalPositions(right_pre), pointTime(left_outer_descent.points.back()));
      if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
            left_outer_descent, right_hold_pre, std::string(task.id) + " LEFT_OUTER_DESCENT") ||
          !executeSync(left, left_outer_descent, right, right_hold_pre))
      {
        all_complete = false;
        break;
      }

      if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
            finalPositions(left_outer_descent), finalPositions(right_pre),
            sidePose(source.position.x, initial_left_contact_y, source.position.z, true),
            std::string(task.id) + " LEFT_CONTACT", &left_contact, true))
      {
        all_complete = false;
        break;
      }
      right_hold_pre = holdTrajectory(
        right_pre, finalPositions(right_pre), pointTime(left_contact.points.back()));
      if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
            left_contact, right_hold_pre, std::string(task.id) + " LEFT_CONTACT") ||
          !executeSync(left, left_contact, right, right_hold_pre))
      {
        all_complete = false;
        break;
      }

      // 左臂已经到达侧面后，Cube 仍是 Dynamic Rigid Body。不能再使用任务起始
      // 时的静态 source：即使没有吸附，接近气流/接触也可能造成毫米级漂移。
      // 右臂的目标必须由这一帧 Isaac Ground Truth 重新计算。
      std::this_thread::sleep_for(250ms);
      const auto [cube_after_left_contact, left_contact_revision] = cubes.get(task.cube_index);
      (void)left_contact_revision;
      const double right_live_contact_y =
        cube_after_left_contact.position.y + kCubeHalf + kSideContactGap;
      // 空载的右侧外部接近可以使用 RRTConnect。对三个候选，先在机器人尚未
      // 吸附 Cube 时预演后续的短接触和 COMMON_LIFT；只有能通过完整 FCL 的
      // 候选才会发送任何右臂命令。这样不会把“同一 TCP 的偶发绕腕解”留到
      // 吸附后才暴露为桌面碰撞。
      std::vector<trajectory_msgs::msg::JointTrajectory> right_outer_candidates;
      if (!right.planPoseCandidates(right_group,
            sidePose(cube_after_left_contact.position.x,
              right_live_contact_y + kPreContactOffsetY, cube_after_left_contact.position.z, false),
            std::string(task.id) + " RIGHT_OUTER_APPROACH", &right_outer_candidates))
      {
        all_complete = false;
        break;
      }
      bool right_candidate_safe = false;
      for (std::size_t candidate_index = 0; candidate_index < right_outer_candidates.size(); ++candidate_index)
      {
        auto candidate_outer = right_outer_candidates[candidate_index];
        auto candidate_left_hold = holdTrajectory(
          left_contact, finalPositions(left_contact), pointTime(candidate_outer.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_left_hold, candidate_outer,
              std::string(task.id) + " RIGHT_OUTER_CANDIDATE_" + std::to_string(candidate_index + 1)))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_contact;
        if (!planCommonCartesian(node, right_group, right.groupName(), left.groupName(),
              finalPositions(candidate_outer), finalPositions(left_contact),
              sidePose(cube_after_left_contact.position.x, right_live_contact_y,
                cube_after_left_contact.position.z, false),
              std::string(task.id) + " RIGHT_CONTACT_CANDIDATE_" + std::to_string(candidate_index + 1),
              &candidate_contact, true))
        {
          continue;
        }
        candidate_left_hold = holdTrajectory(
          left_contact, finalPositions(left_contact), pointTime(candidate_contact.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_left_hold, candidate_contact,
              std::string(task.id) + " RIGHT_CONTACT_CANDIDATE_" + std::to_string(candidate_index + 1)))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_left_lift;
        trajectory_msgs::msg::JointTrajectory candidate_right_lift;
        const double candidate_left_y = cube_after_left_contact.position.y - kCubeHalf - kSideContactGap;
        const double candidate_right_y = cube_after_left_contact.position.y + kCubeHalf + kSideContactGap;
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(candidate_contact),
              sidePose(cube_after_left_contact.position.x, candidate_left_y,
                cube_after_left_contact.position.z + kLiftHeight, true),
              sidePose(cube_after_left_contact.position.x, candidate_right_y,
                cube_after_left_contact.position.z + kLiftHeight, false),
              world_after_remove,
              std::string(task.id) + " RIGHT_CANDIDATE_COMMON_LIFT_" + std::to_string(candidate_index + 1),
              &candidate_left_lift, &candidate_right_lift))
        {
          continue;
        }

        // 不只检查 LIFT：侧向工具在随后 Y 对齐的 IK 插值中也可能下探到
        // 桌面。闭合前把完整 Z->X->Y->Z->X->Z 负载链路都预演完毕，任何一
        // 段失败就换下一个空载 RRT 候选，不会让物理 Cube 进入危险构型。
        const double candidate_entry_x = task.target.position.x - kPrePushOffsetX;
        const double candidate_release_z = task.target.position.z + kReleaseGapZ;
        const double candidate_transport_z = cube_after_left_contact.position.z + kLiftHeight;
        const double candidate_entry_left_y = task.target.position.y - kCubeHalf - kSideContactGap;
        const double candidate_entry_right_y = task.target.position.y + kCubeHalf + kSideContactGap;
        trajectory_msgs::msg::JointTrajectory candidate_left_x, candidate_right_x;
        trajectory_msgs::msg::JointTrajectory candidate_left_y_path, candidate_right_y_path;
        trajectory_msgs::msg::JointTrajectory candidate_left_descent, candidate_right_descent;
        trajectory_msgs::msg::JointTrajectory candidate_left_push, candidate_right_push;
        trajectory_msgs::msg::JointTrajectory candidate_left_retreat, candidate_right_retreat;
        const std::string candidate_prefix = std::string(task.id) + " RIGHT_CANDIDATE_" +
          std::to_string(candidate_index + 1);
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_lift), finalPositions(candidate_right_lift),
              sidePose(candidate_entry_x, candidate_left_y, candidate_transport_z, true),
              sidePose(candidate_entry_x, candidate_right_y, candidate_transport_z, false),
              world_after_remove, candidate_prefix + " COMMON_X_TRAVEL",
              &candidate_left_x, &candidate_right_x) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_x), finalPositions(candidate_right_x),
              sidePose(candidate_entry_x, candidate_entry_left_y, candidate_transport_z, true),
              sidePose(candidate_entry_x, candidate_entry_right_y, candidate_transport_z, false),
              world_after_remove, candidate_prefix + " COMMON_Y_ALIGN",
              &candidate_left_y_path, &candidate_right_y_path) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_y_path), finalPositions(candidate_right_y_path),
              sidePose(candidate_entry_x, candidate_entry_left_y, candidate_release_z, true),
              sidePose(candidate_entry_x, candidate_entry_right_y, candidate_release_z, false),
              world_after_remove, candidate_prefix + " COMMON_DESCENT_TO_ENTRY",
              &candidate_left_descent, &candidate_right_descent) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_descent), finalPositions(candidate_right_descent),
              sidePose(task.target.position.x, candidate_entry_left_y, candidate_release_z, true),
              sidePose(task.target.position.x, candidate_entry_right_y, candidate_release_z, false),
              world_after_remove, candidate_prefix + " COMMON_SIDE_SHORT_PUSH",
              &candidate_left_push, &candidate_right_push) ||
            !planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(candidate_left_push), finalPositions(candidate_right_push),
              sidePose(task.target.position.x, candidate_entry_left_y,
                candidate_release_z + kLiftHeight, true),
              sidePose(task.target.position.x, candidate_entry_right_y,
                candidate_release_z + kLiftHeight, false),
              world_after_remove, candidate_prefix + " PREPLANNED_COMMON_RETREAT",
              &candidate_left_retreat, &candidate_right_retreat))
        {
          continue;
        }

        right_outer_descent = std::move(candidate_outer);
        right_contact = std::move(candidate_contact);
        right_candidate_safe = true;
        RCLCPP_INFO(node->get_logger(), "%s selected right outer candidate=%zu/%zu after COMMON_LIFT preflight.",
          task.id, candidate_index + 1, right_outer_candidates.size());
        break;
      }
      if (!right_candidate_safe)
      {
        RCLCPP_ERROR(node->get_logger(), "%s no right-side empty-arm candidate passed CONTACT + COMMON_LIFT preflight.",
          task.id);
        all_complete = false;
        break;
      }

      RCLCPP_INFO(node->get_logger(), "%s RIGHT_CONTACT final command q=%s.",
        task.id, formatJointPositions(finalPositions(right_outer_descent)).c_str());
      auto left_hold_contact = holdTrajectory(
        left_contact, finalPositions(left_contact), pointTime(right_outer_descent.points.back()));
      if (!executeSync(left, left_hold_contact, right, right_outer_descent))
      {
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s RIGHT_CONTACT final command q=%s.",
        task.id, formatJointPositions(finalPositions(right_contact)).c_str());
      left_hold_contact = holdTrajectory(
        left_contact, finalPositions(left_contact), pointTime(right_contact.points.back()));
      if (!executeSync(left, left_hold_contact, right, right_contact))
      {
        all_complete = false;
        break;
      }

      // CLOSED 仅代表 Surface Gripper 创建了约束，不代表两侧都吸在对应位置。
      // 在任何一只吸盘打开前，以 Cube + 左/右 TCP Ground Truth 建立闭环接触
      // 门控；失败时只在未吸附状态下以最新 Cube pose 微调并复测，绝不带着
      // 歪斜几何进入共同抬升。
      bool pre_close_geometry_ok = false;
      geometry_msgs::msg::Pose grasp_pose;
      for (int alignment_attempt = 1; alignment_attempt <= kRetries; ++alignment_attempt)
      {
        std::this_thread::sleep_for(250ms);
        const auto [live_cube, live_revision] = cubes.get(task.cube_index);
        (void)live_revision;
        if (validateDualSideAttachment(node, task, live_cube, left_tcp.get(), right_tcp.get(),
              "PRE_CLOSE_GEOMETRY"))
        {
          grasp_pose = live_cube;
          pre_close_geometry_ok = true;
          break;
        }
        if (alignment_attempt == kRetries)
        {
          break;
        }

        RCLCPP_WARN(node->get_logger(),
          "%s PRE_CLOSE_GEOMETRY retry=%d/%d: no suction constraint exists; reacquire both "
          "TCP targets from the latest Cube Ground Truth.",
          task.id, alignment_attempt, kRetries);
        trajectory_msgs::msg::JointTrajectory left_reacquire;
        trajectory_msgs::msg::JointTrajectory right_reacquire;
        const double live_left_y = live_cube.position.y - kCubeHalf - kSideContactGap;
        const double live_right_y = live_cube.position.y + kCubeHalf + kSideContactGap;
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(right_contact),
              sidePose(live_cube.position.x, live_left_y, live_cube.position.z, true),
              sidePose(live_cube.position.x, live_right_y, live_cube.position.z, false),
              world_after_remove, std::string(task.id) + " PRE_CLOSE_REACQUIRE",
              &left_reacquire, &right_reacquire) ||
            !executeSync(left, left_reacquire, right, right_reacquire))
        {
          RCLCPP_ERROR(node->get_logger(), "%s PRE_CLOSE_REACQUIRE failed.", task.id);
          break;
        }
        left_contact = std::move(left_reacquire);
        right_contact = std::move(right_reacquire);
      }
      if (!pre_close_geometry_ok)
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s cannot establish corresponding dual side contact; do not enable suction.", task.id);
        all_complete = false;
        break;
      }

      // 在物理吸附前先对首个负载段做完整预检。这样若当前空载 RRT 末端
      // 构型会让共同 LIFT 的 Cartesian IK 支路擦桌面，就在两杯仍 OFF 时
      // 停止，而不是建立约束后才发现没有安全的负载轨迹。
      trajectory_msgs::msg::JointTrajectory preflight_left_lift;
      trajectory_msgs::msg::JointTrajectory preflight_right_lift;
      const double preflight_left_y = grasp_pose.position.y - kCubeHalf - kSideContactGap;
      const double preflight_right_y = grasp_pose.position.y + kCubeHalf + kSideContactGap;
      if (!planAndCheckCommon(node, left_group, right_group, left, right,
            finalPositions(left_contact), finalPositions(right_contact),
            sidePose(grasp_pose.position.x, preflight_left_y,
              grasp_pose.position.z + kLiftHeight, true),
            sidePose(grasp_pose.position.x, preflight_right_y,
              grasp_pose.position.z + kLiftHeight, false),
            world_after_remove, std::string(task.id) + " PRE_CLOSE_COMMON_LIFT_PREFLIGHT",
            &preflight_left_lift, &preflight_right_lift))
      {
        RCLCPP_ERROR(node->get_logger(),
          "%s cannot preflight a safe COMMON_LIFT; suction remains OFF.", task.id);
        all_complete = false;
        break;
      }

      std::thread left_on([&]() { left.suction(true); });
      std::thread right_on([&]() { right.suction(true); });
      left_on.join();
      right_on.join();
      if (!left.waitSuction(true, 3.0) || !right.waitSuction(true, 3.0))
      {
        RCLCPP_ERROR(node->get_logger(), "%s side suction did not close on both arms.", task.id);
        left.suction(false);
        right.suction(false);
        all_complete = false;
        break;
      }

      std::this_thread::sleep_for(250ms);
      const auto [attachment_pose, attachment_revision] = cubes.get(task.cube_index);
      (void)attachment_revision;
      if (!validateDualSideAttachment(node, task, attachment_pose, left_tcp.get(), right_tcp.get(),
            "POST_CLOSE"))
      {
        left.suction(false);
        right.suction(false);
        all_complete = false;
        break;
      }

      // 以闭合后真实锁定的 Cube pose 作为所有负载 Cartesian 段的起点，不能再
      // 回退到任务开始时的 source。这样 Z->X->Y->Z 的每段都从同一真实几何出发。
      grasp_pose = attachment_pose;
      const double grasp_left_contact_y =
        grasp_pose.position.y - kCubeHalf - kSideContactGap;
      const double grasp_right_contact_y =
        grasp_pose.position.y + kCubeHalf + kSideContactGap;

      const auto trace_held = [&](const std::string& phase)
      {
        // 让 PhysX 与 ROS Ground Truth 至少完成数帧更新，再记录共同物体真实状态。
        std::this_thread::sleep_for(250ms);
        const auto [pose, revision] = cubes.get(task.cube_index);
        (void)revision;
        RCLCPP_INFO(node->get_logger(),
          "%s %s GT=(%.3f, %.3f, %.3f), suction=(left:%s right:%s).",
          task.id, phase.c_str(), pose.position.x, pose.position.y, pose.position.z,
          left.isClosed() ? "CLOSED" : "OPEN", right.isClosed() ? "CLOSED" : "OPEN");
        if (!left.isClosed() || !right.isClosed())
        {
          RCLCPP_ERROR(node->get_logger(), "%s %s: side suction lost before release.",
            task.id, phase.c_str());
          return false;
        }
        return validateDualSideAttachment(node, task, pose, left_tcp.get(), right_tcp.get(), phase);
      };

      auto stage = [&](const std::string& name, const geometry_msgs::msg::Pose& left_target,
                       const geometry_msgs::msg::Pose& right_target,
                       const trajectory_msgs::msg::JointTrajectory& left_start,
                       const trajectory_msgs::msg::JointTrajectory& right_start,
                       trajectory_msgs::msg::JointTrajectory* left_result,
                       trajectory_msgs::msg::JointTrajectory* right_result)
      {
        return planAndCheckCommon(node, left_group, right_group, left, right,
          finalPositions(left_start), finalPositions(right_start), left_target, right_target,
          world_after_remove, std::string(task.id) + " " + name, left_result, right_result);
      };

      trajectory_msgs::msg::JointTrajectory left_lift, right_lift;
      if (!stage("COMMON_LIFT",
            sidePose(grasp_pose.position.x, grasp_left_contact_y,
              grasp_pose.position.z + kLiftHeight, true),
            sidePose(grasp_pose.position.x, grasp_right_contact_y,
              grasp_pose.position.z + kLiftHeight, false),
            left_contact, right_contact, &left_lift, &right_lift) ||
          !executeSync(left, left_lift, right, right_lift) ||
          !trace_held("COMMON_LIFT"))
      {
        all_complete = false;
        break;
      }

      const double entry_x = task.target.position.x - kPrePushOffsetX;
      const double release_z = task.target.position.z + kReleaseGapZ;
      const double entry_left_y = task.target.position.y - kCubeHalf - kSideContactGap;
      const double entry_right_y = task.target.position.y + kCubeHalf + kSideContactGap;
      const double transport_z = grasp_pose.position.z + kLiftHeight;

      // 吸住 Cube 后不再允许 OMPL 产生任意弯绕路径。共同负载轨迹被显式拆为
      // Z(lift) -> X(travel) -> Y(align) -> Z(descent) -> X(short push)；每段均
      // 在两臂完整 RobotState 上做同步 FCL 检查。
      trajectory_msgs::msg::JointTrajectory left_transport_x, right_transport_x;
      if (!stage("COMMON_X_TRAVEL",
            sidePose(entry_x, grasp_left_contact_y, transport_z, true),
            sidePose(entry_x, grasp_right_contact_y, transport_z, false),
            left_lift, right_lift, &left_transport_x, &right_transport_x) ||
          !executeSync(left, left_transport_x, right, right_transport_x) ||
          !trace_held("COMMON_X_TRAVEL"))
      {
        all_complete = false;
        break;
      }

      trajectory_msgs::msg::JointTrajectory left_transport_y, right_transport_y;
      if (!stage("COMMON_Y_ALIGN",
            sidePose(entry_x, entry_left_y, transport_z, true),
            sidePose(entry_x, entry_right_y, transport_z, false),
            left_transport_x, right_transport_x, &left_transport_y, &right_transport_y) ||
          !executeSync(left, left_transport_y, right, right_transport_y) ||
          !trace_held("COMMON_Y_ALIGN"))
      {
        all_complete = false;
        break;
      }

      trajectory_msgs::msg::JointTrajectory left_descent, right_descent;
      if (!stage("COMMON_DESCENT_TO_ENTRY",
            sidePose(entry_x, entry_left_y, release_z, true),
            sidePose(entry_x, entry_right_y, release_z, false),
            left_transport_y, right_transport_y, &left_descent, &right_descent) ||
          !executeSync(left, left_descent, right, right_descent) ||
          !trace_held("COMMON_DESCENT_TO_ENTRY"))
      {
        all_complete = false;
        break;
      }
      trajectory_msgs::msg::JointTrajectory left_push, right_push;
      if (!stage("COMMON_SIDE_SHORT_PUSH",
            sidePose(task.target.position.x, entry_left_y, release_z, true),
            sidePose(task.target.position.x, entry_right_y, release_z, false),
            left_descent, right_descent, &left_push, &right_push) ||
          !executeSync(left, left_push, right, right_push) ||
          !trace_held("COMMON_SIDE_SHORT_PUSH"))
      {
        all_complete = false;
        break;
      }

      // 物理释放前预规划共同竖直退出；释放落稳并回写 World 后只执行该轨迹。
      trajectory_msgs::msg::JointTrajectory left_retreat, right_retreat;
      if (!stage("PREPLANNED_COMMON_RETREAT",
            sidePose(task.target.position.x, entry_left_y, release_z + kLiftHeight, true),
            sidePose(task.target.position.x, entry_right_y, release_z + kLiftHeight, false),
            left_push, right_push, &left_retreat, &right_retreat))
      {
        all_complete = false;
        break;
      }
      const auto [unused_pose, release_revision] = cubes.get(task.cube_index);
      (void)unused_pose;
      std::thread left_off([&]() { left.suction(false); });
      std::thread right_off([&]() { right.suction(false); });
      left_off.join();
      right_off.join();
      if (!left.waitSuction(false, 3.0) || !right.waitSuction(false, 3.0))
      {
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(1500ms);
      geometry_msgs::msg::Pose settled;
      if (!cubes.waitNew(task.cube_index, std::max(source_revision, release_revision), 2.0, &settled) ||
          !validPlacement(node, task, settled))
      {
        all_complete = false;
        break;
      }
      if (!scene.applyCollisionObject(cubeObject(object_id, settled)))
      {
        all_complete = false;
        break;
      }
      std::this_thread::sleep_for(300ms);
      if (!executeSync(left, left_retreat, right, right_retreat))
      {
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s PASS: dual side suction -> lift -> transport -> descent -> 20 mm short push -> release.", task.id);
    }
    success = all_complete;
  } while (false);

  executor.cancel();
  if (spin.joinable())
  {
    spin.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
