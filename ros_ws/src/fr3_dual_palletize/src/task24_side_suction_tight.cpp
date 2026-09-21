// Task24：短 L 型侧面吸盘的离线紧协调码垛执行器。
//
// 设计边界：不读取选择器，不做在线任务排序。8 件 Cube 按固定离线任务表完成
// “先 YZ 墙、再 X 向第二面墙”的顺序；每一件仍由 Isaac Ground Truth 门禁、MoveIt
// 规划、同步 FCL 与双 Surface Gripper 物理闭环共同验证。该文件复用 Task11--13 的
// 共同 lift / transport / descent / release 原则，但侧吸几何和“墙优先 + 短推”是
// 独立 Task24。任何中心侧吸可达性失败都会明确停止并留下单臂短推后备入口，绝不
// 以移动到侧面边缘的假吸点继续执行。

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
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
// Task24-H：与 Isaac 场景和 MoveIt 环境发布器保持同构。0.200 m 顶面由
// Franka 官方 FR3 move_to_start 姿态及 Duo 手册的高可操作性工作半径确定，
// 不是通过反复试高度得到的经验值。
constexpr double kTableTopZ = 0.200;
constexpr double kBottomZ = kTableTopZ + kCubeHalf;
constexpr double kUpperZ = kBottomZ + kCubeSize;
// 在尚未建立 Surface Gripper D6 约束的 CONTACT 阶段，Cup Collider 不能以
// 0 mm 穿入动态 Cube，否则两个机械臂的接近会先把 Cube 推偏。保留 1 mm 的
// 名义空气隙：远小于 3 mm 捕获阈值，且 Ground Truth 仍要求两侧实际间隙各
// 不大于 2 mm、相差不大于 1 mm；这不是允许悬空吸附。
constexpr double kSideContactCommandGap = 0.001;
// 阵列 TCP 必须对准 Cube 侧面几何中心：四杯以 TCP 为中心镜像分布，才能让
// 杯面完整落在侧面中央区域。Y 向仍严格保持 1 mm 名义贴合间隙，Surface
// Gripper 在 3 mm 门限内闭合。若中心位姿没有可达通道，应先走单臂短推预调整，
// 绝不能把吸点上移到 Cube 边缘来规避碰撞。
constexpr double kSideContactCommandZOffset = 0.000;
// 这两项是 Isaac Ground Truth 的硬门限，不是命令偏置：两个 Cup 必须各自离
// 对应侧面不超过 2 mm，并且左右实际间隙相差不超过 1 mm，才允许 CLOSE。
constexpr double kMaxAttachmentGap = 0.002;
constexpr double kMaxAttachmentGapAsymmetry = 0.001;
constexpr double kMinAttachmentGap = -0.001;
constexpr double kPreContactOffsetY = 0.100;
// L 型工具的竖直段在侧向姿态下会向桌面方向占据额外空间。160 mm 抬升在
// COMMON_X_TRAVEL 的中间采样中仍会擦到桌面；提高到 280 mm 后，工具最低点
// 仍保有明确净空。该值是载荷共同运输高度，不改变最终放置高度。
constexpr double kLiftHeight = 0.280;
// 只为短推的 entry 留出 20 mm X 向余量；不是长距离推送。
constexpr double kPrePushOffsetX = 0.020;
// L 型阵列在侧面接触姿态下的最低实体比 Cup 接触面更低。Cube 以 1 mm
// 的桌面释放余量下降时，阵列本体会在下降路径中擦到桌面。释放高度改为
// 20 mm：仍是低速、短距离的物理落稳，而不是以放宽 ACM 换取可达性。
// 释放后必须以 Isaac Ground Truth 回读并通过 validPlacement()，否则任务失败。
constexpr double kReleaseGapZ = 0.020;
constexpr double kCartesianStep = 0.002;
constexpr double kMinCartesianFraction = 0.999;
constexpr double kFclSamplePeriod = 0.010;
constexpr double kPlacementTolerance = 0.010;
// Isaac 与 MoveIt 的固定 L 型工具 frame 存在约 2.1 mm 的已实测静态标定差；
// x/z 采用 2.5 mm 门限。该门限仍远小于 10 mm Cup 半径，而 Y 向实际贴合间隙
// 与左右对称性继续由上方独立的 2 mm / 1 mm 硬门限约束。否则 CLOSED 也不能
// 说明阵列 Cup 已对应贴合 Cube，禁止进入共同运输。
constexpr double kAttachmentAlignmentTolerance = 0.0025;
constexpr double kAttachmentOrientationToleranceDeg = 3.0;
constexpr double kPi = 3.14159265358979323846;
// 这是空载冗余构型的候选池大小，不是失败重试次数。每个候选都会在完整
// 双臂 FCL 负载链路上筛选；只执行第一个通过者。3 个样本不足以覆盖 FR3
// 在侧向 L 型工具下的肘部翻转分支，因此扩大为 8 个确定上限的样本。
constexpr int kRrtCandidateCount = 8;
// 一个 8 解候选池可能恰好全落在同一类不利的 FR3 冗余腕部构型。右臂还未
// 接触 Cube 时，允许最多三批独立 RRTConnect 采样；每一条仍必须通过完整
// FCL 负载链路，绝不是降低碰撞门限或在失败后盲目执行。
constexpr int kRrtCandidateBatches = 3;
constexpr int kRetries = 3;
// 0.8 kg 共同搬运物在每段末端需要额外收敛时间，避免刚到短推终点就 release。
constexpr int kFinalCommandHold = 200;
// 双侧吸盘共同搬运不是“两个独立轨迹恰好同时开始”。必须由同一控制时钟发布
// 左右命令，且采用零起止速度的共同 S 曲线进度，避免两套 PD 在阶段边界收到
// 不同相位/突变速度命令后给 Cube 施加瞬态扭矩。
constexpr auto kDualCommandPeriod = 10ms;
// 发送最终 command 并不等于 Isaac Articulation 已到达该关节状态。Task24 的
// 刚体接触和 Isaac PD 伺服在静态低位姿会留下约 1.5 deg 的关节稳态残差；这里
// 只把 2 deg 作为“停止继续跟随”的门槛，不能当成几何验收。真正的吸附前和每段
// 搬运后仍必须通过 validateDualSideAttachment() 的 2 mm TCP/Cube 实测门限。
constexpr double kJointSettleToleranceRad = 0.035;
constexpr double kJointSettleTimeoutSec = 12.0;

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
  // 先消耗最靠近远墙的供料列，保证未处理 Cube 不会挡住 x=0.820 的第一面墙。
  {"task24_yz_wall_far_bottom_left", 6, worldPose(0.820, -0.120, kBottomZ)},
  {"task24_yz_wall_far_bottom_right", 7, worldPose(0.820, +0.120, kBottomZ)},
  {"task24_yz_wall_far_upper_left", 4, worldPose(0.820, -0.120, kUpperZ)},
  {"task24_yz_wall_far_upper_right", 5, worldPose(0.820, +0.120, kUpperZ)},
  // 远墙完成后，按相同规则向供料区外侧退回，构造第二面 X 向墙。
  {"task24_yz_wall_near_bottom_left", 2, worldPose(0.640, -0.120, kBottomZ)},
  {"task24_yz_wall_near_bottom_right", 3, worldPose(0.640, +0.120, kBottomZ)},
  {"task24_yz_wall_near_upper_left", 0, worldPose(0.640, -0.120, kUpperZ)},
  {"task24_yz_wall_near_upper_right", 1, worldPose(0.640, +0.120, kUpperZ)},
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
  if (trajectory.points.empty())
  {
    return;
  }
  // computeCartesianPath() 在目标与起点完全重合（例如已经对准目标 Y 的
  // COMMON_Y_ALIGN）时会合法地返回仅含一个、t=0 的点。它不是规划失败，
  // 而是零位移保持；补出一个相同的 30 ms 终点，才能与另一臂同步并继续做
  // FCL 采样和原子双臂命令。不能把该阶段跳过，否则左右时序会失去统一接口。
  if (trajectory.points.size() == 1)
  {
    auto endpoint = trajectory.points.front();
    setPointTime(endpoint, 0.03);
    trajectory.points.push_back(std::move(endpoint));
    return;
  }
  if (pointTime(trajectory.points.back()) > 1e-6)
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
  auto pose = worldPose(0.55, 0.0, 0.100);
  auto object = cubeObject("task24_table", pose);
  object.primitives.front().dimensions = {1.20, 0.80, 0.200};
  return object;
}

bool copyRobotDescriptions(const rclcpp::Node::SharedPtr& node)
{
  // 在本节点加入工作 executor 前完成参数复制。不能使用 SyncParametersClient：
  // 若 DDS 中短暂存在同名 MoveIt 内部服务端，sync get_parameters() 可能无限
  // 等待而没有日志。显式驱动异步 future，并给每一步确定的 10 s 上限；超时即
  // 失败退出，绝不把“零命令预检”卡成无输出进程。
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(10s))
  {
    RCLCPP_ERROR(node->get_logger(), "Task24 无法连接 /move_group。");
    return false;
  }
  const auto future = client->get_parameters({"robot_description", "robot_description_semantic"});
  if (rclcpp::spin_until_future_complete(node, future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),
      "Task24 读取 /move_group RobotModel 参数超时；请确认只启动一套 MoveIt。");
    return false;
  }
  const auto values = future.get();
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
  const double left_gap = cube.position.y - left_tcp.position.y - kCubeHalf;
  const double right_gap = right_tcp.position.y - cube.position.y - kCubeHalf;
  const double gap_asymmetry = std::abs(left_gap - right_gap);
  const double x_mismatch = std::max({
    std::abs(cube.position.x - left_tcp.position.x),
    std::abs(cube.position.x - right_tcp.position.x),
    std::abs(left_tcp.position.x - right_tcp.position.x)});
  const double z_mismatch = std::max({
    std::abs(cube.position.z + kSideContactCommandZOffset - left_tcp.position.z),
    std::abs(cube.position.z + kSideContactCommandZOffset - right_tcp.position.z),
    std::abs(left_tcp.position.z - right_tcp.position.z)});
  const double tilt_deg = orientationAngleDeg(cube.orientation);

  RCLCPP_INFO(node->get_logger(),
    "%s %s ATTACHMENT: x=%.3f mm z=%.3f mm left_gap=%.3f mm right_gap=%.3f mm gap_delta=%.3f mm tilt=%.3f deg.",
    task.id, phase.c_str(), x_mismatch * 1000.0, z_mismatch * 1000.0,
    left_gap * 1000.0, right_gap * 1000.0, gap_asymmetry * 1000.0, tilt_deg);
  if (x_mismatch > kAttachmentAlignmentTolerance ||
      z_mismatch > kAttachmentAlignmentTolerance ||
      left_gap < kMinAttachmentGap || right_gap < kMinAttachmentGap ||
      left_gap > kMaxAttachmentGap || right_gap > kMaxAttachmentGap ||
      gap_asymmetry > kMaxAttachmentGapAsymmetry ||
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
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/" + side_ + "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr state)
      {
        std::lock_guard<std::mutex> lock(joint_mutex_);
        joint_positions_.clear();
        const std::size_t count = std::min(state->name.size(), state->position.size());
        for (std::size_t index = 0; index < count; ++index)
        {
          joint_positions_[state->name[index]] = state->position[index];
        }
        seen_joint_state_.store(true);
        joint_condition_.notify_all();
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
          seen_state_.load() && seen_joint_state_.load())
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
    // 主动采样有限组候选并选择关节累计位移最小者，避免偶发选择绕过关节极限的
    // 大回环解；这不是放宽碰撞约束，MoveIt 仍逐候选做碰撞检查。
    std::vector<std::pair<double, trajectory_msgs::msg::JointTrajectory>> candidates;
    for (int attempt = 1; attempt <= kRrtCandidateCount; ++attempt)
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
          "%s %s RRTConnect candidate sample=%d/%d points=%zu joint_travel=%.3f.",
          side_.c_str(), label.c_str(), attempt, kRrtCandidateCount, candidate.points.size(), cost);
        candidates.emplace_back(cost, candidate);
        continue;
      }
      RCLCPP_WARN(node_->get_logger(), "%s %s RRTConnect candidate sample failed=%d/%d.",
        side_.c_str(), label.c_str(), attempt, kRrtCandidateCount);
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
      command.header.frame_id = "task24_single";
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

  // 由 executeSync 的唯一调度线程调用。两个 Arm 使用同一 stamp 和同一 phase，
  // 因此 ROS Graph 会在同一物理控制周期内收到相同阶段的左右目标，而不是由
  // 两个 sleep_for(10ms) 线程产生不可控的相位漂移。
  void publishAt(const trajectory_msgs::msg::JointTrajectory& input, double logical_time,
                 const builtin_interfaces::msg::Time& stamp) const
  {
    std::vector<std::string> names;
    names.reserve(input.joint_names.size());
    for (const auto& name : input.joint_names)
    {
      names.push_back(name.rfind(prefix_, 0) == 0 ? name.substr(prefix_.size()) : name);
    }
    sensor_msgs::msg::JointState command;
    command.header.stamp = stamp;
    command.header.frame_id = "task24_dual_sync";
    command.name = std::move(names);
    command.position = interpolate(input, logical_time);
    joint_pub_->publish(command);
  }

  builtin_interfaces::msg::Time nowMsg() const
  {
    const auto nanoseconds = node_->now().nanoseconds();
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
    stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
    return stamp;
  }

  double timeScale() const
  {
    return time_scale_;
  }

  bool waitAtTarget(const trajectory_msgs::msg::JointTrajectory& trajectory,
                    double tolerance_rad, double timeout_sec) const
  {
    if (trajectory.joint_names.empty() || trajectory.points.empty() ||
        trajectory.joint_names.size() != trajectory.points.back().positions.size())
    {
      return false;
    }
    const auto& target = trajectory.points.back().positions;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    double last_max_error = std::numeric_limits<double>::infinity();
    while (std::chrono::steady_clock::now() < deadline)
    {
      {
        std::unique_lock<std::mutex> lock(joint_mutex_);
        bool complete = true;
        last_max_error = 0.0;
        for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index)
        {
          const auto& full_name = trajectory.joint_names[index];
          const std::string name = full_name.rfind(prefix_, 0) == 0
            ? full_name.substr(prefix_.size()) : full_name;
          const auto found = joint_positions_.find(name);
          if (found == joint_positions_.end())
          {
            complete = false;
            last_max_error = std::numeric_limits<double>::infinity();
            break;
          }
          last_max_error = std::max(last_max_error, std::abs(found->second - target[index]));
        }
        if (complete && last_max_error <= tolerance_rad)
        {
          RCLCPP_INFO(node_->get_logger(), "%s final joint-state settled: max_error=%.3f deg.",
            side_.c_str(), last_max_error * 180.0 / kPi);
          return true;
        }
        joint_condition_.wait_for(lock, 50ms);
      }
    }
    const std::string max_error_text = std::isfinite(last_max_error)
      ? std::to_string(last_max_error * 180.0 / kPi) + " deg"
      : "missing joint state";
    RCLCPP_ERROR(node_->get_logger(), "%s final joint-state did not settle within %.1f s: max_error=%s.",
      side_.c_str(), timeout_sec, max_error_text.c_str());
    return false;
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
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::atomic_bool seen_state_{false};
  std::atomic_bool seen_joint_state_{false};
  std::atomic_bool closed_{false};
  mutable std::mutex joint_mutex_;
  mutable std::condition_variable joint_condition_;
  std::unordered_map<std::string, double> joint_positions_;
};

bool executeSync(const Arm& left, const trajectory_msgs::msg::JointTrajectory& left_trajectory,
                 const Arm& right, const trajectory_msgs::msg::JointTrajectory& right_trajectory)
{
  auto left_command = left_trajectory;
  auto right_command = right_trajectory;
  ensureTiming(left_command);
  ensureTiming(right_command);
  if (left_command.points.empty() || right_command.points.empty() ||
      left_command.joint_names.empty() || right_command.joint_names.empty())
  {
    return false;
  }

  const double left_duration = pointTime(left_command.points.back());
  const double right_duration = pointTime(right_command.points.back());
  const double common_duration = std::max(left_duration, right_duration);
  if (common_duration <= 1e-6 || std::abs(left.timeScale() - right.timeScale()) > 1e-9)
  {
    return false;
  }

  const auto start = std::chrono::steady_clock::now() + 100ms;
  std::this_thread::sleep_until(start);

  // time_scale_ 原本已经被 Arm 持有；通过左臂读取后两个 Arm 构造时传入同一值。
  // 使用 0..1 的三次平滑时间律 3u^2-2u^3：起止速度为零，两个 Articulation
  // 共享同一 phase 与总时长。Task24 实测它的连续倾角小于五次时间律，故保留。
  const double physical_duration = common_duration * left.timeScale();
  std::size_t tick = 0;
  while (true)
  {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start).count();
    const double linear_phase = std::clamp(elapsed / physical_duration, 0.0, 1.0);
    const double smooth_phase = linear_phase * linear_phase * (3.0 - 2.0 * linear_phase);
    const auto stamp = left.nowMsg();
    left.publishAt(left_command, smooth_phase * left_duration, stamp);
    right.publishAt(right_command, smooth_phase * right_duration, stamp);
    if (linear_phase >= 1.0)
    {
      break;
    }
    ++tick;
    std::this_thread::sleep_until(start + tick * kDualCommandPeriod);
  }

  for (int repeat = 0; repeat < kFinalCommandHold; ++repeat)
  {
    const auto stamp = left.nowMsg();
    left.publishAt(left_command, left_duration, stamp);
    right.publishAt(right_command, right_duration, stamp);
    std::this_thread::sleep_for(kDualCommandPeriod);
  }
  return left.waitAtTarget(left_command, kJointSettleToleranceRad, kJointSettleTimeoutSec) &&
    right.waitAtTarget(right_command, kJointSettleToleranceRad, kJointSettleTimeoutSec);
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
    if (trajectory->points.empty())
    {
      return false;
    }

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
        RCLCPP_ERROR(node->get_logger(), "%s FCL pair: %s <-> %s.",
          label.c_str(), pair.first.c_str(), pair.second.c_str());
        for (const auto& contact : contacts)
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s FCL contact: pos=(%.3f, %.3f, %.3f) depth=%.3f mm.",
            label.c_str(), contact.pos.x(), contact.pos.y(), contact.pos.z(),
            contact.depth * 1000.0);
        }
        // 记录碰撞采样点的 link 原点，区分“目标姿态本身过低”和“Cartesian
        // IK 分支在中途下探”。这只是诊断，不放宽任何碰撞规则。
        for (const auto& name : {pair.first, pair.second})
        {
          if (model->hasLinkModel(name))
          {
            const auto& p = state.getGlobalLinkTransform(name).translation();
            RCLCPP_ERROR(node->get_logger(), "%s FCL link %s origin=(%.3f, %.3f, %.3f).",
              label.c_str(), name.c_str(), p.x(), p.y(), p.z());
            // Task24 侧吸盘的 collision 依次是：竖杆、横杆、面板、四个 Cup。
            // 输出各 primitive 的世界原点，以便定位真实擦碰实体；不据此放宽 ACM。
            const auto* link = model->getLinkModel(name);
            const auto& local_origins = link->getCollisionOriginTransforms();
            for (std::size_t collision_index = 0; collision_index < local_origins.size(); ++collision_index)
            {
              const auto world = state.getGlobalLinkTransform(name) * local_origins[collision_index];
              const auto& origin = world.translation();
              RCLCPP_ERROR(node->get_logger(),
                "%s FCL collision_primitive=%zu world_origin=(%.3f, %.3f, %.3f).",
                label.c_str(), collision_index, origin.x(), origin.y(), origin.z());
            }
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
  const int requested = node->declare_parameter<int>("max_cubes", static_cast<int>(kTasks.size()));
  // Task24-M 默认执行完整 8 件。Scene、bridge 与 MoveIt 同时维护全部真实 Cube
  // CollisionObject；任何调用方若只做小批量诊断，必须显式同时降低这两个参数。
  const int active_cube_count = node->declare_parameter<int>("active_cube_count", static_cast<int>(kTasks.size()));
  const double time_scale = node->declare_parameter<double>("execution_time_scale", 3.0);
  // 只做 MoveIt/IK/FCL 链路筛选，绝不发布 joint 或 suction command。它用于在
  // 改动台面、供料或目标坐标前先验证 FR3 的可达工作区，避免把几何试错带入
  // Isaac 物理执行。
  const bool planning_only = node->declare_parameter<bool>("planning_only", false);
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
    // 从 CLOSE 起，只要任一后续规划、执行或 Ground Truth 门禁失败，就必须
    // 同步释放并确认 OPEN。这样失败不会把 Cube 留在高处或把下一轮带入脏状态。
    // 失败路径和正常 RELEASE 都共用这个闭环：先确认两个物理吸盘均 OPEN，
    // 再继续退出。日志不得把正常释放误写成 safe abort。
    const auto openBothAndConfirm = [&](const char* context)
    {
      std::thread left_off([&]() { left.suction(false); });
      std::thread right_off([&]() { right.suction(false); });
      left_off.join();
      right_off.join();
      if (!left.waitSuction(false, 3.0) || !right.waitSuction(false, 3.0))
      {
        RCLCPP_ERROR(node->get_logger(), "Task24 emergency SUCTION OFF was not confirmed on both arms.");
        return false;
      }
      RCLCPP_INFO(node->get_logger(),
        "Task24 %s: both side suctions are OPEN.", context);
      return true;
    };
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

      const double initial_left_contact_y = source.position.y - kCubeHalf - kSideContactCommandGap;
      const double initial_right_contact_y = source.position.y + kCubeHalf + kSideContactCommandGap;
      // 大 Cube 不能让两臂同时在低位 PRE_CONTACT：先到位的手会占据另一只手的
      // OMPL 接近通道。两臂先在共同 lift 高度的两侧待命，再以受 FCL 门禁的
      // 左->右微时序下降至 CONTACT；真正负载阶段仍从 COMMON_LIFT 起严格同步。
      const double pre_contact_z = source.position.z + kSideContactCommandZOffset + kLiftHeight;
      // 左臂的 HIGH_PRE_CONTACT 不能只取“关节行程最短”的 RRT 解。FR3 的冗余
      // 姿态会让该解在后续纯 Z 外侧下降中产生 link2 <-> link7 自碰。这里在
      // 任何机器人命令发出前，对完整的空载左侧接近链逐个筛选候选，绝不把这
      // 一对真实自碰加入 ACM。
      std::vector<trajectory_msgs::msg::JointTrajectory> left_pre_candidates;
      if (!left.planPoseCandidates(left_group,
            sidePose(source.position.x, initial_left_contact_y - kPreContactOffsetY, pre_contact_z, true),
            std::string(task.id) + " LEFT HIGH_PRE_CONTACT", &left_pre_candidates))
      {
        all_complete = false;
        break;
      }
      trajectory_msgs::msg::JointTrajectory right_pre;
      if (!right.planPose(right_group,
            sidePose(source.position.x, initial_right_contact_y + kPreContactOffsetY, pre_contact_z, false),
            std::string(task.id) + " RIGHT HIGH_PRE_CONTACT", &right_pre))
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
        source.position.x, initial_left_contact_y - kPreContactOffsetY,
        source.position.z + kSideContactCommandZOffset, true);
      trajectory_msgs::msg::JointTrajectory left_pre;
      trajectory_msgs::msg::JointTrajectory left_outer_descent;
      trajectory_msgs::msg::JointTrajectory right_outer_descent;
      bool left_candidate_safe = false;
      for (std::size_t candidate_index = 0;
           candidate_index < left_pre_candidates.size(); ++candidate_index)
      {
        const auto& candidate_pre = left_pre_candidates[candidate_index];
        const std::string candidate_prefix = std::string(task.id) + " LEFT_CANDIDATE_" +
          std::to_string(candidate_index + 1);

        // 实际执行保持左臂先到高位、右臂再到高位的微时序；分别验证两个子段，
        // 不能只验证两个 RRT 轨迹同起同落的理想化情况。
        auto right_hold_initial = holdTrajectory(
          right_pre, right_pre.points.front().positions, pointTime(candidate_pre.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_pre, right_hold_initial, candidate_prefix + " HIGH_LEFT"))
        {
          continue;
        }
        auto left_hold_high = holdTrajectory(
          candidate_pre, finalPositions(candidate_pre), pointTime(right_pre.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              left_hold_high, right_pre, candidate_prefix + " HIGH_RIGHT"))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_outer;
        if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
              finalPositions(candidate_pre), finalPositions(right_pre), left_low_pre,
              candidate_prefix + " OUTER_DESCENT", &candidate_outer, true))
        {
          continue;
        }
        auto right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(candidate_outer.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_outer, right_hold_pre, candidate_prefix + " OUTER_DESCENT"))
        {
          continue;
        }

        trajectory_msgs::msg::JointTrajectory candidate_contact;
        if (!planCommonCartesian(node, left_group, left.groupName(), right.groupName(),
              finalPositions(candidate_outer), finalPositions(right_pre),
              sidePose(source.position.x, initial_left_contact_y,
                source.position.z + kSideContactCommandZOffset, true),
              candidate_prefix + " CONTACT", &candidate_contact, true))
        {
          continue;
        }
        right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(candidate_contact.points.back()));
        if (!validateSync(node, left_group.getRobotModel(), world_after_remove,
              candidate_contact, right_hold_pre, candidate_prefix + " CONTACT"))
        {
          continue;
        }

        left_pre = candidate_pre;
        left_outer_descent = std::move(candidate_outer);
        left_contact = std::move(candidate_contact);
        left_candidate_safe = true;
        RCLCPP_INFO(node->get_logger(), "%s selected LEFT_CANDIDATE_%zu/%zu: "
          "HIGH_PRE_CONTACT -> OUTER_DESCENT -> CONTACT FCL PASS.", task.id,
          candidate_index + 1, left_pre_candidates.size());
        break;
      }
      if (!left_candidate_safe)
      {
        RCLCPP_ERROR(node->get_logger(), "%s no FCL-safe left empty-approach candidate was found.", task.id);
        all_complete = false;
        break;
      }

      // planning_only 必须是严格的“零命令”模式：它以最新收到的 source pose
      // 预演后续右侧空载接近和完整共同负载链，绝不能为了取得 live pose 而先
      // 执行左侧接近。真实执行模式才允许左侧先落到 CONTACT，并随后从 Isaac
      // Ground Truth 刷新 Cube 位置，补偿无吸附接近期间可能出现的微小漂移。
      geometry_msgs::msg::Pose cube_after_left_contact = source;
      if (!planning_only)
      {
        if (!left.executeAt(left_pre, std::chrono::steady_clock::now()) ||
            !right.executeAt(right_pre, std::chrono::steady_clock::now()) ||
            !left.waitAtTarget(left_pre, kJointSettleToleranceRad, kJointSettleTimeoutSec) ||
            !right.waitAtTarget(right_pre, kJointSettleToleranceRad, kJointSettleTimeoutSec))
        {
          all_complete = false;
          break;
        }

        auto right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(left_outer_descent.points.back()));
        if (!executeSync(left, left_outer_descent, right, right_hold_pre))
        {
          all_complete = false;
          break;
        }

        right_hold_pre = holdTrajectory(
          right_pre, finalPositions(right_pre), pointTime(left_contact.points.back()));
        if (!executeSync(left, left_contact, right, right_hold_pre))
        {
          all_complete = false;
          break;
        }

        std::this_thread::sleep_for(250ms);
        const auto [live_cube_after_left_contact, left_contact_revision] = cubes.get(task.cube_index);
        (void)left_contact_revision;
        cube_after_left_contact = live_cube_after_left_contact;
      }
      const double right_live_contact_y =
        cube_after_left_contact.position.y + kCubeHalf + kSideContactCommandGap;
      // 空载的右侧外部接近可以使用 RRTConnect。先在机器人尚未吸附 Cube 时
      // 预演后续完整负载链；只有通过者才会发送右臂命令。一个有限候选池可能
      // 恰好只采到同一类绕腕解，因此在左臂保持静止、两杯仍 OFF 时最多重采样
      // 三批；不会把一次 OMPL 随机结果变成物理失败。
      std::vector<trajectory_msgs::msg::JointTrajectory> right_outer_candidates;
      const auto right_outer_target = sidePose(
        cube_after_left_contact.position.x,
        right_live_contact_y + kPreContactOffsetY,
        cube_after_left_contact.position.z + kSideContactCommandZOffset, false);
      for (int batch = 1; batch <= kRrtCandidateBatches; ++batch)
      {
        std::vector<trajectory_msgs::msg::JointTrajectory> batch_candidates;
        if (!right.planPoseCandidates(right_group, right_outer_target,
              std::string(task.id) + " RIGHT_OUTER_APPROACH batch=" + std::to_string(batch),
              &batch_candidates))
        {
          RCLCPP_WARN(node->get_logger(), "%s RIGHT_OUTER_APPROACH batch=%d/%d produced no OMPL candidate.",
            task.id, batch, kRrtCandidateBatches);
          continue;
        }
        right_outer_candidates.insert(right_outer_candidates.end(),
          std::make_move_iterator(batch_candidates.begin()),
          std::make_move_iterator(batch_candidates.end()));
      }
      if (right_outer_candidates.empty())
      {
        RCLCPP_ERROR(node->get_logger(), "%s RIGHT_OUTER_APPROACH exhausted %d RRT candidate batches.",
          task.id, kRrtCandidateBatches);
        all_complete = false;
        break;
      }
      RCLCPP_INFO(node->get_logger(), "%s RIGHT_OUTER_APPROACH evaluating %zu candidates from %d batches.",
        task.id, right_outer_candidates.size(), kRrtCandidateBatches);
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
                cube_after_left_contact.position.z + kSideContactCommandZOffset, false),
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
        const double candidate_left_y = cube_after_left_contact.position.y - kCubeHalf - kSideContactCommandGap;
        const double candidate_right_y = cube_after_left_contact.position.y + kCubeHalf + kSideContactCommandGap;
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(candidate_contact),
              sidePose(cube_after_left_contact.position.x, candidate_left_y,
                cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight, true),
              sidePose(cube_after_left_contact.position.x, candidate_right_y,
                cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight, false),
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
        const double candidate_release_z =
          task.target.position.z + kSideContactCommandZOffset + kReleaseGapZ;
        const double candidate_transport_z =
          cube_after_left_contact.position.z + kSideContactCommandZOffset + kLiftHeight;
        const double candidate_entry_left_y = task.target.position.y - kCubeHalf - kSideContactCommandGap;
        const double candidate_entry_right_y = task.target.position.y + kCubeHalf + kSideContactCommandGap;
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
        RCLCPP_INFO(node->get_logger(), "%s selected right outer candidate=%zu/%zu after full loaded-chain preflight.",
          task.id, candidate_index + 1, right_outer_candidates.size());
        break;
      }
      if (!right_candidate_safe)
      {
        RCLCPP_ERROR(node->get_logger(), "%s no right-side empty-arm candidate passed the full CONTACT -> LIFT -> X -> Y -> DESCENT -> PUSH -> RETREAT preflight.",
          task.id);
        all_complete = false;
        break;
      }

      if (planning_only)
      {
        // 零命令预检同样必须推进 Planning Scene：本件虽不会在 Isaac 中真的
        // 搬运，但下一件的 FCL 需要把它视为已经按离线目标落稳的垛墙障碍物。
        // 真实执行路径则会用 release 后最新的 Isaac Ground Truth 覆盖此位姿。
        if (!scene.applyCollisionObject(cubeObject(object_id, task.target)))
        {
          RCLCPP_ERROR(node->get_logger(),
            "%s planning-only cannot restore target CollisionObject %s.",
            task.id, object_id.c_str());
          all_complete = false;
          break;
        }
        std::this_thread::sleep_for(100ms);
        RCLCPP_INFO(node->get_logger(),
          "%s TASK24 PLANNING-ONLY PASS: both empty-arm approaches and the complete "
          "Z->X->Y->Z->X->Z chain passed IK/FCL; target Cube was restored to the "
          "planning scene for following tasks; no joint or suction command was published.",
          task.id);
        continue;
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
        const double live_left_y = live_cube.position.y - kCubeHalf - kSideContactCommandGap;
        const double live_right_y = live_cube.position.y + kCubeHalf + kSideContactCommandGap;
        if (!planAndCheckCommon(node, left_group, right_group, left, right,
              finalPositions(left_contact), finalPositions(right_contact),
              sidePose(live_cube.position.x, live_left_y,
                live_cube.position.z + kSideContactCommandZOffset, true),
              sidePose(live_cube.position.x, live_right_y,
                live_cube.position.z + kSideContactCommandZOffset, false),
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
      const double preflight_left_y = grasp_pose.position.y - kCubeHalf - kSideContactCommandGap;
      const double preflight_right_y = grasp_pose.position.y + kCubeHalf + kSideContactCommandGap;
      if (!planAndCheckCommon(node, left_group, right_group, left, right,
            finalPositions(left_contact), finalPositions(right_contact),
            sidePose(grasp_pose.position.x, preflight_left_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, true),
            sidePose(grasp_pose.position.x, preflight_right_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, false),
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
        openBothAndConfirm("safe abort after close failure");
        all_complete = false;
        break;
      }

      std::this_thread::sleep_for(250ms);
      const auto [attachment_pose, attachment_revision] = cubes.get(task.cube_index);
      (void)attachment_revision;
      if (!validateDualSideAttachment(node, task, attachment_pose, left_tcp.get(), right_tcp.get(),
            "POST_CLOSE"))
      {
        openBothAndConfirm("safe abort after attachment validation failure");
        all_complete = false;
        break;
      }

      // 以闭合后真实锁定的 Cube pose 作为所有负载 Cartesian 段的起点，不能再
      // 回退到任务开始时的 source。这样 Z->X->Y->Z 的每段都从同一真实几何出发。
      grasp_pose = attachment_pose;
      const double grasp_left_contact_y =
        grasp_pose.position.y - kCubeHalf - kSideContactCommandGap;
      const double grasp_right_contact_y =
        grasp_pose.position.y + kCubeHalf + kSideContactCommandGap;

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
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, true),
            sidePose(grasp_pose.position.x, grasp_right_contact_y,
              grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight, false),
            left_contact, right_contact, &left_lift, &right_lift) ||
          !executeSync(left, left_lift, right, right_lift) ||
          !trace_held("COMMON_LIFT"))
      {
        openBothAndConfirm("safe abort after COMMON_LIFT failure");
        all_complete = false;
        break;
      }

      const double entry_x = task.target.position.x - kPrePushOffsetX;
      const double release_z =
        task.target.position.z + kSideContactCommandZOffset + kReleaseGapZ;
      const double entry_left_y = task.target.position.y - kCubeHalf - kSideContactCommandGap;
      const double entry_right_y = task.target.position.y + kCubeHalf + kSideContactCommandGap;
      const double transport_z =
        grasp_pose.position.z + kSideContactCommandZOffset + kLiftHeight;

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
        openBothAndConfirm("safe abort after COMMON_X_TRAVEL failure");
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
        openBothAndConfirm("safe abort after COMMON_Y_ALIGN failure");
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
        openBothAndConfirm("safe abort after COMMON_DESCENT_TO_ENTRY failure");
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
        openBothAndConfirm("safe abort after COMMON_SIDE_SHORT_PUSH failure");
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
        openBothAndConfirm("safe abort after retreat preplanning failure");
        all_complete = false;
        break;
      }
      const auto [unused_pose, release_revision] = cubes.get(task.cube_index);
      (void)unused_pose;
      if (!openBothAndConfirm("normal release"))
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
