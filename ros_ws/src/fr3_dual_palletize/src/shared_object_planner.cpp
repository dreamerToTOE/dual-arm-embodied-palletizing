#include "fr3_dual_palletize/shared_object_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <thread>
#include <utility>

#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/world.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "fr3_dual_palletize/runtime_box_state.hpp"

using namespace std::chrono_literals;

namespace fr3_dual_palletize
{
namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double SCENE_SYNC_WAIT_SEC = 0.30;
constexpr double SYNCHRONIZED_TRAJECTORY_RESAMPLE_SEC = 0.010;

enum class Side
{
  LEFT,
  RIGHT,
};

struct ArmContext
{
  Side side{Side::LEFT};
  std::string group_name;
  std::string partner_group_name;
  std::string eef_link;
  std::string suction_link;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group;
  const moveit::core::JointModelGroup* joint_model_group{nullptr};
  const moveit::core::JointModelGroup* partner_joint_model_group{nullptr};
  std::vector<double> home_q;
};

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
    static_cast<double>(point.time_from_start.nanosec) * 1.0e-9;
}

void setPointTime(trajectory_msgs::msg::JointTrajectoryPoint& point, double seconds)
{
  auto sec = static_cast<std::int32_t>(std::floor(seconds));
  auto nanosec = static_cast<std::int64_t>(std::llround(
    (seconds - static_cast<double>(sec)) * 1.0e9));
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
  if (trajectory.points.empty() || pointTime(trajectory.points.back()) > 1.0e-6)
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

double trajectoryDuration(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  return trajectory.points.empty() ? 0.0 : pointTime(trajectory.points.back());
}

bool synchronizeDurations(
  trajectory_msgs::msg::JointTrajectory& first,
  trajectory_msgs::msg::JointTrajectory& second,
  const moveit::core::RobotModelConstPtr& model,
  const std::string& first_eef_link,
  const std::string& second_eef_link,
  double suction_tcp_offset_z)
{
  ensureTiming(first);
  ensureTiming(second);
  if (first.points.empty() || second.points.empty())
  {
    return false;
  }
  const double first_duration = trajectoryDuration(first);
  const double second_duration = trajectoryDuration(second);
  if (first_duration <= 1.0e-6 || second_duration <= 1.0e-6 ||
      first.points.size() < 2 || second.points.size() < 2)
  {
    return false;
  }
  const double duration = std::max(first_duration, second_duration);
  if (!model || first_eef_link.empty() || second_eef_link.empty() ||
      suction_tcp_offset_z <= 0.0)
  {
    return false;
  }
  const std::size_t synchronized_samples = static_cast<std::size_t>(
    std::ceil(duration / SYNCHRONIZED_TRAJECTORY_RESAMPLE_SEC));
  if (synchronized_samples == 0)
  {
    return false;
  }
  const auto tcpProgress = [model, suction_tcp_offset_z](
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::string& eef_link)
  {
    std::vector<double> cumulative;
    cumulative.reserve(trajectory.points.size());
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    Eigen::Vector3d previous = Eigen::Vector3d::Zero();
    double total = 0.0;
    for (std::size_t index = 0; index < trajectory.points.size(); ++index)
    {
      state.setVariablePositions(trajectory.joint_names, trajectory.points[index].positions);
      state.update();
      const Eigen::Vector3d current = (state.getGlobalLinkTransform(eef_link) *
        Eigen::Translation3d(0.0, 0.0, suction_tcp_offset_z)).translation();
      if (index > 0)
      {
        total += (current - previous).norm();
      }
      cumulative.push_back(total);
      previous = current;
    }
    if (total <= 1.0e-9)
    {
      for (std::size_t index = 0; index < cumulative.size(); ++index)
      {
        cumulative[index] = static_cast<double>(index) /
          static_cast<double>(cumulative.size() - 1);
      }
      return cumulative;
    }
    for (auto& value : cumulative)
    {
      value /= total;
    }
    return cumulative;
  };
  const auto resampleProgress = [duration, synchronized_samples](
    trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::vector<double>& original_progress)
  {
    const auto original = trajectory.points;
    trajectory.points.clear();
    trajectory.points.reserve(synchronized_samples + 1);
    for (std::size_t index = 0; index <= synchronized_samples; ++index)
    {
      // 两条 Cartesian path 的点数可能不同。按各自点号线性缩放总时长会让
      // 较短的一条在同一共享时刻走得更慢，破坏共同刚体约束。这里按各自
      // Cartesian 离散进度归一化到同一时轴，并加密至 10 ms。这样实际共享
      // 时钟执行与 FCL 采样之间不会因稀疏 joint 插值产生毫米级相对漂移。
      const double progress = static_cast<double>(index) /
        static_cast<double>(synchronized_samples);
      auto upper_it = std::lower_bound(
        original_progress.begin(), original_progress.end(), progress);
      const std::size_t upper = upper_it == original_progress.end() ?
        original.size() - 1 : static_cast<std::size_t>(upper_it - original_progress.begin());
      const std::size_t lower = upper == 0 ? 0 : upper - 1;
      const double span = original_progress[upper] - original_progress[lower];
      const double alpha = span <= 1.0e-12 ? 0.0 :
        std::clamp((progress - original_progress[lower]) / span, 0.0, 1.0);
      auto point = original[lower];
      point.positions.resize(original[lower].positions.size());
      for (std::size_t joint = 0; joint < point.positions.size(); ++joint)
      {
        point.positions[joint] = original[lower].positions[joint] + alpha *
          (original[upper].positions[joint] - original[lower].positions[joint]);
      }
      setPointTime(point, duration * progress);
      trajectory.points.push_back(std::move(point));
    }
  };
  const auto first_progress = tcpProgress(first, first_eef_link);
  const auto second_progress = tcpProgress(second, second_eef_link);
  if (first_progress.size() != first.points.size() ||
      second_progress.size() != second.points.size())
  {
    return false;
  }
  resampleProgress(first, first_progress);
  resampleProgress(second, second_progress);
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
    const double interval = after_time - before_time;
    if (interval <= 1.0e-9)
    {
      return points[index].positions;
    }
    const double alpha = (time_sec - before_time) / interval;
    std::vector<double> positions(points[index].positions.size());
    for (std::size_t joint = 0; joint < positions.size(); ++joint)
    {
      positions[joint] = points[index - 1].positions[joint] + alpha *
        (points[index].positions[joint] - points[index - 1].positions[joint]);
    }
    return positions;
  }
  return points.back().positions;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
{
  const double sin_yaw = 2.0 * (
    quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 * (
    quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion normalizedQuaternion(geometry_msgs::msg::Quaternion quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (!std::isfinite(norm) || norm <= 1.0e-9)
  {
    return geometry_msgs::msg::Quaternion{};
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

geometry_msgs::msg::Pose interpolatePose(
  const geometry_msgs::msg::Pose& first,
  const geometry_msgs::msg::Pose& second,
  double alpha)
{
  alpha = std::clamp(alpha, 0.0, 1.0);
  geometry_msgs::msg::Pose result;
  result.position.x = first.position.x + alpha * (second.position.x - first.position.x);
  result.position.y = first.position.y + alpha * (second.position.y - first.position.y);
  result.position.z = first.position.z + alpha * (second.position.z - first.position.z);
  const auto first_q = normalizedQuaternion(first.orientation);
  const auto second_q = normalizedQuaternion(second.orientation);
  const Eigen::Quaterniond first_eigen(first_q.w, first_q.x, first_q.y, first_q.z);
  const Eigen::Quaterniond second_eigen(second_q.w, second_q.x, second_q.y, second_q.z);
  const Eigen::Quaterniond blended = first_eigen.slerp(alpha, second_eigen).normalized();
  result.orientation.x = blended.x();
  result.orientation.y = blended.y();
  result.orientation.z = blended.z();
  result.orientation.w = blended.w();
  return result;
}

geometry_msgs::msg::Pose makeTopDownLink8Pose(
  const geometry_msgs::msg::Pose& suction_tcp_pose,
  double suction_tcp_offset_z)
{
  geometry_msgs::msg::Pose result;
  result.position = suction_tcp_pose.position;
  // 在 Rz(yaw) * Rx(pi) 下 link8 局部 +Z 指向世界 -Z；规划 tip 是 link8，
  // 所以它必须位于希望到达的 suction TCP 上方 0.105 m。
  result.position.z += suction_tcp_offset_z;
  const double half_yaw = 0.5 * yawFromQuaternion(suction_tcp_pose.orientation);
  result.orientation.x = std::cos(half_yaw);
  result.orientation.y = std::sin(half_yaw);
  result.orientation.z = 0.0;
  result.orientation.w = 0.0;
  return result;
}

geometry_msgs::msg::Pose suctionTcpPoseForGrasp(
  const geometry_msgs::msg::Pose& box_pose,
  const GraspCandidate& grasp,
  double contact_clearance_z)
{
  auto result = composePose(box_pose, grasp.local_pose);
  // 顶部吸附接触的释放/接触间隙沿世界竖直方向定义；Box 可带任意 yaw。
  result.position.z += contact_clearance_z;
  return result;
}

geometry_msgs::msg::Pose link8TargetForGrasp(
  const geometry_msgs::msg::Pose& box_pose,
  const GraspCandidate& grasp,
  double vertical_offset,
  const SharedObjectPlanConfig& config)
{
  auto tcp = suctionTcpPoseForGrasp(box_pose, grasp, config.contact_clearance_z);
  tcp.position.z += vertical_offset;
  return makeTopDownLink8Pose(tcp, config.suction_tcp_offset_z);
}

double distance3d(const Eigen::Vector3d& first, const Eigen::Vector3d& second)
{
  return (first - second).norm();
}

double distanceToSegment(
  const Eigen::Vector3d& point,
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& end)
{
  const Eigen::Vector3d direction = end - start;
  const double length_squared = direction.squaredNorm();
  if (length_squared <= 1.0e-12)
  {
    return distance3d(point, start);
  }
  const double progress = std::clamp(
    (point - start).dot(direction) / length_squared, 0.0, 1.0);
  return distance3d(point, start + progress * direction);
}

double quaternionAngularDistance(
  const Eigen::Quaterniond& first,
  const Eigen::Quaterniond& second)
{
  const double dot = std::abs(first.normalized().dot(second.normalized()));
  return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

bool chooseDualGrasps(
  const BoxSpec& box,
  GraspCandidate& left,
  GraspCandidate& right,
  std::string& error)
{
  if (box.grasp_candidates.size() < 2)
  {
    error = "共享物体至少需要两个运行时 grasp candidates";
    return false;
  }
  const auto first = std::min_element(
    box.grasp_candidates.begin(), box.grasp_candidates.end(),
    [](const GraspCandidate& a, const GraspCandidate& b)
    {
      return a.local_pose.position.y < b.local_pose.position.y;
    });
  const auto second = std::max_element(
    box.grasp_candidates.begin(), box.grasp_candidates.end(),
    [](const GraspCandidate& a, const GraspCandidate& b)
    {
      return a.local_pose.position.y < b.local_pose.position.y;
    });
  if (first == second)
  {
    error = "无法从 grasp candidates 选择两个不同吸附点";
    return false;
  }
  const auto insideTopFace = [&box](const GraspCandidate& grasp)
  {
    return std::abs(grasp.local_pose.position.x) <= 0.5 * box.dimensions[0] + 1.0e-6 &&
      std::abs(grasp.local_pose.position.y) <= 0.5 * box.dimensions[1] + 1.0e-6 &&
      std::abs(grasp.local_pose.position.z - 0.5 * box.dimensions[2]) <= 0.005;
  };
  if (!insideTopFace(*first) || !insideTopFace(*second))
  {
    error = "双吸盘 grasp candidate 不在 Box 顶面有效范围";
    return false;
  }
  const double separation = std::hypot(
    first->local_pose.position.x - second->local_pose.position.x,
    first->local_pose.position.y - second->local_pose.position.y);
  if (separation < 0.030)
  {
    error = "双吸盘 grasp separation 小于 30 mm，无法保证独立吸附";
    return false;
  }
  left = *first;
  right = *second;
  return true;
}

bool configureGroup(ArmContext& arm)
{
  arm.group->setPlannerId("RRTConnectkConfigDefault");
  arm.group->setPlanningTime(5.0);
  arm.group->setNumPlanningAttempts(1);
  arm.group->setMaxVelocityScalingFactor(0.15);
  arm.group->setMaxAccelerationScalingFactor(0.15);
  arm.group->setPoseReferenceFrame("world");
  arm.group->setEndEffectorLink(arm.eef_link);
  arm.joint_model_group = arm.group->getRobotModel()->getJointModelGroup(arm.group_name);
  arm.partner_joint_model_group = arm.group->getRobotModel()->getJointModelGroup(
    arm.partner_group_name);
  if (!arm.joint_model_group || !arm.partner_joint_model_group)
  {
    return false;
  }
  const auto state = arm.group->getCurrentState(3.0);
  if (!state)
  {
    return false;
  }
  state->copyJointGroupPositions(arm.joint_model_group, arm.home_q);
  return arm.home_q.size() == arm.joint_model_group->getVariableCount();
}

bool setStartState(
  ArmContext& arm,
  const std::vector<double>& own_q,
  const std::vector<double>& partner_q)
{
  if (own_q.size() != arm.joint_model_group->getVariableCount() ||
      partner_q.size() != arm.partner_joint_model_group->getVariableCount())
  {
    return false;
  }
  auto state = arm.group->getCurrentState(3.0);
  if (!state)
  {
    return false;
  }
  state->setJointGroupPositions(arm.joint_model_group, own_q);
  state->setJointGroupPositions(arm.partner_joint_model_group, partner_q);
  state->update();
  arm.group->setStartState(*state);
  return true;
}

bool planPose(
  const rclcpp::Node::SharedPtr& node,
  ArmContext& arm,
  const std::vector<double>& own_q,
  const std::vector<double>& partner_q,
  const geometry_msgs::msg::Pose& target,
  const std::string& stage,
  int max_retries,
  trajectory_msgs::msg::JointTrajectory& output)
{
  arm.group->clearPoseTargets();
  if (!arm.group->setPoseTarget(target, arm.eef_link))
  {
    return false;
  }
  for (int attempt = 1; attempt <= max_retries; ++attempt)
  {
    if (!setStartState(arm, own_q, partner_q))
    {
      return false;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm.group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS &&
        !plan.trajectory_.joint_trajectory.points.empty())
    {
      output = plan.trajectory_.joint_trajectory;
      ensureTiming(output);
      RCLCPP_INFO(
        node->get_logger(), "Task21 tight %s %s RRTConnect OK, attempt=%d/%d",
        arm.group_name.c_str(), stage.c_str(), attempt, max_retries);
      return true;
    }
    RCLCPP_WARN(
      node->get_logger(), "Task21 tight %s %s RRTConnect retry=%d/%d",
      arm.group_name.c_str(), stage.c_str(), attempt, max_retries);
  }
  return false;
}

bool planCartesian(
  const rclcpp::Node::SharedPtr& node,
  ArmContext& arm,
  const std::vector<double>& own_q,
  const std::vector<double>& partner_q,
  const geometry_msgs::msg::Pose& target,
  const std::string& stage,
  bool avoid_collisions,
  const SharedObjectPlanConfig& config,
  trajectory_msgs::msg::JointTrajectory& output)
{
  for (int attempt = 1; attempt <= config.max_planning_retries; ++attempt)
  {
    if (!setStartState(arm, own_q, partner_q))
    {
      return false;
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error;
    const double fraction = arm.group->computeCartesianPath(
      std::vector<geometry_msgs::msg::Pose>{target}, config.cartesian_eef_step, 0.0,
      trajectory, avoid_collisions, &error);
    RCLCPP_INFO(
      node->get_logger(), "Task21 tight %s %s Cartesian fraction=%.4f error=%d attempt=%d/%d",
      arm.group_name.c_str(), stage.c_str(), fraction, error.val, attempt,
      config.max_planning_retries);
    if (fraction >= config.cartesian_min_fraction &&
        !trajectory.joint_trajectory.points.empty())
    {
      output = trajectory.joint_trajectory;
      ensureTiming(output);
      return true;
    }
  }
  return false;
}

std::string firstCollisionPair(const collision_detection::CollisionResult& result)
{
  if (result.contacts.empty())
  {
    return "unknown";
  }
  const auto& contact = *result.contacts.begin();
  return contact.first.first + " <-> " + contact.first.second;
}

bool validateStage(
  const rclcpp::Node::SharedPtr& node,
  const moveit::core::RobotModelConstPtr& model,
  const std::vector<moveit_msgs::msg::CollisionObject>& static_objects,
  const BoxSpec& box,
  const GraspCandidate& left_grasp,
  const GraspCandidate& right_grasp,
  const SharedObjectStage& stage,
  const SharedObjectPlanConfig& config,
  SharedObjectPlan& report,
  std::string& error)
{
  if (!model || stage.left.points.empty() || stage.right.points.empty())
  {
    error = "共享阶段输入不完整";
    return false;
  }
  const double duration = std::max(trajectoryDuration(stage.left), trajectoryDuration(stage.right));
  if (duration <= 1.0e-6)
  {
    error = "共享阶段 duration 无效";
    return false;
  }

  auto scene = std::make_shared<planning_scene::PlanningScene>(model);
  for (const auto& object : static_objects)
  {
    if (!scene->processCollisionObjectMsg(object))
    {
      error = "无法载入静态 CollisionObject " + object.id;
      return false;
    }
  }
  // 这是私有 FCL scene 的最小化接触许可，绝不写入 move_group ACM：共享 Box
  // 与两只吸盘杯的顶面接触是任务定义，其余 Box--robot / Box--world 组合仍检测。
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  acm.setEntry(box.id, "left_fr3_compact_suction", true);
  acm.setEntry(box.id, "right_fr3_compact_suction", true);

  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(duration / config.collision_sample_period_sec));
  for (std::size_t index = 0; index <= sample_count; ++index)
  {
    const double time = std::min(
      static_cast<double>(index) * config.collision_sample_period_sec, duration);
    const auto left_positions = interpolatePositions(stage.left, time);
    const auto right_positions = interpolatePositions(stage.right, time);
    if (left_positions.size() != stage.left.joint_names.size() ||
        right_positions.size() != stage.right.joint_names.size())
    {
      error = "共享阶段 joint trajectory 无效";
      return false;
    }
    const double alpha = duration <= 1.0e-9 ? 1.0 : time / duration;
    const auto nominal_box_pose = interpolatePose(stage.box_start, stage.box_end, alpha);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setVariablePositions(stage.left.joint_names, left_positions);
    state.setVariablePositions(stage.right.joint_names, right_positions);
    state.update();
    auto collision_box_pose = nominal_box_pose;
    if (stage.enforce_grasp_constraint)
    {
      // 共同阶段的 Box 不以“按墙钟线性插值的理想点”冒充实际几何。先由两个
      // FK TCP 各自反推 Box center，取中点作为 FCL 中的保守共享物体 pose；随后
      // 检查两条反推结果的一致性和其对规划参考路径的偏差。
      const Eigen::Isometry3d left_tcp = state.getGlobalLinkTransform("left_fr3_link8") *
        Eigen::Translation3d(0.0, 0.0, config.suction_tcp_offset_z);
      const Eigen::Isometry3d right_tcp = state.getGlobalLinkTransform("right_fr3_link8") *
        Eigen::Translation3d(0.0, 0.0, config.suction_tcp_offset_z);
      const auto nominal_left = suctionTcpPoseForGrasp(
        nominal_box_pose, left_grasp, config.contact_clearance_z);
      const auto nominal_right = suctionTcpPoseForGrasp(
        nominal_box_pose, right_grasp, config.contact_clearance_z);
      const Eigen::Vector3d nominal_center(
        nominal_box_pose.position.x, nominal_box_pose.position.y, nominal_box_pose.position.z);
      const Eigen::Vector3d left_offset(
        nominal_left.position.x - nominal_center.x(),
        nominal_left.position.y - nominal_center.y(),
        nominal_left.position.z - nominal_center.z());
      const Eigen::Vector3d right_offset(
        nominal_right.position.x - nominal_center.x(),
        nominal_right.position.y - nominal_center.y(),
        nominal_right.position.z - nominal_center.z());
      const Eigen::Vector3d left_center = left_tcp.translation() - left_offset;
      const Eigen::Vector3d right_center = right_tcp.translation() - right_offset;
      const Eigen::Vector3d inferred_center = 0.5 * (left_center + right_center);
      collision_box_pose.position.x = inferred_center.x();
      collision_box_pose.position.y = inferred_center.y();
      collision_box_pose.position.z = inferred_center.z();

      const auto expectedLeft = suctionTcpPoseForGrasp(
        collision_box_pose, left_grasp, config.contact_clearance_z);
      const auto expectedRight = suctionTcpPoseForGrasp(
        collision_box_pose, right_grasp, config.contact_clearance_z);
      const auto expected_q = makeTopDownLink8Pose(
        expectedLeft, config.suction_tcp_offset_z).orientation;
      const Eigen::Quaterniond expected_orientation(
        expected_q.w, expected_q.x, expected_q.y, expected_q.z);
      const auto evaluateTcp = [&](const Eigen::Isometry3d& tcp_transform,
        const geometry_msgs::msg::Pose& expected, double& max_error,
        double& position_error, double& orientation_error)
      {
        const Eigen::Vector3d expected_position(
          expected.position.x, expected.position.y, expected.position.z);
        position_error = distance3d(tcp_transform.translation(), expected_position);
        max_error = std::max(max_error, position_error);
        const Eigen::Quaterniond actual_orientation(tcp_transform.rotation());
        orientation_error = quaternionAngularDistance(actual_orientation, expected_orientation);
        report.max_tcp_orientation_error_rad = std::max(
          report.max_tcp_orientation_error_rad, orientation_error);
        // 10 µm 数值余量仅消除 double 插值/rounding 边界。
        return position_error <= config.tcp_position_tolerance_m + 1.0e-5 &&
          orientation_error <= config.tcp_orientation_tolerance_rad;
      };
      double left_position_error = 0.0;
      double right_position_error = 0.0;
      double left_orientation_error = 0.0;
      double right_orientation_error = 0.0;
      const bool left_ok = evaluateTcp(left_tcp, expectedLeft, report.max_left_tcp_error_m,
        left_position_error, left_orientation_error);
      const bool right_ok = evaluateTcp(right_tcp, expectedRight, report.max_right_tcp_error_m,
        right_position_error, right_orientation_error);
      const Eigen::Vector3d stage_start(
        stage.box_start.position.x, stage.box_start.position.y, stage.box_start.position.z);
      const Eigen::Vector3d stage_end(
        stage.box_end.position.x, stage.box_end.position.y, stage.box_end.position.z);
      const double tracking_error = distanceToSegment(inferred_center, stage_start, stage_end);
      report.max_box_path_tracking_error_m = std::max(
        report.max_box_path_tracking_error_m, tracking_error);
      if (!left_ok || !right_ok ||
          tracking_error > config.box_path_tracking_tolerance_m + 1.0e-5)
      {
        std::ostringstream stream;
        stream << stage.name << " shared TCP/Box constraint exceeds limit at t=" << time <<
          " s (left=" << left_position_error * 1000.0 << " mm/" <<
          left_orientation_error * 180.0 / PI << " deg, right=" <<
          right_position_error * 1000.0 << " mm/" <<
          right_orientation_error * 180.0 / PI << " deg, box_lateral=" <<
          tracking_error * 1000.0 << " mm)";
        error = stream.str();
        return false;
      }
    }

    // 在同一个私有 scene 中替换由当前双 TCP 反推的共享 Box pose，保持其他
    // runtime Box、桌面和双臂全部可见。该操作不触碰外部 move_group Planning Scene。
    scene->getWorldNonConst()->removeObject(box.id);
    if (!scene->processCollisionObjectMsg(makeCollisionObject(box, collision_box_pose)))
    {
      error = "无法更新私有 FCL 中的共享 Box pose";
      return false;
    }
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult collision;
    scene->checkCollision(request, collision, state);
    if (collision.collision)
    {
      const Eigen::Vector3d left_tcp = (state.getGlobalLinkTransform("left_fr3_link8") *
        Eigen::Translation3d(0.0, 0.0, config.suction_tcp_offset_z)).translation();
      const Eigen::Vector3d right_tcp = (state.getGlobalLinkTransform("right_fr3_link8") *
        Eigen::Translation3d(0.0, 0.0, config.suction_tcp_offset_z)).translation();
      std::ostringstream stream;
      stream << stage.name << " FCL collision at t=" << time << " s: " <<
        firstCollisionPair(collision) << " (left_tcp=" << left_tcp.x() << "," <<
        left_tcp.y() << "," << left_tcp.z() << ", right_tcp=" << right_tcp.x() << "," <<
        right_tcp.y() << "," << right_tcp.z() << ")";
      error = stream.str();
      return false;
    }

  }
  report.fcl_samples += sample_count + 1;
  RCLCPP_INFO(
    node->get_logger(), "Task21 tight %s shared-object FCL + TCP constraint PASS, samples=%zu",
    stage.name.c_str(), sample_count + 1);
  return true;
}

bool removeWorldObject(
  moveit::planning_interface::PlanningSceneInterface& scene,
  const std::string& object_id)
{
  const auto existing = scene.getObjects({object_id});
  if (existing.empty())
  {
    return false;
  }
  scene.removeCollisionObjects({object_id});
  std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
  return true;
}

}  // namespace

SharedObjectPlanner::SharedObjectPlanner(
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<std::mutex> planning_scene_mutex,
  SharedObjectPlanConfig config)
  : node_(std::move(node)),
    planning_scene_mutex_(std::move(planning_scene_mutex)),
    config_(std::move(config))
{
}

bool SharedObjectPlanner::plan(
  const BoxSpec& box,
  const PlacementSpec& placement,
  SharedObjectPlan& output)
{
  output = SharedObjectPlan();
  if (!node_ || !planning_scene_mutex_ || box.id.empty() ||
      placement.object_id != box.id || config_.max_planning_retries <= 0 ||
      config_.suction_tcp_offset_z <= 0.0 || config_.contact_clearance_z < 0.0 ||
      config_.pre_contact_clearance_z <= 0.0 || config_.lift_height <= 0.0 ||
      config_.cartesian_eef_step <= 0.0 || config_.cartesian_min_fraction <= 0.0 ||
      config_.cartesian_min_fraction > 1.0 || config_.collision_sample_period_sec <= 0.0 ||
      config_.tcp_position_tolerance_m <= 0.0 || config_.tcp_orientation_tolerance_rad <= 0.0 ||
      config_.box_path_tracking_tolerance_m <= 0.0)
  {
    output.diagnostics = "SharedObjectPlanner 输入或参数无效";
    return false;
  }

  GraspCandidate left_grasp;
  GraspCandidate right_grasp;
  if (!chooseDualGrasps(box, left_grasp, right_grasp, output.diagnostics))
  {
    return false;
  }
  output.left_grasp = left_grasp;
  output.right_grasp = right_grasp;

  std::unique_lock<std::mutex> lock(*planning_scene_mutex_);
  moveit::planning_interface::PlanningSceneInterface moveit_scene;
  bool object_removed = false;
  const auto restoreSourceObject = [&]()
  {
    if (!object_removed)
    {
      return;
    }
    moveit_scene.applyCollisionObject(makeCollisionObject(box, box.initial_pose));
    std::this_thread::sleep_for(std::chrono::duration<double>(SCENE_SYNC_WAIT_SEC));
    object_removed = false;
  };

  do
  {
    ArmContext left;
    left.side = Side::LEFT;
    left.group_name = "left_arm";
    left.partner_group_name = "right_arm";
    left.eef_link = "left_fr3_link8";
    left.suction_link = "left_fr3_compact_suction";
    left.group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, left.group_name);
    ArmContext right;
    right.side = Side::RIGHT;
    right.group_name = "right_arm";
    right.partner_group_name = "left_arm";
    right.eef_link = "right_fr3_link8";
    right.suction_link = "right_fr3_compact_suction";
    right.group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_, right.group_name);
    if (!configureGroup(left) || !configureGroup(right) ||
        left.home_q.size() != right.home_q.size())
    {
      output.diagnostics = "无法读取双 FR3 当前 joint state / planning group";
      break;
    }

    // PRE_CONTACT 保持 Box 在 World，使 MoveIt 对独立到位路径做常规碰撞检查。
    // 两臂按实际执行语义顺序到位：right 的规划显式冻结 left 在其 PRE_CONTACT。
    const auto left_pre_target = link8TargetForGrasp(
      box.initial_pose, left_grasp, config_.pre_contact_clearance_z, config_);
    const auto right_pre_target = link8TargetForGrasp(
      box.initial_pose, right_grasp, config_.pre_contact_clearance_z, config_);
    trajectory_msgs::msg::JointTrajectory left_pre;
    trajectory_msgs::msg::JointTrajectory right_pre;
    bool pre_contact_pair_found = false;
    // 右臂 PRE_CONTACT 的可行性依赖左臂到位后的冗余关节解。仅对右臂在同一
    // 左臂解上重试会反复命中同一个死角；所以把三次预算用在“左/右组合”上。
    for (int pair_attempt = 1; pair_attempt <= config_.max_planning_retries; ++pair_attempt)
    {
      if (!planPose(node_, left, left.home_q, right.home_q, left_pre_target,
            "SEQUENTIAL_PRE_CONTACT", 1, left_pre))
      {
        continue;
      }
      if (planPose(node_, right, right.home_q, finalPositions(left_pre), right_pre_target,
          "SEQUENTIAL_PRE_CONTACT", 1, right_pre))
      {
        pre_contact_pair_found = true;
        RCLCPP_INFO(
          node_->get_logger(), "Task21 tight PRE_CONTACT pair selected, attempt=%d/%d",
          pair_attempt, config_.max_planning_retries);
        break;
      }
      RCLCPP_WARN(
        node_->get_logger(), "Task21 tight PRE_CONTACT pair rejected; resampling left posture, "
        "attempt=%d/%d", pair_attempt, config_.max_planning_retries);
    }
    if (!pre_contact_pair_found)
    {
      output.diagnostics = "SEQUENTIAL_PRE_CONTACT RRTConnect 三次重试仍失败";
      break;
    }
    SharedObjectStage pre_stage;
    pre_stage.name = "SEQUENTIAL_PRE_CONTACT";
    pre_stage.left = left_pre;
    pre_stage.right = right_pre;
    pre_stage.box_start = box.initial_pose;
    pre_stage.box_end = box.initial_pose;
    // 该阶段按 left 后 right 的真实时序执行，故 duration 是串行和而非 max。
    pre_stage.duration_sec = trajectoryDuration(left_pre) + trajectoryDuration(right_pre);
    output.stages.push_back(std::move(pre_stage));

    // CONTACT 后直到共同退出，MoveIt 无法把同一 Box 同时 attach 到两个 link。
    // 因此从外部 World 暂时移除；下方私有 FCL scene 会在每个采样点恢复它的真实
    // 几何，并仅允许两只吸盘杯与其接触。
    if (!removeWorldObject(moveit_scene, box.id))
    {
      output.diagnostics = "运行时 SharedBox 不在 MoveIt World，拒绝假设其存在";
      break;
    }
    object_removed = true;
    std::vector<moveit_msgs::msg::CollisionObject> static_world;
    for (const auto& item : moveit_scene.getObjects())
    {
      if (item.first != box.id)
      {
        static_world.push_back(item.second);
      }
    }

    const auto makeStage = [&](const std::string& name,
      const std::vector<double>& left_start, const std::vector<double>& right_start,
      const geometry_msgs::msg::Pose& left_target,
      const geometry_msgs::msg::Pose& right_target,
      const geometry_msgs::msg::Pose& box_start,
      const geometry_msgs::msg::Pose& box_end,
      bool avoid_collisions,
      bool enforce_grasp_constraint,
      SharedObjectStage& stage)
    {
      stage = SharedObjectStage();
      stage.name = name;
      stage.box_start = box_start;
      stage.box_end = box_end;
      stage.enforce_grasp_constraint = enforce_grasp_constraint;
      return planCartesian(node_, left, left_start, right_start, left_target, name,
          avoid_collisions, config_, stage.left) &&
        planCartesian(node_, right, right_start, left_start, right_target, name,
          avoid_collisions, config_, stage.right) &&
        synchronizeDurations(
          stage.left, stage.right, left.group->getRobotModel(), left.eef_link,
          right.eef_link, config_.suction_tcp_offset_z) &&
        ((stage.duration_sec = std::max(trajectoryDuration(stage.left),
          trajectoryDuration(stage.right))) > 0.0) &&
        validateStage(node_, left.group->getRobotModel(), static_world, box, left_grasp,
          right_grasp, stage, config_, output, output.diagnostics);
    };

    const auto source_contact_left = link8TargetForGrasp(
      box.initial_pose, left_grasp, 0.0, config_);
    const auto source_contact_right = link8TargetForGrasp(
      box.initial_pose, right_grasp, 0.0, config_);
    SharedObjectStage contact;
    if (!makeStage("DUAL_CONTACT", finalPositions(left_pre), finalPositions(right_pre),
          source_contact_left, source_contact_right, box.initial_pose, box.initial_pose,
          true, false, contact))
    {
      if (output.diagnostics.empty())
      {
        output.diagnostics = "DUAL_CONTACT Cartesian / FCL 失败";
      }
      break;
    }
    output.stages.push_back(std::move(contact));

    auto lifted_pose = box.initial_pose;
    lifted_pose.position.z += config_.lift_height;
    SharedObjectStage lift;
    if (!makeStage("COMMON_LIFT", finalPositions(output.stages.back().left),
          finalPositions(output.stages.back().right),
          link8TargetForGrasp(lifted_pose, left_grasp, 0.0, config_),
          link8TargetForGrasp(lifted_pose, right_grasp, 0.0, config_),
          box.initial_pose, lifted_pose, false, true, lift))
    {
      if (output.diagnostics.empty())
      {
        output.diagnostics = "COMMON_LIFT Cartesian / FCL 失败";
      }
      break;
    }
    output.stages.push_back(std::move(lift));

    auto placed_pose = placement.target_pose;
    // PlacementSpec 是物体中心的已验收几何契约；这里仅验证 support_height 与
    // Box bottom 一致，避免在 tight path 内悄悄改写 Task19 的动态放置高度。
    const double expected_center_z = placement.support_height + 0.5 * box.dimensions[2];
    if (std::abs(placed_pose.position.z - expected_center_z) > 0.003)
    {
      output.diagnostics = "PlacementSpec center Z 与 support_height / Box height 不一致";
      break;
    }
    auto transport_pose = placed_pose;
    transport_pose.position.z += config_.lift_height;
    SharedObjectStage transport;
    if (!makeStage("COMMON_TRANSPORT", finalPositions(output.stages.back().left),
          finalPositions(output.stages.back().right),
          link8TargetForGrasp(transport_pose, left_grasp, 0.0, config_),
          link8TargetForGrasp(transport_pose, right_grasp, 0.0, config_),
          lifted_pose, transport_pose, false, true, transport))
    {
      if (output.diagnostics.empty())
      {
        output.diagnostics = "COMMON_TRANSPORT Cartesian / FCL 失败";
      }
      break;
    }
    output.stages.push_back(std::move(transport));

    SharedObjectStage descent;
    if (!makeStage("COMMON_DESCENT", finalPositions(output.stages.back().left),
          finalPositions(output.stages.back().right),
          link8TargetForGrasp(placed_pose, left_grasp, 0.0, config_),
          link8TargetForGrasp(placed_pose, right_grasp, 0.0, config_),
          transport_pose, placed_pose, false, true, descent))
    {
      if (output.diagnostics.empty())
      {
        output.diagnostics = "COMMON_DESCENT Cartesian / FCL 失败";
      }
      break;
    }
    output.stages.push_back(std::move(descent));

    // 退出在物理释放前完成规划；FCL 仍携带 SharedBox，避免将已放置大件永久忽略。
    SharedObjectStage retreat;
    if (!makeStage("PREPLANNED_COMMON_RETREAT", finalPositions(output.stages.back().left),
          finalPositions(output.stages.back().right),
          link8TargetForGrasp(placed_pose, left_grasp, config_.pre_contact_clearance_z, config_),
          link8TargetForGrasp(placed_pose, right_grasp, config_.pre_contact_clearance_z, config_),
          // RETREAT 在物理释放前生成，但实际执行发生在 SUCTION OFF / settle / World
          // writeback 之后。Box 因而固定在放置位，不再对两个 TCP 施加刚体约束。
          placed_pose, placed_pose, false, false, retreat))
    {
      if (output.diagnostics.empty())
      {
        output.diagnostics = "PREPLANNED_COMMON_RETREAT Cartesian / FCL 失败";
      }
      break;
    }
    output.stages.push_back(std::move(retreat));

    output.duration_sec = 0.0;
    for (const auto& stage : output.stages)
    {
      output.duration_sec += stage.duration_sec;
    }
    output.planning_cost = output.duration_sec;
    output.valid = true;
    std::ostringstream stream;
    stream << "shared-object preflight PASS: grasps=" << left_grasp.id << "," <<
      right_grasp.id << ", stages=" << output.stages.size() << ", fcl_samples=" <<
      output.fcl_samples;
    output.diagnostics = stream.str();
  } while (false);

  restoreSourceObject();
  return output.valid;
}

}  // namespace fr3_dual_palletize
