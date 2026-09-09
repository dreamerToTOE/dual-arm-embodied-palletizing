#include "fr3_dual_palletize/spatiotemporal_conflict_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <utility>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace fr3_dual_palletize
{
namespace
{
constexpr double BOX_SIZE = 0.030;
constexpr double BOX_HALF = BOX_SIZE * 0.5;
constexpr double SUCTION_TCP_OFFSET_Z = 0.105;
constexpr double CONTACT_TCP_CLEARANCE = 0.001;
constexpr double PI = 3.14159265358979323846;

enum class ObjectState
{
  WORLD_INITIAL,
  ATTACHED,
  WORLD_RELEASED,
};

double pointTime(const trajectory_msgs::msg::JointTrajectoryPoint& point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

bool startsWith(const std::string& value, const std::string& prefix)
{
  return value.rfind(prefix, 0) == 0;
}

bool isArmBody(const std::string& body)
{
  return startsWith(body, "left_fr3_") || startsWith(body, "right_fr3_");
}

bool isLeftArmBody(const std::string& body)
{
  return startsWith(body, "left_fr3_");
}

bool isRightArmBody(const std::string& body)
{
  return startsWith(body, "right_fr3_");
}

bool isCandidateObject(
  const std::string& body,
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right)
{
  return body == left.object_id || body == right.object_id;
}

bool isOwnArmObjectPair(
  const std::string& arm_body,
  const std::string& object_body,
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right)
{
  return (isLeftArmBody(arm_body) && object_body == left.object_id) ||
         (isRightArmBody(arm_body) && object_body == right.object_id);
}

std::string classifyPair(
  const std::string& body_a,
  const std::string& body_b,
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right)
{
  if ((isLeftArmBody(body_a) && isRightArmBody(body_b)) ||
      (isRightArmBody(body_a) && isLeftArmBody(body_b)))
  {
    return "ARM_ARM";
  }

  if (isArmBody(body_a) && isCandidateObject(body_b, left, right))
  {
    return isOwnArmObjectPair(body_a, body_b, left, right) ? "" : "ARM_OBJECT";
  }

  if (isArmBody(body_b) && isCandidateObject(body_a, left, right))
  {
    return isOwnArmObjectPair(body_b, body_a, left, right) ? "" : "ARM_OBJECT";
  }

  if (isCandidateObject(body_a, left, right) &&
      isCandidateObject(body_b, left, right) && body_a != body_b)
  {
    return "OBJECT_OBJECT";
  }

  // 机器人自身、同臂与自身 Box、桌面等均已在单臂 MoveIt 规划阶段处理；
  // Task08-B 只报告两个任务彼此引入的动态冲突。
  return "";
}

bool validateTrajectory(
  const TaskTrajectoryCandidate& candidate,
  std::string& error)
{
  const auto& trajectory = candidate.trajectory;
  if (candidate.planning_group.empty() || candidate.object_id.empty() ||
      candidate.eef_link.empty())
  {
    error = "candidate 元数据不完整。";
    return false;
  }

  if (trajectory.joint_names.empty() || trajectory.points.empty())
  {
    error = candidate.label + " 缺少 JointTrajectory。";
    return false;
  }

  double previous_time = -1.0;
  for (const auto& point : trajectory.points)
  {
    if (point.positions.size() != trajectory.joint_names.size())
    {
      error = candidate.label + " 的 joint_names 与 positions 数量不一致。";
      return false;
    }

    const double time = pointTime(point);
    if (time + 1e-9 < previous_time)
    {
      error = candidate.label + " 的 time_from_start 非单调。";
      return false;
    }
    previous_time = time;
  }

  bool have_attach = false;
  bool have_detach = false;
  double attach_time = -1.0;
  double detach_time = -1.0;
  for (const auto& event : candidate.events)
  {
    if (event.object_name != candidate.object_id)
    {
      continue;
    }
    if (event.type == TaskEventType::ATTACH)
    {
      have_attach = true;
      attach_time = event.time_sec;
    }
    else if (event.type == TaskEventType::DETACH)
    {
      have_detach = true;
      detach_time = event.time_sec;
    }
  }

  if (!have_attach || !have_detach || attach_time < 0.0 || detach_time < attach_time)
  {
    error = candidate.label + " 缺少有效 ATTACH / DETACH 事件。";
    return false;
  }

  if (detach_time > pointTime(trajectory.points.back()) + 1e-6)
  {
    error = candidate.label + " 的 DETACH 时间超出轨迹范围。";
    return false;
  }

  return true;
}

std::vector<double> interpolate(
  const trajectory_msgs::msg::JointTrajectory& trajectory,
  double time_sec)
{
  const auto& points = trajectory.points;
  if (time_sec <= pointTime(points.front()))
  {
    return points.front().positions;
  }

  if (time_sec >= pointTime(points.back()))
  {
    return points.back().positions;
  }

  for (std::size_t i = 1; i < points.size(); ++i)
  {
    const double before_time = pointTime(points[i - 1]);
    const double after_time = pointTime(points[i]);
    if (time_sec > after_time)
    {
      continue;
    }

    const double duration = after_time - before_time;
    if (duration <= 1e-9)
    {
      return points[i].positions;
    }

    const double alpha = (time_sec - before_time) / duration;
    std::vector<double> values(points[i].positions.size());
    for (std::size_t joint = 0; joint < values.size(); ++joint)
    {
      values[joint] = points[i - 1].positions[joint] +
        alpha * (points[i].positions[joint] - points[i - 1].positions[joint]);
    }
    return values;
  }

  return points.back().positions;
}

ObjectState objectStateAt(
  const TaskTrajectoryCandidate& candidate,
  double time_sec)
{
  double attach_time = std::numeric_limits<double>::infinity();
  double detach_time = std::numeric_limits<double>::infinity();
  for (const auto& event : candidate.events)
  {
    if (event.object_name != candidate.object_id)
    {
      continue;
    }
    if (event.type == TaskEventType::ATTACH)
    {
      attach_time = event.time_sec;
    }
    else if (event.type == TaskEventType::DETACH)
    {
      detach_time = event.time_sec;
    }
  }

  if (time_sec < attach_time)
  {
    return ObjectState::WORLD_INITIAL;
  }
  if (time_sec < detach_time)
  {
    return ObjectState::ATTACHED;
  }
  return ObjectState::WORLD_RELEASED;
}

moveit_msgs::msg::CollisionObject makeWorldBox(
  const TaskTrajectoryCandidate& candidate,
  ObjectState state)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = candidate.object_id;

  shape_msgs::msg::SolidPrimitive shape;
  shape.type = shape_msgs::msg::SolidPrimitive::BOX;
  shape.dimensions = {BOX_SIZE, BOX_SIZE, BOX_SIZE};
  object.primitives.push_back(shape);
  object.primitive_poses.push_back(
    state == ObjectState::WORLD_INITIAL ?
    candidate.initial_object_pose : candidate.planned_release_pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

void attachBoxToState(
  moveit::core::RobotState& state,
  planning_scene::PlanningScene& scene,
  const TaskTrajectoryCandidate& candidate)
{
  std::vector<shapes::ShapeConstPtr> shapes;
  shapes.push_back(std::make_shared<shapes::Box>(BOX_SIZE, BOX_SIZE, BOX_SIZE));

  EigenSTL::vector_Isometry3d shape_poses;
  shape_poses.push_back(Eigen::Isometry3d::Identity());

  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
  relative_pose.translation().z() =
    SUCTION_TCP_OFFSET_Z + BOX_HALF + CONTACT_TCP_CLEARANCE;
  relative_pose.rotate(Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX()));

  const std::vector<std::string> touch_links = candidate.object_touch_links.empty() ?
    std::vector<std::string>{candidate.eef_link} : candidate.object_touch_links;
  state.attachBody(
    candidate.object_id,
    relative_pose,
    shapes,
    shape_poses,
    touch_links,
    candidate.eef_link);

  auto& acm = scene.getAllowedCollisionMatrixNonConst();
  for (const auto& link : touch_links)
  {
    acm.setEntry(candidate.object_id, link, true);
  }
}

bool addObjectForTime(
  planning_scene::PlanningScene& scene,
  moveit::core::RobotState& state,
  const TaskTrajectoryCandidate& candidate,
  double time_sec)
{
  const auto object_state = objectStateAt(candidate, time_sec);
  if (object_state == ObjectState::ATTACHED)
  {
    attachBoxToState(state, scene, candidate);
    return true;
  }
  return scene.processCollisionObjectMsg(makeWorldBox(candidate, object_state));
}

}  // namespace

SpatioTemporalConflictDetector::SpatioTemporalConflictDetector(
  moveit::core::RobotModelConstPtr robot_model,
  std::vector<moveit_msgs::msg::CollisionObject> static_world_objects)
  : robot_model_(std::move(robot_model)),
    static_world_objects_(std::move(static_world_objects))
{
}

ConflictReport SpatioTemporalConflictDetector::check(
  const TaskTrajectoryCandidate& left,
  const TaskTrajectoryCandidate& right,
  double sample_period_sec,
  double left_start_delay_sec,
  double right_start_delay_sec) const
{
  ConflictReport report;
  report.sample_period_sec = sample_period_sec;
  report.left_start_delay_sec = left_start_delay_sec;
  report.right_start_delay_sec = right_start_delay_sec;

  if (!robot_model_)
  {
    report.error = "RobotModel 为空。";
    return report;
  }
  if (sample_period_sec <= 0.0)
  {
    report.error = "采样周期必须大于 0。";
    return report;
  }
  if (left_start_delay_sec < 0.0 || right_start_delay_sec < 0.0)
  {
    report.error = "启动延迟不能为负。";
    return report;
  }
  if (!validateTrajectory(left, report.error) ||
      !validateTrajectory(right, report.error))
  {
    return report;
  }

  const auto started = std::chrono::steady_clock::now();
  auto base_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  for (const auto& object : static_world_objects_)
  {
    if (object.id == left.object_id || object.id == right.object_id)
    {
      continue;
    }
    if (!base_scene->processCollisionObjectMsg(object))
    {
      report.error = "无法载入静态 CollisionObject: " + object.id;
      return report;
    }
  }

  const double left_duration = pointTime(left.trajectory.points.back());
  const double right_duration = pointTime(right.trajectory.points.back());
  report.horizon_sec = std::max(
    left_start_delay_sec + left_duration,
    right_start_delay_sec + right_duration);

  // 使用整数 sample index 避免浮点累积误差，并保证必定检查精确的末点。
  const std::size_t sample_count = static_cast<std::size_t>(
    std::ceil(report.horizon_sec / sample_period_sec));
  bool inside_conflict_window = false;
  for (std::size_t sample_index = 0;
       sample_index <= sample_count;
       ++sample_index)
  {
    const double sample_time = std::min(
      static_cast<double>(sample_index) * sample_period_sec,
      report.horizon_sec);
    auto sample_scene = base_scene->diff();
    const double left_local_time = sample_time - left_start_delay_sec;
    const double right_local_time = sample_time - right_start_delay_sec;
    auto left_q = interpolate(left.trajectory, std::max(0.0, left_local_time));
    auto right_q = interpolate(right.trajectory, std::max(0.0, right_local_time));

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.setVariablePositions(left.trajectory.joint_names, left_q);
    state.setVariablePositions(right.trajectory.joint_names, right_q);
    state.update();

    if (!addObjectForTime(*sample_scene, state, left, left_local_time) ||
        !addObjectForTime(*sample_scene, state, right, right_local_time))
    {
      report.error = "无法在采样时刻重建 Box 的碰撞状态。";
      return report;
    }
    state.update();

    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 1000;
    request.max_contacts_per_pair = 10;
    collision_detection::CollisionResult result;
    sample_scene->checkCollision(request, result, state);
    ++report.samples_checked;

    std::vector<ConflictEvent> sample_events;
    if (result.collision)
    {
      for (const auto& [pair, contacts] : result.contacts)
      {
        static_cast<void>(contacts);
        const std::string type = classifyPair(pair.first, pair.second, left, right);
        if (type.empty())
        {
          continue;
        }

        sample_events.push_back(ConflictEvent{
          sample_time,
          type,
          pair.first,
          pair.second
        });
      }
    }

    if (sample_events.empty())
    {
      inside_conflict_window = false;
      continue;
    }

    if (!report.conflict)
    {
      report.conflict = true;
      report.first_conflict_time_sec = sample_time;
      // 兼容 Task08 既有日志：events 始终表示首个冲突采样点。
      report.events = sample_events;
    }

    if (!inside_conflict_window)
    {
      const auto& first_event = sample_events.front();
      report.conflict_windows.push_back(ConflictWindow{
        sample_time,
        sample_time,
        first_event.type,
        first_event.body_a,
        first_event.body_b,
        1
      });
      inside_conflict_window = true;
    }
    else
    {
      auto& window = report.conflict_windows.back();
      window.end_time_sec = sample_time;
      ++window.samples;
    }
  }

  report.valid = true;
  report.wall_time_sec = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  return report;
}

}  // namespace fr3_dual_palletize
